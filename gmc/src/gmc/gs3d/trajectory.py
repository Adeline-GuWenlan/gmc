"""Strict GS3D result I/O, variable-z sampling, and independent replay."""
from __future__ import annotations

from dataclasses import fields
import json
import math
from pathlib import Path

import numpy as np

from .contracts import BodySpec, GoalRegion, Pose3
from .validation import verify_linear_trajectory, verify_path


def trajectory_poses(trajectory: dict) -> list[Pose3]:
    rows = np.asarray(trajectory.get("poses"), float)
    if rows.ndim != 2 or rows.shape[1] != 4 or not np.isfinite(rows).all():
        raise ValueError("GS3D trajectory poses must be finite [x,y,z,yaw] rows")
    return [Pose3(tuple(map(float, row[:3])), float(row[3])) for row in rows]


def sample_linear_trajectory(trajectory: dict, *, dt_s: float | None = None) -> dict:
    """Sample exported xyz/yaw/timing; z is never reconstructed from a robot band."""
    if trajectory.get("interpolation") != "linear_xyz_yaw" or trajectory.get("segments"):
        raise ValueError("only the GS3D linear trajectory schema is supported here")
    poses = trajectory_poses(trajectory)
    rows = np.asarray([(*q.xyz, q.yaw) for q in poses], float)
    times = np.asarray(trajectory.get("time_s"), float)
    dt = float(trajectory.get("control_dt_s") if dt_s is None else dt_s)
    if (times.shape != (len(rows),) or not np.isfinite(times).all() or times[0] != 0
            or (len(times) > 1 and np.any(np.diff(times) <= 0))
            or not math.isfinite(dt) or dt <= 0):
        raise ValueError("invalid trajectory timing")
    if len(rows) == 1:
        sample_times, samples = np.array([0.]), rows.copy()
    else:
        sample_times = np.arange(0., times[-1], dt)
        if not len(sample_times) or sample_times[-1] != times[-1]:
            sample_times = np.r_[sample_times, times[-1]]
        samples = np.empty((len(sample_times), 4), float)
        for column in range(4):
            samples[:, column] = np.interp(sample_times, times, rows[:, column])
    return {"schema_version": "gs3d.replay-samples.v1",
            "time_s": sample_times.tolist(), "poses": samples.tolist(),
            "source_control_dt_s": float(trajectory["control_dt_s"]),
            "sample_dt_s": dt, "altitude_range_m": float(np.ptp(samples[:, 2]))}


def _body_from_result(result: dict) -> tuple[BodySpec, dict]:
    payload = result.get("robot", {})
    names = {field.name for field in fields(BodySpec)}
    body = BodySpec(**{key: payload[key] for key in names})
    limits = payload.get("limits")
    if not isinstance(limits, dict):
        raise ValueError("serialized robot limits are required")
    return body, limits


def validate_plan_result(result: dict) -> None:
    if result.get("schema_version") != "gs3d.v1":
        raise ValueError("not a GS3D v1 result; legacy Pose2 rows are never reinterpreted")
    original = np.asarray(result.get("original_goal"), float)
    if original.shape != (4,) or not np.isfinite(original).all():
        raise ValueError("original_goal must be finite [x,y,z,yaw]")
    if result.get("status") == "success":
        if result.get("trajectory") is None or result.get("attained_goal") is None:
            raise ValueError("successful result requires trajectory and attained goal")
        trajectory_poses(result["trajectory"])
        json.dumps(result, allow_nan=False)
    elif result.get("trajectory") is not None or result.get("attained_goal") is not None:
        raise ValueError("failed results cannot contain executable trajectories")


def write_plan(path, result: dict) -> Path:
    validate_plan_result(result)
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return output


def read_plan(path) -> dict:
    def reject_constant(value):
        raise ValueError(f"non-finite JSON constant is forbidden: {value}")

    result = json.loads(Path(path).read_text(), parse_constant=reject_constant)
    validate_plan_result(result)
    return result


def replay_plan(result: dict, oracle) -> dict:
    """Independently recheck the exported poses, motion limits and terminal error."""
    validate_plan_result(result)
    if result["status"] != "success":
        return {"passed": False, "reason": "result_not_success", "samples": None}
    body, limits = _body_from_result(result)
    poses = trajectory_poses(result["trajectory"])
    original = Pose3(tuple(map(float, result["original_goal"][:3])),
                     float(result["original_goal"][3]))
    goal = GoalRegion(original, float(result["position_tolerance_m"]),
                      float(result["yaw_tolerance_rad"]))
    margin = float(result["diagnostics"]["margin_m"])
    geometry = verify_path(oracle, poses, body, margin_m=margin, goal=goal)
    kinematics = verify_linear_trajectory(result["trajectory"], body, limits)
    samples = sample_linear_trajectory(result["trajectory"])
    recorded = np.asarray(result["attained_goal"], float)
    replayed = np.asarray((*poses[-1].xyz, poses[-1].yaw), float)
    attained_matches = bool(np.allclose(recorded, replayed, rtol=0., atol=1e-12))
    return {"passed": bool(geometry["passed"] and kinematics["passed"] and attained_matches),
            "geometry": geometry, "kinematics": kinematics,
            "attained_matches": attained_matches, "samples": samples,
            "variable_z": samples["altitude_range_m"] > 0.}
