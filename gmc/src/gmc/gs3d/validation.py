"""Independent whole-path and linear unicycle checks for GS3D consumers."""
from __future__ import annotations

from dataclasses import asdict
import math

import numpy as np

from .contracts import BodySpec, BodyOracle, GoalRegion, Pose3
from .oracle import validate_body, validate_pose


GOAL_ROUNDOFF_M = 1e-9


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def goal_reached(q: Pose3, goal: GoalRegion):
    return (np.linalg.norm(np.asarray(q.xyz) - goal.original.xyz)
            <= goal.position_tolerance_m + GOAL_ROUNDOFF_M
            and abs(angle_delta(q.yaw, goal.original.yaw)) <= goal.yaw_tolerance_rad + 1e-12)


def validate_goal(goal: GoalRegion):
    validate_pose(goal.original)
    if (not np.isfinite([goal.position_tolerance_m, goal.yaw_tolerance_rad]).all()
            or not 0 <= goal.position_tolerance_m <= .5
            or not 0 <= goal.yaw_tolerance_rad <= math.pi):
        raise ValueError("goal tolerances must be finite, position in [0,.5] m and yaw in [0,pi]")


def verify_path(oracle: BodyOracle, poses: list[Pose3], body: BodySpec, *, margin_m: float,
                goal: GoalRegion | None = None) -> dict:
    """Recheck every closed segment, including singleton paths; no sample certificate."""
    if not poses:
        return {"passed": False, "safety": "invalid", "reason": "empty_path", "reports": []}
    try:
        validate_body(body)
        for q in poses:
            validate_pose(q)
        if goal is not None:
            validate_goal(goal)
    except (ValueError, TypeError, AttributeError) as exc:
        return {"passed": False, "safety": "invalid", "reason": str(exc), "reports": []}
    reports = []
    edges = zip(poses, poses[1:]) if len(poses) > 1 else [(poses[0], poses[0])]
    for a, b in edges:
        if body.motion == "ground_unicycle":
            displacement = np.asarray(b.xyz) - a.xyz
            length_xy = np.linalg.norm(displacement[:2])
            if length_xy > 1e-10:
                heading = math.atan2(displacement[1], displacement[0])
                if abs(angle_delta(a.yaw, b.yaw)) > 1e-8 or abs(angle_delta(a.yaw, heading)) > 1e-8:
                    return {"passed": False, "safety": "invalid", "reason": "ground_lateral_slip_or_turn_during_translation", "reports": reports}
            elif abs(displacement[2]) > 1e-8:
                return {"passed": False, "safety": "invalid", "reason": "ground_independent_climb", "reports": reports}
        report = oracle.edge(a, b, body, margin_m=margin_m)
        reports.append(asdict(report))
        if report.occupancy != "free" or report.safety != "continuous_bound":
            return {"passed": False, "safety": report.safety, "reason": report.reason, "reports": reports}
    if goal is not None and not goal_reached(poses[-1], goal):
        return {"passed": False, "safety": "invalid", "reason": "goal_tolerance_exceeded", "reports": reports}
    return {"passed": True, "safety": "continuous_bound", "reason": "all_closed_edges_verified",
            "clearance_lower_m": min(r["clearance_lower_m"] for r in reports), "reports": reports}


def verify_linear_trajectory(trajectory, body: BodySpec, limits: dict) -> dict:
    """Check timing, declared speed and yaw-rate; acceleration is explicitly unproven."""
    try:
        rows = np.asarray(trajectory["poses"], dtype=float)
        times = np.asarray(trajectory["time_s"], dtype=float)
        dt = trajectory["control_dt_s"]
        if (rows.ndim != 2 or rows.shape[1] != 4 or len(rows) < 1
                or times.shape != (len(rows),) or not np.isfinite(rows).all()
                or not np.isfinite(times).all() or times[0] != 0
                or np.any(np.diff(times) <= 0) or not np.isfinite(dt) or dt <= 0
                or trajectory["interpolation"] != "linear_xyz_yaw" or trajectory["segments"]):
            raise ValueError("invalid linear trajectory shape/timing/interpolation")
        if len(rows) > 1:
            elapsed = np.diff(times)
            difference = np.diff(rows, axis=0)
            speed = np.linalg.norm(difference[:, :3], axis=1) / elapsed
            yaw_rate = np.abs(difference[:, 3]) / elapsed
            if np.any(speed > limits["max_speed_mps"] + 1e-9) or np.any(yaw_rate > limits["max_yaw_rate_radps"] + 1e-9):
                raise ValueError("speed_or_yaw_rate_exceeded")
            if body.motion == "uav_translation" and np.any(np.abs(difference[:, 2]) / elapsed > limits["max_vertical_speed_mps"] + 1e-9):
                raise ValueError("vertical_speed_exceeded")
        return {"passed": True, "acceleration_verified": False,
                "limitation": "piecewise_linear_velocity_discontinuities_at_knots"}
    except (ValueError, TypeError, KeyError) as exc:
        return {"passed": False, "reason": str(exc), "acceleration_verified": False}
