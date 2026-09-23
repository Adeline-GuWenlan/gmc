"""Helpers for the real-scene GS3D integration mission.

The helpers in this module deliberately retain world xyz trajectories.  Route
coordinates are used only to express the frozen showcase task and coverage
prism; collision authority remains the full 3-D Gaussian/body oracle.
"""
from __future__ import annotations

from dataclasses import dataclass
from itertools import product
import math

import numpy as np

from .contracts import BodySpec, GoalRegion, Pose3
from .trajectory import sample_linear_trajectory, trajectory_poses
from .validation import verify_linear_trajectory, verify_path


@dataclass(frozen=True)
class RouteBoxKnownSpace:
    """A closed oriented route-frame prism used as assumed map coverage."""

    origin_world_m: tuple[float, float, float]
    world_to_route: tuple[tuple[float, float, float], ...]
    lower_route_m: tuple[float, float, float]
    upper_route_m: tuple[float, float, float]

    def __post_init__(self):
        origin = np.asarray(self.origin_world_m, float)
        rotation = np.asarray(self.world_to_route, float)
        lower = np.asarray(self.lower_route_m, float)
        upper = np.asarray(self.upper_route_m, float)
        if (origin.shape != (3,) or rotation.shape != (3, 3)
                or lower.shape != (3,) or upper.shape != (3,)
                or not np.isfinite([*origin, *rotation.ravel(), *lower, *upper]).all()
                or np.any(lower >= upper)
                or not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-10)):
            raise ValueError("route known-space frame must be finite, ordered and orthonormal")

    def world_bounds(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        lower, upper = np.asarray(self.lower_route_m), np.asarray(self.upper_route_m)
        corners = np.asarray(list(product(*zip(lower, upper))), float)
        world = corners @ np.asarray(self.world_to_route) + np.asarray(self.origin_world_m)
        return tuple(map(float, world.min(axis=0))), tuple(map(float, world.max(axis=0)))

    def contains_aabb(self, lower, upper) -> bool:
        lo, hi = np.asarray(lower, float), np.asarray(upper, float)
        if (lo.shape != (3,) or hi.shape != (3,)
                or not np.isfinite([*lo, *hi]).all() or np.any(lo > hi)):
            return False
        corners = np.asarray(list(product(*zip(lo, hi))), float)
        route = (corners - np.asarray(self.origin_world_m)) @ np.asarray(self.world_to_route).T
        domain_lo, domain_hi = np.asarray(self.lower_route_m), np.asarray(self.upper_route_m)
        return bool(np.all(route >= domain_lo - 1e-12) and np.all(route <= domain_hi + 1e-12))


def concatenate_linear_trajectories(results: list[dict]) -> dict:
    """Join successful leg trajectories without hiding their planner results."""
    if not results or any(result.get("status") != "success" for result in results):
        raise ValueError("all mission legs must be successful GS3D results")
    rows: list[list[float]] = []
    times: list[float] = []
    offset = 0.0
    control_dt = None
    for index, result in enumerate(results):
        trajectory = result["trajectory"]
        if trajectory.get("interpolation") != "linear_xyz_yaw" or trajectory.get("segments"):
            raise ValueError("only unsmoothed linear GS3D legs may be concatenated at A5")
        leg_rows = np.asarray(trajectory["poses"], float)
        leg_times = np.asarray(trajectory["time_s"], float)
        if index and not np.allclose(leg_rows[0], rows[-1], rtol=0., atol=1e-12):
            raise ValueError("mission legs are not position/yaw continuous")
        start = 0 if index == 0 else 1
        rows.extend(leg_rows[start:].tolist())
        times.extend((leg_times[start:] + offset).tolist())
        offset += float(leg_times[-1])
        if control_dt is None:
            control_dt = float(trajectory["control_dt_s"])
        elif not math.isclose(control_dt, float(trajectory["control_dt_s"])):
            raise ValueError("mission legs disagree on physical control dt")
    if len(times) > 1 and np.any(np.diff(times) <= 0):
        raise ValueError("concatenated mission time must be strictly increasing")
    return {"poses": rows, "time_s": times, "interpolation": "linear_xyz_yaw",
            "segments": [], "control_dt_s": control_dt}


def replay_linear_mission(trajectory: dict, oracle, body: BodySpec, *, margin_m: float,
                          goal: GoalRegion) -> dict:
    """Independently replay a multi-call mission with the shared oracle."""
    poses = trajectory_poses(trajectory)
    geometry = verify_path(oracle, poses, body, margin_m=margin_m, goal=goal)
    limits = {"max_speed_mps": .5 if body.motion == "uav_translation" else .3,
              "max_vertical_speed_mps": .3 if body.motion == "uav_translation" else 0.,
              "max_yaw_rate_radps": 1., "max_acceleration_mps2": .5 if body.motion == "uav_translation" else .3,
              "max_yaw_acceleration_radps2": 1.}
    kinematics = verify_linear_trajectory(trajectory, body, limits)
    samples = sample_linear_trajectory(trajectory)
    return {"passed": bool(geometry["passed"] and kinematics["passed"]),
            "geometry": geometry, "kinematics": kinematics, "samples": samples,
            "variable_z": samples["altitude_range_m"] > 0.}


def _crossing_interval(local_rows: np.ndarray, times: np.ndarray, s_interval,
                       *, expected_z: float,
                       lateral_bounds_m: tuple[float, float]) -> dict | None:
    """Analytically locate a crossing, merging consecutive linear segments.

    A lattice route may cross the required forward interval using several
    short diagonal/lateral segments.  Each accepted segment is clipped in
    route ``s`` and the resulting closed intervals are unioned.  This is not a
    sampled collision claim; collision is independently certified by replay.
    """
    target_lo, target_hi = map(float, s_interval)
    lateral_lo, lateral_hi = map(float, lateral_bounds_m)
    clipped = []
    for a, b, ta, tb in zip(local_rows, local_rows[1:], times, times[1:]):
        if (a[1] < lateral_lo - 1e-10 or a[1] > lateral_hi + 1e-10
                or b[1] < lateral_lo - 1e-10 or b[1] > lateral_hi + 1e-10
                or abs(a[2] - expected_z) > 1e-7 or abs(b[2] - expected_z) > 1e-7
                or abs(b[0] - a[0]) < 1e-12):
            continue
        segment_lo, segment_hi = sorted((float(a[0]), float(b[0])))
        overlap_lo, overlap_hi = max(segment_lo, target_lo), min(segment_hi, target_hi)
        if overlap_hi <= overlap_lo + 1e-12:
            continue
        fractions = [(value - a[0]) / (b[0] - a[0]) for value in (overlap_lo, overlap_hi)]
        crossing_times = [float(ta + fraction * (tb - ta)) for fraction in fractions]
        clipped.append({"s": [overlap_lo, overlap_hi], "time": sorted(crossing_times)})
    clipped.sort(key=lambda row: row["s"][0])
    components = []
    for row in clipped:
        if not components or row["s"][0] > components[-1]["s"][1] + 1e-9:
            components.append({"s": list(row["s"]), "time": list(row["time"]),
                               "segments": 1})
        else:
            component = components[-1]
            component["s"][1] = max(component["s"][1], row["s"][1])
            component["time"] = [min(component["time"][0], row["time"][0]),
                                 max(component["time"][1], row["time"][1])]
            component["segments"] += 1
    for component in components:
        span = component["s"][1] - component["s"][0]
        if span >= .2 - 1e-10:
            return {"route_s_m": component["s"], "horizontal_span_m": span,
                    "time_s": component["time"], "merged_linear_segments": component["segments"],
                    "lateral_centre_bounds_m": [lateral_lo, lateral_hi],
                    "centre_height_above_floor_m": expected_z,
                    "source": "analytic union of clipped exported linear xyz segments"}
    return None


def _bezier_crossing_interval(trajectory, origin, rotation, s_interval, *,
                              expected_z, lateral_bounds_m):
    """Prove a crossing from controls, never chords between replay samples.

    Constant-height, laterally enclosed, forward-monotone control hulls
    continuously cover their endpoint s interval. Whole-section time bounds
    deliberately overestimate the crossing time, preserving a strict order test.
    """
    offset, component = 0., None
    for segment in trajectory["segments"]:
        end_time = offset + float(segment["duration_s"])
        valid = segment["type"] == "cubic_bezier_ease5"
        if valid:
            control = (np.asarray(segment["control_points_xyz"], float) - origin) @ rotation.T
            valid = bool(control.shape == (4, 3) and np.isfinite(control).all()
                         and np.all(np.abs(control[:, 2] - expected_z) <= 1e-7)
                         and np.all(control[:, 1] >= lateral_bounds_m[0] - 1e-10)
                         and np.all(control[:, 1] <= lateral_bounds_m[1] + 1e-10)
                         and np.all(np.diff(control[:, 0]) >= -1e-12))
        if valid:
            lo = max(float(control[0, 0]), s_interval[0])
            hi = min(float(control[-1, 0]), s_interval[1])
            valid = hi > lo + 1e-12
        if valid:
            if component is None or lo > component["route_s_m"][1] + 1e-9:
                component = {"route_s_m": [lo, hi], "time_s": [offset, end_time],
                             "verified_bezier_sections": 1}
            else:
                component["route_s_m"][1] = hi
                component["time_s"][1] = end_time
                component["verified_bezier_sections"] += 1
            span = component["route_s_m"][1] - component["route_s_m"][0]
            if span >= .2 - 1e-10:
                return {**component, "horizontal_span_m": span,
                        "lateral_centre_bounds_m": list(lateral_bounds_m),
                        "centre_height_above_floor_m": expected_z,
                        "source": "continuous monotone Bezier control hull; conservative whole-section time bounds"}
        else:
            component = None
        offset = end_time
    return None


def ordered_uav_gate_evidence(trajectory: dict, scene_manifest: dict,
                              replay: dict) -> dict:
    """Reconcile the frozen ordered light/table gates with an exported mission."""
    rows = np.asarray(trajectory["poses"], float)
    times = np.asarray(trajectory["time_s"], float)
    origin = np.asarray(scene_manifest["route_frame"]["origin_world_m"], float)
    rotation = np.asarray(scene_manifest["route_frame"]["world_to_route"], float)
    local = (rows[:, :3] - origin) @ rotation.T
    light = next(row for row in scene_manifest["manual_geometry"]
                 if row["role"] == "hanging_light_shade")
    light_underside = float(light["mean_route_m"][2] - light["semiaxes_route_m"][2])
    table_top = float(scene_manifest["existing_table_arrangement"]["top_height_above_floor_m"])
    body = .10
    margin = float(scene_manifest["configuration"]["body_clearance_margin_m"])
    light_lateral = float(light["semiaxes_route_m"][1]) - .25
    table_envelope = scene_manifest["existing_table_arrangement"]["route_envelope_m"]
    table_lateral = (float(table_envelope["lower"][1]) + .25,
                     float(table_envelope["upper"][1]) - .25)
    if trajectory.get("interpolation") == "piecewise_bezier":
        low = _bezier_crossing_interval(trajectory, origin, rotation, (-.20, .20),
                                        expected_z=.65,
                                        lateral_bounds_m=(-light_lateral, light_lateral))
        high = _bezier_crossing_interval(trajectory, origin, rotation, (.92, 1.88),
                                         expected_z=1.40, lateral_bounds_m=table_lateral)
    elif trajectory.get("interpolation") == "linear_xyz_yaw":
        low = _crossing_interval(local, times, (-.20, .20), expected_z=.65,
                                 lateral_bounds_m=(-light_lateral, light_lateral))
        high = _crossing_interval(local, times, (.92, 1.88), expected_z=1.40,
                                  lateral_bounds_m=table_lateral)
    else:
        raise ValueError("unsupported ordered-gate trajectory interpolation")
    altitude_range = float(np.ptp(local[:, 2]))
    clearance = replay.get("geometry", {}).get("clearance_lower_m")
    gates = {
        "low_crossing_found": low is not None,
        "high_crossing_found": high is not None,
        "ordered_low_before_high": bool(low and high and low["time_s"][1] < high["time_s"][0]),
        "altitude_range_at_least_0p50_m": altitude_range >= .50 - 1e-12,
        "low_body_top_below_light_with_margin": .65 + body <= light_underside - margin + 1e-12,
        "high_body_bottom_above_table_with_margin": 1.40 - body >= table_top + margin - 1e-12,
        "continuous_clearance_strictly_above_0p05_m": clearance is not None and clearance > .05,
        "independent_replay_passed": bool(replay.get("passed")),
    }
    return {"schema_version": "gs3d.uav-ordered-gates.v1", "low_crossing": low,
            "high_crossing": high, "altitude_range_m": altitude_range,
            "floor_relative_centre_z_range_m": [float(local[:, 2].min()), float(local[:, 2].max())],
            "world_centre_z_range_m": [float(rows[:, 2].min()), float(rows[:, 2].max())],
            "uav_body": {"radius_m": .25, "half_height_m": body, "margin_m": margin},
            "light": {"gaussian_id": light["gaussian_id"],
                      "underside_above_floor_m": light_underside},
            "table": {"source_support_count": scene_manifest["existing_table_arrangement"]["count"],
                      "top_above_floor_m": table_top},
            "continuous_clearance_lower_m": clearance, "gates": gates,
            "all_pass": all(gates.values())}
