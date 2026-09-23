"""Bounded post-planning smoothing with conservative 3-D curve certificates.

The smoother never turns an unverified input into an executable trajectory.  A
cubic Bezier section is authorized by covering it with a tube around a verified
linear chord: the Bezier convex-hull deviation is added to the oracle margin.
This retains the full Gaussian/body oracle as collision authority.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from time import perf_counter
from typing import Iterable

import numpy as np

from .contracts import BodySpec, GoalRegion, Pose3
from .robots import EvidenceBoundedPlaneSupport
from .validation import angle_delta, validate_goal, verify_linear_trajectory, verify_path
from .oracle import validate_body


EASE_D1_MAX = 15.0 / 8.0
EASE_D2_MAX = 10.0 / math.sqrt(3.0)


@dataclass(frozen=True)
class SmoothingConfig:
    max_shortcut_span: int = 64
    max_certificate_depth: int = 12
    certificate_deviation_m: float = .01
    duration_safety_factor: float = 1.05
    min_turn_improvement_fraction: float = .10
    metric_samples_per_segment: int = 101

    def __post_init__(self):
        values = (self.certificate_deviation_m, self.duration_safety_factor,
                  self.min_turn_improvement_fraction)
        if (self.max_shortcut_span < 1 or self.max_certificate_depth < 0
                or self.metric_samples_per_segment < 3
                or not np.isfinite(values).all()
                or self.certificate_deviation_m <= 0
                or self.duration_safety_factor < 1
                or not 0 <= self.min_turn_improvement_fraction < 1):
            raise ValueError("invalid smoothing configuration")


def _rows(trajectory: dict) -> tuple[np.ndarray, np.ndarray]:
    rows = np.asarray(trajectory.get("poses"), float)
    times = np.asarray(trajectory.get("time_s"), float)
    if (rows.ndim != 2 or rows.shape[1] != 4 or len(rows) < 1
            or times.shape != (len(rows),) or not np.isfinite(rows).all()
            or not np.isfinite(times).all() or times[0] != 0
            or (len(times) > 1 and np.any(np.diff(times) <= 0))):
        raise ValueError("invalid raw trajectory")
    return rows, times


def _unique_positions(rows: np.ndarray) -> tuple[np.ndarray, list[int]]:
    keep = [0]
    for index in range(1, len(rows)):
        if np.linalg.norm(rows[index, :3] - rows[keep[-1], :3]) > 1e-10:
            keep.append(index)
    return rows[keep, :3].copy(), keep


def _pose(point, yaw=0.) -> Pose3:
    return Pose3(tuple(map(float, point)), float(yaw))


def _shortcut(points: np.ndarray, oracle, body: BodySpec, margin_m: float,
              protected: set[int], max_span: int) -> tuple[np.ndarray, list[int], int]:
    """Greedy farthest-visible shortcutting while retaining protected indices."""
    kept, attempts = [0], 0
    current = 0
    while current < len(points) - 1:
        next_protected = min((i for i in protected if i > current), default=len(points) - 1)
        upper = min(next_protected, current + max_span)
        chosen = current + 1
        for candidate in range(upper, current, -1):
            attempts += 1
            report = oracle.edge(_pose(points[current]), _pose(points[candidate]), body,
                                 margin_m=margin_m)
            if report.occupancy == "free" and report.safety == "continuous_bound":
                chosen = candidate
                break
        kept.append(chosen)
        current = chosen
    return points[kept], kept, attempts


def _natural_beziers(points: np.ndarray) -> list[np.ndarray]:
    """Natural cubic interpolant converted exactly to cubic Bezier sections."""
    if len(points) < 2:
        return []
    if len(points) == 2:
        delta = (points[1] - points[0]) / 3.
        return [np.asarray([points[0], points[0] + delta,
                            points[0] + 2 * delta, points[1]])]
    h = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if np.any(h <= 1e-10):
        raise ValueError("spline anchors must be spatially distinct")
    count = len(points)
    matrix = np.zeros((count, count))
    rhs = np.zeros((count, 3))
    matrix[0, 0] = matrix[-1, -1] = 1.
    for i in range(1, count - 1):
        matrix[i, i - 1], matrix[i, i], matrix[i, i + 1] = h[i - 1], 2 * (h[i - 1] + h[i]), h[i]
        rhs[i] = 6 * ((points[i + 1] - points[i]) / h[i]
                      - (points[i] - points[i - 1]) / h[i - 1])
    second = np.linalg.solve(matrix, rhs)
    sections = []
    for i, width in enumerate(h):
        start_derivative = ((points[i + 1] - points[i]) / width
                            - width * (2 * second[i] + second[i + 1]) / 6)
        end_derivative = ((points[i + 1] - points[i]) / width
                          + width * (second[i] + 2 * second[i + 1]) / 6)
        sections.append(np.asarray([points[i], points[i] + width * start_derivative / 3,
                                    points[i + 1] - width * end_derivative / 3,
                                    points[i + 1]], float))
    return sections


def _split(control: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    ab = (control[:-1] + control[1:]) / 2
    abc = (ab[:-1] + ab[1:]) / 2
    centre = (abc[0] + abc[1]) / 2
    return np.asarray([control[0], ab[0], abc[0], centre]), np.asarray([centre, abc[1], ab[2], control[3]])


def _point_segment_distance(point, a, b) -> float:
    direction = b - a
    scale = float(direction @ direction)
    if scale == 0:
        return float(np.linalg.norm(point - a))
    u = float(np.clip((point - a) @ direction / scale, 0., 1.))
    return float(np.linalg.norm(point - (a + u * direction)))


def _deviation(control: np.ndarray) -> float:
    return max(_point_segment_distance(point, control[0], control[-1])
               for point in control[1:-1])


def _curve_domain(control: np.ndarray, prepared, body: BodySpec, margin_m: float) -> tuple[bool, str]:
    half = np.asarray([body.radius_m, body.radius_m, body.half_height_m])
    lower, upper = control.min(axis=0) - half, control.max(axis=0) + half
    slack = 1e-10
    if np.any(lower - prepared.lower <= margin_m + slack) or np.any(prepared.upper - upper <= margin_m + slack):
        return False, "curve_workspace_margin_unproven"
    if not prepared.scene.known_space.contains_aabb(tuple(lower), tuple(upper)):
        return False, "curve_map_unknown"
    if body.motion == "ground_unicycle":
        support = prepared.scene.support
        if support is None:
            return False, "curve_ground_support_missing"
        xy_lower = control[:, :2].min(axis=0) - body.radius_m
        xy_upper = control[:, :2].max(axis=0) + body.radius_m
        bounds = support.height_bounds(tuple(xy_lower), tuple(xy_upper))
        if bounds is None:
            return False, "curve_ground_footprint_support_unproven"
        # Control-point heights imply an entire Bezier lies on a surface only
        # for an affine plane (or a provider proving constant height throughout
        # this whole footprint). Arbitrary nonlinear supports need a curve API.
        if (not isinstance(support, EvidenceBoundedPlaneSupport)
                and bounds[0] != bounds[1]):
            return False, "curve_nonaffine_support_unproven"
        max_travel = getattr(support, "max_travel_m", 0.)
        if bounds[1] - bounds[0] > max_travel + 1e-10:
            return False, "curve_ground_support_travel_exceeded"
        expected = [support.height(float(p[0]), float(p[1])) for p in control]
        offset = body.ground_clearance_m + body.half_height_m
        if any(z is None for z in expected) or not np.allclose(
                control[:, 2], np.asarray(expected) + offset, rtol=0., atol=1e-7):
            return False, "curve_off_support_manifold"
    return True, "curve_domain_verified"


def _certify_section(control: np.ndarray, oracle, body: BodySpec, margin_m: float,
                     config: SmoothingConfig) -> dict:
    leaves, worst_depth, clearance = 0, 0, math.inf
    stack = [(control, 0)]
    failures = []
    while stack:
        piece, depth = stack.pop()
        worst_depth = max(worst_depth, depth)
        valid, reason = _curve_domain(piece, oracle.prepared, body, margin_m)
        if not valid:
            return {"passed": False, "reason": reason, "leaves": leaves,
                    "max_depth": worst_depth, "clearance_lower_m": None}
        deviation = _deviation(piece)
        if deviation > config.certificate_deviation_m and depth < config.max_certificate_depth:
            left, right = _split(piece)
            stack.extend(((right, depth + 1), (left, depth + 1)))
            continue
        report = oracle.edge(_pose(piece[0]), _pose(piece[-1]), body,
                             margin_m=margin_m + deviation)
        if report.occupancy == "free" and report.safety == "continuous_bound":
            leaves += 1
            clearance = min(clearance, float(report.clearance_lower_m) - deviation)
            continue
        if depth < config.max_certificate_depth:
            left, right = _split(piece)
            stack.extend(((right, depth + 1), (left, depth + 1)))
        else:
            failures.append({"reason": report.reason, "occupancy": report.occupancy,
                             "safety": report.safety, "deviation_m": deviation})
            break
    return {"passed": not failures,
            "reason": "bezier_tube_continuous_bound" if not failures else "curve_certificate_failed",
            "leaves": leaves, "max_depth": worst_depth,
            "clearance_lower_m": clearance if not failures else None,
            "failures": failures}


def _geometry_bounds(control: np.ndarray, *, tangent_yaw: bool) -> dict:
    first = 3 * np.diff(control, axis=0)
    second = 6 * (control[2:] - 2 * control[1:-1] + control[:-2])
    third = control[3] - 3 * control[2] + 3 * control[1] - control[0]
    third *= 6
    speed_u = max(map(float, np.linalg.norm(first, axis=1)))
    acceleration_u = max(map(float, np.linalg.norm(second, axis=1)))
    vertical_u = max(map(float, np.abs(first[:, 2])))
    result = {"position_d1_bound_m": speed_u, "position_d2_bound_m": acceleration_u,
              "vertical_d1_bound_m": vertical_u, "yaw_d1_bound_rad": 0.,
              "yaw_d2_bound_rad": 0.}
    if tangent_yaw:
        planar = first[:, :2]
        chord = control[-1, :2] - control[0, :2]
        chord_norm = np.linalg.norm(chord)
        if chord_norm <= 1e-12:
            raise ValueError("ground Bezier section has zero chord")
        axis = chord / chord_norm
        tangent_floor = float(np.min(planar @ axis))
        if tangent_floor <= 1e-8:
            raise ValueError("ground spline tangent may stop or reverse")
        velocity = max(map(float, np.linalg.norm(planar, axis=1)))
        acceleration = max(map(float, np.linalg.norm(second[:, :2], axis=1)))
        jerk = float(np.linalg.norm(third[:2]))
        theta1 = velocity * acceleration / tangent_floor ** 2
        theta2 = (velocity * jerk / tangent_floor ** 2
                  + 2 * velocity ** 2 * acceleration ** 2 / tangent_floor ** 4)
        result.update({"tangent_projection_floor_m": tangent_floor,
                       "yaw_d1_bound_rad": theta1, "yaw_d2_bound_rad": theta2})
    return result


def _duration(bounds: dict, limits: dict, *, tangent_yaw: bool,
              safety_factor: float) -> float:
    requirements = [bounds["position_d1_bound_m"] * EASE_D1_MAX / limits["max_speed_mps"]]
    if limits["max_acceleration_mps2"] > 0:
        requirements.append(math.sqrt((bounds["position_d2_bound_m"] * EASE_D1_MAX ** 2
                                       + bounds["position_d1_bound_m"] * EASE_D2_MAX)
                                      / limits["max_acceleration_mps2"]))
    if bounds["vertical_d1_bound_m"] > 0:
        if limits["max_vertical_speed_mps"] <= 0:
            raise ValueError("vertical spline motion forbidden by limits")
        requirements.append(bounds["vertical_d1_bound_m"] * EASE_D1_MAX
                            / limits["max_vertical_speed_mps"])
    if tangent_yaw:
        requirements.append(bounds["yaw_d1_bound_rad"] * EASE_D1_MAX
                            / limits["max_yaw_rate_radps"])
        if limits["max_yaw_acceleration_radps2"] > 0:
            requirements.append(math.sqrt((bounds["yaw_d2_bound_rad"] * EASE_D1_MAX ** 2
                                           + bounds["yaw_d1_bound_rad"] * EASE_D2_MAX)
                                          / limits["max_yaw_acceleration_radps2"]))
    return max(.05, max(requirements) * safety_factor)


def _ease(u):
    return u ** 3 * (10 + u * (-15 + 6 * u))


def _bezier(control: np.ndarray, u):
    u = np.asarray(u, float)[..., None]
    return ((1 - u) ** 3 * control[0] + 3 * (1 - u) ** 2 * u * control[1]
            + 3 * (1 - u) * u ** 2 * control[2] + u ** 3 * control[3])


def _bezier_d1(control: np.ndarray, u):
    u = np.asarray(u, float)[..., None]
    delta = np.diff(control, axis=0)
    return 3 * ((1 - u) ** 2 * delta[0] + 2 * (1 - u) * u * delta[1] + u ** 2 * delta[2])


def _rotation_segment(point, start_yaw, end_yaw, limits, safety_factor):
    delta = angle_delta(end_yaw, start_yaw)
    if abs(delta) <= 1e-10:
        return None
    duration = safety_factor * max(abs(delta) * EASE_D1_MAX / limits["max_yaw_rate_radps"],
                                   math.sqrt(abs(delta) * EASE_D2_MAX
                                             / limits["max_yaw_acceleration_radps2"]), .05)
    return {"type": "rotate_ease5", "position_xyz": list(map(float, point)),
            "yaw_start_rad": float(start_yaw), "yaw_delta_rad": float(delta),
            "duration_s": duration,
            "dynamics_bounds": {"max_speed_mps": 0., "max_acceleration_mps2": 0.,
                                 "max_yaw_rate_radps": abs(delta) * EASE_D1_MAX / duration,
                                 "max_yaw_acceleration_radps2": abs(delta) * EASE_D2_MAX / duration ** 2}}


def _translation_segment(control, body, limits, safety, config, *, constant_yaw=0.):
    tangent = body.motion == "ground_unicycle"
    bounds = _geometry_bounds(control, tangent_yaw=tangent)
    duration = _duration(bounds, limits, tangent_yaw=tangent,
                         safety_factor=config.duration_safety_factor)
    yaw0 = float(math.atan2(*_bezier_d1(control, 0.)[:2][::-1])) if tangent else float(constant_yaw)
    yaw1 = float(math.atan2(*_bezier_d1(control, 1.)[:2][::-1])) if tangent else float(constant_yaw)
    dynamics = {
        "max_speed_mps": bounds["position_d1_bound_m"] * EASE_D1_MAX / duration,
        "max_vertical_speed_mps": bounds["vertical_d1_bound_m"] * EASE_D1_MAX / duration,
        "max_acceleration_mps2": (bounds["position_d2_bound_m"] * EASE_D1_MAX ** 2
                                   + bounds["position_d1_bound_m"] * EASE_D2_MAX) / duration ** 2,
        "max_yaw_rate_radps": bounds["yaw_d1_bound_rad"] * EASE_D1_MAX / duration,
        "max_yaw_acceleration_radps2": (bounds["yaw_d2_bound_rad"] * EASE_D1_MAX ** 2
                                        + bounds["yaw_d1_bound_rad"] * EASE_D2_MAX) / duration ** 2,
    }
    return {"type": "cubic_bezier_ease5", "control_points_xyz": control.tolist(),
            "yaw_profile": "tangent_of_xyz_bezier" if tangent else "constant",
            "yaw_start_rad": yaw0, "yaw_end_rad": yaw1, "duration_s": duration,
            "geometry_derivative_bounds": bounds, "dynamics_bounds": dynamics,
            "continuous_safety": safety}


def _sample_segments(segments: list[dict], dt_s: float) -> tuple[np.ndarray, np.ndarray]:
    rows, times, offset = [], [], 0.
    for index, segment in enumerate(segments):
        duration = float(segment["duration_s"])
        count = max(2, int(math.ceil(duration / dt_s)) + 1)
        local_t = np.linspace(0., duration, count)
        progress = _ease(local_t / duration)
        if segment["type"] == "rotate_ease5":
            xyz = np.repeat(np.asarray(segment["position_xyz"])[None, :], count, axis=0)
            yaw = segment["yaw_start_rad"] + segment["yaw_delta_rad"] * progress
        else:
            control = np.asarray(segment["control_points_xyz"])
            xyz = _bezier(control, progress)
            if segment["yaw_profile"] == "tangent_of_xyz_bezier":
                derivative = _bezier_d1(control, progress)
                yaw = np.unwrap(np.arctan2(derivative[:, 1], derivative[:, 0]))
            else:
                yaw = np.full(count, segment["yaw_start_rad"])
        if rows:
            yaw += 2 * math.pi * round((rows[-1][3] - yaw[0]) / (2 * math.pi))
        start = 0 if index == 0 else 1
        rows.extend(np.c_[xyz, yaw][start:].tolist())
        times.extend((offset + local_t[start:]).tolist())
        offset += duration
    return np.asarray(rows), np.asarray(times)


def _turn_metric_from_vectors(vectors: np.ndarray) -> float:
    vectors = vectors[np.linalg.norm(vectors, axis=1) > 1e-10]
    if len(vectors) < 2:
        return 0.
    unit = vectors / np.linalg.norm(vectors, axis=1)[:, None]
    dots = np.clip(np.sum(unit[:-1] * unit[1:], axis=1), -1., 1.)
    return float(np.sum(np.arccos(dots)))


def raw_metrics(trajectory: dict, body: BodySpec) -> dict:
    rows, times = _rows(trajectory)
    differences = np.diff(rows[:, :3], axis=0)
    moving = differences[np.linalg.norm(differences, axis=1) > 1e-10]
    vectors = moving[:, :2] if body.motion == "ground_unicycle" else moving
    turn = _turn_metric_from_vectors(vectors)
    lengths = np.linalg.norm(moving, axis=1)
    headings = []
    for a, b in zip(vectors, vectors[1:]):
        angle = math.acos(float(np.clip(a @ b / np.linalg.norm(a) / np.linalg.norm(b), -1., 1.)))
        headings.append(angle / max(.5 * (np.linalg.norm(a) + np.linalg.norm(b)), 1e-12))
    return {"path_length_m": float(lengths.sum()), "turn_metric_rad": turn,
            "discrete_curvature_proxy_max_radpm": max(headings, default=0.),
            "altitude_range_m": float(np.ptp(rows[:, 2])), "duration_s": float(times[-1]),
            "translation_segments": int(len(moving))}


def smooth_metrics(segments: list[dict], body: BodySpec, samples_per_segment: int) -> dict:
    points, tangents = [], []
    for segment in segments:
        if segment["type"] != "cubic_bezier_ease5":
            continue
        control = np.asarray(segment["control_points_xyz"])
        u = np.linspace(0., 1., samples_per_segment)
        xyz, derivative = _bezier(control, u), _bezier_d1(control, u)
        if points:
            xyz, derivative = xyz[1:], derivative[1:]
        points.extend(xyz.tolist()); tangents.extend(derivative.tolist())
    points, tangents = np.asarray(points), np.asarray(tangents)
    if len(points) < 2:
        return {"path_length_m": 0., "turn_metric_rad": 0., "sampled_curvature_max_radpm": 0.,
                "altitude_range_m": 0., "duration_s": sum(s["duration_s"] for s in segments)}
    vectors = tangents[:, :2] if body.motion == "ground_unicycle" else tangents
    turn = _turn_metric_from_vectors(vectors)
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    curvature = []
    for index in range(len(vectors) - 1):
        a, b = vectors[index:index + 2]
        if np.linalg.norm(a) > 1e-10 and np.linalg.norm(b) > 1e-10:
            angle = math.acos(float(np.clip(a @ b / np.linalg.norm(a) / np.linalg.norm(b), -1., 1.)))
            curvature.append(angle / max(steps[min(index, len(steps) - 1)], 1e-12))
    return {"path_length_m": float(steps.sum()), "turn_metric_rad": turn,
            "sampled_curvature_max_radpm": max(curvature, default=0.),
            "altitude_range_m": float(np.ptp(points[:, 2])),
            "duration_s": float(sum(s["duration_s"] for s in segments)),
            "metric_sampling": {"samples_per_bezier": samples_per_segment,
                                 "not_used_for_collision_authorization": True}}


def verify_piecewise_bezier(trajectory: dict, oracle, body: BodySpec, limits: dict,
                            *, margin_m: float, goal: GoalRegion,
                            config: SmoothingConfig) -> dict:
    segments = trajectory.get("segments")
    if trajectory.get("interpolation") != "piecewise_bezier" or not isinstance(segments, list) or not segments:
        return {"passed": False, "reason": "invalid_piecewise_bezier_schema"}
    try:
        validate_body(body)
        validate_goal(goal)
        required_limits = ("max_speed_mps", "max_vertical_speed_mps",
                           "max_acceleration_mps2", "max_yaw_rate_radps",
                           "max_yaw_acceleration_radps2")
        if (not np.isfinite([limits[key] for key in required_limits]).all()
                or any(limits[key] < 0 for key in required_limits)
                or not math.isfinite(margin_m) or margin_m < 0):
            raise ValueError("invalid smooth limits or margin")
        dt_s = float(trajectory["control_dt_s"])
        if not math.isfinite(dt_s) or dt_s <= 0:
            raise ValueError("invalid control dt")
        previous_yaw = None
        for segment in segments:
            duration = float(segment["duration_s"])
            if not math.isfinite(duration) or duration <= 0:
                raise ValueError("invalid_smooth_segment_duration")
            if segment["type"] == "rotate_ease5":
                yaw_start = float(segment["yaw_start_rad"])
                yaw_end = yaw_start + float(segment["yaw_delta_rad"])
            elif segment["type"] == "cubic_bezier_ease5":
                control = np.asarray(segment["control_points_xyz"], float)
                if control.shape != (4, 3) or not np.isfinite(control).all():
                    raise ValueError("invalid_bezier_controls_or_yaw_profile")
                if body.motion == "ground_unicycle":
                    _geometry_bounds(control, tangent_yaw=True)
                    first, last = control[1] - control[0], control[-1] - control[-2]
                    yaw_start, yaw_end = math.atan2(first[1], first[0]), math.atan2(last[1], last[0])
                else:
                    yaw_start = yaw_end = float(segment["yaw_start_rad"])
            else:
                raise ValueError("unknown_smooth_segment_type")
            if not np.isfinite([yaw_start, yaw_end]).all():
                raise ValueError("invalid_smooth_segment_yaw")
            if previous_yaw is not None and abs(angle_delta(yaw_start, previous_yaw)) > 1e-9:
                raise ValueError("smooth_segment_yaw_discontinuity")
            previous_yaw = yaw_end
        replay_rows, replay_times = _sample_segments(segments, dt_s)
        stored_rows, stored_times = _rows(trajectory)
        if (replay_rows.shape != stored_rows.shape or replay_times.shape != stored_times.shape
                or not np.allclose(replay_rows, stored_rows, rtol=0., atol=1e-10)
                or not np.allclose(replay_times, stored_times, rtol=0., atol=1e-10)):
            raise ValueError("serialized smooth replay differs from authoritative segments")
    except (ValueError, TypeError, KeyError, IndexError) as exc:
        return {"passed": False, "reason": str(exc)}
    expected = np.asarray(segments[0].get("position_xyz", segments[0].get("control_points_xyz", [None])[0]), float)
    minimum_clearance, leaves, max_depth = math.inf, 0, 0
    maxima = {key: 0. for key in ("max_speed_mps", "max_vertical_speed_mps",
                                   "max_acceleration_mps2", "max_yaw_rate_radps",
                                   "max_yaw_acceleration_radps2")}
    for segment in segments:
        duration = float(segment.get("duration_s", math.nan))
        if not math.isfinite(duration) or duration <= 0:
            return {"passed": False, "reason": "invalid_smooth_segment_duration"}
        if segment["type"] == "rotate_ease5":
            start = end = np.asarray(segment["position_xyz"], float)
            delta = float(segment["yaw_delta_rad"])
            if start.shape != (3,) or not np.isfinite([*start, delta]).all():
                return {"passed": False, "reason": "invalid_rotation_segment"}
            safety = oracle.pose(_pose(start), body, margin_m=margin_m)
            if safety.occupancy != "free" or safety.safety != "continuous_bound":
                return {"passed": False, "reason": safety.reason}
            minimum_clearance = min(minimum_clearance, float(safety.clearance_lower_m))
            actual_dynamics = {"max_speed_mps": 0., "max_vertical_speed_mps": 0.,
                               "max_acceleration_mps2": 0.,
                               "max_yaw_rate_radps": abs(delta) * EASE_D1_MAX / duration,
                               "max_yaw_acceleration_radps2": abs(delta) * EASE_D2_MAX / duration ** 2}
        elif segment["type"] == "cubic_bezier_ease5":
            control = np.asarray(segment["control_points_xyz"], float)
            required_profile = "tangent_of_xyz_bezier" if body.motion == "ground_unicycle" else "constant"
            if (control.shape != (4, 3) or not np.isfinite(control).all()
                    or segment.get("yaw_profile") != required_profile):
                return {"passed": False, "reason": "invalid_bezier_controls_or_yaw_profile"}
            start, end = control[0], control[-1]
            certificate = _certify_section(control, oracle, body, margin_m, config)
            if not certificate["passed"]:
                return {"passed": False, "reason": certificate["reason"], "certificate": certificate}
            minimum_clearance = min(minimum_clearance, certificate["clearance_lower_m"])
            leaves += certificate["leaves"]; max_depth = max(max_depth, certificate["max_depth"])
            try:
                bounds = _geometry_bounds(control, tangent_yaw=body.motion == "ground_unicycle")
            except ValueError as exc:
                return {"passed": False, "reason": str(exc)}
            actual_dynamics = {
                "max_speed_mps": bounds["position_d1_bound_m"] * EASE_D1_MAX / duration,
                "max_vertical_speed_mps": bounds["vertical_d1_bound_m"] * EASE_D1_MAX / duration,
                "max_acceleration_mps2": (bounds["position_d2_bound_m"] * EASE_D1_MAX ** 2
                                           + bounds["position_d1_bound_m"] * EASE_D2_MAX) / duration ** 2,
                "max_yaw_rate_radps": bounds["yaw_d1_bound_rad"] * EASE_D1_MAX / duration,
                "max_yaw_acceleration_radps2": (bounds["yaw_d2_bound_rad"] * EASE_D1_MAX ** 2
                                                + bounds["yaw_d1_bound_rad"] * EASE_D2_MAX) / duration ** 2,
            }
        else:
            return {"passed": False, "reason": "unknown_smooth_segment_type"}
        if not np.allclose(start, expected, rtol=0., atol=1e-9):
            return {"passed": False, "reason": "smooth_segment_position_discontinuity"}
        expected = end
        for key, value in actual_dynamics.items():
            maxima[key] = max(maxima[key], float(value))
    checks = {
        "speed": maxima["max_speed_mps"] <= limits["max_speed_mps"] + 1e-9,
        "vertical_speed": maxima["max_vertical_speed_mps"] <= limits["max_vertical_speed_mps"] + 1e-9,
        "acceleration": maxima["max_acceleration_mps2"] <= limits["max_acceleration_mps2"] + 1e-9,
        "yaw_rate": maxima["max_yaw_rate_radps"] <= limits["max_yaw_rate_radps"] + 1e-9,
        "yaw_acceleration": maxima["max_yaw_acceleration_radps2"] <= limits["max_yaw_acceleration_radps2"] + 1e-9,
    }
    terminal = _pose(stored_rows[-1, :3], stored_rows[-1, 3])
    goal_ok = (np.linalg.norm(stored_rows[-1, :3] - np.asarray(goal.original.xyz))
               <= goal.position_tolerance_m + 1e-9
               and abs(angle_delta(terminal.yaw, goal.original.yaw))
               <= goal.yaw_tolerance_rad + 1e-9)
    return {"passed": bool(all(checks.values()) and goal_ok),
            "reason": "continuous_curve_and_dynamics_bounds_verified" if all(checks.values()) and goal_ok else "dynamics_or_goal_failed",
            "safety": "continuous_bound", "clearance_lower_m": minimum_clearance,
            "certificate_leaves": leaves, "certificate_max_depth": max_depth,
            "dynamics_bounds": maxima, "limit_checks": checks, "goal_reached": bool(goal_ok),
            "continuity": "position velocity and acceleration are continuous; ease5 makes velocity and acceleration zero at segment joins"}


def optimize_trajectory(raw_trajectory: dict, oracle, body: BodySpec, limits: dict,
                        *, margin_m: float, goal: GoalRegion,
                        protected_position_indices: Iterable[int] = (),
                        config: SmoothingConfig = SmoothingConfig()) -> dict:
    """Optimize a verified raw path and select it only after complete revalidation."""
    started = perf_counter()
    rows, _ = _rows(raw_trajectory)
    raw_poses = [_pose(row[:3], row[3]) for row in rows]
    raw_geometry = verify_path(oracle, raw_poses, body, margin_m=margin_m, goal=goal)
    raw_dynamics = verify_linear_trajectory(raw_trajectory, body, limits)
    raw_verified = bool(raw_geometry["passed"] and raw_dynamics["passed"])
    fallback = {"retained": True, "verified": raw_verified,
                "geometry": {key: raw_geometry.get(key) for key in ("passed", "safety", "reason", "clearance_lower_m")},
                "kinematics": raw_dynamics}
    if not raw_verified:
        return {"schema_version": "gs3d.smoothing.v1", "selected": "none",
                "success": False, "reason": "raw_input_not_verified", "raw_fallback": fallback,
                "optimizer_wall_s": perf_counter() - started}
    positions, original_indices = _unique_positions(rows)
    protected = {0, len(positions) - 1}
    requested = set(map(int, protected_position_indices))
    if any(i < 0 or i >= len(positions) for i in requested):
        raise ValueError("protected position index out of bounds")
    protected |= requested
    anchors, kept, shortcut_attempts = _shortcut(positions, oracle, body, margin_m,
                                                  protected, config.max_shortcut_span)
    try:
        # Protected positions are hard mission boundaries.  They split the
        # spline, so a low traverse and vertical rise cannot be rounded into a
        # diagonal shortcut.  Ease timing stops with zero v/a at each section.
        protected_anchor_rows = sorted(kept.index(i) for i in protected)
        natural_controls = []
        for first, last in zip(protected_anchor_rows, protected_anchor_rows[1:]):
            natural_controls.extend(_natural_beziers(anchors[first:last + 1]))
        smooth_segments, spline_tension, rejected_tensions = [], None, []
        for tension in (1., .5, .25, .125, 0.):
            trial = []
            failure = None
            for natural in natural_controls:
                delta = natural[-1] - natural[0]
                linear = np.asarray([natural[0], natural[0] + delta / 3,
                                     natural[0] + 2 * delta / 3, natural[-1]])
                control = linear + tension * (natural - linear)
                certificate = _certify_section(control, oracle, body, margin_m, config)
                if not certificate["passed"]:
                    failure = certificate["reason"]
                    break
                trial.append(_translation_segment(control, body, limits,
                                                   certificate, config,
                                                   constant_yaw=rows[0, 3]))
            if failure is None:
                smooth_segments, spline_tension = trial, tension
                break
            rejected_tensions.append({"tension": tension, "reason": failure})
        if spline_tension is None:
            raise ValueError(rejected_tensions[-1]["reason"])
        if not smooth_segments:
            raise ValueError("no_translation_curve")
        segments = []
        if body.motion == "ground_unicycle":
            rotation = _rotation_segment(anchors[0], rows[0, 3],
                                         smooth_segments[0]["yaw_start_rad"], limits,
                                         config.duration_safety_factor)
            if rotation: segments.append(rotation)
        for index, segment in enumerate(smooth_segments):
            if index and body.motion == "ground_unicycle":
                rotation = _rotation_segment(
                    segment["control_points_xyz"][0],
                    smooth_segments[index - 1]["yaw_end_rad"], segment["yaw_start_rad"],
                    limits, config.duration_safety_factor)
                if rotation: segments.append(rotation)
            segments.append(segment)
        if body.motion == "ground_unicycle":
            rotation = _rotation_segment(anchors[-1], smooth_segments[-1]["yaw_end_rad"],
                                         goal.original.yaw, limits,
                                         config.duration_safety_factor)
            if rotation: segments.append(rotation)
        samples, sample_times = _sample_segments(segments, float(raw_trajectory["control_dt_s"]))
        trajectory = {"poses": samples.tolist(), "time_s": sample_times.tolist(),
                      "interpolation": "piecewise_bezier", "segments": segments,
                      "control_dt_s": float(raw_trajectory["control_dt_s"])}
        verification = verify_piecewise_bezier(trajectory, oracle, body, limits,
                                                margin_m=margin_m, goal=goal, config=config)
        before, after = raw_metrics(raw_trajectory, body), smooth_metrics(
            segments, body, config.metric_samples_per_segment)
        denominator = max(before["turn_metric_rad"], 1e-12)
        improvement = (before["turn_metric_rad"] - after["turn_metric_rad"]) / denominator
        nontrivial = before["turn_metric_rad"] > 1e-6
        quality = (not nontrivial) or improvement >= config.min_turn_improvement_fraction
        selected = bool(verification["passed"] and quality)
        reason = ("smoothed_verified" if selected else
                  "turn_improvement_below_threshold" if verification["passed"] else verification["reason"])
        return {"schema_version": "gs3d.smoothing.v1", "selected": "smoothed" if selected else "raw_verified_fallback",
                "success": True, "reason": reason,
                "trajectory": trajectory if selected else raw_trajectory,
                "candidate_trajectory": trajectory, "candidate_verification": verification,
                "raw_fallback": fallback, "metrics": {"raw": before, "smoothed_candidate": after,
                    "turn_improvement_fraction": improvement, "required_improvement_fraction": config.min_turn_improvement_fraction,
                    "nontrivial_turn_case": nontrivial},
                "optimization": {"input_position_rows": len(positions), "anchor_rows": len(anchors),
                    "anchor_source_indices": [int(original_indices[i]) for i in kept],
                    "protected_position_indices": sorted(protected), "shortcut_attempts": shortcut_attempts,
                    "spline_tension": spline_tension, "rejected_tensions": rejected_tensions,
                    "config": asdict(config)}, "optimizer_wall_s": perf_counter() - started,
                "limitations": ["collision safety is a floating-point conservative tube bound relative to the map and oracle",
                    "reported curvature maxima and turn metrics are sampled diagnostics, not collision authorization",
                    "upright bodies omit UAV attitude, tracking error, wind, and closed-loop flight dynamics"]}
    except (ValueError, FloatingPointError, np.linalg.LinAlgError) as exc:
        return {"schema_version": "gs3d.smoothing.v1", "selected": "raw_verified_fallback",
                "success": True, "reason": f"candidate_rejected:{exc}",
                "trajectory": raw_trajectory, "raw_fallback": fallback,
                "metrics": {"raw": raw_metrics(raw_trajectory, body)},
                "optimization": {"input_position_rows": len(positions), "anchor_rows": len(anchors),
                                 "shortcut_attempts": shortcut_attempts, "config": asdict(config)},
                "optimizer_wall_s": perf_counter() - started}
