"""M8 whole-path verification (Guide §11.2): C0 continuity, workspace
containment (positions, including holes), per-segment continuous safety, and
the eps_clear margin. Recomputes everything from supports and the curve."""
from dataclasses import dataclass

import numpy as np
import shapely

from ..mobility.witness import PoseCurve, SegmentKind
from ..types import Pose2, canonical_angle
from .continuous import rotation_interval_safe, translation_safe


@dataclass(frozen=True)
class VerifyReport:
    certified: bool
    min_clearance: float
    failed_segment: int | None
    reason: str


def verify_curve(oracles, workspace, curve: PoseCurve, eps_clear: float,
                 theta_min: float, *, expected_start: Pose2 | None = None,
                 expected_goal: Pose2 | None = None) -> VerifyReport:
    eps_clear, theta_min = float(eps_clear), float(theta_min)
    if not np.isfinite(eps_clear) or eps_clear < 0.0:
        raise ValueError("eps_clear must be finite and non-negative")
    if not np.isfinite(theta_min) or theta_min <= 0.0:
        raise ValueError("theta_min must be finite and positive")
    if (workspace is None or workspace.is_empty or not workspace.is_valid
            or not np.isfinite(workspace.area) or workspace.area <= 0.0):
        return VerifyReport(False, -np.inf, None, "invalid_workspace")
    if curve is None or not curve.segments:
        return VerifyReport(False, -np.inf, None, "empty_curve")
    # Reuse the same complete obstacle set for every primitive.  Without this
    # freeze, a one-shot iterable could be consumed by the first segment and
    # silently turn all later collision checks into empty-scene checks.
    oracles = tuple(oracles)

    def pose_values(pose):
        try:
            xy = np.asarray(pose.xy, dtype=float)
            theta = float(pose.theta)
        except (AttributeError, TypeError, ValueError):
            return None
        if (xy.shape != (2,) or not np.all(np.isfinite(xy))
                or not np.isfinite(theta)):
            return None
        return xy, theta

    def same_xy(a, b):
        # A verifier must not silently widen the requested path by a fixed
        # metric tolerance: at large coordinates or exact contact, even a
        # sub-nanometre displacement can change collision status.
        return bool(np.array_equal(np.asarray(a, dtype=float),
                                   np.asarray(b, dtype=float)))

    def same_angle(a, b):
        # Preserve periodic equivalence without treating a small, non-zero
        # rotation as zero.
        return canonical_angle(float(a)) == canonical_angle(float(b))

    def matches_endpoint(actual, expected):
        expected_values = pose_values(expected)
        return (expected_values is not None
                and same_xy(actual[0], expected_values[0])
                and same_angle(actual[1], expected_values[1]))

    first_values = pose_values(curve.segments[0].q0)
    last_values = pose_values(curve.segments[-1].q1)
    if first_values is None or last_values is None:
        return VerifyReport(False, -np.inf, 0, "invalid_pose")
    if expected_start is not None and not matches_endpoint(
            first_values, expected_start):
        return VerifyReport(False, -np.inf, 0, "start_endpoint_mismatch")
    if expected_goal is not None and not matches_endpoint(
            last_values, expected_goal):
        return VerifyReport(
            False, -np.inf, len(curve.segments) - 1,
            "goal_endpoint_mismatch")

    def workspace_clearance(geometry):
        """Exact polygonal center-clearance to outer and hole boundaries.

        ``covers`` alone accepts a pose or segment on the boundary.  That is
        correct for a zero-margin membership question, but it cannot support
        an ``eps_clear`` certificate or a positive minimum-clearance report.
        Shapely's distance to the complete polygon boundary includes both the
        exterior ring and every hole ring.
        """
        try:
            distance = float(workspace.boundary.distance(geometry))
            workspace_bounds = np.asarray(workspace.bounds, dtype=float)
            geometry_bounds = np.asarray(geometry.bounds, dtype=float)
        except Exception:
            return -np.inf
        if (not np.isfinite(distance)
                or workspace_bounds.shape != (4,)
                or geometry_bounds.shape != (4,)
                or not np.all(np.isfinite(workspace_bounds))
                or not np.all(np.isfinite(geometry_bounds))):
            return -np.inf
        # GEOS returns a binary64 distance, not a directed rounding bound.
        # Subtract a coordinate-scale forward-error envelope before using it
        # as an eps_clear certificate; otherwise a one-ulp upward result at a
        # slanted boundary can certify a path whose true clearance is <= eps.
        coordinate_scale = max(
            1.0, abs(distance),
            float(np.max(np.abs(workspace_bounds), initial=0.0)),
            float(np.max(np.abs(geometry_bounds), initial=0.0)),
        )
        slack = (64.0 * np.finfo(np.float64).eps * coordinate_scale)
        return float(np.nextafter(distance - slack, -np.inf))

    prev = None
    cmin = np.inf
    for k, seg in enumerate(curve.segments):
        poses = [seg.q0] + list(seg.control_points) + [seg.q1]
        values = [pose_values(p) for p in poses]
        if any(v is None for v in values):
            return VerifyReport(False, float(cmin), k, "invalid_pose")
        pts = [v[0] for v in values]
        thetas = [v[1] for v in values]
        if prev is not None:
            if (not same_xy(pts[0], prev[0])
                    or not same_angle(thetas[0], prev[1])):
                return VerifyReport(False, float(cmin), k, "discontinuity")
        if seg.kind is SegmentKind.TRANSLATION:
            if any(not same_angle(theta, thetas[0])
                   for theta in thetas[1:]):
                return VerifyReport(
                    False, float(cmin), k, "translation_changes_theta")
            chain = shapely.LineString([tuple(p) for p in pts])
            if not workspace.covers(chain):
                return VerifyReport(False, float(cmin), k, "workspace")
            boundary_lb = workspace_clearance(chain)
            cmin = min(cmin, boundary_lb)
            if boundary_lb <= eps_clear:
                return VerifyReport(
                    False, float(cmin), k, "workspace_clearance")
            for a, b in zip(pts[:-1], pts[1:]):
                ok, c = translation_safe(oracles, a, b, thetas[0],
                                         floor=eps_clear)
                cmin = min(cmin, c)
                if not ok:
                    return VerifyReport(False, float(cmin), k, "collision")
        elif seg.kind is SegmentKind.ROTATION:
            if any(not same_xy(p, pts[0]) for p in pts[1:]):
                return VerifyReport(
                    False, float(cmin), k, "rotation_changes_position")
            if not workspace.covers(shapely.Point(tuple(pts[0]))):
                return VerifyReport(False, float(cmin), k, "workspace")
            boundary_lb = workspace_clearance(
                shapely.Point(tuple(pts[0])))
            cmin = min(cmin, boundary_lb)
            if boundary_lb <= eps_clear:
                return VerifyReport(
                    False, float(cmin), k, "workspace_clearance")
            for th0, th1 in zip(thetas[:-1], thetas[1:]):
                ok, c = rotation_interval_safe(
                    oracles, pts[0], th0, th1, theta_min, floor=eps_clear)
                cmin = min(cmin, c)
                if not ok:
                    return VerifyReport(
                        False, float(cmin), k, "rotation_unresolved")
        else:
            return VerifyReport(False, float(cmin), k,
                                "local_steering_not_supported_v0")
        prev = (pts[-1], thetas[-1])
    # the eps_clear demand is enforced as the certification floor of every
    # segment check above (same criterion the lift used) — comparing two
    # independent conservative estimates post-hoc would reject valid curves
    return VerifyReport(True, float(cmin), None, "ok")
