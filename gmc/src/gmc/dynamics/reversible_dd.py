"""Reversible differential drive certifier (Guide §12.1, manual §8.2).

For a drift-free unicycle with v and omega of either sign, rotate-in-place
and straight translate (forward or backward) are directly executable, so any
verified TRT PoseCurve is kinematically feasible.  A raw polygon is not a
certificate that its contents are collision-free, so certification also
recomputes whole-curve geometric safety from a bound scene/robot context."""
from dataclasses import dataclass
import time
from typing import ClassVar

import numpy as np
import shapely
from shapely.geometry import MultiPolygon, Polygon

from ..geometry.support import build_oracles
from ..mobility.witness import PoseCurve, SegmentKind
from ..types import CertStatus, Pose2, Result, canonical_angle, rotation2
from ..verification.path import verify_curve


@dataclass(frozen=True)
class ReversibleDiffDrive:
    """Frozen capability declaration for this specific v0 backend."""

    allow_reverse: bool = True
    allow_in_place_rotation: bool = True
    name: ClassVar[str] = "reversible_differential_drive"

    def __post_init__(self):
        if not isinstance(self.allow_reverse, (bool, np.bool_)):
            raise ValueError("allow_reverse must be boolean")
        if not isinstance(self.allow_in_place_rotation, (bool, np.bool_)):
            raise ValueError("allow_in_place_rotation must be boolean")


@dataclass(frozen=True)
class CorridorVerificationContext:
    """Inputs needed to prove both corridor containment and collision safety.

    ``region`` is the local geometric corridor.  The certifier deliberately
    rebuilds all pair oracles from ``scene`` and ``robot`` instead of trusting
    a caller-supplied candidate subset or a prior SAFE boolean.
    """

    region: object
    scene: object
    robot: object
    eps_clear: float
    theta_min: float


def _same_xy(a: Pose2, b: Pose2) -> bool:
    return bool(np.array_equal(np.asarray(a.xy, dtype=float),
                               np.asarray(b.xy, dtype=float)))


def _same_angle(a: float, b: float) -> bool:
    if not (np.isfinite(a) and np.isfinite(b)):
        return False
    return canonical_angle(float(a)) == canonical_angle(float(b))


def _finite_pose(q: Pose2) -> bool:
    return bool(np.asarray(q.xy).shape == (2,)
                and np.all(np.isfinite(q.xy)) and np.isfinite(q.theta))


class ReversibleDDCertifier:
    def certify(self, corridor, q_start: Pose2, q_goal: Pose2,
                dynamics, budget) -> Result[PoseCurve]:
        started = time.perf_counter()
        try:
            max_wall_seconds = float(getattr(budget, "max_wall_seconds"))
        except (AttributeError, TypeError, ValueError):
            return Result(None, CertStatus.UNKNOWN, "invalid_budget")
        if not np.isfinite(max_wall_seconds) or max_wall_seconds < 0.0:
            return Result(None, CertStatus.UNKNOWN, "invalid_budget")
        deadline = started + max_wall_seconds

        def out_of_time() -> bool:
            return time.perf_counter() >= deadline

        if not isinstance(dynamics, ReversibleDiffDrive):
            return Result(None, CertStatus.UNKNOWN, "wrong_dynamics_model")
        if not (dynamics.allow_reverse and dynamics.allow_in_place_rotation):
            return Result(None, CertStatus.UNKNOWN,
                          "unsupported_dynamics_capabilities")
        if not (_finite_pose(q_start) and _finite_pose(q_goal)):
            return Result(None, CertStatus.UNKNOWN, "invalid_endpoint")
        if not isinstance(corridor, CorridorVerificationContext):
            return Result(None, CertStatus.UNKNOWN,
                          "uncertified_corridor_context")
        region = corridor.region
        try:
            bounds = np.asarray(region.bounds, dtype=float)
            area = float(region.area)
            corridor_valid = (
                isinstance(region, (Polygon, MultiPolygon))
                and not region.is_empty and region.is_valid
                and bounds.shape == (4,) and np.all(np.isfinite(bounds))
                and np.isfinite(area) and area > 0.0
            )
            eps_clear = float(corridor.eps_clear)
            theta_min = float(corridor.theta_min)
            tolerances_valid = (np.isfinite(eps_clear) and eps_clear >= 0.0
                                and np.isfinite(theta_min)
                                and theta_min > 0.0)
        except (AttributeError, TypeError, ValueError):
            corridor_valid = False
            tolerances_valid = False
        if not corridor_valid or not tolerances_valid:
            return Result(None, CertStatus.UNKNOWN, "invalid_corridor")
        if out_of_time():
            return Result(None, CertStatus.UNKNOWN, "dynamic_budget_exhausted")
        curve = getattr(budget, "candidate_curve", None)
        if curve is None:
            return Result(None, CertStatus.UNKNOWN, "no_candidate_curve")
        if not isinstance(curve, PoseCurve) or not curve.segments:
            return Result(None, CertStatus.UNKNOWN, "empty_candidate_curve")

        first, last = curve.segments[0], curve.segments[-1]
        if (not _same_xy(first.q0, q_start)
                or not _same_angle(first.q0.theta, q_start.theta)
                or not _same_xy(last.q1, q_goal)
                or not _same_angle(last.q1.theta, q_goal.theta)):
            return Result(None, CertStatus.UNKNOWN, "endpoint_mismatch")

        previous = None
        for seg in curve.segments:
            if out_of_time():
                return Result(None, CertStatus.UNKNOWN,
                              "dynamic_budget_exhausted")
            if seg.kind is SegmentKind.LOCAL_STEERING:
                return Result(None, CertStatus.UNKNOWN,
                              "steering_not_reversible_dd")
            pts = [seg.q0] + list(seg.control_points) + [seg.q1]
            if not all(_finite_pose(p) for p in pts):
                return Result(None, CertStatus.UNKNOWN, "invalid_segment_pose")
            if previous is not None and (
                    not _same_xy(previous.q1, seg.q0)
                    or not _same_angle(previous.q1.theta, seg.q0.theta)):
                return Result(None, CertStatus.UNKNOWN, "discontinuous_curve")

            if seg.kind is SegmentKind.TRANSLATION:
                if not all(_same_angle(p.theta, seg.q0.theta) for p in pts):
                    return Result(None, CertStatus.UNKNOWN,
                                  "translation_changes_heading")
                # Extended precision avoids turning the tolerance into a
                # macroscopic lateral-motion allowance on long segments.
                # Use the same canonical rotation primitive as support
                # geometry.  This gives exact quadrantal headings and avoids
                # a long-double/libm path disagreeing with the model itself.
                heading = np.asarray(
                    rotation2(canonical_angle(seg.q0.theta))[:, 0],
                    dtype=np.longdouble,
                )
                for p0, p1 in zip(pts[:-1], pts[1:], strict=True):
                    displacement = (np.asarray(p1.xy, dtype=np.longdouble)
                                    - np.asarray(p0.xy,
                                                 dtype=np.longdouble))
                    # Reversible drive permits either sign along its heading,
                    # but it cannot translate sideways.  Test every polyline
                    # leg, since endpoints alone could hide a lateral detour.
                    term0 = displacement[0] * heading[1]
                    term1 = displacement[1] * heading[0]
                    lateral = abs(term0 - term1)
                    roundoff = (np.longdouble(64.0)
                                * np.finfo(np.longdouble).eps
                                * max(np.longdouble(1.0), abs(term0),
                                      abs(term1)))
                    if lateral > roundoff:
                        return Result(None, CertStatus.UNKNOWN,
                                      "translation_not_heading_aligned")
                chain = shapely.LineString([tuple(p.xy) for p in pts])
            elif seg.kind is SegmentKind.ROTATION:
                if not all(_same_xy(p, seg.q0) for p in pts):
                    return Result(None, CertStatus.UNKNOWN,
                                  "rotation_changes_position")
                chain = shapely.Point(tuple(seg.q0.xy))
            else:
                return Result(None, CertStatus.UNKNOWN,
                              "unsupported_segment_kind")
            if not region.covers(chain):
                return Result(None, CertStatus.UNKNOWN, "outside_corridor")
            previous = seg
        if out_of_time():
            return Result(None, CertStatus.UNKNOWN,
                          "dynamic_budget_exhausted")
        try:
            # Use every pair, not the compiler's pruned candidate list.  This
            # makes the M9 certificate independent of graph and BVH booleans.
            oracles = build_oracles(corridor.scene, corridor.robot)
            geometric = verify_curve(
                oracles, corridor.scene.workspace, curve,
                eps_clear, theta_min,
                expected_start=q_start, expected_goal=q_goal,
            )
        except (AttributeError, TypeError, ValueError, FloatingPointError) as exc:
            return Result(
                None, CertStatus.UNKNOWN, "geometric_verification_error",
                uncertainty_sources=(type(exc).__name__,),
            )
        if out_of_time():
            return Result(None, CertStatus.UNKNOWN,
                          "dynamic_budget_exhausted")
        if not geometric.certified:
            return Result(
                None, CertStatus.UNKNOWN, "geometric_verification_failed",
                uncertainty_sources=(geometric.reason,),
                diagnostics={
                    "failed_segment": (
                        -1 if geometric.failed_segment is None
                        else int(geometric.failed_segment)),
                    "min_clearance": float(geometric.min_clearance),
                },
            )
        return Result(
            curve, CertStatus.CERTIFIED, "trt_executable_and_verified",
            diagnostics={"min_clearance": float(geometric.min_clearance)},
        )
