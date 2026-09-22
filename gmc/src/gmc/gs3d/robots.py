"""Robot and ground-support adapters for the shared GS3D oracle.

Ground robots remain upright on an explicitly evidenced support manifold.  This
module does not turn missing support into free space and never projects Gaussian
obstacles into two dimensions.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from gmc.height.ply3d import GaussianScene3D

from .contracts import BodySpec, Pose3


UAV = BodySpec("uav", radius_m=.25, half_height_m=.10,
               motion="uav_translation", ground_clearance_m=0.)
SWEEPER = BodySpec("sweeper", radius_m=.175, half_height_m=.04,
                   motion="ground_unicycle", ground_clearance_m=.02)
CYLINDER = BodySpec("cylinder", radius_m=.30, half_height_m=.865,
                    motion="ground_unicycle", ground_clearance_m=.02)


def robot_catalog() -> dict[str, BodySpec]:
    """Return the frozen physical body dimensions as a fresh mapping."""
    return {body.name: body for body in (UAV, SWEEPER, CYLINDER)}


@dataclass(frozen=True)
class BoxKnownSpace:
    """Closed known map-domain box with optional closed unknown holes."""

    lower: tuple[float, float, float]
    upper: tuple[float, float, float]
    holes: tuple[tuple[tuple[float, float, float],
                       tuple[float, float, float]], ...] = ()

    def __post_init__(self):
        lower, upper = np.asarray(self.lower, float), np.asarray(self.upper, float)
        if (lower.shape != (3,) or upper.shape != (3,)
                or not np.isfinite([*lower, *upper]).all()
                or np.any(lower >= upper)):
            raise ValueError("known-space bounds must be finite ordered vec3 values")
        for hole_lower, hole_upper in self.holes:
            hlo, hhi = np.asarray(hole_lower, float), np.asarray(hole_upper, float)
            if (hlo.shape != (3,) or hhi.shape != (3,)
                    or not np.isfinite([*hlo, *hhi]).all() or np.any(hlo >= hhi)):
                raise ValueError("unknown-hole bounds must be finite ordered vec3 values")

    def contains_aabb(self, lower, upper) -> bool:
        lo, hi = np.asarray(lower, float), np.asarray(upper, float)
        domain_lo, domain_hi = np.asarray(self.lower), np.asarray(self.upper)
        if (lo.shape != (3,) or hi.shape != (3,) or not np.isfinite([*lo, *hi]).all()
                or np.any(lo > hi) or np.any(lo < domain_lo) or np.any(hi > domain_hi)):
            return False
        for hole_lower, hole_upper in self.holes:
            if np.all(hi >= hole_lower) and np.all(lo <= hole_upper):
                return False
        return True


@dataclass(frozen=True)
class EvidenceBoundedPlaneSupport:
    """Planar contact manifold limited by independently declared floor evidence.

    ``evidence`` describes where the entire swept footprint is known to have
    support.  ``height_error_m`` records the maximum checked mismatch between the
    plane and the floor evidence; it is not silently added to the robot height.
    Long edges are accepted only if plane travel is within ``max_travel_m`` and
    the plane tilt is within ``max_slope_deg``.
    """

    normal: tuple[float, float, float]
    point: tuple[float, float, float]
    evidence: BoxKnownSpace
    height_error_m: float = 0.
    max_height_error_m: float = .05
    max_travel_m: float = .05
    max_slope_deg: float = 5.

    def __post_init__(self):
        normal, point = np.asarray(self.normal, float), np.asarray(self.point, float)
        values = [*normal, *point, self.height_error_m, self.max_height_error_m,
                  self.max_travel_m, self.max_slope_deg]
        if (normal.shape != (3,) or point.shape != (3,)
                or not np.isfinite(values).all() or np.linalg.norm(normal) == 0
                or normal[2] <= 0 or self.height_error_m < 0
                or self.max_height_error_m < 0 or self.max_travel_m < 0
                or not 0 <= self.max_slope_deg < 90):
            raise ValueError("invalid ground support parameters")
        unit = normal / np.linalg.norm(normal)
        tilt = math.degrees(math.acos(float(np.clip(unit[2], -1., 1.))))
        if tilt > self.max_slope_deg or self.height_error_m > self.max_height_error_m:
            raise ValueError("floor evidence exceeds contact slope/height limits")
        object.__setattr__(self, "normal", tuple(map(float, unit)))
        object.__setattr__(self, "point", tuple(map(float, point)))

    @property
    def slope_deg(self) -> float:
        return math.degrees(math.acos(float(self.normal[2])))

    def height(self, x: float, y: float) -> float | None:
        if not np.isfinite([x, y]).all():
            return None
        lo = (float(x), float(y), self.evidence.lower[2])
        hi = (float(x), float(y), self.evidence.upper[2])
        if not self.evidence.contains_aabb(lo, hi):
            return None
        n, p = np.asarray(self.normal), np.asarray(self.point)
        return float(p[2] - (n[0] * (x - p[0]) + n[1] * (y - p[1])) / n[2])

    def height_bounds(self, lower_xy, upper_xy):
        lo, hi = np.asarray(lower_xy, float), np.asarray(upper_xy, float)
        if (lo.shape != (2,) or hi.shape != (2,) or not np.isfinite([*lo, *hi]).all()
                or np.any(lo > hi)):
            return None
        evidence_lo = (float(lo[0]), float(lo[1]), self.evidence.lower[2])
        evidence_hi = (float(hi[0]), float(hi[1]), self.evidence.upper[2])
        if not self.evidence.contains_aabb(evidence_lo, evidence_hi):
            return None
        zs = [self.height(x, y) for x in (lo[0], hi[0]) for y in (lo[1], hi[1])]
        if any(z is None for z in zs):
            return None
        return float(min(zs)), float(max(zs))

    def supports_segment(self, a, b, radius_m: float) -> bool:
        pa, pb = np.asarray(a, float), np.asarray(b, float)
        if (pa.shape != (3,) or pb.shape != (3,) or not np.isfinite([*pa, *pb, radius_m]).all()
                or radius_m <= 0):
            return False
        lo = np.minimum(pa[:2], pb[:2]) - radius_m
        hi = np.maximum(pa[:2], pb[:2]) + radius_m
        bounds = self.height_bounds(lo, hi)
        if bounds is None or bounds[1] - bounds[0] > self.max_travel_m + 1e-12:
            return False
        za, zb = self.height(pa[0], pa[1]), self.height(pb[0], pb[1])
        return (za is not None and zb is not None
                and abs(pa[2] - pb[2] - (za - zb)) <= 1e-7)


def supported_pose(x: float, y: float, yaw: float, body: BodySpec,
                   support: EvidenceBoundedPlaneSupport) -> Pose3:
    """Construct a body-centre pose on the declared support manifold."""
    if body.motion != "ground_unicycle":
        raise ValueError("supported_pose is only defined for ground robots")
    z = support.height(float(x), float(y))
    if z is None or not np.isfinite(yaw):
        raise ValueError("pose lies outside known support or has invalid yaw")
    return Pose3((float(x), float(y), z + body.ground_clearance_m + body.half_height_m),
                 float(yaw))


def crop_by_support_aabb(scene: GaussianScene3D, lower, upper, *, level: float,
                         tau: float = 0.) -> tuple[GaussianScene3D, dict]:
    """Crop only supports whose full 3D ellipsoid AABBs overlap a closed box.

    This is a conservative acceleration adapter, not a 2D shadow or centre crop.
    The returned provenance records counts so a consumer can audit scaling.
    """
    lo, hi = np.asarray(lower, float), np.asarray(upper, float)
    if (lo.shape != (3,) or hi.shape != (3,) or not np.isfinite([*lo, *hi, level, tau]).all()
            or np.any(lo >= hi) or level <= 0 or not 0 <= tau <= 1):
        raise ValueError("invalid support crop")
    support_lo, support_hi = scene.aabb(level)
    opacity = scene.opacity > tau
    overlap = opacity & np.all(support_hi >= lo, axis=1) & np.all(support_lo <= hi, axis=1)
    selected = scene.subset(overlap)
    return selected, {"crop_rule": "full_3d_ellipsoid_aabb_overlap",
                      "input_supports": int(len(scene)),
                      "opacity_selected": int(np.count_nonzero(opacity)),
                      "selected_supports": int(len(selected)),
                      "bounds_min": lo.tolist(), "bounds_max": hi.tolist(),
                      "level": float(level), "tau": float(tau)}
