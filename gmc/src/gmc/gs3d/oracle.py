"""Shared full-3D body and continuous-edge authority for all robot types."""
from __future__ import annotations

from dataclasses import dataclass, field
from time import perf_counter

import numpy as np

from .contracts import BodySpec, OracleReport, Pose3, SceneSpec, SearchBudget
from .geometry import box_distance, cylinder_ellipsoid_bound, numerical_slack, vec3
from .spatial import SupportIndex


class BudgetExceeded(RuntimeError):
    pass


@dataclass
class QueryBudget:
    limits: SearchBudget
    entered_at: float = field(default_factory=perf_counter)
    oracle_calls: int = 0
    narrowphase_pairs: int = 0

    def __post_init__(self):
        if not np.isfinite(self.limits.max_wall_s) or self.limits.max_wall_s <= 0:
            raise ValueError("max_wall_s must be finite and positive")
        for key in ("max_expansions", "max_oracle_calls", "max_narrowphase_pairs"):
            v = getattr(self.limits, key)
            if isinstance(v, bool) or not isinstance(v, (int, np.integer)) or v < 1:
                raise ValueError(f"{key} must be a positive integer")

    def check(self):
        if perf_counter() - self.entered_at >= self.limits.max_wall_s:
            raise BudgetExceeded("max_wall_s")

    def call(self):
        self.check()
        if self.oracle_calls >= self.limits.max_oracle_calls:
            raise BudgetExceeded("max_oracle_calls")
        self.oracle_calls += 1

    def pair(self):
        self.check()
        if self.narrowphase_pairs >= self.limits.max_narrowphase_pairs:
            raise BudgetExceeded("max_narrowphase_pairs")
        self.narrowphase_pairs += 1


def validate_body(body: BodySpec):
    if (not np.isfinite([body.radius_m, body.half_height_m, body.ground_clearance_m]).all()
            or body.radius_m <= 0 or body.half_height_m <= 0 or body.ground_clearance_m < 0):
        raise ValueError("body radius/half-height must be positive; ground clearance nonnegative")
    if body.motion not in ("uav_translation", "ground_unicycle"):
        raise ValueError("unsupported body motion model")


def validate_pose(q):
    p = vec3(q.xyz, "pose.xyz")
    if not np.isfinite(q.yaw):
        raise ValueError("pose yaw must be finite")
    return p


class PreparedScene:
    """Validate covariances once, snapshot selected arrays, build conservative BVH.

    No opacity or covariance repair silently deletes obstacles. Nonnegative tiny
    eigenvalues are outward-floored by adding a scalar identity, and counted.
    The coverage/support providers must be immutable for this prepared lifetime.
    """

    def __init__(self, scene: SceneSpec, *, check=None):
        started = perf_counter()
        self.scene = scene
        self.lower = vec3(scene.bounds_min, "bounds_min").copy()
        self.upper = vec3(scene.bounds_max, "bounds_max").copy()
        if np.any(self.upper <= self.lower):
            raise ValueError("bounds must be strictly ordered on all three axes")
        if not np.isfinite([scene.tau, scene.level]).all() or not 0 <= scene.tau <= 1 or scene.level <= 0:
            raise ValueError("tau must be in [0,1] and level finite and positive")
        if not scene.scene_id or not callable(getattr(scene.known_space, "contains_aabb", None)):
            raise ValueError("scene identity and explicit known-space authority are required")
        raw = scene.gaussians
        means, covs = np.asarray(raw.means), np.asarray(raw.covs)
        n = len(means)
        if means.shape != (n, 3) or covs.shape != (n, 3, 3):
            raise ValueError("invalid Gaussian shapes")
        if raw.ids.shape != (n,) or raw.opacity.shape != (n,) or len(np.unique(raw.ids)) != n:
            raise ValueError("invalid or duplicate Gaussian IDs")
        if not np.isfinite(raw.opacity).all() or np.any((raw.opacity < 0) | (raw.opacity > 1)):
            raise ValueError("opacity must be finite and in [0,1]")
        selected_means, selected_covs, selected_ids = [], [], []
        floored, symmetrized, maximum_floor_added = 0, 0, 0.
        for start in range(0, n, 16384):
            if check:
                check()
            c = np.array(covs[start:start + 16384], dtype=float, copy=True)
            m = means[start:start + 16384]
            if not np.isfinite(c).all() or not np.isfinite(m).all():
                raise ValueError("nonfinite Gaussian geometry")
            asym = np.max(np.abs(c - c.transpose(0, 2, 1)), axis=(1, 2))
            scale = np.maximum(np.max(np.abs(c), axis=(1, 2)), 1e-12)
            if np.any(asym > 1e-10 * scale):
                raise ValueError("asymmetric covariance")
            symmetrized += int(np.count_nonzero(asym))
            c = .5 * (c + c.transpose(0, 2, 1))
            eigen = np.linalg.eigvalsh(c)
            if np.any(eigen[:, 0] < 0):
                raise ValueError("negative covariance eigenvalue")
            adjustment = np.maximum(1e-12 - eigen[:, 0], 0.)
            # Make the outward addition representable even if a huge principal
            # variance would swallow a 1e-12 diagonal increment in float64.
            adjustment = np.where(adjustment > 0,
                np.maximum(adjustment, 32 * np.finfo(float).eps * scale), 0.)
            floored += int(np.count_nonzero(adjustment))
            maximum_floor_added = max(maximum_floor_added, float(adjustment.max(initial=0.)))
            c += adjustment[:, None, None] * np.eye(3)
            keep = raw.opacity[start:start + 16384] > scene.tau
            selected_means.append(np.array(m[keep], dtype=float, copy=True))
            selected_covs.append(c[keep])
            selected_ids.append(np.array(raw.ids[start:start + 16384][keep], copy=True))
        self.means = np.concatenate(selected_means) if n else np.empty((0, 3))
        self.covs = np.concatenate(selected_covs) if n else np.empty((0, 3, 3))
        self.ids = np.concatenate(selected_ids) if n else np.empty(0, dtype=np.int64)
        self.index = SupportIndex(self.means, self.covs, scene.level, check=check)
        for arr in (self.means, self.covs, self.ids, self.lower, self.upper):
            arr.flags.writeable = False
        self.stats = {"input_supports": n, "active_supports": len(self.ids),
                      "outward_floored_covariances": floored,
                      "covariance_floor_max_added_m2": maximum_floor_added,
                      "roundoff_symmetrized_covariances": symmetrized,
                      "bvh_nodes": len(self.index.nodes),
                      "index_build_wall_s": perf_counter() - started}


class GaussianBodyOracle:
    def __init__(self, scene: SceneSpec | PreparedScene, *, budget: QueryBudget | None = None):
        self.prepared = scene if isinstance(scene, PreparedScene) else PreparedScene(scene)
        self.budget = budget
        self.stats = {"oracle_calls": 0, "narrowphase_pairs": 0, "candidate_count": 0,
                      "bvh_nodes_visited": 0, "gjk_iterations": 0}

    def pose(self, q: Pose3, body: BodySpec, *, margin_m: float) -> OracleReport:
        return self.edge(q, q, body, margin_m=margin_m)

    def edge(self, a: Pose3, b: Pose3, body: BodySpec, *, margin_m: float) -> OracleReport:
        if self.budget:
            self.budget.call()
        self.stats["oracle_calls"] += 1
        try:
            pa, pb = validate_pose(a), validate_pose(b)
            validate_body(body)
            if not np.isfinite(margin_m) or margin_m < 0:
                raise ValueError("margin must be finite and nonnegative")
        except (ValueError, TypeError, AttributeError) as exc:
            return OracleReport("unknown", "invalid", None, str(exc))
        p = self.prepared
        half = np.array([body.radius_m, body.radius_m, body.half_height_m])
        lower, upper = np.minimum(pa, pb) - half, np.maximum(pa, pb) + half
        slack = numerical_slack(lower, upper, p.lower, p.upper)
        boundary = float(min(np.min(lower - p.lower), np.min(p.upper - upper))) - slack
        if boundary < -2 * slack:
            return OracleReport("occupied", "continuous_bound", 0., "body_exceeds_workspace_bounds")
        # A known space that can test the swept vertical cylinder itself is asked that; the world
        # AABB is only a sufficient test, too strict in a rotated route frame (F5 REPLAY-AABB).
        known = p.scene.known_space
        swept = getattr(known, "contains_swept_cylinder", None)
        covered = (swept(pa, pb, body.radius_m, body.half_height_m) if swept is not None
                   else known.contains_aabb(tuple(lower), tuple(upper)))
        if not covered:
            return OracleReport("unknown", "unresolved", None, "map_unknown")
        if boundary <= margin_m:
            return OracleReport("unknown", "unresolved", max(0., boundary), "workspace_margin_unproven")
        if body.motion == "ground_unicycle":
            support = p.scene.support
            if support is None:
                return OracleReport("unknown", "unresolved", None, "ground_support_missing")
            for q in (pa, pb):
                z = support.height(float(q[0]), float(q[1]))
                if z is None or not np.isfinite(z):
                    return OracleReport("unknown", "unresolved", None, "ground_support_missing")
                if abs(q[2] - (z + body.ground_clearance_m + body.half_height_m)) > 1e-7:
                    return OracleReport("unknown", "invalid", None, "ground_pose_off_support_manifold")
            height_range = support.height_bounds(tuple(lower[:2]), tuple(upper[:2]))
            if (height_range is None or len(height_range) != 2
                    or not np.isfinite(height_range).all() or height_range[0] > height_range[1]
                    or not support.supports_segment(tuple(pa), tuple(pb), body.radius_m)):
                return OracleReport("unknown", "unresolved", None, "ground_swept_support_unproven")
            # The provider owns whole-footprint contact travel/slope and manifold
            # conformity; every Gaussian still collides with the chassis below.

        # Omitted boxes are farther than padding in at least one coordinate.
        # Thus the returned global bound is capped by padding, never infinity.
        padding = max(.10, margin_m + .05) + 4 * slack
        bound = min(boundary, padding - slack)
        candidates, pairs = 0, 0
        unknown_ids, unknown_bound = [], bound
        check = self.budget.check if self.budget else None
        for ids in p.index.query(lower - padding, upper + padding, check=check, stats=self.stats):
            candidates += len(ids)
            self.stats["candidate_count"] += len(ids)
            distances = box_distance(lower, upper, p.index.lower[ids], p.index.upper[ids]) - slack
            for idx, distance in zip(ids, distances):
                if distance > margin_m:
                    bound = min(bound, float(distance))
                    continue
                if self.budget:
                    self.budget.pair()
                pairs += 1
                self.stats["narrowphase_pairs"] += 1
                result = cylinder_ellipsoid_bound(pa, pb, body.radius_m, body.half_height_m,
                    p.means[idx], p.covs[idx], p.scene.level, margin_m=margin_m, check=check)
                self.stats["gjk_iterations"] += result.iterations
                if result.overlap:
                    return OracleReport("occupied", "continuous_bound", 0., result.reason,
                                        (int(p.ids[idx]),), candidates, pairs)
                bound = min(bound, result.clearance_lower_m)
                if result.clearance_lower_m <= margin_m:
                    unknown_ids.append(int(p.ids[idx]))
                    unknown_bound = min(unknown_bound, result.clearance_lower_m)
        if unknown_ids:
            return OracleReport("unknown", "unresolved", max(0., min(bound, unknown_bound)),
                                "geometry_or_margin_unproven", tuple(unknown_ids[:32]), candidates, pairs)
        return OracleReport("free", "continuous_bound", float(bound),
                            "swept_support_separation_bound_capped", (), candidates, pairs)
