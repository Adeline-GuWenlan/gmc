"""Pair C-obstacles of the axisymmetric UAV (uav.md §5), planning frame and C-space domain.

For scene Gaussian i (level-kappa ellipsoid E_i) and the upright cylinder body
Cyl(r, h) with clearance margin m, the set of body centres at distance <= m is

    O_i = E_i (+) Cyl(r, h) (+) B(m),
    h_i(u) = u.mu_i + kappa sqrt(u^T Sigma_i u) + r |u_xy| + h |u_z| + m |u|.

This is exactly the forbidden set of ``gmc.gs3d.oracle`` (clearance > margin is
free).  ``O_i`` is centrally symmetric about mu_i, so h_i(u) = u.mu_i + rho_i(u)
with an even rho_i.  Evaluated rho values are rounded outward, support points are
plain binary64 (they are only used for inner sets, which are shrunk by slack).

The planning frame is the query box's own frame: a rigid map ``x_p = R (x_w - o)``
whose R fixes the z axis (rotation or reflection about z), under which the
cylinder is invariant.  Nothing is projected; covariances rotate as R S R^T.
"""
from __future__ import annotations

from dataclasses import dataclass
from itertools import product

import numpy as np
from scipy.optimize import linprog

from gmc.gs3d.contracts import BodySpec, SceneSpec
from gmc.gs3d.geometry import numerical_slack
from gmc.gs3d.oracle import PreparedScene

EPS = np.finfo(float).eps


@dataclass(frozen=True)
class PlanningFrame:
    R: np.ndarray       # world -> plan rows; must fix the z axis
    origin: np.ndarray  # world point mapped to the plan origin

    def __post_init__(self):
        R = np.asarray(self.R, float)
        o = np.asarray(self.origin, float)
        if (R.shape != (3, 3) or o.shape != (3,) or not np.isfinite(R).all()
                or not np.isfinite(o).all() or not np.allclose(R @ R.T, np.eye(3), atol=1e-10)):
            raise ValueError("planning frame must be a finite orthonormal map")
        if not (np.allclose(R[2], (0, 0, 1), atol=1e-12) and np.allclose(R[:, 2], (0, 0, 1), atol=1e-12)):
            raise ValueError("planning frame must fix z: the upright cylinder is only invariant about z")
        object.__setattr__(self, "R", R)
        object.__setattr__(self, "origin", o)

    @classmethod
    def identity(cls) -> "PlanningFrame":
        return cls(np.eye(3), np.zeros(3))

    def to_plan(self, world):
        return (np.asarray(world, float) - self.origin) @ self.R.T

    def to_world(self, plan):
        return np.asarray(plan, float) @ self.R + self.origin

    def json(self) -> dict:
        return {"world_to_plan": self.R.tolist(), "origin_world_m": self.origin.tolist()}


def _sym6(covs):
    return np.stack([covs[:, 0, 0], covs[:, 1, 1], covs[:, 2, 2],
                     2 * covs[:, 0, 1], 2 * covs[:, 0, 2], 2 * covs[:, 1, 2]], axis=1)


def _dir6(U):
    return np.stack([U[:, 0] ** 2, U[:, 1] ** 2, U[:, 2] ** 2,
                     U[:, 0] * U[:, 1], U[:, 0] * U[:, 2], U[:, 1] * U[:, 2]], axis=1)


class PairSet:
    """Candidate (scene Gaussian, body cylinder) pairs, all in the planning frame."""

    def __init__(self, means, covs, ids, *, level, radius, half_height, margin):
        means = np.array(means, dtype=float, copy=True).reshape(-1, 3)
        covs = np.array(covs, dtype=float, copy=True).reshape(-1, 3, 3)
        ids = np.array(ids, dtype=np.int64, copy=True).reshape(-1)
        n = len(means)
        if covs.shape != (n, 3, 3) or ids.shape != (n,):
            raise ValueError("means (N,3), covs (N,3,3), ids (N,) required")
        if not (np.isfinite(means).all() and np.isfinite(covs).all()):
            raise ValueError("nonfinite pair geometry")
        values = (level, radius, half_height, margin)
        if not np.isfinite(values).all() or level <= 0 or radius <= 0 or half_height <= 0 or margin < 0:
            raise ValueError("invalid level/body/margin")
        self.means, self.covs, self.ids = means, covs, ids
        self.level, self.radius, self.half_height, self.margin = map(float, values)
        self._s6 = _sym6(covs)
        self._abs6 = _sym6(np.abs(covs))
        semi = self.level * np.sqrt(np.maximum(np.diagonal(covs, axis1=1, axis2=2), 0.))
        self.slack = numerical_slack(means, semi, self.radius, self.half_height)
        body = np.array([self.radius, self.radius, self.half_height]) + self.margin
        # Support values on +-axes, rounded outward: the exact AABB of O_i.
        half = (self.level * np.sqrt(np.diagonal(covs, axis1=1, axis2=2)
                                     + 32 * EPS * np.abs(np.diagonal(covs, axis1=1, axis2=2)))
                * (1 + 8 * EPS) + body)
        self.aabb_lower = means - half - self.slack
        self.aabb_upper = means + half + self.slack
        for a in (self.means, self.covs, self.ids, self.aabb_lower, self.aabb_upper):
            a.flags.writeable = False

    def __len__(self):
        return len(self.means)

    def subset(self, index) -> "PairSet":
        return PairSet(self.means[index], self.covs[index], self.ids[index], level=self.level,
                       radius=self.radius, half_height=self.half_height, margin=self.margin)

    def rho(self, U, index=None) -> np.ndarray:
        """Outward-rounded even part rho_i(u) of the support function, shape (n, K)."""
        U = np.atleast_2d(np.asarray(U, float))
        s6 = self._s6 if index is None else self._s6[index]
        a6 = self._abs6 if index is None else self._abs6[index]
        d6 = _dir6(U)
        q = s6 @ d6.T + 32 * EPS * (a6 @ np.abs(d6).T)
        r = (self.level * np.sqrt(np.maximum(q, 0.)) + self.radius * np.hypot(U[:, 0], U[:, 1])
             + self.half_height * np.abs(U[:, 2]) + self.margin * np.linalg.norm(U, axis=1))
        return r * (1 + 16 * EPS)

    def support(self, U, index=None) -> np.ndarray:
        """h_i(u) for unit u; shape (n,) for one direction, (n, K) for K directions."""
        U = np.asarray(U, float)
        single = U.ndim == 1
        U2 = np.atleast_2d(U)
        means = self.means if index is None else self.means[index]
        h = means @ U2.T + self.rho(U2, index)
        return h[:, 0] if single else h

    def support_points(self, U, index=None) -> np.ndarray:
        """Points x_i(u) in O_i with u.x_i(u) = h_i(u); shape (n, K, 3)."""
        U = np.atleast_2d(np.asarray(U, float))
        means = self.means if index is None else self.means[index]
        covs = self.covs if index is None else self.covs[index]
        cu = np.einsum("nij,kj->nki", covs, U)
        den = np.sqrt(np.maximum(np.einsum("nki,ki->nk", cu, U), 0.))
        ell = self.level * np.divide(cu, den[..., None], out=np.zeros_like(cu),
                                     where=den[..., None] > 0)
        xy = np.hypot(U[:, 0], U[:, 1])
        cyl = np.zeros((len(U), 3))
        safe = xy > 0
        cyl[safe, 0] = self.radius * U[safe, 0] / xy[safe]
        cyl[safe, 1] = self.radius * U[safe, 1] / xy[safe]
        cyl[:, 2] = self.half_height * np.sign(U[:, 2])
        ball = self.margin * U / np.linalg.norm(U, axis=1, keepdims=True)
        return means[:, None, :] + ell + (cyl + ball)[None]


# ----------------------------------------------------------------------------- domain

@dataclass(frozen=True)
class Domain:
    """Body-centre set accepted by the baseline's workspace/known-space checks, plan frame.

    Rows ``A x <= b`` are nominal thresholds; ``row_slack`` > 0 means strictly inside.
    """
    A: np.ndarray
    b: np.ndarray
    labels: tuple
    bbox_lower: np.ndarray
    bbox_upper: np.ndarray
    tol: float

    def row_slack(self, q) -> float:
        return float(np.min(self.b - self.A @ np.asarray(q, float)))

    def box_status(self, centre, half) -> str:
        """'inside' (strictly, with tol), 'outside' (one row violated everywhere) or 'partial'."""
        c, d = np.asarray(centre, float), np.asarray(half, float)
        ac, ad = self.A @ c, np.abs(self.A) @ d
        if np.all(ac + ad <= self.b - self.tol):
            return "inside"
        if np.any(ac - ad > self.b + self.tol):
            return "outside"
        return "partial"

    def json(self) -> dict:
        return {"rows": [{"a": a.tolist(), "b": float(b), "label": l}
                         for a, b, l in zip(self.A, self.b, self.labels)],
                "bbox_lower": self.bbox_lower.tolist(), "bbox_upper": self.bbox_upper.tolist(),
                "tol": self.tol}


def _known_box_rows(known, frame: PlanningFrame, a_world):
    """Rows for 'world AABB of the body inside the known region' (closed)."""
    inner = getattr(known, "inner", known)
    if hasattr(inner, "lower_route_m"):
        Rk = np.asarray(inner.world_to_route, float)
        ok = np.asarray(inner.origin_world_m, float)
        if not (np.allclose(Rk, frame.R, atol=1e-12) and np.allclose(ok, frame.origin, atol=1e-12)):
            raise ValueError("planning frame must equal the known-space prism frame")
        lo, hi = np.asarray(inner.lower_route_m, float), np.asarray(inner.upper_route_m, float)
        tol = 1e-12  # RouteBoxKnownSpace.contains_aabb tolerance
    elif hasattr(inner, "lower") and hasattr(inner, "upper"):
        if getattr(inner, "holes", ()):
            raise ValueError("known-space holes are not supported by the aerial3d domain")
        if not np.allclose(frame.R, np.eye(3)) or not np.allclose(frame.origin, 0):
            raise ValueError("axis-aligned known box requires the identity planning frame")
        lo, hi = np.asarray(inner.lower, float), np.asarray(inner.upper, float)
        tol = 0.
    else:
        raise ValueError(f"unsupported known-space type {type(inner).__name__}")
    e = np.abs(frame.R) @ a_world
    rows, rhs, labels = [], [], []
    for k, ax in enumerate("uvz" if hasattr(inner, "lower_route_m") else "xyz"):
        unit = np.eye(3)[k]
        rows += [unit, -unit]
        rhs += [hi[k] - e[k] + tol, -(lo[k] + e[k] - tol)]
        labels += [f"known:{ax}_max", f"known:{ax}_min"]
    return rows, rhs, labels


def domain_from_scene(scene: SceneSpec, body: BodySpec, *, margin_m: float,
                      frame: PlanningFrame | None = None) -> tuple[PlanningFrame, Domain]:
    """Planning frame + the body-centre domain the baseline oracle would accept."""
    if body.motion != "uav_translation":
        raise ValueError("aerial3d plans the axisymmetric UAV only")
    inner = getattr(scene.known_space, "inner", scene.known_space)
    if frame is None:
        frame = (PlanningFrame(np.asarray(inner.world_to_route, float),
                               np.asarray(inner.origin_world_m, float))
                 if hasattr(inner, "lower_route_m") else PlanningFrame.identity())
    a_world = np.array([body.radius_m, body.radius_m, body.half_height_m])
    rows, rhs, labels = _known_box_rows(scene.known_space, frame, a_world)
    w_lo, w_hi = np.asarray(scene.bounds_min, float), np.asarray(scene.bounds_max, float)
    # Baseline: boundary = min(lower - w_lo, w_hi - upper) - slack must exceed margin.
    slack_b = numerical_slack(w_lo, w_hi)
    for j, ax in enumerate("xyz"):
        # x_w = R^T x_p + origin; row on x_p for world axis j.
        a = frame.R[:, j]
        rows += [a, -a]
        rhs += [w_hi[j] - a_world[j] - margin_m - slack_b - frame.origin[j],
                -(w_lo[j] + a_world[j] + margin_m + slack_b - frame.origin[j])]
        labels += [f"world:{ax}_max", f"world:{ax}_min"]
    A, b = np.asarray(rows, float), np.asarray(rhs, float)
    lower, upper = np.empty(3), np.empty(3)
    for k in range(3):
        for sign, out in ((1., lower), (-1., upper)):
            c = np.zeros(3)
            c[k] = sign
            res = linprog(c, A_ub=A, b_ub=b, bounds=[(None, None)] * 3, method="highs")
            if res.status != 0:
                raise ValueError("empty or unbounded C-space domain")
            out[k] = res.x[k]
    tol = numerical_slack(lower, upper, w_lo, w_hi)
    dom = Domain(A, b, tuple(labels), lower, upper, tol)
    for arr in (dom.A, dom.b, dom.bbox_lower, dom.bbox_upper):
        arr.flags.writeable = False
    return frame, dom


def pairs_from_scene(scene: SceneSpec, body: BodySpec, frame: PlanningFrame, domain: Domain, *,
                     margin_m: float, pad_m: float = 1e-3,
                     prepared: PreparedScene | None = None) -> tuple[PairSet, dict]:
    """Candidate pairs: baseline's validated opacity>tau supports whose O_i can reach the domain.

    Uses ``gs3d.oracle.PreparedScene`` for covariance validation/flooring and the tau
    filter, so the Gaussian set is exactly the baseline oracle's.  Prunes (conservative):
    exact AABB of O_i vs the domain bbox, and domain rows that O_i lies wholly beyond.
    """
    prep = prepared if prepared is not None else PreparedScene(scene)
    if prep.scene is not scene:
        raise ValueError("prepared scene identity mismatch")
    means = frame.to_plan(prep.means)
    covs = np.einsum("ij,njk,lk->nil", frame.R, prep.covs, frame.R)
    covs = .5 * (covs + covs.transpose(0, 2, 1))
    everything = PairSet(means, covs, prep.ids, level=scene.level, radius=body.radius_m,
                         half_height=body.half_height_m, margin=margin_m)
    keep = (np.all(everything.aabb_upper >= domain.bbox_lower - pad_m, axis=1)
            & np.all(everything.aabb_lower <= domain.bbox_upper + pad_m, axis=1))
    after_aabb = int(keep.sum())
    idx = np.flatnonzero(keep)
    if len(idx):
        # min over O_i of a.x = a.mu - rho(a) (rho even); beyond row b + pad -> cannot reach D.
        norms = np.linalg.norm(domain.A, axis=1)
        units = domain.A / norms[:, None]
        low = everything.means[idx] @ units.T - everything.rho(units, idx)
        beyond = np.any(low > (domain.b / norms)[None, :] + pad_m, axis=1)
        idx = idx[~beyond]
    pairs = everything.subset(idx)
    stats = {"input_supports": int(prep.stats["input_supports"]),
             "opacity_selected": int(len(prep.ids)),
             "after_domain_aabb": after_aabb, "candidate_pairs": int(len(pairs)),
             "pruned_pairs": int(len(prep.ids) - len(pairs)),
             "prune_rules": ["opacity>tau (baseline semantics)",
                             "exact AABB(O_i) vs domain bbox (+pad)",
                             "domain row a.x<=b with min_{O_i} a.x > b+pad"],
             "pad_m": pad_m, "covariance_preparation": dict(prep.stats)}
    return pairs, stats
