"""Full-covariance ellipsoid versus upright cylinder/segment geometry.

Only independently evaluated separating support planes authorize free space.
GJK supplies directions, not an assumed exact distance. The translated cylinder
swept along a segment is convex and its support is the maximum endpoint support.
All returned bounds include floating-point slack; these are numerical continuous
bounds for the declared geometric model, not exact-arithmetic certificates.
"""
from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from typing import Callable

import numpy as np


ABS_SLACK_M = 1e-8


def numerical_slack(*values) -> float:
    scale = max((float(np.max(np.abs(v))) for v in values if np.size(v)), default=1.)
    return ABS_SLACK_M + 128 * np.finfo(float).eps * max(1., scale)


def vec3(value, name="vector") -> np.ndarray:
    value = np.asarray(value, dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError(f"{name} must contain three finite coordinates")
    return value


def box_distance(lower, upper, other_lower, other_upper):
    """Euclidean distance between AABBs (vectorized on the leading dimension)."""
    gaps = np.maximum(np.maximum(np.asarray(lower) - other_upper,
                                  other_lower - np.asarray(upper)), 0.)
    return np.linalg.norm(gaps, axis=-1)


@dataclass(frozen=True)
class PairBound:
    clearance_lower_m: float
    overlap: bool
    iterations: int
    reason: str


def _cylinder_support(n, radius, half_height):
    xy = np.linalg.norm(n[:2])
    return np.array([radius * n[0] / xy if xy else 0.,
                     radius * n[1] / xy if xy else 0.,
                     half_height * np.sign(n[2])])


def _centre_in_sweep(a, b, radius, half_height, slack):
    """Strict witness that the ellipsoid centre (origin) lies in swept body."""
    v = b - a
    lo, hi = 0., 1.
    if abs(v[2]) > slack:
        ends = sorted(((-half_height + slack - a[2]) / v[2],
                       (half_height - slack - a[2]) / v[2]))
        lo, hi = max(lo, ends[0]), min(hi, ends[1])
    elif abs(a[2]) >= half_height - slack:
        return False
    if lo > hi:
        return False
    vv = float(v[:2] @ v[:2])
    t = np.clip(-float(a[:2] @ v[:2]) / vv, lo, hi) if vv else lo
    return np.linalg.norm((a + t * v)[:2]) < radius - slack


def _tetra_contains_origin(vertices, slack):
    if len(vertices) != 4:
        return False
    # A positive distance from every oriented face is a strict interior witness.
    for i in range(4):
        face = np.delete(vertices, i, axis=0)
        n = np.cross(face[1] - face[0], face[2] - face[0])
        norm = np.linalg.norm(n)
        if norm <= slack * slack:
            return False
        n /= norm
        opposite = float((vertices[i] - face[0]) @ n)
        origin = float(-face[0] @ n)
        if abs(opposite) <= slack or origin * np.sign(opposite) <= slack:
            return False
    return True


def _closest_hull(vertices):
    """Closest point in a tiny convex hull by exhaustive active faces (<=5)."""
    best, best_norm, active = vertices[0], float(vertices[0] @ vertices[0]), vertices[:1]
    for size in range(1, min(4, len(vertices)) + 1):
        for indices in combinations(range(len(vertices)), size):
            face = vertices[list(indices)]
            if size == 1:
                weights = np.ones(1)
            else:
                basis = (face[1:] - face[0]).T
                coeff, _, _, _ = np.linalg.lstsq(basis, -face[0], rcond=1e-13)
                weights = np.r_[1. - coeff.sum(), coeff]
                if np.min(weights) < -1e-12:
                    continue
                weights = np.maximum(weights, 0.)
                weights /= weights.sum()
            point = weights @ face
            norm = float(point @ point)
            if norm <= best_norm:
                best, best_norm = point, norm
                active = face[weights > 1e-12]
    return best, active


def cylinder_ellipsoid_bound(a, b, radius, half_height, mean, covariance, level,
                             *, margin_m=0., max_iterations=40,
                             check: Callable[[], None] | None = None) -> PairBound:
    """Conservative distance bound for the entire linear swept cylinder.

    Inputs are validated by the prepared oracle. An overlap witness is either
    the ellipsoid centre strictly inside the sweep or a Minkowski tetrahedron
    strictly enclosing zero. Tangency/near-margin/nonconvergence stay unresolved.
    A lower bound can be negative even for geometrically separated shapes.
    """
    slack = numerical_slack(a, b, mean, level * np.sqrt(np.diag(covariance)))
    a, b = np.asarray(a) - mean, np.asarray(b) - mean
    if _centre_in_sweep(a, b, radius, half_height, slack):
        return PairBound(0., True, 0, "ellipsoid_centre_in_swept_body")

    def support_min(n):
        cn = covariance @ n
        den = np.sqrt(max(float(n @ cn), 0.))
        ellipsoid = level * cn / den if den else np.zeros(3)
        centre = a if a @ n <= b @ n else b
        return centre + _cylinder_support(-n, radius, half_height) - ellipsoid

    def gap(n):
        # Evaluate the analytic supports independently of the simplex solver.
        return (min(float(a @ n), float(b @ n))
                - radius * np.linalg.norm(n[:2]) - half_height * abs(n[2])
                - level * np.sqrt(max(float(n @ covariance @ n), 0.)) - slack)

    delta = b - a
    t = np.clip(-float(a @ delta) / float(delta @ delta), 0., 1.) if delta @ delta else 0.
    direction = a + t * delta
    norm = np.linalg.norm(direction)
    direction = direction / norm if norm > slack else np.array([1., 0., 0.])
    directions = [direction, *np.eye(3), *(-np.eye(3))]
    best = -float("inf")
    for n in directions:
        best = max(best, gap(n))
        if best > margin_m:
            return PairBound(best, False, 0, "separating_support_plane")
    simplex = np.array([support_min(direction)])
    point = simplex[0]
    for iteration in range(1, max_iterations + 1):
        if check:
            check()
        norm = np.linalg.norm(point)
        if norm <= slack:
            break
        n = point / norm
        best = max(best, gap(n))
        if best > margin_m:
            return PairBound(best, False, iteration, "separating_support_plane")
        vertex = support_min(n)
        if np.min(np.linalg.norm(simplex - vertex, axis=1)) <= slack:
            break
        vertices = np.vstack((simplex, vertex))
        for ids in combinations(range(len(vertices)), 4):
            if _tetra_contains_origin(vertices[list(ids)], slack):
                return PairBound(0., True, iteration, "minkowski_interior_witness")
        point, simplex = _closest_hull(vertices)
    return PairBound(best, False, iteration, "unresolved_contact_or_margin")
