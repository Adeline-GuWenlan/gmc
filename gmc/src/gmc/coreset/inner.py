"""Inner macro primitives: ``E_minus subset union(members)``.

The outer coreset grows obstacles, which keeps ``SAFE`` sound.  The *possible*
side needs the opposite nesting -- obstacles that are contained in the truth --
so that free space only grows and a global cut certifying ``UNREACHABLE``
remains valid.  Together they give the design revision's sandwich

    C_minus  subset  C_star  subset  C_plus

so a coarsened scene can only turn a decidable query into ``UNKNOWN``.

Certification here is polygonal rather than analytic, and deliberately
conservative on both sides:

  * members are polygonised with ``outer=False``  -> ``P_members subset union``
  * the candidate is polygonised with ``outer=True`` -> ``E_minus subset P_cand``

so ``P_cand subset P_members`` implies ``E_minus subset union(members)``.  Both
inclusions are strict set inclusions, so the only soundness gap is GEOS'
predicate arithmetic, not the discretisation.  That is weaker than the outer
side's direction-plus-Lipschitz certificate, and the status reflects it.
"""
from dataclasses import dataclass

import numpy as np
import shapely
from shapely.geometry import MultiPolygon, Polygon
from shapely.ops import unary_union

from ..types import CertStatus, GaussianSupport2D, SceneModel2D
from .macro import ellipse_polygon

MEMBER_VERTICES = 128
CANDIDATE_VERTICES = 192
SCALE_ITERS = 40


@dataclass(frozen=True)
class InnerMacro:
    support: GaussianSupport2D
    contained: bool
    area_fraction: float      # area(E_minus) / area(union members)
    n_members: int
    status: CertStatus


def _largest_polygon(geom):
    if isinstance(geom, Polygon):
        return geom
    if isinstance(geom, MultiPolygon) and len(geom.geoms):
        return max(geom.geoms, key=lambda g: g.area)
    return None


def _principal_axes(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Orthonormal axes and their relative extents for a point cloud."""
    centred = points - points.mean(axis=0)
    if len(points) < 3:
        return np.eye(2), np.ones(2)
    cov = centred.T @ centred / max(len(points) - 1, 1)
    eigval, eigvec = np.linalg.eigh(0.5 * (cov + cov.T))
    eigval = np.maximum(eigval, 0.0)
    if eigval[1] <= 0.0:
        return np.eye(2), np.ones(2)
    ratio = np.sqrt(eigval / eigval[1])
    return eigvec, np.maximum(ratio, 1e-3)


def inner_ellipse(members, primitive_id: int = 0) -> InnerMacro:
    """Largest ellipse (up to a 1D scale search) inscribed in the member union."""
    members = list(members)
    if not members:
        raise ValueError("inner_ellipse needs at least one member")
    levels = {m.level for m in members}
    level = levels.pop() if len(levels) == 1 else 1.0
    if len(members) == 1:
        only = members[0]
        return InnerMacro(
            GaussianSupport2D(only.mean, only.covariance, only.level,
                              primitive_id),
            True, 1.0, 1, CertStatus.CERTIFIED)

    member_polys = [ellipse_polygon(m, MEMBER_VERTICES, outer=False)
                    for m in members]
    union = unary_union(member_polys)
    target = _largest_polygon(union)
    if target is None or target.area <= 0.0:
        raise ValueError("member union has no positive-area component")

    line = shapely.maximum_inscribed_circle(target, tolerance=1e-5)
    center = np.asarray(line.coords[0], dtype=np.float64)
    r_in = float(line.length)
    if not np.isfinite(r_in) or r_in <= 0.0:
        raise ValueError("member union admits no inscribed disc")

    boundary = np.asarray(target.exterior.coords[:-1], dtype=np.float64)
    axes, ratio = _principal_axes(boundary)

    def candidate(scale: float) -> GaussianSupport2D:
        semi = np.maximum(scale * ratio, 1e-12)
        S = axes @ np.diag(semi ** 2) @ axes.T
        return GaussianSupport2D(mean=center, covariance=S / (level ** 2),
                                 level=level, primitive_id=primitive_id)

    def fits(support) -> bool:
        return ellipse_polygon(support, CANDIDATE_VERTICES,
                               outer=True).within(target)

    # Anisotropic search along the principal axes.
    lo, hi = 0.0, 2.0 * float(np.hypot(*(boundary.max(axis=0)
                                         - boundary.min(axis=0))))
    for _ in range(SCALE_ITERS):
        mid = 0.5 * (lo + hi)
        if fits(candidate(mid)):
            lo = mid
        else:
            hi = mid
    best = candidate(lo) if lo > 0.0 else None

    # The inscribed disc is the guaranteed fallback, but its radius must be
    # bisected rather than assumed: containment is tested against the
    # ``outer=True`` polygon, which inflates by 1/cos(pi/n), so a disc at
    # exactly the maximum inscribed radius provably fails its own test.
    def disc_at(radius: float) -> GaussianSupport2D:
        return GaussianSupport2D(
            mean=center, covariance=np.eye(2) * (max(radius, 1e-12) / level) ** 2,
            level=level, primitive_id=primitive_id)

    d_lo, d_hi = 0.0, r_in
    for _ in range(SCALE_ITERS):
        d_mid = 0.5 * (d_lo + d_hi)
        if fits(disc_at(d_mid)):
            d_lo = d_mid
        else:
            d_hi = d_mid
    if d_lo <= 0.0:
        raise ValueError("member union admits no inscribed ellipse")
    disc = disc_at(d_lo)

    def area_of(s):
        eig = np.linalg.eigvalsh(s.covariance) * (s.level ** 2)
        semi = np.sqrt(np.maximum(eig, 0.0))
        return float(np.pi * semi[0] * semi[1])

    chosen = disc
    if best is not None and fits(best) and area_of(best) > area_of(disc):
        chosen = best
    return InnerMacro(chosen, True, area_of(chosen) / union.area, len(members),
                      CertStatus.EMPIRICALLY_VALIDATED)


def inner_scene(scene: SceneModel2D, groups) -> SceneModel2D:
    """Build the possible-side scene from an outer coreset's partition.

    ``groups`` is a sequence of index tuples -- use ``CoresetResult.cut_members``
    so both sides share one partition and the sandwich is group-aligned.
    """
    supports = []
    for k, members in enumerate(groups):
        macro = inner_ellipse([scene.supports[i] for i in members],
                              primitive_id=k)
        supports.append(macro.support)
    return SceneModel2D(supports=tuple(supports), workspace=scene.workspace,
                        name=f"{scene.name}_inner{len(supports)}")
