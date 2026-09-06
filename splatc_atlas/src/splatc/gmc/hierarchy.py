"""Two-level certified slice queries (coarse merged layer + raw refinement).

Protocol (worklog gmc_G1.md):
  - Coarse layer: merged tangent polygons (+) robot polygon (exact convex
    Minkowski). The cover is conservative, so coarse-FREE / coarse-OPEN
    verdicts are certified.
  - Refinement: where coarse says CLOSED and the answer matters, replace hot
    buckets (scene polygon within reach of the query patch) by their raw
    covariance-add members and recompute. The mixed set still covers the true
    forbidden set, so refined-OPEN is certified too.
  - Adaptive growth: a refined-CLOSED verdict may err only via far-field
    conservatism (true path detouring outside the patch), so grow the patch
    and repeat; once every bucket is hot the set is fully raw and the verdict
    is exact. Termination is structural.
"""
import numpy as np
import shapely
from shapely.geometry import MultiPoint

from .cluster import MergedScene
from .geometry import (circum_factor, ellipse_polys, forbidden_ellipse_polys,
                       minkowski_sum, robot_cov, rot2, unit_dirs)


def robot_poly(theta, lam0, rho, ndir=24):
    """Circumscribed CCW polygon of the robot support ellipse at angle theta,
    centered at the origin."""
    lam = robot_cov(theta, lam0)
    return ellipse_polys(np.zeros((1, 2)), lam[None], rho, ndir,
                         factor=circum_factor(ndir))[0]


def coarse_forbidden(ms: MergedScene, theta, lam0, subset=None):
    """Shapely polygons of the coarse forbidden primitives at theta."""
    polys = ms.polys if subset is None else ms.polys[subset]
    return shapely.polygons(minkowski_sum(polys, robot_poly(theta, lam0, ms.rho, ms.ndir)))


def raw_forbidden(mu, S, theta, lam0, rho, ndir=24):
    """Shapely polygons of raw covariance-add forbidden primitives at theta."""
    return shapely.polygons(forbidden_ellipse_polys(mu, S, theta, lam0, rho, ndir))


def free_topology(polys, workspace, probes=()):
    """((n_components, n_holes), gate_open, free_geometry).

    gate_open: some single free component intersects EVERY probe geometry.
    Probes must be sets/segments, not points — fattened obstacles can swallow
    a point probe and misreport a visibly open passage (worklog gmc_H2.md)."""
    union = shapely.union_all(list(polys))
    free = workspace.difference(union)
    geoms = [g for g in getattr(free, "geoms", [free])
             if g.geom_type == "Polygon" and g.area > 1e-9]
    holes = sum(len(g.interiors) for g in geoms)
    gate_open = None
    if probes:
        gate_open = any(all(g.intersects(p) for p in probes) for g in geoms)
    return (len(geoms), holes), gate_open, free


def hot_buckets(ms: MergedScene, patch, reach):
    """Bucket mask: scene polygon within `reach` of the query patch (a
    forbidden primitive can influence connectivity inside the patch iff its
    scene polygon intersects patch (+) robot support)."""
    region = patch.buffer(reach)
    geoms = shapely.polygons(ms.polys)
    return np.array([g.intersects(region) for g in geoms])


def gate_query(ms: MergedScene, theta, lam0, workspace, probes,
               growth_step=None, max_rounds=32):
    """Certified two-level gate query with adaptive patch growth.

    Returns dict with verdict ("open"/"closed"), certificate level
    ("coarse" / "refined-<k>" / "exact"), and refinement stats."""
    half_len = ms.rho * float(np.sqrt(np.linalg.eigvalsh(lam0)[-1]))
    if growth_step is None:
        growth_step = half_len
    _, gate_open, _ = free_topology(coarse_forbidden(ms, theta, lam0),
                                    workspace, probes)
    if gate_open:
        return {"verdict": "open", "level": "coarse", "rounds": 0, "raw_members": 0}
    patch = MultiPoint([c for p in probes for c in p.coords]).convex_hull.buffer(
        half_len + 0.3)
    for k in range(1, max_rounds + 1):
        hot = hot_buckets(ms, patch, half_len)
        raw_idx = (np.concatenate([ms.buckets[i] for i in np.flatnonzero(hot)])
                   if hot.any() else np.empty(0, np.int64))
        mixed = list(raw_forbidden(ms.mu[raw_idx], ms.S[raw_idx], theta, lam0,
                                   ms.rho, ms.ndir))
        if not hot.all():
            mixed += list(coarse_forbidden(ms, theta, lam0, subset=~hot))
        _, gate_open, _ = free_topology(mixed, workspace, probes)
        if gate_open:
            return {"verdict": "open", "level": f"refined-{k}", "rounds": k,
                    "raw_members": int(len(raw_idx))}
        if hot.all():
            return {"verdict": "closed", "level": "exact", "rounds": k,
                    "raw_members": int(len(raw_idx))}
        patch = patch.buffer(growth_step)
    raise RuntimeError("gate_query failed to converge within max_rounds")
