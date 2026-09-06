"""Conservative splat clustering (module M1) — merged tangent-polygon primitives.

Buckets splats by (grid cell, major-axis angle bin) and represents each bucket
by the circumscribed tangent polygon of the union of member rho-support
ellipses (support-exact per direction, no ellipse-fit slack).

Soundness chain, for every member i, robot cov Lam(theta), any theta:
    E(S_i + Lam) subset E(S_i) (+) E(Lam)      [sqrt(a+b) <= sqrt a + sqrt b]
                 subset T_m    (+) E(Lam)      [tangent poly covers member,
                                                theta-independent]
                 subset T_m    (+) R_poly      [circumscribed robot polygon]
NOTE the direct covariance-add construction on a merged shape is UNSOUND
(scene-level containment does not survive robot convolution; measured in
m1_cluster_pilot v1) — merged primitives must go through the Minkowski route.

Pilot calibration on K2 door_A (worklog gmc_G1.md): defaults (cell=0.5,
angle_bins=8) sit on the fidelity/cost Pareto front; gate fidelity is
insensitive to these parameters and exactness is recovered by the refinement
layer (hierarchy.py), so treat them as pure cost knobs.
"""
from dataclasses import dataclass, field

import numpy as np

from .geometry import support_ellipses, tangent_polys, unit_dirs


def major_angle(S):
    _, evecs = np.linalg.eigh(S)
    v = evecs[:, :, 1]
    return np.arctan2(v[:, 1], v[:, 0]) % np.pi


def bucket_indices(mu, S, cell=0.5, angle_bins=8):
    """List of index arrays, one per (cell, angle-bin) bucket."""
    ang = major_angle(S)
    c = np.floor(mu / cell).astype(np.int64)
    ab = np.floor(ang / (np.pi / angle_bins)).astype(np.int64) % angle_bins
    key = c[:, 0] * 1_000_003 + c[:, 1] * 1009 + ab
    order = np.argsort(key)
    ks = key[order]
    bounds = np.flatnonzero(np.r_[True, ks[1:] != ks[:-1], True])
    return [order[i:j] for i, j in zip(bounds[:-1], bounds[1:])]


@dataclass
class MergedScene:
    mu: np.ndarray            # (n, 2) raw splat means
    S: np.ndarray             # (n, 2, 2) raw splat covariances
    rho: float
    polys: np.ndarray         # (n_buckets, ndir, 2) merged tangent polygons
    buckets: list = field(repr=False)   # member index arrays per bucket
    ndir: int = 24
    cell: float = 0.5
    angle_bins: int = 8
    margin: float = 1.001


def build_merged(mu, S, rho=2.0, cell=0.5, angle_bins=8, ndir=24, margin=1.001):
    """Cluster raw splats into a MergedScene of tangent-polygon primitives."""
    buckets = bucket_indices(mu, S, cell, angle_bins)
    dirs = unit_dirs(ndir)
    # margin inflates the RADIUS term only (rho*margin). Scaling the whole
    # support value h = c.u + rho*sqrt(u^T S u) would SHRINK it wherever
    # c.u < 0 and pull tangent lines inward — a leakage bug caught by
    # tests/test_gmc.py::TestClusterSoundness on first run.
    h_all = support_ellipses(mu, S, dirs, rho * margin)
    h = np.stack([h_all[idx].max(axis=0) for idx in buckets])
    polys = tangent_polys(h, dirs)
    return MergedScene(mu=mu, S=S, rho=rho, polys=polys, buckets=buckets,
                       ndir=ndir, cell=cell, angle_bins=angle_bins, margin=margin)
