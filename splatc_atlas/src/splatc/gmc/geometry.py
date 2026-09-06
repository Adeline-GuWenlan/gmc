"""Convex geometry kernels for the GMC contact-complex layer.

Pure numpy. Conventions:
  - Ellipse {x : (x-c)^T S^-1 (x-c) <= rho^2} for SPD 2x2 S.
  - Polygons are (..., n, 2) vertex arrays in CCW order.
  - `factor` >= 1 circumscribes (polygon contains ellipse), factor 1 inscribes
    (vertices on the boundary). Circumscribed factor for n directions is
    1/cos(pi/n).

Soundness-critical invariants (tested in tests/test_gmc.py):
  I1  minkowski_sum(P, Q) is the exact Minkowski sum for convex CCW inputs.
  I2  ellipse_polys(..., factor=circum_factor(n)) contains the true ellipse.
  I3  tangent_polys(support of a set) contains every member of the set.
"""
import numpy as np

_EPS = 1e-12


def circum_factor(ndir):
    return 1.0 / np.cos(np.pi / ndir)


def unit_dirs(ndir):
    a = np.linspace(0, 2 * np.pi, ndir, endpoint=False)
    return np.stack([np.cos(a), np.sin(a)], axis=1)


def _chol2(S):
    """Batched 2x2 Cholesky factors (l11, l21, l22) with degeneracy guard."""
    a, b, d = S[..., 0, 0], S[..., 0, 1], S[..., 1, 1]
    sa = np.sqrt(np.maximum(a, _EPS))
    l21 = b / sa
    l22 = np.sqrt(np.maximum(d - l21 ** 2, _EPS))
    return sa, l21, l22


def ellipse_polys(mu, S, rho, ndir=24, factor=1.0):
    """(n, ndir, 2) CCW polygons of rho-ellipses; factor>=1 circumscribes."""
    ang = np.linspace(0, 2 * np.pi, ndir, endpoint=False)
    cx, sx = np.cos(ang), np.sin(ang)
    sa, l21, l22 = _chol2(S)
    r = rho * factor
    px = mu[:, 0, None] + r * sa[:, None] * cx[None]
    py = mu[:, 1, None] + r * (l21[:, None] * cx[None] + l22[:, None] * sx[None])
    return np.stack([px, py], axis=-1)


def rot2(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def robot_cov(theta, lam0):
    R = rot2(theta)
    return R @ lam0 @ R.T


def forbidden_ellipse_polys(mu, S, theta, lam0, rho, ndir=24, factor=1.0):
    """Raw covariance-add forbidden primitives E(S_i + R lam0 R^T) at pose
    angle theta (exact overlap-threshold semantics)."""
    return ellipse_polys(mu, S + robot_cov(theta, lam0)[None], rho, ndir, factor)


def support_ellipses(mu, S, dirs, rho):
    """(n, k) support values h_i(u_k) = c_i.u_k + rho*sqrt(u_k^T S_i u_k)."""
    quad = np.einsum("ki,nij,kj->nk", dirs, S, dirs)
    return mu @ dirs.T + rho * np.sqrt(np.maximum(quad, 0.0))


def tangent_polys(h, dirs):
    """(m, k, 2) polygons from support values h (m, k) over directions dirs
    (k, 2) at uniformly increasing angles: vertex_j = intersection of tangent
    lines j and j+1. Circumscribes the underlying convex sets by construction."""
    u1, u2 = dirs, np.roll(dirs, -1, axis=0)
    det = u1[:, 0] * u2[:, 1] - u1[:, 1] * u2[:, 0]
    h2 = np.roll(h, -1, axis=-1)
    vx = (h * u2[:, 1] - h2 * u1[:, 1]) / det
    vy = (h2 * u1[:, 0] - h * u2[:, 0]) / det
    return np.stack([vx, vy], axis=-1)


def _bottom_vertex(P, tol=1e-9):
    """(n, 2) bottom-most (then left-most) vertex per polygon (n, p, 2)."""
    ymin = P[..., 1].min(axis=-1, keepdims=True)
    x_masked = np.where(P[..., 1] <= ymin + tol, P[..., 0], np.inf)
    idx = np.argmin(x_masked, axis=-1)
    return P[np.arange(len(P)), idx]


def minkowski_sum(P, Q):
    """Exact Minkowski sum of each convex CCW polygon in P (n, p, 2) with one
    convex CCW polygon Q (q, 2). Returns (n, p+q, 2).

    Edge-merge construction with two robustness rules learned the hard way:
      1. Edge angles are normalized to [0, 2pi) before sorting — plain atan2
         range (-pi, pi] inserts downward edges first and yields
         self-intersecting chains.
      2. The chain is built from an ARBITRARY cyclic cut and then translated
         so its bottom vertex matches bottom(P) + bottom(Q). Anchoring the
         start vertex directly is NOT safe: an edge pointing at angle ~0 can
         land at ~2pi from float noise in atan2, moving it to the other end
         of the sort — a legal cyclic rotation (the shape is unchanged; angle
         0 and 2pi are cyclically adjacent) that displaces a start-anchored
         chain by that edge's full length."""
    n = len(P)
    eP = np.roll(P, -1, axis=1) - P
    eQ = np.roll(Q, -1, axis=0) - Q
    edges = np.concatenate([eP, np.broadcast_to(eQ[None], (n, len(Q), 2))], axis=1)
    ang = np.mod(np.arctan2(edges[..., 1], edges[..., 0]), 2 * np.pi)
    order = np.argsort(ang, axis=1)
    se = np.take_along_axis(edges, order[..., None].repeat(2, -1), axis=1)
    csum = np.cumsum(se, axis=1)
    chain = np.concatenate([np.zeros((n, 1, 2)), csum[:, :-1]], axis=1)
    anchor = _bottom_vertex(P) + _bottom_vertex(Q[None])[0]
    return chain - _bottom_vertex(chain)[:, None, :] + anchor[:, None, :]
