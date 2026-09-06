"""Hard-support contact functions (problem_spec.md §2-§3).

Checker #1: Perram-Wertheim contact value, h = sqrt(F_PW) - 1 (dimensionless
scaling margin; sign agrees exactly with hard-set separation).
Checker #2: independent metric checker via point-to-ellipse distance.

Both are vectorized over point/pair arrays. The scene side of the pilot is
discs only, which keeps PW closed-form-diagonal; the general ellipse-ellipse
PW path is provided for tests and later scene families.
"""
from __future__ import annotations

import numpy as np

# ---------------------------------------------------------------------------
# Checker #1: Perram-Wertheim
# ---------------------------------------------------------------------------


def pw_ellipse_disc(u1, u2, a, b, R, iters=48):
    """PW contact value for an axis-aligned ellipse (semi-axes a,b, at origin)
    vs discs of radius R centered at body-frame points (u1,u2).

    S(lam) = lam(1-lam) * [u1^2/D1 + u2^2/D2],
    D1 = (1-lam) a^2 + lam R^2,  D2 = (1-lam) b^2 + lam R^2.
    Returns h = sqrt(max_lam S) - 1.
    """
    u1 = np.asarray(u1, dtype=np.float64)
    u2 = np.asarray(u2, dtype=np.float64)
    u1sq, u2sq = u1 * u1, u2 * u2
    a2, b2, R2 = a * a, b * b, R * R

    lo = np.zeros_like(u1sq)
    hi = np.ones_like(u1sq)
    for _ in range(iters):
        lam = 0.5 * (lo + hi)
        D1 = (1.0 - lam) * a2 + lam * R2
        D2 = (1.0 - lam) * b2 + lam * R2
        # g = dS/dlam
        g = (u1sq * ((1.0 - 2.0 * lam) * D1 - lam * (1.0 - lam) * (R2 - a2)) / (D1 * D1)
             + u2sq * ((1.0 - 2.0 * lam) * D2 - lam * (1.0 - lam) * (R2 - b2)) / (D2 * D2))
        take_hi = g > 0.0
        lo = np.where(take_hi, lam, lo)
        hi = np.where(take_hi, hi, lam)
    lam = 0.5 * (lo + hi)
    D1 = (1.0 - lam) * a2 + lam * R2
    D2 = (1.0 - lam) * b2 + lam * R2
    F = lam * (1.0 - lam) * (u1sq / D1 + u2sq / D2)
    return np.sqrt(F) - 1.0


def pw_ellipse_ellipse(r1, r2, Ainv, Binv, iters=48):
    """General 2D PW for pairs: center offsets (r1,r2) world frame, with
    A^-1, B^-1 given as (..,2,2) arrays (inverse quadratic-form matrices,
    i.e. A^-1 = Rot @ diag(a^2,b^2) @ Rot^T).  Vectorized over leading dims."""
    r = np.stack([np.asarray(r1, dtype=np.float64),
                  np.asarray(r2, dtype=np.float64)], axis=-1)
    Ainv = np.asarray(Ainv, dtype=np.float64)
    Binv = np.asarray(Binv, dtype=np.float64)
    dM = Binv - Ainv  # dM/dlam

    def inv2(M):
        det = M[..., 0, 0] * M[..., 1, 1] - M[..., 0, 1] * M[..., 1, 0]
        out = np.empty_like(M)
        out[..., 0, 0] = M[..., 1, 1]
        out[..., 1, 1] = M[..., 0, 0]
        out[..., 0, 1] = -M[..., 0, 1]
        out[..., 1, 0] = -M[..., 1, 0]
        return out / det[..., None, None]

    def S_and_g(lam):
        lamE = lam[..., None, None]
        M = (1.0 - lamE) * Ainv + lamE * Binv
        Minv = inv2(M)
        v = np.einsum('...ij,...j->...i', Minv, r)
        q = np.einsum('...i,...i->...', r, v)          # r^T M^-1 r
        qd = np.einsum('...i,...ij,...j->...', v, dM, v)  # r^T M^-1 dM M^-1 r
        S = lam * (1.0 - lam) * q
        g = (1.0 - 2.0 * lam) * q - lam * (1.0 - lam) * qd
        return S, g

    lo = np.zeros(r.shape[:-1])
    hi = np.ones(r.shape[:-1])
    for _ in range(iters):
        lam = 0.5 * (lo + hi)
        _, g = S_and_g(lam)
        take_hi = g > 0.0
        lo = np.where(take_hi, lam, lo)
        hi = np.where(take_hi, hi, lam)
    lam = 0.5 * (lo + hi)
    S, _ = S_and_g(lam)
    return np.sqrt(np.maximum(S, 0.0)) - 1.0


# ---------------------------------------------------------------------------
# Checker #2: independent metric checker (point-to-ellipse distance)
# ---------------------------------------------------------------------------


def point_ellipse_distance(p1, p2, a, b, iters=90):
    """Distance from body-frame points (p1,p2) to the boundary of the ellipse
    (x/a)^2+(y/b)^2=1; returns 0 for points inside.  Standard root-finding on
    F(t) = (a e1/(t+a^2))^2 + (b e2/(t+b^2))^2 - 1, bisection (independent of
    the PW math path)."""
    e1 = np.abs(np.asarray(p1, dtype=np.float64))
    e2 = np.abs(np.asarray(p2, dtype=np.float64))
    inside = (e1 / a) ** 2 + (e2 / b) ** 2 <= 1.0

    d = np.hypot(e1, e2)
    t_lo = np.zeros_like(d)
    t_hi = np.maximum(a, b) * d + max(a, b) ** 2 + 1.0
    # expand upper bound where needed
    for _ in range(60):
        F_hi = (a * e1 / (t_hi + a * a)) ** 2 + (b * e2 / (t_hi + b * b)) ** 2 - 1.0
        if not np.any(F_hi > 0.0):
            break
        t_hi = np.where(F_hi > 0.0, 2.0 * t_hi, t_hi)
    for _ in range(iters):
        t = 0.5 * (t_lo + t_hi)
        F = (a * e1 / (t + a * a)) ** 2 + (b * e2 / (t + b * b)) ** 2 - 1.0
        go_up = F > 0.0
        t_lo = np.where(go_up, t, t_lo)
        t_hi = np.where(go_up, t_hi, t)
    t = 0.5 * (t_lo + t_hi)
    cx = a * a * e1 / (t + a * a)
    cy = b * b * e2 / (t + b * b)
    dist = np.hypot(e1 - cx, e2 - cy)
    return np.where(inside, 0.0, dist)


def checker2_signed(p1, p2, a, b, R):
    """Independent signed metric margin for disc(R) at body-frame (p1,p2) vs
    the robot ellipse: dist(center, ellipse) - R, with 0-distance for centers
    inside (=> negative margin)."""
    dist = point_ellipse_distance(p1, p2, a, b)
    inside = (np.abs(p1) / a) ** 2 + (np.abs(p2) / b) ** 2 <= 1.0
    return np.where(inside, -R - 1e-12, dist - R)


# ---------------------------------------------------------------------------
# Workspace containment (closed-form support projections)
# ---------------------------------------------------------------------------


def support_half_widths(a, b, theta):
    """Projection half-widths of the heading-theta ellipse onto world x / y."""
    c, s = np.cos(theta), np.sin(theta)
    px = np.sqrt((a * c) ** 2 + (b * s) ** 2)
    py = np.sqrt((a * s) ** 2 + (b * c) ** 2)
    return px, py
