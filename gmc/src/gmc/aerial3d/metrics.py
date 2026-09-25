"""Path smoothness metrics shared by C1 and C2 for both methods (same code, same definitions).

Raw polyline (after dropping zero-length segments, tolerance 1e-10 m):

* ``path_length_m``           sum of segment lengths
* ``n_vertices``              polyline vertices (``n_segments`` = n_vertices - 1)
* ``total_turning_rad``       sum of turning angles theta_i at interior vertices
* ``max_turning_rad``         largest theta_i
* ``curvature_integral_rad``  discrete total curvature = sum theta_i (the integral of
                              |kappa| ds of any smooth curve through a turn theta is >= theta)
* ``bending_energy_per_m``    discrete int kappa^2 ds = sum theta_i^2 / ((l_{i-1} + l_i) / 2)
* ``vertical_travel_m``, ``altitude_range_m``

After the **same** ``gs3d.smoothing.optimize_trajectory`` post-process on either
method's raw path, ``smooth_segments_metrics`` gives, over its cubic-Bezier +
quintic-ease segments, the exact integrated squared jerk
``int |x'''(t)|^2 dt`` (Gauss-Legendre, exact for these polynomials) and duration.
"""
from __future__ import annotations

import numpy as np

_GL_X, _GL_W = np.polynomial.legendre.leggauss(24)
_GL_U, _GL_W = (_GL_X + 1) / 2, _GL_W / 2


def polyline_metrics(points) -> dict:
    P = np.atleast_2d(np.asarray(points, float))
    if len(P) > 1:
        keep = np.r_[True, np.linalg.norm(np.diff(P, axis=0), axis=1) > 1e-10]
        P = P[keep]
    seg = np.diff(P, axis=0)
    lengths = np.linalg.norm(seg, axis=1)
    theta = np.empty(0)
    if len(seg) > 1:
        u = seg / lengths[:, None]
        theta = np.arccos(np.clip(np.sum(u[:-1] * u[1:], axis=1), -1., 1.))
    bending = float(np.sum(theta ** 2 / (.5 * (lengths[:-1] + lengths[1:])))) if len(theta) else 0.
    return {"path_length_m": float(lengths.sum()), "n_vertices": int(len(P)),
            "n_segments": int(len(seg)),
            "total_turning_rad": float(theta.sum()), "max_turning_rad": float(theta.max(initial=0.)),
            "curvature_integral_rad": float(theta.sum()), "bending_energy_per_m": bending,
            "vertical_travel_m": float(np.abs(seg[:, 2]).sum()) if len(seg) else 0.,
            "altitude_range_m": float(np.ptp(P[:, 2])) if len(P) else 0.}


def _ease_derivatives(u):
    e1 = 30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4
    e2 = 60 * u - 180 * u ** 2 + 120 * u ** 3
    e3 = 60 - 360 * u + 360 * u ** 2
    e0 = u ** 3 * (10 + u * (-15 + 6 * u))
    return e0, e1, e2, e3


def _bezier_derivatives(control, s):
    P0, P1, P2, P3 = control
    s = s[:, None]
    b1 = 3 * ((1 - s) ** 2 * (P1 - P0) + 2 * (1 - s) * s * (P2 - P1) + s ** 2 * (P3 - P2))
    b2 = 6 * ((1 - s) * (P2 - 2 * P1 + P0) + s * (P3 - 2 * P2 + P1))
    b3 = np.broadcast_to(6 * (P3 - 3 * P2 + 3 * P1 - P0), b1.shape)
    return b1, b2, b3


def bezier_ease5_jerk_sq(control, duration_s: float) -> float:
    """int_0^T |d^3/dt^3 B(e(t/T))|^2 dt for a cubic Bezier B and quintic ease e."""
    control = np.asarray(control, float)
    T = float(duration_s)
    e0, e1, e2, e3 = _ease_derivatives(_GL_U)
    b1, b2, b3 = _bezier_derivatives(control, e0)
    jerk = (b3 * e1[:, None] ** 3 + 3 * b2 * (e1 * e2)[:, None] + b1 * e3[:, None]) / T ** 3
    return float(T * np.sum(_GL_W * np.sum(jerk ** 2, axis=1)))


def smooth_segments_metrics(segments) -> dict:
    jerk, duration, length, n = 0., 0., 0., 0
    for seg in segments:
        duration += float(seg["duration_s"])
        if seg.get("type") != "cubic_bezier_ease5":
            continue  # rotate-in-place segments: no translation, zero jerk
        control = np.asarray(seg["control_points_xyz"], float)
        n += 1
        jerk += bezier_ease5_jerk_sq(control, seg["duration_s"])
        b1, _, _ = _bezier_derivatives(control, _GL_U)
        length += float(np.sum(_GL_W * np.linalg.norm(b1, axis=1)))
    return {"integrated_squared_jerk": jerk, "duration_s": duration,
            "smooth_path_length_m": length, "bezier_segments": n,
            "definition": "exact int |x'''|^2 dt over cubic-Bezier + quintic-ease segments "
                          "(24-point Gauss-Legendre per segment)"}
