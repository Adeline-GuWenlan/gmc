"""Smooth Gaussian overlap c_ij and its pose derivatives (module 2, Sprint B).

Contract split (00 §1.3): c_ij provides SMOOTH cues (active pairs, gradients,
refinement signals) for the compiler; it is NEVER collision truth — that stays
with the hard-support checkers.

Convention: each primitive's covariance is its hard-support quadratic form
(Sigma = Rot diag(a^2, b^2) Rot^T, i.e. tau = 1).  The unnormalized overlap of
scene Gaussian i (cov S_i, center mu_i) with the robot Gaussian at pose
q = (p, theta) is

  c(q) = exp(-1/2 d^T M^-1 d),  d = mu_i - p,  M(theta) = S_i + Sigma_R(theta).

Value is monotone in the Mahalanobis distance; the normalization constant is
dropped deliberately (documented; only relative structure feeds the compiler).
"""
from __future__ import annotations

import numpy as np


def robot_cov(a, b, theta):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return R @ np.diag([a * a, b * b]) @ R.T


def _drobot_cov_dtheta(a, b, theta):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    dR = np.array([[-s, -c], [c, -s]])
    D = np.diag([a * a, b * b])
    return dR @ D @ R.T + R @ D @ dR.T


def overlap(scene_cov, mu, robot, q):
    """c(q) for one scene Gaussian; q=(x, y, theta)."""
    x, y, theta = q
    M = np.asarray(scene_cov) + robot_cov(robot.a, robot.b, theta)
    d = np.asarray(mu, dtype=float) - np.array([x, y])
    v = np.linalg.solve(M, d)
    return float(np.exp(-0.5 * d @ v))


def overlap_grad(scene_cov, mu, robot, q):
    """(c, dc/dx, dc/dy, dc/dtheta) — analytic."""
    x, y, theta = q
    Sr = robot_cov(robot.a, robot.b, theta)
    M = np.asarray(scene_cov) + Sr
    d = np.asarray(mu, dtype=float) - np.array([x, y])
    Minv = np.linalg.inv(M)
    v = Minv @ d
    c = float(np.exp(-0.5 * d @ v))
    # d/dp (-1/2 d^T Minv d) with d = mu - p  =>  + Minv d
    gx, gy = c * v
    dM = _drobot_cov_dtheta(robot.a, robot.b, theta)
    dtheta = c * 0.5 * float(v @ dM @ v)
    return c, float(gx), float(gy), float(dtheta)
