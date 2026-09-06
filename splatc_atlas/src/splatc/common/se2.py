"""SE(2) conventions frozen in problem_spec.md §1."""
from __future__ import annotations

import numpy as np

TWO_PI = 2.0 * np.pi


def wrap_angle(theta):
    """Wrap to [0, 2*pi)."""
    return np.mod(theta, TWO_PI)


def wrap_diff(dtheta):
    """Wrap to (-pi, pi]."""
    return np.pi - np.mod(np.pi - dtheta, TWO_PI)


def rot(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def world_to_body(theta, vx, vy):
    """Rotate world-frame vectors into the body frame (vectorized)."""
    c, s = np.cos(theta), np.sin(theta)
    return c * vx + s * vy, -s * vx + c * vy


def se2_step_cost(dx, dy, dtheta, ell_r):
    """Frozen common path cost increment (problem_spec §1)."""
    return np.sqrt(dx * dx + dy * dy + (ell_r * wrap_diff(dtheta)) ** 2)


def path_cost(poses, ell_r):
    poses = np.asarray(poses, dtype=float)
    d = np.diff(poses, axis=0)
    return float(np.sum(se2_step_cost(d[:, 0], d[:, 1], d[:, 2], ell_r)))
