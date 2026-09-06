"""Independent collision oracle (Guide §5.3 "Independent oracle").

Ground truth for tests must NOT reuse the support code.  Collision between two
solid ellipses is decided by the convex problem

    min_x (x-c1)^T Q1^-1 (x-c1)
    s.t.  (x-c2)^T Q2^-1 (x-c2) <= l2^2.

We use the Perram--Wertheim contact function.  With ``P1=l1^2 Q1``,
``P2=l2^2 Q2`` and ``d=c2-c1``, the ellipses intersect exactly when

    max_{a in [0,1]} a(1-a) d^T ((1-a)P1 + aP2)^-1 d <= 1.

The derivative has the required positive/negative endpoint signs and a unique
interior maximum.  A bracketed scalar root avoids the previous SLSQP failure
mode in which an unsuccessful solve was silently read as collision-free.
"""
import numpy as np
from scipy.optimize import brentq

from ..types import GaussianSupport2D, rotation2


def ellipses_collide(c1, Q1, l1, c2, Q2, l2) -> bool:
    c1, c2 = np.asarray(c1, float), np.asarray(c2, float)
    Q1, Q2 = np.asarray(Q1, float), np.asarray(Q2, float)
    l1, l2 = float(l1), float(l2)
    if (c1.shape != (2,) or c2.shape != (2,)
            or Q1.shape != (2, 2) or Q2.shape != (2, 2)
            or not np.all(np.isfinite(c1)) or not np.all(np.isfinite(c2))
            or not np.all(np.isfinite(Q1)) or not np.all(np.isfinite(Q2))
            or not np.isfinite(l1) or not np.isfinite(l2)
            or l1 <= 0.0 or l2 <= 0.0):
        raise ValueError("ellipse parameters must be finite with positive levels")
    if (not np.allclose(Q1, Q1.T, rtol=1e-10, atol=1e-12)
            or not np.allclose(Q2, Q2.T, rtol=1e-10, atol=1e-12)):
        raise ValueError("ellipse covariance must be symmetric")
    Q1 = 0.5 * (Q1 + Q1.T)
    Q2 = 0.5 * (Q2 + Q2.T)
    if (np.linalg.eigvalsh(Q1)[0] <= 0.0
            or np.linalg.eigvalsh(Q2)[0] <= 0.0):
        raise ValueError("ellipse covariance must be positive definite")

    P1 = np.asarray((l1 * l1) * Q1, dtype=np.longdouble)
    P2 = np.asarray((l2 * l2) * Q2, dtype=np.longdouble)
    delta = np.asarray(c2 - c1, dtype=np.longdouble)
    if np.all(delta == 0.0):
        return True
    change = P2 - P1

    def solve_spd2(matrix, rhs):
        """Long-double 2x2 solve; NumPy linalg downcasts/rejects float128."""
        a = matrix[0, 0]
        b = (matrix[0, 1] + matrix[1, 0]) / np.longdouble(2.0)
        c = matrix[1, 1]
        determinant = a * c - b * b
        if not np.isfinite(determinant) or determinant <= 0.0:
            raise FloatingPointError("non-positive contact matrix determinant")
        x = (c * rhs[0] - b * rhs[1]) / determinant
        y = (a * rhs[1] - b * rhs[0]) / determinant
        result = np.asarray([x, y], dtype=np.longdouble)
        if not np.all(np.isfinite(result)):
            raise FloatingPointError("non-finite contact solve")
        return result

    def value_and_derivative(alpha: float):
        a = np.longdouble(alpha)
        matrix = P1 + a * change
        solved = solve_spd2(matrix, delta)
        quadratic = np.dot(delta, solved)
        factor = a * (np.longdouble(1.0) - a)
        value = factor * quadratic
        derivative = ((np.longdouble(1.0) - np.longdouble(2.0) * a)
                      * quadratic - factor * np.dot(solved, change @ solved))
        if not np.isfinite(value) or not np.isfinite(derivative):
            raise FloatingPointError("non-finite contact function")
        return float(value), float(derivative)

    try:
        derivative0 = value_and_derivative(0.0)[1]
        derivative1 = value_and_derivative(1.0)[1]
        if not derivative0 > 0.0 or not derivative1 < 0.0:
            raise FloatingPointError("contact derivative is not bracketed")
        alpha = brentq(lambda a: value_and_derivative(a)[1], 0.0, 1.0,
                       xtol=5e-15, rtol=4.0 * np.finfo(float).eps,
                       maxiter=200)
        maximum = value_and_derivative(float(alpha))[0]
    except (ValueError, FloatingPointError) as exc:
        # This function supplies independent ground truth. A numerical failure
        # is not evidence of free space and must never become False.
        raise RuntimeError("ellipse collision oracle unresolved") from exc
    # Dimensionless contact tolerance.  It covers the observed binary64 input
    # representation error at analytic tangency without swallowing a 1e-10
    # outward separation in the high-condition regression below.
    return bool(maximum <= 1.0 + 1e-12)


def body_world(body: GaussianSupport2D, t, theta: float):
    """World-frame (center, shape, level) of a body support at pose (t, theta)."""
    R = rotation2(theta)
    return (np.asarray(t, float) + R @ body.mean,
            R @ body.covariance @ R.T, body.level)


def pose_collides(scene_support: GaussianSupport2D,
                  body: GaussianSupport2D, t, theta: float,
                  ledger=None) -> bool:
    """Independent membership test t in O_ij^theta."""
    if ledger is not None:
        ledger.charge("pair_collision_evals")
    # Cancel the common world translation before constructing the body centre.
    # Forming ``t + R@nu`` near 1e14 and subtracting ``scene.mean`` later can
    # discard the local contact offset and corrupt this independent oracle.
    R = rotation2(theta)
    local_body_center = (
        np.asarray(t, dtype=np.longdouble)
        - np.asarray(scene_support.mean, dtype=np.longdouble)
        + np.asarray(R, dtype=np.longdouble)
        @ np.asarray(body.mean, dtype=np.longdouble)
    )
    Qb = R @ body.covariance @ R.T
    return ellipses_collide(
        np.zeros(2, dtype=float), scene_support.covariance,
        scene_support.level,
        np.asarray(local_body_center, dtype=float), Qb, body.level,
    )


def scene_pose_collides(scene, robot, t, theta: float, ledger=None) -> bool:
    if ledger is not None:
        ledger.charge("pose_collision_queries")
    return any(pose_collides(s, b, t, theta, ledger=ledger)
               for s in scene.supports for b in robot.supports)
