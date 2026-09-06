"""M1 pair support oracle (Guide §5) — the core GS-native geometry kernel.

Pair C-obstacle (exact set, generally NOT an ellipse):
    O_ij^theta = mu_i - R_theta nu_j + (E_i^0 (+) (-R_theta R_j^0))
Support function:
    h(u) = u.(mu_i - R nu_j) + kappa_i sqrt(u^T Sigma_i u)
                             + rho_j sqrt(u^T R Lambda_j R^T u)
Support point = gradient of h (a true boundary point of O).
"""
from collections import OrderedDict
from dataclasses import KW_ONLY, dataclass, field

import numpy as np

from ..types import GaussianSupport2D, PairID, canonical_angle, rotation2


def _unit_rows(U: np.ndarray) -> np.ndarray:
    """Return finite, nonzero direction rows with a strict ``(K, 2)`` API."""
    rows = np.asarray(U, dtype=float)
    if rows.ndim == 1:
        if rows.shape != (2,):
            raise ValueError("support direction must have shape (2,) or (K, 2)")
        rows = rows[None, :]
    if rows.ndim != 2 or rows.shape[1] != 2 or rows.shape[0] == 0:
        raise ValueError("support directions must have nonempty shape (K, 2)")
    if not np.all(np.isfinite(rows)):
        raise ValueError("support directions must be finite")
    norms = np.linalg.norm(rows, axis=1)
    if not np.all(np.isfinite(norms)) or np.any(norms <= 0.0):
        raise ValueError("support directions must be nonzero")
    return rows / norms[:, None]


@dataclass
class PairOracle:
    pair_id: PairID
    scene: GaussianSupport2D
    body: GaussianSupport2D
    # Keep ``calls`` as the fourth positional argument for compatibility with
    # the original v0 dataclass constructor.  Every P5 field is keyword-only.
    calls: int = 0
    _: KW_ONLY
    revision: int = 0
    value_calls: int = 0
    point_calls: int = 0
    ledger: object | None = field(default=None, repr=False)
    max_cached_frames: int = 4096
    _theta_cache: OrderedDict = field(default_factory=OrderedDict, repr=False)

    def __post_init__(self):
        if int(self.max_cached_frames) != self.max_cached_frames \
                or self.max_cached_frames < 1:
            raise ValueError("max_cached_frames must be positive")
        self.max_cached_frames = int(self.max_cached_frames)
        if not isinstance(self._theta_cache, OrderedDict):
            self._theta_cache = OrderedDict(self._theta_cache)

    def _frame(self, theta: float):
        # Never quantize an orientation cache key: an arbitrarily small angle
        # change can cause a macroscopic translation when a body support has a
        # large local offset.  Python float keys preserve the exact input value
        # (with harmless -0.0/0.0 coalescing).
        key = canonical_angle(theta)
        hit = self._theta_cache.get(key)
        if hit is None:
            R = rotation2(theta)
            # Keep the pair centre in extended precision.  Forming a world
            # support value and later subtracting ``u @ t`` loses all local
            # clearance bits when both terms are O(1e14).  The long-double
            # centre is also the reference used to bound the one unavoidable
            # rounding step when local envelope vertices are exported to
            # binary64 world coordinates for GEOS.
            R_long = np.asarray(R, dtype=np.longdouble)
            center_long = (np.asarray(self.scene.mean, dtype=np.longdouble)
                           - R_long @ np.asarray(
                               self.body.mean, dtype=np.longdouble))
            center = np.asarray(center_long, dtype=np.float64)
            hit = (R, center, R @ self.body.covariance @ R.T, center_long)
            self._theta_cache[key] = hit
            while len(self._theta_cache) > self.max_cached_frames:
                self._theta_cache.popitem(last=False)
        else:
            self._theta_cache.move_to_end(key)
            if self.ledger is not None:
                self.ledger.charge("cache_hits")
        return hit

    def support_values(self, theta: float, U: np.ndarray) -> np.ndarray:
        """Batch support values for unit directions U of shape (K, 2)."""
        U = _unit_rows(U)
        if self.ledger is not None:
            self.ledger.charge("support_value_evals", len(U))
        _, center, A, _ = self._frame(theta)
        self.calls += len(U)
        self.value_calls += len(U)
        qs = np.einsum("ki,ij,kj->k", U, self.scene.covariance, U)
        qb = np.einsum("ki,ij,kj->k", U, A, U)
        return (U @ center + self.scene.level * np.sqrt(qs)
                + self.body.level * np.sqrt(qb))

    def support_points(self, theta: float, U: np.ndarray) -> np.ndarray:
        """Batch boundary points achieving u^T p = h(u), shape (K, 2)."""
        U = _unit_rows(U)
        if self.ledger is not None:
            self.ledger.charge("support_point_evals", len(U))
        _, center, A, _ = self._frame(theta)
        self.calls += len(U)
        self.point_calls += len(U)
        Su = U @ self.scene.covariance.T
        Au = U @ A.T
        qs = np.sqrt(np.einsum("ki,ki->k", U, Su))
        qb = np.sqrt(np.einsum("ki,ki->k", U, Au))
        return (center[None, :] + self.scene.level * Su / qs[:, None]
                + self.body.level * Au / qb[:, None])

    def center(self, theta: float) -> np.ndarray:
        return self._frame(theta)[1].copy()

    def center_longdouble(self, theta: float) -> np.ndarray:
        """Pair centre before the final binary64 world-coordinate cast."""
        return self._frame(theta)[3].copy()

    def local_support_values(self, theta: float, U: np.ndarray) -> np.ndarray:
        """Support radii about the pair centre, without a world translation.

        This has the same accounting semantics as :meth:`support_values` and
        is the numerically safe primitive for local envelope construction.
        """
        U = _unit_rows(U)
        if self.ledger is not None:
            self.ledger.charge("support_value_evals", len(U))
        _, _, A, _ = self._frame(theta)
        self.calls += len(U)
        self.value_calls += len(U)
        qs = np.einsum("ki,ij,kj->k", U, self.scene.covariance, U)
        qb = np.einsum("ki,ij,kj->k", U, A, U)
        return (self.scene.level * np.sqrt(qs)
                + self.body.level * np.sqrt(qb))

    def local_support_points(self, theta: float, U: np.ndarray) -> np.ndarray:
        """Boundary points relative to the pair centre, shape ``(K, 2)``."""
        U = _unit_rows(U)
        if self.ledger is not None:
            self.ledger.charge("support_point_evals", len(U))
        _, _, A, _ = self._frame(theta)
        self.calls += len(U)
        self.point_calls += len(U)
        Su = U @ self.scene.covariance.T
        Au = U @ A.T
        qs = np.sqrt(np.einsum("ki,ki->k", U, Su))
        qb = np.sqrt(np.einsum("ki,ki->k", U, Au))
        return (self.scene.level * Su / qs[:, None]
                + self.body.level * Au / qb[:, None])

    def separation_values(self, theta: float, t: np.ndarray,
                          U: np.ndarray) -> np.ndarray:
        """Return valid directional separation bounds in a pair-local frame.

        Algebraically this is ``u @ t - h_world(u)``.  Computing that formula
        literally subtracts two large, nearly equal world projections.  Here
        the translation is cancelled first in extended precision, so the
        returned local clearance retains the information present in the
        binary64 input coordinates.  Calls are charged exactly like one batch
        of support-value evaluations.
        """
        U = _unit_rows(U)
        t = np.asarray(t, dtype=float)
        if t.shape != (2,) or not np.all(np.isfinite(t)):
            raise ValueError("t must be a finite vector of shape (2,)")
        if self.ledger is not None:
            self.ledger.charge("support_value_evals", len(U))
        R, _, _, _ = self._frame(theta)
        self.calls += len(U)
        self.value_calls += len(U)

        ld = np.longdouble
        eps = np.finfo(ld).eps
        U_long = np.asarray(U, dtype=ld)
        # Evaluate t - (mu - R nu) without ever materialising a rounded
        # O(1e14) support value.  All operands here originate as binary64, so
        # long-double arithmetic preserves their small representable offset.
        # Binary64 operands are represented exactly in long double.  Their
        # nearby subtraction is exact by Sterbenz' lemma, so its forward-error
        # scale is the *local difference*, not the world-coordinate magnitude.
        delta = (np.asarray(t, dtype=ld)
                 - np.asarray(self.scene.mean, dtype=ld))
        rotated_body_mean = (np.asarray(R, dtype=ld)
                             @ np.asarray(self.body.mean, dtype=ld))
        local_t = delta + rotated_body_mean
        gamma5 = (ld(5.0) * eps) / (ld(1.0) - ld(5.0) * eps)
        local_error = gamma5 * (
            np.abs(delta)
            + np.abs(np.asarray(R, dtype=ld))
            @ np.abs(np.asarray(self.body.mean, dtype=ld))
        )
        scene_cov = np.asarray(self.scene.covariance, dtype=ld)
        R_long = np.asarray(R, dtype=ld)
        body_cov = (R_long
                    @ np.asarray(self.body.covariance, dtype=ld)
                    @ R_long.T)
        qs = np.einsum("ki,ij,kj->k", U_long, scene_cov, U_long)
        qb = np.einsum("ki,ij,kj->k", U_long, body_cov, U_long)
        radial = (ld(self.scene.level) * np.sqrt(qs)
                  + ld(self.body.level) * np.sqrt(qb))
        projection = U_long @ local_t
        gamma16 = (ld(16.0) * eps) / (ld(1.0) - ld(16.0) * eps)
        projection_error = (
            np.abs(U_long) @ local_error
            + gamma16 * (np.abs(U_long) @ np.abs(local_t))
        )
        radial_error = gamma16 * np.maximum(ld(1.0), np.abs(radial))
        # Subtract the scale-derived forward-error envelope, then round once
        # more toward -inf on export.  Thus every result remains a lower bound
        # rather than merely a high-accuracy estimate of separation.
        result = projection - radial - projection_error - radial_error
        exported = np.asarray(result, dtype=float)
        return np.nextafter(exported, -np.inf)

    def disc_clearance_lower_bound(self, theta: float,
                                   t: np.ndarray) -> float:
        """Conservative local-frame lower bound used for pair pruning."""
        t = np.asarray(t, dtype=float)
        if t.shape != (2,) or not np.all(np.isfinite(t)):
            return -np.inf
        R, _, _, _ = self._frame(theta)
        ld = np.longdouble
        eps = np.finfo(ld).eps
        R_long = np.asarray(R, dtype=ld)
        delta = (np.asarray(t, dtype=ld)
                 - np.asarray(self.scene.mean, dtype=ld))
        local_t = delta + R_long @ np.asarray(self.body.mean, dtype=ld)
        gamma8 = (ld(8.0) * eps) / (ld(1.0) - ld(8.0) * eps)
        operand_scale = (
            np.abs(delta)
            + np.abs(R_long) @ np.abs(
                np.asarray(self.body.mean, dtype=ld))
        )
        error = gamma8 * (
            np.sqrt(np.sum(operand_scale * operand_scale))
            + np.sqrt(np.sum(local_t * local_t))
            + ld(self.radius_bound())
        )
        value = (np.sqrt(np.sum(local_t * local_t))
                 - ld(np.nextafter(self.radius_bound(), np.inf))
                 - error)
        return float(np.nextafter(np.asarray(value, dtype=float), -np.inf))

    def translate_local_points(self, theta: float, points: np.ndarray):
        """Export local points to binary64 world coordinates with an error LB.

        Returns ``(world_points, error_bound)`` where every exported vertex is
        within ``error_bound`` (Euclidean norm) of the exact translation of
        the binary64 inputs and rotation matrix.  The bound combines measured
        long-double-to-binary64 cast error with standard gamma bounds for the
        few long-double multiply/add operations; it is therefore scale-aware,
        not an arbitrary absolute epsilon.
        """
        points = np.asarray(points, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 2
                or not np.all(np.isfinite(points))):
            raise ValueError("local points must have finite shape (K, 2)")
        R, _, _, center_long = self._frame(theta)
        ld = np.longdouble
        eps = np.finfo(ld).eps

        # Four rounded operations bound each component of mu - R nu.
        gamma4 = (ld(4.0) * eps) / (ld(1.0) - ld(4.0) * eps)
        products = (np.abs(np.asarray(R, dtype=ld))
                    @ np.abs(np.asarray(self.body.mean, dtype=ld)))
        center_error = gamma4 * (
            np.abs(np.asarray(self.scene.mean, dtype=ld)) + products)

        ideal_computed = (center_long[None, :]
                          + np.asarray(points, dtype=ld))
        world = np.asarray(ideal_computed, dtype=np.float64)
        cast_error = np.abs(np.asarray(world, dtype=ld) - ideal_computed)
        # One more long-double addition formed ``ideal_computed``.
        gamma1 = eps / (ld(1.0) - eps)
        add_error = gamma1 * (np.abs(center_long)[None, :]
                              + np.abs(np.asarray(points, dtype=ld)))
        component_error = cast_error + center_error[None, :] + add_error
        norms = np.sqrt(np.sum(component_error * component_error, axis=1))
        bound = float(np.max(norms, initial=ld(0.0)))
        return world, float(np.nextafter(bound, np.inf))

    def radius_bound(self) -> float:
        """theta-independent bound on the C-obstacle circumradius about its
        center: kappa*sqrt(lmax(Sigma)) + rho*sqrt(lmax(Lambda))."""
        return self.scene.bounding_radius() + self.body.bounding_radius()

    def inradius_bound(self) -> float:
        """Radius of a centred Euclidean ball provably contained in the pair."""
        scene_min = float(np.linalg.eigvalsh(
            self.scene.covariance)[0])
        body_min = float(np.linalg.eigvalsh(self.body.covariance)[0])
        return (self.scene.level * np.sqrt(scene_min)
                + self.body.level * np.sqrt(body_min))

    def theta_lipschitz(self) -> float:
        """Hausdorff rate of O(theta) in theta: |d center/d theta| <= |nu_j|
        plus body-support rotation rate <= rho*sqrt(lmax(Lambda))."""
        # Compute the 2x2 spectral expression in extended precision and round
        # the final rate outward.  A downward-rounded rate could turn a
        # near-contact rotation interval into a false certificate.
        ld = np.longdouble
        eps = np.finfo(ld).eps
        mean = np.asarray(self.body.mean, dtype=ld)
        cov = np.asarray(self.body.covariance, dtype=ld)
        mean_norm = np.sqrt(np.sum(mean * mean))
        spectral_gap = np.sqrt(
            (cov[0, 0] - cov[1, 1]) ** 2 + ld(4.0) * cov[0, 1] ** 2)
        lambda_max = ld(0.5) * (
            cov[0, 0] + cov[1, 1] + spectral_gap)
        body_radius = ld(self.body.level) * np.sqrt(lambda_max)
        raw = mean_norm + body_radius
        error = (ld(64.0) * eps
                 * max(ld(1.0), abs(mean_norm), abs(body_radius),
                       np.max(np.abs(cov))))
        exported = float(np.nextafter(raw + error, ld(np.inf)))
        return float(np.nextafter(exported, np.inf))

    def bump_revision(self):
        self.revision += 1
        self._theta_cache.clear()


def support_value(pair: PairOracle, theta: float, u: np.ndarray) -> float:
    return float(pair.support_values(theta, u[None])[0])


def support_point(pair: PairOracle, theta: float, u: np.ndarray) -> np.ndarray:
    return pair.support_points(theta, u[None])[0]


def build_oracles(scene, robot, ledger=None) -> list[PairOracle]:
    return [PairOracle(PairID(s.primitive_id, b.primitive_id), s, b,
                       ledger=ledger)
            for s in scene.supports for b in robot.supports]


def unit_dirs(angles: np.ndarray) -> np.ndarray:
    angles = np.asarray(angles, float)
    # ``cos``/``sin`` rows can miss unit norm by one ulp.  Return the same
    # canonical normalized rows that PairOracle will consume, so an envelope
    # never solves tangent intersections with directions that differ subtly
    # from the directions used to compute their support values.
    return _unit_rows(np.stack([np.cos(angles), np.sin(angles)], axis=1))
