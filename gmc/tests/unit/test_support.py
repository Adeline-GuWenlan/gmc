"""T0 tests (Guide §5.3): T-Pair-01/02/04."""
import numpy as np
import pytest

from gmc.geometry.support import PairOracle, support_point, support_value
from gmc.types import GaussianSupport2D, PairID, rotation2

RNG = np.random.default_rng(20260826)


def random_support(pid=0, scale=1.0, aspect_max=10.0):
    ang = RNG.uniform(0, np.pi)
    s1 = RNG.uniform(0.1, 1.0) * scale
    s2 = s1 / RNG.uniform(1.0, aspect_max)
    R = rotation2(ang)
    cov = R @ np.diag([s1 ** 2, s2 ** 2]) @ R.T
    return GaussianSupport2D(mean=RNG.uniform(-2, 2, 2), covariance=cov,
                             level=RNG.uniform(0.5, 2.5), primitive_id=pid)


def random_oracle():
    return PairOracle(PairID(0, 0), random_support(0), random_support(1))


class TestTPair01:
    def test_support_point_achieves_support_value(self):
        for _ in range(200):
            o = random_oracle()
            th = RNG.uniform(0, 2 * np.pi)
            u = RNG.normal(size=2)
            u /= np.linalg.norm(u)
            h = support_value(o, th, u)
            p = support_point(o, th, u)
            assert abs(u @ p - h) < 1e-10


class TestTPair02:
    def test_translation_equivariance(self):
        o = random_oracle()
        d = np.array([0.7, -1.3])
        shifted = PairOracle(o.pair_id,
                             GaussianSupport2D(o.scene.mean + d,
                                               o.scene.covariance,
                                               o.scene.level, 0),
                             o.body)
        th, u = 0.9, np.array([0.6, 0.8])
        assert support_value(shifted, th, u) == pytest.approx(
            support_value(o, th, u) + u @ d, abs=1e-12)

    def test_rotation_equivariance(self):
        """Rotating the WORLD by phi: scene mean/cov rotate, body heading
        adds phi, directions rotate — support value is invariant."""
        o = random_oracle()
        phi = 0.77
        Rp = rotation2(phi)
        rot_scene = GaussianSupport2D(Rp @ o.scene.mean,
                                      Rp @ o.scene.covariance @ Rp.T,
                                      o.scene.level, 0)
        o2 = PairOracle(o.pair_id, rot_scene, o.body)
        th = 0.4
        u = np.array([1.0, 0.0])
        h1 = support_value(o, th, u)
        h2 = support_value(o2, th + phi, Rp @ u)
        assert h2 == pytest.approx(h1, abs=1e-12)


class TestTPair04:
    def test_high_condition_covariance_no_nan(self):
        cov = np.diag([1.0, 1e-12])
        s = GaussianSupport2D(np.zeros(2), cov, 1.0, 0)
        o = PairOracle(PairID(0, 0), s, random_support(1))
        U = np.stack([np.cos(np.linspace(0, 2 * np.pi, 64, endpoint=False)),
                      np.sin(np.linspace(0, 2 * np.pi, 64, endpoint=False))], 1)
        h = o.support_values(0.3, U)
        p = o.support_points(0.3, U)
        assert np.isfinite(h).all() and np.isfinite(p).all()

    def test_batch_matches_scalar(self):
        o = random_oracle()
        th = 1.1
        U = RNG.normal(size=(32, 2))
        U /= np.linalg.norm(U, axis=1, keepdims=True)
        hv = o.support_values(th, U)
        for u, h in zip(U, hv):
            assert support_value(o, th, u) == pytest.approx(float(h), abs=1e-12)
