"""T-Pair-03 (Guide §5.3): support-based envelopes vs an INDEPENDENT
collision oracle (SLSQP convex feasibility — no support code involved).

Sandwich consistency on random configurations:
  inside inner  => truly colliding   (no false obstacle from inner)
  truly colliding => inside outer    (outer has zero false negatives)
"""
import numpy as np
import shapely

from gmc.geometry.c_obstacle import pose_collides
from gmc.geometry.envelopes import approximate_pair
from gmc.geometry.support import PairOracle
from gmc.types import GaussianSupport2D, PairID, rotation2

RNG = np.random.default_rng(7)


def random_support(pid, scale=0.8):
    ang = RNG.uniform(0, np.pi)
    s1 = RNG.uniform(0.15, 0.8) * scale
    s2 = s1 / RNG.uniform(1.0, 6.0)
    R = rotation2(ang)
    return GaussianSupport2D(RNG.uniform(-1, 1, 2),
                             R @ np.diag([s1 ** 2, s2 ** 2]) @ R.T,
                             RNG.uniform(0.8, 2.0), pid)


class TestPairEquivalence:
    def test_sandwich_vs_independent_oracle(self, cfg):
        n_checked = 0
        for _ in range(8):
            o = PairOracle(PairID(0, 0), random_support(0), random_support(1))
            th = RNG.uniform(0, 2 * np.pi)
            sw = approximate_pair(o, th, cfg.pair_approx)
            for _ in range(25):
                t = o.center(th) + RNG.uniform(-1.5, 1.5, 2)
                pt = shapely.Point(t)
                in_inner = sw.inner.covers(pt)
                in_outer = sw.outer.covers(pt)
                truth = pose_collides(o.scene, o.body, t, th)
                if in_inner:
                    assert truth, f"inner point not colliding: {t} th={th}"
                if truth:
                    assert in_outer, f"colliding point outside outer: {t}"
                n_checked += 1
        assert n_checked == 200
