"""Lemma 2 evidence: certified Hausdorff bound is a true upper bound and
the sandwich orders support functions everywhere (dense audit)."""
import numpy as np

from gmc.geometry.envelopes import approximate_pair
from gmc.geometry.support import unit_dirs

from ..unit.test_support import RNG, random_oracle


class TestCertifiedBound:
    def test_bound_dominates_dense_audit(self, cfg_theorem):
        for _ in range(10):
            o = random_oracle()
            th = RNG.uniform(0, 2 * np.pi)
            sw = approximate_pair(o, th, cfg_theorem.pair_approx)
            ang = RNG.uniform(0, 2 * np.pi, 800)
            U = unit_dirs(ang)
            h_true = o.support_values(th, U)
            vin = np.array(sw.inner.exterior.coords[:-1])
            vout = np.array(sw.outer.exterior.coords[:-1])
            h_in = np.max(U @ vin.T, axis=1)
            h_out = np.max(U @ vout.T, axis=1)
            excess = float(np.max(h_out - h_true))
            deficit = float(np.max(h_true - h_in))
            assert excess <= sw.hausdorff_upper + 1e-9
            assert deficit <= sw.hausdorff_upper + 1e-9
            assert np.all(h_in <= h_true + 1e-9)
            assert np.all(h_true <= h_out + 1e-9)

    def test_certified_status_reached(self, cfg_theorem):
        """Deterministic oracle: collection-order-independent (the shared
        module RNG made this flaky across pytest invocation orders — caught
        on the HPC run of the unit+theorem subset)."""
        from gmc.geometry.support import PairOracle
        from gmc.types import CertStatus, GaussianSupport2D, PairID
        rng = np.random.default_rng(4242)
        from ..unit.test_support import random_support
        import numpy as _np
        scene = GaussianSupport2D(rng.uniform(-2, 2, 2),
                                  _np.diag([0.09, 0.02]), 1.5, 0)
        body = GaussianSupport2D(rng.uniform(-0.3, 0.3, 2),
                                 _np.diag([0.25, 0.04]), 1.0, 1)
        o = PairOracle(PairID(0, 0), scene, body)
        sw = approximate_pair(o, 0.4, cfg_theorem.pair_approx)
        assert sw.status is CertStatus.CERTIFIED
        assert sw.hausdorff_upper <= cfg_theorem.pair_approx.eps_pair
