"""T1 tests (Guide §6.4): nesting, monotone refinement, no false-safe."""
import numpy as np
import shapely

import gmc.geometry.envelopes as envelope_module
from gmc.config import PairApproxCfg
from gmc.geometry.envelopes import (_convex_contains_vertices,
                                    _polygon_orientation, approximate_pair)
from gmc.geometry.support import PairOracle, unit_dirs
from gmc.types import CertStatus, GaussianSupport2D, PairID

from .test_support import RNG, random_oracle


def _hulls_support(poly, U):
    v = np.array(poly.exterior.coords[:-1])
    return np.max(U @ v.T, axis=1)


class TestNesting:
    def test_prototype_cannot_call_theorem_proof_gates(self, cfg,
                                                       monkeypatch):
        def forbidden(*_args, **_kwargs):
            raise AssertionError("prototype entered a theorem-only proof gate")

        monkeypatch.setattr(
            envelope_module, "_local_numeric_allowances", forbidden,
        )
        monkeypatch.setattr(
            envelope_module, "_outer_support_margin", forbidden,
        )
        monkeypatch.setattr(
            envelope_module, "_convex_contains_vertices", forbidden,
        )
        sandwich = approximate_pair(
            random_oracle(), 0.37, cfg.pair_approx,
        )
        assert sandwich.status is CertStatus.APPROX_UNCERTIFIED

    def test_exact_convex_audit_rejects_one_ulp_outward_tamper(self):
        outer = np.array([
            [-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0],
        ])
        boundary_inner = np.array([
            [0.0, 0.0], [1.0, 0.0], [0.0, 0.5],
        ])
        assert _convex_contains_vertices(outer, boundary_inner)
        # The audit has no geometric acceptance tolerance: crossing the exact
        # binary64 boundary by one representable step is a real violation.
        tampered = boundary_inner.copy()
        tampered[1, 0] = np.nextafter(1.0, np.inf)
        assert not _convex_contains_vertices(outer, tampered)
        # Polygon winding is not part of the proof contract.
        assert _convex_contains_vertices(outer[::-1], boundary_inner)

    def test_exact_convex_audit_is_stable_under_large_translation(self):
        # Direct world-coordinate shoelace products are O(1e20) here while
        # the square's signed area is O(1).  The floating filter must either
        # resolve the correct winding after recentering or use exact dyadics.
        base = 1.0e10
        ccw = np.array([
            [base, base],
            [base + 1.0, base],
            [base + 1.0, base + 1.0],
            [base, base + 1.0],
        ])
        inside = np.array([[base + 0.5, base + 0.5]])
        outside = np.array([[
            np.nextafter(base + 1.0, np.inf), base + 0.5,
        ]])

        assert _polygon_orientation(ccw) == 1
        assert _polygon_orientation(ccw[::-1]) == -1
        assert _convex_contains_vertices(ccw, inside)
        assert _convex_contains_vertices(ccw[::-1], inside)
        assert not _convex_contains_vertices(ccw, outside)
        assert not _convex_contains_vertices(ccw[::-1], outside)

    def test_locked_cross_platform_sandwich_is_exactly_nested(self):
        # On x86_64 Linux the previous longdouble-epsilon construction left a
        # -6e-17 edge margin and GEOS correctly rejected this pair.  arm64's
        # binary64 long double happened to pad it enough, hiding the bug.
        scene = GaussianSupport2D(
            [-1.7759108276441289, -0.0741700185239269],
            [[0.06592785289643381, 0.0032248564894160276],
             [0.0032248564894160276, 0.08782439292279344]],
            1.4938518932481657, 0,
        )
        body = GaussianSupport2D(
            [0.19351256127660843, 0.23952754733395376],
            [[0.5675141956552066, -0.07650943528674853],
             [-0.07650943528674853, 0.08600320477968897]],
            1.0408592564057868, 0,
        )
        oracle = PairOracle(PairID(0, 0), scene, body)
        sandwich = approximate_pair(
            oracle, -0.4700181754397601,
            PairApproxCfg(
                initial_directions=16, max_directions=256,
                eps_pair=4e-3, certificate_mode="theorem",
            ),
        )
        outer = np.asarray(sandwich.outer.exterior.coords[:-1])
        inner = np.asarray(sandwich.inner.exterior.coords[:-1])

        assert sandwich.status is CertStatus.CERTIFIED
        assert _convex_contains_vertices(outer, inner)
        # P3 still uses this as a diagnostic prerequisite; the constructive
        # padding should leave enough positive room for both predicates.
        assert sandwich.outer.covers(sandwich.inner)

    def test_inner_true_outer_support_ordering(self, cfg):
        for _ in range(20):
            o = random_oracle()
            th = RNG.uniform(0, 2 * np.pi)
            sw = approximate_pair(o, th, cfg.pair_approx)
            ang = RNG.uniform(0, 2 * np.pi, 256)
            U = unit_dirs(ang)
            h_true = o.support_values(th, U)
            assert np.all(_hulls_support(sw.inner, U) <= h_true + 1e-9)
            assert np.all(h_true <= _hulls_support(sw.outer, U) + 1e-9)

    def test_boundary_points_inside_outer(self, cfg):
        o = random_oracle()
        th = 0.6
        sw = approximate_pair(o, th, cfg.pair_approx)
        bd = o.support_points(th, unit_dirs(
            np.linspace(0, 2 * np.pi, 300, endpoint=False)))
        assert sw.outer.buffer(1e-9).contains(shapely.MultiPoint(bd))


class TestMonotoneRefinement:
    def test_adaptive_rounds_reuse_promoted_midpoint_supports(self):
        scene = GaussianSupport2D(
            [0.0, 0.0], [[0.0225, 0.0], [0.0, 0.0225]], 1.0, 0,
        )
        body = GaussianSupport2D(
            [0.0, 0.0], [[0.25, 0.0], [0.0, 0.04]], 1.0, 0,
        )
        oracle = PairOracle(PairID(0, 0), scene, body)
        cfg = PairApproxCfg(
            initial_directions=32,
            max_directions=128,
            eps_pair=1e-14,
            certificate_mode="prototype",
        )

        sandwich = approximate_pair(oracle, 0.2, cfg)

        assert len(sandwich.angles) == 128
        # Each final support point is evaluated once.  Values comprise the
        # final 128 directions plus their 128 last-round midpoints; midpoint
        # values promoted by earlier rounds are cache hits, not re-evaluated.
        assert oracle.point_calls == 128
        assert oracle.value_calls == 256
        assert sandwich.support_calls == 384

    def test_gap_decreases_with_directions(self, cfg_theorem):
        o = random_oracle()
        th = 1.0
        gaps = []
        from dataclasses import replace
        for max_dirs in (16, 32, 64, 128):
            pa = replace(cfg_theorem.pair_approx, eps_pair=1e-12,
                         max_directions=max_dirs)
            sw = approximate_pair(o, th, pa)
            gaps.append(sw.hausdorff_upper)
        assert all(g2 <= g1 + 1e-12 for g1, g2 in zip(gaps, gaps[1:]))
        areas_in, areas_out = [], []
        for max_dirs in (16, 64):
            pa = replace(cfg_theorem.pair_approx, eps_pair=1e-12,
                         max_directions=max_dirs)
            sw = approximate_pair(o, th, pa)
            areas_in.append(sw.inner.area)
            areas_out.append(sw.outer.area)
        assert areas_in[1] >= areas_in[0] - 1e-12    # inner grows
        assert areas_out[1] <= areas_out[0] + 1e-12  # outer shrinks


class TestNoFalseSafe:
    def test_random_colliding_points_never_outside_outer(self, cfg):
        """A point on/inside the true obstacle must be inside outer."""
        for _ in range(10):
            o = random_oracle()
            th = RNG.uniform(0, 2 * np.pi)
            sw = approximate_pair(o, th, cfg.pair_approx)
            interior = o.support_points(
                th, unit_dirs(RNG.uniform(0, 2 * np.pi, 50))) * 0.999 \
                + 0.001 * o.center(th)[None, :]
            for p in interior:
                assert sw.outer.buffer(1e-9).covers(shapely.Point(p))
