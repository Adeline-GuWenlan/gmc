"""GMC layer tests: convex kernels, M1 clustering soundness, two-level queries.

Run:  cd splatc_atlas && PYTHONPATH=src python -m pytest tests/test_gmc.py -q
"""
import numpy as np
import pytest

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import shapely
from shapely.geometry import LineString, Point, box as shapely_box

from splatc.gmc import (build_merged, circum_factor, coarse_forbidden,
                        ellipse_polys, forbidden_ellipse_polys, free_topology,
                        gate_query, minkowski_sum, raw_forbidden, robot_cov,
                        rot2, support_ellipses, tangent_polys, unit_dirs)

RNG = np.random.default_rng(20260825)
RHO = 2.0


def random_convex(n_pts, scale=1.0, center=(0, 0)):
    """Convex CCW polygon with exactly n_pts vertices: affine image of a
    regular n-gon (random SPD stretch + rotation)."""
    ang = np.linspace(0, 2 * np.pi, n_pts, endpoint=False)
    circle = np.stack([np.cos(ang), np.sin(ang)], axis=1)
    A = rot2(RNG.uniform(0, np.pi)) @ np.diag(RNG.uniform(0.3, 1.0, 2) * scale)
    return circle @ A.T + np.asarray(center, float)


def random_cov(aspect_max=20.0, scale=0.3):
    a = RNG.uniform(0, np.pi)
    s1 = RNG.uniform(0.05, 1.0) * scale
    s2 = s1 / RNG.uniform(1.0, aspect_max)
    R = rot2(a)
    return R @ np.diag([s1 ** 2, s2 ** 2]) @ R.T


class TestMinkowski:
    def test_square_analytic(self):
        sq = lambda s: np.array([[0, 0], [s, 0], [s, s], [0, s]], float)
        out = minkowski_sum(sq(1.0)[None], sq(2.0))[0]
        poly = shapely.Polygon(out)
        assert poly.is_valid
        assert poly.area == pytest.approx(9.0, abs=1e-12)
        assert poly.equals(shapely_box(0, 0, 3, 3))

    def test_vs_bruteforce_hull(self):
        for _ in range(25):
            P = random_convex(RNG.integers(3, 12))
            Q = random_convex(RNG.integers(3, 12), scale=0.7, center=(1.3, -0.4))
            got = shapely.Polygon(minkowski_sum(P[None], Q)[0])
            ref = shapely.MultiPoint(
                (P[:, None, :] + Q[None, :, :]).reshape(-1, 2)).convex_hull
            assert got.is_valid
            assert got.symmetric_difference(ref).area < 1e-9 * max(ref.area, 1)

    def test_batched_matches_single(self):
        P = np.stack([random_convex(6), random_convex(6, center=(2, 2))])
        Q = random_convex(5)
        both = minkowski_sum(P, Q)
        for i in range(2):
            one = minkowski_sum(P[i][None], Q)[0]
            assert shapely.Polygon(both[i]).equals(shapely.Polygon(one))


class TestEllipsePolys:
    def test_circumscribed_contains_inscribed_inside(self):
        for _ in range(10):
            mu = RNG.normal(size=(1, 2))
            S = random_cov()[None]
            ins = shapely.Polygon(ellipse_polys(mu, S, RHO, 24, factor=1.0)[0])
            cir = shapely.Polygon(ellipse_polys(mu, S, RHO, 24,
                                                factor=circum_factor(24))[0])
            bd = ellipse_polys(mu, S, RHO, 173, factor=1.0)[0]  # dense boundary
            assert cir.buffer(1e-9).contains(shapely.MultiPoint(bd))
            assert ins.area < cir.area

    def test_forbidden_matches_mahalanobis(self):
        mu = np.array([[0.3, -0.2]])
        S = random_cov()[None]
        lam0 = np.diag([0.36, 0.0225])
        th = 0.7
        A = S[0] + robot_cov(th, lam0)
        ins = shapely.Polygon(forbidden_ellipse_polys(mu, S, th, lam0, RHO, 48)[0])
        pts = RNG.normal(size=(400, 2)) * 1.5
        inv = np.linalg.inv(A)
        mah = np.einsum("ni,ij,nj->n", pts - mu[0], inv, pts - mu[0])
        for p, m in zip(pts, mah):
            if m < (RHO * 0.97) ** 2:
                assert ins.covers(Point(p))
            elif m > (RHO * 1.03) ** 2:
                assert not ins.covers(Point(p))

    def test_degenerate_thin_cov_no_crash(self):
        S = np.array([[[1.0, 0.0], [0.0, 1e-14]]])
        polys = ellipse_polys(np.zeros((1, 2)), S, RHO, 24)
        assert np.isfinite(polys).all()


class TestClusterSoundness:
    def _scene(self, n=150):
        mu = RNG.uniform(-3, 3, size=(n, 2))
        S = np.stack([random_cov() for _ in range(n)])
        return mu, S

    def test_tangent_poly_contains_members(self):
        mu, S = self._scene(40)
        ms = build_merged(mu, S, rho=RHO)
        geoms = shapely.polygons(ms.polys)
        bd = ellipse_polys(mu, S, RHO, 96, factor=1.0)
        for bi, idx in enumerate(ms.buckets):
            g = geoms[bi].buffer(1e-9)
            for i in idx:
                assert g.contains(shapely.MultiPoint(bd[i])), \
                    f"bucket {bi} does not cover member {i}"

    def test_merged_cover_is_conservative_all_theta(self):
        mu, S = self._scene(120)
        ms = build_merged(mu, S, rho=RHO)
        lam0 = np.diag([0.36, 0.0225])
        for th in np.linspace(0, np.pi, 7, endpoint=False):
            raw = shapely.union_all(list(raw_forbidden(mu, S, th, lam0, RHO)))
            merged = shapely.union_all(list(coarse_forbidden(ms, th, lam0)))
            leak = raw.difference(merged).area
            assert leak < 1e-9, f"leak {leak} at theta={th}"

    def test_every_splat_in_exactly_one_bucket(self):
        mu, S = self._scene(80)
        ms = build_merged(mu, S)
        all_idx = np.concatenate(ms.buckets)
        assert len(all_idx) == 80
        assert len(np.unique(all_idx)) == 80


class TestTwoLevelGate:
    def _door_scene(self, gap=2.0, n_per_wall=60):
        """Vertical wall along x=0 with a gap around y=0."""
        ys = np.concatenate([np.linspace(-4, -gap / 2, n_per_wall),
                             np.linspace(gap / 2, 4, n_per_wall)])
        mu = np.stack([np.zeros_like(ys), ys], axis=1)
        S = np.tile(np.diag([0.03 ** 2, 0.08 ** 2])[None], (len(ys), 1, 1))
        return mu, S

    def test_adaptive_matches_raw_all_theta(self):
        mu, S = self._door_scene()
        ms = build_merged(mu, S, rho=RHO)
        lam0 = np.diag([0.55 ** 2, 0.12 ** 2])   # support 2.2 x 0.48 > gap 2.0
        ws = shapely_box(-3, -4.5, 3, 4.5)
        probes = [LineString([(-2.4, -0.3), (-1.2, 0.3)]),
                  LineString([(1.2, -0.3), (2.4, 0.3)])]
        opens_ref, opens_two = [], []
        for th in np.linspace(0, np.pi, 36, endpoint=False):
            _, gate_raw, _ = free_topology(
                list(raw_forbidden(mu, S, th, lam0, RHO)), ws, probes)
            res = gate_query(ms, th, lam0, ws, probes)
            opens_ref.append(bool(gate_raw))
            opens_two.append(res["verdict"] == "open")
        assert opens_two == opens_ref
        assert any(opens_ref) and not all(opens_ref)  # scene exercises both

    def test_open_is_certified_never_false(self):
        """Two-level OPEN must never contradict raw CLOSED (soundness dir)."""
        mu = RNG.uniform(-2, 2, size=(60, 2))
        S = np.stack([random_cov(scale=0.4) for _ in range(60)])
        ms = build_merged(mu, S, rho=RHO)
        lam0 = np.diag([0.3 ** 2, 0.1 ** 2])
        ws = shapely_box(-3, -3, 3, 3)
        probes = [LineString([(-2.7, -2.7), (-2.7, -2.2)]),
                  LineString([(2.7, 2.2), (2.7, 2.7)])]
        for th in np.linspace(0, np.pi, 9, endpoint=False):
            res = gate_query(ms, th, lam0, ws, probes)
            if res["verdict"] == "open":
                _, gate_raw, _ = free_topology(
                    list(raw_forbidden(mu, S, th, lam0, RHO)), ws, probes)
                assert gate_raw, "two-level OPEN contradicted raw CLOSED"

    def test_probe_segments_survive_swallowing(self):
        """A probe segment partially covered by an obstacle still registers."""
        mu, S = self._door_scene(gap=3.0)
        ms = build_merged(mu, S, rho=RHO)
        lam0 = np.diag([0.2 ** 2, 0.1 ** 2])
        ws = shapely_box(-3, -4.5, 3, 4.5)
        blocked = LineString([(0.0, 2.5), (-2.0, 0.0)])   # crosses the wall
        free_line = LineString([(1.2, -0.3), (2.4, 0.3)])
        _, gate_open, _ = free_topology(
            list(raw_forbidden(mu, S, 0.0, lam0, RHO)), ws,
            [blocked, free_line])
        assert gate_open  # both segments still touch the connected free side


class TestSlice:
    """Hard-support fixed-theta slice (splatc.gmc.slice) on bench primitives."""

    def _bench(self):
        from splatc.datasets.g1_gate import make_g1_scene, robot_library
        return make_g1_scene(0.9), robot_library()["R_long_ellipse"]

    def test_witnesses_certified_on_frozen_bench(self):
        from splatc.gmc.slice import build_slice, certify_witnesses
        scene, robot = self._bench()
        for th in np.linspace(0, np.pi, 8, endpoint=False):
            sl = build_slice(scene, robot, th, ndir=48, side="outer")
            ok, margins = certify_witnesses(sl, scene, robot)
            assert ok, f"witness failed at theta={th}"
            assert (margins > 0).all()

    def test_workspace_body_containment_semantics(self):
        """Regression for the keyhole theta=90 finding: the slice must erode
        the workspace by the robot support (bench eval_points semantics), or
        zero-width wrap-around passages appear along workspace edges."""
        from splatc.datasets.g1_gate import robot_library
        from splatc.gaussian_geometry.primitives import SceneGeometry
        from splatc.gmc.slice import build_slice
        from scipy import ndimage
        ys = np.linspace(-0.8, 0.8, 41)
        wallpts = np.stack([np.zeros_like(ys), ys], axis=1)
        scene = SceneGeometry("edge_wall", (-2.0, 2.0, -1.2, 1.2),
                              [(0.10, wallpts)])
        robot = robot_library()["R_long_ellipse"]
        th = np.pi / 2                       # vertical robot, py = a = 0.6
        sl = build_slice(scene, robot, th, ndir=48, side="outer")
        assert sl.n_components == 2          # center-only box would give 1
        X, Y = np.meshgrid(np.linspace(-2, 2, 161), np.linspace(-1.2, 1.2, 97))
        f, _ = scene.eval_points(robot, th, X, Y)
        _, n_o = ndimage.label(np.asarray(f).reshape(X.shape))
        assert n_o == 2

    def test_gate_sandwich_vs_analytic(self):
        from splatc.datasets.g1_gate import projection_radius
        from splatc.gmc.slice import build_slice, probes_connected
        scene, robot = self._bench()
        probes = [LineString([(-2.0, -0.3), (-2.0, 0.3)]),
                  LineString([(2.0, -0.3), (2.0, 0.3)])]
        for th in np.linspace(0, np.pi, 45, endpoint=False):
            truth = projection_radius(robot.a, robot.b, th) < 0.45
            o = probes_connected(build_slice(scene, robot, th, 48, "outer"), probes)
            i = probes_connected(build_slice(scene, robot, th, 48, "inner"), probes)
            assert not (o and not truth), f"outer-open vs truth-closed at {th}"
            assert not (truth and not i), f"truth-open vs inner-closed at {th}"


class TestEvents:
    """G2 certified gate-interval finder (splatc.gmc.events)."""

    def _setup(self):
        from splatc.datasets.g1_gate import make_g1_scene, robot_library
        probes = [LineString([(-2.0, -0.3), (-2.0, 0.3)]),
                  LineString([(2.0, -0.3), (2.0, 0.3)])]
        return make_g1_scene(0.9), robot_library()["R_long_ellipse"], probes

    def test_two_sided_brackets_contain_analytic_events(self):
        from splatc.datasets.g1_gate import gate_half_angle
        from splatc.gmc.events import certified_gate_intervals
        scene, robot, probes = self._setup()
        half = gate_half_angle(robot.a, robot.b, 0.9)
        tol = np.deg2rad(0.5)
        rep_o = certified_gate_intervals(scene, robot, probes, tol, side="outer")
        rep_i = certified_gate_intervals(scene, robot, probes, tol, side="inner")
        assert len(rep_o.brackets) == 2 and len(rep_i.brackets) == 2
        for t in (half, np.pi - half):
            lo = min(min(a for a, b, *_ in r.brackets if abs((a + b) / 2 - t) < 0.1)
                     for r in (rep_o, rep_i))
            hi = max(max(b for a, b, *_ in r.brackets if abs((a + b) / 2 - t) < 0.1)
                     for r in (rep_o, rep_i))
            assert lo <= t <= hi, f"analytic event {t} outside sandwich"

    def test_coverage_is_complete(self):
        from splatc.gmc.events import certified_gate_intervals
        scene, robot, probes = self._setup()
        rep = certified_gate_intervals(scene, robot, probes, np.deg2rad(0.5))
        pieces = ([(a, b) for a, b, _ in rep.intervals]
                  + [(a, b) for a, b, *_ in rep.brackets]
                  + list(rep.unresolved))
        pieces.sort()
        cursor = 0.0
        for a, b in pieces:
            assert a <= cursor + 1e-9, f"coverage gap before {a}"
            cursor = max(cursor, b)
        assert cursor >= np.pi - 1e-9

    def test_certified_interval_verdicts_hold(self):
        from splatc.gmc.events import certified_gate_intervals
        from splatc.gmc.slice import build_slice, probes_connected
        scene, robot, probes = self._setup()
        rep = certified_gate_intervals(scene, robot, probes, np.deg2rad(0.5))
        rng = np.random.default_rng(7)
        checked = 0
        for a, b, v in rep.intervals:
            if b - a < 1e-6 or checked >= 12:
                continue
            th = rng.uniform(a, b)
            sl = build_slice(scene, robot, th, 48, "outer")
            assert probes_connected(sl, probes) == v, \
                f"certificate violated inside ({a},{b}) at {th}"
            checked += 1
        assert checked >= 6
