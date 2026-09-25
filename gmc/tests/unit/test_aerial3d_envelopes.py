"""Certified pair envelopes (uav.md §6): direction set, outer/inner polytopes, sandwich audit, box tests."""
import numpy as np
import pytest

from gmc.aerial3d.envelopes import EnvelopeTable, direction_set, sandwich_audit
from gmc.aerial3d.pairs import PairSet

R, H, M, KAPPA = .25, .10, .05, 2.


def _rotation(seed):
    q, _ = np.linalg.qr(np.random.default_rng(seed).normal(size=(3, 3)))
    return q * np.sign(np.linalg.det(q))


def _unit(n, seed=0):
    v = np.random.default_rng(seed).normal(size=(n, 3))
    return v / np.linalg.norm(v, axis=1, keepdims=True)


def _pairs(means, sds, rot_seeds):
    covs = [(_rotation(s) if s is not None else np.eye(3)) @ np.diag(np.asarray(sd, float) ** 2)
            @ (_rotation(s) if s is not None else np.eye(3)).T for sd, s in zip(sds, rot_seeds)]
    return PairSet(np.asarray(means, float), np.asarray(covs), np.arange(len(means)),
                   level=KAPPA, radius=R, half_height=H, margin=M)


def _outside_depth(pairs, i, points, V):
    """max_v (v.x - h(v)) per point: > 0 proves the point is outside O_i."""
    return np.max(points @ V.T - pairs.support(V, [i])[0][None, :], axis=1)


@pytest.fixture(scope="module")
def zoo():
    # analytic sphere, random anisotropic, adversarial needle / pancake / near-degenerate
    return _pairs([(0, 0, 1), (1, .5, .7), (-1, 0, 1.2), (0, 1.5, .4), (2, -1, 1.)],
                  [(.1, .1, .1), (.3, .05, .12), (1e-4, 1.2, 1e-4), (.8, .8, 1e-3), (.4, 1e-5, 1e-5)],
                  [None, 1, 2, 3, 4])


def test_direction_set_is_half_sphere_with_axes_and_fine_cover():
    U = direction_set()
    assert U.shape == (81, 3)  # level-2 icosphere: 162 = 81 antipodal pairs, axes included
    assert np.allclose(np.linalg.norm(U, axis=1), 1., atol=1e-15)
    for e in np.eye(3):
        assert np.any(np.all(U == e, axis=1))
    full = np.vstack([U, -U])
    assert len(np.unique(full.round(12), axis=0)) == 162  # no antipodal duplicates
    worst = np.max(np.arccos(np.clip(np.max(_unit(20000, 3) @ full.T, axis=1), -1, 1)))
    assert np.degrees(worst) < 12.


def test_outer_polytope_contains_exact_support_points(zoo):
    table = EnvelopeTable(zoo)
    assert np.array_equal(table.rho, zoo.rho(table.U))
    V = _unit(3000, 7)
    for i in range(len(zoo)):
        A, b = table.outer_rows(i)
        X = zoo.support_points(V, [i])[0]  # points of the exact O_i
        assert np.max(X @ A.T - b[None, :]) <= 0.


def test_inner_polytope_is_inside_exact_and_excludes_outside_points(zoo):
    table = EnvelopeTable(zoo)
    V = _unit(20000, 8)
    for i in range(len(zoo)):
        eq = table.inner_equations(i)
        assert table.points_in_inner(i, zoo.means[i][None])[0]
        # a point just beyond the exact support in +x is outside the inner polytope
        beyond = zoo.means[i] + (zoo.support(np.eye(3)[0], [i])[0] - zoo.means[i][0] + 1e-3) * np.eye(3)[0]
        assert not table.points_in_inner(i, beyond[None])[0]
        # sampled points certified inside the inner polytope are inside O_i
        rng = np.random.default_rng(i)
        box = np.stack([zoo.aabb_lower[i], zoo.aabb_upper[i]])
        P = rng.uniform(box[0], box[1], size=(4000, 3))
        inside = table.points_in_inner(i, P)
        assert inside.any()
        assert np.max(_outside_depth(zoo, i, P[inside], V)) <= 1e-12
        assert eq.shape[1] == 4


def test_sandwich_audit_passes_on_analytic_random_and_adversarial_pairs(zoo):
    table = EnvelopeTable(zoo)
    audit = sandwich_audit(table, range(len(zoo)), n_dense=5000, seed=0)
    assert audit["passed"], audit
    assert audit["pairs_audited"] == len(zoo)
    for key in ("inner_vertices_outside_exact_max", "exact_points_outside_outer_max",
                "inner_vertices_outside_outer_max"):
        assert audit[key] <= 0.


def test_box_separation_gap_is_a_certified_disjointness_witness(zoo):
    table = EnvelopeTable(zoo)
    V = _unit(20000, 9)
    rng = np.random.default_rng(4)
    safe_seen = hit_seen = 0
    for _ in range(300):
        i = int(rng.integers(len(zoo)))
        c = zoo.means[i] + rng.uniform(-1.2, 1.2, 3)
        d = rng.uniform(.01, .15, 3)
        gap, k, sign = table.best_gap([i], c, d)
        pts = c + rng.uniform(-1, 1, size=(200, 3)) * d
        depth = _outside_depth(zoo, i, pts, V)
        if gap[0] > 0:
            safe_seen += 1
            assert np.min(depth) > 0  # every sampled box point lies outside O_i
            # the witness plane really separates: box max along n < O_i min along n
            n = sign[0] * table.U[k[0]]
            assert c @ n + np.abs(n) @ d < n @ zoo.means[i] - zoo.rho(n[None], [i])[0, 0]
        if np.min(depth) <= -1e-9:
            hit_seen += 1
            assert gap[0] < 0  # a box with a point inside O_i can never be certified
    assert safe_seen > 50 and hit_seen > 20


def test_direction_refinement_separates_where_fixed_directions_fail():
    # A long thin needle along a non-grid direction, box close alongside it.
    axis = np.array([1., .37, .21]); axis /= np.linalg.norm(axis)
    helper = np.cross(axis, [0, 0, 1.]); helper /= np.linalg.norm(helper)
    Q = np.column_stack([axis, helper, np.cross(axis, helper)])
    cov = Q @ np.diag([1.5, 1e-3, 1e-3]) ** 2 @ Q.T
    pairs = PairSet(np.zeros((1, 3)), cov[None], [0], level=KAPPA, radius=R, half_height=H, margin=M)
    table = EnvelopeTable(pairs)
    normal = helper  # box sits beside the needle, displaced along its normal
    reach = pairs.support(normal, [0])[0]
    c = normal * (reach + .02 + .01)
    d = np.full(3, .01 / np.sqrt(3))
    fixed, _, _ = table.best_gap([0], c, d)
    gap, u = table.refine_gap(0, c, d)
    assert gap > fixed[0]
    assert gap > 0
    assert abs(np.linalg.norm(u) - 1) < 1e-12


def test_box_in_inner_is_sound_and_depth_is_necessary(zoo):
    table = EnvelopeTable(zoo)
    V = _unit(20000, 10)
    rng = np.random.default_rng(5)
    inside_seen = 0
    for _ in range(300):
        i = int(rng.integers(len(zoo)))
        c = zoo.means[i] + rng.uniform(-.4, .4, 3)
        d = rng.uniform(.005, .08, 3)
        ok = table.box_in_inner(i, c, d)
        depth = table.depth([i], c, d)[0]
        if ok:
            inside_seen += 1
            assert depth >= 0  # inside inner implies inside outer
            corners = c + np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * d
            pts = np.vstack([corners, c + rng.uniform(-1, 1, size=(100, 3)) * d])
            assert np.max(_outside_depth(zoo, i, pts, V)) <= 1e-12
    assert inside_seen > 30


def test_covering_radius_and_refinement_bound_are_sound(zoo):
    """No direction beats best_fixed_gap + theta_cover * (|mu-c| + R_i + |d|)."""
    table = EnvelopeTable(zoo)
    full = np.vstack([table.U, -table.U])
    worst = np.max(np.arccos(np.clip(np.max(_unit(50000, 11) @ full.T, axis=1), -1, 1)))
    assert worst <= table.cover_angle <= np.radians(12.)
    # circumradius bounds every support point's distance from the mean
    X = zoo.support_points(_unit(5000, 12))
    assert np.all(np.linalg.norm(X - zoo.means[:, None], axis=2) <= table.circumradius[:, None])
    rng = np.random.default_rng(13)
    for _ in range(200):
        i = int(rng.integers(len(zoo)))
        c = zoo.means[i] + rng.uniform(-1., 1., 3)
        d = rng.uniform(.01, .2, 3)
        fixed, _, _ = table.best_gap([i], c, d)
        bound = table.refine_upper_bound([i], c, d, fixed)[0]
        refined, _ = table.refine_gap(i, c, d)
        V = _unit(20000, 14)
        dense = np.max(np.abs(V @ (zoo.means[i] - c)) - zoo.rho(V, [i])[0] - np.abs(V) @ d)
        assert max(refined, dense) <= bound + 1e-12
