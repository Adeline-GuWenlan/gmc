"""Coreset gates C0 (cut validity) and C1 (macro sandwich).

These are the compression gates the v2 design revision places *before* the
existing compiler gates T0-T4.  C1 is the one that carries the soundness of the
whole layer: if union(members) is not inside E_plus, coarsening can delete an
obstacle and the compiler will happily certify a path straight through it.
"""
import numpy as np
import pytest
from shapely.ops import unary_union

from gmc.coreset.coarsen import CoarsenParams, coarsen_scene, uniform_cut
from gmc.coreset.hierarchy import build_hierarchy, iter_nodes, node_features
from gmc.coreset.macro import (certify_outer_containment, ellipse_polygon,
                               outer_ellipse, shape_matrix, union_support_values)
from gmc.io.robot_io import ellipse_robot
from gmc.synth import keyhole, single_door
from gmc.types import GaussianSupport2D, SceneModel2D

ROBOT = ellipse_robot(0.50, 0.20)


def _scene():
    return single_door(width=0.6)


def _random_group(rng, n, spread=1.0):
    out = []
    for k in range(n):
        angle = rng.uniform(0.0, np.pi)
        semi = rng.uniform(0.02, 0.30, size=2)
        R = np.array([[np.cos(angle), -np.sin(angle)],
                      [np.sin(angle), np.cos(angle)]])
        cov = R @ np.diag(semi ** 2) @ R.T
        out.append(GaussianSupport2D(mean=rng.uniform(-spread, spread, size=2),
                                     covariance=cov, level=1.0,
                                     primitive_id=k))
    return out


# ---------------------------------------------------------------- hierarchy

def test_hierarchy_is_deterministic():
    scene = _scene()
    a = [(n.node_id, n.members) for n in iter_nodes(build_hierarchy(scene))]
    b = [(n.node_id, n.members) for n in iter_nodes(build_hierarchy(scene))]
    assert a == b


def test_hierarchy_leaves_partition_the_scene():
    scene = _scene()
    root = build_hierarchy(scene)
    leaves = [n for n in iter_nodes(root) if n.is_leaf]
    covered = [i for n in leaves for i in n.members]
    assert sorted(covered) == list(range(len(scene.supports)))
    assert len(covered) == len(set(covered))


def test_parent_members_are_the_union_of_children():
    root = build_hierarchy(_scene())
    for node in iter_nodes(root):
        if node.is_leaf:
            continue
        merged = sorted(i for c in node.children for i in c.members)
        assert merged == sorted(node.members)


def test_bounding_disc_covers_every_member_ellipse():
    scene = _scene()
    for node in iter_nodes(build_hierarchy(scene)):
        for i in node.members:
            s = scene.supports[i]
            reach = np.linalg.norm(s.mean - node.center) + s.bounding_radius()
            assert reach <= node.radius + 1e-12


def test_node_features_are_finite_and_robot_normalised():
    scene = _scene()
    root = build_hierarchy(scene)
    feats = node_features(scene, root, robot_scale=0.2)
    assert feats["count"] == len(scene.supports)
    assert all(np.all(np.isfinite(v)) for v in feats.values()
               if isinstance(v, (int, float, list)))


# ------------------------------------------------------------------ C0 cut

@pytest.mark.parametrize("eps", [0.01, 0.05, 0.2])
def test_c0_cut_is_legal(eps):
    """Every leaf covered exactly once; no ancestor and descendant both in."""
    scene = _scene()
    res = coarsen_scene(scene, ROBOT, CoarsenParams(clearance_tol=eps))
    covered = [i for members in res.cut_members for i in members]
    assert sorted(covered) == list(range(len(scene.supports)))
    assert len(covered) == len(set(covered))
    sets = [set(m) for m in res.cut_members]
    for i, a in enumerate(sets):
        for j, b in enumerate(sets):
            if i != j:
                assert not (a <= b) and not (b <= a)


def test_c0_finest_cut_reproduces_the_scene():
    """The leaf cut must be a deterministic fallback to the original scene."""
    scene = _scene()
    res = uniform_cut(scene, len(scene.supports))
    assert res.n_macro == len(scene.supports)
    for macro, original in zip(res.macros, scene.supports):
        assert macro.n_members == 1
    orig = unary_union([ellipse_polygon(s, 128, outer=False)
                        for s in scene.supports])
    coarse = unary_union([ellipse_polygon(s, 128, outer=False)
                          for s in res.scene.supports])
    assert coarse.symmetric_difference(orig).area < 1e-9


# -------------------------------------------------------------- C1 sandwich

@pytest.mark.parametrize("seed", range(12))
def test_c1_macro_contains_union_dense_directions(seed):
    rng = np.random.default_rng(seed)
    members = _random_group(rng, int(rng.integers(2, 12)))
    macro = outer_ellipse(members, primitive_id=0)
    assert macro.certified
    assert certify_outer_containment(members, macro.support, 8192) >= 0.0


@pytest.mark.parametrize("seed", range(8))
def test_c1_macro_contains_member_interiors_pointwise(seed):
    """Direction sampling can miss; also check explicit interior points."""
    rng = np.random.default_rng(1000 + seed)
    members = _random_group(rng, int(rng.integers(2, 8)))
    macro = outer_ellipse(members, primitive_id=0)
    S_inv = np.linalg.inv(shape_matrix(macro.support))
    for m in members:
        L = np.linalg.cholesky(shape_matrix(m))
        z = rng.normal(size=(4000, 2))
        z /= np.maximum(np.linalg.norm(z, axis=1, keepdims=True), 1e-300)
        z *= rng.uniform(0.0, 1.0, size=(4000, 1)) ** 0.5
        pts = m.mean + z @ L.T
        d = pts - macro.support.mean
        assert np.max(np.einsum("ki,ij,kj->k", d, S_inv, d)) <= 1.0 + 1e-9


def test_c1_survives_a_deliberately_loose_mvee():
    """Soundness must come from certify+inflate, not from MVEE convergence."""
    from gmc.coreset import macro as macro_mod
    rng = np.random.default_rng(7)
    members = _random_group(rng, 9)
    original = macro_mod._mvee
    try:
        macro_mod._mvee = lambda pts, tol=1e-9, max_iter=400: original(
            pts, tol=1e-3, max_iter=1)
        macro = outer_ellipse(members, primitive_id=0)
    finally:
        macro_mod._mvee = original
    assert macro.certified
    assert certify_outer_containment(members, macro.support, 8192) >= 0.0


def test_c1_handles_a_collinear_group():
    members = [GaussianSupport2D(mean=np.array([0.1 * k, 0.0]),
                                 covariance=np.eye(2) * 1e-4, level=1.0,
                                 primitive_id=k) for k in range(9)]
    macro = outer_ellipse(members, primitive_id=0)
    assert macro.certified
    eig = np.linalg.eigvalsh(shape_matrix(macro.support))
    assert eig[0] > 0.0


def test_c1_single_member_is_the_identity():
    rng = np.random.default_rng(3)
    only = _random_group(rng, 1)[0]
    macro = outer_ellipse([only], primitive_id=5)
    assert macro.n_members == 1
    assert np.allclose(macro.support.mean, only.mean)
    assert np.allclose(macro.support.covariance, only.covariance)


def test_certification_rejects_a_shrunken_ellipse():
    """The certificate must be able to fail, or it proves nothing."""
    rng = np.random.default_rng(11)
    members = _random_group(rng, 6)
    macro = outer_ellipse(members, primitive_id=0)
    shrunk = GaussianSupport2D(macro.support.mean,
                               macro.support.covariance * 0.64,
                               macro.support.level, 0)
    assert certify_outer_containment(members, shrunk, 4096) < 0.0


# ------------------------------------------------- C2 geometric monotonicity

@pytest.mark.parametrize("family", ["door", "keyhole"])
def test_c2_coarse_obstacles_cover_the_original_obstacles(family):
    """Free space may only shrink -- this is what keeps SAFE sound."""
    scene = single_door(width=0.6) if family == "door" else keyhole(0.7)
    res = coarsen_scene(scene, ROBOT, CoarsenParams(clearance_tol=0.05))
    orig = unary_union([ellipse_polygon(s, 96, outer=False)
                        for s in scene.supports])
    coarse = unary_union([ellipse_polygon(s, 96, outer=True)
                          for s in res.scene.supports])
    assert orig.difference(coarse).area < 1e-9


def test_smaller_tolerance_never_compresses_more():
    """Monotone in eps at a fixed passage tolerance."""
    scene = _scene()
    root = build_hierarchy(scene)
    counts = [coarsen_scene(scene, ROBOT,
                            CoarsenParams(clearance_tol=e, passage_tol=0.004),
                            hierarchy=root).n_macro
              for e in (0.002, 0.01, 0.04)]
    assert counts[0] >= counts[1] >= counts[2]


def test_critical_radius_locates_the_door():
    """The persistence bisection must find the door half-width, not a grid
    point near it.  For a door of width w the critical erosion radius is w/2."""
    scene = single_door(width=0.6)
    res = coarsen_scene(scene, ROBOT,
                        CoarsenParams(clearance_tol=0.002, passage_tol=0.004))
    assert res.critical_radii, "no critical radius found"
    assert min(abs(r - 0.30) for r in res.critical_radii) < 0.01


def test_passage_test_can_be_disabled():
    scene = _scene()
    root = build_hierarchy(scene)
    on = coarsen_scene(scene, ROBOT,
                       CoarsenParams(clearance_tol=0.01, passage_tol=0.004),
                       hierarchy=root)
    off = coarsen_scene(scene, ROBOT,
                        CoarsenParams(clearance_tol=0.01, passage_tol=0.0),
                        hierarchy=root)
    assert not on.critical_radii or on.n_macro <= off.n_macro


def _two_door_scene(w_wide=0.9, w_narrow=0.45, wall_t=0.6):
    """Thick wall with a wide door at y=+1 and a narrow one at y=-1."""
    from shapely.geometry import box as shapely_box
    from gmc.synth import DISC_R, SPACING, _disc
    half_t = wall_t / 2.0
    xs = np.arange(-half_t + DISC_R, half_t - DISC_R + 1e-9, SPACING)
    bands = [(-3.0, -1.0 - w_narrow / 2.0 - DISC_R),
             (-1.0 + w_narrow / 2.0 + DISC_R, 1.0 - w_wide / 2.0 - DISC_R),
             (1.0 + w_wide / 2.0 + DISC_R, 3.0)]
    raw = []
    for y0, y1 in bands:
        for y in np.arange(y0, y1 + 1e-9, SPACING):
            for x in xs:
                raw.append(_disc((x, y), DISC_R, len(raw)))
    supports = tuple(GaussianSupport2D(d.mean, d.covariance, d.level, i)
                     for i, d in enumerate(raw))
    return SceneModel2D(supports=supports,
                        workspace=shapely_box(-3.0, -2.5, 3.0, 2.5),
                        name="two_door")


def test_robot_conditioning_changes_the_cut_on_a_multi_scale_scene():
    """Conditioning is only observable when passages exist at different
    scales.  On single_door every robot negotiates the same door edges, so
    identical cuts there are a property of the scene, not evidence about
    conditioning -- hence the two-door scene."""
    scene = _two_door_scene()
    root = build_hierarchy(scene)
    params = CoarsenParams(clearance_tol=0.002, target_gate_tol=0.02)
    small = coarsen_scene(scene, ellipse_robot(0.20, 0.10), params,
                          hierarchy=root)
    large = coarsen_scene(scene, ellipse_robot(0.40, 0.25), params,
                          hierarchy=root)
    assert set(map(frozenset, small.cut_members)) \
        != set(map(frozenset, large.cut_members))


# ------------------------------------------------- gate-sensitivity tuning

def test_gate_sensitivity_matches_the_analytic_derivative():
    """dtheta/dr must agree with a finite difference of the closed-form gate."""
    from gmc.coreset.coarsen import gate_sensitivity
    from gmc.synth import gate_half_angle
    a, b, r = 0.50, 0.20, 0.30
    analytic = gate_sensitivity(ellipse_robot(a, b), r)
    h = 1e-6
    fd = (gate_half_angle(a, b, 2.0 * (r + h))
          - gate_half_angle(a, b, 2.0 * (r - h))) / (2.0 * h)
    assert analytic == pytest.approx(fd, rel=1e-4)


def test_gate_sensitivity_abstains_outside_the_gate_band():
    from gmc.coreset.coarsen import gate_sensitivity
    robot = ellipse_robot(0.50, 0.20)
    assert gate_sensitivity(robot, 0.10) is None   # below b: never fits
    assert gate_sensitivity(robot, 0.90) is None   # above a: always fits


def test_near_circular_robot_needs_a_tighter_radius_tolerance():
    """The whole point of the conversion: same scene, 4x the sensitivity."""
    from gmc.coreset.coarsen import gate_sensitivity
    wide = gate_sensitivity(ellipse_robot(0.50, 0.20), 0.30)
    fat = gate_sensitivity(ellipse_robot(0.35, 0.28), 0.30)
    assert fat > 4.0 * wide


@pytest.mark.parametrize("axes", [(0.50, 0.20), (0.40, 0.15), (0.35, 0.28)])
def test_target_gate_tol_is_honoured(axes):
    """Contract: the certified gate error must stay within the requested
    budget for every robot at one shared setting."""
    import sys
    sys.path.insert(0, "experiments")
    from coreset_pareto import gate_threshold
    from gmc.config import load_config
    from gmc.synth import gate_half_angle

    a, b = axes
    target = 0.02
    scene = single_door(width=0.6)
    robot = ellipse_robot(a, b)
    res = coarsen_scene(scene, robot,
                        CoarsenParams(clearance_tol=0.002,
                                      target_gate_tol=target))
    got = gate_threshold(res.scene, robot, load_config("configs/toy.yaml"))
    err = got["threshold"] - gate_half_angle(a, b, 0.6)
    assert err <= 1e-4, "gate must never open wider than the truth"
    assert abs(err) <= target
    assert res.n_macro < len(scene.supports) / 4


# ----------------------------------------------------- inner (possible) side

def _inside_any(point, members) -> bool:
    """Analytic membership: is `point` inside at least one member ellipse?"""
    for m in members:
        d = np.asarray(point, float) - m.mean
        S = shape_matrix(m)
        if float(d @ np.linalg.solve(S, d)) <= 1.0 + 1e-12:
            return True
    return False


@pytest.mark.parametrize("seed", range(6))
def test_inner_macro_is_contained_pointwise(seed):
    """E_minus subset union(members), checked analytically rather than through
    polygons -- the polygon test inside inner_ellipse is conservative, but this
    is the property the possible-side compile actually relies on."""
    from gmc.coreset.inner import inner_ellipse
    rng = np.random.default_rng(500 + seed)
    # A connected blob: overlapping discs along a short random walk.
    centres = np.cumsum(rng.normal(scale=0.06, size=(7, 2)), axis=0)
    members = [GaussianSupport2D(mean=c, covariance=np.eye(2) * 0.01,
                                 level=1.0, primitive_id=k)
               for k, c in enumerate(centres)]
    macro = inner_ellipse(members, primitive_id=0)
    L = np.linalg.cholesky(shape_matrix(macro.support))
    z = rng.normal(size=(3000, 2))
    z /= np.maximum(np.linalg.norm(z, axis=1, keepdims=True), 1e-300)
    z *= rng.uniform(0.0, 1.0, size=(3000, 1)) ** 0.5
    pts = macro.support.mean + z @ L.T
    assert all(_inside_any(p, members) for p in pts)


def _sample_interior(support, rng, n=1500):
    """Uniform-ish sample of points strictly inside an ellipse."""
    L = np.linalg.cholesky(shape_matrix(support))
    z = rng.normal(size=(n, 2))
    z /= np.maximum(np.linalg.norm(z, axis=1, keepdims=True), 1e-300)
    z *= rng.uniform(0.0, 1.0, size=(n, 1)) ** 0.5
    return support.mean + z @ L.T


def _inside(point, support, tol=1e-9) -> bool:
    d = np.asarray(point, float) - support.mean
    return float(d @ np.linalg.solve(shape_matrix(support), d)) <= 1.0 + tol


def test_full_sandwich_holds_analytically_on_the_real_partition():
    """C_minus subset C_star subset C_plus, checked without polygons.

    A polygon-vs-polygon test cannot decide this: for a single-member group the
    inner macro *is* the member, so comparing its circumscribed polygon against
    the member's inscribed polygon reports the ring between them (~3e-6 area)
    as a spurious leak.  Both inclusions are exact statements about ellipses, so
    they are checked as such.
    """
    from gmc.coreset.inner import inner_ellipse
    rng = np.random.default_rng(20260904)
    scene = _scene()
    res = coarsen_scene(scene, ROBOT,
                        CoarsenParams(clearance_tol=0.002,
                                      target_gate_tol=0.02))
    assert len(res.cut_members) == res.n_macro

    for group, outer in zip(res.cut_members, res.scene.supports):
        members = [scene.supports[i] for i in group]
        # C_star subset C_plus: every member ellipse inside its group's macro.
        for m in members:
            pts = _sample_interior(m, rng, 400)
            assert all(_inside(p, outer) for p in pts)
        # C_minus subset C_star: the inner macro inside the member union.
        inner = inner_ellipse(members, primitive_id=0).support
        for p in _sample_interior(inner, rng, 400):
            assert any(_inside(p, m) for m in members)


def test_inner_scene_is_group_aligned_and_smaller():
    from gmc.coreset.inner import inner_scene
    scene = _scene()
    res = coarsen_scene(scene, ROBOT,
                        CoarsenParams(clearance_tol=0.002,
                                      target_gate_tol=0.02))
    inner = inner_scene(scene, res.cut_members)
    assert len(inner.supports) == res.n_macro

    def area(s):
        eig = np.linalg.eigvalsh(s.covariance) * (s.level ** 2)
        semi = np.sqrt(np.maximum(eig, 0.0))
        return float(np.pi * semi[0] * semi[1])

    for lo, hi in zip(inner.supports, res.scene.supports):
        assert area(lo) <= area(hi) + 1e-12
