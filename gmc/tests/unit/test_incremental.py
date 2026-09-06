from dataclasses import replace

import numpy as np
import shapely

from gmc.budget import BudgetExceeded, WorkLedger
from gmc.geometry.envelopes import approximate_pair
from gmc.mobility.graph import compile_mobility
from gmc.orientation.slab_builder import build_slabs
from gmc.spatial.bvh import candidate_pairs
from gmc.spatial.incremental import (IncrementalMobilityCompiler,
                                     IncrementalSliceCompiler)
from gmc.spatial.slice_compiler import build_slice
from gmc.types import GaussianSupport2D, RobotModel2D, SceneModel2D

from ..conftest import make_cfg
from gmc.io.robot_io import ellipse_robot


def _disc(pid, x, y, radius=0.16):
    return GaussianSupport2D(
        mean=np.array([x, y]), covariance=np.eye(2) * radius**2,
        level=1.0, primitive_id=pid,
    )


def _scene(supports):
    return SceneModel2D(
        supports=tuple(supports), workspace=shapely.box(-2, -1, 2, 1),
        name="incremental_test",
    )


def _assert_equivalent(actual, expected):
    assert [len(actual.D_safe), len(actual.D_possible)] == [
        len(expected.D_safe), len(expected.D_possible)
    ]
    assert actual.C_plus.symmetric_difference(expected.C_plus).area < 1e-12
    assert actual.C_minus.symmetric_difference(expected.C_minus).area < 1e-12
    assert [s.pair_id for s in actual.sandwiches] == [
        s.pair_id for s in expected.sandwiches
    ]


def test_repeated_orientation_reuses_all_pair_envelopes():
    cfg = make_cfg(eps_pair=1e-2)
    scene = _scene([_disc(0, -0.5, 0), _disc(1, 0.5, 0)])
    robot = ellipse_robot(0.25, 0.12)
    compiler = IncrementalSliceCompiler(scene, robot, cfg)

    first = compiler.compile_slice(0.37)
    assert first.support_calls > 0
    second = compiler.compile_slice(0.37)

    assert second.support_calls == 0
    assert compiler.stats.last_computed == 0
    assert compiler.stats.last_reused == len(compiler.oracles)
    _assert_equivalent(second, build_slice(scene, robot, 0.37, cfg))


def test_one_scene_edit_recomputes_only_affected_pairs():
    cfg = make_cfg(eps_pair=1e-2)
    scene = _scene([_disc(0, -0.5, 0), _disc(1, 0.5, 0),
                    _disc(2, 0, 0.5)])
    robot = ellipse_robot(0.25, 0.12)
    compiler = IncrementalSliceCompiler(scene, robot, cfg)
    compiler.compile_slice(0.21)

    edited = _scene([_disc(0, -0.5, 0), _disc(1, 0.62, 0),
                     _disc(2, 0, 0.5)])
    changed = compiler.update_models(scene=edited)
    updated = compiler.compile_slice(0.21)
    full = build_slice(edited, robot, 0.21, cfg)

    assert {p.scene_id for p in changed} == {1}
    assert compiler.stats.last_computed == len(robot.supports)
    assert compiler.stats.last_reused == len(compiler.oracles) - len(robot.supports)
    assert 0 < updated.support_calls < full.support_calls
    _assert_equivalent(updated, full)


def test_removed_pair_cannot_survive_cache():
    cfg = make_cfg(eps_pair=1e-2)
    robot = ellipse_robot(0.25, 0.12)
    initial = _scene([_disc(0, -0.5, 0), _disc(1, 0.5, 0)])
    compiler = IncrementalSliceCompiler(initial, robot, cfg)
    compiler.compile_slice(0.0)

    reduced = _scene([_disc(0, -0.5, 0)])
    changed = compiler.update_models(scene=reduced)
    actual = compiler.compile_slice(0.0)

    assert {p.scene_id for p in changed} == {1}
    assert {s.pair_id.scene_id for s in actual.sandwiches} == {0}
    _assert_equivalent(actual, build_slice(reduced, robot, 0.0, cfg))


def test_support_arrays_are_value_immutable():
    support = _disc(0, 0.0, 0.0)
    with np.testing.assert_raises(ValueError):
        support.mean[0] = 1.0
    with np.testing.assert_raises(ValueError):
        support.covariance[0, 0] = 2.0


def test_duplicate_primitive_ids_are_rejected_before_pair_cache_keying():
    with np.testing.assert_raises(ValueError):
        _scene([_disc(0, -0.5, 0), _disc(0, 0.5, 0)])


def test_incremental_pipeline_matches_clean_full_rebuild_after_edit():
    cfg = make_cfg(eps_pair=2e-2, theta_min=0.35,
                   initial_intervals=4)
    robot = ellipse_robot(0.25, 0.12)
    initial = _scene([_disc(0, -0.45, 0), _disc(1, 0.45, 0)])
    compiler = IncrementalMobilityCompiler(initial, robot, cfg)
    compiler.compile()

    edited = _scene([_disc(0, -0.45, 0), _disc(1, 0.6, 0)])
    compiler.update_models(scene=edited)
    incremental = compiler.compile()

    oracles = candidate_pairs(edited, robot, edited.workspace)
    full_dec = build_slabs(edited, robot, cfg, oracles)
    full_mc = compile_mobility(edited, robot, cfg, oracles, full_dec)

    inc_slabs = incremental.decomposition.slabs
    assert [(s.interval.lo, s.interval.hi, s.kind) for s in inc_slabs] == [
        (s.interval.lo, s.interval.hi, s.kind) for s in full_dec.slabs
    ]
    for key, actual in incremental.decomposition.slice_cache.items():
        _assert_equivalent(actual, full_dec.slice_cache[key])
    assert set(incremental.mobility.M_safe.nodes) == set(full_mc.M_safe.nodes)
    assert set(incremental.mobility.M_safe.edges) == set(full_mc.M_safe.edges)
    assert set(incremental.mobility.M_possible.nodes) == \
        set(full_mc.M_possible.nodes)
    assert set(incremental.mobility.M_possible.edges) == \
        set(full_mc.M_possible.edges)
    assert incremental.envelopes_reused > 0
    assert incremental.envelopes_computed > 0
    assert incremental.support_calls == sum(
        incremental.work_delta["totals"][kind]
        for kind in ("support_value_evals", "support_point_evals")
    )
    assert incremental.support_calls >= incremental.envelope_support_calls


def test_same_theta_edit_updates_one_union_leaf_not_whole_tree():
    cfg = make_cfg(eps_pair=1e-2)
    scene = _scene([
        _disc(i, -1.4 + 0.4 * i, 0.25 * ((i % 2) - 0.5))
        for i in range(8)
    ])
    robot = ellipse_robot(0.25, 0.12)
    compiler = IncrementalSliceCompiler(scene, robot, cfg)
    compiler.compile_slice(0.31)
    full_tree_operations = compiler.stats.last_union_operations

    edited_supports = list(scene.supports)
    edited_supports[3] = _disc(3, -1.4 + 0.4 * 3, 0.22)
    edited = _scene(edited_supports)
    compiler.update_models(scene=edited)
    actual = compiler.compile_slice(0.31)

    assert compiler.stats.last_union_leaf_updates == 1
    assert 0 < compiler.stats.last_union_operations < full_tree_operations
    _assert_equivalent(actual, build_slice(edited, robot, 0.31, cfg))


def test_adjacent_orientation_bypasses_history_dependent_direction_seed():
    cfg = make_cfg(mode="theorem", eps_pair=1e-2)
    scene = _scene([_disc(0, -0.5, 0), _disc(1, 0.5, 0)])
    robot = ellipse_robot(0.31, 0.09)
    compiler = IncrementalSliceCompiler(scene, robot, cfg)
    compiler.compile_slice(0.41)
    continued = compiler.compile_slice(0.43)
    clean = build_slice(scene, robot, 0.43, cfg)

    assert compiler.stats.last_orientation_seeded == 0
    assert compiler.stats.last_orientation_seeds_bypassed == len(
        compiler.oracles
    )
    assert compiler.stats.last_computed == len(compiler.oracles)
    assert continued.support_calls == clean.support_calls
    for actual, expected in zip(continued.sandwiches, clean.sandwiches):
        assert actual.status == expected.status
        np.testing.assert_array_equal(actual.angles, expected.angles)
        assert actual.outer.equals_exact(expected.outer, 0.0)
        assert actual.inner.equals_exact(expected.inner, 0.0)


def test_orientation_result_is_stable_after_cache_eviction():
    cfg = make_cfg(mode="theorem", eps_pair=1e-2)
    scene = _scene([_disc(0, -0.5, 0), _disc(1, 0.5, 0)])
    robot = ellipse_robot(0.31, 0.09)
    compiler = IncrementalSliceCompiler(
        scene, robot, cfg, max_cached_orientations=2
    )

    compiler.compile_slice(0.40)
    before_eviction = compiler.compile_slice(0.43)
    compiler.compile_slice(0.80)
    compiler.compile_slice(1.10)
    after_eviction = compiler.compile_slice(0.43)
    clean = build_slice(scene, robot, 0.43, cfg)

    for actual in (before_eviction, after_eviction):
        _assert_equivalent(actual, clean)
        assert [s.status for s in actual.sandwiches] == [
            s.status for s in clean.sandwiches
        ]
        for got, expected in zip(actual.sandwiches, clean.sandwiches):
            np.testing.assert_array_equal(got.angles, expected.angles)
            assert got.outer.equals_exact(expected.outer, 0.0)
            assert got.inner.equals_exact(expected.inner, 0.0)


def test_pair_approximation_seed_argument_cannot_change_result():
    cfg = make_cfg(mode="theorem", eps_pair=1e-2)
    scene = _scene([_disc(0, 0.0, 0.0)])
    robot = ellipse_robot(0.31, 0.09)
    clean_oracle = candidate_pairs(scene, robot, scene.workspace)[0]
    seeded_oracle = candidate_pairs(scene, robot, scene.workspace)[0]

    clean = approximate_pair(clean_oracle, 0.43, cfg.pair_approx)
    seeded = approximate_pair(
        seeded_oracle,
        0.43,
        cfg.pair_approx,
        seed_angles=np.linspace(0.0, 2.0 * np.pi, 37, endpoint=False),
    )

    assert seeded.status is clean.status
    np.testing.assert_array_equal(seeded.angles, clean.angles)
    assert seeded.outer.equals_exact(clean.outer, 0.0)
    assert seeded.inner.equals_exact(clean.inner, 0.0)


def test_theta_invariant_body_reuses_envelopes_and_union_without_work():
    cfg = make_cfg(eps_pair=1e-2)
    scene = _scene([_disc(0, -0.5, 0), _disc(1, 0.5, 0)])
    robot = ellipse_robot(0.2, 0.2)
    compiler = IncrementalSliceCompiler(scene, robot, cfg)
    first = compiler.compile_slice(0.0)
    second = compiler.compile_slice(0.7)

    assert compiler.stats.last_theta_invariant_reuses == len(compiler.oracles)
    assert compiler.stats.last_computed == 0
    assert second.support_calls == 0
    assert compiler.stats.last_union_leaf_updates == 0
    assert compiler.stats.last_union_operations == 0
    assert second.C_plus.equals_exact(first.C_plus, 0.0)
    assert second.C_minus.equals_exact(first.C_minus, 0.0)


def test_nearly_isotropic_large_body_is_not_reused_as_exactly_invariant():
    cfg = make_cfg(eps_pair=1e9)
    # The cache-identity test intentionally places an envelope at radius 1e12.
    # Use a representable GEOS precision there; the default 1e-8 grid is below
    # one binary64 ULP and is now correctly rejected by the M0 resolution gate.
    cfg = replace(
        cfg,
        geometry=replace(cfg.geometry, workspace_precision=1e-3),
    )
    scene = _scene([_disc(0, 0.0, 0.0)])
    body = GaussianSupport2D(
        mean=np.zeros(2),
        covariance=np.diag([1e24 + 4e11, 1e24 - 4e11]),
        level=1.0,
        primitive_id=0,
    )
    robot = RobotModel2D((body,), "large-nearly-isotropic")
    compiler = IncrementalSliceCompiler(scene, robot, cfg)
    compiler.compile_slice(0.0)
    compiler.compile_slice(np.pi / 2.0)

    assert compiler.stats.last_theta_invariant_reuses == 0
    assert compiler.stats.last_computed == 1


def test_pair_config_change_invalidates_cached_envelopes():
    cfg = make_cfg(eps_pair=2e-2)
    scene = _scene([_disc(0, 0.0, 0.0)])
    robot = ellipse_robot(0.25, 0.12)
    compiler = IncrementalSliceCompiler(scene, robot, cfg)
    compiler.compile_slice(0.2)
    compiler.cfg = replace(
        cfg, pair_approx=replace(cfg.pair_approx, eps_pair=1e-2)
    )
    compiler.compile_slice(0.2)

    assert compiler.stats.config_invalidations == 1
    assert compiler.stats.last_computed == len(compiler.oracles)


def test_budget_failure_does_not_commit_partial_slice_cache():
    cfg = make_cfg(eps_pair=1e-4)
    scene = _scene([_disc(0, 0.0, 0.0)])
    robot = ellipse_robot(0.25, 0.12)
    ledger = WorkLedger(limits={"support_value_evals": 20})
    compiler = IncrementalSliceCompiler(scene, robot, cfg, ledger=ledger)

    with np.testing.assert_raises(BudgetExceeded):
        compiler.compile_slice(0.2)
    assert compiler.cache_size == 0
    assert compiler.stats.builds == 0
    assert compiler.stats.failed_builds == 1
    assert ledger.total("support_value_evals") == 16


def test_seed_lookup_is_pair_indexed_and_orientation_cache_is_bounded():
    cfg = make_cfg(eps_pair=5e-2)
    supports = [_disc(i, -1.5 + 0.09 * i, 0.0, radius=0.05)
                for i in range(32)]
    scene = _scene(supports)
    robot = ellipse_robot(0.12, 0.07)
    compiler = IncrementalSliceCompiler(
        scene, robot, cfg, max_cached_orientations=4
    )

    for theta in np.linspace(0.0, 0.5, 7):
        compiler.compile_slice(float(theta))

    n_pairs = len(compiler.oracles)
    assert n_pairs == len(supports)
    assert compiler.stats.last_seed_lookup_candidates <= 2 * n_pairs
    assert compiler.stats.last_union_lookup_candidates <= 2
    assert compiler.cache_size <= 4 * n_pairs
    assert len(compiler._union_cache) <= 4
    assert sum(map(len, compiler._union_angles.values())) <= 4
    assert all(len(oracle._theta_cache) <= 4 for oracle in compiler.oracles)
