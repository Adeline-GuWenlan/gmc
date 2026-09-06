"""Adversarial regressions for P5 incremental and event-search contracts."""
from dataclasses import replace

import numpy as np
import pytest
import shapely

from gmc.io.robot_io import ellipse_robot
from gmc.spatial.incremental import IncrementalSliceCompiler
from gmc.spatial.slice_compiler import build_slice
from gmc.types import GaussianSupport2D, SceneModel2D

from ..conftest import make_cfg


def _support(primitive_id, xy, radius):
    return GaussianSupport2D(
        mean=np.asarray(xy, dtype=float),
        covariance=np.eye(2) * float(radius) ** 2,
        level=1.0,
        primitive_id=primitive_id,
    )


def _scene(specs):
    return SceneModel2D(
        supports=tuple(
            _support(primitive_id, xy, radius)
            for primitive_id, (xy, radius) in enumerate(specs)
        ),
        workspace=shapely.box(-2.0, -1.5, 2.0, 1.5),
        name="p5_history_regression",
    )


def _component_union(components):
    return shapely.union_all(
        [component.geometry for component in components]
    )


def _assert_geometry_equal(actual, expected):
    assert actual.symmetric_difference(expected).area <= 1e-11


def _assert_slice_semantics_equal(actual, expected):
    assert actual.status is expected.status
    assert len(actual.D_safe) == len(expected.D_safe)
    assert len(actual.D_possible) == len(expected.D_possible)
    _assert_geometry_equal(actual.C_plus, expected.C_plus)
    _assert_geometry_equal(actual.C_minus, expected.C_minus)
    _assert_geometry_equal(
        _component_union(actual.D_safe),
        _component_union(expected.D_safe),
    )
    _assert_geometry_equal(
        _component_union(actual.D_possible),
        _component_union(expected.D_possible),
    )
    assert [sandwich.pair_id for sandwich in actual.sandwiches] == [
        sandwich.pair_id for sandwich in expected.sandwiches
    ]
    for got, want in zip(actual.sandwiches, expected.sandwiches):
        assert got.status is want.status
        np.testing.assert_array_equal(got.angles, want.angles)
        _assert_geometry_equal(got.outer, want.outer)
        _assert_geometry_equal(got.inner, want.inner)


@pytest.mark.parametrize("case_index", range(12))
def test_clean_and_incremental_match_across_updates_and_cache_eviction(
    case_index,
):
    """A compact subset of the fixed-RNG 60-case/1080-comparison fuzz run."""
    rng = np.random.default_rng(20260901 + case_index)
    support_count = int(rng.integers(1, 5))
    specs = [
        (
            (
                float(rng.uniform(-1.25, 1.25)),
                float(rng.uniform(-0.75, 0.75)),
            ),
            float(rng.uniform(0.05, 0.24)),
        )
        for _ in range(support_count)
    ]
    current_scene = _scene(specs)
    major = float(rng.uniform(0.18, 0.42))
    minor = float(rng.uniform(0.06, major - 0.01))
    robot = ellipse_robot(major, minor)
    cfg = make_cfg(mode="theorem", eps_pair=2e-2)
    cfg = replace(
        cfg,
        pair_approx=replace(cfg.pair_approx, max_directions=64),
    )
    compiler = IncrementalSliceCompiler(
        current_scene, robot, cfg, max_cached_orientations=3
    )
    target = float(rng.uniform(0.0, 2.0 * np.pi))

    def compare_at(theta):
        incremental = compiler.compile_slice(theta)
        clean = build_slice(current_scene, robot, theta, cfg)
        _assert_slice_semantics_equal(incremental, clean)

    compare_at(target)
    for offset in (0.37, 0.91, 1.73, 2.41):
        compare_at(target + offset)
    compare_at(target)

    for update_index in range(2):
        selected = int(rng.integers(0, support_count))
        old_xy, old_radius = specs[selected]
        if update_index == 0:
            specs[selected] = (
                (
                    old_xy[0] + float(rng.uniform(-0.12, 0.12)),
                    old_xy[1] + float(rng.uniform(-0.08, 0.08)),
                ),
                old_radius,
            )
        else:
            specs[selected] = (
                old_xy,
                max(0.03, old_radius * float(rng.uniform(0.8, 1.2))),
            )
        current_scene = _scene(specs)
        compiler.update_models(scene=current_scene)
        compare_at(target)
        for offset in (0.29, 1.17, 2.03, 2.89):
            compare_at(target + offset + 0.07 * update_index)
        compare_at(target)
