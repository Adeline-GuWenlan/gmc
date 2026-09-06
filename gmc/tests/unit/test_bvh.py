"""Conservativeness and pruning tests for the scene-support BVH."""

import numpy as np
import pytest
from shapely.geometry import Point, box

from gmc.geometry.support import PairOracle, build_oracles, unit_dirs
from gmc.spatial.bvh import (SceneSupportBVH, candidate_pairs,
                             pair_bounding_disc, query_candidate_pairs)
from gmc.types import (GaussianSupport2D, PairID, RobotModel2D,
                       SceneModel2D)


def _support(mean, axes, primitive_id, angle=0.0, level=1.0):
    c, s = np.cos(angle), np.sin(angle)
    rotation = np.array([[c, -s], [s, c]])
    covariance = rotation @ np.diag(np.square(axes)) @ rotation.T
    return GaussianSupport2D(np.asarray(mean, dtype=float), covariance,
                             level, primitive_id)


def _flat_candidates(scene, robot, workspace):
    """The exact v0 loop, kept independent of the hierarchy traversal."""
    expanded = workspace.buffer(robot.max_rotational_radius())
    keep = []
    for oracle in build_oracles(scene, robot):
        center, radius = pair_bounding_disc(oracle)
        if expanded.distance(Point(center)) <= radius:
            keep.append(oracle)
    return keep


def _ids(oracles):
    return [(o.pair_id.scene_id, o.pair_id.body_id) for o in oracles]


def test_pair_bounding_disc_contains_anisotropic_obstacle_samples():
    scene_support = _support((1.2, -0.7), (1.4, 0.12), 3,
                             angle=0.65, level=1.7)
    body_support = _support((0.8, -0.35), (0.55, 0.08), 7,
                            angle=-0.4, level=1.3)
    oracle = PairOracle(PairID(3, 7), scene_support, body_support)
    center, radius = pair_bounding_disc(oracle)
    directions = unit_dirs(np.linspace(0.0, 2.0 * np.pi, 257)[:-1])

    for theta in np.linspace(0.0, 2.0 * np.pi, 33):
        points = oracle.support_points(theta, directions)
        assert np.max(np.linalg.norm(points - center, axis=1)) <= radius + 1e-12


@pytest.mark.parametrize("leaf_size", [1, 3, 8, 32])
def test_hierarchy_exactly_matches_flat_candidate_set_and_order(leaf_size):
    rng = np.random.default_rng(20260901)
    workspace = box(-3.0, -2.0, 3.0, 2.0)
    scene_supports = []
    for primitive_id in range(73):
        scene_supports.append(_support(
            rng.uniform((-18.0, -12.0), (18.0, 12.0)),
            rng.uniform((0.03, 0.04), (1.2, 0.8)),
            primitive_id,
            angle=rng.uniform(-np.pi, np.pi),
            level=rng.uniform(0.7, 2.1),
        ))
    body_supports = tuple(
        _support(rng.uniform(-0.8, 0.8, 2),
                 rng.uniform((0.05, 0.05), (0.7, 0.5)),
                 100 + body_id,
                 angle=rng.uniform(-np.pi, np.pi),
                 level=rng.uniform(0.8, 1.5))
        for body_id in range(4)
    )
    scene = SceneModel2D(tuple(scene_supports), workspace, "random")
    robot = RobotModel2D(body_supports, "multi-part")

    expected = _flat_candidates(scene, robot, workspace)
    index = SceneSupportBVH.from_scene(scene, leaf_size=leaf_size)
    result = query_candidate_pairs(scene, robot, workspace, index=index)

    assert _ids(result.oracles) == _ids(expected)
    assert _ids(candidate_pairs(scene, robot, workspace)) == _ids(expected)
    assert result.stats.pair_tests + result.stats.pruned_pairs == \
        result.stats.total_pairs
    assert result.stats.candidate_pairs == len(expected)


def test_far_scene_clusters_are_pruned_before_leaf_pair_tests():
    workspace = box(-2.0, -2.0, 2.0, 2.0)
    near = [
        _support((0.25 * (i % 4), 0.25 * (i // 4)), (0.08, 0.04), i)
        for i in range(8)
    ]
    far = [
        _support((80.0 + 0.15 * (i % 64),
                  -25.0 + 0.15 * (i // 64)),
                 (0.06, 0.03), 8 + i, angle=0.2)
        for i in range(1016)
    ]
    scene = SceneModel2D(tuple(near + far), workspace, "far-clusters")
    robot = RobotModel2D((
        _support((0.0, 0.0), (0.22, 0.12), 0),
        _support((0.35, 0.0), (0.16, 0.09), 1, angle=0.3),
    ))
    index = SceneSupportBVH.from_scene(scene, leaf_size=8)

    result = index.query(robot, workspace)
    expected = _flat_candidates(scene, robot, workspace)

    assert _ids(result.oracles) == _ids(expected)
    assert result.stats.nodes_pruned > 0
    assert result.stats.pruned_pairs > 0
    assert result.stats.pair_tests < result.stats.total_pairs // 8
    assert result.stats.pair_test_fraction < 0.125
    assert result.stats.nodes_visited < index.node_count * len(robot.supports)
    assert len(result.decisions) == result.stats.total_pairs
    assert len({row.record_id for row in result.decisions}) == \
        result.stats.total_pairs
    assert [(row.scene_index, row.body_index)
            for row in result.decisions] == sorted(
                (row.scene_index, row.body_index)
                for row in result.decisions)
    retained = {
        (row.scene_id, row.body_id)
        for row in result.decisions if row.decision == "RETAINED"
    }
    assert retained == set(_ids(result.oracles))
    internal = [row for row in result.decisions
                if row.proof_scope == "internal_node"]
    assert internal
    assert all(row.decision == "REJECTED" for row in internal)
    assert all(row.finite_comparison for row in internal)
    assert all(row.workspace_distance > row.comparison_rhs
               for row in internal)
    assert all(row.descendant_containment_margin >= -1e-12
               for row in internal)


def test_empty_scene_has_zero_work_and_no_candidates():
    workspace = box(-1.0, -1.0, 1.0, 1.0)
    scene = SceneModel2D((), workspace, "empty")
    robot = RobotModel2D((_support((0.0, 0.0), (0.2, 0.1), 0),))

    result = query_candidate_pairs(scene, robot, workspace)

    assert result.oracles == ()
    assert result.stats.total_pairs == 0
    assert result.stats.nodes_visited == 0
    assert result.stats.pair_test_fraction == 0.0
    assert result.decisions == ()


def test_reused_index_rejects_a_different_scene():
    workspace = box(-1.0, -1.0, 1.0, 1.0)
    support = _support((0.0, 0.0), (0.2, 0.1), 0)
    scene = SceneModel2D((support,), workspace, "first")
    copied_scene = SceneModel2D((
        _support((0.0, 0.0), (0.2, 0.1), 0),
    ), workspace, "copy")
    robot = RobotModel2D((_support((0.0, 0.0), (0.1, 0.1), 0),))
    index = SceneSupportBVH.from_scene(scene)

    with pytest.raises(ValueError, match="different scene"):
        query_candidate_pairs(copied_scene, robot, workspace, index=index)
