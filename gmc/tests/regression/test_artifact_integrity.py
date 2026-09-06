"""Run fingerprints cover all input fields that affect geometry."""
import numpy as np
import shapely
from shapely.geometry import Polygon

from gmc.reporting.artifacts import robot_hash, scene_hash
from gmc.spatial.bvh import candidate_pairs
from gmc.types import GaussianSupport2D, RobotModel2D, SceneModel2D
from gmc.verification.invariants import check_manifest_reproducibility


def _support(pid, mean=(0.0, 0.0)):
    return GaussianSupport2D(
        mean=np.asarray(mean, dtype=float), covariance=np.eye(2),
        level=1.0, primitive_id=pid,
    )


def test_scene_hash_includes_workspace_holes():
    exterior = [(-2, -2), (2, -2), (2, 2), (-2, 2), (-2, -2)]
    without_hole = SceneModel2D((_support(1),), Polygon(exterior))
    with_hole = SceneModel2D(
        (_support(1),),
        Polygon(exterior, holes=[[
            (-0.5, -0.5), (0.5, -0.5), (0.5, 0.5),
            (-0.5, 0.5), (-0.5, -0.5),
        ]]),
    )
    assert scene_hash(without_hole) != scene_hash(with_hole)


def test_input_hashes_include_primitive_ids():
    workspace = shapely.box(-2, -2, 2, 2)
    assert scene_hash(SceneModel2D((_support(1),), workspace)) != scene_hash(
        SceneModel2D((_support(2),), workspace)
    )
    assert robot_hash(RobotModel2D((_support(1),))) != robot_hash(
        RobotModel2D((_support(2),))
    )


def _ordered_pair_ids(scene, robot):
    return [(oracle.pair_id.scene_id, oracle.pair_id.body_id)
            for oracle in candidate_pairs(scene, robot, scene.workspace)]


def test_hashes_bind_the_tuple_order_used_by_pair_enumeration():
    workspace = shapely.box(-10, -10, 10, 10)
    scene_a = SceneModel2D(
        (_support(20, (-1.0, 0.0)), _support(10, (1.0, 0.0))),
        workspace,
    )
    scene_same = SceneModel2D(
        (_support(20, (-1.0, 0.0)), _support(10, (1.0, 0.0))),
        workspace,
    )
    scene_reordered = SceneModel2D(tuple(reversed(scene_a.supports)),
                                   workspace)
    robot_a = RobotModel2D(
        (_support(4, (-0.1, 0.0)), _support(3, (0.1, 0.0))))
    robot_same = RobotModel2D(
        (_support(4, (-0.1, 0.0)), _support(3, (0.1, 0.0))))
    robot_reordered = RobotModel2D(tuple(reversed(robot_a.supports)))

    assert scene_hash(scene_a) == scene_hash(scene_same)
    assert robot_hash(robot_a) == robot_hash(robot_same)
    assert _ordered_pair_ids(scene_a, robot_a) == \
        _ordered_pair_ids(scene_same, robot_same)

    # Reordering changes the compiler's scene-major/body-minor PairID stream,
    # so I7 requires the corresponding input fingerprint to change as well.
    assert scene_hash(scene_a) != scene_hash(scene_reordered)
    assert robot_hash(robot_a) != robot_hash(robot_reordered)
    assert _ordered_pair_ids(scene_a, robot_a) != \
        _ordered_pair_ids(scene_reordered, robot_a)
    assert _ordered_pair_ids(scene_a, robot_a) != \
        _ordered_pair_ids(scene_a, robot_reordered)


def test_i7_checker_does_not_turn_honest_missing_artifacts_into_pass():
    manifest = {
        "scene_hash": "scene", "robot_hash": "robot",
        "length_unit": "meter", "workspace": {},
        "scene_support_levels": [], "robot_support_levels": [],
        "software": {}, "numerical_validation": {},
        "i7_complete": False,
        "artifact_scope": {"i7_complete": False},
        "config": {
            "source": "input/config.yaml", "eps_pair": 1e-3,
            "certificate_mode": "theorem", "initial_directions": 16,
            "max_directions": 256, "initial_intervals": 16,
            "theta_min": 1e-4, "max_depth": 18,
            "max_support_calls": 1, "max_wall_seconds": 1.0,
            "eps_clear": 1e-3, "workspace_precision": 1e-8,
        },
    }
    assert not check_manifest_reproducibility(manifest)
