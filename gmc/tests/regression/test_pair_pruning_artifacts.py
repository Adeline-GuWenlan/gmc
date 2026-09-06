"""Every BVH pair decision is exact, complete, and frozen-input replayable."""

import copy

import numpy as np
import shapely

from gmc.reporting.proof_records import (
    pair_pruning_payload,
    validate_pair_pruning_payload,
)
from gmc.spatial.bvh import query_candidate_pairs
from gmc.types import GaussianSupport2D, RobotModel2D, SceneModel2D


def _support(mean, radius, primitive_id):
    return GaussianSupport2D(
        mean=np.asarray(mean, dtype=float),
        covariance=np.eye(2) * radius * radius,
        level=1.0,
        primitive_id=primitive_id,
    )


def _fixture():
    workspace = shapely.box(-1.0, -1.0, 1.0, 1.0)
    scene = SceneModel2D((
        _support((0.0, 0.0), 0.1, 10),
        _support((50.0, 0.0), 0.1, 20),
        _support((51.0, 0.0), 0.1, 30),
    ), workspace, "pruning-proof")
    robot = RobotModel2D((
        _support((0.0, 0.0), 0.2, 4),
        _support((0.1, 0.0), 0.15, 5),
    ), "two-part")
    return scene, robot


def test_pair_pruning_payload_covers_cartesian_product_and_replays():
    scene, robot = _fixture()
    query = query_candidate_pairs(
        scene, robot, scene.workspace, leaf_size=1,
    )
    payload = pair_pruning_payload(query, leaf_size=1)
    report = validate_pair_pruning_payload(
        payload, scene, robot, scene.workspace,
    )

    assert report["valid"], report
    assert payload["decision_count"] == 6
    assert len(payload["decisions"]) == 6
    assert {row["decision"] for row in payload["decisions"]} == {
        "RETAINED", "REJECTED",
    }
    assert any(row["proof_scope"] == "internal_node"
               for row in payload["decisions"])
    assert [row["record_id"] for row in payload["decisions"]] == [
        f"pair_pruning[{scene_index},{body_index}]"
        for scene_index in range(3) for body_index in range(2)
    ]


def test_pair_pruning_replay_rejects_edited_bound_and_deleted_pair():
    scene, robot = _fixture()
    payload = pair_pruning_payload(
        query_candidate_pairs(
            scene, robot, scene.workspace, leaf_size=1,
        ),
        leaf_size=1,
    )

    edited = copy.deepcopy(payload)
    edited["decisions"][0]["threshold"]["hex"] = "0x1.0p+99"
    assert not validate_pair_pruning_payload(
        edited, scene, robot, scene.workspace,
    )["valid"]

    deleted = copy.deepcopy(payload)
    deleted["decisions"].pop()
    deleted["decision_count"] -= 1
    report = validate_pair_pruning_payload(
        deleted, scene, robot, scene.workspace,
    )
    assert not report["valid"]
    assert "cartesian_pair_completeness" in report["errors"]
