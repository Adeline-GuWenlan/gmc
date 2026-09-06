"""An incomplete/stale pair list must never compile obstacles away."""
import numpy as np
import pytest

from gmc.io.robot_io import ellipse_robot
from gmc.mobility.graph import compile_mobility
from gmc.mobility.query import query
from gmc.orientation.slab_builder import build_slabs
from gmc.spatial.bvh import candidate_pairs
from gmc.spatial.slice_compiler import build_slice
from gmc.synth import single_obstacle
from gmc.types import PlanStatus, Pose2

from ..conftest import make_cfg


def _models():
    scene = single_obstacle(0.5, workspace=(-2.0, 2.0, -2.0, 2.0))
    return scene, ellipse_robot(0.3, 0.2)


def test_empty_oracle_override_cannot_create_certified_free_slice():
    scene, robot = _models()
    with pytest.raises(ValueError, match="candidate oracle set"):
        build_slice(scene, robot, 0.0, make_cfg(mode="theorem"), oracles=[])
    with pytest.raises(ValueError, match="candidate oracle set"):
        build_slabs(scene, robot, make_cfg(), oracles=[])


def test_mutating_a_bound_candidate_set_is_detected_before_query():
    scene, robot = _models()
    cfg = make_cfg()
    oracles = candidate_pairs(scene, robot, scene.workspace)
    decomposition = build_slabs(scene, robot, cfg, oracles)
    compiler = compile_mobility(scene, robot, cfg, oracles, decomposition)
    compiler.oracles.pop()
    result = query(Pose2(np.array([0.0, 0.0]), 0.0),
                   Pose2(np.array([0.0, 0.0]), 0.0), compiler)
    assert result.status is PlanStatus.INTERNAL_ERROR
    assert result.report["reason"] == "decomposition_input_binding_failed"
    assert "candidate_pair_identity" in result.report["failed_bindings"]
