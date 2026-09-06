"""End-to-end single door (Guide §14.2 integration, §16.3 demo semantics)."""
import numpy as np
import pytest

from gmc.io.robot_io import ellipse_robot
from gmc.mobility.graph import compile_mobility
from gmc.mobility.query import query
from gmc.orientation.slab_builder import build_slabs
from gmc.spatial.bvh import candidate_pairs
from gmc.synth import single_door
from gmc.types import PlanStatus, Pose2
from gmc.verification.invariants import check_all

from ..conftest import SMALL_WS, make_cfg


@pytest.fixture(scope="module")
def door_compiled():
    cfg = make_cfg(theta_min=5e-3, initial_intervals=8)
    scene = single_door(0.6, workspace=SMALL_WS)
    robot = ellipse_robot(0.5, 0.2)   # half-angle ~ 33 deg: passable
    oracles = candidate_pairs(scene, robot, scene.workspace)
    dec = build_slabs(scene, robot, cfg, oracles)
    mc = compile_mobility(scene, robot, cfg, oracles, dec)
    return cfg, scene, robot, mc


class TestSingleDoor:
    def test_invariants(self, door_compiled):
        *_, mc = door_compiled
        inv = check_all(mc)
        assert all(inv.values()), inv

    def test_cross_door_reachable_with_certified_path(self, door_compiled):
        *_, mc = door_compiled
        res = query(Pose2(np.array([-1.7, 0.0]), 0.1),
                    Pose2(np.array([1.7, 0.0]), 0.1), mc)
        assert res.status is PlanStatus.REACHABLE
        assert res.curve is not None
        assert res.clearance_lower_bound > 0

    def test_closed_theta_start_still_reachable_via_rotation(self, door_compiled):
        *_, mc = door_compiled
        res = query(Pose2(np.array([-1.7, 0.0]), np.pi / 2),
                    Pose2(np.array([1.7, 0.0]), np.pi / 2), mc)
        assert res.status is PlanStatus.REACHABLE


class TestSealedDoor:
    def test_cut_without_global_cover_abstains(self):
        cfg = make_cfg(theta_min=1e-2, initial_intervals=8)
        scene = single_door(0.35, workspace=SMALL_WS)   # w < 2b = 0.4
        robot = ellipse_robot(0.5, 0.2)
        oracles = candidate_pairs(scene, robot, scene.workspace)
        dec = build_slabs(scene, robot, cfg, oracles)
        mc = compile_mobility(scene, robot, cfg, oracles, dec)
        res = query(Pose2(np.array([-1.7, 0.0]), 0.0),
                    Pose2(np.array([1.7, 0.0]), 0.0), mc)
        assert res.status is PlanStatus.UNKNOWN
        assert res.report["reason"] == \
            "possible_cut_is_not_a_global_certificate"
        assert "possible_cut_candidate" in res.report
        assert res.report["global_possible_cover_status"] == "UNKNOWN"
