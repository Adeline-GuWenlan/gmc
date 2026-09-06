"""Keyhole pipeline (Guide §14.2): wide position aperture, narrow
orientation gate into the chamber."""
import numpy as np
import pytest

from gmc.io.robot_io import ellipse_robot
from gmc.mobility.graph import compile_mobility
from gmc.mobility.query import query
from gmc.orientation.slab_builder import build_slabs
from gmc.spatial.bvh import candidate_pairs
from gmc.synth import keyhole
from gmc.types import PlanStatus, Pose2

from ..conftest import make_cfg


@pytest.fixture(scope="module")
def keyhole_compiled():
    cfg = make_cfg(theta_min=5e-3, initial_intervals=8)
    scene = keyhole(slot_width=0.7, workspace=(-2.4, 2.4, -1.8, 1.8))
    robot = ellipse_robot(0.45, 0.2)
    oracles = candidate_pairs(scene, robot, scene.workspace)
    dec = build_slabs(scene, robot, cfg, oracles)
    mc = compile_mobility(scene, robot, cfg, oracles, dec)
    return cfg, scene, robot, mc


class TestKeyhole:
    def test_enter_chamber(self, keyhole_compiled):
        *_, mc = keyhole_compiled
        res = query(Pose2(np.array([2.0, 0.0]), 0.0),
                    Pose2(np.array([0.0, 0.0]), 0.0), mc)
        assert res.status is PlanStatus.REACHABLE
        assert res.curve is not None

    def test_chamber_to_outside_far_side(self, keyhole_compiled):
        *_, mc = keyhole_compiled
        res = query(Pose2(np.array([0.0, 0.0]), 0.0),
                    Pose2(np.array([-2.0, 0.0]), np.pi), mc)
        assert res.status is PlanStatus.REACHABLE
