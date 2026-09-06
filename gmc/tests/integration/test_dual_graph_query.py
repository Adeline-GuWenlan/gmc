"""Dual-graph semantics (Guide §14.2): possible bounds safe; three-valued
outputs behave per §1.2."""
import numpy as np
import pytest

from gmc.io.robot_io import ellipse_robot
from gmc.mobility.graph import compile_mobility
from gmc.mobility.query import query
from gmc.orientation.slab_builder import build_slabs
from gmc.spatial.bvh import candidate_pairs
from gmc.synth import dual_route
from gmc.types import PlanStatus, Pose2

from ..conftest import make_cfg


@pytest.fixture(scope="module")
def dual_compiled():
    cfg = make_cfg(theta_min=5e-3, initial_intervals=8)
    scene = dual_route(w1=0.9, w2=0.6, workspace=(-2.4, 2.4, -2.0, 2.0))
    robot = ellipse_robot(0.4, 0.18)
    oracles = candidate_pairs(scene, robot, scene.workspace)
    dec = build_slabs(scene, robot, cfg, oracles)
    mc = compile_mobility(scene, robot, cfg, oracles, dec)
    return mc


class TestDualGraph:
    def test_possible_contains_safe(self, dual_compiled):
        """I3 graph nesting via the safe->possible component mapping (the
        graphs use separate namespaces; containment is semantic)."""
        mc = dual_compiled
        for n, d in mc.M_safe.nodes(data=True):
            assert (d["slab"], d["comp"]) in mc.safe_to_possible
        for a, b in mc.M_safe.edges:
            pa = mc.possible_of_safe(mc.M_safe.nodes[a]["slab"],
                                     mc.M_safe.nodes[a]["comp"])
            pb = mc.possible_of_safe(mc.M_safe.nodes[b]["slab"],
                                     mc.M_safe.nodes[b]["comp"])
            assert mc.M_possible.has_edge(pa, pb)

    def test_cross_wall_reachable(self, dual_compiled):
        res = query(Pose2(np.array([-1.8, 0.0]), 0.0),
                    Pose2(np.array([1.8, 0.0]), 0.0), dual_compiled)
        assert res.status is PlanStatus.REACHABLE

    def test_pose_inside_obstacle_invalid(self, dual_compiled):
        res = query(Pose2(np.array([0.0, 1.8]), np.pi / 2),
                    Pose2(np.array([1.8, 0.0]), 0.0), dual_compiled)
        assert res.status is PlanStatus.INVALID_GEOMETRY
