"""Path lifting + independent verification + reversible-DD certifier
(Guide §11, §12)."""
import numpy as np
import pytest

from gmc.dynamics.reversible_dd import (CorridorVerificationContext,
                                        ReversibleDDCertifier,
                                        ReversibleDiffDrive)
from gmc.io.robot_io import ellipse_robot
from gmc.mobility.graph import compile_mobility
from gmc.mobility.query import query
from gmc.mobility.witness import SegmentKind
from gmc.orientation.slab_builder import build_slabs
from gmc.spatial.bvh import candidate_pairs
from gmc.synth import single_door
from gmc.types import CertStatus, PlanStatus, Pose2
from gmc.verification.path import verify_curve

from ..conftest import SMALL_WS, make_cfg


@pytest.fixture(scope="module")
def plan():
    cfg = make_cfg(theta_min=5e-3, initial_intervals=8)
    scene = single_door(0.6, workspace=SMALL_WS)
    robot = ellipse_robot(0.5, 0.2)
    oracles = candidate_pairs(scene, robot, scene.workspace)
    dec = build_slabs(scene, robot, cfg, oracles)
    mc = compile_mobility(scene, robot, cfg, oracles, dec)
    res = query(Pose2(np.array([-1.7, 0.0]), 0.3),
                Pose2(np.array([1.7, 0.3]), 2.8), mc)
    return cfg, scene, robot, oracles, res


class TestLifting:
    def test_reachable_and_curve_shape(self, plan):
        cfg, scene, robot, oracles, res = plan
        assert res.status is PlanStatus.REACHABLE
        kinds = {s.kind for s in res.curve.segments}
        assert kinds <= {SegmentKind.TRANSLATION, SegmentKind.ROTATION}

    def test_c0_continuity(self, plan):
        *_, res = plan
        for a, b in zip(res.curve.segments[:-1], res.curve.segments[1:]):
            assert np.linalg.norm(b.q0.xy - a.q1.xy) < 1e-9
            assert abs(b.q0.theta - a.q1.theta) < 1e-9

    def test_independent_verifier_repasses(self, plan):
        cfg, scene, robot, oracles, res = plan
        rep = verify_curve(oracles, scene.workspace, res.curve,
                           cfg.query.eps_clear, cfg.orientation.theta_min)
        assert rep.certified, rep
        assert rep.min_clearance >= cfg.query.eps_clear

    def test_reversible_dd_rejects_holonomic_sideways_legs(self, plan):
        cfg, scene, robot, oracles, res = plan

        class Budget:
            candidate_curve = res.curve
            max_wall_seconds = 5.0

        cert = ReversibleDDCertifier().certify(
            CorridorVerificationContext(
                scene.workspace, scene, robot,
                eps_clear=cfg.query.eps_clear,
                theta_min=cfg.orientation.theta_min,
            ), res.curve.segments[0].q0,
            res.curve.segments[-1].q1, ReversibleDiffDrive(), Budget())
        # The holonomic lift may translate in a direction different from the
        # current body heading.  It is a valid SE(2) path, but it is not by
        # itself a reversible differential-drive trajectory; a Reeds--Shepp
        # or other local steering backend would be needed to replace it.
        assert cert.status is CertStatus.UNKNOWN
        assert cert.reason_code == "translation_not_heading_aligned"
