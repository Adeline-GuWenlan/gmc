"""Regressions for finite-sample orientation evidence and query abstention."""
from dataclasses import replace
import importlib
from types import SimpleNamespace

import networkx as nx
import numpy as np

from gmc.io.robot_io import ellipse_robot
from gmc.mobility.graph import MobilityCompiler, compile_mobility
from gmc.mobility.query import lift, query
from gmc.mobility.witness import PoseCurve, PoseSegment, SegmentKind
from gmc.orientation.intervals import Interval
from gmc.orientation.slab_builder import build_slabs
from gmc.spatial.bvh import candidate_pairs
from gmc.synth import single_door, single_obstacle
from gmc.types import (CertStatus, GaussianSupport2D, PlanStatus, Pose2,
                       RobotModel2D)
from gmc.verification.path import verify_curve

from ..conftest import make_cfg


def _pre_rotated_narrow_gate():
    """A real 16-degree gate hidden between the 8-sample orientations."""
    cfg = make_cfg(theta_min=2e-3, initial_intervals=8)
    scene = single_door(0.42, workspace=(-2.0, 2.0, -2.0, 2.0))
    base = ellipse_robot(0.5, 0.2).supports[0]
    alpha = -np.pi / 16.0
    R = np.array([[np.cos(alpha), -np.sin(alpha)],
                  [np.sin(alpha), np.cos(alpha)]])
    body = GaussianSupport2D(
        mean=base.mean,
        covariance=R @ base.covariance @ R.T,
        level=base.level,
        primitive_id=base.primitive_id,
    )
    robot = RobotModel2D((body,), name="pre_rotated_ellipse")
    oracles = candidate_pairs(scene, robot, scene.workspace)
    dec = build_slabs(scene, robot, cfg, oracles)
    mc = compile_mobility(scene, robot, cfg, oracles, dec)
    return cfg, scene, oracles, dec, mc


def test_three_sample_regular_candidates_do_not_certify_global_cover():
    _, _, _, dec, _ = _pre_rotated_narrow_gate()

    # The old implementation produced eight "regular" slabs and treated
    # their possible-graph cut as a proof.  They remain useful candidates for
    # independently verified reachable paths, but they are not interval
    # certificates.
    assert len(dec.slabs) == 8
    assert all(s.kind == "regular" for s in dec.slabs)
    assert all(s.status is CertStatus.EMPIRICALLY_VALIDATED for s in dec.slabs)
    assert all(s.predicates.regular_candidate for s in dec.slabs)
    assert not any(s.predicates.certifies_no_event for s in dec.slabs)
    assert dec.global_possible_cover_status is CertStatus.UNKNOWN
    assert (dec.global_possible_cover_provenance["evidence_scope"]
            == "finite_left_mid_right_samples_only")


def test_hidden_narrow_gate_cut_abstains_instead_of_false_unreachable():
    cfg, scene, oracles, _, mc = _pre_rotated_narrow_gate()
    q0 = Pose2(np.array([-1.5, 0.0]), np.pi / 16.0)
    q1 = Pose2(np.array([1.5, 0.0]), np.pi / 16.0)

    # Independent evidence that the graph cut is false: the straight path at
    # the hidden legal orientation clears the true supports by about 9.9 mm.
    straight = PoseCurve((PoseSegment(SegmentKind.TRANSLATION, q0, q1),))
    verified = verify_curve(oracles, scene.workspace, straight,
                            cfg.query.eps_clear, cfg.orientation.theta_min)
    assert verified.certified, verified
    assert verified.min_clearance > cfg.query.eps_clear

    result = query(q0, q1, mc)
    assert result.status is PlanStatus.UNKNOWN
    assert result.report["reason"] == "possible_cut_is_not_a_global_certificate"
    assert any(a[0].startswith("possible_graph_disconnected")
               for a in result.ambiguity)


def _single_node_query_fixture(*, max_support_calls=5_000_000,
                               max_wall_seconds=360.0):
    """One deliberately permissive graph node backed by a real pair oracle."""
    cfg = make_cfg()
    cfg = replace(
        cfg,
        query=replace(cfg.query, max_support_calls=max_support_calls,
                      max_wall_seconds=max_wall_seconds),
    )
    scene = single_obstacle(r=0.5, workspace=(-2.0, 2.0, -2.0, 2.0))
    robot = ellipse_robot(0.2, 0.2)
    oracles = candidate_pairs(scene, robot, scene.workspace)
    component = SimpleNamespace(
        geometry=scene.workspace,
        representative=np.array([1.5, 0.0]),
    )
    mid_slice = SimpleNamespace(D_safe=(component,), D_possible=(component,))
    slab = SimpleNamespace(
        slab_id=0,
        interval=Interval(-0.1, 0.1),
        mid_slice=mid_slice,
        kind="regular",
    )
    dec = SimpleNamespace(
        slabs=[slab],
        global_possible_cover_status=CertStatus.UNKNOWN,
        global_possible_cover_provenance={"reason": "test_unproven"},
    )
    mc = MobilityCompiler(scene, robot, cfg, oracles, dec)
    mc.M_safe = nx.Graph()
    mc.M_safe.add_node("safe", slab=0, comp=0)
    mc.M_possible = nx.Graph()
    mc.M_possible.add_node("possible", slab=0, comp=0)

    def locate(_pose, side="safe"):
        if side == "safe":
            return "safe", slab, 0
        return "possible", slab, 0

    mc.locate = locate
    return mc


def test_zero_motion_query_emits_stationary_primitive_and_checks_contact():
    mc = _single_node_query_fixture()
    # Scene radius 0.5 + robot radius 0.2: x=0.7 is exact contact.
    q = Pose2(np.array([0.7, 0.0]), 0.0)
    curve, _, fail = lift(mc, ["safe"], q, q)
    assert fail is None
    assert len(curve.segments) == 1
    assert curve.segments[0].kind is SegmentKind.TRANSLATION

    result = query(q, q, mc)
    assert result.status is PlanStatus.UNKNOWN
    assert result.status is not PlanStatus.REACHABLE
    assert any(a[0] == "verify_failed" for a in result.ambiguity)


def test_lift_does_not_drop_tiny_nonzero_endpoint_rotation():
    mc = _single_node_query_fixture()
    start = Pose2(np.array([1.5, 0.0]), 5e-13)
    goal = Pose2(np.array([1.5, 0.0]), 0.0)

    curve, _, failure = lift(mc, ["safe"], start, goal)

    assert failure is None
    assert curve.segments
    first = curve.segments[0]
    assert first.kind is SegmentKind.ROTATION
    assert first.q0.theta == start.theta
    assert first.q1.theta == 0.0

    report = verify_curve(
        mc.oracles, mc.scene.workspace, curve,
        mc.cfg.query.eps_clear, mc.cfg.orientation.theta_min,
        expected_start=start, expected_goal=goal,
    )
    assert report.certified, report


def test_zero_support_budget_is_hard_and_returns_unknown():
    mc = _single_node_query_fixture(max_support_calls=0)
    q = Pose2(np.array([1.5, 0.0]), 0.0)
    before = sum(o.calls for o in mc.oracles)

    result = query(q, q, mc)

    assert result.status is PlanStatus.UNKNOWN
    assert result.report["reason"] == "query_support_budget_exhausted"
    assert result.report["query_support_calls"] == 0
    assert sum(o.calls for o in mc.oracles) == before
    exhausted = [a for a in result.ambiguity
                 if a[0] == "support_budget_exhausted"]
    assert exhausted and exhausted[0][1]["limit"] == 0


def test_verification_finishing_after_wall_deadline_cannot_return_reachable(
        monkeypatch):
    mc = _single_node_query_fixture(max_wall_seconds=1.0)
    q = Pose2(np.array([1.5, 0.0]), 0.0)
    query_module = importlib.import_module("gmc.mobility.query")
    real_verify = query_module.verify_curve
    clock = [0.0]

    monkeypatch.setattr(query_module.time, "perf_counter", lambda: clock[0])

    def verify_that_crosses_deadline(*args, **kwargs):
        report = real_verify(*args, **kwargs)
        assert report.certified
        clock[0] = 2.0
        return report

    monkeypatch.setattr(query_module, "verify_curve", verify_that_crosses_deadline)
    result = query_module.query(q, q, mc)

    assert result.status is PlanStatus.UNKNOWN
    assert result.report["reason"] == "query_wall_budget_exhausted"
    assert result.report["stage"] == "independent_verification"
    assert result.curve is None


def test_free_pose_missing_from_finite_possible_graph_is_unknown():
    mc = _single_node_query_fixture()
    q = Pose2(np.array([1.5, 0.0]), 0.0)
    mc.M_safe.clear()
    mc.M_possible.clear()
    mc.locate = lambda _pose, _side="safe": None

    result = query(q, q, mc)

    assert result.status is PlanStatus.UNKNOWN
    assert result.report["reason"] == \
        "finite_possible_graph_pose_coverage_missing"
    assert any(item[0] == "free_pose_not_located_in_finite_possible_graph"
               for item in result.ambiguity)
