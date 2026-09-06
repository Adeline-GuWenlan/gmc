"""Mobility/query semantics for certified orientation-interval covers.

These are end-to-end regressions for the P3 bridge: interval projections,
not event-free midpoint predicates, carry the global possible-space theorem.
"""
from dataclasses import replace

import networkx as nx
import numpy as np
import pytest

import gmc.mobility.graph as graph_module
from gmc.io.robot_io import ellipse_robot
from gmc.mobility.graph import compile_mobility
from gmc.mobility.query import query
from gmc.mobility.witness import PoseCurve, PoseSegment, SegmentKind
from gmc.orientation.slab_builder import build_slabs
from gmc.spatial.bvh import candidate_pairs
from gmc.synth import single_door
from gmc.types import (CertStatus, GaussianSupport2D, PlanStatus, Pose2,
                       Result, RobotModel2D)
from gmc.verification.invariants import (check_graph_nesting,
                                         check_no_silent_fallback,
                                         check_witness_ownership)
from gmc.verification.path import verify_curve

from ..conftest import make_cfg


@pytest.fixture(scope="module")
def hidden_gate_cover():
    """A legal orientation hidden inside an event-uncertain coarse slab."""
    cfg = make_cfg(
        mode="theorem", eps_pair=2e-2, theta_min=2e-3,
        initial_intervals=8,
    )
    # Stop event-driven refinement deliberately.  The interval certificate
    # must remain sound even though some slabs are classified uncertain.
    cfg = replace(
        cfg,
        orientation=replace(cfg.orientation, max_depth=0),
    )
    scene = single_door(
        0.42, wall_t=0.18, workspace=(-2.0, 2.0, -1.2, 1.2),
    )
    base = ellipse_robot(0.5, 0.2).supports[0]
    alpha = -np.pi / 16.0
    rotation = np.array([
        [np.cos(alpha), -np.sin(alpha)],
        [np.sin(alpha), np.cos(alpha)],
    ])
    body = GaussianSupport2D(
        mean=base.mean,
        covariance=rotation @ base.covariance @ rotation.T,
        level=base.level,
        primitive_id=base.primitive_id,
    )
    robot = RobotModel2D((body,), name="pre_rotated_ellipse")
    oracles = candidate_pairs(scene, robot, scene.workspace)
    decomposition = build_slabs(scene, robot, cfg, oracles)
    return cfg, scene, robot, oracles, decomposition


def test_hidden_narrow_gate_stays_in_possible_cover(hidden_gate_cover):
    cfg, scene, robot, oracles, decomposition = hidden_gate_cover
    assert decomposition.global_possible_cover_status is CertStatus.CERTIFIED
    assert any(slab.kind == "uncertain" for slab in decomposition.slabs)
    assert all(slab.cover_slice is not None
               and slab.cover_slice.status is CertStatus.CERTIFIED
               for slab in decomposition.slabs)

    mc = compile_mobility(scene, robot, cfg, oracles, decomposition)
    q0 = Pose2(np.array([-1.5, 0.0]), np.pi / 16.0)
    q1 = Pose2(np.array([1.5, 0.0]), np.pi / 16.0)

    # Independent continuous evidence that this hidden orientation really is
    # traversable.  The interval possible graph must not cut it away.
    straight = PoseCurve((PoseSegment(SegmentKind.TRANSLATION, q0, q1),))
    verified = verify_curve(
        oracles, scene.workspace, straight,
        cfg.query.eps_clear, cfg.orientation.theta_min,
    )
    assert verified.certified, verified

    loc0 = mc.locate(q0, "possible")
    loc1 = mc.locate(q1, "possible")
    assert loc0 is not None and loc1 is not None
    assert nx.has_path(mc.M_possible, loc0[0], loc1[0])
    assert query(q0, q1, mc).status is not PlanStatus.UNREACHABLE

    # SAFE nodes may now come from the slab-wide proof even where the event
    # detector does not claim regularity.
    uncertain_ids = {
        slab.slab_id for slab in decomposition.slabs
        if slab.kind == "uncertain"
    }
    assert any(data["slab"] in uncertain_ids
               for _, data in mc.M_safe.nodes(data=True))
    for slab in decomposition.slabs:
        safe_nodes = sum(
            data["slab"] == slab.slab_id
            for _, data in mc.M_safe.nodes(data=True)
        )
        possible_nodes = sum(
            data["slab"] == slab.slab_id
            for _, data in mc.M_possible.nodes(data=True)
        )
        assert safe_nodes == len(slab.cover_slice.D_safe)
        assert possible_nodes == len(slab.cover_slice.D_possible)
    assert check_graph_nesting(mc)
    assert check_no_silent_fallback(mc)


def test_safe_cover_edges_still_require_and_replay_rotation_witness(
        hidden_gate_cover, monkeypatch):
    cfg, scene, robot, oracles, decomposition = hidden_gate_cover
    normal = compile_mobility(scene, robot, cfg, oracles, decomposition)
    assert normal.M_safe.number_of_edges() > 0
    assert check_witness_ownership(normal, replay=True)

    # Component overlap alone is never promoted to SAFE.  When the independent
    # continuous checker abstains, all upper-bound transitions survive as
    # POSSIBLE while the corresponding SAFE edges disappear.
    def unresolved_rotation(*_args, **_kwargs):
        return Result(
            None, CertStatus.UNKNOWN, "injected_unresolved_rotation",
            uncertainty_sources=("test_injection",),
        )

    monkeypatch.setattr(graph_module, "rotate_witness", unresolved_rotation)
    replay_denied = compile_mobility(
        scene, robot, cfg, oracles, decomposition,
    )
    assert replay_denied.M_safe.number_of_edges() == 0
    assert replay_denied.M_possible.number_of_edges() > 0
    assert all(data["status"] == "POSSIBLE"
               for _, _, data in replay_denied.M_possible.edges(data=True))


def test_certified_interval_cover_closes_a_sealed_door():
    cfg = make_cfg(
        mode="theorem", eps_pair=1e-2, theta_min=1e-2,
        initial_intervals=16,
    )
    # This gap is far below the ellipse's 0.4 m minimum width.  Sixteen
    # interval covers are tight enough that the upper graph preserves the
    # separating wall rather than opening an approximation artefact.
    scene = single_door(
        0.05, wall_t=0.18, workspace=(-2.0, 2.0, -1.2, 1.2),
    )
    robot = ellipse_robot(0.5, 0.2)
    oracles = candidate_pairs(scene, robot, scene.workspace)
    decomposition = build_slabs(scene, robot, cfg, oracles)
    mc = compile_mobility(scene, robot, cfg, oracles, decomposition)

    start = Pose2(np.array([-1.5, 0.0]), 0.0)
    goal = Pose2(np.array([1.5, 0.0]), 0.0)
    result = query(start, goal, mc)

    assert decomposition.global_possible_cover_status is CertStatus.CERTIFIED
    assert not any(slab.predicates.certifies_no_event
                   for slab in decomposition.slabs)
    assert result.status is PlanStatus.UNREACHABLE
    assert result.report["global_possible_cover"]["evidence_scope"] == \
        "certified_full_circle_interval_cover"

    # Simulate corrupt/stale aggregate metadata.  Losing even one per-slab
    # certificate is now rejected at the compile-time decomposition binding
    # gate; it cannot produce an executable graph at all.
    decomposition.slabs[0] = replace(
        decomposition.slabs[0], cover_slice=None,
    )
    with pytest.raises(ValueError, match="decomposition structure mismatch"):
        compile_mobility(scene, robot, cfg, oracles, decomposition)
