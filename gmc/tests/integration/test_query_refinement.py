"""Query-local refinement, revision, budget, and provenance regressions."""
from dataclasses import replace
import importlib

import networkx as nx
import numpy as np
import pytest

from gmc.io.robot_io import ellipse_robot
from gmc.mobility.graph import compile_mobility
from gmc.mobility.query import PlanResult, _refinement_targets, query
from gmc.mobility.refinement import refine_compiler
from gmc.orientation.slab_builder import build_slabs
from gmc.spatial.bvh import candidate_pairs
from gmc.synth import empty_scene
from gmc.types import CertStatus, PlanStatus, Pose2, Result

from ..conftest import make_cfg


def _empty_compiler(*, rounds=2, support_calls=1000, max_depth=4,
                    mode="theorem"):
    cfg = make_cfg(mode=mode, initial_intervals=2)
    cfg = replace(
        cfg,
        orientation=replace(cfg.orientation, max_depth=max_depth),
        query=replace(
            cfg.query,
            max_support_calls=support_calls,
            max_refinement_rounds=rounds,
        ),
    )
    scene = empty_scene(workspace=(-1.0, 1.0, -1.0, 1.0))
    robot = ellipse_robot(0.2, 0.1)
    oracles = candidate_pairs(scene, robot, scene.workspace)
    decomposition = build_slabs(scene, robot, cfg, oracles)
    compiler = compile_mobility(
        scene, robot, cfg, oracles, decomposition,
    )
    return compiler


def _staged_attempt(monkeypatch, terminal=PlanStatus.REACHABLE):
    query_module = importlib.import_module("gmc.mobility.query")
    calls = []

    def staged(*_args, **kwargs):
        compiler = _args[2]
        calls.append(compiler.graph_revision)
        if len(calls) == 1:
            return PlanResult(
                PlanStatus.UNKNOWN,
                report={"reason": "injected_graph_ambiguity"},
                ambiguity=[("uncertain_slab", 0)],
            )
        return PlanResult(terminal, report={"reason": "injected_terminal"})

    monkeypatch.setattr(query_module, "_query_once", staged)
    return calls


def test_one_refinement_turns_unknown_into_terminal_result(monkeypatch):
    compiler = _empty_compiler(rounds=1)
    calls = _staged_attempt(monkeypatch)
    pose = Pose2(np.array([0.0, 0.0]), 0.2)

    result = query(pose, pose, compiler)

    assert result.status is PlanStatus.REACHABLE
    assert calls == [0, 1]
    assert compiler.graph_revision == 1
    assert result.report["refinement_rounds"] == 1
    record = result.report["refinement_trace"][0]
    assert record["revision_before"] == 0
    assert record["revision_after"] == 1
    assert max(b - a for a, b in record["child_intervals"]) < \
        record["parent_interval"][1] - record["parent_interval"][0]


@pytest.mark.parametrize(
    "terminal_status",
    [PlanStatus.REACHABLE, PlanStatus.UNREACHABLE],
    ids=["reachable", "unreachable"],
)
def test_terminal_result_commit_crossing_wall_deadline_abstains(
        monkeypatch, terminal_status):
    compiler = _empty_compiler(rounds=0)
    pose = Pose2(np.array([0.0, 0.0]), 0.2)
    query_module = importlib.import_module("gmc.mobility.query")
    clock = {"terminal": False, "terminal_reads": 0}
    wall_budget = float(compiler.cfg.query.max_wall_seconds)

    def controlled_clock():
        if not clock["terminal"]:
            return 0.0
        clock["terminal_reads"] += 1
        if clock["terminal_reads"] == 1:
            # common_report still records an in-budget completion time.
            return wall_budget - 1.0
        # The immediately following terminal commit is now out of budget.
        return wall_budget + 1.0

    def terminal_attempt(*_args, **_kwargs):
        clock["terminal"] = True
        return PlanResult(
            terminal_status,
            report={"reason": "injected_terminal"},
        )

    monkeypatch.setattr(query_module.time, "perf_counter", controlled_clock)
    monkeypatch.setattr(query_module, "_query_once", terminal_attempt)

    result = query_module.query(pose, pose, compiler)

    assert clock["terminal_reads"] >= 2
    assert result.status is PlanStatus.UNKNOWN
    assert result.report["reason"] == "query_wall_budget_exhausted"
    assert result.report["stage"] == "terminal_result_commit"


def test_zero_support_budget_performs_no_refinement(monkeypatch):
    compiler = _empty_compiler(rounds=1, support_calls=0)
    _staged_attempt(monkeypatch)
    pose = Pose2(np.array([0.0, 0.0]), 0.2)

    result = query(pose, pose, compiler)

    assert result.status is PlanStatus.UNKNOWN
    assert result.report["reason"] == "query_support_budget_exhausted"
    assert result.report["refinement_stop_reason"] == \
        "support_budget_exhausted"
    assert compiler.graph_revision == 0


def test_nonrefinable_target_returns_unknown_no_progress(monkeypatch):
    compiler = _empty_compiler(rounds=2, max_depth=0)
    _staged_attempt(monkeypatch)
    pose = Pose2(np.array([0.0, 0.0]), 0.2)

    result = query(pose, pose, compiler)

    assert result.status is PlanStatus.UNKNOWN
    assert result.report["refinement_stop_reason"] == "no_progress"
    assert compiler.graph_revision == 0
    assert result.ambiguity[-1][0] == "refinement_no_progress"


def test_runtime_graph_invariant_failure_is_internal_error():
    compiler = _empty_compiler(rounds=1)
    edge = next(iter(compiler.M_possible.edges))
    compiler.M_possible.remove_edge(*edge)
    pose = Pose2(np.array([0.0, 0.0]), 0.2)

    result = query(pose, pose, compiler)

    assert result.status is PlanStatus.INTERNAL_ERROR
    assert result.report["reason"] == "structural_invariant_failed"
    assert "I3_graph_nesting" in result.report["failed_invariants"]


def test_same_target_produces_deterministic_revision_and_split():
    left = _empty_compiler(rounds=1)
    right = _empty_compiler(rounds=1)

    left_outcome = refine_compiler(left, 0)
    right_outcome = refine_compiler(right, 0)

    assert left_outcome.changed and right_outcome.changed
    assert left_outcome.record["revision_after"] == 1
    assert left_outcome.record["child_intervals"] == \
        right_outcome.record["child_intervals"]
    assert left_outcome.record["new_slices_built"] == 2
    assert right_outcome.record["new_slices_built"] == 2
    # Parent endpoints and midpoint were reused; only quarter angles are new.
    assert len(left.decomposition.slice_cache) == 6


def test_refine_slab_keeps_parent_cache_atomic_on_builder_failure():
    compiler = _empty_compiler(rounds=1)
    decomposition = compiler.decomposition
    original_cache = dict(decomposition.slice_cache)
    original_builder = decomposition.slice_builder
    calls = 0

    def fail_on_second_new_slice(theta):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise FloatingPointError("injected second-child failure")
        # Build through the ordinary path for the first quarter slice.
        from gmc.spatial.slice_compiler import build_slice
        return build_slice(
            compiler.scene, compiler.robot, theta, compiler.cfg,
            compiler.oracles,
        )

    decomposition.slice_builder = fail_on_second_new_slice
    try:
        try:
            from gmc.orientation.slab_builder import refine_slab
            refine_slab(
                compiler.scene, compiler.robot, compiler.cfg,
                compiler.oracles, decomposition, 0,
            )
        except FloatingPointError:
            pass
        else:
            raise AssertionError("injected refinement failure was hidden")
    finally:
        decomposition.slice_builder = original_builder

    assert decomposition.revision == 0
    assert decomposition.slice_cache == original_cache
    assert len(decomposition.slice_cache) == len(original_cache)


def test_witness_failure_never_deletes_possible_adjacency(monkeypatch):
    compiler = _empty_compiler(rounds=1)
    graph_module = importlib.import_module("gmc.mobility.graph")

    def unresolved_rotation(*_args, **_kwargs):
        return Result(
            None, CertStatus.UNKNOWN, "injected_unresolved_rotation",
            uncertainty_sources=("test_injection",),
        )

    monkeypatch.setattr(graph_module, "rotate_witness", unresolved_rotation)
    outcome = refine_compiler(compiler, 0)

    assert outcome.changed
    assert compiler.M_safe.number_of_edges() == 0
    assert nx.is_connected(compiler.M_possible)
    assert compiler.M_possible.number_of_edges() > 0
    assert all(data["status"] == "POSSIBLE"
               for *_edge, data in compiler.M_possible.edges(data=True))


def test_strict_owner_and_all_exact_boundary_memberships():
    compiler = _empty_compiler(rounds=0)
    seam = Pose2(np.array([0.0, 0.0]), 0.0)
    boundary = Pose2(np.array([0.0, 0.0]), np.pi)

    assert compiler.locate(seam, "possible")[1].slab_id == 0
    assert [item[1].slab_id
            for item in compiler.locate_all(seam, "possible")] == [0, 1]
    assert compiler.locate(boundary, "possible")[1].slab_id == 1
    assert [item[1].slab_id
            for item in compiler.locate_all(boundary, "possible")] == [0, 1]


def test_query_uses_all_endpoint_memberships_before_any_formal_cut(
        monkeypatch):
    compiler = _empty_compiler(rounds=0)
    start = Pose2(np.array([-0.3, 0.0]), np.pi)
    goal = Pose2(np.array([0.3, 0.0]), np.pi)
    start_memberships = compiler.locate_all(start, "possible")
    goal_memberships = compiler.locate_all(goal, "possible")
    isolated = "injected_boundary_membership"
    compiler.M_possible.add_node(isolated, slab=0, comp=0)
    original = compiler.locate_all

    def memberships(pose, side="safe"):
        if side == "safe":
            return []
        if np.array_equal(pose.xy, start.xy):
            return [(isolated, compiler.decomposition.slabs[0], 0),
                    start_memberships[0]]
        if np.array_equal(pose.xy, goal.xy):
            return [goal_memberships[-1]]
        return original(pose, side)

    monkeypatch.setattr(compiler, "locate_all", memberships)
    result = query(start, goal, compiler)

    # The first start membership is isolated, but another admissible exact
    # boundary membership connects.  A first-hit locate implementation would
    # have emitted a false formal cut here.
    assert result.status is PlanStatus.UNKNOWN
    assert result.status is not PlanStatus.UNREACHABLE


@pytest.mark.parametrize(
    "formal_failures",
    [(), ("I3_graph_nesting",)],
    ids=["otherwise_unreachable", "otherwise_internal_error"],
)
def test_formal_cut_invariant_replay_crossing_wall_deadline_abstains(
        monkeypatch, formal_failures):
    compiler = _empty_compiler(rounds=0)
    start = Pose2(np.array([0.0, 0.0]), 0.2)
    goal = Pose2(np.array([0.0, 0.0]), -0.2)
    compiler.M_safe.remove_edges_from(list(compiler.M_safe.edges))
    compiler.M_possible.remove_edges_from(list(compiler.M_possible.edges))

    query_module = importlib.import_module("gmc.mobility.query")
    refinement_module = importlib.import_module("gmc.mobility.refinement")
    clock = [0.0]
    checks = []

    monkeypatch.setattr(query_module.time, "perf_counter", lambda: clock[0])

    def invariant_replay_that_crosses_deadline(_compiler):
        checks.append(len(checks) + 1)
        if len(checks) == 1:
            # Allow the ordinary entry invariant check to reach the formal
            # possible-graph cut.  The cut-specific replay is the expensive
            # operation under test.
            return ()
        assert len(checks) == 2
        clock[0] = float(compiler.cfg.query.max_wall_seconds) + 1.0
        return formal_failures

    monkeypatch.setattr(
        refinement_module,
        "structural_invariant_failures",
        invariant_replay_that_crosses_deadline,
    )

    result = query_module.query(start, goal, compiler)

    assert checks == [1, 2]
    assert result.status is PlanStatus.UNKNOWN
    assert result.report["reason"] == "query_wall_budget_exhausted"
    assert result.report["stage"] == "formal_cut_invariant_check"


def test_compile_rejects_truncated_interval_possible_components():
    compiler = _empty_compiler(rounds=0)
    decomposition = compiler.decomposition
    slab = decomposition.slabs[0]
    truncated_cover = replace(slab.cover_slice, D_possible=())
    decomposition.slabs[0] = replace(slab, cover_slice=truncated_cover)

    try:
        compile_mobility(
            compiler.scene, compiler.robot, compiler.cfg, compiler.oracles,
            decomposition,
        )
    except ValueError as exc:
        assert "possible_components_complete" in str(exc)
    else:
        raise AssertionError("truncated interval possible cover was accepted")


def test_uncertified_possible_cut_targets_missing_interval_cover_first():
    compiler = _empty_compiler(rounds=1, mode="prototype")
    start = Pose2(np.array([-0.3, 0.0]), 0.2)
    goal = Pose2(np.array([0.3, 0.0]), 0.2)
    start_node = compiler.locate(start, "possible")[0]
    goal_node = compiler.locate(goal, "possible")[0]
    ambiguity = [(
        "possible_graph_disconnected_without_certified_global_cover",
        {"start_nodes": [start_node], "goal_nodes": [goal_node]},
    )]

    targets = _refinement_targets(
        compiler, start, goal, ambiguity,
    )

    # Prototype slabs have no interval theorem, so the widest/lowest missing
    # cover is refined instead of treating the finite-sample cut as terminal.
    assert targets
    assert targets[0] == 0
