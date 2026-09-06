"""M7 three-valued query-and-refine engine + path lifting (Guide §10, §7.4
of the design manual).

REACHABLE only after independent continuous verification; UNREACHABLE only
from a possible-graph disconnection; everything else refines within budget or
returns UNKNOWN with the ambiguity set (I5)."""
import time
from contextlib import contextmanager
from dataclasses import dataclass, field

import networkx as nx
import numpy as np

from ..budget import BudgetExceeded
from ..types import CertStatus, PlanStatus, Pose2, canonical_angle
from ..verification.path import VerifyReport, verify_curve
from .graph import MobilityCompiler, node_id
from .lineage import (certified_cover_slice, component_slice,
                      slab_has_safe_components)
from .witness import (PoseCurve, PoseSegment, SegmentKind, rotate_witness,
                      translate_witness)


@dataclass
class PlanResult:
    status: PlanStatus
    curve: PoseCurve | None = None
    graph_path: list = field(default_factory=list)
    clearance_lower_bound: float | None = None
    report: dict = field(default_factory=dict)
    ambiguity: list = field(default_factory=list)
    segment_provenance: list = field(default_factory=list)
    verification_report: VerifyReport | None = None


@dataclass(frozen=True)
class SegmentProvenance:
    """Why a lifted segment exists and which slab/edge can be refined.

    Intra-slab translation is intentionally distinct from the graph edge that
    happens to follow it.  A failed dynamic translation therefore cannot be
    misinterpreted as permission to delete a SAFE/POSSIBLE transition.
    """

    kind: str
    slab_id: int | None = None
    edge: tuple[str, str] | None = None
    endpoint: str | None = None


@dataclass
class _QuerySupportBudget:
    """One combined hard budget for value and point support evaluations."""

    limit: int
    used: int = 0

    def charge(self, kind: str, amount: int = 1) -> None:
        if kind not in ("support_value_evals", "support_point_evals"):
            return
        amount = int(amount)
        if self.used + amount > self.limit:
            raise BudgetExceeded("query_support_evals", self.limit,
                                 self.used, amount)
        self.used += amount


class _QueryLedgerProxy:
    """Enforce the query budget while preserving an oracle's run ledger."""

    def __init__(self, budget: _QuerySupportBudget, delegate):
        self.budget = budget
        self.delegate = delegate

    def charge(self, kind: str, amount: int = 1, *, phase=None) -> None:
        self.budget.charge(kind, amount)
        if self.delegate is not None:
            if phase is None:
                self.delegate.charge(kind, amount)
            else:
                self.delegate.charge(kind, amount, phase=phase)


@contextmanager
def _bounded_query_support(oracles, limit: int):
    """Temporarily meter every pair oracle used by lifting/verification.

    PairOracle charges its ledger before evaluating a batch.  Installing the
    proxy therefore prevents the batch from running at all when it would
    exceed the combined value+point query budget.
    """
    meter = _QuerySupportBudget(int(limit))
    originals = []
    for oracle in oracles:
        if not hasattr(oracle, "ledger"):
            continue
        originals.append((oracle, oracle.ledger))
        oracle.ledger = _QueryLedgerProxy(meter, oracle.ledger)
    try:
        yield meter
    finally:
        for oracle, original in originals:
            oracle.ledger = original


def _edge_arc(mc: MobilityCompiler, na: str, nb: str):
    """Certified anchor list and the arc oriented na -> nb."""
    data = mc.M_safe.get_edge_data(na, nb)
    anchors, th_from, th_to = data["witness"]
    slab_a = mc.M_safe.nodes[na]["slab"]
    mid_a = mc.decomposition.slabs[slab_a].interval.midpoint
    if canonical_angle(th_from) == canonical_angle(mid_a):
        return [np.asarray(t) for t in anchors], th_from, th_to
    return [np.asarray(t) for t in anchors], th_to, th_from


def lift(mc: MobilityCompiler, graph_path: list, start: Pose2, goal: Pose2):
    """Translate-rotate-translate chain through the safe graph path, with a
    dynamic-programming choice among each edge's certified anchors (§9.1
    intra-slab movement): the crossing traverse lands wherever a feasible
    in/out anchor pair exists — typically a wide-orientation slab — instead
    of being pinned to event-adjacent slabs.

    Returns ``(curve, provenance, fail)``.  ``provenance`` maps each segment
    to its actual owner; it is not a blanket graph-edge invalidation map.
    """
    oracles = mc.oracles
    theta_min = mc.cfg.orientation.theta_min
    floor = mc.cfg.query.eps_clear

    def comp_geom(node):
        n = mc.M_safe.nodes[node]
        slab = mc.decomposition.slabs[n["slab"]]
        return component_slice(slab).D_safe[n["comp"]].geometry

    def mid_angle(node):
        return mc.decomposition.slabs[mc.M_safe.nodes[node]["slab"]].interval.midpoint

    def slab_id(node):
        return int(mc.M_safe.nodes[node]["slab"])

    def rot(t, th0, th1):
        if canonical_angle(th1) == canonical_angle(th0):
            return []
        res = rotate_witness(t, th0, th1, oracles, theta_min, floor=floor)
        return None if res.status is not CertStatus.CERTIFIED else [res.value]

    def trans(node, p0, p1, theta):
        if np.array_equal(np.asarray(p1), np.asarray(p0)):
            return []
        res = translate_witness(comp_geom(node), p0, p1, theta, oracles,
                                floor=floor)
        return None if res.status is not CertStatus.CERTIFIED else [res.value]

    th0 = _nearest_lift(float(start.theta), mid_angle(graph_path[0]))
    head = rot(np.asarray(start.xy, float), float(start.theta), th0)
    if head is None:
        return None, [], SegmentProvenance(
            "endpoint", slab_id(graph_path[0]), endpoint="start_rotation",
        )

    edges = list(zip(graph_path[:-1], graph_path[1:]))
    # states: (anchor_pos, theta, segments, segment provenance) after each edge
    states = [(np.asarray(start.xy, float), th0,
               list(head), [SegmentProvenance(
                   "endpoint", slab_id(graph_path[0]),
                   endpoint="start_rotation",
               )] * len(head))]
    for na, nb in edges:
        anchors, th_from, th_to = _edge_arc(mc, na, nb)
        new_states = []
        last_failure = None
        for a in anchors:
            for pos, cth, segs, provenance in states:
                th_from_l = _nearest_lift(cth, th_from)
                leg = trans(na, pos, a, cth)
                if leg is None:
                    last_failure = SegmentProvenance(
                        "intra_slab_translation", slab_id(na),
                    )
                    continue
                pre = rot(a, cth, th_from_l)
                if pre is None:
                    last_failure = SegmentProvenance(
                        "intra_slab_rotation", slab_id(na),
                    )
                    continue
                arc = rot(a, th_from_l, th_from_l + (th_to - th_from))
                if arc is None:
                    last_failure = SegmentProvenance(
                        "safe_edge_rotation", slab_id(na), edge=(na, nb),
                    )
                    continue
                add = leg + pre + arc
                add_provenance = (
                    [SegmentProvenance(
                        "intra_slab_translation", slab_id(na),
                    )] * len(leg)
                    + [SegmentProvenance(
                        "intra_slab_rotation", slab_id(na),
                    )] * len(pre)
                    + [SegmentProvenance(
                        "safe_edge_rotation", slab_id(na), edge=(na, nb),
                    )] * len(arc)
                )
                new_states.append((np.asarray(a), th_from_l + (th_to - th_from),
                                   segs + add,
                                   provenance + add_provenance))
                break          # keep first feasible predecessor per anchor
        if not new_states:
            return (None, [], last_failure or SegmentProvenance(
                "safe_edge_rotation", slab_id(na), edge=(na, nb),
            ))
        states = new_states

    goal_xy = np.asarray(goal.xy, float)
    last = graph_path[-1]
    last_failure = None
    for pos, cth, segs, provenance in states:
        tail_t = trans(last, pos, goal_xy, cth)
        if tail_t is None:
            last_failure = SegmentProvenance(
                "endpoint", slab_id(last), endpoint="goal_translation",
            )
            continue
        th_g = _nearest_lift(cth, float(goal.theta))
        tail_r = rot(goal_xy, cth, th_g)
        if tail_r is None:
            last_failure = SegmentProvenance(
                "endpoint", slab_id(last), endpoint="goal_rotation",
            )
            continue
        add = tail_t + tail_r
        final_segments = list(segs + add)
        final_provenance = provenance + (
            [SegmentProvenance(
                "endpoint", slab_id(last), endpoint="goal_translation",
            )] * len(tail_t)
            + [SegmentProvenance(
                "endpoint", slab_id(last), endpoint="goal_rotation",
            )] * len(tail_r)
        )
        if not final_segments:
            # A zero-motion query still needs an independently verified point
            # certificate.  Represent it as a degenerate fixed-theta
            # translation so the verifier checks workspace and clearance;
            # an empty curve would otherwise certify a colliding pose
            # vacuously with +inf clearance.
            q = Pose2(np.array(start.xy, dtype=float, copy=True),
                      float(start.theta))
            final_segments.append(PoseSegment(SegmentKind.TRANSLATION, q, q))
            final_provenance.append(SegmentProvenance(
                "endpoint", slab_id(last), endpoint="stationary",
            ))
        return (PoseCurve(segments=tuple(final_segments)),
                final_provenance, None)
    return (None, [], last_failure or SegmentProvenance(
        "endpoint", slab_id(last), endpoint="goal_translation",
    ))


def _nearest_lift(base: float, target_mod: float) -> float:
    """Unwrapped angle congruent to target_mod nearest to base."""
    k = round((base - target_mod) / (2 * np.pi))
    return target_mod + 2 * np.pi * k


def query(start: Pose2, goal: Pose2, mc: MobilityCompiler,
          max_paths: int = 20) -> PlanResult:
    """Budgeted SAFE -> verify -> local-refine -> retry query loop.

    A refinement never removes an upper-graph edge merely because a witness or
    lifted translation failed.  It bisects one implicated orientation slab and
    recompiles the complete derived graph from conservative geometry.  Thus a
    prototype graph may gain a verified REACHABLE path, while UNREACHABLE is
    still reserved for a theorem-mode full-circle interval cover.
    """
    budget = mc.cfg.query
    t0 = time.perf_counter()
    deadline = t0 + float(budget.max_wall_seconds)
    all_ambiguity: list = []
    refinement_trace: list = []

    def elapsed() -> float:
        return time.perf_counter() - t0

    def wall_exhausted() -> bool:
        return time.perf_counter() >= deadline

    def common_report(support_calls: int) -> dict:
        return {
            "wall_s": elapsed(),
            "wall_budget_s": float(budget.max_wall_seconds),
            "query_support_calls": int(support_calls),
            "query_support_budget": int(budget.max_support_calls),
            "refinement_rounds": len(refinement_trace),
            "graph_revision": int(getattr(mc, "graph_revision", 0)),
            "refinement_trace": list(refinement_trace),
        }

    def wall_result(stage: str, support_calls: int,
                    *, paths_tried: int = 0) -> PlanResult:
        detail = {
            "stage": stage,
            "paths_tried": int(paths_tried),
            **common_report(support_calls),
        }
        return PlanResult(
            PlanStatus.UNKNOWN,
            report={"reason": "query_wall_budget_exhausted", **detail},
            ambiguity=[
                *all_ambiguity, ("wall_budget_exhausted", detail),
            ],
        )

    def internal_error(reason: str, support_calls: int, **details) -> PlanResult:
        return PlanResult(
            PlanStatus.INTERNAL_ERROR,
            report={
                "reason": reason,
                **details,
                **common_report(support_calls),
            },
            ambiguity=list(all_ambiguity),
        )

    from ..orientation.provenance import decomposition_binding_failures
    from ..spatial.bvh import validate_candidate_oracles
    if getattr(mc, "structural_contract", False):
        binding_failures = decomposition_binding_failures(
            mc.decomposition, mc.scene, mc.robot, mc.cfg, mc.oracles,
        )
        if binding_failures:
            return internal_error(
                "decomposition_input_binding_failed", 0,
                failed_bindings=list(binding_failures),
            )
    mc.oracles = validate_candidate_oracles(
        mc.scene, mc.robot, mc.scene.workspace, mc.oracles,
    )
    if wall_exhausted():
        return wall_result("oracle_provenance_validation", 0)

    max_support_calls = int(budget.max_support_calls)
    max_rounds = int(getattr(budget, "max_refinement_rounds", 0))
    if max_support_calls < 0:
        raise ValueError("max_support_calls must be non-negative")
    if max_rounds < 0:
        raise ValueError("max_refinement_rounds must be non-negative")

    from .refinement import (
        RefinementInvariantError,
        refine_compiler,
        structural_invariant_failures,
    )

    try:
        with _bounded_query_support(mc.oracles,
                                    max_support_calls) as support_meter:
            for attempt in range(max_rounds + 1):
                if wall_exhausted():
                    return wall_result("before_query_attempt",
                                       support_meter.used)

                if getattr(mc, "structural_contract", False):
                    try:
                        failures = structural_invariant_failures(mc)
                    except Exception as exc:
                        return internal_error(
                            "structural_invariant_check_failed",
                            support_meter.used,
                            error_type=type(exc).__name__, error=str(exc),
                        )
                    # A stale aggregate global-cover flag is certificate data:
                    # the possible-cut logic below safely downgrades it to
                    # UNKNOWN.  Other graph contradictions are program errors.
                    stale_cover = _stale_global_cover(mc)
                    if stale_cover:
                        failures = tuple(
                            name for name in failures
                            if name not in {
                                "I3_graph_nesting",
                                "I5_no_silent_fallback",
                            }
                        )
                    if failures:
                        return internal_error(
                            "structural_invariant_failed",
                            support_meter.used,
                            failed_invariants=list(failures),
                        )

                result = _query_once(
                    start, goal, mc, max_paths=max_paths,
                    wall_exhausted=wall_exhausted,
                    wall_result=lambda stage, paths_tried=0: wall_result(
                        stage, support_meter.used, paths_tried=paths_tried,
                    ),
                    elapsed=elapsed,
                    support_meter=support_meter,
                )
                current_ambiguity = list(result.ambiguity)
                all_ambiguity.extend(current_ambiguity)
                result.ambiguity = list(all_ambiguity)
                result.report.update(common_report(support_meter.used))
                if result.status is not PlanStatus.UNKNOWN:
                    # Commit no terminal claim after its wall budget.  Even
                    # when the attempt checked its own expensive stages, the
                    # final report assembly above can race the deadline.
                    if wall_exhausted():
                        return wall_result(
                            "terminal_result_commit", support_meter.used,
                        )
                    return result
                if result.report.get("reason") == \
                        "query_wall_budget_exhausted":
                    return result

                if attempt >= max_rounds:
                    result.report["refinement_stop_reason"] = \
                        "round_budget_exhausted"
                    result.ambiguity.append((
                        "refinement_budget_exhausted",
                        {"limit": max_rounds},
                    ))
                    return result

                if support_meter.used >= max_support_calls:
                    result.report["refinement_stop_reason"] = \
                        "support_budget_exhausted"
                    result.report["reason"] = \
                        "query_support_budget_exhausted"
                    result.ambiguity.append((
                        "support_budget_exhausted",
                        {
                            "kind": "query_support_evals",
                            "used": support_meter.used,
                            "requested": 0,
                            "limit": max_support_calls,
                        },
                    ))
                    return result

                targets = _refinement_targets(
                    mc, start, goal, current_ambiguity,
                )
                progress = None
                for target in targets:
                    revision_before = int(getattr(mc, "graph_revision", 0))
                    try:
                        outcome = refine_compiler(mc, target)
                    except RefinementInvariantError as exc:
                        return internal_error(
                            "refined_graph_invariant_failed",
                            support_meter.used,
                            failed_invariants=list(exc.failures),
                            target_slab_id=target,
                        )
                    except BudgetExceeded:
                        raise
                    except Exception as exc:
                        # Refinement code/numeric machinery must not disappear
                        # into ordinary geometric UNKNOWN.
                        return internal_error(
                            "refinement_internal_error",
                            support_meter.used,
                            target_slab_id=target,
                            error_type=type(exc).__name__, error=str(exc),
                        )
                    if not outcome.changed:
                        continue
                    record = dict(outcome.record)
                    parent_width = (record["parent_interval"][1]
                                    - record["parent_interval"][0])
                    child_widths = [b - a for a, b
                                    in record["child_intervals"]]
                    revision_after = int(getattr(mc, "graph_revision", 0))
                    if (revision_after <= revision_before
                            or not child_widths
                            or max(child_widths) >= parent_width):
                        return internal_error(
                            "refinement_made_no_progress",
                            support_meter.used,
                            target_slab_id=target,
                            refinement_record=record,
                        )
                    refinement_trace.append(record)
                    progress = record
                    break

                if progress is None:
                    result.report["refinement_stop_reason"] = "no_progress"
                    result.ambiguity.append((
                        "refinement_no_progress",
                        {"candidate_slab_ids": targets},
                    ))
                    return result
                if wall_exhausted():
                    return wall_result("refinement_rebuild",
                                       support_meter.used)
    except BudgetExceeded as exc:
        exhausted = {
            "kind": exc.kind,
            "used": exc.used,
            "requested": exc.requested,
            "limit": exc.limit,
        }
        all_ambiguity.append(("support_budget_exhausted", exhausted))
        return PlanResult(
            PlanStatus.UNKNOWN,
            report={
                "reason": "query_support_budget_exhausted",
                **common_report(exc.used),
            },
            ambiguity=all_ambiguity,
        )

    # The bounded loop always returns.  Reaching this branch is a program
    # error, not an admissible geometric UNKNOWN.
    return internal_error("query_loop_fell_through", 0)


def _query_once(start: Pose2, goal: Pose2, mc: MobilityCompiler, *,
                max_paths: int, wall_exhausted, wall_result, elapsed,
                support_meter) -> PlanResult:
    """One immutable graph attempt; refinement is owned by :func:`query`."""
    budget = mc.cfg.query
    ambiguity = []
    loc_s_all = _pose_locations(mc, start, "safe")
    loc_g_all = _pose_locations(mc, goal, "safe")
    loc_s = loc_s_all[0] if loc_s_all else None
    loc_g = loc_g_all[0] if loc_g_all else None
    if wall_exhausted():
        return wall_result("safe_pose_location")
    connected_safe_pairs = []
    for source in loc_s_all:
        for target in loc_g_all:
            if (source[0] in mc.M_safe and target[0] in mc.M_safe
                    and nx.has_path(mc.M_safe, source[0], target[0])):
                connected_safe_pairs.append((
                    nx.shortest_path_length(
                        mc.M_safe, source[0], target[0], weight="weight",
                    ),
                    source[0], target[0],
                ))
    if connected_safe_pairs:
        if wall_exhausted():
            return wall_result("safe_connectivity")
        paths_tried = 0
        seen_paths = set()
        exhausted_paths = False
        for _, source, target in sorted(connected_safe_pairs):
            generator = nx.shortest_simple_paths(
                mc.M_safe, source, target, weight="weight",
            )
            for path in generator:
                signature = tuple(path)
                if signature in seen_paths:
                    continue
                seen_paths.add(signature)
                if wall_exhausted():
                    return wall_result(
                        "safe_path_enumeration", paths_tried=paths_tried,
                    )
                if paths_tried >= max_paths:
                    ambiguity.append((
                        "path_budget_exhausted", paths_tried,
                    ))
                    exhausted_paths = True
                    break
                paths_tried += 1
                curve, segment_provenance, failure = lift(
                    mc, path, start, goal,
                )
                if wall_exhausted():
                    return wall_result(
                        "path_lifting", paths_tried=paths_tried,
                    )
                if curve is None:
                    ambiguity.append(("lift_failed", failure))
                    continue
                report = verify_curve(
                    mc.oracles, mc.scene.workspace, curve,
                    budget.eps_clear, mc.cfg.orientation.theta_min,
                    expected_start=start, expected_goal=goal,
                )
                if wall_exhausted():
                    return wall_result(
                        "independent_verification",
                        paths_tried=paths_tried,
                    )
                if report.certified:
                    return PlanResult(
                        PlanStatus.REACHABLE, curve=curve, graph_path=path,
                        clearance_lower_bound=report.min_clearance,
                        report={
                            "verify": report.reason,
                            "paths_tried": paths_tried,
                            "wall_s": elapsed(),
                            "query_support_calls": support_meter.used,
                        },
                        ambiguity=ambiguity,
                        segment_provenance=list(segment_provenance),
                        verification_report=report,
                    )
                provenance = (
                    segment_provenance[report.failed_segment]
                    if report.failed_segment is not None
                    and 0 <= report.failed_segment < len(segment_provenance)
                    else None
                )
                if (provenance is not None
                        and provenance.kind == "safe_edge_rotation"):
                    return PlanResult(
                        PlanStatus.INTERNAL_ERROR,
                        report={
                            "reason": (
                                "certified_safe_edge_failed_verification"
                            ),
                            "verify_reason": report.reason,
                            "segment_provenance": provenance,
                        },
                        ambiguity=ambiguity,
                    )
                ambiguity.append((
                    "verify_failed", report.reason, provenance,
                ))
            if exhausted_paths:
                break

    if wall_exhausted():
        return wall_result("before_possible_graph_analysis")
    loc_sp_all = _pose_locations(mc, start, "possible")
    loc_gp_all = _pose_locations(mc, goal, "possible")
    loc_sp = loc_sp_all[0] if loc_sp_all else None
    loc_gp = loc_gp_all[0] if loc_gp_all else None
    if wall_exhausted():
        return wall_result("possible_pose_location")
    if loc_sp is None or loc_gp is None:
        return _classify_unlocated_poses(
            start, goal, mc, loc_sp, loc_gp, ambiguity,
            wall_exhausted, wall_result,
        )

    connected_possible_pairs = [
        (source[0], target[0])
        for source in loc_sp_all
        for target in loc_gp_all
        if nx.has_path(mc.M_possible, source[0], target[0])
    ]
    possible_connected = bool(connected_possible_pairs)
    if wall_exhausted():
        return wall_result("possible_connectivity")
    if not possible_connected:
        start_components = {
            source[0]: len(nx.node_connected_component(
                mc.M_possible, source[0],
            ))
            for source in loc_sp_all
        }
        cut_candidate = {
            "start_nodes": [located[0] for located in loc_sp_all],
            "goal_nodes": [located[0] for located in loc_gp_all],
            "start_component_sizes": start_components,
            "all_endpoint_node_pairs_disconnected": True,
        }
        recorded_cover_status = getattr(
            mc.decomposition, "global_possible_cover_status",
            CertStatus.UNKNOWN,
        )
        cover_provenance = getattr(
            mc.decomposition, "global_possible_cover_provenance",
            {
                "reason": "global_possible_cover_status_missing",
                "evidence_scope": "unknown",
            },
        )
        slabs = tuple(getattr(mc.decomposition, "slabs", ()))
        all_interval_covers = bool(slabs) and all(
            certified_cover_slice(slab) is not None for slab in slabs
        )
        theorem_mode = mc.cfg.pair_approx.certificate_mode == "theorem"
        cover_status = (
            CertStatus.CERTIFIED
            if theorem_mode
            and recorded_cover_status is CertStatus.CERTIFIED
            and all_interval_covers
            else CertStatus.UNKNOWN
        )
        if cover_status is CertStatus.CERTIFIED:
            from .refinement import structural_invariant_failures

            formal_failures = structural_invariant_failures(mc)
            # This replay can be substantially more expensive than the graph
            # cut itself.  Its result, whether a clean certificate or a
            # structural contradiction, is not admissible after the query
            # deadline: wall-budget exhaustion must fail closed first.
            if wall_exhausted():
                return wall_result("formal_cut_invariant_check")
            if formal_failures:
                return PlanResult(
                    PlanStatus.INTERNAL_ERROR,
                    report={
                        "reason": "formal_cut_invariant_failed",
                        "failed_invariants": list(formal_failures),
                        "cut_candidate": cut_candidate,
                    },
                    ambiguity=ambiguity,
                )
            return PlanResult(
                PlanStatus.UNREACHABLE,
                report={
                    "cut_certificate": cut_candidate,
                    "global_possible_cover": cover_provenance,
                },
                ambiguity=ambiguity,
            )
        ambiguity.append((
            "possible_graph_disconnected_without_certified_global_cover",
            cut_candidate,
        ))
        return PlanResult(
            PlanStatus.UNKNOWN,
            report={
                "reason": "possible_cut_is_not_a_global_certificate",
                "possible_cut_candidate": cut_candidate,
                "global_possible_cover_status": cover_status.name,
                "global_possible_cover": cover_provenance,
                "all_slab_interval_covers_certified": all_interval_covers,
            },
            ambiguity=ambiguity,
        )
    best_possible_pair = min(
        connected_possible_pairs,
        key=lambda pair: (
            nx.shortest_path_length(
                mc.M_possible, pair[0], pair[1], weight="weight",
            ),
            pair,
        ),
    )
    ambiguity.extend(_query_ambiguity(
        mc, best_possible_pair[0], best_possible_pair[1], loc_s, loc_g,
    ))
    if wall_exhausted():
        return wall_result("ambiguity_extraction")
    return PlanResult(
        PlanStatus.UNKNOWN,
        report={"reason": "safe_graph_ambiguous"},
        ambiguity=ambiguity,
    )


def _pose_locations(mc, pose, side: str):
    """Use complete closed-set memberships for compiled graphs.

    Lightweight hand-built compiler doubles predate ``locate_all`` semantics;
    they continue to use their explicitly supplied deterministic ``locate``.
    """
    if getattr(mc, "structural_contract", False):
        return list(mc.locate_all(pose, side))
    located = mc.locate(pose, side)
    return [] if located is None else [located]


def _classify_unlocated_poses(start, goal, mc, loc_sp, loc_gp, ambiguity,
                              wall_exhausted, wall_result):
    """Distinguish invalid endpoints from conservative graph coverage gaps."""
    import shapely
    from ..geometry.c_obstacle import scene_pose_collides

    missing = []
    invalid = []
    unresolved = []
    for name, pose, located in (
            ("start", start, loc_sp), ("goal", goal, loc_gp)):
        if located is not None:
            continue
        missing.append(name)
        if not mc.scene.workspace.covers(
                shapely.Point(tuple(np.asarray(pose.xy, dtype=float)))):
            invalid.append((name, "outside_workspace"))
            continue
        try:
            if scene_pose_collides(mc.scene, mc.robot, pose.xy, pose.theta):
                invalid.append((name, "in_collision"))
        except (RuntimeError, ValueError, FloatingPointError) as exc:
            unresolved.append((name, type(exc).__name__))
    if wall_exhausted():
        return wall_result("independent_pose_classification")
    if invalid:
        return PlanResult(
            PlanStatus.INVALID_GEOMETRY,
            report={
                "reason": "invalid_start_or_goal_pose", "invalid": invalid,
            },
            ambiguity=ambiguity,
        )
    ambiguity.append((
        "free_pose_not_located_in_finite_possible_graph",
        {"poses": missing, "oracle_unresolved": unresolved},
    ))
    return PlanResult(
        PlanStatus.UNKNOWN,
        report={
            "reason": "finite_possible_graph_pose_coverage_missing",
            "poses": missing,
            "oracle_unresolved": unresolved,
        },
        ambiguity=ambiguity,
    )


def _stale_global_cover(mc) -> bool:
    status = getattr(
        mc.decomposition, "global_possible_cover_status", CertStatus.UNKNOWN,
    )
    return status is CertStatus.CERTIFIED and any(
        certified_cover_slice(slab) is None
        for slab in getattr(mc.decomposition, "slabs", ())
    )


def _refinement_targets(mc, start, goal, ambiguity) -> list[int]:
    """Rank implicated, refinable slabs deterministically."""
    if not getattr(mc, "structural_contract", False):
        return []
    ranked = []

    def add(slab_id, priority):
        if isinstance(slab_id, (int, np.integer)):
            ranked.append((int(priority), int(slab_id)))

    for item in ambiguity:
        if not item:
            continue
        kind = item[0]
        if kind in {"lift_failed", "verify_failed"}:
            provenance = item[-1]
            if isinstance(provenance, SegmentProvenance):
                add(provenance.slab_id, 0)
                if provenance.edge is not None:
                    for node in provenance.edge:
                        if node in mc.M_safe:
                            add(mc.M_safe.nodes[node].get("slab"), 0)
        elif kind == "possible_only_edge" and len(item) > 1:
            for node in item[1]:
                if node in mc.M_possible:
                    add(mc.M_possible.nodes[node].get("slab"), 1)
        elif kind == "uncertain_slab" and len(item) > 1:
            add(item[1], 2)
        elif kind == "safe_path_gap_slab" and len(item) > 1:
            add(item[1], 2)
        elif kind == \
                "possible_graph_disconnected_without_certified_global_cover":
            # A non-formal upper-graph cut is precisely where local
            # tightening is useful.  First repair any slab that lacks an
            # interval-wide cover; otherwise refine the endpoint-side slabs
            # named by the cut candidate.  Never promote this cut directly to
            # UNREACHABLE.
            for slab in mc.decomposition.slabs:
                if certified_cover_slice(slab) is None:
                    add(slab.slab_id, 1)
            if len(item) > 1 and isinstance(item[1], dict):
                candidate = item[1]
                for key in ("start_nodes", "goal_nodes"):
                    for node in candidate.get(key, ()):
                        if node in mc.M_possible:
                            add(mc.M_possible.nodes[node].get("slab"), 2)
        elif kind == "start_not_in_safe":
            located = mc.locate(start, "possible")
            if located is not None:
                add(located[1].slab_id, 3)
        elif kind == "goal_not_in_safe":
            located = mc.locate(goal, "possible")
            if located is not None:
                add(located[1].slab_id, 3)

    unique = {}
    for priority, slab_id in ranked:
        unique[slab_id] = min(priority, unique.get(slab_id, priority))
    candidates = []
    for slab_id, priority in unique.items():
        if not 0 <= slab_id < len(mc.decomposition.slabs):
            continue
        slab = mc.decomposition.slabs[slab_id]
        if (slab.interval.width <= mc.cfg.orientation.theta_min
                or int(getattr(slab, "depth", 0))
                >= mc.cfg.orientation.max_depth):
            continue
        candidates.append((
            priority, -float(slab.interval.width),
            float(slab.interval.lo), slab_id,
        ))
    return [slab_id for *_prefix, slab_id in sorted(candidates)]


def _query_ambiguity(mc, ns_p, ng_p, loc_s, loc_g):
    amb = []
    if loc_s is None:
        amb.append(("start_not_in_safe", None))
    if loc_g is None:
        amb.append(("goal_not_in_safe", None))
    try:
        path = nx.shortest_path(mc.M_possible, ns_p, ng_p, weight="weight")
        for na, nb in zip(path[:-1], path[1:]):
            if mc.M_possible.get_edge_data(na, nb).get("status") == "POSSIBLE":
                amb.append(("possible_only_edge", (na, nb)))
        for n in path:
            slab = mc.decomposition.slabs[mc.M_possible.nodes[n]["slab"]]
            if not slab_has_safe_components(slab):
                amb.append(("uncertain_slab", slab.slab_id))
        safe_connected = bool(
            loc_s is not None and loc_g is not None
            and nx.has_path(mc.M_safe, loc_s[0], loc_g[0])
        )
        if not safe_connected:
            # Even when an upper edge already owns some unrelated SAFE
            # witness, this particular possible route may lack a lower-graph
            # realization.  Every slab on the shortest upper route is then a
            # legitimate local tightening target.
            for n in path:
                slab_id = int(mc.M_possible.nodes[n]["slab"])
                amb.append(("safe_path_gap_slab", slab_id))
    except nx.NetworkXNoPath:
        pass
    return amb
