"""External Atlas G1 checks for the full connectivity-interval compiler."""
from __future__ import annotations

import json
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

from gmc.budget import WORK_KINDS
from gmc.benchmarking.atlas import AtlasAssetStore
import gmc.benchmarking.gate_intervals as gate_metrics
from gmc.benchmarking.gate_intervals import (
    _acceptance_decision,
    _cross_stage_contract_errors,
    _formal_profile_contract,
    _stage_gate,
    evaluate_gate_interval_report,
    evaluate_gate_interval_sets,
)
from gmc.benchmarking.runner import run_benchmark_spec
from gmc.orientation.connectivity_intervals import (
    ConnectivityInterval,
    ConnectivityIntervalReport,
    ConnectivityVerdict,
    TransitionBracket,
    UnresolvedInterval,
)
from gmc.orientation.intervals import TWO_PI, Interval
from gmc.orientation.provenance import capture_decomposition_binding
from gmc.spatial.bvh import (
    BVHQueryStats,
    CandidateOracleList,
    CandidatePairQuery,
    validate_candidate_oracles,
)
from gmc.types import PairID


PROJECT = Path(__file__).resolve().parents[2]
WORKSPACE = PROJECT.parent
ATLAS = WORKSPACE / "splatc_atlas"


def _case(query_id: str):
    split = "validation" if query_id.startswith("val_") else "dev"
    return AtlasAssetStore(ATLAS).load_cases(
        [split], case_ids={query_id}, partial_gate_only=False,
    )[0][0]


def test_full_s1_truth_is_recomputed_with_four_partial_gate_events():
    case = _case("val_004")
    expected = AtlasAssetStore._gate_half_angle(0.60, 0.25, 0.52)

    assert case.gate_half_angle_deg == float(np.rad2deg(expected))
    assert len(case.truth_events_rad(TWO_PI)) == 4
    assert np.isclose(
        sum(hi - lo for lo, hi in case.truth_open_intervals_rad(TWO_PI)),
        4.0 * expected,
        rtol=0.0,
        atol=1e-14,
    )


def _synthetic_partial_report(case, monkeypatch, *, bracket_width_deg):
    half = float(np.deg2rad(bracket_width_deg / 2.0))
    events = case.truth_events_rad(TWO_PI)
    boxes = [(event - half, event + half) for event in events]
    assert all(0.0 < lo < hi < TWO_PI for lo, hi in boxes)
    edges = sorted({0.0, TWO_PI, *(value for box in boxes for value in box)})
    truth = case.truth_open_intervals_rad(TWO_PI)

    def in_piece(theta, pieces):
        return any(lo < theta < hi for lo, hi in pieces)

    leaves = []
    slabs = []
    for slab_id, (lo, hi) in enumerate(zip(edges, edges[1:])):
        mid = 0.5 * (lo + hi)
        if in_piece(mid, boxes):
            verdict = ConnectivityVerdict.UNKNOWN
            reason = "synthetic_transition_unknown"
        elif in_piece(mid, truth):
            verdict = ConnectivityVerdict.OPEN
            reason = "same_strict_interval_safe_component"
        else:
            verdict = ConnectivityVerdict.CLOSED
            reason = "separate_upper_component_closures"
        interval = Interval(lo, hi)
        leaf = ConnectivityInterval(
            interval, verdict, slab_id, 0, reason, {},
        )
        leaves.append(leaf)
        slabs.append(SimpleNamespace(
            slab_id=slab_id, interval=interval, depth=0,
        ))
    safe = gate_metrics._merge_leaf_intervals(
        leaves, {ConnectivityVerdict.OPEN},
    )
    possible = gate_metrics._merge_leaf_intervals(
        leaves, {ConnectivityVerdict.OPEN, ConnectivityVerdict.UNKNOWN},
    )
    stop_reason = "theta_min_reached"
    runs, bracket_runs = gate_metrics._rebuild_unknown_runs(
        leaves, stop_reason,
    )
    tolerance = float(np.deg2rad(5.0))
    scene = SimpleNamespace(
        workspace=SimpleNamespace(wkb=b"synthetic-workspace"),
        supports=(),
    )
    robot = SimpleNamespace(supports=())
    cfg = gate_metrics._compiler_config({}, source="synthetic")
    oracles = ()
    decomposition = SimpleNamespace(
        slabs=slabs,
        revision=0,
        refinement_history=(),
        input_binding=capture_decomposition_binding(
            scene, robot, cfg, oracles,
        ),
    )
    by_id = {leaf.slab_id: leaf for leaf in leaves}
    monkeypatch.setattr(
        gate_metrics, "classify_translation_interval",
        lambda slab, _start, _goal: by_id[slab.slab_id],
    )
    report = ConnectivityIntervalReport(
        period=TWO_PI,
        start_xy=tuple(map(float, case.episode["start_pose"][:2])),
        goal_xy=tuple(map(float, case.episode["goals"][0])),
        intervals=tuple(leaves),
        theta_safe=tuple(Interval(*bounds) for bounds in safe),
        theta_possible=tuple(Interval(*bounds) for bounds in possible),
        unresolved=tuple(UnresolvedInterval(
            Interval(*item["interval"]), item["reasons"], item["slab_ids"],
        ) for item in runs),
        transition_brackets=tuple(TransitionBracket(
            Interval(*item["interval"]), item["left"], item["right"],
        ) for item in bracket_runs),
        refinement_trace=(),
        initial_revision=0,
        final_revision=0,
        stop_reason=stop_reason,
        decomposition=decomposition,
        resolution_met=all(
            item["interval"][1] - item["interval"][0] <= tolerance
            for item in runs
        ),
        event_tolerance=tolerance,
    )
    return report, {
        "scene": scene,
        "robot": robot,
        "cfg": cfg,
        "oracles": oracles,
    }


def test_tuple_predictions_and_truth_shaped_fake_unknown_are_rejected(
        monkeypatch):
    case = _case("dev_007")
    with np.testing.assert_raises(TypeError):
        evaluate_gate_interval_sets(
            case, theta_safe=(), theta_possible=(), unresolved=(),
            transition_brackets=(),
        )

    report, context = _synthetic_partial_report(
        case, monkeypatch, bracket_width_deg=4.0,
    )
    truth = tuple(Interval(lo, hi)
                  for lo, hi in case.truth_open_intervals_rad(TWO_PI))
    forged = replace(
        report,
        theta_safe=truth,
        theta_possible=truth,
        # Keep only one producer-looking UNKNOWN while retaining four
        # brackets.  The leaf reconstruction, not these summaries, must win.
        unresolved=report.unresolved[:1],
    )
    metrics = evaluate_gate_interval_report(
        case, forged, event_tolerance=np.deg2rad(5.0), **context,
    )
    assert not metrics["structural_contract_pass"]
    assert "reported_theta_safe_not_leaf_reconstruction" in metrics[
        "structural_contract_errors"]
    assert "reported_theta_possible_not_leaf_reconstruction" in metrics[
        "structural_contract_errors"]
    assert "reported_unknown_not_maximal_leaf_reconstruction" in metrics[
        "structural_contract_errors"]
    assert "reported_transition_bracket_not_unknown_subset" in metrics[
        "structural_contract_errors"]


def test_nonmaximal_bracket_and_leaf_provenance_drift_are_rejected(
        monkeypatch):
    case = _case("dev_007")
    report, context = _synthetic_partial_report(
        case, monkeypatch, bracket_width_deg=4.0,
    )
    first = report.transition_brackets[0]
    narrowed = replace(first, interval=Interval(
        first.interval.lo + 0.01, first.interval.hi - 0.01,
    ))
    forged = replace(
        report,
        intervals=(replace(
            report.intervals[0], diagnostics={"forged": True},
        ), *report.intervals[1:]),
        transition_brackets=(narrowed, *report.transition_brackets[1:]),
    )

    metrics = evaluate_gate_interval_report(
        case, forged, event_tolerance=np.deg2rad(5.0), **context,
    )

    assert not metrics["structural_contract_pass"]
    assert "leaf_0_diagnostics_provenance_drift" in metrics[
        "structural_contract_errors"]
    assert (
        "reported_brackets_not_maximal_opposite_neighbour_reconstruction"
        in metrics["structural_contract_errors"]
    )
    assert "reported_transition_bracket_not_unknown_subset" in metrics[
        "structural_contract_errors"]


def test_report_from_different_scene_binding_is_rejected(monkeypatch):
    case = _case("dev_007")
    report, context = _synthetic_partial_report(
        case, monkeypatch, bracket_width_deg=4.0,
    )
    wrong_scene = SimpleNamespace(
        workspace=context["scene"].workspace,
        supports=(),
    )

    metrics = evaluate_gate_interval_report(
        case, report, event_tolerance=np.deg2rad(5.0),
        **{**context, "scene": wrong_scene},
    )

    assert not metrics["structural_contract_pass"]
    assert "decomposition_binding_scene_identity" in metrics[
        "structural_contract_errors"]


def test_fake_report_numeric_budget_overflows_fail_postcheck(monkeypatch):
    case = _case("dev_007")
    report, context = _synthetic_partial_report(
        case, monkeypatch, bracket_width_deg=4.0,
    )
    stats = BVHQueryStats(
        scene_supports=0, body_supports=0, total_pairs=0,
        tree_nodes=0, tree_depth=0, nodes_visited=0, nodes_pruned=0,
        leaves_visited=0, pair_tests=0, pruned_pairs=0,
        candidate_pairs=0,
    )
    monkeypatch.setattr(
        gate_metrics, "query_candidate_pairs",
        lambda *_args: CandidatePairQuery((), stats),
    )
    deadlines = {}

    def fake_build(*_args, deadline, **_kwargs):
        deadlines["build"] = deadline
        return report.decomposition

    monkeypatch.setattr(
        gate_metrics, "build_connectivity_slabs", fake_build,
    )

    def fake_certified(*_args, max_support_calls, max_wall_seconds, deadline,
                       **_kwargs):
        deadlines["connectivity"] = deadline
        trace = ({
            "changed": True,
            "reason": "binary_interval_split",
            "revision_before": 0,
            "revision_after": 1,
            "parent_interval": [0.0, TWO_PI],
            "child_intervals": [[0.0, np.pi], [np.pi, TWO_PI]],
        },)
        report.decomposition.revision = 1
        report.decomposition.refinement_history = trace
        return replace(
            report,
            refinement_trace=trace,
            final_revision=1,
            support_calls=max_support_calls + 1,
            max_support_calls=max_support_calls,
            wall_seconds=max_wall_seconds + 1.0,
            max_wall_seconds=max_wall_seconds,
        )

    monkeypatch.setattr(
        gate_metrics, "certified_connectivity_intervals", fake_certified,
    )
    instance = SimpleNamespace(
        case=case, scene=context["scene"], robot=context["robot"],
    )

    row = gate_metrics.run_gate_interval_case(
        instance, context["cfg"], limits={},
        max_refinements=0, max_wall_seconds=100.0,
        event_tolerance=np.deg2rad(5.0),
    )

    assert not row["acceptance"]["pass"]
    assert "report_support_budget_exceeded" in row[
        "timing"]["budget_reasons"]
    assert "report_wall_budget_exceeded" in row[
        "timing"]["budget_reasons"]
    assert "report_refinement_budget_exceeded" in row[
        "timing"]["budget_reasons"]
    assert deadlines["build"] == deadlines["connectivity"]


def test_runner_rejects_a_non_support_budget_stop_reason(monkeypatch):
    case = _case("dev_007")
    report, context = _synthetic_partial_report(
        case, monkeypatch, bracket_width_deg=4.0,
    )
    stats = BVHQueryStats(
        scene_supports=0, body_supports=0, total_pairs=0,
        tree_nodes=0, tree_depth=0, nodes_visited=0, nodes_pruned=0,
        leaves_visited=0, pair_tests=0, pruned_pairs=0,
        candidate_pairs=0,
    )
    monkeypatch.setattr(
        gate_metrics, "query_candidate_pairs",
        lambda *_args: CandidatePairQuery((), stats),
    )
    monkeypatch.setattr(
        gate_metrics, "build_connectivity_slabs",
        lambda *_args, **_kwargs: report.decomposition,
    )
    stop_reason = "union_operations_budget_exhausted"
    runs, _ = gate_metrics._rebuild_unknown_runs(
        report.intervals, stop_reason,
    )

    def fake_certified(*_args, max_support_calls, max_wall_seconds,
                       **_kwargs):
        return replace(
            report,
            unresolved=tuple(UnresolvedInterval(
                Interval(*item["interval"]), item["reasons"], item["slab_ids"],
            ) for item in runs),
            stop_reason=stop_reason,
            max_support_calls=max_support_calls,
            max_wall_seconds=max_wall_seconds,
            wall_seconds=0.0,
            resolution_met=False,
        )

    monkeypatch.setattr(
        gate_metrics, "certified_connectivity_intervals", fake_certified,
    )
    instance = SimpleNamespace(
        case=case, scene=context["scene"], robot=context["robot"],
    )
    row = gate_metrics.run_gate_interval_case(
        instance, context["cfg"], limits={},
        max_refinements=0, max_wall_seconds=100.0,
        event_tolerance=np.deg2rad(5.0),
    )

    assert row["metrics"]["structural_contract_pass"]
    assert row["timing"]["budget_reasons"] == [stop_reason]
    assert row["timing"]["budget_exceeded"]
    assert stop_reason in row["acceptance"]["errors"]
    assert not row["acceptance"]["pass"]


def test_budget_stop_keeps_leaf_resolution_without_claiming_completion(
        monkeypatch):
    case = _case("dev_007")
    report, context = _synthetic_partial_report(
        case, monkeypatch, bracket_width_deg=4.0,
    )
    stop_reason = "wall_budget_exhausted"
    runs, _ = gate_metrics._rebuild_unknown_runs(
        report.intervals, stop_reason,
    )
    stopped = replace(
        report,
        unresolved=tuple(UnresolvedInterval(
            Interval(*item["interval"]), item["reasons"], item["slab_ids"],
        ) for item in runs),
        stop_reason=stop_reason,
        resolution_met=False,
    )

    metrics = evaluate_gate_interval_report(
        case, stopped, event_tolerance=np.deg2rad(5.0), **context,
    )
    acceptance = _acceptance_decision(
        metrics, certified_complete=False,
        budget_reasons=(stop_reason,),
        event_tolerance=np.deg2rad(5.0),
    )

    assert metrics["structural_contract_pass"]
    assert metrics["resolution_met"]
    assert metrics["sandwich_holds_in_measure"]
    assert not acceptance["pass"]
    assert stop_reason in acceptance["errors"]
    assert "structural_contract_failed" not in acceptance["errors"]
    assert "partial_resolution_not_met" not in acceptance["errors"]


def test_budget_stopped_report_cannot_forge_resolution_completion(monkeypatch):
    case = _case("dev_007")
    report, context = _synthetic_partial_report(
        case, monkeypatch, bracket_width_deg=4.0,
    )
    stop_reason = "wall_budget_exhausted"
    runs, _ = gate_metrics._rebuild_unknown_runs(
        report.intervals, stop_reason,
    )
    forged = replace(
        report,
        unresolved=tuple(UnresolvedInterval(
            Interval(*item["interval"]), item["reasons"], item["slab_ids"],
        ) for item in runs),
        stop_reason=stop_reason,
        resolution_met=True,
    )

    metrics = evaluate_gate_interval_report(
        case, forged, event_tolerance=np.deg2rad(5.0), **context,
    )

    assert not metrics["structural_contract_pass"]
    assert metrics["structural_contract_errors"] == [
        "reported_completion_despite_budget_stop",
    ]
    assert not metrics["resolution_met"]


def test_report_completion_flag_must_be_a_strict_bool(monkeypatch):
    case = _case("dev_007")
    report, context = _synthetic_partial_report(
        case, monkeypatch, bracket_width_deg=4.0,
    )
    forged = replace(report, resolution_met=1)

    metrics = evaluate_gate_interval_report(
        case, forged, event_tolerance=np.deg2rad(5.0), **context,
    )

    assert not metrics["structural_contract_pass"]
    assert metrics["structural_contract_errors"] == [
        "reported_completion_flag_not_bool",
    ]
    assert not metrics["resolution_met"]


def test_fifty_two_degree_brackets_cannot_pass_five_degree_gate(monkeypatch):
    case = _case("dev_007")
    report, context = _synthetic_partial_report(
        case, monkeypatch, bracket_width_deg=52.0,
    )
    metrics = evaluate_gate_interval_report(
        case, report, event_tolerance=np.deg2rad(5.0), **context,
    )
    assert metrics["structural_contract_pass"]
    assert metrics["event_recall"] == 1.0
    assert metrics["bracket_recall"] == 1.0
    assert np.isclose(metrics["max_merged_bracket_width_deg"], 52.0)
    assert not metrics["resolution_met"]
    acceptance = _acceptance_decision(
        metrics, certified_complete=False, budget_reasons=(),
        event_tolerance=np.deg2rad(5.0),
    )
    assert not acceptance["pass"]
    assert "partial_resolution_not_met" in acceptance["errors"]
    assert "partial_bracket_width_exceeds_tolerance" in acceptance["errors"]


def test_four_degree_partial_brackets_satisfy_strict_acceptance(monkeypatch):
    for query_id in ("dev_007", "val_004"):
        case = _case(query_id)
        report, context = _synthetic_partial_report(
            case, monkeypatch, bracket_width_deg=4.0,
        )
        metrics = evaluate_gate_interval_report(
            case, report, event_tolerance=np.deg2rad(5.0), **context,
        )
        acceptance = _acceptance_decision(
            metrics, certified_complete=False, budget_reasons=(),
            event_tolerance=np.deg2rad(5.0),
        )

        assert metrics["safe_measure_deg"] > 0.0
        assert metrics["certified_closed_measure_deg"] > 0.0
        assert metrics["events_covered_by_unresolved"] == 4
        assert metrics["matched_transition_brackets"] == 4
        assert metrics["spurious_transition_brackets"] == 0
        assert metrics["resolution_met"]
        assert acceptance["pass"], acceptance


def test_four_representative_cases_use_all_supports_and_full_compiler():
    spec = {
        "name": "atlas-full-compiler-four-case-smoke",
        "kind": "atlas_g1_full_gate_intervals",
        "atlas_root": str(ATLAS),
        "splits": ["dev", "validation"],
        "families": ["G1"],
        "partial_gate_only": False,
        "event_tolerance_deg": 5.0,
        "case_ids": ["dev_000", "dev_002", "dev_007", "val_004"],
        "expected": {
            "expected_n_cases": 4,
            "expected_case_ids": [
                "dev_000", "dev_002", "dev_007", "val_004",
            ],
            "acceptance_policy": "all_cases_containment",
        },
        "compiler": {
            "geometry": {
                "support_level_scene": 1.0,
                "support_level_robot": 1.0,
            },
            "pair_approx": {
                "initial_directions": 8,
                "max_directions": 8,
                "eps_pair": 0.25,
                "certificate_mode": "theorem",
            },
            "orientation": {
                "initial_intervals": 1,
                "theta_min": 0.1,
                "max_depth": 0,
            },
            "query": {
                "max_support_calls": 1_000_000,
                "max_wall_seconds": 120,
                "eps_clear": 2e-3,
                "max_refinement_rounds": 0,
            },
        },
        "budget": {
            "limits_per_case": {
                "support_value_evals": 1_000_000,
                "support_point_evals": 1_000_000,
            },
            "max_refinements_per_case": 1,
            "wall_warning_seconds_per_case": 120,
        },
    }
    report = run_benchmark_spec(spec)

    assert report["scope"]["n_cases"] == 4
    assert report["scope"]["full_scene_supports"]
    assert report["scope"]["full_orientation_period_deg"] == 360.0
    assert not report["compiler_contract"]["pair_kernel_only"]
    assert "theorem midpoint PairSandwichSlice" in report[
        "compiler_contract"]["path"]
    assert "theorem fixed slices" not in report["compiler_contract"]["path"]
    assert not report["truth_contract"]["legacy_splatc_gmc_called"]
    assert report["truth_contract"]["oracle_record_role"] == (
        "package-inventory checked, not independently pinned, "
        "cross-check only"
    )
    assert "independently pinned G1 registry" in report[
        "truth_contract"]["primary"]
    assert {row["metrics"]["analytic_class"] for row in report["cases"]} == {
        "closed", "full_open", "partial",
    }
    assert all(row["compiler"]["candidate_pairs"] >= 390
               for row in report["cases"])
    assert all(row["compiler"]["report"] is not None
               for row in report["cases"])
    assert all(row["compiler"]["report"]["event_tolerance_deg"] == 5.0
               for row in report["cases"])
    assert all(row["metrics"]["sandwich_holds_in_measure"]
               for row in report["cases"])
    assert set(report["aggregate"]["ledger_totals"]) == set(WORK_KINDS)
    assert report["gate"]["pass"], report["gate"]
    # The actual report is an artifact, so numpy scalar leakage is forbidden.
    json.dumps(report, sort_keys=True)


def test_staged_report_keeps_initial_and_refined_aggregates_separate():
    common_compiler = {
        "geometry": {
            "support_level_scene": 1.0,
            "support_level_robot": 1.0,
        },
        "pair_approx": {
            "initial_directions": 8,
            "max_directions": 8,
            "eps_pair": 0.25,
            "certificate_mode": "theorem",
        },
        "orientation": {
            "initial_intervals": 1,
            "theta_min": 0.1,
            "max_depth": 0,
        },
        "query": {
            "max_support_calls": 1_000_000,
            "max_wall_seconds": 120,
            "eps_clear": 2e-3,
        },
    }
    common_budget = {
        "limits_per_case": {
            "support_value_evals": 1_000_000,
            "support_point_evals": 1_000_000,
        },
        "max_refinements_per_case": 1,
    }
    report = run_benchmark_spec({
        "name": "two-stage-smoke",
        "kind": "atlas_g1_full_gate_intervals",
        "atlas_root": str(ATLAS),
        "splits": ["dev"],
        "families": ["G1"],
        "partial_gate_only": False,
        "event_tolerance_deg": 5.0,
        "compiler": common_compiler,
        "budget": common_budget,
        "stages": [
            {
                "stage": "initial",
                "case_ids": ["dev_000"],
                "expected": {
                    "expected_n_cases": 1,
                    "expected_case_ids": ["dev_000"],
                    "acceptance_policy": "all_cases_containment",
                },
            },
            {
                "stage": "refined",
                "case_ids": ["dev_002"],
                "expected": {
                    "expected_n_cases": 1,
                    "expected_case_ids": ["dev_002"],
                    "acceptance_policy": "all_cases_containment",
                },
            },
        ],
    })

    assert report["kind"] == "atlas_g1_full_gate_intervals_staged"
    assert report["stage_order"] == ["initial", "refined"]
    assert set(report["aggregate_by_stage"]) == {"initial", "refined"}
    assert report["cross_stage_contract"][
        "initial_and_refined_results_are_not_pooled"
    ]
    assert report["profile_contract"] == {
        "mode": "diagnostic_only", "profile": None, "validated": False,
    }
    assert report["asset_integrity_scope"].startswith(
        "selection-invariant")
    assert set(report["asset_selection_integrity_by_stage"]) == {
        "initial", "refined",
    }
    assert report["gate"]["pass"], report["gate"]
    assert [stage["scope"]["case_ids"] for stage in report["stages"]] == [
        ["dev_000"], ["dev_002"],
    ]
    json.dumps(report, sort_keys=True)


def test_formal_profile_rejects_stage_case_tolerance_and_budget_drift():
    path = PROJECT / "configs" / "benchmarks" / (
        "atlas_g1_full_gate_intervals.yaml"
    )
    spec = yaml.safe_load(path.read_text())
    contract = _formal_profile_contract(spec)
    assert contract["validated"]
    assert contract["stage_order"] == [
        "initial_42_soundness", "refined_16_acceptance",
    ]
    initial, refined = spec["stages"]
    expected_initial_ids = tuple(
        [f"dev_{index:03d}" for index in range(24)]
        + [f"val_{index:03d}" for index in range(18)]
    )
    expected_refined_ids = (
        "dev_000", "dev_001", "dev_002", "dev_004", "dev_007",
        "dev_010", "dev_013", "dev_016", "dev_019", "dev_022",
        "val_001", "val_004", "val_007", "val_010", "val_013",
        "val_016",
    )
    assert spec["formal_profile"] == "atlas_g1_full_gate_intervals_v1"
    assert spec["event_tolerance_deg"] == 5.0
    assert "case_ids" not in initial
    assert tuple(initial["expected"]["expected_case_ids"]) == \
        expected_initial_ids
    assert tuple(refined["case_ids"]) == expected_refined_ids
    assert tuple(refined["expected"]["expected_case_ids"]) == \
        expected_refined_ids
    assert [initial["expected"]["expected_n_cases"],
            refined["expected"]["expected_n_cases"]] == [42, 16]
    assert [initial["expected"]["acceptance_policy"],
            refined["expected"]["acceptance_policy"]] == [
        "all_cases_containment", "all_cases_accept",
    ]
    common_geometry = {
        "workspace_precision": 1.0e-8,
        "min_cov_eigenvalue": 1.0e-10,
        "support_level_scene": 1.0,
        "support_level_robot": 1.0,
    }
    common_logging = {
        "save_intermediate_geometry": False,
        "save_failed_cases": True,
    }
    assert initial["compiler"] == {
        "length_unit": "meter",
        "geometry": common_geometry,
        "pair_approx": {
            "initial_directions": 16, "max_directions": 64,
            "eps_pair": 1.0e-2, "certificate_mode": "theorem",
        },
        "orientation": {
            "initial_intervals": 4, "theta_min": 2.0e-2,
            "max_depth": 0,
        },
        "query": {
            "max_support_calls": 5_000_000,
            "max_wall_seconds": 120,
            "eps_clear": 2.0e-3,
            "max_refinement_rounds": 0,
        },
        "logging": common_logging,
    }
    assert refined["compiler"] == {
        "length_unit": "meter",
        "geometry": common_geometry,
        "pair_approx": {
            "initial_directions": 32, "max_directions": 128,
            "eps_pair": 2.0e-3, "certificate_mode": "theorem",
        },
        "orientation": {
            "initial_intervals": 32,
            "theta_min": 1.7453292519943296e-2,
            "max_depth": 8,
        },
        "query": {
            "max_support_calls": 50_000_000,
            "max_wall_seconds": 1800,
            "eps_clear": 2.0e-3,
            "max_refinement_rounds": 128,
        },
        "logging": common_logging,
    }
    assert initial["budget"] == {
        "limits_per_case": {
            "support_value_evals": 5_000_000,
            "support_point_evals": 5_000_000,
        },
        "max_refinements_per_case": 1,
        "max_wall_seconds_per_case": 120,
    }
    assert refined["budget"] == {
        "limits_per_case": {
            "support_value_evals": 50_000_000,
            "support_point_evals": 50_000_000,
        },
        "max_refinements_per_case": 128,
        "max_wall_seconds_per_case": 1800,
    }

    attacks = []
    missing_stage = deepcopy(spec)
    missing_stage["stages"] = missing_stage["stages"][1:]
    attacks.append(missing_stage)

    swapped_stages = deepcopy(spec)
    swapped_stages["stages"].reverse()
    attacks.append(swapped_stages)

    shrunken = deepcopy(spec)
    refined = shrunken["stages"][1]
    refined["case_ids"] = refined["case_ids"][:1]
    refined["expected"]["expected_n_cases"] = 1
    refined["expected"]["expected_case_ids"] = refined["case_ids"]
    attacks.append(shrunken)

    wrong_tolerance = deepcopy(spec)
    wrong_tolerance["event_tolerance_deg"] = 52.0
    attacks.append(wrong_tolerance)

    weak_compiler = deepcopy(spec)
    weak_compiler["stages"][1]["compiler"]["pair_approx"][
        "max_directions"] = 8
    attacks.append(weak_compiler)

    weak_budget = deepcopy(spec)
    weak_budget["stages"][1]["budget"][
        "max_refinements_per_case"] = 0
    attacks.append(weak_budget)

    for attack in attacks:
        with np.testing.assert_raises(ValueError):
            run_benchmark_spec(attack, spec_dir=path.parent)

    # The public evaluator has no inherited-profile capability.  Neither a
    # replayed name nor an object imported from the module can skip validation.
    assert not hasattr(gate_metrics, "_FORMAL_PROFILE_TOKEN")
    with np.testing.assert_raises(TypeError):
        gate_metrics.run_atlas_gate_interval_spec(
            deepcopy(spec), spec_dir=path.parent,
            _profile_token=object(),
        )

    diagnostic = deepcopy(spec)
    diagnostic.pop("formal_profile")
    assert _formal_profile_contract(diagnostic) == {
        "mode": "diagnostic_only", "profile": None, "validated": False,
    }


def test_stage_gate_rejects_expected_case_and_policy_drift():
    report = {
        "scope": {"n_cases": 1, "case_ids": ["dev_000"]},
        "cases": [{
            "query_id": "dev_000",
            "metrics": {
                "structural_contract_pass": True,
                "theta_safe_subset_theta_true": True,
                "theta_true_subset_theta_possible": True,
            },
            "timing": {"budget_exceeded": False},
            "acceptance": {"pass": True},
        }],
    }
    gate = _stage_gate(report, {
        "expected_n_cases": 2,
        "expected_case_ids": ["dev_001"],
        "acceptance_policy": "unknown_policy",
    })

    assert not gate["pass"]
    assert set(gate["errors"]) == {
        "expected_n_cases_mismatch",
        "expected_case_ids_mismatch",
        "acceptance_policy_invalid",
    }


def _accepted_partial_stage_report():
    metrics = {
        "structural_contract_pass": True,
        "analytic_class": "partial",
        "safe_measure_deg": 20.0,
        "certified_closed_measure_deg": 20.0,
        "sandwich_holds_in_measure": True,
        "event_count": 4,
        "events_covered_by_unresolved": 4,
        "event_recall": 1.0,
        "transition_bracket_count": 4,
        "matched_transition_brackets": 4,
        "bracket_recall": 1.0,
        "spurious_transition_brackets": 0,
        "resolution_met": True,
        "max_merged_bracket_width_deg": 4.0,
        "event_tolerance_deg": 5.0,
    }
    acceptance = _acceptance_decision(
        metrics, certified_complete=False, budget_reasons=(),
        event_tolerance=np.deg2rad(5.0),
    )
    row = {
        "query_id": "dev_007",
        "metrics": metrics,
        "compiler": {
            "report": {
                "unresolved": [{"lo_deg": 10.0}],
                "stop_reason": "event_tolerance_met",
                "resolution_met": True,
            },
            "certified_complete": False,
            "stop_reason": "event_tolerance_met",
        },
        "timing": {"budget_exceeded": False, "budget_reasons": []},
        "acceptance": {**acceptance, "note": "not part of recomputation"},
    }
    report = {
        "scope": {"n_cases": 1, "case_ids": ["dev_007"]},
        "cases": [row],
    }
    expected = {
        "expected_n_cases": 1,
        "expected_case_ids": ["dev_007"],
        "acceptance_policy": "all_cases_accept",
    }
    return report, expected


def test_all_cases_accept_recomputes_and_rejects_forged_cached_pass():
    report, expected = _accepted_partial_stage_report()
    assert _stage_gate(report, expected)["pass"]

    forged = deepcopy(report)
    forged["cases"][0]["metrics"]["resolution_met"] = False
    gate = _stage_gate(forged, expected)

    assert not gate["pass"]
    assert "dev_007:acceptance_cache_drift" in gate["errors"]
    assert "dev_007:acceptance_failed" in gate["errors"]


def test_stage_gate_recovers_budget_stops_hidden_by_cleared_timing():
    report, expected = _accepted_partial_stage_report()
    stop_reason = "wall_budget_exhausted"

    for source in ("compiler", "report", "both"):
        forged = deepcopy(report)
        compiler = forged["cases"][0]["compiler"]
        if source in ("compiler", "both"):
            compiler["stop_reason"] = stop_reason
        if source in ("report", "both"):
            compiler["report"]["stop_reason"] = stop_reason
        compiler["report"]["resolution_met"] = False

        gate = _stage_gate(forged, expected)

        assert not gate["pass"]
        assert (
            "dev_007:compiler_report_stop_reason_drift" in gate["errors"]
        ) is (source != "both")
        assert "dev_007:budget_stop_missing_from_timing" in gate["errors"]
        assert "dev_007:budget_exceeded_cache_drift" in gate["errors"]
        assert "dev_007:acceptance_cache_drift" in gate["errors"]
        assert "dev_007:acceptance_failed" in gate["errors"]


def test_stage_gate_rejects_forged_budget_completion_and_non_bool_flag():
    report, expected = _accepted_partial_stage_report()
    stop_reason = "wall_budget_exhausted"

    forged_completion = deepcopy(report)
    compiler = forged_completion["cases"][0]["compiler"]
    compiler["stop_reason"] = stop_reason
    compiler["report"]["stop_reason"] = stop_reason
    compiler["report"]["resolution_met"] = True
    compiler["report"]["unresolved"] = [{"lo_deg": 10.0}]
    forged_completion["cases"][0]["timing"] = {
        "budget_exceeded": True,
        "budget_reasons": [stop_reason],
    }
    gate = _stage_gate(forged_completion, expected)
    assert not gate["pass"]
    assert (
        "dev_007:serialized_report_completion_despite_budget_stop"
        in gate["errors"]
    )
    assert "dev_007:acceptance_failed" in gate["errors"]

    forged_type = deepcopy(report)
    forged_type["cases"][0]["compiler"]["report"][
        "resolution_met"
    ] = 1
    gate = _stage_gate(forged_type, expected)
    assert not gate["pass"]
    assert (
        "dev_007:serialized_report_completion_flag_not_bool"
        in gate["errors"]
    )


def test_cross_stage_truth_asset_and_metric_contract_drift_is_rejected():
    reference = {
        "stage": "initial",
        "gate": {"declared": True, "pass": True},
        "profile_contract": {
            "mode": "diagnostic_only", "profile": None,
            "validated": False,
        },
        "truth_contract": {"primary": "sealed_manifest_formula"},
        "asset_integrity": {"dev": {
            "manifest_sha256": "manifest-a",
            "selected_oracle_records": [{"query_id": "dev_000"}],
            "oracle_records_package_inventory_checked": True,
        }},
        "metric_contract": {"event_tolerance_deg": 5.0},
    }
    selection_only = deepcopy(reference)
    selection_only["stage"] = "refined"
    selection_only["asset_integrity"]["dev"][
        "selected_oracle_records"] = [{"query_id": "dev_007"}]
    selection_only["asset_integrity"]["dev"][
        "oracle_records_package_inventory_checked"] = False
    assert not _cross_stage_contract_errors([reference, selection_only])

    truth_drift = deepcopy(selection_only)
    truth_drift["truth_contract"]["primary"] = "oracle_record"
    asset_drift = deepcopy(selection_only)
    asset_drift["asset_integrity"]["dev"][
        "manifest_sha256"] = "manifest-b"
    metric_drift = deepcopy(selection_only)
    metric_drift["metric_contract"]["event_tolerance_deg"] = 52.0

    assert _cross_stage_contract_errors([reference, truth_drift]) == [
        "refined:truth_contract_drift",
    ]
    assert _cross_stage_contract_errors([reference, asset_drift]) == [
        "refined:asset_contract_drift",
    ]
    assert _cross_stage_contract_errors([reference, metric_drift]) == [
        "refined:metric_contract_drift",
    ]


def test_budget_exception_keeps_finally_timing_and_fails_acceptance(
        monkeypatch):
    case = _case("dev_000")
    stats = BVHQueryStats(
        scene_supports=0, body_supports=0, total_pairs=0,
        tree_nodes=0, tree_depth=0, nodes_visited=0, nodes_pruned=0,
        leaves_visited=0, pair_tests=0, pruned_pairs=0,
        candidate_pairs=0,
    )
    monkeypatch.setattr(
        gate_metrics, "query_candidate_pairs",
        lambda *_args: CandidatePairQuery((), stats),
    )

    def exhaust(_scene, _robot, _cfg, _oracles, *, ledger, deadline):
        assert deadline == 14.0
        ledger.charge("support_value_evals", 2)

    monkeypatch.setattr(gate_metrics, "build_connectivity_slabs", exhaust)
    ticks = iter((10.0, 11.0, 13.0, 15.0))
    monkeypatch.setattr(
        gate_metrics.time, "perf_counter", lambda: next(ticks),
    )
    instance = SimpleNamespace(
        case=case,
        scene=SimpleNamespace(
            workspace=SimpleNamespace(wkb=b"budget-exception-workspace"),
        ),
        robot=object(),
    )
    cfg = gate_metrics._compiler_config({}, source="synthetic")

    row = gate_metrics.run_gate_interval_case(
        instance, cfg,
        limits={"support_value_evals": 1},
        max_refinements=0,
        max_wall_seconds=4.0,
        event_tolerance=np.deg2rad(5.0),
    )

    assert row["timing"]["full_compiler_seconds"] == 2.0
    assert row["timing"]["connectivity_interval_seconds"] == 0.0
    assert row["timing"]["total_wall_seconds"] == 5.0
    assert row["timing"]["budget_exceeded"]
    assert not row["acceptance"]["pass"]
    assert "support_value_evals_budget_exhausted" in row[
        "acceptance"]["errors"]
    assert "wall_budget_exceeded" in row["acceptance"]["errors"]


def test_case_deadline_skips_connectivity_and_builder_gets_all_pairs(
        monkeypatch):
    case = _case("dev_000")
    scene_supports = tuple(
        SimpleNamespace(primitive_id=index) for index in range(3)
    )
    body_support = SimpleNamespace(primitive_id=0)
    workspace = SimpleNamespace(wkb=b"all-candidate-pairs-workspace")
    scene = SimpleNamespace(workspace=workspace, supports=scene_supports)
    robot = SimpleNamespace(supports=(body_support,))
    source_oracles = tuple(
        SimpleNamespace(
            pair_id=PairID(index, 0),
            scene=scene_supports[index], body=body_support,
        )
        for index in range(3)
    )
    stats = BVHQueryStats(
        scene_supports=3, body_supports=1, total_pairs=3,
        tree_nodes=1, tree_depth=0, nodes_visited=1, nodes_pruned=0,
        leaves_visited=1, pair_tests=3, pruned_pairs=0,
        candidate_pairs=3,
    )
    decisions = (SimpleNamespace(record_id="retained-pair-proof"),)
    monkeypatch.setattr(
        gate_metrics, "query_candidate_pairs",
        lambda *_args: CandidatePairQuery(source_oracles, stats, decisions),
    )
    clock = [10.0]
    monkeypatch.setattr(
        gate_metrics.time, "perf_counter", lambda: clock[0],
    )
    observed = {}

    def build(_scene, _robot, _cfg, oracles, *, ledger, deadline):
        assert isinstance(oracles, CandidateOracleList)
        assert oracles.matches(scene, robot, workspace)
        assert validate_candidate_oracles(
            scene, robot, workspace, oracles,
        ) is oracles
        assert oracles._pruning_decisions == decisions
        assert oracles._bvh_stats is stats
        observed["pair_ids"] = tuple(oracle.pair_id for oracle in oracles)
        observed["ledger"] = ledger
        observed["deadline"] = deadline
        clock[0] = deadline
        return SimpleNamespace()

    monkeypatch.setattr(gate_metrics, "build_connectivity_slabs", build)

    def connectivity_must_not_run(*_args, **_kwargs):
        raise AssertionError("connectivity entered after the case deadline")

    monkeypatch.setattr(
        gate_metrics, "certified_connectivity_intervals",
        connectivity_must_not_run,
    )
    instance = SimpleNamespace(
        case=case, scene=scene, robot=robot,
    )
    cfg = gate_metrics._compiler_config({}, source="synthetic")

    row = gate_metrics.run_gate_interval_case(
        instance, cfg, limits={}, max_refinements=0,
        max_wall_seconds=4.0, event_tolerance=np.deg2rad(5.0),
    )

    assert observed["pair_ids"] == (
        PairID(0, 0), PairID(1, 0), PairID(2, 0),
    )
    assert observed["ledger"] is not None
    assert observed["deadline"] == 14.0
    assert row["compiler"]["candidate_pairs"] == 3
    assert row["compiler"]["report"] is None
    assert row["timing"]["full_compiler_seconds"] == 4.0
    assert row["timing"]["connectivity_interval_seconds"] == 0.0
    assert row["timing"]["total_wall_seconds"] == 4.0
    assert row["timing"]["budget_reasons"] == ["wall_budget_exhausted"]
    assert not row["acceptance"]["pass"]
