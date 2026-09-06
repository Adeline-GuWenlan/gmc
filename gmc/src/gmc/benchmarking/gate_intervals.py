"""Full-compiler SplatC-Atlas G1 connectivity-interval evaluation.

This runner is deliberately separate from :mod:`gmc.benchmarking.events`.
The event microbenchmark probes only the two jamb support functions; this
module materialises every frozen G1 support, constructs the theorem-mode GMC
orientation decomposition, and then certifies fixed-translation connectivity
over the complete orientation circle.

Primary truth is recomputed by ``AtlasAssetStore`` from split-manifest fields
bound by the independently pinned split seal and from robot axes in the
independently pinned G1 registry.  OracleRecord data is package-inventory
checked but not independently pinned; it is reported only as a secondary
cross-check and is never used to construct a label, interval, or event.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
from numbers import Integral, Real
import time
from pathlib import Path

import numpy as np

from ..budget import BudgetExceeded, WORK_KINDS, WorkLedger
from ..config import (Config, GeometryCfg, LoggingCfg, OrientationCfg,
                      PairApproxCfg, QueryCfg)
from ..geometry.support import PairOracle
from ..orientation.connectivity_intervals import (
    ConnectivityInterval,
    ConnectivityIntervalReport,
    ConnectivityVerdict,
    TransitionBracket,
    UnresolvedInterval,
    certified_connectivity_intervals,
    classify_translation_interval,
)
from ..orientation.intervals import TWO_PI, Interval
from ..orientation.provenance import decomposition_binding_failures
from ..orientation.slab_builder import build_connectivity_slabs
from ..spatial.bvh import CandidateOracleList, query_candidate_pairs
from ..spatial.slice_compiler import CompilationDeadlineExceeded
from .atlas import AtlasAssetStore
from .metrics import evaluate_event_brackets


_MEASURE_ATOL = 128.0 * np.finfo(float).eps * TWO_PI

_FORMAL_PROFILE = "atlas_g1_full_gate_intervals_v1"
_FORMAL_INITIAL_IDS = tuple(
    [f"dev_{index:03d}" for index in range(24)]
    + [f"val_{index:03d}" for index in range(18)]
)
_FORMAL_REFINED_IDS = (
    "dev_000", "dev_001", "dev_002", "dev_004", "dev_007",
    "dev_010", "dev_013", "dev_016", "dev_019", "dev_022",
    "val_001", "val_004", "val_007", "val_010", "val_013", "val_016",
)
_FORMAL_INITIAL_COMPILER = {
    "length_unit": "meter",
    "geometry": {
        "workspace_precision": 1.0e-8,
        "min_cov_eigenvalue": 1.0e-10,
        "support_level_scene": 1.0,
        "support_level_robot": 1.0,
    },
    "pair_approx": {
        "initial_directions": 16,
        "max_directions": 64,
        "eps_pair": 1.0e-2,
        "certificate_mode": "theorem",
    },
    "orientation": {
        "initial_intervals": 4,
        "theta_min": 2.0e-2,
        "max_depth": 0,
    },
    "query": {
        "max_support_calls": 5_000_000,
        "max_wall_seconds": 120,
        "eps_clear": 2.0e-3,
        "max_refinement_rounds": 0,
    },
    "logging": {
        "save_intermediate_geometry": False,
        "save_failed_cases": True,
    },
}
_FORMAL_REFINED_COMPILER = {
    "length_unit": "meter",
    "geometry": {
        "workspace_precision": 1.0e-8,
        "min_cov_eigenvalue": 1.0e-10,
        "support_level_scene": 1.0,
        "support_level_robot": 1.0,
    },
    "pair_approx": {
        "initial_directions": 32,
        "max_directions": 128,
        "eps_pair": 2.0e-3,
        "certificate_mode": "theorem",
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
    "logging": {
        "save_intermediate_geometry": False,
        "save_failed_cases": True,
    },
}
_FORMAL_INITIAL_BUDGET = {
    "limits_per_case": {
        "support_value_evals": 5_000_000,
        "support_point_evals": 5_000_000,
    },
    "max_refinements_per_case": 1,
    "max_wall_seconds_per_case": 120,
}
_FORMAL_REFINED_BUDGET = {
    "limits_per_case": {
        "support_value_evals": 50_000_000,
        "support_point_evals": 50_000_000,
    },
    "max_refinements_per_case": 128,
    "max_wall_seconds_per_case": 1800,
}


def _compiler_config(raw: dict, *, source: str) -> Config:
    """Build a frozen GMC config from a benchmark-local compiler mapping."""
    if not isinstance(raw, dict):
        raise ValueError("compiler must be a mapping")
    geometry = raw.get("geometry", {})
    pair = raw.get("pair_approx", {})
    orientation = raw.get("orientation", {})
    query = raw.get("query", {})
    logging = raw.get("logging", {})
    return Config(
        length_unit=str(raw.get("length_unit", "meter")),
        geometry=GeometryCfg(
            workspace_precision=geometry.get("workspace_precision", 1e-8),
            min_cov_eigenvalue=geometry.get("min_cov_eigenvalue", 1e-10),
            # Atlas hard supports already contain their frozen levels.
            support_level_scene=geometry.get("support_level_scene", 1.0),
            support_level_robot=geometry.get("support_level_robot", 1.0),
        ),
        pair_approx=PairApproxCfg(
            initial_directions=pair.get("initial_directions", 32),
            max_directions=pair.get("max_directions", 128),
            eps_pair=pair.get("eps_pair", 5e-3),
            certificate_mode=pair.get("certificate_mode", "theorem"),
        ),
        orientation=OrientationCfg(
            initial_intervals=orientation.get("initial_intervals", 4),
            theta_min=orientation.get("theta_min", 1e-2),
            max_depth=orientation.get("max_depth", 12),
        ),
        query=QueryCfg(
            max_support_calls=query.get("max_support_calls", 20_000_000),
            max_wall_seconds=query.get("max_wall_seconds", 600.0),
            eps_clear=query.get("eps_clear", 2e-3),
            max_refinement_rounds=query.get("max_refinement_rounds", 0),
        ),
        logging=LoggingCfg(
            save_intermediate_geometry=logging.get(
                "save_intermediate_geometry", False),
            save_failed_cases=logging.get("save_failed_cases", True),
        ),
        source_path=source,
    )


def _formal_stage_contracts() -> list[dict]:
    """Return the code-owned v1 stage contract used by the formal gate."""
    return [
        {
            "stage": "initial_42_soundness",
            "expected": {
                "expected_n_cases": 42,
                "expected_case_ids": list(_FORMAL_INITIAL_IDS),
                "acceptance_policy": "all_cases_containment",
            },
            "compiler": _FORMAL_INITIAL_COMPILER,
            "budget": _FORMAL_INITIAL_BUDGET,
        },
        {
            "stage": "refined_16_acceptance",
            "case_ids": list(_FORMAL_REFINED_IDS),
            "expected": {
                "expected_n_cases": 16,
                "expected_case_ids": list(_FORMAL_REFINED_IDS),
                "acceptance_policy": "all_cases_accept",
            },
            "compiler": _FORMAL_REFINED_COMPILER,
            "budget": _FORMAL_REFINED_BUDGET,
        },
    ]


def _formal_profile_contract(spec: dict) -> dict:
    """Validate the immutable formal profile before any benchmark work.

    Specs without ``formal_profile`` remain useful diagnostics, but their
    reports are labelled diagnostic-only and cannot represent this profile.
    """
    declared = spec.get("formal_profile")
    if declared is None:
        return {
            "mode": "diagnostic_only",
            "profile": None,
            "validated": False,
        }
    if declared != _FORMAL_PROFILE:
        raise ValueError(f"unknown formal_profile: {declared!r}")

    errors = []
    expected_top = {
        "name": "atlas-g1-full-gate-connectivity-intervals-phase1",
        "kind": "atlas_g1_full_gate_intervals",
        "formal_profile": _FORMAL_PROFILE,
        "splits": ["dev", "validation"],
        "families": ["G1"],
        "partial_gate_only": False,
        "event_tolerance_deg": 5.0,
    }
    allowed_keys = {*expected_top, "atlas_root", "stages"}
    if set(spec) != allowed_keys:
        errors.append("formal_top_level_keys_drift")
    for key, expected in expected_top.items():
        if spec.get(key) != expected:
            errors.append(f"formal_{key}_drift")
    if not isinstance(spec.get("atlas_root"), str) \
            or not spec["atlas_root"]:
        errors.append("formal_atlas_root_invalid")
    if spec.get("stages") != _formal_stage_contracts():
        errors.append("formal_stage_order_or_contract_drift")
    if errors:
        raise ValueError(
            "formal profile contract mismatch: " + ", ".join(errors)
        )
    return {
        "mode": "formal",
        "profile": _FORMAL_PROFILE,
        "validated": True,
        "stage_order": [
            "initial_42_soundness", "refined_16_acceptance",
        ],
        "initial_case_count": 42,
        "refined_case_count": 16,
        "event_tolerance_deg": 5.0,
    }


def _linearize(intervals, period: float = TWO_PI) \
        -> tuple[tuple[float, float], ...]:
    """Canonical union of optionally wrapped interval-like objects."""
    pieces: list[tuple[float, float]] = []
    for item in intervals:
        lo = float(item.lo if hasattr(item, "lo") else item[0])
        hi = float(item.hi if hasattr(item, "hi") else item[1])
        width = hi - lo
        if not np.isfinite([lo, hi]).all() or width <= 0.0:
            raise ValueError("angular intervals must be finite and nonempty")
        if width >= period - _MEASURE_ATOL:
            pieces.append((0.0, period))
            continue
        start = lo % period
        end = start + width
        if end <= period:
            pieces.append((start, end))
        else:
            pieces.extend(((start, period), (0.0, end - period)))
    pieces.sort()
    merged: list[tuple[float, float]] = []
    for lo, hi in pieces:
        if merged and lo <= merged[-1][1] + _MEASURE_ATOL:
            merged[-1] = (merged[-1][0], max(merged[-1][1], hi))
        else:
            merged.append((lo, hi))
    return tuple(merged)


def _measure(intervals) -> float:
    return float(sum(hi - lo for lo, hi in _linearize(intervals)))


def _intersection_measure(left, right) -> float:
    a = _linearize(left)
    b = _linearize(right)
    total = 0.0
    i = j = 0
    while i < len(a) and j < len(b):
        total += max(0.0, min(a[i][1], b[j][1])
                     - max(a[i][0], b[j][0]))
        if a[i][1] <= b[j][1]:
            i += 1
        else:
            j += 1
    return float(total)


def _difference_measure(left, right) -> float:
    value = _measure(left) - _intersection_measure(left, right)
    return float(max(0.0, value))


def _periodic_contains(interval, theta: float,
                       period: float = TWO_PI) -> bool:
    lo = float(interval.lo if hasattr(interval, "lo") else interval[0])
    hi = float(interval.hi if hasattr(interval, "hi") else interval[1])
    event = float(theta % period)
    return any(lo - _MEASURE_ATOL <= shifted <= hi + _MEASURE_ATOL
               for shifted in (event - period, event, event + period))


def _merge_leaf_intervals(leaves, verdicts) \
        -> tuple[tuple[float, float], ...]:
    """Exact producer-independent periodic merge of selected leaf intervals."""
    selected = [
        (float(item.interval.lo), float(item.interval.hi))
        for item in leaves if item.verdict in verdicts
    ]
    if not selected:
        return ()
    merged: list[tuple[float, float]] = []
    for lo, hi in selected:
        if merged and lo == merged[-1][1]:
            merged[-1] = (merged[-1][0], hi)
        else:
            merged.append((lo, hi))
    if (len(merged) > 1 and merged[0][0] == 0.0
            and merged[-1][1] == TWO_PI):
        seam = (merged[-1][0], merged[0][1] + TWO_PI)
        merged = [*merged[1:-1], seam]
        merged.sort()
    return tuple(merged)


def _rebuild_unknown_runs(leaves, stop_reason: str):
    """Rebuild maximal cyclic UNKNOWN runs and their neighbour provenance."""
    n_items = len(leaves)
    known = [index for index, item in enumerate(leaves)
             if item.verdict is not ConnectivityVerdict.UNKNOWN]
    if not known:
        reasons = tuple(dict.fromkeys(
            [item.reason for item in leaves] + [stop_reason]
        ))
        return ({
            "interval": (0.0, TWO_PI),
            "reasons": reasons,
            "slab_ids": tuple(item.slab_id for item in leaves),
            "left": None,
            "right": None,
        },), ()

    runs = []
    brackets = []
    anchor = known[0]
    offset = 1
    while offset < n_items:
        index = (anchor + offset) % n_items
        if leaves[index].verdict is not ConnectivityVerdict.UNKNOWN:
            offset += 1
            continue
        indices = []
        while offset < n_items:
            index = (anchor + offset) % n_items
            if leaves[index].verdict is not ConnectivityVerdict.UNKNOWN:
                break
            indices.append(index)
            offset += 1
        left = leaves[(indices[0] - 1) % n_items]
        right = leaves[(indices[-1] + 1) % n_items]
        lo = float(leaves[indices[0]].interval.lo)
        hi = float(leaves[indices[-1]].interval.hi)
        if indices[-1] < indices[0] or hi <= lo:
            hi += TWO_PI
        run = {
            "interval": (lo, hi),
            "reasons": tuple(dict.fromkeys(
                [leaves[index].reason for index in indices] + [stop_reason]
            )),
            "slab_ids": tuple(leaves[index].slab_id for index in indices),
            "left": left.verdict,
            "right": right.verdict,
        }
        runs.append(run)
        if left.verdict is not right.verdict:
            brackets.append(run)
    runs.sort(key=lambda item: item["interval"][0])
    brackets.sort(key=lambda item: item["interval"][0])
    return tuple(runs), tuple(brackets)


def _interval_tuple(item) -> tuple[float, float]:
    return float(item.interval.lo), float(item.interval.hi)


def _asset_contract_view(integrity: dict) -> dict:
    """Drop case-selection evidence while retaining the sealed-input contract.

    Stages intentionally select different records, so their per-record hashes
    cannot be byte-identical.  Everything that authenticates the package,
    split manifests, and runtime source must still agree exactly.
    """
    return {
        split: {
            key: value for key, value in split_integrity.items()
            if key not in {
                "selected_oracle_records",
                "oracle_records_package_inventory_checked",
            }
        }
        for split, split_integrity in integrity.items()
    }


def _validate_and_rebuild_report(
        case, report, event_tolerance: float, *, scene, robot, cfg,
        oracles):
    """Validate report provenance and rebuild all angular sets from leaves."""
    if not isinstance(report, ConnectivityIntervalReport):
        raise TypeError(
            "gate evaluator requires a real ConnectivityIntervalReport; "
            "preassembled interval tuples are not evidence"
        )
    event_tolerance = float(event_tolerance)
    if not np.isfinite(event_tolerance) or event_tolerance <= 0.0:
        raise ValueError("event_tolerance must be finite and positive")

    errors: list[str] = []
    raw_leaves = tuple(report.intervals)
    if not raw_leaves:
        errors.append("leaf_tiling_empty")
    if any(not isinstance(item, ConnectivityInterval) for item in raw_leaves):
        errors.append("leaf_type_invalid")
    leaves = tuple(item for item in raw_leaves
                   if isinstance(item, ConnectivityInterval))
    leaves = tuple(sorted(leaves, key=lambda item: item.interval.lo))
    if leaves:
        if float(leaves[0].interval.lo) != 0.0:
            errors.append("leaf_tiling_missing_zero")
        if float(leaves[-1].interval.hi) != TWO_PI:
            errors.append("leaf_tiling_missing_two_pi")
        for left, right in zip(leaves, leaves[1:]):
            if float(left.interval.hi) != float(right.interval.lo):
                errors.append("leaf_tiling_gap_or_overlap")
                break
        if any(
                not np.isfinite([item.interval.lo, item.interval.hi]).all()
                or item.interval.hi <= item.interval.lo
                for item in leaves):
            errors.append("leaf_interval_invalid")
    ids = tuple(item.slab_id for item in leaves)
    if len(set(ids)) != len(ids):
        errors.append("leaf_slab_id_duplicate")
    if float(report.period) != TWO_PI:
        errors.append("report_period_not_full_s1")
    if report.semantics != (
            "theta_safe subset true_connectivity subset theta_possible"):
        errors.append("report_semantics_drift")

    expected_start = tuple(map(float, case.episode["start_pose"][:2]))
    expected_goal = tuple(map(float, case.episode["goals"][0]))
    if tuple(map(float, report.start_xy)) != expected_start:
        errors.append("report_start_not_manifest_query")
    if tuple(map(float, report.goal_xy)) != expected_goal:
        errors.append("report_goal_not_manifest_query")
    if report.event_tolerance != event_tolerance:
        errors.append("report_event_tolerance_drift")

    decomposition = report.decomposition
    if decomposition is None:
        errors.append("report_decomposition_missing")
        slab_by_id = {}
    else:
        try:
            binding_errors = decomposition_binding_failures(
                decomposition, scene, robot, cfg, tuple(oracles),
            )
        except Exception as exc:
            errors.append(
                "decomposition_binding_check_failed_"
                f"{type(exc).__name__}"
            )
        else:
            errors.extend(
                reason if reason.startswith("decomposition_")
                else f"decomposition_binding_{reason}"
                for reason in binding_errors
            )
        if int(getattr(decomposition, "revision", -1)) != int(
                report.final_revision):
            errors.append("report_decomposition_revision_drift")
        slabs = tuple(getattr(decomposition, "slabs", ()))
        slab_by_id = {getattr(item, "slab_id", None): item for item in slabs}
        if len(slab_by_id) != len(slabs):
            errors.append("decomposition_slab_id_duplicate")
        if set(slab_by_id) != set(ids):
            errors.append("leaf_decomposition_slab_ids_drift")

    for leaf in leaves:
        slab = slab_by_id.get(leaf.slab_id)
        if slab is None:
            continue
        if (float(slab.interval.lo), float(slab.interval.hi)) != (
                float(leaf.interval.lo), float(leaf.interval.hi)):
            errors.append(f"leaf_{leaf.slab_id}_interval_provenance_drift")
            continue
        try:
            rebuilt = classify_translation_interval(
                slab, expected_start, expected_goal,
            )
        except Exception as exc:  # validator records, never trusts on failure
            errors.append(
                f"leaf_{leaf.slab_id}_reclassification_failed_"
                f"{type(exc).__name__}"
            )
            continue
        if rebuilt.verdict is not leaf.verdict:
            errors.append(f"leaf_{leaf.slab_id}_verdict_provenance_drift")
        if rebuilt.reason != leaf.reason:
            errors.append(f"leaf_{leaf.slab_id}_reason_provenance_drift")
        if int(rebuilt.depth) != int(leaf.depth):
            errors.append(f"leaf_{leaf.slab_id}_depth_provenance_drift")
        if rebuilt.diagnostics != leaf.diagnostics:
            errors.append(
                f"leaf_{leaf.slab_id}_diagnostics_provenance_drift")

    try:
        initial_revision = int(report.initial_revision)
        final_revision = int(report.final_revision)
    except (TypeError, ValueError, OverflowError):
        initial_revision = final_revision = 0
        errors.append("report_revision_invalid")
    revision_delta = final_revision - initial_revision
    if revision_delta != len(report.refinement_trace):
        errors.append("refinement_trace_revision_count_drift")
    for offset, record in enumerate(report.refinement_trace):
        before = initial_revision + offset
        after = before + 1
        if not isinstance(record, dict):
            errors.append("refinement_trace_record_type_invalid")
            continue
        parent_raw = record.get("parent_interval", ())
        children_raw = record.get("child_intervals", ())
        parent = (tuple(parent_raw)
                  if isinstance(parent_raw, (tuple, list)) else ())
        children = tuple(
            tuple(item) for item in children_raw
            if isinstance(item, (tuple, list))
        ) if isinstance(children_raw, (tuple, list)) else ()
        midpoint_partition = bool(
            len(parent) == 2
            and len(children) == 2
            and all(len(child) == 2 for child in children)
            and children[0][0] == parent[0]
            and children[0][1] == children[1][0]
            and children[1][1] == parent[1]
            and parent[0] < children[0][1] < parent[1]
        )
        if (record.get("changed") is not True
                or record.get("revision_before") != before
                or record.get("revision_after") != after
                or record.get("reason") != "binary_interval_split"
                or not midpoint_partition):
            errors.append("refinement_trace_record_provenance_drift")
    if decomposition is not None:
        history = tuple(getattr(decomposition, "refinement_history", ()))
        trace = tuple(report.refinement_trace)
        if trace and (len(history) < len(trace)
                      or tuple(history[-len(trace):]) != trace):
            errors.append("refinement_trace_not_decomposition_history_tail")

    safe = _merge_leaf_intervals(leaves, {ConnectivityVerdict.OPEN})
    possible = _merge_leaf_intervals(
        leaves, {ConnectivityVerdict.OPEN, ConnectivityVerdict.UNKNOWN},
    )
    runs, brackets = _rebuild_unknown_runs(leaves, report.stop_reason)
    unknown = tuple(item["interval"] for item in runs)
    rebuilt_brackets = tuple(item["interval"] for item in brackets)

    if any(not isinstance(item, Interval) for item in report.theta_safe):
        errors.append("reported_theta_safe_type_invalid")
    if any(not isinstance(item, Interval) for item in report.theta_possible):
        errors.append("reported_theta_possible_type_invalid")
    reported_safe = tuple(
        (float(item.lo), float(item.hi)) for item in report.theta_safe
        if isinstance(item, Interval)
    )
    reported_possible = tuple(
        (float(item.lo), float(item.hi)) for item in report.theta_possible
        if isinstance(item, Interval)
    )
    if reported_safe != safe:
        errors.append("reported_theta_safe_not_leaf_reconstruction")
    if reported_possible != possible:
        errors.append("reported_theta_possible_not_leaf_reconstruction")
    if _difference_measure(safe, possible) > _MEASURE_ATOL:
        errors.append("theta_safe_not_subset_theta_possible")

    if any(not isinstance(item, UnresolvedInterval)
           for item in report.unresolved):
        errors.append("reported_unknown_type_invalid")
    reported_runs = tuple(
        {
            "interval": _interval_tuple(item),
            "reasons": tuple(item.reasons),
            "slab_ids": tuple(item.slab_ids),
        }
        for item in report.unresolved
        if isinstance(item, UnresolvedInterval)
    )
    expected_runs = tuple(
        {key: item[key] for key in ("interval", "reasons", "slab_ids")}
        for item in runs
    )
    if reported_runs != expected_runs:
        errors.append("reported_unknown_not_maximal_leaf_reconstruction")

    if any(not isinstance(item, TransitionBracket)
           for item in report.transition_brackets):
        errors.append("reported_bracket_type_invalid")
    reported_brackets = tuple(
        {
            "interval": _interval_tuple(item),
            "left": item.left_verdict,
            "right": item.right_verdict,
            "minimum_transitions": int(item.minimum_transitions),
            "isolated": bool(item.isolated),
        }
        for item in report.transition_brackets
        if isinstance(item, TransitionBracket)
    )
    reported_unknown_intervals = {
        item["interval"] for item in reported_runs
    }
    if any(item["interval"] not in reported_unknown_intervals
           for item in reported_brackets):
        errors.append("reported_transition_bracket_not_unknown_subset")
    expected_brackets = tuple(
        {
            "interval": item["interval"],
            "left": item["left"],
            "right": item["right"],
            "minimum_transitions": 1,
            "isolated": False,
        }
        for item in brackets
    )
    if reported_brackets != expected_brackets:
        errors.append(
            "reported_brackets_not_maximal_opposite_neighbour_reconstruction"
        )
    if any(item["interval"] not in unknown for item in expected_brackets):
        errors.append("transition_bracket_not_unknown_subset")

    # Leaf resolution and successful completion within the producer's budget
    # are deliberately different facts.  The producer keeps
    # ``report.resolution_met`` false after any hard-budget stop, even when the
    # last committed leaf tiling already happens to meet the requested angular
    # tolerance.  The evaluator independently reconstructs the geometric fact
    # from those committed leaves, while still rejecting a producer that claims
    # budget-aware completion after reporting budget exhaustion.
    unknown_run_resolution_met = bool(
        not unknown
        or max(hi - lo for lo, hi in unknown) <= event_tolerance
    )
    budget_stopped = bool(
        isinstance(report.stop_reason, str)
        and report.stop_reason.endswith("_budget_exhausted")
    )
    producer_completion = report.resolution_met
    if not isinstance(producer_completion, bool):
        errors.append("reported_completion_flag_not_bool")
    elif budget_stopped and producer_completion:
        errors.append("reported_completion_despite_budget_stop")
    elif (not budget_stopped
          and producer_completion != unknown_run_resolution_met):
        errors.append("reported_completion_not_unknown_run_resolution")
    return {
        "errors": tuple(dict.fromkeys(errors)),
        "safe": safe,
        "possible": possible,
        "unknown": unknown,
        "brackets": rebuilt_brackets,
        "resolution_met": unknown_run_resolution_met,
    }


def evaluate_gate_interval_report(
        case, report, *, event_tolerance: float, scene, robot, cfg,
        oracles) -> dict:
    """Independently validate and score one real compiler report."""
    rebuilt = _validate_and_rebuild_report(
        case, report, event_tolerance,
        scene=scene, robot=robot, cfg=cfg, oracles=oracles,
    )
    truth = case.truth_open_intervals_rad(TWO_PI)
    safe = rebuilt["safe"]
    possible = rebuilt["possible"]
    unknown = rebuilt["unknown"]
    brackets = rebuilt["brackets"]
    events = case.truth_events_rad(TWO_PI)
    false_open = _difference_measure(safe, truth)
    false_closed = _difference_measure(truth, possible)
    event_hits = sum(
        any(_periodic_contains(interval, event) for interval in unknown)
        for event in events
    )
    bracket_metrics = evaluate_event_brackets(
        events, brackets, period=TWO_PI,
    )
    structural_pass = not rebuilt["errors"]
    safe_subset_truth = bool(
        structural_pass and false_open <= _MEASURE_ATOL)
    truth_subset_possible = bool(
        structural_pass and false_closed <= _MEASURE_ATOL)
    max_bracket_width = bracket_metrics["max_bracket_width_deg"]
    return {
        "analytic_class": (
            "closed" if not truth
            else "full_open" if _measure(truth) >= TWO_PI - _MEASURE_ATOL
            else "partial"
        ),
        "structural_contract_pass": structural_pass,
        "structural_contract_errors": list(rebuilt["errors"]),
        "theta_safe_subset_theta_possible": bool(
            structural_pass
            and _difference_measure(safe, possible) <= _MEASURE_ATOL
        ),
        "theta_safe_subset_theta_true": safe_subset_truth,
        "theta_true_subset_theta_possible": truth_subset_possible,
        "sandwich_holds_in_measure": bool(
            safe_subset_truth and truth_subset_possible),
        "resolution_met": bool(
            structural_pass and rebuilt["resolution_met"]),
        "event_tolerance_deg": float(np.rad2deg(event_tolerance)),
        "truth_measure_deg": float(np.rad2deg(_measure(truth))),
        "safe_measure_deg": float(np.rad2deg(_measure(safe))),
        "possible_measure_deg": float(np.rad2deg(_measure(possible))),
        "certified_closed_measure_deg": float(np.rad2deg(
            max(0.0, TWO_PI - _measure(possible))
        )),
        "unresolved_measure_deg": float(np.rad2deg(_measure(unknown))),
        "false_open_measure_deg": float(np.rad2deg(false_open)),
        "false_closed_measure_deg": float(np.rad2deg(false_closed)),
        "event_count": len(events),
        "events_covered_by_unresolved": int(event_hits),
        "event_recall": float(event_hits / len(events)) if events else 1.0,
        "transition_bracket_count": len(brackets),
        "matched_transition_brackets": int(
            bracket_metrics["matched_truth_count"]),
        "bracket_recall": float(bracket_metrics["event_recall"]),
        "transition_bracket_recall": float(
            bracket_metrics["event_recall"]),
        "spurious_transition_brackets": int(
            bracket_metrics["spurious_bracket_count"]),
        "max_merged_bracket_width_deg": max_bracket_width,
    }


def evaluate_gate_interval_sets(*_args, **_kwargs):
    """Reject the old tuple-based scoring surface.

    Accepting preassembled safe/possible/unknown tuples let a caller provide
    truth itself as the prediction and fabricate transition brackets.  Only a
    replayable leaf report is now valid evidence.
    """
    raise TypeError(
        "tuple-based gate evaluation is forbidden; use "
        "evaluate_gate_interval_report with a real compiler report"
    )


def _serialize_report(report) -> dict:
    return {
        "semantics": report.semantics,
        "period_deg": float(np.rad2deg(report.period)),
        "initial_revision": int(report.initial_revision),
        "final_revision": int(report.final_revision),
        "stop_reason": report.stop_reason,
        "n_refinements": int(report.n_refinements),
        "resolution_met": bool(report.resolution_met),
        "event_tolerance_deg": (
            float(np.rad2deg(report.event_tolerance))
            if report.event_tolerance is not None else None
        ),
        "support_calls": int(report.support_calls),
        "max_support_calls": report.max_support_calls,
        "wall_seconds": float(report.wall_seconds),
        "max_wall_seconds": report.max_wall_seconds,
        "intervals": [
            {
                "slab_id": int(item.slab_id),
                "depth": int(item.depth),
                "lo_deg": float(np.rad2deg(item.interval.lo)),
                "hi_deg": float(np.rad2deg(item.interval.hi)),
                "verdict": item.verdict.value,
                "reason": item.reason,
                "diagnostics": item.diagnostics,
            }
            for item in report.intervals
        ],
        "theta_safe_deg": [
            [float(np.rad2deg(item.lo)), float(np.rad2deg(item.hi))]
            for item in report.theta_safe
        ],
        "theta_possible_deg": [
            [float(np.rad2deg(item.lo)), float(np.rad2deg(item.hi))]
            for item in report.theta_possible
        ],
        "unresolved": [
            {
                "lo_deg": float(np.rad2deg(item.interval.lo)),
                "hi_deg": float(np.rad2deg(item.interval.hi)),
                "reasons": list(item.reasons),
                "slab_ids": list(item.slab_ids),
            }
            for item in report.unresolved
        ],
        "transition_brackets": [
            {
                "lo_deg": float(np.rad2deg(item.interval.lo)),
                "hi_deg": float(np.rad2deg(item.interval.hi)),
                "left_verdict": item.left_verdict.value,
                "right_verdict": item.right_verdict.value,
                "minimum_transitions": int(item.minimum_transitions),
                "isolated": bool(item.isolated),
            }
            for item in report.transition_brackets
        ],
        "refinement_trace": [dict(item) for item in report.refinement_trace],
    }


def _ledger_oracles(candidate_query, ledger: WorkLedger, *,
                    scene, robot, workspace):
    wrapped = [
        PairOracle(
            oracle.pair_id, oracle.scene, oracle.body, ledger=ledger,
        )
        for oracle in candidate_query.oracles
    ]
    return CandidateOracleList(
        wrapped, scene, robot, workspace,
        decisions=candidate_query.decisions,
        stats=candidate_query.stats,
    )


def _failed_metrics(case, errors: list[str], event_tolerance: float) -> dict:
    truth = case.truth_open_intervals_rad(TWO_PI)
    events = case.truth_events_rad(TWO_PI)
    return {
        "analytic_class": (
            "closed" if not truth
            else "full_open" if _measure(truth) >= TWO_PI - _MEASURE_ATOL
            else "partial"
        ),
        "structural_contract_pass": False,
        "structural_contract_errors": list(errors),
        "theta_safe_subset_theta_possible": False,
        "theta_safe_subset_theta_true": False,
        "theta_true_subset_theta_possible": False,
        "sandwich_holds_in_measure": False,
        "resolution_met": False,
        "event_tolerance_deg": float(np.rad2deg(event_tolerance)),
        "truth_measure_deg": float(np.rad2deg(_measure(truth))),
        "safe_measure_deg": None,
        "possible_measure_deg": None,
        "certified_closed_measure_deg": None,
        "unresolved_measure_deg": None,
        "false_open_measure_deg": None,
        "false_closed_measure_deg": None,
        "event_count": len(events),
        "events_covered_by_unresolved": 0,
        "event_recall": 0.0 if events else 1.0,
        "transition_bracket_count": 0,
        "matched_transition_brackets": 0,
        "bracket_recall": 0.0 if events else 1.0,
        "transition_bracket_recall": 0.0 if events else 1.0,
        "spurious_transition_brackets": 0,
        "max_merged_bracket_width_deg": None,
    }


def _acceptance_decision(metrics: dict, *, certified_complete: bool,
                         budget_reasons, event_tolerance: float) -> dict:
    """Apply the per-case gate without conflating UNKNOWN with unsoundness."""
    errors = list(budget_reasons)
    if not metrics["structural_contract_pass"]:
        errors.append("structural_contract_failed")
    analytic_class = metrics["analytic_class"]
    if analytic_class == "full_open":
        if not certified_complete:
            errors.append("full_open_not_fully_certified")
        if (metrics["safe_measure_deg"] is None
                or metrics["safe_measure_deg"] < 360.0
                - float(np.rad2deg(_MEASURE_ATOL))):
            errors.append("full_open_safe_measure_incomplete")
    elif analytic_class == "closed":
        if not certified_complete:
            errors.append("closed_not_fully_certified")
        if (metrics["possible_measure_deg"] is None
                or metrics["possible_measure_deg"]
                > float(np.rad2deg(_MEASURE_ATOL))):
            errors.append("closed_possible_measure_nonzero")
    else:
        partial_checks = {
            "partial_safe_arc_missing": (
                metrics["safe_measure_deg"] is None
                or metrics["safe_measure_deg"] <= 0.0),
            "partial_certified_closed_arc_missing": (
                metrics["certified_closed_measure_deg"] is None
                or metrics["certified_closed_measure_deg"] <= 0.0),
            "partial_sandwich_unsound": not metrics[
                "sandwich_holds_in_measure"],
            "partial_event_recall_incomplete": not (
                metrics["event_count"] == 4
                and metrics["events_covered_by_unresolved"] == 4
                and metrics["event_recall"] == 1.0),
            "partial_bracket_matching_incomplete": not (
                metrics["transition_bracket_count"] == 4
                and metrics["matched_transition_brackets"] == 4
                and metrics["bracket_recall"] == 1.0
                and metrics["spurious_transition_brackets"] == 0),
            "partial_resolution_not_met": not metrics["resolution_met"],
            "partial_bracket_width_exceeds_tolerance": (
                metrics["max_merged_bracket_width_deg"] is None
                or metrics["max_merged_bracket_width_deg"]
                > float(np.rad2deg(event_tolerance))
                + float(np.rad2deg(_MEASURE_ATOL))),
        }
        errors.extend(name for name, failed in partial_checks.items() if failed)
    errors = list(dict.fromkeys(errors))
    return {
        "pass": not errors,
        "reason": "acceptance_policy_satisfied" if not errors else errors[0],
        "errors": errors,
    }


def _report_budget_contract_errors(
        report, *, max_support_calls: int, max_wall_seconds: float,
        max_refinements: int) -> list[str]:
    """Independently check producer-reported limits and consumption."""
    errors = []
    support_calls = report.support_calls
    support_limit = report.max_support_calls
    if (isinstance(support_limit, bool)
            or not isinstance(support_limit, Integral)
            or int(support_limit) != int(max_support_calls)):
        errors.append("report_support_limit_contract_drift")
    if (isinstance(support_calls, bool)
            or not isinstance(support_calls, Integral)
            or int(support_calls) < 0):
        errors.append("report_support_calls_invalid")
    elif int(support_calls) > int(max_support_calls):
        errors.append("report_support_budget_exceeded")

    wall = report.wall_seconds
    wall_limit = report.max_wall_seconds
    if (isinstance(wall_limit, bool)
            or not isinstance(wall_limit, Real)
            or not np.isfinite(float(wall_limit))
            or float(wall_limit) != float(max_wall_seconds)):
        errors.append("report_wall_limit_contract_drift")
    if (isinstance(wall, bool)
            or not isinstance(wall, Real)
            or not np.isfinite(float(wall))
            or float(wall) < 0.0):
        errors.append("report_wall_seconds_invalid")
    elif float(wall) > float(max_wall_seconds):
        errors.append("report_wall_budget_exceeded")

    refinements = report.n_refinements
    if (isinstance(refinements, bool)
            or not isinstance(refinements, Integral)
            or int(refinements) < 0):
        errors.append("report_refinement_count_invalid")
    elif int(refinements) > int(max_refinements):
        errors.append("report_refinement_budget_exceeded")
    return errors


def run_gate_interval_case(instance, cfg: Config, *, limits: dict,
                           max_refinements: int,
                           max_wall_seconds: float,
                           event_tolerance: float) -> dict:
    """Run one complete-scene S1 evaluation under one absolute deadline."""
    case = instance.case
    start_xy = tuple(map(float, case.episode["start_pose"][:2]))
    goals = case.episode["goals"]
    if len(goals) != 1:
        raise ValueError(
            "Atlas G1 full interval v0 requires exactly one goal per episode"
        )
    goal_xy = tuple(map(float, goals[0]))
    event_tolerance = float(event_tolerance)
    max_wall_seconds = float(max_wall_seconds)
    ledger = WorkLedger(limits=limits, default_phase="compile")
    t0 = time.perf_counter()
    case_deadline = t0 + max_wall_seconds
    candidate_query = None
    oracles = ()
    report = None
    budget_reasons: list[str] = []
    build_seconds = 0.0
    interval_seconds = 0.0
    interval_wall_limit = None
    try:
        build_start = time.perf_counter()
        try:
            candidate_query = query_candidate_pairs(
                instance.scene, instance.robot, instance.scene.workspace,
            )
            oracles = _ledger_oracles(
                candidate_query, ledger,
                scene=instance.scene, robot=instance.robot,
                workspace=instance.scene.workspace,
            )
            with ledger.phase("full_compiler"):
                decomposition = build_connectivity_slabs(
                    instance.scene, instance.robot, cfg, oracles,
                    ledger=ledger, deadline=case_deadline,
                )
        finally:
            build_finished = time.perf_counter()
            build_seconds = build_finished - build_start
        if build_finished >= case_deadline:
            # The case wall allowance includes candidate selection and the
            # complete connectivity-specific build.  Do not start another
            # synchronous classification after that allowance has expired.
            budget_reasons.append("wall_budget_exhausted")
        else:
            interval_start = time.perf_counter()
            remaining_wall = max(0.0, case_deadline - interval_start)
            interval_wall_limit = remaining_wall
            try:
                with ledger.phase("connectivity_interval_refinement"):
                    report = certified_connectivity_intervals(
                        instance.scene, instance.robot, cfg, oracles,
                        decomposition, start_xy, goal_xy,
                        max_refinements=max_refinements,
                        max_support_calls=cfg.query.max_support_calls,
                        max_wall_seconds=remaining_wall,
                        event_tolerance=event_tolerance,
                        deadline=case_deadline,
                    )
            finally:
                interval_seconds = time.perf_counter() - interval_start
    except CompilationDeadlineExceeded:
        budget_reasons.append("wall_budget_exhausted")
    except BudgetExceeded as exc:
        budget_reasons.append(f"{exc.kind}_budget_exhausted")
    finally:
        wall_seconds = time.perf_counter() - t0

    if wall_seconds > max_wall_seconds:
        budget_reasons.append("wall_budget_exceeded")
    if report is not None:
        if (isinstance(report.stop_reason, str)
                and report.stop_reason.endswith("_budget_exhausted")):
            budget_reasons.append(report.stop_reason)
        budget_reasons.extend(_report_budget_contract_errors(
            report,
            max_support_calls=cfg.query.max_support_calls,
            max_wall_seconds=interval_wall_limit,
            max_refinements=max_refinements,
        ))
        metrics = evaluate_gate_interval_report(
            case, report, event_tolerance=event_tolerance,
            scene=instance.scene, robot=instance.robot, cfg=cfg,
            oracles=oracles,
        )
        serialized = _serialize_report(report)
        stop_reason = report.stop_reason
        reasons = [reason for item in report.unresolved
                   for reason in item.reasons]
        certified_complete = not report.unresolved
        slab_count = len(report.intervals)
        final_revision = int(report.final_revision)
    else:
        stop_reason = budget_reasons[0] if budget_reasons else "unknown_failure"
        metrics = _failed_metrics(case, [stop_reason], event_tolerance)
        serialized = None
        reasons = []
        certified_complete = False
        slab_count = None
        final_revision = None

    ledger_snapshot = ledger.snapshot()
    for kind, limit in limits.items():
        if ledger_snapshot["totals"][kind] > int(limit):
            budget_reasons.append(f"{kind}_postcheck_budget_exceeded")
    budget_reasons = list(dict.fromkeys(budget_reasons))
    unknown_reasons = tuple(dict.fromkeys([
        *reasons, *metrics["structural_contract_errors"], *budget_reasons,
    ]))
    analytic = {
        "source": (
            "formula independently recomputed from a split manifest bound "
            "by the pinned split seal and robot axes from the pinned G1 "
            "registry"
        ),
        "gate_center_deg": float(case.gate_center_deg),
        "gate_half_angle_deg": float(case.gate_half_angle_deg),
        "open_intervals_deg": [
            [float(np.rad2deg(lo)), float(np.rad2deg(hi))]
            for lo, hi in case.truth_open_intervals_rad(TWO_PI)
        ],
        "event_angles_deg": [
            float(np.rad2deg(value))
            for value in case.truth_events_rad(TWO_PI)
        ],
    }

    acceptance = _acceptance_decision(
        metrics, certified_complete=certified_complete,
        budget_reasons=budget_reasons, event_tolerance=event_tolerance,
    )
    return {
        "query_id": case.query_id,
        "split": case.split,
        "family": case.family,
        "robot_id": case.robot_id,
        "scene": dict(case.episode["scene"]),
        "start_xy": list(start_xy),
        "goal_xy": list(goal_xy),
        "atlas_truth": analytic,
        "oracle_record_cross_check": {
            "role": (
                "package_inventory_checked_not_independently_pinned_"
                "secondary_only"
            ),
            "convergence_status": case.convergence_status,
            "fine_reachable_per_goal": (
                list(case.fine_reachable_per_goal)
                if case.fine_reachable_per_goal is not None else None
            ),
            "measured_gate_half_angle_deg":
                case.measured_gate_half_angle_deg,
        },
        "compiler": {
            "candidate_pairs": int(candidate_query.stats.candidate_pairs),
            "candidate_query_stats": asdict(candidate_query.stats),
            "slab_count": slab_count,
            "final_revision": final_revision,
            "certified_complete": bool(certified_complete),
            "status": "CERTIFIED_COMPLETE" if certified_complete else "UNKNOWN",
            "stop_reason": stop_reason,
            "unknown_or_budget_reasons": list(unknown_reasons),
            "report": serialized,
        },
        "acceptance": {
            **acceptance,
            "note": (
                "Partial gates correctly retain UNKNOWN transition brackets; "
                "this acceptance flag does not replace compiler three-valued "
                "status."
            ),
        },
        "metrics": metrics,
        "ledger": ledger_snapshot,
        "timing": {
            "full_compiler_seconds": float(build_seconds),
            "connectivity_interval_seconds": float(interval_seconds),
            "total_wall_seconds": float(wall_seconds),
            "max_wall_seconds": max_wall_seconds,
            "budget_exceeded": bool(budget_reasons),
            "budget_reasons": list(budget_reasons),
        },
    }


def _aggregate(rows: list[dict]) -> dict:
    n = len(rows)
    events = sum(row["metrics"]["event_count"] for row in rows)
    event_hits = sum(row["metrics"]["events_covered_by_unresolved"]
                     for row in rows)
    bracket_hits = sum(row["metrics"]["matched_transition_brackets"]
                       for row in rows)
    reasons = Counter(
        reason for row in rows
        for reason in row["compiler"]["unknown_or_budget_reasons"]
    )
    ledger_totals = {
        kind: int(sum(row["ledger"]["totals"].get(kind, 0)
                      for row in rows))
        for kind in WORK_KINDS
    }

    def metric_sum(name):
        return float(sum(
            value for row in rows
            if (value := row["metrics"].get(name)) is not None
        ))

    return {
        "n_cases": n,
        "certified_complete_cases": int(sum(
            row["compiler"]["certified_complete"] for row in rows
        )),
        "unknown_cases": int(sum(
            not row["compiler"]["certified_complete"] for row in rows
        )),
        "acceptance_pass_cases": int(sum(
            row["acceptance"]["pass"] for row in rows
        )),
        "structural_contract_pass_cases": int(sum(
            row["metrics"]["structural_contract_pass"] for row in rows
        )),
        "resolution_met_cases": int(sum(
            row["metrics"]["resolution_met"] for row in rows
        )),
        "budget_exceeded_cases": int(sum(
            row["timing"]["budget_exceeded"] for row in rows
        )),
        "theta_safe_subset_theta_true_cases": int(sum(
            row["metrics"]["theta_safe_subset_theta_true"] for row in rows
        )),
        "theta_true_subset_theta_possible_cases": int(sum(
            row["metrics"]["theta_true_subset_theta_possible"] for row in rows
        )),
        "sandwich_holds_in_measure_cases": int(sum(
            row["metrics"]["sandwich_holds_in_measure"] for row in rows
        )),
        "truth_events": int(events),
        "events_covered_by_unresolved": int(event_hits),
        "event_recall": float(event_hits / events) if events else 1.0,
        "events_matched_by_transition_brackets": int(bracket_hits),
        "bracket_recall": float(bracket_hits / events) if events else 1.0,
        "false_open_measure_deg": metric_sum("false_open_measure_deg"),
        "false_closed_measure_deg": metric_sum("false_closed_measure_deg"),
        "unresolved_measure_deg": metric_sum("unresolved_measure_deg"),
        "ledger_totals": ledger_totals,
        "support_value_evals": ledger_totals["support_value_evals"],
        "support_point_evals": ledger_totals["support_point_evals"],
        "wall_seconds": float(sum(
            row["timing"]["total_wall_seconds"] for row in rows
        )),
        "unknown_or_budget_reason_counts": dict(sorted(reasons.items())),
    }


def _recompute_row_acceptance(row: dict):
    """Recompute a serialized row's acceptance instead of trusting its flag."""
    contract_errors = []
    compiler = row.get("compiler")
    metrics = row.get("metrics")
    timing = row.get("timing")
    if not isinstance(compiler, dict) or not isinstance(metrics, dict) \
            or not isinstance(timing, dict):
        raise ValueError("case acceptance inputs must be mappings")

    compiler_stop_reason = compiler.get("stop_reason")
    if (not isinstance(compiler_stop_reason, str)
            or not compiler_stop_reason):
        compiler_stop_reason = None
        contract_errors.append("compiler_stop_reason_invalid")

    payload = compiler.get("report")
    report_stop_reason = None
    report_completion = None
    if payload is None:
        certified_complete = False
    elif not isinstance(payload, dict):
        certified_complete = False
        contract_errors.append("serialized_report_unresolved_invalid")
    else:
        unresolved = payload.get("unresolved")
        if not isinstance(unresolved, list):
            certified_complete = False
            contract_errors.append("serialized_report_unresolved_invalid")
        else:
            certified_complete = not unresolved
        report_stop_reason = payload.get("stop_reason")
        if (not isinstance(report_stop_reason, str)
                or not report_stop_reason):
            report_stop_reason = None
            contract_errors.append("serialized_report_stop_reason_invalid")
        report_completion = payload.get("resolution_met")
        if not isinstance(report_completion, bool):
            report_completion = None
            contract_errors.append(
                "serialized_report_completion_flag_not_bool"
            )
    if compiler.get("certified_complete") is not certified_complete:
        contract_errors.append("certified_complete_cache_drift")

    if (compiler_stop_reason is not None
            and report_stop_reason is not None
            and compiler_stop_reason != report_stop_reason):
        contract_errors.append("compiler_report_stop_reason_drift")
    budget_stop_reasons = list(dict.fromkeys(
        reason for reason in (compiler_stop_reason, report_stop_reason)
        if reason is not None and reason.endswith("_budget_exhausted")
    ))
    if budget_stop_reasons and report_completion is True:
        contract_errors.append(
            "serialized_report_completion_despite_budget_stop"
        )

    raw_reasons = timing.get("budget_reasons")
    if (not isinstance(raw_reasons, list)
            or any(not isinstance(reason, str) or not reason
                   for reason in raw_reasons)):
        budget_reasons = ["budget_reason_contract_invalid"]
        contract_errors.append("budget_reasons_invalid")
    else:
        budget_reasons = list(raw_reasons)
    missing_stop_reasons = [
        reason for reason in budget_stop_reasons
        if reason not in budget_reasons
    ]
    if missing_stop_reasons:
        budget_reasons.extend(missing_stop_reasons)
        contract_errors.append("budget_stop_missing_from_timing")
    reported_budget = timing.get("budget_exceeded")
    if not isinstance(reported_budget, bool) \
            or reported_budget != bool(budget_reasons):
        contract_errors.append("budget_exceeded_cache_drift")
        if reported_budget is True and not budget_reasons:
            budget_reasons.append("reported_budget_exceeded")

    tolerance_deg = metrics.get("event_tolerance_deg")
    if (isinstance(tolerance_deg, bool)
            or not isinstance(tolerance_deg, Real)
            or not np.isfinite(float(tolerance_deg))
            or float(tolerance_deg) <= 0.0):
        raise ValueError("case event tolerance is invalid")
    decision = _acceptance_decision(
        metrics,
        certified_complete=certified_complete,
        budget_reasons=budget_reasons,
        event_tolerance=float(np.deg2rad(tolerance_deg)),
    )
    cached = row.get("acceptance")
    if not isinstance(cached, dict):
        contract_errors.append("acceptance_cache_invalid")
    elif (cached.get("pass") is not decision["pass"]
          or cached.get("reason") != decision["reason"]
          or cached.get("errors") != decision["errors"]):
        contract_errors.append("acceptance_cache_drift")
    return decision, contract_errors


def _stage_gate(report: dict, expected) -> dict:
    """Evaluate a declared stage policy without suppressing its artifact."""
    errors = []
    if not isinstance(expected, dict):
        return {
            "declared": False,
            "pass": False,
            "policy": None,
            "errors": ["expected_stage_contract_missing"],
        }
    expected_count = expected.get("expected_n_cases")
    expected_ids = expected.get("expected_case_ids")
    policy = expected.get("acceptance_policy")
    if (isinstance(expected_count, bool)
            or not isinstance(expected_count, int)
            or expected_count < 1):
        errors.append("expected_n_cases_invalid")
    if (not isinstance(expected_ids, list) or not expected_ids
            or any(not isinstance(value, str) or not value
                   for value in expected_ids)
            or len(set(expected_ids)) != len(expected_ids)):
        errors.append("expected_case_ids_invalid")
        expected_ids = []
    actual_ids = report["scope"]["case_ids"]
    if isinstance(expected_count, int) and not isinstance(expected_count, bool):
        if report["scope"]["n_cases"] != expected_count:
            errors.append("expected_n_cases_mismatch")
    if expected_ids and actual_ids != expected_ids:
        errors.append("expected_case_ids_mismatch")

    rows = report["cases"]
    if policy == "all_cases_containment":
        for row in rows:
            metrics = row["metrics"]
            if not metrics["structural_contract_pass"]:
                errors.append(f"{row['query_id']}:structural_contract_failed")
            if not metrics["theta_safe_subset_theta_true"]:
                errors.append(f"{row['query_id']}:safe_not_subset_truth")
            if not metrics["theta_true_subset_theta_possible"]:
                errors.append(f"{row['query_id']}:truth_not_subset_possible")
            if row["timing"]["budget_exceeded"]:
                errors.append(f"{row['query_id']}:budget_exceeded")
    elif policy == "all_cases_accept":
        for row in rows:
            try:
                decision, acceptance_contract_errors = \
                    _recompute_row_acceptance(row)
            except (KeyError, TypeError, ValueError, OverflowError):
                errors.append(f"{row['query_id']}:acceptance_record_invalid")
                continue
            errors.extend(
                f"{row['query_id']}:{reason}"
                for reason in acceptance_contract_errors
            )
            if not decision["pass"]:
                errors.append(f"{row['query_id']}:acceptance_failed")
    else:
        errors.append("acceptance_policy_invalid")
    return {
        "declared": True,
        "pass": not errors,
        "policy": policy,
        "expected_n_cases": expected_count,
        "expected_case_ids": expected_ids,
        "errors": errors,
    }


def _cross_stage_contract_errors(stage_reports) -> list[str]:
    """Reject stage-gate failures and evaluator/input contract drift."""
    errors: list[str] = []
    reference = stage_reports[0]
    for row in stage_reports:
        if not row["gate"]["declared"]:
            errors.append(
                f"{row['stage']}:expected_stage_contract_missing")
        if not row["gate"]["pass"]:
            errors.append(f"{row['stage']}:stage_gate_failed")
        if row["truth_contract"] != reference["truth_contract"]:
            errors.append(f"{row['stage']}:truth_contract_drift")
        if _asset_contract_view(row["asset_integrity"]) != \
                _asset_contract_view(reference["asset_integrity"]):
            errors.append(f"{row['stage']}:asset_contract_drift")
        if row["metric_contract"] != reference["metric_contract"]:
            errors.append(f"{row['stage']}:metric_contract_drift")
        if row["profile_contract"] != reference["profile_contract"]:
            errors.append(f"{row['stage']}:profile_contract_drift")
    return errors


def run_atlas_gate_interval_spec(spec: dict, *, spec_dir: str | Path = ".",
                                 allow_blind: bool = False) -> dict:
    """Execute a sealed-manifest Atlas full-compiler interval spec.

    Formal status is available only through a complete top-level profile
    validation.  The public API intentionally has no inherited-profile
    capability: stage execution is an implementation detail below, so callers
    cannot mark an arbitrary weak leaf spec as formal.
    """
    if spec.get("kind") != "atlas_g1_full_gate_intervals":
        raise ValueError("wrong benchmark kind for Atlas gate intervals")
    profile_contract = _formal_profile_contract(spec)
    base = Path(spec_dir).resolve()
    return _run_atlas_gate_interval_spec(
        spec, base=base, allow_blind=allow_blind,
        profile_contract=profile_contract,
    )


def _run_atlas_gate_interval_spec(spec: dict, *, base: Path,
                                  allow_blind: bool,
                                  profile_contract: dict) -> dict:
    """Execute a spec after its public top-level profile was validated."""
    stages = spec.get("stages")
    if stages is not None:
        if (not isinstance(stages, list) or not stages
                or any(not isinstance(stage, dict) for stage in stages)):
            raise ValueError("stages must be a nonempty list of mappings")
        common = {key: value for key, value in spec.items()
                  if key != "stages"}
        stage_reports = []
        stage_names = set()
        for index, stage in enumerate(stages):
            stage_name = str(stage.get("stage", f"stage_{index}"))
            if stage_name in stage_names:
                raise ValueError(f"duplicate benchmark stage: {stage_name}")
            stage_names.add(stage_name)
            merged = {**common, **stage}
            merged.pop("stage", None)
            merged.pop("formal_profile", None)
            merged["name"] = f"{common.get('name', 'atlas-g1')}:{stage_name}"
            result = _run_atlas_gate_interval_spec(
                merged, base=base, allow_blind=allow_blind,
                profile_contract=profile_contract,
            )
            stage_reports.append({"stage": stage_name, **result})
        contract_errors = _cross_stage_contract_errors(stage_reports)
        actual_stage_order = [row["stage"] for row in stage_reports]
        if (profile_contract["validated"]
                and actual_stage_order != profile_contract["stage_order"]):
            contract_errors.append("formal_stage_order_mismatch")
        result = {
            "schema_version": 1,
            "name": str(spec.get(
                "name", "atlas-g1-full-gate-connectivity-intervals")),
            "kind": "atlas_g1_full_gate_intervals_staged",
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "stage_order": actual_stage_order,
            "profile_contract": profile_contract,
            "truth_contract": stage_reports[0]["truth_contract"],
            "metric_contract": stage_reports[0]["metric_contract"],
            "asset_integrity": _asset_contract_view(
                stage_reports[0]["asset_integrity"]),
            "asset_integrity_scope": (
                "selection-invariant sealed-input contract; per-stage "
                "selected-record evidence is stored separately"
            ),
            "asset_selection_integrity_by_stage": {
                row["stage"]: row["asset_integrity"]
                for row in stage_reports
            },
            "stages": stage_reports,
            "aggregate_by_stage": {
                row["stage"]: row["aggregate"] for row in stage_reports
            },
            "cross_stage_contract": {
                "independent_cold_runs": True,
                "initial_and_refined_results_are_not_pooled": True,
                "refined_stage_may_select_a_declared_case_subset": True,
            },
            "gate": {
                "declared": True,
                "pass": not contract_errors,
                "policy": "all_declared_stage_gates_and_contracts",
                "errors": contract_errors,
                "formal_profile": profile_contract["profile"],
                "formal_profile_validated": profile_contract["validated"],
                "scope": profile_contract["mode"],
            },
        }
        return result
    atlas_path = Path(spec["atlas_root"])
    if not atlas_path.is_absolute():
        atlas_path = (base / atlas_path).resolve()
    store = AtlasAssetStore(atlas_path)
    splits = tuple(map(str, spec.get("splits", ("dev", "validation"))))
    if "blind" in splits and not allow_blind:
        # AtlasAssetStore also gates this, but fail before any non-blind work.
        raise ValueError(
            "blind Atlas evaluation requires explicit allow_blind=True"
        )
    case_ids_raw = spec.get("case_ids")
    case_ids = set(map(str, case_ids_raw)) if case_ids_raw else None
    families_raw = spec.get("families", ["G1"])
    robot_ids_raw = spec.get("robot_ids")
    cases, integrity = store.load_cases(
        splits,
        case_ids=case_ids,
        families=set(map(str, families_raw)) if families_raw else None,
        robot_ids=set(map(str, robot_ids_raw)) if robot_ids_raw else None,
        partial_gate_only=bool(spec.get("partial_gate_only", False)),
        allow_blind=allow_blind,
    )
    cfg = _compiler_config(
        spec.get("compiler", {}), source=f"{base}:inline_compiler",
    )
    if cfg.pair_approx.certificate_mode != "theorem":
        raise ValueError(
            "Atlas connectivity intervals require theorem certificate mode"
        )

    budget = spec.get("budget", {})
    limits_raw = budget.get("limits_per_case", {
        "support_value_evals": cfg.query.max_support_calls,
        "support_point_evals": cfg.query.max_support_calls,
    })
    if not isinstance(limits_raw, dict):
        raise ValueError("budget.limits_per_case must be a mapping")
    limits = {str(key): int(value) for key, value in limits_raw.items()}
    max_refinements = int(budget.get(
        "max_refinements_per_case", cfg.query.max_refinement_rounds,
    ))
    if max_refinements < 0:
        raise ValueError("max_refinements_per_case must be non-negative")
    max_wall = float(budget.get(
        "max_wall_seconds_per_case",
        budget.get("wall_warning_seconds_per_case",
                   cfg.query.max_wall_seconds),
    ))
    if not np.isfinite(max_wall) or max_wall < 0.0:
        raise ValueError("per-case wall limit must be finite and non-negative")
    event_tolerance_deg = float(spec.get("event_tolerance_deg", 5.0))
    if not np.isfinite(event_tolerance_deg) or event_tolerance_deg <= 0.0:
        raise ValueError("event_tolerance_deg must be finite and positive")
    event_tolerance = float(np.deg2rad(event_tolerance_deg))

    rows = []
    materialization_seconds = 0.0
    for case in cases:
        started = time.perf_counter()
        instance = store.materialize(case)
        materialization_seconds += time.perf_counter() - started
        rows.append(run_gate_interval_case(
            instance, cfg, limits=limits,
            max_refinements=max_refinements,
            max_wall_seconds=max_wall,
            event_tolerance=event_tolerance,
        ))

    report = {
        "schema_version": 1,
        "name": str(spec.get(
            "name", "atlas-g1-full-gate-connectivity-intervals")),
        "kind": "atlas_g1_full_gate_intervals",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "profile_contract": profile_contract,
        "scope": {
            "splits": list(splits),
            "blind_allowed": bool(allow_blind),
            "n_cases": len(rows),
            "case_ids": [row["query_id"] for row in rows],
            "full_scene_supports": True,
            "full_orientation_period_deg": 360.0,
        },
        "truth_contract": {
            "primary": (
                "G1 formula recomputed from split-manifest fields bound by "
                "the independently pinned split seal and robot axes from "
                "the independently pinned G1 registry"
            ),
            "split_manifest_binding": (
                "digest recorded in independently pinned split_seals.json"
            ),
            "robot_registry_binding": (
                "runtime path and digest verified against an independently "
                "pinned source digest"
            ),
            "oracle_record_role": (
                "package-inventory checked, not independently pinned, "
                "cross-check only"
            ),
            "legacy_splatc_gmc_called": False,
            "manifest_fields": list(store.MANIFEST_FIELDS),
            "oracle_record_fields": list(store.RECORD_FIELDS),
        },
        "compiler_contract": {
            "path": (
                "all Atlas supports -> conservative candidate pairs -> "
                "theorem midpoint PairSandwichSlice sets -> interval-wide "
                "pair certificates and free-space cover assemblies -> "
                "fixed-translation connectivity intervals"
            ),
            "pair_kernel_only": False,
            "certificate_mode": cfg.pair_approx.certificate_mode,
            "config": {
                "geometry": asdict(cfg.geometry),
                "pair_approx": asdict(cfg.pair_approx),
                "orientation": asdict(cfg.orientation),
                "query": asdict(cfg.query),
            },
        },
        "metric_contract": {
            "sandwich": "theta_safe subset theta_true subset theta_possible",
            "event_recall": (
                "fraction of analytic transitions covered by any unresolved run"
            ),
            "bracket_recall": (
                "fraction of analytic transitions one-to-one matched to an "
                "unresolved run with opposite certified neighbours"
            ),
            "false_open_measure": "measure(theta_safe minus theta_true)",
            "false_closed_measure": "measure(theta_true minus theta_possible)",
            "angle_unit": "degree",
            "event_tolerance_deg": event_tolerance_deg,
            "sets_rebuilt_from_leaf_tiling": True,
        },
        "budget_contract": {
            "limits_per_case": limits,
            "max_refinements_per_case": max_refinements,
            "max_wall_seconds_per_case": max_wall,
            "wall_limit_is_acceptance_cap": True,
            "materialization_charged_to_method_budget": False,
        },
        "asset_integrity": integrity,
        "atlas_evaluation": {
            "geometry_materializations": len(rows),
            "materialization_wall_seconds": float(materialization_seconds),
        },
        "cases": rows,
        "aggregate": _aggregate(rows),
        "limitations": [
            "phase 1 evaluates non-blind G1 fixed-translation connectivity; "
            "it is not a free-final-orientation path-query benchmark",
            "the wall threshold is reported after a case and is not an "
            "interrupting real-time cap",
            "an UNKNOWN or zero bracket recall is retained as such and never "
            "promoted into a failed or successful connectivity claim",
            "the blind split remains forbidden without explicit post-freeze "
            "opt-in",
        ],
    }
    report["gate"] = _stage_gate(report, spec.get("expected"))
    report["gate"].update({
        "formal_profile": profile_contract["profile"],
        "formal_profile_validated": profile_contract["validated"],
        "scope": profile_contract["mode"],
    })
    return report


__all__ = [
    "evaluate_gate_interval_report",
    "evaluate_gate_interval_sets",
    "run_atlas_gate_interval_spec",
    "run_gate_interval_case",
]
