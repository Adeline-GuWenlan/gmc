"""Executable SplatC-Atlas paired event benchmark runner."""
from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import time

import numpy as np
import yaml

from .atlas import AtlasAssetStore
from .events import run_event_arm
from .metrics import aggregate_arm, evaluate_event_brackets


ARMS = ("gmc_pair_resolved", "strong_adaptive_scalar")


def _resolve(base: Path, value: str | Path) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (base / path).resolve()


def _budget_caps(budget: dict) -> tuple[int, ...]:
    """Return a strictly increasing cap schedule with legacy single-cap support."""
    raw = budget.get("caps_per_arm_per_case")
    if raw is None:
        caps = (int(budget.get("cap_per_arm_per_case", 2048)),)
    else:
        if isinstance(raw, (str, bytes)):
            raise ValueError("support-eval caps must be an ordered integer list")
        caps = tuple(int(value) for value in raw)
        if not caps:
            raise ValueError("support-eval cap schedule must not be empty")
        if "cap_per_arm_per_case" in budget:
            legacy = int(budget["cap_per_arm_per_case"])
            if legacy != caps[-1]:
                raise ValueError(
                    "cap_per_arm_per_case must equal the final scaling cap"
                )
    if any(cap < 2 for cap in caps):
        raise ValueError("support-eval caps must be at least two")
    if any(right <= left for left, right in zip(caps, caps[1:])):
        raise ValueError("support-eval caps must be strictly increasing")
    return caps


def _arm_payload(instance, truth, arm: str, *, cap: int,
                 tolerance: float, initial_intervals: int) -> dict:
    t0 = time.perf_counter()
    result, ledger = run_event_arm(
        instance,
        arm,
        support_eval_cap=cap,
        tolerance=tolerance,
        initial_intervals=initial_intervals,
    )
    wall = time.perf_counter() - t0
    brackets = list(result.brackets)
    return {
        "search": {
            "brackets_deg": [
                [float(np.rad2deg(lo)), float(np.rad2deg(hi))]
                for lo, hi in brackets
            ],
            "budget_exhausted": result.budget_exhausted,
            "evaluated_angles": result.evaluated_angles,
            "unresolved_intervals": result.unresolved_intervals,
        },
        "metrics": evaluate_event_brackets(truth, brackets),
        "ledger": ledger,
        "wall_seconds": float(wall),
    }


def run_benchmark_spec(spec: dict, *, spec_dir: str | Path = ".",
                       allow_blind: bool = False) -> dict:
    if spec.get("kind") == "p5_system_scaling":
        from .system import run_p5_system_spec
        return run_p5_system_spec(spec, spec_dir=spec_dir)
    if spec.get("kind") == "atlas_g1_full_gate_intervals":
        from .gate_intervals import run_atlas_gate_interval_spec
        return run_atlas_gate_interval_spec(
            spec, spec_dir=spec_dir, allow_blind=allow_blind,
        )
    if spec.get("kind") != "atlas_g1_paired_event":
        raise ValueError(
            "benchmark kind must be atlas_g1_paired_event, "
            "atlas_g1_full_gate_intervals, or p5_system_scaling"
        )
    base = Path(spec_dir).resolve()
    t_assets = time.perf_counter()
    store = AtlasAssetStore(_resolve(base, spec["atlas_root"]))
    splits = tuple(str(s) for s in spec.get("splits", ["dev"]))
    case_ids_raw = spec.get("case_ids")
    case_ids = set(map(str, case_ids_raw)) if case_ids_raw else None
    cases, integrity = store.load_cases(
        splits,
        case_ids=case_ids,
        families=set(map(str, spec.get("families", ["G1"]))),
        robot_ids=set(map(str, spec.get("robot_ids", ["R_long_ellipse"]))),
        partial_gate_only=bool(spec.get("partial_gate_only", True)),
        allow_blind=allow_blind,
    )
    asset_load_seconds = time.perf_counter() - t_assets

    budget = spec.get("budget", {})
    currency = str(budget.get("currency", "support_value_evals"))
    if currency != "support_value_evals":
        raise ValueError("paired event v0 supports only support_value_evals")
    caps = _budget_caps(budget)
    max_cap = caps[-1]
    search = spec.get("search", {})
    tolerance = float(np.deg2rad(search.get("event_tolerance_deg", 0.25)))
    initial_intervals = int(search.get("initial_intervals", 8))

    rows_by_cap: dict[int, list[dict]] = {cap: [] for cap in caps}
    atlas_eval_seconds = 0.0
    for case in cases:
        t_eval = time.perf_counter()
        instance = store.materialize(case)
        truth = case.truth_events_rad()
        atlas_eval_seconds += time.perf_counter() - t_eval
        case_base = {
            "query_id": case.query_id,
            "split": case.split,
            "family": case.family,
            "robot_id": case.robot_id,
            "scene": dict(case.episode["scene"]),
            "oracle_record": {
                "convergence_status": case.convergence_status,
                "fine_reachable_per_goal": (
                    list(case.fine_reachable_per_goal)
                    if case.fine_reachable_per_goal is not None else None
                ),
                "measured_gate_half_angle_deg":
                    case.measured_gate_half_angle_deg,
            },
            "atlas_truth": {
                "source": (
                    "corridor-gate formula recomputed from a split manifest "
                    "bound by pinned split_seals.json and robot axes from the "
                    "pinned G1 registry; OracleRecord analytic fields are "
                    "package-inventory checked but not independently pinned "
                    "and serve only as a cross-check"
                ),
                "gate_center_deg": case.gate_center_deg,
                "gate_half_angle_deg": case.gate_half_angle_deg,
                "event_angles_deg": [float(np.rad2deg(v)) for v in truth],
            },
        }
        for cap in caps:
            row = {**case_base, "arms": {}}
            for arm in ARMS:
                row["arms"][arm] = _arm_payload(
                    instance,
                    truth,
                    arm,
                    cap=cap,
                    tolerance=tolerance,
                    initial_intervals=initial_intervals,
                )
            rows_by_cap[cap].append(row)

    scaling_curve = [
        {
            "cap_per_arm_per_case": cap,
            "aggregate": {
                arm: aggregate_arm(rows_by_cap[cap], arm) for arm in ARMS
            },
        }
        for cap in caps
    ]
    rows = rows_by_cap[max_cap]
    aggregate = scaling_curve[-1]["aggregate"]
    integrity_rows = list(integrity.values())
    source_integrity = integrity_rows[0]["runtime_source_integrity"]
    selected_record_checks = [
        record
        for split_integrity in integrity_rows
        for record in split_integrity["selected_oracle_records"]
    ]
    package_manifests = sorted({
        (
            split_integrity["package_manifest"]["path"],
            split_integrity["package_manifest"]["sha256"],
            split_integrity["package_manifest"]["externally_authenticated"],
        )
        for split_integrity in integrity_rows
    })
    runtime_sources_verified = all(
        module["package_inventory_checked"]
        and module["independently_pinned"]
        for module in source_integrity["modules"].values()
    )
    generator_and_registry_pinned = bool(
        source_integrity["generator_source_sealed"]
        and source_integrity["robot_registry_source_sealed"]
    )
    return {
        "schema_version": 1,
        "name": str(spec.get("name", "atlas-g1-paired-event")),
        "kind": "atlas_g1_paired_event",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "evaluation_contract": {
            "role": "diagnostic_equal_budget_correctness_check",
            "formal_closure_evidence": False,
            "acceptance_gate": None,
            "metric_based_process_failure": False,
            "note": (
                "This paired event-kernel benchmark is diagnostic.  Formal "
                "closure uses the separately gated full-compiler benchmark."
            ),
        },
        "scope": {
            "splits": list(splits),
            "families": list(map(str, spec.get("families", ["G1"]))),
            "robot_ids": list(map(str, spec.get(
                "robot_ids", ["R_long_ellipse"]))),
            "partial_gate_only": bool(spec.get("partial_gate_only", True)),
            "blind_allowed": bool(allow_blind),
            "n_cases": len(rows),
        },
        "input_contract": {
            "manifest_fields": list(store.MANIFEST_FIELDS),
            "oracle_record_fields": list(store.RECORD_FIELDS),
            "truth_priority": [
                (
                    "formula recomputed from fields in a split manifest "
                    "bound by independently pinned split_seals.json, using "
                    "robot axes from the independently pinned G1 registry"
                ),
                (
                    "package-inventory-checked but not independently pinned "
                    "OracleRecord.analytic (cross-check only)"
                ),
                (
                    "package-inventory-checked but not independently pinned "
                    "OracleRecord.measured_gate_half_angle_deg "
                    "(secondary only)"
                ),
            ],
            "package_inventory_role": (
                "runtime checksum inventory with its own reported digest; "
                "not an externally authenticated signature"
            ),
        },
        "adapter_contract": {
            "scene_disc_mapping": "covariance=radius^2*I, level=1",
            "robot_ellipse_mapping": "covariance=diag(a^2,b^2), level=1",
            "workspace_mapping": "exact Atlas axis-aligned workspace box",
            "oracle_record_analytic_cross_check": True,
            "method_code_imported_from_atlas": False,
            "generator_source_imported_from_atlas": True,
            "generator_source_sealed": bool(
                source_integrity["generator_source_sealed"]
            ),
            "generator_runtime_path_and_hash_verified":
                runtime_sources_verified,
            "robot_registry_source": source_integrity[
                "robot_registry_source"
            ],
            "robot_registry_source_sealed": bool(
                source_integrity["robot_registry_source_sealed"]
            ),
            "legacy_method_import_guard": source_integrity[
                "legacy_import_guard"
            ],
        },
        "asset_integrity": integrity,
        "asset_integrity_scope": {
            "split_seals": {
                "binding": "package inventory plus independent pinned digest",
                "all_independently_pinned": all(
                    row["split_seals"]["independently_pinned"]
                    for row in integrity_rows
                ),
            },
            "split_manifests": {
                "binding": "digest recorded in pinned split_seals.json",
                "package_inventory_checked_splits": sorted(
                    split_name for split_name, row in integrity.items()
                    if row["package_inventory_checked"]
                ),
                "pinned_split_seal_only_splits": sorted(
                    split_name for split_name, row in integrity.items()
                    if not row["package_inventory_checked"]
                ),
            },
            "runtime_atlas_sources": {
                "generator_and_robot_registry_independently_pinned":
                    generator_and_registry_pinned,
                "actual_import_paths_and_hashes_verified":
                    runtime_sources_verified,
                "modules": source_integrity["modules"],
            },
            "oracle_records": {
                "role": "secondary cross-check and reporting only",
                "selected_count": len(selected_record_checks),
                "all_selected_package_inventory_checked": bool(
                    selected_record_checks
                ) and all(
                    record["package_inventory_checked"]
                    for record in selected_record_checks
                ),
                "independently_pinned": bool(selected_record_checks) and all(
                    record["independently_pinned"]
                    for record in selected_record_checks
                ),
            },
            "package_manifests": [
                {
                    "path": path,
                    "sha256": digest,
                    "externally_authenticated": authenticated,
                }
                for path, digest, authenticated in package_manifests
            ],
        },
        "budget_contract": {
            "currency": currency,
            # Retained for consumers of the original single-cap schema.  Case
            # details and the top-level aggregate always describe this max cap.
            "cap_per_arm_per_case": max_cap,
            "caps_per_arm_per_case": list(caps),
            "case_detail_cap_per_arm_per_case": max_cap,
            "same_cap_for_both_arms": True,
            "atlas_evaluation_charged_to_method_budget": False,
        },
        "atlas_evaluation": {
            "asset_load_and_seal_check_wall_seconds": float(asset_load_seconds),
            "analytic_truth_lookups": len(rows),
            "geometry_materializations": len(rows),
            "wall_seconds": float(atlas_eval_seconds),
            "total_wall_seconds": float(asset_load_seconds + atlas_eval_seconds),
            "accounting": "separate external-evaluation overhead",
        },
        "search_contract": {
            "period_deg": 180.0,
            "event_tolerance_deg": float(np.rad2deg(tolerance)),
            "terminal_interval_width_bound_deg": float(
                np.rad2deg(tolerance)
            ),
            "merged_brackets_can_exceed_terminal_tolerance": True,
            "initial_intervals": initial_intervals,
            "root_free_certificate": (
                "scalar: abs(gap(mid)) > L*r; pair-min: all lower bounds "
                "> 0 or any upper bound < 0"
            ),
            "continuous_zero_regions": "unresolved",
        },
        "comparison_contract": {
            "aggregate_event": "zero boundary of min(pair_gaps)",
            "same_underlying_pair_support_queries": True,
            "symmetric_g1_expected_to_tie": True,
            "discriminative_method_advantage_claimed": False,
        },
        "cases": rows,
        "aggregate": aggregate,
        "scaling_curve": scaling_curve,
        "limitations": [
            "v0 covers frozen G1 corridor single-door partial-angle cases only",
            "the paired GMC arm is the pair-resolved gate-support event kernel, "
            "not the full mobility compiler",
            "the strong scalar arm sees the exact min of the same pair gaps",
            "on the current symmetric G1 cases both arms query the same pair "
            "gaps and are expected to tie; this is a correctness/equal-budget "
            "check, not evidence of a method advantage",
            "this diagnostic kind has no acceptance gate and is not formal "
            "closure evidence",
            "each scaling point is an independent cold event-search run; case "
            "details are retained only at the maximum cap",
            "dense reachability, K2, non-gate split/merge events, P4 and blind "
            "evaluation are outside this runner",
        ],
    }


def run_benchmark_file(spec_path: str | Path, *, output: str | Path | None = None,
                       allow_blind: bool = False) -> dict:
    path = Path(spec_path).resolve()
    spec = yaml.safe_load(path.read_text())
    report = run_benchmark_spec(
        spec, spec_dir=path.parent, allow_blind=allow_blind
    )
    if output is not None:
        out = Path(output).resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        tmp = out.with_suffix(out.suffix + ".tmp")
        tmp.write_text(json.dumps(report, indent=2, sort_keys=True))
        tmp.replace(out)
    return report
