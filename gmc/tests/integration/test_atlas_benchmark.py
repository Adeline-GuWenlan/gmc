from pathlib import Path
from copy import deepcopy
import hashlib
import sys
from types import ModuleType

import pytest

from gmc.benchmarking.atlas import (AtlasAssetError, AtlasAssetStore,
                                    BlindSplitForbidden)
from gmc.benchmarking.runner import run_benchmark_spec


WORKSPACE = Path(__file__).resolve().parents[3]
ATLAS = WORKSPACE / "splatc_atlas"


def test_blind_is_forbidden_without_explicit_opt_in():
    store = AtlasAssetStore(ATLAS)
    with pytest.raises(BlindSplitForbidden):
        store.load_cases(["blind"])


def test_blind_opt_in_keeps_withheld_manifest_scope_explicit():
    store = AtlasAssetStore(ATLAS)
    cases, integrity = store.load_cases(
        ["blind"], case_ids={"blind_000"}, partial_gate_only=False,
        allow_blind=True,
    )
    assert [case.query_id for case in cases] == ["blind_000"]
    assert integrity["blind"]["sealed_artifact"] == "pinned_split_seal"
    assert not integrity["blind"]["package_inventory_checked"]
    assert integrity["blind"]["selected_oracle_records"] == []
    assert not integrity["blind"][
        "oracle_records_package_inventory_checked"
    ]


def test_oracle_record_analytic_truth_is_cross_checked_against_manifest():
    store = AtlasAssetStore(ATLAS)
    cases, _ = store.load_cases(["dev"], case_ids={"dev_001"},
                                partial_gate_only=False)
    analytic = deepcopy(cases[0].record["analytic"])
    analytic["gate_half_angle_deg"] += 0.1
    with pytest.raises(AtlasAssetError, match="gate_half_angle_deg mismatch"):
        store._validate_record_analytic(cases[0].episode, analytic)


def test_one_sealed_atlas_case_runs_both_arms_at_equal_cap():
    spec = {
        "name": "atlas-integration-smoke",
        "kind": "atlas_g1_paired_event",
        "atlas_root": str(ATLAS),
        "splits": ["dev"],
        "families": ["G1"],
        "robot_ids": ["R_long_ellipse"],
        "partial_gate_only": True,
        "case_ids": ["dev_001"],
        "budget": {
            "currency": "support_value_evals",
            "cap_per_arm_per_case": 1024,
        },
        "search": {"event_tolerance_deg": 0.5, "initial_intervals": 8},
    }
    report = run_benchmark_spec(spec)
    assert report["asset_integrity"]["dev"]["sha256"] == (
        "a3e5f12b3a2166b7d57dc8b46bb3db6dadc0c7cdbdbf39b5cae013853101ab31"
    )
    integrity = report["asset_integrity"]["dev"]
    assert integrity["sealed_artifact"] == (
        "pinned_split_seal+round4_package_inventory"
    )
    assert not integrity["oracle_records_sealed"]
    assert integrity["oracle_records_package_inventory_checked"]
    assert not integrity["package_manifest"]["externally_authenticated"]
    assert len(integrity["package_manifest"]["sha256"]) == 64
    assert integrity["split_seals"]["independently_pinned"]
    assert integrity["generator_source_sealed"]
    assert integrity["robot_registry_source_sealed"]
    source = integrity["runtime_source_integrity"]
    assert source["generator_source_sealed"]
    assert source["robot_registry_source_sealed"]
    g1_source = source["modules"]["splatc.datasets.g1_gate"]
    assert g1_source["sha256"] == (
        "f3dbd542098079c18f4b2fbec53c12cdfcaf82e283fa0c8c26df953f94897424"
    )
    assert Path(g1_source["path"]).resolve() == (
        ATLAS / "outputs" / "round4_final_package" / "src_snapshot"
        / "src" / "splatc" / "datasets" / "g1_gate.py"
    ).resolve()
    evaluation = report["evaluation_contract"]
    assert evaluation["role"] == "diagnostic_equal_budget_correctness_check"
    assert not evaluation["formal_closure_evidence"]
    assert evaluation["acceptance_gate"] is None
    assert not evaluation["metric_based_process_failure"]
    assert "gate" not in report
    adapter = report["adapter_contract"]
    assert adapter["generator_source_sealed"]
    assert adapter["generator_runtime_path_and_hash_verified"]
    assert adapter["robot_registry_source_sealed"]
    assert adapter["robot_registry_source"] == (
        "splatc.datasets.g1_gate:robot_library"
    )
    assert adapter["legacy_method_import_guard"]["forbidden_namespace"] == (
        "splatc.gmc and descendants"
    )
    scope = report["asset_integrity_scope"]
    assert scope["split_seals"]["all_independently_pinned"]
    assert scope["split_manifests"][
        "package_inventory_checked_splits"
    ] == ["dev"]
    assert scope["split_manifests"]["pinned_split_seal_only_splits"] == []
    assert scope["runtime_atlas_sources"][
        "generator_and_robot_registry_independently_pinned"
    ]
    assert scope["runtime_atlas_sources"][
        "actual_import_paths_and_hashes_verified"
    ]
    assert scope["oracle_records"] == {
        "role": "secondary cross-check and reporting only",
        "selected_count": 1,
        "all_selected_package_inventory_checked": True,
        "independently_pinned": False,
    }
    assert len(scope["package_manifests"]) == 1
    assert not scope["package_manifests"][0]["externally_authenticated"]
    assert "pinned G1 registry" in report["cases"][0]["atlas_truth"][
        "source"
    ]
    assert "not independently pinned" in report["input_contract"][
        "truth_priority"
    ][1]
    assert not report["comparison_contract"][
        "discriminative_method_advantage_claimed"
    ]
    assert "merged_bracket_width_bound_deg" not in report["search_contract"]
    assert report["atlas_evaluation"]["analytic_truth_lookups"] == 1
    assert not report["budget_contract"][
        "atlas_evaluation_charged_to_method_budget"
    ]
    assert report["budget_contract"]["caps_per_arm_per_case"] == [1024]
    assert len(report["scaling_curve"]) == 1
    assert report["scaling_curve"][0]["aggregate"] == report["aggregate"]
    used = []
    for arm in ("gmc_pair_resolved", "strong_adaptive_scalar"):
        result = report["cases"][0]["arms"][arm]
        assert result["metrics"]["event_recall"] == 1.0
        assert not result["search"]["budget_exhausted"]
        assert result["ledger"]["limits"]["support_value_evals"] == 1024
        assert result["ledger"]["totals"]["support_value_evals"] <= 1024
        used.append(result["ledger"]["totals"]["support_value_evals"])
    assert used[0] == used[1]


def test_scaling_curve_uses_ordered_equal_caps_and_keeps_max_cap_details():
    caps = [64, 128, 256]
    spec = {
        "name": "atlas-scaling-smoke",
        "kind": "atlas_g1_paired_event",
        "atlas_root": str(ATLAS),
        "splits": ["dev"],
        "families": ["G1"],
        "robot_ids": ["R_long_ellipse"],
        "partial_gate_only": True,
        "case_ids": ["dev_001"],
        "budget": {
            "currency": "support_value_evals",
            "caps_per_arm_per_case": caps,
        },
        "search": {"event_tolerance_deg": 0.5, "initial_intervals": 8},
    }
    report = run_benchmark_spec(spec)
    assert [point["cap_per_arm_per_case"]
            for point in report["scaling_curve"]] == caps
    assert report["budget_contract"]["cap_per_arm_per_case"] == caps[-1]
    assert report["budget_contract"][
        "case_detail_cap_per_arm_per_case"
    ] == caps[-1]
    assert report["aggregate"] == report["scaling_curve"][-1]["aggregate"]

    for point in report["scaling_curve"]:
        cap = point["cap_per_arm_per_case"]
        for arm in ("gmc_pair_resolved", "strong_adaptive_scalar"):
            assert point["aggregate"][arm]["support_value_evals"] <= cap
    for arm in ("gmc_pair_resolved", "strong_adaptive_scalar"):
        assert report["cases"][0]["arms"][arm]["ledger"]["limits"][
            "support_value_evals"
        ] == caps[-1]
        aggregate = report["aggregate"][arm]
        assert aggregate["unknown_cases"] == 0
        assert aggregate["certified_complete_cases"] == 1
        assert aggregate["max_bracket_width_deg"] is not None
        assert aggregate["max_midpoint_error_deg"] is not None


def test_split_manifest_is_hashed_and_parsed_from_one_read(monkeypatch):
    store = AtlasAssetStore(ATLAS)
    target = store.manifest_path("dev").resolve()
    original_read_bytes = Path.read_bytes
    original = original_read_bytes(target)
    calls = 0

    def flipping_read_bytes(path):
        nonlocal calls
        if path.resolve() == target:
            calls += 1
            # A second open observes a different valid document.  A correct
            # loader never asks for it because it reuses the sealed payload.
            return original if calls == 1 else b'{"episodes": []}'
        return original_read_bytes(path)

    monkeypatch.setattr(Path, "read_bytes", flipping_read_bytes)
    cases, _ = store.load_cases(
        ["dev"], case_ids={"dev_001"}, partial_gate_only=False,
    )
    assert [case.query_id for case in cases] == ["dev_001"]
    assert calls == 1


def test_split_manifest_tamper_is_rejected(monkeypatch):
    store = AtlasAssetStore(ATLAS)
    target = store.manifest_path("dev").resolve()
    original_read_bytes = Path.read_bytes

    def tampered_read_bytes(path):
        data = original_read_bytes(path)
        return data + b" " if path.resolve() == target else data

    monkeypatch.setattr(Path, "read_bytes", tampered_read_bytes)
    with pytest.raises(AtlasAssetError, match="manifest seal mismatch"):
        store.load_cases(
            ["dev"], case_ids={"dev_001"}, partial_gate_only=False,
        )


def test_package_inventory_cannot_reseal_pinned_generator(monkeypatch):
    package = ATLAS / "outputs" / "round4_final_package"
    inventory = (package / "MANIFEST.sha256").resolve()
    generator = (
        package / "src_snapshot" / "src" / "splatc" / "datasets"
        / "g1_gate.py"
    ).resolve()
    original_read_bytes = Path.read_bytes
    original_generator = original_read_bytes(generator)
    tampered_generator = original_generator + b"\n# forged source\n"
    original_digest = hashlib.sha256(original_generator).hexdigest().encode()
    forged_digest = hashlib.sha256(tampered_generator).hexdigest().encode()

    def forged_read_bytes(path):
        resolved = path.resolve()
        data = original_read_bytes(path)
        if resolved == inventory:
            return data.replace(original_digest, forged_digest, 1)
        if resolved == generator:
            return tampered_generator
        return data

    monkeypatch.setattr(Path, "read_bytes", forged_read_bytes)
    store = AtlasAssetStore(ATLAS)
    with pytest.raises(AtlasAssetError, match="pinned checksum mismatch"):
        store.load_cases(
            ["dev"], case_ids={"dev_001"}, partial_gate_only=False,
        )


def test_selected_oracle_record_is_checked_against_package_inventory(
        monkeypatch):
    store = AtlasAssetStore(ATLAS)
    target = (store.records_root / "dev_001.json").resolve()
    original_read_bytes = Path.read_bytes

    def tampered_read_bytes(path):
        data = original_read_bytes(path)
        return data + b" " if path.resolve() == target else data

    monkeypatch.setattr(Path, "read_bytes", tampered_read_bytes)
    with pytest.raises(AtlasAssetError, match="package checksum mismatch"):
        store.load_cases(
            ["dev"], case_ids={"dev_001"}, partial_gate_only=False,
        )


def test_runtime_guard_rejects_new_legacy_import_only(monkeypatch):
    store = AtlasAssetStore(ATLAS)
    cases, _ = store.load_cases(
        ["dev"], case_ids={"dev_001"}, partial_gate_only=False,
    )
    module = store._load_atlas_generator()
    original_make_scene = module.make_g1_scene
    unrelated_name = "splatc.gmcustom.preexisting"
    forbidden_name = "splatc.gmc.legacy_probe"
    monkeypatch.setitem(sys.modules, unrelated_name, ModuleType(unrelated_name))
    monkeypatch.delitem(sys.modules, forbidden_name, raising=False)

    def poisoned_make_scene(*args, **kwargs):
        sys.modules[forbidden_name] = ModuleType(forbidden_name)
        return original_make_scene(*args, **kwargs)

    monkeypatch.setattr(module, "make_g1_scene", poisoned_make_scene)
    with pytest.raises(AtlasAssetError, match="legacy import guard"):
        store.materialize(cases[0])
    assert unrelated_name in sys.modules
    assert forbidden_name not in sys.modules
