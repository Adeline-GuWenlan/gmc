import json
from types import SimpleNamespace

import numpy as np

from gmc.cli import cmd_benchmark, main
from gmc.benchmarking.metrics import (circular_distance,
                                      evaluate_event_brackets)
from gmc.benchmarking.events import adaptive_isolate_events
from gmc.benchmarking.runner import _budget_caps, run_benchmark_file


def test_circular_distance_wraps_at_period():
    assert np.isclose(circular_distance(0.01, np.pi - 0.01, np.pi), 0.02)


def test_event_metrics_count_hits_spurious_and_widths():
    truth = [0.2, 1.2]
    brackets = [(0.19, 0.21), (1.19, 1.21), (2.0, 2.1)]
    metrics = evaluate_event_brackets(truth, brackets)
    assert metrics["event_recall"] == 1.0
    assert metrics["matched_truth_count"] == 2
    assert metrics["spurious_bracket_count"] == 1
    assert metrics["max_bracket_width_deg"] > metrics["mean_midpoint_error_deg"]


def test_empty_prediction_has_zero_recall_for_nonempty_truth():
    metrics = evaluate_event_brackets([0.5], [])
    assert metrics["event_recall"] == 0.0
    assert metrics["predicted_bracket_count"] == 0


def test_budget_caps_support_legacy_and_reject_unordered_schedules():
    assert _budget_caps({"cap_per_arm_per_case": 128}) == (128,)
    assert _budget_caps({"caps_per_arm_per_case": [64, 128, 256]}) == (
        64, 128, 256
    )
    with np.testing.assert_raises(ValueError):
        _budget_caps({"caps_per_arm_per_case": [128, 64]})


def test_periodic_boundary_bracket_matches_zero_without_spurious_count():
    metrics = evaluate_event_brackets(
        [0.0], [(0.0, 0.01), (np.pi - 0.01, np.pi)], period=np.pi
    )
    assert metrics["event_recall"] == 1.0
    assert metrics["spurious_bracket_count"] == 0
    assert metrics["predicted_bracket_count"] == 1
    assert np.isclose(metrics["max_bracket_width_deg"], np.rad2deg(0.02))


def test_search_merges_first_and_last_brackets_for_boundary_root():
    result = adaptive_isolate_events(
        lambda theta: np.asarray([np.sin(2.0 * theta)]),
        lipschitz=2.0,
        tolerance=0.02,
        initial_intervals=4,
        period=np.pi,
    )
    boundary = [bracket for bracket in result.brackets
                if bracket[0] < np.pi < bracket[1]]
    assert len(boundary) == 1
    metrics = evaluate_event_brackets([0.0, np.pi / 2.0], result.brackets)
    assert metrics["event_recall"] == 1.0
    assert metrics["spurious_bracket_count"] == 0


def test_pair_min_ignores_root_of_pair_dominated_by_negative_pair():
    result = adaptive_isolate_events(
        lambda theta: np.asarray([np.sin(2.0 * theta), -2.0]),
        lipschitz=np.asarray([2.0, 1.0]),
        tolerance=0.01,
        initial_intervals=8,
        period=np.pi,
        aggregation="minimum",
    )
    assert result.brackets == ()
    assert result.unresolved_intervals == 0
    assert not result.budget_exhausted


def test_continuous_zero_region_is_unresolved_not_complete_event():
    result = adaptive_isolate_events(
        lambda _theta: np.asarray([0.0]),
        lipschitz=1.0,
        tolerance=0.1,
        initial_intervals=4,
        period=np.pi,
    )
    assert result.brackets == ()
    assert result.unresolved_intervals > 0
    assert not result.budget_exhausted


def test_event_search_rejects_invalid_bounds_and_outputs():
    with np.testing.assert_raises(ValueError):
        adaptive_isolate_events(
            lambda _theta: np.asarray([1.0]),
            lipschitz=-1.0,
            tolerance=0.1,
            initial_intervals=4,
        )
    with np.testing.assert_raises(ValueError):
        adaptive_isolate_events(
            lambda _theta: np.asarray([np.nan]),
            lipschitz=1.0,
            tolerance=0.1,
            initial_intervals=4,
        )


def test_one_bracket_cannot_match_two_truth_events():
    metrics = evaluate_event_brackets(
        [0.2, 0.8], [(0.1, 0.9)], period=np.pi
    )
    assert metrics["truth_count"] == 2
    assert metrics["predicted_bracket_count"] == 1
    assert metrics["matched_truth_count"] == 1
    assert metrics["event_recall"] == 0.5
    assert metrics["spurious_bracket_count"] == 0


def test_cli_prints_staged_aggregate_without_post_write_failure(
        monkeypatch, tmp_path, capsys):
    spec_path = tmp_path / "staged.yaml"
    spec_path.write_text(
        "kind: atlas_g1_full_gate_intervals\n"
        "stages:\n"
        "  - stage: initial_42_soundness\n"
        "  - stage: refined_16_acceptance\n"
    )
    passing_gate = {"declared": True, "pass": True, "errors": []}
    staged = {
        "kind": "atlas_g1_full_gate_intervals_staged",
        "aggregate_by_stage": {
            "initial_42_soundness": {"cases": 42},
            "refined_16_acceptance": {"cases": 16},
        },
        "profile_contract": {
            "mode": "diagnostic_only", "profile": None,
            "validated": False,
        },
        "stage_order": [
            "initial_42_soundness", "refined_16_acceptance",
        ],
        "stages": [
            {"stage": "initial_42_soundness", "gate": passing_gate},
            {"stage": "refined_16_acceptance", "gate": passing_gate},
        ],
        "gate": passing_gate,
    }

    def fake_runner(spec, *, output, allow_blind):
        assert spec == str(spec_path)
        assert output == str(tmp_path / "report.json")
        assert allow_blind is False
        return staged

    monkeypatch.setattr(
        "gmc.benchmarking.runner.run_benchmark_file", fake_runner,
    )
    status = cmd_benchmark(SimpleNamespace(
        spec=str(spec_path), output=str(tmp_path / "report.json"),
        allow_blind=False,
    ))
    assert status == 0
    stdout = capsys.readouterr().out
    assert "initial_42_soundness" in stdout
    assert "refined_16_acceptance" in stdout


def test_cli_returns_nonzero_for_written_failed_gate(monkeypatch, tmp_path):
    spec_path = tmp_path / "failed.yaml"
    spec_path.write_text("kind: atlas_g1_full_gate_intervals\n")
    failed = {
        "kind": "atlas_g1_full_gate_intervals",
        "aggregate": {"n_cases": 1},
        "profile_contract": {
            "mode": "diagnostic_only", "profile": None,
            "validated": False,
        },
        "gate": {
            "declared": True, "pass": False,
            "errors": ["declared_gate_failed"],
        },
    }

    def fake_runner(_spec, *, output, allow_blind):
        assert output == str(tmp_path / "failed.json")
        assert allow_blind is False
        return failed

    monkeypatch.setattr(
        "gmc.benchmarking.runner.run_benchmark_file", fake_runner,
    )
    status = cmd_benchmark(SimpleNamespace(
        spec=str(spec_path), output=str(tmp_path / "failed.json"),
        allow_blind=False,
    ))
    assert status == 1


def test_full_gate_cli_fails_closed_on_missing_or_malformed_gate(
        monkeypatch, tmp_path, capsys):
    spec_path = tmp_path / "full.yaml"
    spec_path.write_text("kind: atlas_g1_full_gate_intervals\n")
    base = {
        "kind": "atlas_g1_full_gate_intervals",
        "aggregate": {},
        "profile_contract": {
            "mode": "diagnostic_only", "profile": None,
            "validated": False,
        },
    }
    attacks = [
        {},
        {"gate": "not-a-mapping"},
        {"gate": {"pass": True, "errors": []}},
        {"gate": {"declared": True, "errors": []}},
        {"gate": {"declared": True, "pass": True}},
        {"gate": {"declared": False, "pass": True, "errors": []}},
        {"gate": {"declared": True, "pass": "false", "errors": []}},
        {"gate": {"declared": True, "pass": True, "errors": ["failed"]}},
        {"kind": "atlas_g1_paired_event", "gate": {
            "declared": True, "pass": True, "errors": [],
        }},
    ]

    for attack in attacks:
        report = {**base, **attack}
        monkeypatch.setattr(
            "gmc.benchmarking.runner.run_benchmark_file",
            lambda *_args, _report=report, **_kwargs: _report,
        )
        status = cmd_benchmark(SimpleNamespace(
            spec=str(spec_path), output=None, allow_blind=False,
        ))
        assert status == 1
    capsys.readouterr()


def test_main_propagates_benchmark_return_code(monkeypatch):
    monkeypatch.setattr("gmc.cli.cmd_benchmark", lambda _args: 7)
    assert main(["benchmark", "unused.yaml"]) == 7


def test_runner_atomically_publishes_failed_gate_report(
        monkeypatch, tmp_path):
    spec_path = tmp_path / "spec.yaml"
    output_path = tmp_path / "report.json"
    spec_path.write_text("kind: synthetic\n")
    failed = {
        "aggregate": {"n_cases": 0},
        "gate": {"pass": False, "errors": ["expected_n_cases_mismatch"]},
    }
    monkeypatch.setattr(
        "gmc.benchmarking.runner.run_benchmark_spec",
        lambda _spec, *, spec_dir, allow_blind: failed,
    )

    result = run_benchmark_file(spec_path, output=output_path)

    assert result is failed
    assert json.loads(output_path.read_text()) == failed
    assert not output_path.with_suffix(".json.tmp").exists()
