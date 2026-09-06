from pathlib import Path

import yaml

from gmc.benchmarking.runner import run_benchmark_file


GMC_ROOT = Path(__file__).resolve().parents[2]


def test_p5_system_curves_are_conservative_and_incremental_work_is_bounded(
        tmp_path):
    spec = {
        "name": "p5-system-test",
        "kind": "p5_system_scaling",
        "profile_top_n": 5,
        "bvh": {
            "seed": 20260901,
            "support_counts": [32, 96],
            "leaf_size": 4,
            "workspace": [-2.0, -2.0, 2.0, 2.0],
            "near_fraction": 0.125,
            "robot_axes": [0.22, 0.12],
        },
        "incremental": {
            "support_counts": [8, 16],
            "theta": 0.31,
            "workspace": [-3.0, -3.0, 3.0, 3.0],
            "layout_radius": 1.4,
            "scene_radius": 0.055,
            "robot_axes": [0.18, 0.09],
            "edit_delta": [0.025, -0.015],
            "semantic_area_tolerance": 1e-12,
            "gmc_config": str(GMC_ROOT / "configs" / "default.yaml"),
            "pair_approx": {
                "initial_directions": 16,
                "max_directions": 64,
                "eps_pair": 1e-2,
                "certificate_mode": "theorem",
            },
        },
    }
    spec_path = tmp_path / "p5.yaml"
    spec_path.write_text(yaml.safe_dump(spec))
    report = run_benchmark_file(spec_path)

    assert report["kind"] == "p5_system_scaling"
    assert report["reproducibility"]["atlas_accessed"] is False
    assert report["aggregate"]["bvh"]["all_candidate_sets_match_flat"]
    assert report["aggregate"]["incremental"][
        "all_geometry_semantically_equal"
    ]
    assert [row["scene_supports"] for row in report["bvh_curve"]] == [32, 96]
    for row in report["bvh_curve"]:
        assert row["pair_tests"] + row["pruned_pairs"] == row["total_pairs"]
        assert row["candidate_pairs"] <= row["pair_tests"]
        assert row["validation"]["candidate_set_matches_flat"]
        assert row["validation"]["pair_accounting_conserved"]
        assert row["peak_memory_bytes"] >= 0
        assert row["build_wall_seconds"] >= 0.0
        assert row["query_wall_seconds"] >= 0.0

    assert [row["scene_supports"]
            for row in report["incremental_curve"]] == [8, 16]
    for row in report["incremental_curve"]:
        assert row["changed_pair_count"] == 1
        assert row["geometry_semantic_diff"]["semantic_equal"]
        assert row["work_relation"]["incremental_support_calls_le_full"]
        assert row["work_relation"]["incremental_union_operations_le_full"]
        assert (row["incremental_edit"]["envelope_support_calls"]
                < row["full_edit"]["envelope_support_calls"])
        assert (row["incremental_edit"]["union_operations"]
                < row["full_edit"]["union_operations"])
        for phase in ("cold_initial", "full_edit", "incremental_edit"):
            assert row[phase]["wall_seconds"] >= 0.0

    for profile in report["profiles"].values():
        assert 1 <= len(profile["top_functions"]) <= 5
        assert all("cumulative_seconds" in row
                   for row in profile["top_functions"])
