import json

from gs3d_benchmark import run


def test_three_robot_benchmark_has_cold_warm_and_physical_dt(tmp_path):
    payload = run(warm_calls=5)
    target = tmp_path / "benchmark.json"
    target.write_text(json.dumps(payload, allow_nan=False))
    loaded = json.loads(target.read_text())
    assert loaded["schema_version"] == "gs3d.benchmark.v1"
    assert [row["robot"] for row in loaded["robots"]] == ["uav", "sweeper", "cylinder"]
    for row in loaded["robots"]:
        assert row["cold"]["status"] == "success"
        assert row["cold"]["timing"]["mode"] == "cold"
        assert row["cold"]["timing"]["preparation_wall_s"] >= 0
        assert row["warm"]["calls"] == 5
        assert len(row["warm"]["algorithm_wall_samples_s"]) == 5
        assert row["warm"]["latency"]["count"] == 5
        assert row["physical_control"]["control_dt_s"] == .05
        assert row["physical_control"]["trajectory_duration_s"] >= 0
