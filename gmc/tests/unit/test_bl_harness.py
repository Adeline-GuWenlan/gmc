"""bl B1 harness tests (``experiments/bl_harness.py``, ``bl_worker.py``, ``bl_testmethods.py``).

Pure tests run anywhere (no scene). Tests marked ``heavy`` load a persisted F3 compile and run only with
``BL_HEAVY=1`` (an sbatch job, ``hpc/baselines/b1_harness_tests.sbatch``): the compile-loading rule of the chain.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np
import pytest

GMC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(GMC / "experiments"))

import bl_harness as H  # noqa: E402
import bl_worker as W  # noqa: E402

heavy = pytest.mark.skipif(os.environ.get("BL_HEAVY") != "1", reason="loads a persisted compile (BL_HEAVY=1, sbatch)")


@pytest.fixture(autouse=True)
def _cwd_gmc(monkeypatch):
    monkeypatch.chdir(GMC)


def _artifact(tmp_path, method, state=None):
    art = tmp_path / f"{method}.setup.pkl"
    W.write_artifact(str(art), state or {"method": method}, {"setup_id": f"test-{method}"})
    return art


def _pairs(n=2):
    return [{"index": k, "pair_id": f"T-{k}", "start_uv": [0., 0.], "goal_uv": [1., 1.], "dist_m": 1.4}
            for k in range(n)]


# ------------------------------------------------------------------------------------------------ pure
def test_complete_snaps_roundoff_and_appends_real_gaps():
    uv, info = H.complete_path([[1e-12, 0.], [.5, .5], [1., 1. + 1e-12]], [0., 0.], [1., 1.], .885)
    assert np.array_equal(uv, [[0., 0.], [.5, .5], [1., 1.]])
    assert not info["prepended_start_segment"] and not info["appended_goal_segment"]
    uv, info = H.complete_path([[.01, 0., .9], [.9, 1., .885]], [0., 0.], [1., 1.], .885)
    assert np.array_equal(uv, [[0., 0.], [.01, 0.], [.9, 1.], [1., 1.]])
    assert info["prepended_start_segment"] and info["appended_goal_segment"]
    assert info["start_gap_m"] == pytest.approx(.01) and info["max_abs_dz_m"] == pytest.approx(.015)


def test_complete_rejects_nonfinite_and_bad_shapes():
    with pytest.raises(ValueError):
        H.complete_path([[0., np.nan]], [0., 0.], [1., 1.], 0.)
    with pytest.raises(ValueError):
        H.complete_path([1., 2.], [0., 0.], [1., 1.], 0.)


def test_outcome_mapping():
    ok = {"judge_passed": True, "judge_geometry_passed": True}
    occ = {"judge_passed": False, "judge_geometry_passed": False, "judge_geometry_occupancy": "occupied"}
    unk = {"judge_passed": False, "judge_geometry_passed": False, "judge_geometry_occupancy": "unknown"}
    kin = {"judge_passed": False, "judge_geometry_passed": True}
    assert H.outcome(False, ok) == "FAIL"
    assert H.outcome(True, ok) == "SUCCESS"
    assert H.outcome(True, occ) == "CLAIMED_COLLIDES"
    assert H.outcome(True, unk) == "CLAIMED_UNPROVEN"
    assert H.outcome(True, kin) == "CLAIMED_KINEMATICS"


def test_artifact_hash_is_checked(tmp_path):
    art = _artifact(tmp_path, "straight")
    W.read_artifact(str(art))
    art.write_bytes(art.read_bytes() + b"x")
    with pytest.raises(ValueError):
        W.read_artifact(str(art))


def test_sleeping_method_times_out_from_outside_and_next_pair_gets_a_fresh_worker(tmp_path):
    art = _artifact(tmp_path, "sleep")
    out = tmp_path / "t.jsonl"
    s = H.run_task("sleep", "S", "cylinder", _pairs(2), out, artifact=art, config={"sleep_s": 60}, timeout_s=2.,
                   judge_mode="defer")
    rows = [json.loads(x) for x in open(out)]
    assert [r["status"] for r in rows] == ["TIMEOUT", "TIMEOUT"]
    assert all(2. <= r["outer_wall_s"] < 10. for r in rows)
    assert rows[0]["reason"] == "query_exceeded_2s"
    assert [r["worker_generation"] for r in rows] == [0, 1]
    p = s["setup_once_proof"]
    assert p["setup_builds_in_this_task"] == 0 and p["worker_starts"] == 2 and p["restarts"] == 1
    assert p["every_start_loaded_the_same_artifact"] and p["same_setup_id_all_rows"]


def test_raising_method_is_an_error_row_and_the_worker_survives(tmp_path):
    art = _artifact(tmp_path, "raise")
    out = tmp_path / "t.jsonl"
    s = H.run_task("raise", "S", "cylinder", _pairs(2), out, artifact=art, config={}, judge_mode="defer")
    rows = [json.loads(x) for x in open(out)]
    assert [r["status"] for r in rows] == ["ERROR", "ERROR"]
    assert rows[0]["reason"].startswith("RuntimeError: test method raises")
    assert s["setup_once_proof"]["worker_starts"] == 1


def test_crashing_worker_is_an_error_row_and_is_restarted(tmp_path):
    art = _artifact(tmp_path, "crash")
    out = tmp_path / "t.jsonl"
    s = H.run_task("crash", "S", "cylinder", _pairs(2), out, artifact=art, config={}, judge_mode="defer")
    rows = [json.loads(x) for x in open(out)]
    assert [r["status"] for r in rows] == ["ERROR", "ERROR"]
    assert rows[0]["reason"] == "worker_died rc=3"
    assert s["setup_once_proof"]["worker_starts"] == 2


def test_method_prints_cannot_corrupt_the_protocol(tmp_path, capfd):
    art = _artifact(tmp_path, "straight")
    w = H.Worker("straight", art, {}, tmp_path / "w.log")
    w.start(timeout=60)
    os.write(1, b"")       # parent fd untouched
    out, _ = w.query({"op": "query", "pair_id": "x", "start_uv": [0, 0], "goal_uv": [1, 0]}, 30)
    w.close()
    assert out["op"] == "result" and out["claimed"] and out["path_uv"] == [[0, 0], [1, 0]]


def test_resume_skips_answered_pairs(tmp_path):
    art = _artifact(tmp_path, "straight")
    out = tmp_path / "t.jsonl"
    H.run_task("straight", "S", "cylinder", _pairs(1), out, artifact=art, config={}, judge_mode="defer")
    s = H.run_task("straight", "S", "cylinder", _pairs(3), out, artifact=art, config={}, judge_mode="defer")
    assert s["resumed_from_checkpoint"] == 1 and s["answered_this_run"] == 2 and s["complete"]
    assert [json.loads(x)["status"] for x in open(out)] == ["CLAIMED_PENDING_JUDGE"] * 3


# ------------------------------------------------------------------------------------------------ heavy (sbatch)
def _s_pairs(ids):
    by = {p["pair_id"]: p for p in H.load_pairs("f4", "S")}
    return [dict(by[i], index=k) for k, i in enumerate(ids)]


def _setup(tmp_path, method):
    side = json.loads(Path(f"{H.scene_export_path('S', 'cylinder')}.json").read_text())
    return _artifact(tmp_path, method, {"method": method, "a3c": side["a3c"], "a3c_sha256": side["a3c_sha256"],
                                        "region": "S", "robot": "cylinder"})


@heavy
def test_gmc_reachable_route_requeried_is_success_and_export_is_byte_identical(tmp_path):
    ids = ["F4S-00000", "F4S-00001", "F4S-00002"]
    pairs = _s_pairs(ids)
    gmc_rows = {json.loads(x)["pair_id"]: json.loads(x)
                for x in open("results/baselines/gmc_rejudged/S/cylinder/rows.jsonl")}
    art = _setup(tmp_path, "gmc_requery")
    J = H.Judge("S", "cylinder")
    w = H.Worker("gmc_requery", art, {}, tmp_path / "w.log")
    w.start(timeout=600)
    try:
        for p in pairs:
            out, _ = w.query({"op": "query", "pair_id": p["pair_id"], "start_uv": p["start_uv"],
                              "goal_uv": p["goal_uv"]}, 120)
            assert out["claimed"], out
            row = dict(H._base_row("gmc_requery", "S", "cylinder", p, {"setup_id": "t"}, 0),
                       claimed=True, claimed_reason=out["claimed_reason"], method_path_uv=out["path_uv"])
            res = H.judge_row(J, row)
            assert row["status"] == "SUCCESS", row["judge_reason"]
            assert not row["prepended_start_segment"] and not row["appended_goal_segment"]
            # the harness's export of GMC's polyline == GMC's own export, pose for pose, time for time
            assert res["trajectory"] == json.loads(json.dumps(out["info"]["gmc_trajectory"]))
            assert res["original_goal"] == out["info"]["gmc_original_goal"]
            assert row["polyline_sha256"] == H.poly_sha(out["info"]["gmc_polyline_world"])
            assert row["polyline_sha256"] == gmc_rows[p["pair_id"]]["polyline_sha256"]
    finally:
        w.close()


@heavy
def test_straight_line_through_an_obstacle_is_claimed_collides(tmp_path):
    art = _setup(tmp_path, "straight")
    out = tmp_path / "t.jsonl"
    H.run_task("straight", "S", "cylinder", _s_pairs(["F4S-00043", "F4S-00068"]), out, artifact=art, config={})
    rows = [json.loads(x) for x in open(out)]
    assert [r["status"] for r in rows] == ["CLAIMED_COLLIDES"] * 2
    assert all(r["judge_geometry_reason"] == "minkowski_interior_witness" for r in rows)
    assert all(r["judge_wall_s"] > 0 and r["path_length_m"] == pytest.approx(r["dist_m"]) for r in rows)


@heavy
def test_astar_replay_is_success(tmp_path):
    art = _setup(tmp_path, "astar_replay")
    out = tmp_path / "t.jsonl"
    pairs = [p for p in H.load_pairs("tuning") if p["region"] == "S"][:3]
    s = H.run_task("astar_replay", "S", "cylinder", pairs, out, artifact=art, config={}, pass_pair=True)
    assert s["status_counts"] == {"SUCCESS": 3}, s["status_counts"]
    assert s["judge_scene"]["a3c_sha256"] == json.loads(Path(f"{s['judge_scene']['a3c']}.json").read_text())["sha256"]


@heavy
def test_sleeping_method_with_inline_judge_times_out(tmp_path):
    art = _setup(tmp_path, "sleep")
    out = tmp_path / "t.jsonl"
    H.run_task("sleep", "S", "cylinder", _s_pairs(["F4S-00000"]), out, artifact=art, config={"sleep_s": 60},
               timeout_s=3.)
    assert json.loads(open(out).readline())["status"] == "TIMEOUT"
