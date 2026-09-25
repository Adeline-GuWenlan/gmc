"""Ground-5k runner (G1 Task 2): watchdog, outcome mapping, GMC-path replay, both planners.

The synthetic hall is 6 x 4 m with one tall block in the middle; the "walled" variant closes
the whole width.  Both planners run for real on it (seconds), each path is replayed through
the shared gs3d replay, and the outcome set of docs/ground5k_design.md §1 is asserted.
"""
import time

import numpy as np
import pytest

from gmc.height.ply3d import GaussianScene3D

import ground5k_common as gc
import ground5k_run as gr

FLOOR = {"z_floor": 0.0, "normal": [0.0, 0.0, 1.0], "centroid": [3.0, 2.0, 0.0],
         "extent": [0.0, 0.0, 6.0, 4.0], "ceiling_height_m": 3.0}
CFG = {"tau": 0.3, "level": 2.0, "margin_m": 0.001,
       "contact": {"max_height_error_m": 0.05, "max_travel_m": 0.05, "max_slope_deg": 5.0},
       "crop": {"z_below_floor_m": 0.10, "z_above_floor_m": 2.0, "pad_m": 1.0},
       "per_query": {"wall_cap_s": 300, "watchdog_grace_s": 5, "memory_cap_gb": 6.0},
       "astar": {"resolution_m": 0.20, "margin_m": 0.001, "seed": 0, "position_tolerance_m": 0.0,
                 "yaw_tolerance_rad": 0.05,
                 "budget": {"max_expansions": 500000, "max_oracle_calls": 5000000,
                            "max_narrowphase_pairs": 20000000}},
       "gmc": {"config": "configs/height_showcase.yaml", "start_yaw": 0.0, "goal_yaw": 0.0}}


# The planner sections come from the frozen config itself, so a key the runner mishandles
# there fails here (pilot 18500647: a descriptive astar.budget.max_wall_s crashed every A* call).
_FROZEN = gc.load_config()
CFG = {**CFG, "astar": _FROZEN["astar"], "gmc": _FROZEN["gmc"],
       "per_query": {**_FROZEN["per_query"], "wall_cap_s": 300, "memory_cap_gb": 6.0}}


def _block_scene(walled=False):
    ys = np.arange(0.05, 4.0, 0.1) if walled else np.arange(1.45, 2.6, 0.1)
    zs = np.arange(0.1, 1.9, 0.2)
    Y, Z = np.meshgrid(ys, zs, indexing="ij")
    means = np.column_stack([np.full(Y.size, 3.0), Y.ravel(), Z.ravel()])
    covs = np.repeat(np.diag([0.06 ** 2] * 3)[None], len(means), 0)
    return GaussianScene3D(means, covs, np.full(len(means), 0.9), np.arange(len(means)), "synth")


PAIR = {"pair_id": "t-00000", "start": [1.0, 2.0], "goal": [5.0, 2.0],
        "crop": {"box_xy": [0.0, 0.0, 6.0, 4.0]}}


# ---------------------------------------------------------------------- watchdog --

def _ok(progress):
    progress("plan")
    progress("capped_done")
    return {"value": 7}


def _sleep(progress):
    progress("compile")
    time.sleep(30)
    return {}


def _hog(progress):
    progress("compile")
    blocks = []
    for _ in range(40):
        blocks.append(np.ones(50_000_000 // 8))   # 50 MB each, touched
        time.sleep(0.02)
    return {}


def _boom(progress):
    progress("query")
    raise RuntimeError("synthetic failure")


def test_watchdog_returns_the_child_result_and_its_stages():
    out = gr.watchdog(_ok, wall_cap_s=20, mem_cap_bytes=4 << 30)
    assert out["status"] == "ok" and out["result"] == {"value": 7}
    assert [s for s, _ in out["stages"]] == ["plan", "capped_done"]
    assert out["peak_rss_bytes"] > 0


def test_watchdog_kills_at_the_wall_cap_and_names_the_stage():
    t0 = time.time()
    out = gr.watchdog(_sleep, wall_cap_s=1.0, mem_cap_bytes=4 << 30)
    assert time.time() - t0 < 10
    assert out["status"] == "killed" and out["kill_reason"] == "wall_cap" and out["stage"] == "compile"


def test_watchdog_kills_at_the_memory_cap():
    out = gr.watchdog(_hog, wall_cap_s=60, mem_cap_bytes=600 << 20)
    assert out["status"] == "killed" and out["kill_reason"] == "memory_cap"


def test_watchdog_reports_a_child_exception():
    out = gr.watchdog(_boom, wall_cap_s=20, mem_cap_bytes=4 << 30)
    assert out["status"] == "error" and "synthetic failure" in out["error"] and out["stage"] == "query"


# ----------------------------------------------------------------- outcome mapping --

def _astar(status, reason="x", **diag):
    return {"status": status, "reason": reason, "diagnostics": {"map_unknown_rejections": 0,
            "unproven_rejections": 0, "occupied_rejections": 0, **diag}}


@pytest.mark.parametrize("status,reason,oracle,faces,expected", [
    ("budget_exhausted", "max_wall_s", {}, {}, "FAIL_BUDGET"),
    ("no_path_on_lattice", "reachable_lattice_exhausted", {}, {}, "FAIL_NO_PATH"),
    ("verification_failed", "reachable_frontier_has_unproven_edges",
     {"ground_support_missing": 9}, {"x_max": 9}, "FAIL_NO_PATH"),
    ("map_unknown", "reachable_frontier_meets_unknown_coverage",
     {"map_unknown": 3}, {"y_min": 3}, "FAIL_NO_PATH"),
    ("map_unknown", "reachable_frontier_meets_unknown_coverage",
     {"map_unknown": 3}, {"y_min": 1, "contact_lo": 2}, "FAIL_UNKNOWN"),
    ("verification_failed", "reachable_frontier_has_unproven_edges",
     {"geometry_or_margin_unproven": 1}, {}, "FAIL_UNKNOWN"),
    ("start_invalid", "start:occupied", {}, {}, "FAIL_UNKNOWN"),
    ("verification_failed", "post_build_verification_failed", {}, {}, "FAIL_UNKNOWN"),
    ("invalid_input", "bad", {}, {}, "ERROR"),
])
def test_astar_outcome_mapping(status, reason, oracle, faces, expected):
    outcome, cause = gr.classify_astar(_astar(status, reason), oracle, faces)
    assert outcome == expected, cause


@pytest.mark.parametrize("status,reason,certified,expected", [
    ("UNREACHABLE", None, None, "FAIL_NO_PATH"),
    ("UNKNOWN", "possible_cut_is_not_a_global_certificate", None, "FAIL_NO_PATH"),
    ("UNKNOWN", "query_wall_budget_exhausted", None, "FAIL_BUDGET"),
    ("UNKNOWN", "query_support_budget_exhausted", None, "FAIL_BUDGET"),
    ("UNKNOWN", "safe_graph_ambiguous", None, "FAIL_UNKNOWN"),
    ("INVALID_GEOMETRY", "invalid_start_or_goal_pose", None, "FAIL_UNKNOWN"),
    ("INTERNAL_ERROR", "formal_cut_invariant_failed", None, "ERROR"),
    ("REACHABLE", None, False, "FAIL_UNKNOWN"),
])
def test_gmc_outcome_mapping(status, reason, certified, expected):
    res = {"status": status, "reason": reason,
           "verify": {"ran": certified is not None, "certified": certified}}
    assert gr.classify_gmc(res)[0] == expected


def test_replay_summary_keeps_the_failing_edge_for_diagnosis():
    rep = {"passed": False, "kinematics": {"passed": True},
           "geometry": {"passed": False, "reason": "geometry_or_margin_unproven", "reports": [
               {"occupancy": "free", "safety": "continuous_bound", "clearance_lower_m": 0.01, "reason": "ok",
                "primitive_ids": [], "candidate_count": 3, "narrowphase_pairs": 1},
               {"occupancy": "unknown", "safety": "unresolved", "clearance_lower_m": 0.0004,
                "reason": "geometry_or_margin_unproven", "primitive_ids": [17, 42], "candidate_count": 900,
                "narrowphase_pairs": 12}]}}
    poses = [[0, 0, 0, 0], [1, 0, 0, 0], [1, 2, 0, 1.57]]
    s = gr._replay_summary(rep, poses)
    assert s["failed_edge"]["index"] == 1
    assert s["failed_edge"]["primitive_ids"] == [17, 42]
    assert s["failed_edge"]["clearance_lower_m"] == 0.0004
    assert s["failed_edge"]["from"] == [1, 0, 0, 0] and s["failed_edge"]["to"] == [1, 2, 0, 1.57]


def test_replay_verdict_decides_success():
    assert gr.with_replay("SUCCESS_VERIFIED", {"passed": True}) == "SUCCESS_VERIFIED"
    assert gr.with_replay("SUCCESS_VERIFIED", {"passed": False}) == "FAIL_REPLAY"


# ------------------------------------------------------------- GMC path conversion --

def test_gmc_curve_becomes_supported_poses_without_duplicates():
    curve = {"segments": [
        {"kind": "ROTATION", "q0": [1, 2, 0], "q1": [1, 2, 0.5], "control_points": []},
        {"kind": "TRANSLATION", "q0": [1, 2, 0.5], "q1": [3, 3, 0.5], "control_points": [[2, 2.5, 0.5]]},
        {"kind": "TRANSLATION", "q0": [3, 3, 0.5], "q1": [5, 2, 0.5], "control_points": []}]}
    poses = gr.gmc_poses(curve, gc.BODIES["cylinder"], FLOOR)
    assert [p.xyz[:2] for p in poses] == [(1, 2), (2, 2.5), (3, 3), (5, 2)]
    assert all(p.xyz[2] == pytest.approx(0.02 + 0.865) for p in poses)


# ------------------------------------------------------ both planners, synthetic hall --

@pytest.fixture(scope="module")
def open_runs():
    scene = _block_scene()
    return {c: gr.run_one(scene, FLOOR, PAIR, c, CFG, lambda *a, **k: None, scene_name="synth")
            for c in gc.COMBOS}


@pytest.mark.parametrize("combo", gc.COMBOS)
def test_each_combination_certifies_and_replays_a_detour(open_runs, combo):
    rec = open_runs[combo]
    assert rec["outcome"] == "SUCCESS_VERIFIED", (rec["cause"], rec.get("planner_status"))
    assert rec["replay"]["passed"] and rec["replay"]["clearance_lb_m"] > CFG["margin_m"]
    xy = np.asarray(rec["path_xy"])
    assert np.allclose(xy[0], PAIR["start"]) and np.allclose(xy[-1], PAIR["goal"])
    assert rec["path_length_m"] > 4.0 + 1e-6            # the block forces a detour
    assert rec["crop"]["selected_supports"] == len(_block_scene())
    for key in ("crop_s", "plan_s", "total_capped_s", "replay_s"):
        assert rec["time"][key] >= 0


def test_gmc_sees_a_splat_just_under_the_chassis_within_the_shared_margin():
    """Pilot: floor splats topping out 0.2-0.8 mm below the sweeper chassis were outside GMC's
    band, so GMC drove over them and the shared replay (1 mm margin) rejected the path.  GMC's
    band is now inflated by that margin, so both planners solve the same problem."""
    top = 0.02 - 0.0005                      # 0.5 mm below the chassis bottom
    sz = 0.005
    means = np.array([[3.0, 2.0, top - 2 * sz]])
    covs = np.array([np.diag([0.15 ** 2, 0.15 ** 2, sz ** 2])])
    scene = GaussianScene3D(means, covs, np.array([0.9]), np.array([0]), "just_under")
    for combo in ("gmc_sweeper", "astar_sweeper"):
        rec = gr.run_one(scene, FLOOR, PAIR, combo, CFG, lambda *a, **k: None, scene_name="synth")
        assert rec["outcome"] == "SUCCESS_VERIFIED", (combo, rec["cause"], rec["replay"])
        assert rec["path_length_m"] > 4.0 + 1e-6, combo      # it went round the splat


def test_walled_hall_is_no_path_for_both_planners():
    scene = _block_scene(walled=True)
    for combo in ("astar_cylinder", "gmc_cylinder"):
        rec = gr.run_one(scene, FLOOR, PAIR, combo, CFG, lambda *a, **k: None, scene_name="synth")
        assert rec["outcome"] == "FAIL_NO_PATH", (combo, rec["cause"], rec.get("planner_status"))
        assert rec["path_xy"] is None


def test_task_is_resumable_and_writes_one_json_per_query(tmp_path):
    scene = _block_scene()
    pairs = [PAIR, {**PAIR, "pair_id": "t-00001", "goal": [5.0, 1.0]}]
    done = gr.run_pairs(scene, FLOOR, pairs, "astar_sweeper", CFG, tmp_path, scene_name="synth")
    assert sorted(p.name for p in (tmp_path / "astar_sweeper").glob("*.json")) == ["t-00000.json", "t-00001.json"]
    assert done == {"ran": 2, "skipped": 0}
    again = gr.run_pairs(scene, FLOOR, pairs, "astar_sweeper", CFG, tmp_path, scene_name="synth")
    assert again == {"ran": 0, "skipped": 2}


def test_child_memory_cap_stays_below_the_slurm_request():
    assert gr.memory_cap_gb(12.5, None) == 12.5                 # no Slurm limit: config cap
    assert gr.memory_cap_gb(12.5, 12 * 1024) == pytest.approx(10.5)
    assert gr.memory_cap_gb(12.5, 6 * 1024) == pytest.approx(4.5)
    assert gr.memory_cap_gb(12.5, 16 * 1024) == 12.5
