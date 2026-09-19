"""Amendment 3 P5: re-select the shared case from P2's own ranked list, with an affordability filter.

P2's chosen case (P2-SW-0) holds 156,426 cylinder supports. Its query ran out of the frozen budget twice
(7,968 s, then 15,027 s at BUDGET=2), so task 1 has two certified routes, not three. P3's ladder
measured where the cylinder certifies: every map up to 46,835 supports did, every map from 73,178 up did
not. P5 walks P2's candidate windows **in P2's own order** and takes the first one whose exact cylinder
map is no larger than the largest map that certified, and whose exact pre-check passes every P2
criterion unchanged. The filter only removes candidates; it never admits one P2's criteria reject.
"""
import pytest

from gmc.height.casesearch import first_affordable


def cand(i, n_cyl, passes=True, failed=()):
    return {"i": i, "window": [0, 0, 1, 1], "start": [0.1, 0.1], "goal": [0.9, 0.9],
            "_n": n_cyl, "_pass": passes, "_failed": list(failed)}


def run(cands, cap, max_checks=None):
    exact_calls = []

    def exact(c):
        exact_calls.append(c["i"])
        return {"pass": c["_pass"], "failed_criteria": c["_failed"], "i": c["i"]}

    chosen, tried = first_affordable(cands, lambda c: c["_n"], exact, cap, max_checks=max_checks)
    return chosen, tried, exact_calls


def test_takes_the_first_candidate_in_rank_order_that_fits_and_passes():
    cands = [cand(0, 156_426), cand(1, 40_000, passes=False, failed=["connected_cylinder"]),
             cand(2, 30_000), cand(3, 10_000)]
    chosen, tried, calls = run(cands, 46_835)
    assert chosen["i"] == 2
    assert [t["verdict"] for t in tried] == ["over_cap", "fails:connected_cylinder", "pass"]


def test_over_cap_candidates_never_reach_the_exact_check():
    cands = [cand(0, 156_426), cand(1, 50_000), cand(2, 46_835)]
    chosen, tried, calls = run(cands, 46_835)
    assert calls == [2]
    assert chosen["i"] == 2          # the cap is inclusive: the largest map that certified is affordable


def test_every_candidate_tried_is_recorded_with_its_cylinder_count():
    cands = [cand(0, 99_000), cand(1, 20_000)]
    _, tried, _ = run(cands, 46_835)
    assert [t["cylinder_supports"] for t in tried] == [99_000, 20_000]
    assert all({"window", "start", "goal"} <= set(t) for t in tried)


def test_no_affordable_passing_candidate_returns_none_and_the_full_record():
    cands = [cand(0, 156_426), cand(1, 20_000, passes=False, failed=["straight_line_under_overhang"])]
    chosen, tried, _ = run(cands, 46_835)
    assert chosen is None
    assert len(tried) == 2


def test_max_checks_bounds_the_walk():
    cands = [cand(i, 99_000) for i in range(10)] + [cand(10, 1_000)]
    chosen, tried, _ = run(cands, 46_835, max_checks=5)
    assert chosen is None and len(tried) == 5


def test_the_filter_cannot_admit_a_candidate_the_exact_check_rejects():
    cands = [cand(0, 100, passes=False, failed=["d1_goal_clear_uav"])]
    chosen, _, _ = run(cands, 46_835)
    assert chosen is None


percase_render = pytest.importorskip("percase_render")


def test_p5_shared_preset_carries_the_boundary_and_fits_one_title_line():
    c = percase_render.claim_for("planefloor", "p5shared")
    assert "USER-APPROVED MANUAL SCENE EDIT" in c and "D1" in c
    assert "three robots" in c and "P5" in c
    assert len(c) <= 135


plane_case_search = pytest.importorskip("plane_case_search")


def test_compare_output_names_default_to_p2s_and_follow_the_tag():
    assert plane_case_search.compare_names("p2c") == ("p2c_three_routes.png", "p2c_shared_case.json")
    assert plane_case_search.compare_names("p5a") == ("p5a_three_routes.png", "p5a_shared_case.json")


def test_step_affordable_end_to_end_on_the_synthetic_table_scene(tmp_path, monkeypatch):
    """The whole step on synth3d's open table scene: over-cap windows are skipped, the passing one is
    written as a case.json showcase_run.py can consume, and P2's chosen case is recorded as kept."""
    import json
    from types import SimpleNamespace

    from gmc.height.synth3d import GOAL, START, WORKSPACE, Z_FLOOR, table_scene
    scene, _ = table_scene("open")
    meta = {"z_floor": Z_FLOOR, "ceiling_height_m": 2.5, "gravity_rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}
    pcs = plane_case_search
    monkeypatch.setattr(pcs, "load_plane_scene", lambda path: (scene, meta))
    monkeypatch.setattr(pcs, "OUT", tmp_path)
    monkeypatch.setattr(pcs, "FIGS", tmp_path / "figs")
    cap_file = tmp_path / "rung2.json"
    cap_file.write_text(json.dumps({"result": {"status": "REACHABLE", "verify": {"certified": True},
                                               "n_supports": 10 ** 9}, "replay3d": {"passed": True}}))
    monkeypatch.setattr(pcs, "CAP_RUN", cap_file)
    win = [float(v) for v in WORKSPACE]
    c = {"window": win, "start": list(START[:2]), "goal": list(GOAL[:2]), "region": "synth",
         "margin": 0.0, "contrast": 1.0, "overhang_note": "synth3d table"}
    (tmp_path / "p2a_search.json").write_text(json.dumps({"candidates": [c]}))
    pcs.step_affordable(SimpleNamespace(case_dir=str(tmp_path / "shared_a")))

    out = json.loads((tmp_path / "p5_affordable_search.json").read_text())
    assert out["p2_chosen_case_kept"]["reported_as_it_stands"] is True
    assert out["tried"][0]["verdict"] == "pass", out["tried"]
    case = json.loads((tmp_path / "shared_a" / "case.json").read_text())
    assert case["window"] == win and len(case["start"]) == 3 and len(case["goal"]) == 3
    assert case["selection"]["label"] == "P5-synth-0" and case["selection"]["cap_supports"] == 10 ** 9
    assert case["claims_boundary"] == pcs.CLAIMS
    assert (tmp_path / "figs" / "p5_affordable0_synth.png").exists()


def test_step_affordable_skips_a_window_over_the_cap(tmp_path, monkeypatch):
    import json
    from types import SimpleNamespace

    from gmc.height.synth3d import GOAL, START, WORKSPACE, Z_FLOOR, table_scene
    scene, _ = table_scene("open")
    meta = {"z_floor": Z_FLOOR, "ceiling_height_m": 2.5, "gravity_rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}
    pcs = plane_case_search
    monkeypatch.setattr(pcs, "load_plane_scene", lambda path: (scene, meta))
    monkeypatch.setattr(pcs, "OUT", tmp_path)
    monkeypatch.setattr(pcs, "FIGS", tmp_path / "figs")
    cap_file = tmp_path / "rung2.json"
    cap_file.write_text(json.dumps({"result": {"status": "REACHABLE", "verify": {"certified": True},
                                               "n_supports": 1}, "replay3d": {"passed": True}}))
    monkeypatch.setattr(pcs, "CAP_RUN", cap_file)
    c = {"window": [float(v) for v in WORKSPACE], "start": list(START[:2]), "goal": list(GOAL[:2]),
         "region": "synth", "margin": 0.0, "contrast": 1.0, "overhang_note": "synth3d table"}
    (tmp_path / "p2a_search.json").write_text(json.dumps({"candidates": [c]}))
    pcs.step_affordable(SimpleNamespace(case_dir=str(tmp_path / "shared_a")))
    out = json.loads((tmp_path / "p5_affordable_search.json").read_text())
    assert out["chosen"] is None and out["tried"][0]["verdict"] == "over_cap"
    assert not (tmp_path / "shared_a" / "case.json").exists()


def test_overlay_layer_paints_occupied_cells_in_its_colour_and_leaves_the_rest_clear():
    """The compare figure's overlay panel said 'grey = cylinder, red = sweeper' but drew yellow and no
    grey: imshow normalised the constant masked array to 0. The layer is now explicit RGBA."""
    import numpy as np
    occ = np.array([[True, False], [False, True]])
    rgba = plane_case_search.mask_rgba(occ, (0.8, 0.1, 0.1), 0.9)
    assert rgba.shape == (2, 2, 4)
    assert np.allclose(rgba[0, 0], (0.8, 0.1, 0.1, 0.9)) and np.allclose(rgba[1, 1], (0.8, 0.1, 0.1, 0.9))
    assert rgba[0, 1, 3] == 0 and rgba[1, 0, 3] == 0
