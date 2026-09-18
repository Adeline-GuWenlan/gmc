"""Amendment 3 P4: the pure part of the timing report (window growth, fits, run rows)."""
import math

import numpy as np
import pytest

from gmc.height.timingreport import (MIN_FIT_POINTS, day_capacity, grow_window, linear_fit,
                                     power_fit, row_from_run, stage_group)


# ------------------------------------------------------------------------------------ grow_window
def _area(w):
    return (w[2] - w[0]) * (w[3] - w[1])


def test_grow_window_returns_the_base_when_the_target_is_not_larger():
    base = [6.15, 4.688, 8.15, 8.312]
    assert grow_window(base, 1.0, bounds=[-10, -10, 30, 30]) == base


def test_grow_window_contains_the_base_and_hits_the_area():
    base = [6.15, 4.688, 8.15, 8.312]
    for a in (14.5, 29.0, 58.0, 144.0):
        w = grow_window(base, a, bounds=[-10, -10, 30, 30])
        assert w[0] <= base[0] and w[1] <= base[1] and w[2] >= base[2] and w[3] >= base[3]
        assert _area(w) == pytest.approx(a, rel=2e-3)


def test_grow_window_shifts_off_a_wall_instead_of_losing_area():
    base = [0.0, 0.0, 2.0, 2.0]
    w = grow_window(base, 36.0, bounds=[-0.5, -0.5, 20, 20])
    assert w[0] >= -0.5 - 1e-9 and w[1] >= -0.5 - 1e-9
    assert _area(w) == pytest.approx(36.0, rel=2e-3)
    assert w[0] <= base[0] and w[2] >= base[2]


def test_grow_window_is_none_when_the_bounds_cannot_hold_the_area():
    assert grow_window([0, 0, 1, 1], 50.0, bounds=[0, 0, 5, 5]) is None


# ------------------------------------------------------------------------------------ fits
def test_power_fit_refuses_fewer_than_four_points():
    f = power_fit([1, 10, 100], [2, 20, 200])
    assert MIN_FIT_POINTS == 4
    assert f["n"] == 3 and f["exponent"] is None
    assert "table" in f["refused"]


def test_power_fit_recovers_an_exact_power_law_and_reports_its_range():
    x = np.array([500.0, 2e3, 1e4, 5e4, 1.5e5])
    f = power_fit(x, 0.004 * x ** 1.1)
    assert f["n"] == 5
    assert f["exponent"] == pytest.approx(1.1, abs=1e-9)
    assert f["coef"] == pytest.approx(0.004, rel=1e-9)
    assert f["x_range"] == [500.0, 1.5e5]
    assert f["r2"] == pytest.approx(1.0)


def test_power_fit_drops_non_positive_and_non_finite_points_before_counting():
    f = power_fit([1, 2, 0, 4, 8, float("nan")], [1, 2, 3, 4, 8, 1])
    assert f["n"] == 4


def test_linear_fit_reports_slope_intercept_and_refuses_three_points():
    x = np.array([1e3, 1e4, 5e4, 1e5])
    f = linear_fit(x, 2.0 + 0.0043 * x)
    assert f["slope"] == pytest.approx(0.0043) and f["intercept"] == pytest.approx(2.0)
    assert linear_fit(x[:3], x[:3])["slope"] is None


def test_day_capacity_inverts_the_power_fit_and_labels_itself_an_extrapolation():
    f = power_fit([1e3, 1e4, 1e5, 1e6], [4.3, 43.0, 430.0, 4300.0])
    d = day_capacity(f, seconds=86400.0)
    assert d["n_supports"] == pytest.approx(86400.0 / 0.0043, rel=1e-6)
    assert d["extrapolation"] is True
    assert d["beyond_largest_measured_x"] == pytest.approx(d["n_supports"] / 1e6)
    assert day_capacity(power_fit([1, 2], [1, 2]))["n_supports"] is None


# ------------------------------------------------------------------------------------ rows
def _run(status="REACHABLE", timing=True, certified=True, replay=True):
    d = {"robot": "sweeper", "budget_scale": 1, "scene": "planefloor",
         "case": {"window": [0.0, 0.0, 2.0, 3.0], "start": [0.0, 0.0, 0.0], "goal": [3.0, 4.0, 0.0]},
         "projection": {"kept": 1234},
         "result": {"status": status, "n_supports": 1234, "compile_seconds": 5.0,
                    "query_seconds": 7.0, "clearance_lb": 0.01,
                    "verify": {"ran": status == "REACHABLE", "certified": certified}},
         "replay3d": {"passed": replay, "min_clearance_lb": 0.002} if status == "REACHABLE" else None}
    if timing:
        d["timing"] = {"by_stage": {"scene_load": 1.0, "project": 2.0, "compile_pairs": 0.5,
                                    "compile_slabs": 4.0, "compile_mobility": 0.5, "query": 7.0,
                                    "verify_curve": 3.0, "replay3d": 0.4}}
    return d


def test_row_from_a_timed_run_has_every_stage_and_the_geometry():
    r = row_from_run(_run(), source="P3", label="x")
    assert r["has_stage_timing"] and r["stages"]["compile_slabs"] == 4.0
    assert r["n_supports"] == 1234 and r["window_area_m2"] == pytest.approx(6.0)
    assert r["ab_dist_m"] == pytest.approx(5.0)
    assert r["success"] and not r["query_is_lower_bound"]
    assert r["compile_seconds"] == 5.0 and r["query_seconds"] == 7.0


def test_row_from_an_amendment2_run_carries_compile_and_query_only():
    r = row_from_run(_run(timing=False), source="A2", label="x")
    assert not r["has_stage_timing"] and r["stages"] is None
    assert r["compile_seconds"] == 5.0 and r["query_seconds"] == 7.0
    assert r["scene"] == "planefloor"


def test_an_unknown_run_is_a_lower_bound_and_not_a_success():
    r = row_from_run(_run(status="UNKNOWN", certified=False), source="P3", label="x")
    assert r["query_is_lower_bound"] and not r["success"]


def test_success_needs_reachable_and_certified_and_replay3d():
    assert not row_from_run(_run(replay=False), source="P3", label="x")["success"]
    assert not row_from_run(_run(certified=False), source="P3", label="x")["success"]


def test_stage_groups_follow_the_users_four_questions():
    assert stage_group("scene_load") == stage_group("floor_replace") == stage_group("project") == "prep"
    assert stage_group("compile_slabs") == "compile" and stage_group("query") == "query"
    assert stage_group("verify_curve") == stage_group("replay3d") == "prove"
    with pytest.raises(ValueError):
        stage_group("compile_paris")


# ------------------------------------------------------------------------------------ sweep rows
from gmc.height.timingreport import compile_total, row_from_sweep  # noqa: E402


def _unit(mode="cold", status="REACHABLE", stages=None):
    return {"kind": "unit", "source": "P4-sweep", "lane": "sweeper", "experiment": "dist", "mode": mode,
            "robot": "sweeper", "window": [0.0, 0.0, 2.0, 3.0], "window_area_m2": 6.0,
            "start": [0.0, 0.0], "goal": [3.0, 4.0], "ab_dist_m": 5.0, "n_supports": 99,
            "stages": stages if stages is not None else
            {"compile_pairs": 1.0, "compile_slabs": 10.0, "compile_mobility": 1.0, "query": 5.0,
             "verify_curve": 2.0, "replay3d": 0.5},
            "status": status, "certified": status == "REACHABLE", "replay3d_passed": status == "REACHABLE",
            "success": status == "REACHABLE", "query_is_lower_bound": status == "UNKNOWN",
            "query_report": {"refinement_rounds": 2}}


def test_compile_total_sums_the_three_compile_stages_and_is_none_without_them():
    assert compile_total({"compile_pairs": 1.0, "compile_slabs": 10.0, "compile_mobility": 1.0}) == 12.0
    assert compile_total({"query": 5.0}) is None
    assert compile_total(None) is None


def test_a_cold_sweep_row_has_compile_and_query_like_a_run_row():
    r = row_from_sweep(_unit())
    assert r["source"] == "P4-sweep" and r["mode"] == "cold" and r["has_stage_timing"]
    assert r["compile_seconds"] == 12.0 and r["query_seconds"] == 5.0
    assert r["success"] and r["refinement_rounds"] == 2


def test_a_warm_sweep_row_has_a_query_but_no_compile():
    r = row_from_sweep(_unit(mode="warm1", stages={"query": 3.0, "verify_curve": 1.0}))
    assert r["compile_seconds"] is None and r["query_seconds"] == 3.0


def test_a_run_row_is_mode_single():
    assert row_from_run(_run(), source="P3", label="x")["mode"] == "single"
