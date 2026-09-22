from dataclasses import replace
import json
import numpy as np
import pytest

from gmc.gs3d.contracts import GoalRegion, PlannerConfig, Pose3, SearchBudget
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.planner import LatticePlanner
from gmc.gs3d.validation import verify_path, verify_linear_trajectory
from gs3d_core_fixtures import (UAV, CYLINDER, SWEEPER, FlatSupport, KnownBox,
                               make_scene, under_over, z_distinction)


def test_projection_trap_and_same_xy_different_z(monkeypatch):
    import gmc.height.project
    calls = []

    def forbidden(*args, **kwargs):
        calls.append(1)
        raise AssertionError("production projection is forbidden")

    monkeypatch.setattr(gmc.height.project, "project_scene", forbidden)
    start, goal = Pose3((-1.5, 0, .7)), GoalRegion(Pose3((1.5, 0, .7)))
    low = z_distinction(.7)
    high = z_distinction(2.)
    low_oracle = GaussianBodyOracle(low)
    high_oracle = GaussianBodyOracle(high)
    assert low_oracle.edge(start, goal.original, UAV, margin_m=.05).occupancy != "free"
    assert high_oracle.edge(start, goal.original, UAV, margin_m=.05).occupancy == "free"
    low_result = LatticePlanner().plan(low, UAV, start, goal, PlannerConfig(resolution_m=.2))
    high_result = LatticePlanner().plan(high, UAV, start, goal, PlannerConfig(resolution_m=.2))
    assert low_result["status"] == high_result["status"] == "success"
    assert low_result["diagnostics"]["path_length_m"] > 3.01
    assert low_result["diagnostics"]["altitude_range_m"] > .1
    assert high_result["diagnostics"]["path_length_m"] == pytest.approx(3.)
    assert high_result["diagnostics"]["altitude_range_m"] == 0
    assert calls == []


@pytest.mark.parametrize("resolution", [.2, .1, .05])
def test_under_over_impossible_for_fixed_z(resolution):
    scene, start, target = under_over()
    result = LatticePlanner().plan(scene, UAV, start, GoalRegion(target),
                                   PlannerConfig(resolution_m=resolution))
    assert result["status"] == "success", result["reason"]
    assert result["safety"] == "continuous_bound"
    assert result["clearance_lower_m"] > .05
    rows = np.asarray(result["trajectory"]["poses"])
    assert np.ptp(rows[:, 2]) >= .5
    # Geometric impossibility independent of planning: even at the farthest
    # legal lateral centre, the two supports impose disjoint altitude ranges.
    max_y = .5 - UAV.radius_m - .05
    light_underside = 2. - np.sqrt(1 - (max_y / 2.) ** 2)
    table_top = .3 + .95 * np.sqrt(1 - (max_y / 2.) ** 2)
    assert light_underside - UAV.half_height_m - .05 < table_top + UAV.half_height_m + .05
    assert rows[0, 2] == .65 and rows[-1, 2] == 1.65
    assert result["diagnostics"]["verification"]["passed"]


@pytest.mark.parametrize("body", [SWEEPER, CYLINDER])
def test_supported_ground_paths_have_stopped_turns(body):
    scene = make_scene([(0, 0, .8)], [(.15, .25, .75)],
                       lower=(-2, -2, -.1), upper=(2, 2, 3), support=FlatSupport())
    z = body.half_height_m + body.ground_clearance_m
    result = LatticePlanner().plan(scene, body, Pose3((-1, 0, z), .7),
        GoalRegion(Pose3((1, 0, z), -.6)), PlannerConfig(resolution_m=.2, margin_m=.001))
    assert result["status"] == "success", result["reason"]
    rows = np.asarray(result["trajectory"]["poses"])
    assert np.ptp(rows[:, 2]) == 0
    assert np.any(np.linalg.norm(np.diff(rows[:, :2], axis=0), axis=1) < 1e-10)
    poses = [Pose3(tuple(row[:3]), row[3]) for row in rows]
    assert verify_path(GaussianBodyOracle(scene), poses, body, margin_m=.001)["passed"]
    assert result["diagnostics"]["kinematics"]["acceleration_verified"] is False


@pytest.mark.parametrize("goal_occupied", [False, True])
def test_relaxed_goal_preserves_unsafe_original_and_reaches_safe_candidate(goal_occupied):
    lower, upper = (-1, -.5, 0), (1.5, .5, 2)
    # Original lies just inside a closed unknown half-space; nearest safe lattice
    # candidate is reachable from the left. Body extent is included in coverage.
    known = KnownBox(lower, upper, holes=((((.86 if goal_occupied else .9), -.5, 0), upper),))
    scene = make_scene([(1., 0, 1)] if goal_occupied else [],
                       [(.05, .3, .3)] if goal_occupied else [],
                       lower=lower, upper=upper, known=known if not goal_occupied else KnownBox(lower, upper))
    body = replace(UAV, radius_m=.1, half_height_m=.1)
    original = Pose3((.91, 0, 1))
    start = Pose3((-.7, 0, 1))
    exact = LatticePlanner().plan(scene, body, start, GoalRegion(original), PlannerConfig(resolution_m=.1, margin_m=.01))
    assert exact["status"] in ("goal_invalid", "map_unknown")
    relaxed = LatticePlanner().plan(scene, body, start, GoalRegion(original, .25), PlannerConfig(resolution_m=.1, margin_m=.01))
    assert relaxed["status"] == "success", relaxed["reason"]
    assert relaxed["original_goal"] == [.91, 0., 1., 0.]
    assert np.linalg.norm(np.array(relaxed["attained_goal"][:3]) - original.xyz) <= .25 + 1e-9
    assert relaxed["endpoint_reports"]["attained"]["occupancy"] == "free"
    assert relaxed["endpoint_reports"]["original"]["occupancy"] != "free"


def test_goal_candidate_reachability_not_nearest_free_selection():
    # A fully blocking wall separates the original from the start, while the
    # .5 m goal ball reaches a safe left-side candidate. Nearest free points on
    # the right must not override the reachable candidate on the left.
    scene = make_scene([(0., 0., 1.)], [(.08, 5., 5.)],
                       lower=(-1., -.25, .6), upper=(1., .25, 1.4))
    body = replace(UAV, radius_m=.05, half_height_m=.05)
    original = Pose3((.25, 0, 1))
    result = LatticePlanner().plan(scene, body, Pose3((-.7, 0, 1)),
        GoalRegion(original, .5), PlannerConfig(resolution_m=.05, margin_m=.01))
    assert result["status"] == "success", result["reason"]
    assert result["attained_goal"][0] < -.13
    assert result["endpoint_reports"]["original"]["occupancy"] == "free"


@pytest.mark.parametrize("budget, expected", [
    (SearchBudget(max_expansions=1), "max_expansions"),
    (SearchBudget(max_oracle_calls=1), "max_oracle_calls"),
    (SearchBudget(max_narrowphase_pairs=1), "max_narrowphase_pairs"),
    (SearchBudget(max_wall_s=1e-12), "max_wall_s")])
def test_distinct_bounded_failure_with_no_executable_path(budget, expected):
    scene, start, target = under_over()
    result = LatticePlanner().plan(scene, UAV, start, GoalRegion(target), PlannerConfig(budget=budget))
    assert result["status"] == "budget_exhausted"
    assert result["reason"] == expected
    assert result["trajectory"] is None and result["attained_goal"] is None
    assert result["diagnostics"]["narrowphase_pairs"] <= budget.max_narrowphase_pairs
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("tolerance", [-.1, .51, np.nan, np.inf])
def test_invalid_goal_tolerance_json_serializable(tolerance):
    scene = make_scene()
    result = LatticePlanner().plan(scene, UAV, Pose3((-1, 0, 1)),
        GoalRegion(Pose3((1, 0, 1)), tolerance), PlannerConfig())
    assert result["status"] == "invalid_input"
    assert result["trajectory"] is None
    json.dumps(result, allow_nan=False)


def test_singleton_path_timing_and_warm_reset():
    scene = make_scene()
    planner = LatticePlanner(PreparedScene(scene))
    q = Pose3((0, 0, 1))
    results = [planner.plan(scene, UAV, q, GoalRegion(q), PlannerConfig()) for _ in range(2)]
    for r in results:
        assert r["status"] == "success"
        assert r["trajectory"]["time_s"] == [0.]
        assert len(r["trajectory"]["poses"]) == 1
        assert r["timings"]["mode"] == "warm"
    assert results[0]["diagnostics"]["oracle_calls"] == results[1]["diagnostics"]["oracle_calls"]


def test_ground_validator_rejects_sideways_motion_and_bad_timing():
    scene = make_scene(support=FlatSupport())
    oracle = GaussianBodyOracle(scene)
    poses = [Pose3((-.5, 0, .06), np.pi / 2), Pose3((.5, 0, .06), np.pi / 2)]
    assert verify_path(oracle, poses, SWEEPER, margin_m=.001)["reason"] == "ground_lateral_slip_or_turn_during_translation"
    assert not verify_path(oracle, [], SWEEPER, margin_m=.001)["passed"]
    q = Pose3((0, 0, 1))
    result = LatticePlanner().plan(scene, UAV, q, GoalRegion(Pose3((.5, 0, 1))), PlannerConfig())
    trajectory = result["trajectory"]
    invalid_limits = {**result["robot"]["limits"], "max_speed_mps": float("nan")}
    assert not verify_linear_trajectory(trajectory, UAV, invalid_limits)["passed"]
    trajectory["time_s"][-1] = 0.
    assert not verify_linear_trajectory(trajectory, UAV, result["robot"]["limits"])["passed"]


def test_timer_hooks_include_endpoint_search_verify():
    from contextlib import contextmanager

    class Timer:
        def __init__(self):
            self.stages = []

        @contextmanager
        def stage(self, name, **kwargs):
            self.stages.append(name)
            yield

    timer = Timer()
    scene = make_scene()
    result = LatticePlanner().plan(scene, UAV, Pose3((-1, 0, 1)),
                                   GoalRegion(Pose3((1, 0, 1))), PlannerConfig(), timer=timer)
    assert result["status"] == "success"
    assert {"index_build", "endpoint_check", "edge_validation", "trajectory_build", "verify"} <= set(timer.stages)


def test_no_path_on_coarse_lattice_is_not_map_unknown_or_budget():
    scene = make_scene([(0, 0, 1)], [(.025, .2, .2)],
                       lower=(-.3, -.1, .85), upper=(.3, .1, 1.15))
    body = replace(UAV, radius_m=.05, half_height_m=.05)
    result = LatticePlanner().plan(scene, body, Pose3((-.15, 0, 1)),
        GoalRegion(Pose3((.15, 0, 1))), PlannerConfig(resolution_m=.5, margin_m=.01))
    assert result["status"] == "no_path_on_lattice", result["reason"]
    assert result["trajectory"] is None
    assert result["diagnostics"]["expansions"] == 1


@pytest.mark.parametrize("change", [
    {"resolution_m": 0}, {"resolution_m": np.nan}, {"margin_m": -1},
    {"budget": SearchBudget(max_expansions=0)}, {"budget": SearchBudget(max_oracle_calls=1.5)},
    {"budget": SearchBudget(max_wall_s=np.inf)}])
def test_invalid_config_cannot_enter_search(change):
    result = LatticePlanner().plan(make_scene(), UAV, Pose3((-.5, 0, 1)),
        GoalRegion(Pose3((.5, 0, 1))), replace(PlannerConfig(), **change))
    assert result["status"] == "invalid_input"
    assert result["diagnostics"]["expansions"] == 0
    assert result["trajectory"] is None
    json.dumps(result, allow_nan=False)
