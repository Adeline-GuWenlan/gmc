from dataclasses import replace
import json

import numpy as np
import pytest

from gmc.gs3d.contracts import GoalRegion, PlannerConfig, Pose3, SceneSpec, SearchBudget
from gmc.gs3d.goals import (CYLINDER_TOLERANCES_M, classify_attempt,
                            cylinder_goal_sensitivity, plan_with_refinement)
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.planner import LatticePlanner
from gmc.gs3d.robots import (CYLINDER, SWEEPER, UAV, BoxKnownSpace,
                             EvidenceBoundedPlaneSupport, crop_by_support_aabb,
                             robot_catalog, supported_pose)
from gmc.gs3d.trajectory import (read_plan, replay_plan, sample_linear_trajectory,
                                 write_plan)
from gs3d_core_fixtures import make_scene, under_over


def flat_support(lower=(-20., -5., -1.), upper=(20., 5., 3.)):
    evidence = BoxKnownSpace(lower, upper)
    return EvidenceBoundedPlaneSupport((0., 0., 1.), (0., 0., 0.), evidence)


def ground_scene(*, obstacle=True, lower=(-6., -2., -.1), upper=(6., 2., 2.2)):
    support = flat_support((lower[0], lower[1], -1.), (upper[0], upper[1], 3.))
    scene = make_scene([(0., 0., .9)] if obstacle else [],
                       [(.15, .45, .8)] if obstacle else [],
                       lower=lower, upper=upper, support=support)
    return scene, support


def test_frozen_robot_dimensions_and_supported_manifold():
    assert robot_catalog() == {"uav": UAV, "sweeper": SWEEPER, "cylinder": CYLINDER}
    assert (UAV.radius_m, UAV.half_height_m) == (.25, .10)
    assert (SWEEPER.radius_m, SWEEPER.half_height_m) == (.175, .04)
    assert (CYLINDER.radius_m, CYLINDER.half_height_m) == (.30, .865)
    support = EvidenceBoundedPlaneSupport((-.01, 0., 1.), (0., 0., 0.),
        BoxKnownSpace((-10., -2., -1.), (10., 2., 3.)), max_travel_m=.2)
    q = supported_pose(3., 0., .4, CYLINDER, support)
    assert q.xyz[2] == pytest.approx(support.height(3., 0.) + .885)
    assert support.slope_deg < 1.
    assert support.supports_segment(q.xyz, supported_pose(5., 0., .4, CYLINDER, support).xyz,
                                    CYLINDER.radius_m)
    with pytest.raises(ValueError, match="slope"):
        EvidenceBoundedPlaneSupport((1., 0., 1.), (0., 0., 0.), support.evidence)


def test_whole_footprint_support_and_contact_travel_fail_closed():
    hole = (((.1, -.1, -1.), (.2, .1, 3.)),)
    support = EvidenceBoundedPlaneSupport((0., 0., 1.), (0., 0., 0.),
        BoxKnownSpace((-2., -1., -1.), (2., 1., 3.), holes=hole))
    q = Pose3((0., 0., .06))
    scene = make_scene(support=support)
    report = GaussianBodyOracle(scene).pose(q, SWEEPER, margin_m=.001)
    assert report.reason == "ground_swept_support_unproven"
    steep_travel = EvidenceBoundedPlaneSupport((-.04, 0., 1.), (0., 0., 0.),
        BoxKnownSpace((-3., -1., -1.), (3., 1., 3.)), max_travel_m=.05)
    a = supported_pose(-1., 0., 0., CYLINDER, steep_travel)
    b = supported_pose(1., 0., 0., CYLINDER, steep_travel)
    assert not steep_travel.supports_segment(a.xyz, b.xyz, CYLINDER.radius_m)


def test_support_crop_uses_ellipsoid_overlap_not_centres():
    scene = make_scene([(10., 0., 1.), (0., 0., 1.)], [(11., .1, .1), (.1, .1, .1)])
    cropped, evidence = crop_by_support_aabb(scene.gaussians, (-.2, -.2, .5), (.2, .2, 1.5),
                                             level=2., tau=.3)
    assert cropped.ids.tolist() == [0, 1]
    assert evidence["crop_rule"] == "full_3d_ellipsoid_aabb_overlap"


@pytest.mark.parametrize("body", [SWEEPER, CYLINDER])
def test_ground_adapters_plan_3d_swept_unicycle(body):
    scene, support = ground_scene()
    start = supported_pose(-1.2, 0., .6, body, support)
    goal = supported_pose(1.2, 0., -.4, body, support)
    result = LatticePlanner().plan(scene, body, start, GoalRegion(goal),
        PlannerConfig(resolution_m=.2, margin_m=.001))
    assert result["status"] == "success", result["reason"]
    rows = np.asarray(result["trajectory"]["poses"])
    assert np.ptp(rows[:, 2]) == 0
    stationary = np.linalg.norm(np.diff(rows[:, :3], axis=0), axis=1) < 1e-12
    assert stationary.any()
    assert result["diagnostics"]["verification"]["passed"]


def test_uav_variable_z_survives_serialization_sampling_and_replay(tmp_path):
    scene, start, target = under_over()
    result = LatticePlanner().plan(scene, UAV, start, GoalRegion(target),
                                   PlannerConfig(resolution_m=.2))
    assert result["status"] == "success"
    path = write_plan(tmp_path / "uav.json", result)
    loaded = read_plan(path)
    samples = sample_linear_trajectory(loaded["trajectory"])
    replay = replay_plan(loaded, GaussianBodyOracle(scene))
    assert samples["altitude_range_m"] >= .5
    assert replay["passed"] and replay["variable_z"]
    assert np.ptp(np.asarray(replay["samples"]["poses"])[:, 2]) >= .5
    bad = json.loads(path.read_text())
    bad["schema_version"] = "legacy.height.v2"
    path.write_text(json.dumps(bad))
    with pytest.raises(ValueError, match="legacy Pose2"):
        read_plan(path)


def test_goal_policy_preserves_original_and_records_attained_error():
    lower, upper = (-1., -.5, 0.), (1.5, .5, 2.)
    known = BoxKnownSpace(lower, upper, holes=(((.9, -.5, 0.), upper),))
    scene = make_scene(lower=lower, upper=upper, known=known)
    body = replace(UAV, radius_m=.1, half_height_m=.1)
    start, original = Pose3((-.7, 0., 1.)), Pose3((.91, 0., 1.))
    planner = LatticePlanner(PreparedScene(scene))
    exact = plan_with_refinement(planner, scene, body, start, GoalRegion(original),
                                 PlannerConfig(resolution_m=.1, margin_m=.01),
                                 refinement_resolution_m=.05)
    assert exact["result"]["status"] == "map_unknown"
    assert len(exact["attempts"]) == 1
    relaxed = plan_with_refinement(planner, scene, body, start, GoalRegion(original, .25),
                                   PlannerConfig(resolution_m=.1, margin_m=.01))
    result = relaxed["result"]
    assert result["status"] == "success"
    assert result["original_goal"] == [.91, 0., 1., 0.]
    assert 0 < result["diagnostics"]["goal_error"]["position_m"] <= .25
    assert relaxed["diagnosis"]["endpoint_map_status"]["original"]["occupancy"] == "unknown"


def test_cylinder_exact_relaxed_matrix_and_separate_failure_causes():
    scene, support = ground_scene(obstacle=False, lower=(-2., -1., -.1), upper=(2., 1., 2.))
    start = supported_pose(-1., 0., 0., CYLINDER, support)
    original = supported_pose(1., 0., 0., CYLINDER, support)
    matrix = cylinder_goal_sensitivity(LatticePlanner(PreparedScene(scene)), scene, start, original,
        PlannerConfig(resolution_m=.2, margin_m=.001), body=CYLINDER)
    assert matrix["tolerances_m"] == list(CYLINDER_TOLERANCES_M)
    assert all(case["original_goal"] == list((*original.xyz, original.yaw)) for case in matrix["cases"])
    assert all(case["result"]["status"] == "success" for case in matrix["cases"])
    limited = LatticePlanner().plan(scene, CYLINDER, start, GoalRegion(original),
        PlannerConfig(resolution_m=.2, margin_m=.001,
                      budget=SearchBudget(max_oracle_calls=1)))
    diagnosis = classify_attempt(limited)
    assert diagnosis["budget_exhaustion"] == "max_oracle_calls"
    assert diagnosis["endpoint_map_status"]["start"]["occupancy"] == "free"
    assert diagnosis["connectivity"]["neighbourhood"] == 8


def test_refinement_is_at_most_once_and_not_used_for_budget(monkeypatch):
    calls = []

    class Planner:
        def plan(self, scene, body, start, goal, config, timer=None):
            calls.append(config.resolution_m)
            return {"status": "no_path_on_lattice", "reason": "reachable_lattice_exhausted",
                    "original_goal": [*goal.original.xyz, goal.original.yaw], "attained_goal": None,
                    "endpoint_reports": {}, "diagnostics": {"termination": "reachable_lattice_exhausted"}}

    scene, start, target = under_over()
    policy = plan_with_refinement(Planner(), scene, UAV, start, GoalRegion(target),
                                  PlannerConfig(resolution_m=.2), refinement_resolution_m=.1)
    assert calls == [.2, .1] and len(policy["attempts"]) == 2
    with pytest.raises(ValueError, match="finer"):
        plan_with_refinement(Planner(), scene, UAV, start, GoalRegion(target),
                             PlannerConfig(resolution_m=.2), refinement_resolution_m=.2)
