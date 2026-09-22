import copy

import numpy as np
import pytest

from gmc.gs3d.contracts import GoalRegion, PlannerConfig, Pose3
from gmc.gs3d.integration import (RouteBoxKnownSpace, concatenate_linear_trajectories,
                                  ordered_uav_gate_evidence, replay_linear_mission)
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.planner import LatticePlanner
from gmc.gs3d.robots import UAV
from gmc.gs3d.scene import candidate_knots_world, manual_edits, route_frame
from gs3d_core_fixtures import make_scene


def test_rotated_route_coverage_checks_complete_world_aabb():
    origin, rotation = route_frame(-1.2)
    known = RouteBoxKnownSpace(tuple(origin), tuple(map(tuple, rotation)),
                               (-.75, -1., .2), (3., 1., 2.))
    lower, upper = known.world_bounds()
    assert known.contains_aabb(origin + [-.1, -.1, .5], origin + [.1, .1, .7])
    assert not known.contains_aabb((lower[0] - .01, lower[1], lower[2]), upper)


def _result(scene, a, b):
    return LatticePlanner(PreparedScene(scene)).plan(
        scene, UAV, a, GoalRegion(b), PlannerConfig(resolution_m=.1, margin_m=.05))


def test_concatenated_mission_replays_actual_variable_z():
    scene = make_scene(lower=(-2., -2., 0.), upper=(3., 2., 2.5))
    points = [Pose3((-.5, 0., .6)), Pose3((.5, 0., .6)),
              Pose3((.5, 0., 1.3)), Pose3((2., 0., 1.3))]
    results = [_result(scene, a, b) for a, b in zip(points, points[1:])]
    trajectory = concatenate_linear_trajectories(results)
    replay = replay_linear_mission(trajectory, GaussianBodyOracle(PreparedScene(scene)), UAV,
                                   margin_m=.05, goal=GoalRegion(points[-1]))
    assert replay["passed"] and replay["variable_z"]
    assert replay["samples"]["altitude_range_m"] == pytest.approx(.7)
    broken = copy.deepcopy(results)
    broken[1]["trajectory"]["poses"][0][0] += .1
    with pytest.raises(ValueError, match="not position/yaw continuous"):
        concatenate_linear_trajectories(broken)


def test_ordered_gate_evidence_uses_exported_linear_path():
    z_floor = -1.2271749593107995
    points = candidate_knots_world(z_floor)
    trajectory = {"poses": [[*point, 0.] for point in points],
                  "time_s": [0., 1.5, 4., 8.], "interpolation": "linear_xyz_yaw",
                  "segments": [], "control_dt_s": .05}
    origin, rotation = route_frame(z_floor)
    edits = []
    for index, edit in enumerate(manual_edits(z_floor), 100):
        local = (np.asarray(edit.mean_world_m) - origin) @ rotation.T
        edits.append({"role": edit.role, "gaussian_id": index,
                      "mean_route_m": local.tolist(),
                      "semiaxes_route_m": list(edit.semiaxes_route_m)})
    manifest = {"route_frame": {"origin_world_m": origin.tolist(),
                                 "world_to_route": rotation.tolist()},
                "manual_geometry": edits,
                "configuration": {"body_clearance_margin_m": .05},
                "existing_table_arrangement": {"top_height_above_floor_m": .905,
                                                "route_envelope_m": {
                                                    "lower": [.65, -1.2, .15],
                                                    "upper": [2.15, 1.2, 1.25]},
                                                "count": 10}}
    replay = {"passed": True, "geometry": {"clearance_lower_m": .069}}
    evidence = ordered_uav_gate_evidence(trajectory, manifest, replay)
    assert evidence["all_pass"]
    assert evidence["low_crossing"]["time_s"][1] < evidence["high_crossing"]["time_s"][0]
    reversed_trajectory = dict(trajectory, poses=list(reversed(trajectory["poses"])))
    assert not ordered_uav_gate_evidence(reversed_trajectory, manifest, replay)["all_pass"]


def test_ordered_high_gate_merges_safe_lattice_staircase():
    z_floor = -1.2271749593107995
    origin, rotation = route_frame(z_floor)
    route = np.array([[-.25, 0., .65], [.5, 0., .65], [.5, 0., 1.4],
                      [.7, -.05, 1.4], [.9, -.10, 1.4], [1.1, -.15, 1.4],
                      [1.3, -.10, 1.4], [1.5, -.05, 1.4], [1.7, 0., 1.4],
                      [1.9, .05, 1.4], [2.5, 0., 1.4]])
    world = route @ rotation + origin
    trajectory = {"poses": [[*point, 0.] for point in world],
                  "time_s": np.linspace(0., 10., len(world)).tolist(),
                  "interpolation": "linear_xyz_yaw", "segments": [], "control_dt_s": .05}
    edits = []
    for index, edit in enumerate(manual_edits(z_floor), 100):
        local = (np.asarray(edit.mean_world_m) - origin) @ rotation.T
        edits.append({"role": edit.role, "gaussian_id": index,
                      "mean_route_m": local.tolist(),
                      "semiaxes_route_m": list(edit.semiaxes_route_m)})
    manifest = {"route_frame": {"origin_world_m": origin.tolist(),
                                 "world_to_route": rotation.tolist()},
                "manual_geometry": edits,
                "configuration": {"body_clearance_margin_m": .05},
                "existing_table_arrangement": {
                    "top_height_above_floor_m": .905, "count": 10,
                    "route_envelope_m": {"lower": [.65, -1.2, .15],
                                         "upper": [2.15, 1.2, 1.25]}}}
    evidence = ordered_uav_gate_evidence(
        trajectory, manifest, {"passed": True, "geometry": {"clearance_lower_m": .06}})
    assert evidence["all_pass"]
    assert evidence["high_crossing"]["merged_linear_segments"] >= 5
