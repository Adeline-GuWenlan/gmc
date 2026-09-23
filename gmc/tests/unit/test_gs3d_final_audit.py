"""Adversarial checks for gaps found in the independent A7 review."""
import copy

import numpy as np
import pytest

from gmc.gs3d.contracts import GoalRegion, Pose3
from gmc.gs3d.integration import _bezier_crossing_interval
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.smoothing import (SmoothingConfig, _curve_domain, _sample_segments,
                                optimize_trajectory, verify_piecewise_bezier)
from gmc.gs3d.timing import PlanTiming
from gs3d_core_fixtures import UAV, CYLINDER, make_scene


LIMITS = {"max_speed_mps": .5, "max_vertical_speed_mps": .3,
          "max_yaw_rate_radps": 1., "max_acceleration_mps2": .5,
          "max_yaw_acceleration_radps2": 1.}


def test_smooth_verifier_rejects_hidden_yaw_jump_with_matching_replay():
    oracle = GaussianBodyOracle(PreparedScene(make_scene(
        lower=(-3., -3., 0.), upper=(3., 3., 3.))))
    raw = {"poses": [[-1., 0., 1., 0.], [0., 0., 1., 0.], [1., 0., 1., 0.]],
           "time_s": [0., 5., 10.], "control_dt_s": .05,
           "interpolation": "linear_xyz_yaw", "segments": []}
    result = optimize_trajectory(raw, oracle, UAV, LIMITS, margin_m=.05,
                                 goal=GoalRegion(Pose3((1., 0., 1.))),
                                 protected_position_indices=(1,))
    assert result["candidate_verification"]["passed"]
    candidate = copy.deepcopy(result["candidate_trajectory"])
    candidate["segments"][0]["yaw_start_rad"] = .5
    candidate["segments"][0]["yaw_end_rad"] = .5
    rows, times = _sample_segments(candidate["segments"], .05)
    candidate.update(poses=rows.tolist(), time_s=times.tolist())
    checked = verify_piecewise_bezier(candidate, oracle, UAV, LIMITS, margin_m=.05,
                                      goal=GoalRegion(Pose3((1., 0., 1.))),
                                      config=SmoothingConfig())
    assert not checked["passed"]
    assert checked["reason"] == "smooth_segment_yaw_discontinuity"


def test_smooth_verifier_rejects_nonfinite_limits():
    oracle = GaussianBodyOracle(PreparedScene(make_scene()))
    candidate = {"interpolation": "piecewise_bezier", "segments": [{}], "control_dt_s": .05}
    checked = verify_piecewise_bezier(candidate, oracle, UAV,
                                      dict(LIMITS, max_speed_mps=np.inf), margin_m=.05,
                                      goal=GoalRegion(Pose3((0., 0., 1.))), config=SmoothingConfig())
    assert not checked["passed"] and "invalid smooth limits" in checked["reason"]


def test_control_heights_do_not_prove_nonlinear_ground_manifold():
    class WavySupport:
        max_travel_m = .05

        def height(self, x, y):
            return .01 * np.sin(3 * np.pi * x)

        def height_bounds(self, lower, upper):
            return -.01, .01

    support = WavySupport()
    scene = make_scene(lower=(-2., -2., -1.), upper=(3., 2., 3.), support=support)
    control = np.c_[np.linspace(0., 1., 4), np.zeros(4),
                    np.full(4, CYLINDER.half_height_m + CYLINDER.ground_clearance_m)]
    assert np.allclose([support.height(x, 0.) for x in control[:, 0]], 0.)
    valid, reason = _curve_domain(control, PreparedScene(scene), CYLINDER, .001)
    assert not valid and reason == "curve_nonaffine_support_unproven"


def test_ordered_bezier_gate_uses_controls_not_favorable_endpoint_samples():
    control = np.c_[np.linspace(-.25, .5, 4), np.zeros(4), np.full(4, .65)]
    trajectory = {"segments": [{"type": "cubic_bezier_ease5", "duration_s": 2.,
                                 "control_points_xyz": control.tolist()}]}
    def check():
        return _bezier_crossing_interval(trajectory, np.zeros(3), np.eye(3), (-.2, .2),
                                         expected_z=.65, lateral_bounds_m=(-.2, .2))
    assert check()["horizontal_span_m"] == pytest.approx(.4)
    # Same endpoint rows would look correct, but the true curve climbs midway.
    trajectory["segments"][0]["control_points_xyz"][1][2] = 1.5
    assert check() is None


def test_uav_preparation_includes_crop_and_index_before_algorithm(monkeypatch):
    import gs3d_integration as runner
    clock = [0.]
    timer = PlanTiming(clock=lambda: clock[0])
    with timer.preparation():
        clock[0] += 3.  # scene load

    def prepare(full, document, timer):
        clock[0] += 4.  # scene crop and index
        return None, None

    class ReachedPlanner(Exception):
        pass

    def plan(*args, preparation_wall_s, **kwargs):
        assert preparation_wall_s == 7.
        raise ReachedPlanner

    monkeypatch.setattr(runner, "_uav_scene", prepare)
    monkeypatch.setattr(runner, "_plan_uav_mission", plan)
    with pytest.raises(ReachedPlanner):
        runner._run_uav(None, {"configuration": {"candidate_path_world_m": []}}, timer,
                        warm_calls=5)


def test_rendered_body_keyframes_follow_actual_lateral_path():
    from gs3d_render import _trajectory_keyframes
    trajectory = {"poses": [[-.25, .12, .65, 0.], [.5, .12, .65, 0.],
                             [.5, -.15, 1.4, 0.], [2., -.15, 1.4, 0.]],
                  "time_s": [0., 2., 5., 9.], "control_dt_s": .05,
                  "interpolation": "linear_xyz_yaw", "segments": []}
    frames, evidence = _trajectory_keyframes(trajectory, np.zeros(3), np.eye(3))
    assert frames[0][1][1] == pytest.approx(.12)
    assert frames[1][1][1] == pytest.approx(-.15)
    assert evidence[0]["time_s"] < evidence[1]["time_s"]
