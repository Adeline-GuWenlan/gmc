import copy

import numpy as np
import pytest

from gmc.gs3d.contracts import GoalRegion, Pose3
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.smoothing import (SmoothingConfig, optimize_trajectory,
                                verify_piecewise_bezier)
from gs3d_core_fixtures import CYLINDER, UAV, FlatSupport, make_scene


UAV_LIMITS = {"max_speed_mps": .5, "max_vertical_speed_mps": .3,
              "max_yaw_rate_radps": 1., "max_acceleration_mps2": .5,
              "max_yaw_acceleration_radps2": 1.}
GROUND_LIMITS = {"max_speed_mps": .3, "max_vertical_speed_mps": 0.,
                 "max_yaw_rate_radps": 1., "max_acceleration_mps2": .3,
                 "max_yaw_acceleration_radps2": 1.}


def linear(rows, step_s=4.):
    return {"poses": rows, "time_s": (np.arange(len(rows)) * step_s).tolist(),
            "interpolation": "linear_xyz_yaw", "segments": [], "control_dt_s": .05}


def test_uav_shortcut_spline_reduces_turn_and_certifies_continuous_dynamics():
    scene = make_scene(lower=(-3., -3., 0.), upper=(3., 3., 3.))
    oracle = GaussianBodyOracle(PreparedScene(scene))
    raw = linear([[-1.5, -1., 1., 0.], [-.5, .5, 1., 0.],
                  [.5, -.5, 1., 0.], [1.5, 1., 1., 0.]], step_s=5.)
    result = optimize_trajectory(raw, oracle, UAV, UAV_LIMITS, margin_m=.05,
                                 goal=GoalRegion(Pose3((1.5, 1., 1.))))
    assert result["selected"] == "smoothed"
    assert result["raw_fallback"]["verified"]
    assert result["metrics"]["turn_improvement_fraction"] > .9
    verification = result["candidate_verification"]
    assert verification["passed"] and verification["safety"] == "continuous_bound"
    assert all(verification["limit_checks"].values())
    assert verification["dynamics_bounds"]["max_acceleration_mps2"] <= .5


def test_protected_uav_positions_prevent_low_rise_diagonal_rounding():
    scene = make_scene(lower=(-2., -2., 0.), upper=(3., 3., 3.))
    oracle = GaussianBodyOracle(PreparedScene(scene))
    raw = linear([[-1., 0., .6, 0.], [0., 0., .6, 0.], [0., 0., 1.4, 0.],
                  [.5, .4, 1.4, 0.], [1., 0., 1.4, 0.]], step_s=4.)
    result = optimize_trajectory(raw, oracle, UAV, UAV_LIMITS, margin_m=.05,
                                 goal=GoalRegion(Pose3((1., 0., 1.4))),
                                 protected_position_indices=(1, 2))
    candidate = result["candidate_trajectory"]
    controls = [np.asarray(s["control_points_xyz"])
                for s in candidate["segments"] if s["type"] == "cubic_bezier_ease5"]
    assert np.allclose(controls[0][:, 2], .6)
    assert np.allclose(controls[1][:, :2], [0., 0.])
    assert np.allclose(controls[1][[0, -1], 2], [.6, 1.4])


def test_ground_output_uses_tangent_yaw_support_and_bounded_rotation():
    scene = make_scene(lower=(-3., -3., -.2), upper=(3., 3., 2.), support=FlatSupport())
    oracle = GaussianBodyOracle(PreparedScene(scene))
    z = CYLINDER.half_height_m + CYLINDER.ground_clearance_m
    raw = linear([[-1., -1., z, 0.], [-1., -1., z, np.pi / 2],
                  [-1., 1., z, np.pi / 2], [-1., 1., z, 0.],
                  [1., 1., z, 0.]], step_s=8.)
    result = optimize_trajectory(raw, oracle, CYLINDER, GROUND_LIMITS, margin_m=.001,
                                 goal=GoalRegion(Pose3((1., 1., z), 0.)))
    assert result["selected"] == "smoothed"
    assert result["candidate_verification"]["passed"]
    translations = [s for s in result["candidate_trajectory"]["segments"]
                    if s["type"] == "cubic_bezier_ease5"]
    assert translations and all(s["yaw_profile"] == "tangent_of_xyz_bezier"
                                for s in translations)
    assert np.ptp(np.asarray(result["candidate_trajectory"]["poses"])[:, 2]) == pytest.approx(0.)


def test_unqualified_candidate_retains_verified_raw_fallback():
    scene = make_scene([(0., 0., 1.)], [(.25, .25, .25)],
                       lower=(-3., -3., 0.), upper=(3., 3., 3.))
    oracle = GaussianBodyOracle(PreparedScene(scene))
    raw = linear([[-1.5, 0., 1., 0.], [-1., 1., 1., 0.],
                  [1., 1., 1., 0.], [1.5, 0., 1., 0.]], step_s=6.)
    robust = optimize_trajectory(
        raw, oracle, UAV, UAV_LIMITS, margin_m=.05,
        goal=GoalRegion(Pose3((1.5, 0., 1.))),
        config=SmoothingConfig(max_certificate_depth=0, certificate_deviation_m=10.))
    assert robust["selected"] == "smoothed"
    assert robust["optimization"]["spline_tension"] == 0.
    assert len(robust["optimization"]["rejected_tensions"]) == 4
    result = optimize_trajectory(
        raw, oracle, UAV, UAV_LIMITS, margin_m=.05,
        goal=GoalRegion(Pose3((1.5, 0., 1.))),
        config=SmoothingConfig(max_certificate_depth=0, certificate_deviation_m=10.,
                               min_turn_improvement_fraction=.99))
    assert result["success"] and result["selected"] == "raw_verified_fallback"
    assert result["raw_fallback"]["verified"]
    assert result["trajectory"] == raw


def test_independent_verifier_recomputes_dynamics_and_replay():
    scene = make_scene(lower=(-3., -3., 0.), upper=(3., 3., 3.))
    oracle = GaussianBodyOracle(PreparedScene(scene))
    raw = linear([[-1., 0., 1., 0.], [0., 1., 1., 0.], [1., 0., 1., 0.]], 5.)
    result = optimize_trajectory(raw, oracle, UAV, UAV_LIMITS, margin_m=.05,
                                 goal=GoalRegion(Pose3((1., 0., 1.))))
    candidate = copy.deepcopy(result["candidate_trajectory"])
    candidate["segments"][0]["dynamics_bounds"] = {key: 0. for key in
                                                     candidate["segments"][0]["dynamics_bounds"]}
    checked = verify_piecewise_bezier(candidate, oracle, UAV, UAV_LIMITS,
                                      margin_m=.05, goal=GoalRegion(Pose3((1., 0., 1.))),
                                      config=SmoothingConfig())
    assert checked["passed"]
    assert checked["dynamics_bounds"]["max_acceleration_mps2"] > 0.
    tight_limits = dict(UAV_LIMITS, max_acceleration_mps2=.001)
    checked = verify_piecewise_bezier(candidate, oracle, UAV, tight_limits,
                                      margin_m=.05, goal=GoalRegion(Pose3((1., 0., 1.))),
                                      config=SmoothingConfig())
    assert not checked["passed"]
    assert not checked["limit_checks"]["acceleration"]
    candidate["segments"][0]["duration_s"] *= .5
    checked = verify_piecewise_bezier(candidate, oracle, UAV, UAV_LIMITS,
                                      margin_m=.05, goal=GoalRegion(Pose3((1., 0., 1.))),
                                      config=SmoothingConfig())
    assert not checked["passed"]
    assert "replay differs" in checked["reason"]
