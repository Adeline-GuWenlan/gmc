"""Regression tests for M0 schema/configuration fail-closed boundaries."""

import numpy as np
import pytest
import yaml
from shapely.geometry import Polygon, box

from gmc.config import (GeometryCfg, LoggingCfg, OrientationCfg,
                        PairApproxCfg, QueryCfg, load_config)
from gmc.io.gs_io import load_scene, validate_models
from gmc.io.robot_io import ellipse_robot, load_robot
from gmc.types import (GaussianSupport2D, PairID, Pose2, RobotModel2D,
                       SceneModel2D)
from gmc.verification.continuous import (clearance_lb,
                                         rotation_interval_safe,
                                         translation_safe)


def _support(covariance=None, *, level=1.0, primitive_id=0):
    covariance = np.eye(2) if covariance is None else covariance
    return GaussianSupport2D(np.zeros(2), covariance, level, primitive_id)


@pytest.mark.parametrize(
    ("mean", "covariance", "level", "primitive_id"),
    [
        ([0.0], np.eye(2), 1.0, 0),
        ([0.0, np.nan], np.eye(2), 1.0, 0),
        ([0.0, 0.0], np.ones((2, 3)), 1.0, 0),
        ([0.0, 0.0], [[1.0, 0.2], [0.0, 1.0]], 1.0, 0),
        ([0.0, 0.0], [[1.0, 0.0], [0.0, 0.0]], 1.0, 0),
        ([0.0, 0.0], [[1.0, 0.0], [0.0, -1.0]], 1.0, 0),
        ([0.0, 0.0], np.eye(2), 0.0, 0),
        ([0.0, 0.0], np.eye(2), np.nan, 0),
        ([0.0, 0.0], np.eye(2), "1", 0),
        ([0.0, 0.0], np.eye(2), 1.0, 1.2),
        ([0.0, 0.0], np.eye(2), 1.0, "1"),
    ],
)
def test_invalid_supports_are_rejected(mean, covariance, level, primitive_id):
    with pytest.raises(ValueError):
        GaussianSupport2D(mean, covariance, level, primitive_id)


def test_support_and_pose_arrays_are_copied_and_immutable():
    mean = np.array([1.0, 2.0])
    covariance = np.eye(2)
    support = GaussianSupport2D(mean, covariance, 1.0, 3)
    xy = np.array([4.0, 5.0])
    pose = Pose2(xy, 0.3)

    mean[0] = covariance[0, 0] = xy[0] = 99.0
    assert np.array_equal(support.mean, [1.0, 2.0])
    assert np.array_equal(support.covariance, np.eye(2))
    assert np.array_equal(pose.xy, [4.0, 5.0])
    with pytest.raises(ValueError):
        support.mean[0] = 0.0
    with pytest.raises(ValueError):
        pose.xy[0] = 0.0


@pytest.mark.parametrize("xy, theta", [([0.0], 0.0), ([0.0, np.inf], 0.0),
                                        ([0.0, 0.0], np.nan)])
def test_invalid_poses_are_rejected(xy, theta):
    with pytest.raises(ValueError):
        Pose2(xy, theta)


def test_robot_and_workspace_invariants_are_enforced():
    with pytest.raises(ValueError, match="at least one"):
        RobotModel2D(())
    with pytest.raises(ValueError, match="workspace"):
        SceneModel2D((), Polygon())
    bow_tie = Polygon([(0, 0), (1, 1), (0, 1), (1, 0), (0, 0)])
    with pytest.raises(ValueError, match="workspace"):
        SceneModel2D((), bow_tie)


def _raw_config():
    return {
        "units": {"length": "meter"},
        "geometry": {
            "workspace_precision": 1e-8,
            "min_cov_eigenvalue": 1e-10,
            "support_level_scene": 2.0,
            "support_level_robot": 1.0,
        },
        "pair_approx": {
            "initial_directions": 16,
            "max_directions": 128,
            "eps_pair": 1e-3,
            "certificate_mode": "theorem",
        },
        "orientation": {
            "initial_intervals": 8,
            "theta_min": 1e-3,
            "max_depth": 12,
        },
        "query": {
            "max_support_calls": 100,
            "max_wall_seconds": 1.0,
            "eps_clear": 0.0,
        },
        "logging": {
            "save_intermediate_geometry": False,
            "save_failed_cases": True,
        },
    }


@pytest.mark.parametrize(
    "factory",
    [
        lambda: GeometryCfg(0.0, 0.0, 2.0, 1.0),
        lambda: GeometryCfg(1e-8, -1.0, 2.0, 1.0),
        lambda: PairApproxCfg(2, 16, 1e-3, "theorem"),
        lambda: PairApproxCfg(16, 8, 1e-3, "theorem"),
        lambda: PairApproxCfg(16, 16, 0.0, "theorem"),
        lambda: PairApproxCfg(16, 16, 1e-3, "claimed"),
        lambda: OrientationCfg(0, 1e-3, 12),
        lambda: OrientationCfg(8, 0.0, 12),
        lambda: OrientationCfg(8, 1e-3, -1),
        lambda: QueryCfg(-1, 1.0, 0.0),
        lambda: QueryCfg(1, -1.0, 0.0),
        lambda: QueryCfg(1, 1.0, -1e-3),
        lambda: LoggingCfg("false", True),
    ],
)
def test_invalid_direct_config_values_are_rejected(factory):
    with pytest.raises(ValueError):
        factory()


def test_yaml_boolean_is_not_coerced_from_a_string(tmp_path):
    raw = _raw_config()
    raw["logging"]["save_intermediate_geometry"] = "false"
    path = tmp_path / "bad_cfg.yaml"
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match="YAML boolean"):
        load_config(path)


@pytest.mark.parametrize("kind, primitive_id", [("scene", 1.2),
                                                 ("robot", "1")])
def test_loaders_do_not_coerce_primitive_ids(tmp_path, kind, primitive_id):
    support = {
        "primitive_id": primitive_id,
        "mean": [0.0, 0.0],
        "covariance": [[1.0, 0.0], [0.0, 1.0]],
        "level": 1.0,
    }
    if kind == "scene":
        raw = {
            "name": "bad",
            "workspace": {
                "exterior": [[-1, -1], [1, -1], [1, 1], [-1, 1],
                             [-1, -1]],
                "holes": [],
            },
            "supports": [support],
        }
        loader = load_scene
    else:
        raw = {"name": "bad", "supports": [support]}
        loader = load_robot
    path = tmp_path / f"{kind}.yaml"
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match="primitive_id"):
        loader(path)


def test_configured_covariance_floor_is_enforced():
    scene = SceneModel2D((_support(np.diag([1e-9, 1.0])),), box(-2, -2, 2, 2))
    robot = ellipse_robot(0.5, 0.2)

    class Geometry:
        min_cov_eigenvalue = 1e-8

    class Cfg:
        geometry = Geometry()

    with pytest.raises(ValueError, match="scene primitive IDs"):
        validate_models(scene, robot, Cfg())


class _NanOracle:
    pair_id = PairID(0, 0)

    def center(self, theta):
        return np.zeros(2)

    def support_values(self, theta, directions):
        return np.full(len(directions), np.nan)

    def theta_lipschitz(self):
        return 0.0


def test_numerical_nan_fails_closed_in_continuous_checks():
    oracle = _NanOracle()
    assert clearance_lb([oracle], np.zeros(2), 0.0) == -np.inf
    ok_t, lb_t = translation_safe(
        [oracle], np.zeros(2), np.ones(2), 0.0)
    ok_r, lb_r = rotation_interval_safe(
        [oracle], np.zeros(2), 0.0, 0.1)
    assert not ok_t and lb_t == -np.inf
    assert not ok_r and lb_r == -np.inf


def test_negative_clearance_floor_is_illegal():
    with pytest.raises(ValueError, match="floor"):
        translation_safe([], np.zeros(2), np.ones(2), 0.0, floor=-1.0)
    with pytest.raises(ValueError, match="floor"):
        rotation_interval_safe([], np.zeros(2), 0.0, 1.0, floor=-1.0)
