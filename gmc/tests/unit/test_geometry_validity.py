"""M0 acceptance and geometry conventions (Guide §4.3, §7.2)."""
import numpy as np

from gmc.geometry.predicates import (check_workspace, polygon_components,
                                     scale_lengths)
from gmc.io.gs_io import load_scene, save_scene, spd_check
from gmc.io.robot_io import ellipse_robot, load_robot, save_robot
from gmc.synth import single_door


class TestM0:
    def test_spd_check(self):
        scene = single_door(0.7)
        assert spd_check(scene.supports, 1e-12) == []

    def test_roundtrip(self, tmp_path):
        scene = single_door(0.6)
        save_scene(scene, tmp_path / "s.yaml")
        back = load_scene(tmp_path / "s.yaml")
        assert len(back.supports) == len(scene.supports)
        for a, b in zip(scene.supports, back.supports):
            assert np.allclose(a.mean, b.mean)
            assert np.allclose(a.covariance, b.covariance)
            assert a.level == b.level and a.primitive_id == b.primitive_id
        assert back.workspace.equals(scene.workspace)
        robot = ellipse_robot(0.6, 0.25)
        save_robot(robot, tmp_path / "r.yaml")
        rb = load_robot(tmp_path / "r.yaml")
        assert np.allclose(rb.supports[0].covariance,
                           robot.supports[0].covariance)

    def test_scale_equivariance(self):
        scene = single_door(0.7)
        scaled = scale_lengths(scene.supports, 2.0)
        for a, b in zip(scene.supports, scaled):
            assert np.allclose(b.mean, a.mean * 2.0)
            assert np.allclose(b.covariance, a.covariance * 4.0)
            assert b.bounding_radius() == a.bounding_radius() * 2.0

    def test_workspace_valid(self):
        assert check_workspace(single_door(0.7).workspace)


class TestComponents:
    def test_sliver_filter(self):
        import shapely
        big = shapely.box(0, 0, 1, 1)
        tiny = shapely.box(2, 2, 2 + 1e-6, 2 + 1e-6)
        parts = polygon_components(shapely.union_all([big, tiny]), 1e-9)
        assert len(parts) == 1
