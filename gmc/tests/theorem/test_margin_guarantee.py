"""Proposition 1 evidence (Guide §4.4 of the manual): the outer envelope
never produces false-safe — every point of F_safe is truly collision-free
(checked against the independent oracle)."""
import numpy as np

from gmc.geometry.c_obstacle import scene_pose_collides
from gmc.io.robot_io import ellipse_robot
from gmc.spatial.slice_compiler import build_slice
from gmc.synth import single_door

from ..conftest import SMALL_WS

RNG = np.random.default_rng(11)


class TestMarginGuarantee:
    def test_f_safe_has_no_false_safe(self, cfg):
        scene = single_door(0.6, workspace=SMALL_WS)
        robot = ellipse_robot(0.5, 0.2)
        th = 0.15
        sl = build_slice(scene, robot, th, cfg)
        checked = 0
        for comp in sl.D_safe:
            minx, miny, maxx, maxy = comp.geometry.bounds
            while checked < 60:
                p = RNG.uniform([minx, miny], [maxx, maxy])
                import shapely
                if not comp.geometry.covers(shapely.Point(p)):
                    continue
                assert not scene_pose_collides(scene, robot, p, th), \
                    f"false-safe at {p}"
                checked += 1
            break
        assert checked == 60
