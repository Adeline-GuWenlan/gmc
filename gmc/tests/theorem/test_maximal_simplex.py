"""T-Nerve-01/02 (Guide §14.4): the nerve must store maximal simplices and
must not fabricate them."""
import numpy as np

from gmc.io.robot_io import ellipse_robot
from gmc.spatial.slice_compiler import build_slice
from gmc.synth import quad_overlap, triple_contact


class TestNerve:
    def test_quad_common_intersection_stored(self, cfg):
        scene = quad_overlap()
        robot = ellipse_robot(0.2, 0.15)
        sl = build_slice(scene, robot, 0.0, cfg, build_nerve=True)
        assert frozenset({0, 1, 2, 3}) in sl.nerve.simplices
        assert all(len(s) < 4 or s == frozenset({0, 1, 2, 3})
                   for s in sl.nerve.simplices)

    def test_pairwise_only_no_triangle(self, cfg):
        """Three C-obstacles pairwise intersecting but with empty triple
        intersection: nerve keeps edges, no 2-simplex (T-Nerve-01)."""
        robot = ellipse_robot(0.15, 0.15)
        # disc r=0.5 + robot 0.15 -> effective radius 0.65
        # pairwise overlap needs sep*sqrt(3) < 1.30 -> sep < 0.7506
        # triple common needs circumradius sep < 0.65
        scene = triple_contact(sep=0.72, r=0.5)
        sl = build_slice(scene, robot, 0.0, cfg, build_nerve=True)
        assert len(sl.nerve.edges) == 3
        assert all(len(s) == 2 for s in sl.nerve.simplices), \
            f"unexpected simplices {sl.nerve.simplices}"

    def test_triple_common_intersection(self, cfg):
        robot = ellipse_robot(0.15, 0.15)
        scene = triple_contact(sep=0.6, r=0.5)   # circumradius 0.6 < 0.65
        sl = build_slice(scene, robot, 0.0, cfg, build_nerve=True)
        assert frozenset({0, 1, 2}) in sl.nerve.simplices
