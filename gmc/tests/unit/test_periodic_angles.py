"""Periodicity conventions, invariant I6 (Guide §1.3, §8)."""
import numpy as np

from gmc.orientation.intervals import TWO_PI, initial_partition
from gmc.types import wrap_angle


class TestPartition:
    def test_covers_period_without_gaps(self):
        for n in (4, 8, 16):
            ivs = initial_partition(n)
            assert len(ivs) == n
            cursor = 0.0
            for iv in ivs:
                assert abs(iv.lo - cursor) < 1e-12
                cursor = iv.hi
            assert abs(cursor - TWO_PI) < 1e-12

    def test_halves(self):
        iv = initial_partition(4)[1]
        assert iv.left_half.hi == iv.right_half.lo == iv.midpoint


class TestWrap:
    def test_wrap_angle(self):
        assert wrap_angle(TWO_PI) == 0.0
        assert abs(wrap_angle(-0.1) - (TWO_PI - 0.1)) < 1e-12

    def test_adjacent_slab_pairs_periodic(self, cfg):
        from gmc.io.robot_io import ellipse_robot
        from gmc.orientation.slab_builder import (adjacent_slab_pairs,
                                                  build_slabs)
        from gmc.synth import single_obstacle
        scene = single_obstacle(0.5, workspace=(-2.0, 2.0, -1.5, 1.5))
        robot = ellipse_robot(0.4, 0.2)
        dec = build_slabs(scene, robot, cfg)
        pairs = adjacent_slab_pairs(dec.slabs, periodic=True)
        assert len(pairs) == len(dec.slabs)
        last, first = pairs[-1]
        assert first.interval.lo == 0.0
        assert abs(last.interval.hi - TWO_PI) < 1e-12
