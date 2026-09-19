"""Amendment 3 P5: the camera-azimuth scan counts a frame as blocked only when a tall cell lies on the
kept stretch of its sight line (beyond the robot's own 0.5 m, short of the renderer's near-cull)."""
import numpy as np
import pytest

scan = pytest.importorskip("plane_camera_scan")

EXT, CELL = (0.0, 0.0, 20.0, 20.0), 0.5


def wall_at(x):
    d = np.zeros((40, 40))
    d[int(x / CELL), :] = 10_000
    return d


def test_a_wall_between_target_and_camera_blocks():
    assert scan.sight_blocked(wall_at(7.0), EXT, CELL, 5.0, 10.0, 0.0, dist=6.0)


def test_the_same_wall_behind_the_target_does_not():
    assert not scan.sight_blocked(wall_at(3.0), EXT, CELL, 5.0, 10.0, 0.0, dist=6.0)


def test_a_wall_inside_the_near_cull_is_ignored():
    # dist 6 at 35 deg: camera 4.91 m out horizontally, near-cull 1.92 m -> kept stretch ends near 3.0 m
    assert not scan.sight_blocked(wall_at(9.5), EXT, CELL, 5.0, 10.0, 0.0, dist=6.0)


def test_low_density_is_not_a_wall():
    d = np.full((40, 40), scan.BLOCK - 1.0)
    assert not scan.sight_blocked(d, EXT, CELL, 5.0, 10.0, 0.0, dist=6.0)
