"""Amendment 3 P3: the long-range sizing and ladder helpers (gmc.height.longrange).

The support count is the number every P3 window is sized by before a job is submitted, so it is pinned
against ``project_scene``'s own ``kept`` count on the synthetic table scene, window by window and robot by
robot. If it drifts, P3 would be sizing windows on a number GMC never sees.
"""
import numpy as np
import pytest

from gmc.height.longrange import (count_supports, density_grid, ladder_goals, min_pool_dist,
                                  route_window, shadow_table)
from gmc.height.prism import robot_table
from gmc.height.project import project_scene
from gmc.height.synth3d import WORKSPACE, Z_FLOOR, table_scene


@pytest.fixture(scope="module")
def scene():
    return table_scene("open")[0]


@pytest.mark.parametrize("robot", ["sweeper", "cylinder", "uav"])
@pytest.mark.parametrize("win", [list(WORKSPACE), [-3.0, -2.0, 0.0, 2.0], [-0.5, -0.8, 0.5, 0.8],
                                 [1.0, 1.3, 3.0, 2.0], [0.05, -2.0, 3.0, 2.0], [0.4, -0.2, 2.0, 0.2]])
def test_support_count_is_project_scenes_kept_count(scene, robot, win):
    rb = robot_table(1.2)[robot]
    tab = shadow_table(scene, rb, z_floor=Z_FLOOR)
    _, stats = project_scene(scene, rb, win, z_floor=Z_FLOOR)
    assert count_supports(tab, win) == stats["kept"]


def test_support_count_is_monotone_in_the_window(scene):
    tab = shadow_table(scene, robot_table(1.2)["cylinder"], z_floor=Z_FLOOR)
    small, big = [-0.5, -0.5, 0.5, 0.5], [-1.5, -1.5, 1.5, 1.5]
    assert count_supports(tab, small) <= count_supports(tab, big) <= tab["n"]


def test_density_grid_counts_every_support_once(scene):
    tab = shadow_table(scene, robot_table(1.2)["cylinder"], z_floor=Z_FLOOR)
    grid, ext = density_grid(tab, [-4.0, -3.0, 4.0, 3.0], 0.5)
    assert grid.shape == (16, 12)
    assert np.isclose(grid.sum() * 0.25, tab["n"])          # per-m2 density times cell area
    assert ext == [-4.0, -3.0, 4.0, 3.0]


def test_route_window_is_the_bbox_of_everything_plus_margin():
    pts = [np.array([[0.0, 0.0], [8.0, 1.0]]), np.array([[2.0, -1.5], [3.0, 2.0]])]
    assert route_window(pts, 0.5, 12.0) == [-0.5, -2.0, 8.5, 2.5]


def test_route_window_rejects_a_side_over_the_cap_and_clips_to_bounds():
    pts = [np.array([[0.0, 0.0], [11.5, 1.0]])]
    assert route_window(pts, 0.5, 12.0) is None                       # 12.5 m side
    assert route_window(pts, 0.5, 12.0, bounds=[-0.2, -5, 11.7, 5]) == [-0.2, -0.5, 11.7, 1.5]


def test_min_pool_dist_never_reports_more_clearance_than_the_fine_map():
    rng = np.random.default_rng(0)
    fine = rng.uniform(0, 1, (37, 41))
    coarse = min_pool_dist(fine, 4)
    assert coarse.shape == (10, 11)
    for i in range(10):
        for j in range(11):
            blk = fine[4 * i:4 * i + 4, 4 * j:4 * j + 4]
            full = blk.shape == (4, 4)
            assert coarse[i, j] == (blk.min() if full else 0.0)     # partial edge blocks count as occupied


def test_ladder_goals_step_up_in_separation_and_stay_near_the_line():
    anchor = np.array([0.0, 0.0])
    xs, ys = np.meshgrid(np.arange(-2, 12.01, 0.25), np.arange(-3, 3.01, 0.25), indexing="ij")
    cands = np.stack([xs.ravel(), ys.ravel()], axis=1)
    rungs = ladder_goals(anchor, cands, [2.5, 4.0, 5.5, 8.0], toward=(10.0, 0.0))
    assert [r["target_m"] for r in rungs] == [2.5, 4.0, 5.5, 8.0]
    for r in rungs:
        g = np.array(r["goal"])
        assert abs(np.hypot(*g) - r["target_m"]) <= 0.2
        assert abs(g[1]) <= 0.25 and g[0] > 0                          # on the ray towards (10, 0)


def test_ladder_goals_reports_a_target_it_cannot_reach():
    rungs = ladder_goals(np.zeros(2), np.array([[1.0, 0.0], [2.5, 0.0]]), [2.5, 6.0], toward=(9, 0))
    assert rungs[0]["goal"] == [2.5, 0.0]
    assert rungs[1]["goal"] is None
