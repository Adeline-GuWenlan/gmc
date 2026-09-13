import numpy as np
import pytest

from gmc.height.ply3d import GaussianScene3D
from gmc.height.prism import robot_table
from gmc.height.project import project_scene
from gmc.io.gs_io import validate_models

from ..conftest import make_cfg

Z_FLOOR = -1.0


def mini_scene():
    """Floor at z=-1: leg needle, tabletop slab, faint splat, far splat."""
    means = np.array([
        [0.0, 0.0, Z_FLOOR + 0.36],    # 0 leg, extent 0.0..0.72 above floor
        [0.0, 0.0, Z_FLOOR + 0.74],    # 1 tabletop, extent 0.72..0.76
        [0.5, 0.0, Z_FLOOR + 0.05],    # 2 faint (opacity 0.1)
        [9.0, 9.0, Z_FLOOR + 0.05],    # 3 far outside the window
    ])
    covs = np.array([
        np.diag([0.015 ** 2, 0.015 ** 2, 0.18 ** 2]),
        np.diag([0.15 ** 2, 0.15 ** 2, 0.01 ** 2]),
        np.eye(3) * 0.01 ** 2,
        np.eye(3) * 0.01 ** 2,
    ])
    return GaussianScene3D(means, covs, np.array([0.9, 0.9, 0.1, 0.9]),
                           np.array([10, 11, 12, 13]), "mini")


WINDOW = (-2.0, -2.0, 2.0, 2.0)


def ids(scene2d):
    return {s.primitive_id for s in scene2d.supports}


def test_robot_table_bands():
    t = robot_table(z_c=1.4)
    assert t["ellipse_toy"].is_2d
    assert t["sweeper"].band_abs(Z_FLOOR) == pytest.approx((-0.98, -0.90))
    assert t["uav"].band_abs(0.0) == pytest.approx((1.3, 1.5))
    assert t["cylinder"].max_radius() == pytest.approx(0.30)
    with pytest.raises(ValueError):
        t["ellipse_toy"].band_abs(0.0)


def test_sweeper_sees_leg_not_tabletop():
    s2, st = project_scene(mini_scene(), robot_table()["sweeper"], WINDOW,
                           z_floor=Z_FLOOR)
    assert ids(s2) == {10}
    assert st["dropped_opacity"] == 1 and st["excluded_band"] == 1
    assert st["outside_window"] == 1 and st["kept"] == 1


def test_cylinder_sees_leg_and_tabletop():
    s2, _ = project_scene(mini_scene(), robot_table()["cylinder"], WINDOW,
                          z_floor=Z_FLOOR)
    assert ids(s2) == {10, 11}


def test_uav_above_table_sees_nothing():
    s2, st = project_scene(mini_scene(), robot_table(z_c=1.20)["uav"], WINDOW,
                           z_floor=Z_FLOOR)
    assert ids(s2) == set() and st["kept"] == 0


def test_lower_tau_only_adds_obstacles():
    hi, _ = project_scene(mini_scene(), robot_table()["sweeper"], WINDOW,
                          z_floor=Z_FLOOR, tau=0.3)
    lo, _ = project_scene(mini_scene(), robot_table()["sweeper"], WINDOW,
                          z_floor=Z_FLOOR, tau=0.05)
    assert ids(hi) < ids(lo) and 12 in ids(lo)


def test_stacked_identical_shadows_are_kept_once():
    base = mini_scene()
    means = np.vstack([base.means[:1], base.means[:1] + [0, 0, 0.1]])
    covs = np.stack([base.covs[0], base.covs[0]])
    s3 = GaussianScene3D(means, covs, np.array([0.9, 0.9]), np.array([5, 6]), "stack")
    s2, st = project_scene(s3, robot_table()["cylinder"], WINDOW, z_floor=Z_FLOOR)
    assert ids(s2) == {5} and st["deduplicated"] == 1


def test_projected_maps_pass_gmc_input_validation():
    cfg = make_cfg()
    for key in ("sweeper", "cylinder", "quadruped"):
        robot = robot_table()[key]
        s2, _ = project_scene(mini_scene(), robot, WINDOW, z_floor=Z_FLOOR)
        validate_models(s2, robot.footprint, cfg)
