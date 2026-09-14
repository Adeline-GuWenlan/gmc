import numpy as np

from gmc.height.floor import apply_floor_rule, floor_surface_mask
from gmc.height.ply3d import GaussianScene3D
from gmc.height.synth3d import table_scene

UP = np.array([0.0, 0.0, 1.0])
ORIGIN = np.zeros(3)
FLAT = (0.05, 0.05, 0.005)


def rot(axis, deg):
    t = np.deg2rad(deg)
    c, s = np.cos(t), np.sin(t)
    i, j = {"x": (1, 2), "y": (2, 0), "z": (0, 1)}[axis]
    R = np.eye(3)
    R[i, i], R[i, j], R[j, i], R[j, j] = c, -s, s, c
    return R


def cov(sigmas, R=np.eye(3)):
    return R @ np.diag(np.square(sigmas)) @ R.T


def make_scene(means, covs, opacity=0.9):
    means = np.atleast_2d(np.asarray(means, float))
    covs = np.asarray(covs, float).reshape(len(means), 3, 3)
    op = np.broadcast_to(np.asarray(opacity, float), (len(means),))
    return GaussianScene3D(means, covs, op, np.arange(len(means)), "floor_test")


def test_flat_floor_splats_with_small_tilt_are_removed():
    rng = np.random.default_rng(0)
    n = 200
    means = np.column_stack([rng.uniform(-1, 1, n), rng.uniform(-1, 1, n), rng.uniform(-0.01, 0.01, n)])
    covs = [cov(FLAT, rot("z", a) @ rot("x", t) @ rot("z", b))
            for a, t, b in zip(rng.uniform(0, 360, n), rng.uniform(0, 5, n), rng.uniform(0, 360, n))]
    assert floor_surface_mask(make_scene(means, covs), UP, ORIGIN).all()


def test_vertical_leg_needles_are_kept():
    rng = np.random.default_rng(1)
    means = np.column_stack([rng.uniform(-1, 1, 20), rng.uniform(-1, 1, 20), np.full(20, 0.04)])
    covs = [cov((0.01, 0.01, 0.1))] * 20
    assert not floor_surface_mask(make_scene(means, covs), UP, ORIGIN).any()


def test_isotropic_blob_near_floor_is_kept():
    s = make_scene([[0.3, -0.2, 0.02]], [cov((0.02, 0.02, 0.02))])
    assert not floor_surface_mask(s, UP, ORIGIN).any()


def test_flat_splat_above_offset_is_kept():
    s = make_scene([[0.0, 0.0, 0.20]], [cov(FLAT)])
    assert not floor_surface_mask(s, UP, ORIGIN).any()


def test_flat_splat_tilted_30_degrees_is_kept():
    s = make_scene([[0.0, 0.0, 0.0]], [cov(FLAT, rot("x", 30.0))])
    assert not floor_surface_mask(s, UP, ORIGIN).any()


def test_offset_is_measured_along_the_plane_normal():
    # Plane tilted 2 deg about y: n = (-sin, 0, cos), z = x tan(2 deg) through p0.
    tilt = 2.0
    n = np.array([-np.sin(np.deg2rad(tilt)), 0.0, np.cos(np.deg2rad(tilt))])
    p0 = np.array([0.3, -0.2, -1.2])
    R = rot("y", -tilt)                       # thinnest axis (local z) -> n
    np.testing.assert_allclose(R @ UP, n, atol=1e-12)
    tan = np.tan(np.deg2rad(tilt))

    xs = np.linspace(-1.0, 1.0, 21)
    follow = np.column_stack([xs, np.linspace(-0.5, 0.5, 21), xs * tan]) + p0
    assert np.abs(follow[:, 2] - p0[2]).max() > 0.034          # up to ~0.04 vertically at x = +-1
    far = np.array([[2.0, 0.0, 2.0 * tan], [-2.0, 0.0, -2.0 * tan]]) + p0   # vertical 0.07, normal 0
    s = make_scene(np.vstack([follow, far]), [cov(FLAT, R)] * 23)
    assert floor_surface_mask(s, n, p0).all()

    # Vertical offset 0.025 from p0, but 0.06 from the plane along n: not floor surface.
    off = make_scene([p0 + [-1.0, 0.0, 0.025]], [cov(FLAT, R)])
    assert abs((off.means[0] - p0) @ n) > 0.05
    assert not floor_surface_mask(off, n, p0).any()


def test_table_scene_loses_nothing():
    scene, _ = table_scene("closed")
    assert floor_surface_mask(scene, UP, ORIGIN).sum() == 0
    kept, stats = apply_floor_rule(scene, {"normal": UP.tolist(), "centroid": ORIGIN.tolist()})
    assert stats["removed"] == 0 and len(kept) == len(scene)


def test_chunked_mask_equals_unchunked():
    rng = np.random.default_rng(3)
    n = 1000
    means = np.column_stack([rng.uniform(-1, 1, n), rng.uniform(-1, 1, n), rng.uniform(-0.1, 0.1, n)])
    covs = [cov((0.05, rng.uniform(0.005, 0.05), rng.uniform(0.002, 0.05)),
                rot("z", rng.uniform(0, 360)) @ rot("x", rng.uniform(0, 40)))
            for _ in range(n)]
    s = make_scene(means, covs)
    full = floor_surface_mask(s, UP, ORIGIN)
    assert 0 < full.sum() < n
    np.testing.assert_array_equal(floor_surface_mask(s, UP, ORIGIN, chunk=7), full)


def test_apply_floor_rule_removes_floor_and_reports_stats():
    means = [[0, 0, 0.0], [1, 0, 0.01], [2, 0, -0.01], [0, 1, 0.04], [0, 2, 0.20]]
    covs = [cov(FLAT)] * 3 + [cov((0.01, 0.01, 0.1)), cov(FLAT)]
    s = make_scene(means, covs, opacity=[0.9, 0.1, 0.5, 0.9, 0.9])
    floor = {"normal": [0.0, 0.0, 2.0], "centroid": [0.0, 0.0, 0.0], "z_floor": 0.0}
    kept, stats = apply_floor_rule(s, floor)
    assert kept.ids.tolist() == [3, 4]
    assert stats["n_input"] == 5 and stats["removed"] == 3
    assert stats["removed_opaque_tau0.3"] == 2
    assert stats["params"] == {"max_offset": 0.05, "max_tilt_deg": 15.0, "max_flatness": 0.5}
