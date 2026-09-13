import numpy as np
import pytest

from gmc.height.ply3d import (GaussianScene3D, covariances_from_scale_rot,
                              crop_box, fit_floor, gravity_rotation,
                              load_3dgs_ply, rotate_scene)

PROPS = ["x", "y", "z", "nx", "ny", "nz", "f_dc_0", "f_dc_1", "f_dc_2",
         "opacity", "scale_0", "scale_1", "scale_2",
         "rot_0", "rot_1", "rot_2", "rot_3"]


def write_ply(path, rows):
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {len(rows)}\n"
              + "".join(f"property float {p}\n" for p in PROPS)
              + "end_header\n")
    arr = np.zeros(len(rows), dtype=[(p, "<f4") for p in PROPS])
    for i, r in enumerate(rows):
        for k, v in r.items():
            arr[i][k] = v
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        arr.tofile(f)


def test_load_decodes_opacity_scale_and_rotation(tmp_path):
    half = np.sqrt(0.5)
    rows = [
        dict(x=1, y=2, z=3, opacity=0.0, scale_0=np.log(0.1),
             scale_1=np.log(0.2), scale_2=np.log(0.3), rot_0=1.0),
        # 90 deg about z: local x axis -> world y axis
        dict(x=0, y=0, z=0, opacity=2.0, scale_0=np.log(0.4),
             scale_1=np.log(0.1), scale_2=np.log(0.1), rot_0=half, rot_3=half),
    ]
    p = tmp_path / "t.ply"
    write_ply(p, rows)
    scene, stats = load_3dgs_ply(p)
    assert stats["rows"] == 2 and stats["nonfinite_dropped"] == 0
    np.testing.assert_allclose(scene.means[0], [1, 2, 3], atol=1e-6)
    np.testing.assert_allclose(scene.opacity, [0.5, 1 / (1 + np.exp(-2.0))],
                               rtol=1e-6)
    np.testing.assert_allclose(scene.covs[0], np.diag([0.01, 0.04, 0.09]),
                               rtol=1e-5)
    np.testing.assert_allclose(scene.covs[1], np.diag([0.01, 0.16, 0.01]),
                               rtol=1e-5, atol=1e-7)
    assert scene.ids.tolist() == [0, 1]


def test_load_rejects_missing_field(tmp_path):
    p = tmp_path / "bad.ply"
    p.write_bytes(b"ply\nformat binary_little_endian 1.0\nelement vertex 0\n"
                  b"property float x\nend_header\n")
    with pytest.raises(ValueError, match="missing"):
        load_3dgs_ply(p)


def test_covariances_are_spd_and_normalise_quaternion():
    rng = np.random.default_rng(0)
    q = rng.normal(size=(50, 4)) * 3.0          # deliberately unnormalised
    s = rng.normal(size=(50, 3)) - 3.0
    C = covariances_from_scale_rot(s, q)
    assert np.all(np.linalg.eigvalsh(C) > 0)
    np.testing.assert_allclose(np.linalg.det(C), np.exp(2 * s.sum(axis=1)),
                               rtol=1e-8)


def _floor_scene(tilt_deg=1.0, z0=-1.0, seed=1):
    rng = np.random.default_rng(seed)
    n = 5000
    xy = rng.uniform(-5, 5, size=(n, 2))
    z = z0 + np.tan(np.deg2rad(tilt_deg)) * xy[:, 0] + rng.normal(0, 0.003, n)
    floor = np.column_stack([xy, z])
    wall = np.column_stack([np.full(1500, 5.0), rng.uniform(-5, 5, 1500),
                            rng.uniform(z0, z0 + 3, 1500)])
    floaters = np.column_stack([rng.uniform(-5, 5, (40, 2)),
                                rng.uniform(-12, -8, 40)])
    means = np.vstack([floor, wall, floaters])
    m = len(means)
    covs = np.tile(np.eye(3) * 1e-4, (m, 1, 1))
    return GaussianScene3D(means, covs, np.full(m, 0.9), np.arange(m), "floor")


def test_fit_floor_ignores_sparse_floaters_and_measures_tilt():
    info = fit_floor(_floor_scene())
    assert abs(info["z_floor"] - (-1.0)) < 0.01
    assert abs(info["tilt_deg"] - 1.0) < 0.2
    assert info["n_inliers"] > 4000


def test_gravity_rotation_levels_the_floor():
    scene = _floor_scene(tilt_deg=3.0)
    info = fit_floor(scene)
    R = gravity_rotation(np.asarray(info["normal"]))
    np.testing.assert_allclose(R @ np.asarray(info["normal"]), [0, 0, 1],
                               atol=1e-12)
    level = rotate_scene(scene, R, np.asarray(info["centroid"]))
    assert fit_floor(level)["tilt_deg"] < 0.2
    np.testing.assert_allclose(np.linalg.det(level.covs),
                               np.linalg.det(scene.covs), rtol=1e-9)


def test_crop_box_and_subset_keep_provenance():
    scene = _floor_scene()
    kept, dropped = crop_box(scene, np.array([-6, -6, -2]),
                             np.array([6, 6, 4]))
    assert dropped == 40 and len(kept) == len(scene) - 40
    assert kept.ids.max() < len(scene) - 40
    lo, hi = kept.aabb(2.0)
    np.testing.assert_allclose(hi - lo, 4 * np.sqrt(1e-4), rtol=1e-9)
