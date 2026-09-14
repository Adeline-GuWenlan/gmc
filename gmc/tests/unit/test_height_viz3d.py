import imageio_ffmpeg
import numpy as np
import pytest
from matplotlib.image import imread

from gmc.height.prism import robot_table
from gmc.height.viz3d import (SH_C0, load_dc_colors, overview_png, ply_vertex_layout, pointcloud_video,
                              weighted_subsample)


def write_ply(path, props, arr, comments=("comment made by test",), trailer=""):
    header = ("ply\nformat binary_little_endian 1.0\n" + "".join(f"{c}\n" for c in comments)
              + f"element vertex {len(arr)}\n" + "".join(f"property {t} {n}\n" for t, n in props)
              + trailer + "end_header\n")
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        arr.tofile(f)


PROPS = [("float", "x"), ("float", "y"), ("float", "z"), ("uchar", "label"), ("float", "f_dc_0"),
         ("float", "f_dc_1"), ("float", "f_dc_2"), ("double", "extra"), ("float", "opacity"),
         ("float", "rot_3")]
DT = {"float": "<f4", "uchar": "u1", "double": "<f8"}


def colour_ply(tmp_path):
    f_dc = np.array([[0.0, 0.0, 0.0], [1.0, -1.0, 0.5], [10.0, -10.0, 0.0],
                     [-1.7724539, 1.7724539, 0.1], [2.0, 2.0, 2.0], [np.nan, 0.0, 0.0]])
    arr = np.zeros(len(f_dc), dtype=[(n, DT[t]) for t, n in PROPS])
    arr["x"] = np.arange(len(f_dc))
    arr["label"] = 200
    arr["extra"] = -1e300
    arr["opacity"] = 3.0
    for k in range(3):
        arr[f"f_dc_{k}"] = f_dc[:, k]
    p = tmp_path / "c.ply"
    write_ply(p, PROPS, arr, trailer="element face 0\nproperty list uchar int vertex_indices\n")
    return p, f_dc


def test_layout_reads_mixed_types_and_ignores_later_elements(tmp_path):
    p, f_dc = colour_ply(tmp_path)
    n, dtype, offset = ply_vertex_layout(p)
    assert n == len(f_dc) and dtype.itemsize == 9 * 4 - 4 + 1 + 8 and dtype.names[3] == "label"
    assert p.stat().st_size == offset + n * dtype.itemsize


def test_dc_colors_known_values_and_id_selection(tmp_path):
    p, f_dc = colour_ply(tmp_path)
    ids = [4, 1, 3, 1, 0, 2]
    rgb = load_dc_colors(p, ids)
    expected = np.clip(0.5 + SH_C0 * f_dc[ids].astype(np.float32).astype(float), 0, 1)
    assert rgb.shape == (6, 3) and rgb.dtype == np.float64
    np.testing.assert_allclose(rgb, expected, atol=1e-6)
    np.testing.assert_allclose(rgb[0], [1.0, 1.0, 1.0])                        # id 4: 0.5 + 2*C0 > 1, clipped
    np.testing.assert_allclose(rgb[2], [0.0, 1.0, 0.5 + 0.1 * SH_C0], atol=1e-6)  # clipped at both ends
    np.testing.assert_allclose(rgb[4], [0.5, 0.5, 0.5])                        # f_dc = 0 -> grey
    np.testing.assert_allclose(load_dc_colors(p, [5])[0], [0.5, 0.5, 0.5])   # NaN -> grey
    assert load_dc_colors(p, []).shape == (0, 3)


def test_dc_colors_rejects_bad_ids_and_missing_fields(tmp_path):
    p, _ = colour_ply(tmp_path)
    with pytest.raises(IndexError):
        load_dc_colors(p, [6])
    props = [("float", "x"), ("float", "f_dc_0")]
    q = tmp_path / "bad.ply"
    write_ply(q, props, np.zeros(2, dtype=[("x", "<f4"), ("f_dc_0", "<f4")]))
    with pytest.raises(ValueError, match="f_dc_1"):
        load_dc_colors(q, [0])


def test_weighted_subsample_excludes_zero_weight():
    w = np.array([0.0, 1.0, 0.0, 2.0, 5.0, 0.0])
    idx = weighted_subsample(w, 3, seed=1)
    assert idx.tolist() == [1, 3, 4]
    assert weighted_subsample(w, 10).tolist() == list(range(6))


def test_pointcloud_video_smoke(tmp_path):
    rng = np.random.default_rng(0)
    pts = rng.uniform([-1.0, -1.0, 0.0], [1.0, 1.0, 1.2], (3000, 3))
    cols = rng.uniform(0, 1, (3000, 3))
    poses = np.column_stack([np.linspace(-0.8, 0.8, 7), np.zeros(7), np.linspace(0, 1, 7)])
    robot = robot_table()["sweeper"]
    out = pointcloud_video(pts, cols, robot, poses, 0.0, tmp_path / "v", title="t\nsecond line",
                           window=(-1, -1, 1, 1), max_frames=6, dpi=40, workers=2, view_half=0.6,
                           start=(-0.8, 0.0), goal=(0.8, 0.0))
    assert out["video"].exists() and out["video"].stat().st_size > 0
    assert out["n_frames"] == 6
    assert imageio_ffmpeg.count_frames_and_secs(str(out["video"]))[0] == 6
    assert [p.name for p in out["frames"]] == ["v_f000.png", "v_f050.png", "v_f100.png"]
    img = imread(out["frames"][1])
    assert img.shape[0] % 2 == 0 and img.shape[1] % 2 == 0
    png = overview_png(pts, cols, robot, poses, 0.0, tmp_path / "o.png", title="o", window=(-1, -1, 1, 1),
                       dpi=40)
    assert imread(png).ndim == 3
