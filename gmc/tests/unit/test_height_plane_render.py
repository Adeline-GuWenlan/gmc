"""Amendment 3 P2c: the renderer must draw the inserted plane-floor tiles, not choke on them.

``load_dc_colors`` indexes the source PLY by splat id. The P1b plane floor inserts analytic tiles whose
ids continue **past the end of that PLY** (first_inserted_id = 7,340,008), so a per-robot video of the
plane-floor scene would raise IndexError on the very geometry the edit added. ``percase_render.dc_colors``
splits the ids: PLY rows keep their DC colour, the inserted tiles get the height colour map, and the
count of the second group is reported so a manifest can never hide them.

Run from gmc/ with PYTHONPATH=src:experiments.
"""
import numpy as np
import pytest

from gmc.height.viz3d import SH_C0, load_dc_colors

percase_render = pytest.importorskip("percase_render")

PROPS = [("float", "x"), ("float", "y"), ("float", "z"),
         ("float", "f_dc_0"), ("float", "f_dc_1"), ("float", "f_dc_2")]
DT = {"float": "<f4"}


@pytest.fixture
def ply(tmp_path):
    f_dc = np.array([[0.0, 0.0, 0.0], [1.0, -1.0, 0.5], [2.0, 2.0, 2.0], [-1.0, 0.5, 0.0]])
    arr = np.zeros(len(f_dc), dtype=[(n, DT[t]) for t, n in PROPS])
    for k in range(3):
        arr[f"f_dc_{k}"] = f_dc[:, k]
    p = tmp_path / "c.ply"
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {len(arr)}\n"
              + "".join(f"property {t} {n}\n" for t, n in PROPS) + "end_header\n")
    with open(p, "wb") as f:
        f.write(header.encode("ascii"))
        arr.tofile(f)
    return p, f_dc


def test_ids_inside_the_ply_keep_exactly_the_dc_colour_load_dc_colors_gives(ply):
    path, _ = ply
    ids = np.array([3, 0, 2])
    rgb, n_missing = percase_render.dc_colors(path, ids)
    assert n_missing == 0
    assert np.allclose(rgb, load_dc_colors(path, ids))


def test_inserted_tiles_are_height_coloured_instead_of_raising_index_error(ply):
    """The regression this file exists for: ids past the PLY used to be an IndexError."""
    path, f_dc = ply
    ids = np.array([1, 7_340_008, 2, 9_999_999])
    z = np.array([0.0, -1.22, 1.0, -1.22])
    with pytest.raises(IndexError):
        load_dc_colors(path, ids)
    rgb, n_missing = percase_render.dc_colors(path, ids, z, z_floor=-1.2271749593107995)
    assert n_missing == 2
    assert rgb.shape == (4, 3) and np.isfinite(rgb).all()
    assert (rgb >= 0).all() and (rgb <= 1).all()
    assert np.allclose(rgb[[0, 2]], np.clip(0.5 + SH_C0 * f_dc[[1, 2]], 0, 1))
    assert np.allclose(rgb[1], rgb[3])            # same height -> same colour


def test_without_heights_the_unknown_ids_fall_back_to_grey(ply):
    path, _ = ply
    rgb, n_missing = percase_render.dc_colors(path, np.array([0, 7_340_008]))
    assert n_missing == 1 and np.allclose(rgb[1], 0.5)


def test_every_id_missing_is_handled_not_just_some(ply):
    path, _ = ply
    rgb, n_missing = percase_render.dc_colors(path, np.array([7_340_008, 7_340_009]),
                                              np.array([-1.0, 0.5]), z_floor=-1.2271749593107995)
    assert n_missing == 2 and rgb.shape == (2, 3) and np.isfinite(rgb).all()
