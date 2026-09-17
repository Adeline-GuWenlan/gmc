"""Amendment 3 P1: the phantom-cell test and the plane-floor replacement.

The plane floor is a user-approved MANUAL scene-definition change, not a
reconstruction method; these tests pin the edit's mechanics, not its realism.
"""
import numpy as np
import pytest

from gmc.height.planefloor import (build_plane_floor, footprint_cell_span,
                                   load_plane_scene, monotone_toward_floor,
                                   open_floor_cell_mask, phantom_cell_mask,
                                   plane_tiles, replace_mask, save_plane_scene,
                                   speckle_stats)
from gmc.height.ply3d import GaussianScene3D


# ------------------------------------------------------------------ P1a --


def test_phantom_is_low_occupancy_with_nothing_above():
    low = np.array([[1, 1, 0], [1, 0, 0], [0, 0, 0]], dtype=bool)
    b1 = np.array([[1, 0, 0], [0, 0, 0], [0, 0, 0]], dtype=bool)
    b2 = np.zeros((3, 3), dtype=bool)
    got = phantom_cell_mask(low, [b1, b2])
    assert got.tolist() == [[False, True, False], [True, False, False],
                            [False, False, False]]


def test_phantom_needs_every_band_above_to_be_free():
    low = np.ones((2, 2), dtype=bool)
    hit_top_band_only = np.array([[0, 0], [0, 1]], dtype=bool)
    got = phantom_cell_mask(low, [np.zeros((2, 2), bool), hit_top_band_only])
    assert got.tolist() == [[True, True], [True, False]]


def test_phantom_mask_rejects_shape_mismatch():
    with pytest.raises(ValueError, match="same shape"):
        phantom_cell_mask(np.ones((2, 2), bool), [np.ones((3, 3), bool)])


def test_speckle_stats_counts_components_and_areas():
    m = np.zeros((10, 10), dtype=bool)
    m[0:2, 0:2] = True                      # 4 cells
    m[5, 5] = True                          # 1 cell
    m[8, 0:4] = True                        # 4 cells
    st = speckle_stats(m, cell=0.05)
    assert st["components"] == 3
    assert st["cells"] == 9
    assert st["area_m2"] == pytest.approx(9 * 0.0025)
    assert st["median_blob_m2"] == pytest.approx(4 * 0.0025)
    assert st["frac_blobs_le_100cm2"] == pytest.approx(1.0)


def test_monotone_toward_floor_is_about_the_lowest_band():
    assert monotone_toward_floor([0.11, 0.13, 0.17, 0.40]) is False
    assert monotone_toward_floor([0.11, 0.13, 0.17, 0.16]) is True
    assert monotone_toward_floor([0.11, 0.13, 0.17, 0.17]) is True


def test_open_floor_is_the_phantom_test_without_the_low_band_half():
    low = np.array([[1, 1, 0], [1, 0, 0], [0, 0, 0]], dtype=bool)
    b1 = np.array([[1, 0, 0], [0, 0, 0], [0, 0, 0]], dtype=bool)
    b2 = np.zeros((3, 3), dtype=bool)
    openf = open_floor_cell_mask([b1, b2])
    np.testing.assert_array_equal(phantom_cell_mask(low, [b1, b2]), low & openf)
    # and it is a strict superset here: cell (0,2) is free above but also free in the low band
    assert openf[0, 2] and not phantom_cell_mask(low, [b1, b2])[0, 2]


def test_open_floor_rejects_shape_mismatch_and_empty_input():
    with pytest.raises(ValueError, match="same shape"):
        open_floor_cell_mask([np.ones((2, 2), bool), np.ones((3, 3), bool)])
    with pytest.raises(ValueError, match="at least one band"):
        open_floor_cell_mask([])


# ------------------------------------------------------------------ P1b --

EXTENT = (0.0, 0.0, 1.0, 1.0)
CELL = 0.05


def test_footprint_cell_span_clips_to_the_grid():
    lo = np.array([[0.02, 0.02], [-1.0, 0.5]])
    hi = np.array([[0.09, 0.09], [0.06, 0.62]])
    i0, i1, j0, j1 = footprint_cell_span(lo, hi, EXTENT, CELL, (20, 20))
    assert (i0[0], i1[0], j0[0], j1[0]) == (0, 1, 0, 1)
    assert (i0[1], i1[1], j0[1], j1[1]) == (0, 1, 10, 12)


def _flat_splat(x, y, z, sxy=0.01, sz=0.005):
    return [x, y, z], np.diag([sxy ** 2, sxy ** 2, sz ** 2])


def _scene(rows, z_floor=0.0):
    means = np.array([r[0] for r in rows], dtype=float)
    covs = np.array([r[1] for r in rows], dtype=float)
    n = len(rows)
    return GaussianScene3D(means, covs, np.full(n, 0.9),
                           np.arange(n, dtype=np.int64), "t")


def test_replace_needs_the_whole_footprint_inside_phantom_cells():
    phantom = np.zeros((20, 20), dtype=bool)
    phantom[4:9, 4:9] = True                       # x,y in [0.20, 0.45)
    rows = [_flat_splat(0.32, 0.32, 0.01),         # deep inside the block
            _flat_splat(0.20, 0.32, 0.01)]         # footprint spills west of it
    m = replace_mask(_scene(rows), phantom, EXTENT, CELL,
                     z_floor=0.0, max_top=0.10, level=2.0)
    assert m.tolist() == [True, False]


def test_replace_needs_the_rho_top_below_the_ceiling_of_the_rule():
    phantom = np.ones((20, 20), dtype=bool)
    rows = [_flat_splat(0.5, 0.5, 0.01, sz=0.005),   # top 0.02
            _flat_splat(0.5, 0.5, 0.09, sz=0.02)]    # top 0.13 > 0.10
    m = replace_mask(_scene(rows), phantom, EXTENT, CELL,
                     z_floor=0.0, max_top=0.10, level=2.0)
    assert m.tolist() == [True, False]


def test_plane_tiles_stay_below_the_band_and_cover_the_extent():
    z_floor, cap = -1.2, -1.2 + 0.02
    tiles = plane_tiles(EXTENT, z_floor=z_floor, normal=(0.0, 0.0, 1.0),
                        centroid=(0.5, 0.5, z_floor), spacing=0.25,
                        sigma_n=0.004, opacity=0.95, first_id=1000,
                        z_top_max=cap - 0.005, level=2.0)
    tops = tiles["means"][:, 2] + 2.0 * np.sqrt(tiles["covs"][:, 2, 2])
    assert tops.max() < cap
    assert np.all(tiles["opacity"] > 0.3)
    assert len(np.unique(tiles["ids"])) == len(tiles["ids"])
    assert tiles["ids"].min() >= 1000
    # every point of the extent is inside some tile's rho=2 ellipse
    gx, gy = np.meshgrid(np.linspace(0.0, 1.0, 21), np.linspace(0.0, 1.0, 21))
    pts = np.column_stack([gx.ravel(), gy.ravel()])
    d = np.linalg.norm(pts[:, None, :] - tiles["means"][None, :, :2], axis=2)
    reach = 2.0 * np.sqrt(tiles["covs"][:, 0, 0])[None, :]
    assert np.all((d <= reach).any(axis=1))


def test_plane_tiles_reject_a_sigma_that_reaches_the_band():
    with pytest.raises(ValueError, match="z_top_max"):
        plane_tiles(EXTENT, z_floor=0.0, normal=(0.0, 0.0, 1.0),
                    centroid=(0.5, 0.5, 0.0), spacing=0.25, sigma_n=0.05,
                    opacity=0.95, first_id=0, z_top_max=0.015, level=2.0)


def test_build_plane_floor_swaps_phantom_splats_for_an_invisible_plane():
    phantom = np.zeros((20, 20), dtype=bool)
    phantom[4:9, 4:9] = True
    rows = [_flat_splat(0.32, 0.32, 0.01),                 # replaced
            _flat_splat(0.20, 0.32, 0.01),                 # spills out: kept
            ([0.7, 0.7, 0.4], np.diag([0.01, 0.01, 0.04]))]  # a table leg: kept
    scene = _scene(rows)
    out, st = build_plane_floor(scene, phantom, EXTENT, CELL,
                                floor={"z_floor": 0.0, "normal": [0, 0, 1],
                                       "centroid": [0.5, 0.5, 0.0]},
                                spacing=0.25, sigma_n=0.004, z_lo=0.02)
    assert st["replaced"] == 1
    assert st["inserted"] == len(out) - (len(scene) - 1)
    survivors = set(out.ids.tolist()[:st["kept"]])
    assert survivors == {1, 2}          # id 0 was the phantom one, and only it
    assert 0 not in set(out.ids.tolist())
    assert np.all(np.diff(out.ids) > 0), "ids must stay sorted for searchsorted"
    # the inserted plane is invisible to a robot band starting at z_floor + 0.02
    ins = out.subset(out.ids >= st["first_inserted_id"])
    tops = ins.aabb(2.0)[1][:, 2]
    assert tops.max() < 0.02
    assert st["inserted_rho_top_max_above_floor"] == pytest.approx(
        float(tops.max()), abs=1e-12)


# ------------------------------------------------------------------ the P2/P3 hand-off --


def test_plane_scene_round_trips_with_its_metadata(tmp_path):
    rows = [_flat_splat(0.3, 0.3, 0.01), ([0.7, 0.7, 0.4], np.diag([0.01, 0.01, 0.04]))]
    scene = _scene(rows)
    meta = {"floor": {"z_floor": -1.227, "normal": [0.0, 0.0, 1.0]}, "params": {"max_top": 0.10}}
    p = save_plane_scene(tmp_path / "sub" / "plane.npz", scene, meta)
    got, got_meta = load_plane_scene(p)
    assert len(got) == len(scene)
    np.testing.assert_allclose(got.means, scene.means)
    np.testing.assert_allclose(got.covs, scene.covs)
    np.testing.assert_array_equal(got.ids, scene.ids)
    assert got_meta == meta
    assert got.name == "showcase_planefloor"


def test_replaced_splats_never_have_mass_above_the_overhead_test_height():
    """The guarantee P1c rests on: a splat under anything at all above 0.10 m is kept.

    A table leg is modelled as a column of splats under a tabletop; the tabletop occupies the
    band above, so the leg's cells are outside the open-floor mask and no leg splat is a
    candidate, whatever max_top is set to.
    """
    bands_above = [np.zeros((20, 20), dtype=bool)]
    bands_above[0][13:17, 13:17] = True             # a tabletop over x,y in [0.65, 0.85)
    openf = open_floor_cell_mask(bands_above)
    rows = [_flat_splat(0.70, 0.70, 0.01),          # leg foot under the top
            _flat_splat(0.72, 0.72, 0.08, sz=0.01),  # leg, higher up
            _flat_splat(0.30, 0.30, 0.01)]          # dust on genuinely open floor
    for max_top in (0.10, 0.20, 0.50):
        m = replace_mask(_scene(rows), openf, EXTENT, CELL,
                         z_floor=0.0, max_top=max_top, level=2.0)
        assert m.tolist() == [False, False, True], f"max_top={max_top}"
