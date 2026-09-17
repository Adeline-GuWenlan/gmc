"""Amendment 3 P1: the phantom-cell test and the plane-floor replacement.

The plane floor is a user-approved MANUAL scene-definition change, not a
reconstruction method; these tests pin the edit's mechanics, not its realism.
"""
import numpy as np
import pytest

from gmc.height.planefloor import (monotone_toward_floor, phantom_cell_mask,
                                   speckle_stats)


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
