"""Plane floor (Amendment 3 §P1): swap the phantom near-floor splats for one analytic plane.

**This is a user-approved MANUAL scene-definition change, not a reconstruction method and not an
outer approximation with a guarantee.** The user's framing was "地面的高斯用大平面替代，手动解决噪音
for now" — a stopgap. Anything computed on the edited scene is sound *with respect to the edited
scene*; the gap between that and the room is the open research question
(`docs/worklog/height_map_diagnosis.md` §4). Every report, figure caption and video title that uses
this module must say so.

This file starts with the half of the edit that decides **which splats are even candidates**:
:func:`phantom_cell_mask` is the diagnosis' discriminating test (occupied in the sweeper band, free
in every band above it), :func:`speckle_stats` is the morphology summary that separates speckle
from furniture outlines, and :func:`monotone_toward_floor` is the acceptance test the whole stage
is judged by.
"""
import numpy as np
from scipy import ndimage

Z_LO_DEFAULT = 0.02        # robot ground clearance (spec §3.3, frozen)
TAU_DEFAULT = 0.3


# ------------------------------------------------------------------ P1a: the phantom test --


def phantom_cell_mask(low_band, bands_above):
    """Cells occupied in the sweeper band and free in **every** band above it.

    This is the discriminating test of `height_map_diagnosis.md` §2: a 2-10 cm obstacle with
    nothing at all above it, repeated over hundreds of m2 of an art gallery, is not furniture.
    """
    low = np.asarray(low_band, dtype=bool)
    above = np.zeros_like(low)
    for band in bands_above:
        b = np.asarray(band, dtype=bool)
        if b.shape != low.shape:
            raise ValueError(f"every band must have the same shape as the low band: "
                             f"{b.shape} != {low.shape}")
        above |= b
    return low & ~above


def speckle_stats(mask, cell):
    """Component count and blob-size summary — speckle (phantom) vs outlines (furniture)."""
    mask = np.asarray(mask, dtype=bool)
    area = float(cell) ** 2
    lab, n = ndimage.label(mask)
    sizes = np.bincount(lab.ravel())[1:] * area if n else np.zeros(0)
    return {"cells": int(mask.sum()), "area_m2": float(mask.sum() * area),
            "components": int(n),
            "median_blob_m2": float(np.median(sizes)) if n else 0.0,
            "frac_blobs_le_100cm2": float((sizes <= 4 * area).mean()) if n else 0.0}


def monotone_toward_floor(fracs):
    """True iff the lowest band is no more occupied than the band above it.

    ``fracs`` is ordered top band first, floor band last — the order the evidence table is read in.
    Occupancy should *increase* with height if anything, because everything standing on the floor
    is seen by the low band too; the as-built showcase map does the opposite (40.7 % against
    17.4 %), which is the defect P1 exists to fix.
    """
    f = [float(v) for v in fracs]
    if len(f) < 2:
        raise ValueError("need at least two bands")
    return bool(f[-1] <= f[-2])
