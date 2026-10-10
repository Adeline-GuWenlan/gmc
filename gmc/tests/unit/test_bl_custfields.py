"""bl B2 cust_fields world-construction tests (``experiments/bl_custfields.py``, numpy parts only; gmc-venv).

The repo-side checks (our squircle level == ``Rectangular.check_point_inside``; the workspace-tree TypeError) run in
the cust_fields env (``bl_custfields_probe.py``)."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

GMC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(GMC / "experiments"))

import bl_custfields as C  # noqa: E402


def test_label8_matches_scipy():
    from scipy import ndimage
    rng = np.random.default_rng(0)
    for p in (.3, .45, .6):
        m = rng.random((60, 80)) < p
        lab, n = C.label8(m)
        ref, nr = ndimage.label(m, structure=np.ones((3, 3), int))
        assert n == nr
        # same partition: a bijection between labels
        pairs = set(zip(lab[m].tolist(), ref[m].tolist()))
        assert len(pairs) == n and len({a for a, _ in pairs}) == n and len({b for _, b in pairs}) == n


def test_hull_rect_and_cover_contain_the_points():
    rng = np.random.default_rng(1)
    for s in (.85, .99):
        for _ in range(20):
            th = rng.uniform(0, np.pi)
            R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
            P = (rng.normal(size=(200, 2)) * rng.uniform(.05, 1., 2)) @ R.T + rng.uniform(-3, 3, 2)
            H = C.convex_hull(P)
            c, a, b, t = C.fit_cover(H, s)
            assert (C.squircle_level(P, c, a, b, t, s) <= 1e-12).all()
            # not absurdly loose: area of the cover <= 2.5 x the min-area rectangle
            c2, ab, t2 = C.min_area_rect(H)
            assert 4 * a * b <= 2.5 * 4 * ab[0] * ab[1] + 1e-9
            # the rectangle really encloses the hull
            X = (H - c2) @ np.array([[math.cos(t2), -math.sin(t2)], [math.sin(t2), math.cos(t2)]])
            assert (np.abs(X) <= ab[None, :] + 1e-9).all()


def test_overlap_test_and_merging():
    s = .85
    A = (np.array([0., 0.]), .5, .3, 0.)
    B = (np.array([1.05, 0.]), .5, .3, 0.)          # 5 cm apart along x
    Cc = (np.array([.95, 0.]), .5, .3, 0.)          # overlaps A
    assert not C.covers_overlap(A, B, s, 0.)
    assert C.covers_overlap(A, B, s, .08)
    assert C.covers_overlap(A, Cc, s, 0.)
    # two touching blobs on a grid merge into one cover, a far one stays separate
    m = np.zeros((40, 80), bool)
    m[10:20, 10:20] = m[12:18, 19:30] = m[25:35, 60:70] = True
    info = {"res_m": .01, "u0": 0., "v0": 0.}
    covers, groups, st = C.build_obstacles(m, np.ones_like(m), info, s, 0.)
    assert st["pieces"] == 2 and st["covers"] == 2 and st["merges"] == 0
    m2 = m.copy()
    m2[20:24, 30:40] = True                           # a separate piece whose cover overlaps the first one's
    covers, groups, st = C.build_obstacles(m2, np.ones_like(m2), info, s, 0.)
    assert st["pieces"] == 3 and st["merges"] >= 1 and st["covers"] == 2
    cm = C.cover_mask(covers, s, info, m2.shape)
    assert cm[m2].all()                               # every piece cell centre is covered


def test_snap_point_needs_a_map_free_segment():
    grid = {"res_m": .01, "u0": 0., "v0": 0.}
    occ = np.zeros((50, 50), np.uint8)
    occ[:, 24:27] = 1                                  # a wall at x in [0.24, 0.27)
    nf_free = (occ == 0).astype(np.uint8)
    nf_free[:, 15:24] = 0                              # the NF world swallowed the wall's left side
    p = np.array([.105, .255])                         # left of the wall
    c, d = C.snap_point(occ, nf_free, grid, p, max_m=.5)
    assert c is not None and c[0] < .24                # the left free strip, not the nearer cell across the wall
    nf_free[:, :24] = 0                                # nothing free on the left: across the wall is not allowed
    assert C.snap_point(occ, nf_free, grid, p, max_m=.5) == (None, None)
    occ2 = occ.copy()
    occ2[25, 10] = 1                                   # the endpoint's own cell occupied (map conservatism): allowed
    nf2 = (occ2 == 0).astype(np.uint8)
    c, d = C.snap_point(occ2, nf2, grid, np.array([.105, .255]), max_m=.5)
    assert c is not None and d <= .01
