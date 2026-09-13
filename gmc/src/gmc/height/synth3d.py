"""Synthetic 3D table scene for the T2 gates (spec §5.2).

Every boundary is placed so the rho=2 support surfaces land exactly on the
nominal numbers: sphere radius R = 2*sigma, centres inset by R.
"""
import numpy as np

from .ply3d import GaussianScene3D

LEVEL = 2.0
WORKSPACE = (-3.0, -2.0, 3.0, 2.0)
Z_FLOOR = 0.0
TABLETOP_XY = (-0.3, -0.6, 0.3, 0.6)
START = (-2.2, 0.0, 0.0)
GOAL = (2.2, 0.0, 0.0)

WALL_SIGMA, WALL_SPACING = 0.04, 0.10
TOP_SIGMA = (0.04, 0.04, 0.01)
LEG_SIGMA = (0.015, 0.015, 0.05)
LEG_XY = [(0.25, 0.55), (0.25, -0.55), (-0.25, 0.55), (-0.25, -0.55)]


def _span(lo, hi, spacing):
    n = max(1, int(np.ceil((hi - lo) / spacing - 1e-9))) + 1
    return np.linspace(lo, hi, n)


def _wall_segment(y_lo, y_hi):
    R = LEVEL * WALL_SIGMA
    ys = _span(y_lo + R, y_hi - R, WALL_SPACING)
    zs = _span(R, 2.5 - R + 0.06, WALL_SPACING)        # top row centre 2.48
    yy, zz = np.meshgrid(ys, zs, indexing="ij")
    pts = np.column_stack([np.zeros(yy.size), yy.ravel(), zz.ravel()])
    return pts, np.tile(np.eye(3) * WALL_SIGMA ** 2, (len(pts), 1, 1))


def table_scene(variant: str):
    if variant not in ("open", "closed"):
        raise ValueError("variant must be 'open' or 'closed'")
    parts, groups = [], {"wall": [], "tabletop": [], "legs": []}

    segments = [(-2.5, -0.6), (0.6, 1.2)] + ([(1.2, 2.5)] if variant == "closed" else [])
    for y_lo, y_hi in segments:
        parts.append(("wall",) + _wall_segment(y_lo, y_hi))

    sx, sy, sz = TOP_SIGMA
    xs = _span(TABLETOP_XY[0] + LEVEL * sx, TABLETOP_XY[2] - LEVEL * sx, 0.10)
    ys = _span(TABLETOP_XY[1] + LEVEL * sy, TABLETOP_XY[3] - LEVEL * sy, 0.10)
    xx, yy = np.meshgrid(xs, ys, indexing="ij")
    top = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, 0.74)])
    parts.append(("tabletop", top, np.tile(np.diag([sx ** 2, sy ** 2, sz ** 2]), (len(top), 1, 1))))

    lx, ly, lz = LEG_SIGMA
    zc = _span(LEVEL * lz, 0.72 - LEVEL * lz, 0.10)
    for x, y in LEG_XY:
        pts = np.column_stack([np.full(len(zc), x), np.full(len(zc), y), zc])
        parts.append(("legs", pts, np.tile(np.diag([lx ** 2, ly ** 2, lz ** 2]), (len(pts), 1, 1))))

    means, covs, next_id = [], [], 0
    for group, pts, cv in parts:
        groups[group].extend(range(next_id, next_id + len(pts)))
        next_id += len(pts)
        means.append(pts)
        covs.append(cv)
    means, covs = np.vstack(means), np.vstack(covs)
    scene = GaussianScene3D(means, covs, np.full(len(means), 0.9),
                            np.arange(len(means)), f"table_{variant}")
    return scene, {"groups": groups, "variant": variant}
