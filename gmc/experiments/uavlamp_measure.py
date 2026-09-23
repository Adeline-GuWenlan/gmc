"""Measure real walls, table tops and the ceiling of the plane-floor hall (read-only).

Raster statistics of opaque (opacity > tau) Gaussians' level-2 AABBs, in world
xy cells, heights relative to the plane floor.  Output: ``measure.npz`` with the
rasters, ``tables.json`` with table-top candidates, and PNG maps to inspect.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage

TAU, LEVEL = .3, 2.


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--archive", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--cell", type=float, default=.10)
    a = p.parse_args(argv)
    out = a.output
    out.mkdir(parents=True, exist_ok=True)
    with np.load(a.archive, allow_pickle=False) as d:
        meta = json.loads(str(d["meta"]))
        op = d["opacity"] > TAU
        means = d["means"][op]
        covs = d["covs"]
        czz, cxx, cyy = covs[op, 2, 2], covs[op, 0, 0], covs[op, 1, 1]
        del covs
    zf = float(meta["z_floor"])
    zc = means[:, 2] - zf
    hz = LEVEL * np.sqrt(czz)
    zlo, zhi = zc - hz, zc + hz
    hxy = LEVEL * np.sqrt(np.maximum(cxx, cyy))
    x0, y0, x1, y1 = meta["extent"]
    c = a.cell
    nx, ny = int(np.ceil((x1 - x0) / c)), int(np.ceil((y1 - y0) / c))
    ix = np.clip(((means[:, 0] - x0) / c).astype(int), 0, nx - 1)
    iy = np.clip(((means[:, 1] - y0) / c).astype(int), 0, ny - 1)
    flat = iy * nx + ix
    # Ignore huge blobs for structure rasters (they are counted separately).
    small = (hz < .6) & (hxy < .6)
    slices = np.arange(.0, 5.6, .1)
    occ = np.zeros((len(slices), ny * nx), np.uint16)
    for k, s in enumerate(slices):
        m = small & (zhi >= s) & (zlo <= s + .1)
        occ[k] = np.minimum(np.bincount(flat[m], minlength=nx * ny), 65535)
    occ = occ.reshape(len(slices), ny, nx)
    present = occ >= 2
    wall_band = (slices >= .3) & (slices < 2.6)
    wall_frac = present[wall_band].mean(axis=0)
    # Table tops: highest occupied slice in .6-1.2 m with nothing from top+.1 to 2.5 m.
    top = np.full((ny, nx), np.nan)
    for k in np.flatnonzero((slices >= .6) & (slices < 1.2)):
        above = present[k + 2:np.searchsorted(slices, 2.5)].any(axis=0)
        cand = present[k] & ~above
        top[cand] = slices[k] + .1
    table_mask = np.isfinite(top)
    lab, n = ndimage.label(table_mask)
    tables = []
    for i in range(1, n + 1):
        m = lab == i
        area = m.sum() * c * c
        if area < .25:
            continue
        yy, xx = np.nonzero(m)
        tables.append({"label": i, "area_m2": float(area),
                       "x_range": [float(x0 + xx.min() * c), float(x0 + (xx.max() + 1) * c)],
                       "y_range": [float(y0 + yy.min() * c), float(y0 + (yy.max() + 1) * c)],
                       "top_slice_median_m": float(np.nanmedian(top[m])),
                       "centroid_xy": [float(x0 + (xx.mean() + .5) * c), float(y0 + (yy.mean() + .5) * c)]})
    tables.sort(key=lambda t: -t["area_m2"])
    # Ceiling: lowest occupied slice above 2.5 m.
    ceil = np.full((ny, nx), np.nan)
    for k in reversed(np.flatnonzero(slices >= 2.5)):
        ceil[present[k]] = slices[k]
    np.savez_compressed(out / "measure.npz", occ=occ, slices=slices, wall_frac=wall_frac, top=top,
                        ceil=ceil, origin=np.array([x0, y0]), cell=c, z_floor=zf)
    (out / "tables.json").write_text(json.dumps({"z_floor": zf, "extent": meta["extent"],
                                                 "ceiling_height_m": meta.get("ceiling_height_m"),
                                                 "tables": tables[:40]}, indent=1))
    ext = [x0, x0 + nx * c, y0, y0 + ny * c]
    for name, img, kw in (("wall_frac", wall_frac, {"cmap": "magma", "vmin": 0, "vmax": 1}),
                          ("table_top", top, {"cmap": "viridis", "vmin": .6, "vmax": 1.2}),
                          ("ceiling_underside", ceil, {"cmap": "plasma", "vmin": 2.5, "vmax": 5.5})):
        fig, ax = plt.subplots(figsize=(9, 13))
        im = ax.imshow(img, origin="lower", extent=ext, **kw)
        plt.colorbar(im, ax=ax, shrink=.6)
        if name == "table_top":
            for t in tables[:25]:
                ax.annotate(str(t["label"]), t["centroid_xy"], color="red", fontsize=7)
        ax.set_title(name); ax.set_xlabel("world x"); ax.set_ylabel("world y")
        ax.set_xticks(np.arange(np.floor(x0), x1, 2)); ax.set_yticks(np.arange(np.floor(y0), y1, 2))
        ax.grid(alpha=.3)
        fig.tight_layout(); fig.savefig(out / f"{name}.png", dpi=110); plt.close(fig)
    for lo_, hi_ in ((.1, .7), (.7, 1.0), (1.1, 2.5), (2.5, 4.0)):
        k = (slices >= lo_) & (slices < hi_)
        img = present[k].mean(axis=0)
        fig, ax = plt.subplots(figsize=(9, 13))
        ax.imshow(img, origin="lower", extent=ext, cmap="Greys", vmin=0, vmax=1)
        ax.set_title(f"occupied fraction of slices {lo_}-{hi_} m"); ax.grid(alpha=.3)
        ax.set_xticks(np.arange(np.floor(x0), x1, 2)); ax.set_yticks(np.arange(np.floor(y0), y1, 2))
        fig.tight_layout(); fig.savefig(out / f"band_{lo_:.1f}_{hi_:.1f}.png", dpi=110); plt.close(fig)
    print(json.dumps({"n_opaque": int(op.sum()), "tables": tables[:10]}, indent=1))


if __name__ == "__main__":
    main()
