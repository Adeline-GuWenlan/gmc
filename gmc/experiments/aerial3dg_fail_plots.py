"""F1 figures (no planner): archive raster around the G2 box (Task 1)."""
from __future__ import annotations

import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

F1 = Path("results/aerial3dg/f1")
G2_BOX = (-9.0, -0.35, 3.7, 2.75)


def box_patch(ax, b, **kw):
    ax.add_patch(Rectangle((b[0], b[1]), b[2] - b[0], b[3] - b[1], fill=False, **kw))


def plot_archive(out=F1 / "range" / "archive_raster.png", zoom=None):
    meta = json.loads((F1 / "range" / "archive_raster.json").read_text())
    A = np.load(F1 / "range" / "archive_raster.npz")
    u0, v0, u1, v1 = meta["window_uv"]
    panels = [("z020_200_centres", "all Gaussian centres, z 0.2-2.0 m (walls)"),
              ("cylinder_cover", "cylinder band 0.02-1.75 m: 2-sigma xy footprint coverage"),
              ("sweeper_cover", "sweeper band 0.02-0.10 m: 2-sigma xy footprint coverage")]
    fig, axes = plt.subplots(len(panels), 1, figsize=(16, 4.6 * len(panels)))
    for ax, (k, title) in zip(axes, panels):
        img = np.log10(1 + A[k].T.astype(float))
        ax.imshow(img, origin="lower", extent=(u0, u1, v0, v1), cmap="magma_r", aspect="equal")
        box_patch(ax, G2_BOX, ec="#1f77b4", lw=2, label="G2 compile box")
        for pu, pv, ls in ((1, 1, "--"), (2, 2, ":")):
            b = (G2_BOX[0] - pu, G2_BOX[1] - pv, G2_BOX[2] + pu, G2_BOX[3] + pv)
            box_patch(ax, b, ec="#2ca02c", lw=1.2, ls=ls, label=f"widened +{pu} m")
        ax.axvline(-5.6, color="c", lw=.8, ls="--")
        ax.axvline(-.85, color="orange", lw=.8, ls="--")
        ax.set(title=title + " (log10(1+count) per 5 cm cell)", xlabel="u (m)", ylabel="v (m)")
        if zoom:
            ax.set(xlim=zoom[:2], ylim=zoom[2:])
        ax.legend(loc="upper right", fontsize=7)
    fig.tight_layout()
    fig.savefig(out, dpi=80)
    print("wrote", out)


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "archive"
    if what == "archive":
        plot_archive()
        plot_archive(F1 / "range" / "archive_raster_west.png", zoom=(-12, -3, -3.5, 5.5))


def plot_zoom(zoom, out):
    plot_archive(out, zoom=zoom)
