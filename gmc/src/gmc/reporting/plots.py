"""Debug visualization (Guide §2.2 Visualization layer)."""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def plot_slice(sl, path: str | Path, title: str = ""):
    fig, ax = plt.subplots(figsize=(9, 6))
    for g in [sl.C_plus]:
        for poly in getattr(g, "geoms", [g]):
            if poly.is_empty:
                continue
            ax.fill(*poly.exterior.xy, color="0.35", zorder=2)
    for comp in sl.D_possible:
        ax.fill(*comp.geometry.exterior.xy, color="tab:orange", alpha=0.25,
                zorder=1)
    for comp in sl.D_safe:
        ax.fill(*comp.geometry.exterior.xy, color="tab:green", alpha=0.35,
                zorder=1)
        ax.plot(*comp.representative, "k*", ms=8, zorder=3)
    ax.set_aspect("equal")
    ax.set_title(title or f"slice theta={sl.theta:.3f} "
                          f"safe={len(sl.D_safe)} possible={len(sl.D_possible)}")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_slabs(decomposition, path: str | Path):
    fig, ax = plt.subplots(figsize=(11, 2.6))
    for s in decomposition.slabs:
        color = "tab:green" if s.kind == "regular" else "tab:red"
        ax.axvspan(np.rad2deg(s.interval.lo), np.rad2deg(s.interval.hi),
                   color=color, alpha=0.4)
    ax.set_xlim(0, 360)
    ax.set_yticks([])
    ax.set_xlabel("theta (deg)")
    ax.set_title("orientation slabs (green=regular candidate, red=uncertain)")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_curve(scene, curve, path: str | Path):
    from ..mobility.witness import SegmentKind
    fig, ax = plt.subplots(figsize=(9, 6))
    for s in scene.supports:
        evals, evecs = np.linalg.eigh(s.covariance)
        ang = np.degrees(np.arctan2(evecs[1, 1], evecs[0, 1]))
        from matplotlib.patches import Ellipse
        ax.add_patch(Ellipse(s.mean, 2 * s.level * np.sqrt(evals[1]),
                             2 * s.level * np.sqrt(evals[0]), angle=ang,
                             color="0.4", alpha=0.7))
    for seg in curve.segments:
        pts = [seg.q0] + list(seg.control_points) + [seg.q1]
        xs = [p.xy[0] for p in pts]
        ys = [p.xy[1] for p in pts]
        if seg.kind is SegmentKind.TRANSLATION:
            ax.plot(xs, ys, "b-", lw=2)
        else:
            ax.plot(xs[0], ys[0], "ro", ms=6)
    ax.set_aspect("equal")
    x0, y0, x1, y1 = scene.workspace.bounds
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_title("lifted trajectory (blue=translate, red=rotate)")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
