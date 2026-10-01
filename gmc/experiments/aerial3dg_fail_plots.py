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


def plot_zoom(zoom, out):
    plot_archive(out, zoom=zoom)


# ----------------------------------------------------------------------------- class maps (Task 2)
START_C, GOAL_C, HI_C, LAMP_C = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"


def _background(ax, robot, window=(-11.2, 5.9, -2.6, 5.0)):
    meta = json.loads((F1 / "range" / "archive_raster.json").read_text())
    A = np.load(F1 / "range" / "archive_raster.npz")
    u0, v0, u1, v1 = meta["window_uv"]
    ax.imshow(np.log10(1 + A[f"{robot}_cover"].T.astype(float)), origin="lower", extent=(u0, u1, v0, v1),
              cmap="Greys", vmin=0, vmax=3, aspect="equal")
    box_patch(ax, G2_BOX, ec="#444", lw=1.4)
    ax.set(xlim=window[:2], ylim=window[2:], xlabel="u (m)", ylabel="v (m)")


def plot_class_maps(out=F1 / "class_maps.png"):
    import csv
    rows = list(csv.DictReader(open(F1 / "classes.csv")))
    panels = [("sweeper", ["SW-EP"], "SW-EP: endpoint over a sub-floor Gaussian (within buffer)"),
              ("sweeper", ["SW-KIN"], "SW-KIN: replay yaw-rate round-off (route shown faint)"),
              ("cylinder", ["CY-LAMP"], "CY-LAMP: certified cut at lamp + bulkhead"),
              ("cylinder", ["CY-GAP"], "CY-GAP: safe graph split at u~-5.6 (W1 routes faint)"),
              ("cylinder", ["CY-EP", "CY-EP-LAT"], "CY-EP / CY-EP-LAT: endpoint within buffer")]
    fig, axes = plt.subplots(len(panels), 1, figsize=(14, 4.3 * len(panels)))
    for ax, (robot, cls, title) in zip(axes, panels):
        _background(ax, robot)
        sel = [r for r in rows if r["robot"] == robot and r["class"] in cls]
        if any(c in ("SW-EP", "CY-EP") for c in cls):     # only the endpoint that failed certification
            ep = [json.loads(r["evidence"]).get("endpoint") for r in sel]
            S = np.array([[float(r["start_u"]), float(r["start_v"])] for r, e in zip(sel, ep) if e == "start"]).reshape(-1, 2)
            G = np.array([[float(r["goal_u"]), float(r["goal_v"])] for r, e in zip(sel, ep) if e == "goal"]).reshape(-1, 2)
        else:
            S = np.array([[float(r["start_u"]), float(r["start_v"])] for r in sel]).reshape(-1, 2)
            G = np.array([[float(r["goal_u"]), float(r["goal_v"])] for r in sel]).reshape(-1, 2)
        if "SW-KIN" in cls:
            rp = json.loads((F1 / "diag" / "replay_sweeper.json").read_text())
            for x in rp["rows"]:
                if x.get("polyline_route"):
                    P = np.asarray(x["polyline_route"])
                    ax.plot(P[:, 0], P[:, 1], color=START_C, lw=.6, alpha=.35)
        if "CY-GAP" in cls:
            w = F1 / "widen" / "w1" / "cylinder" / "fast_00.jsonl"
            if w.exists():
                want = {int(r["index"]) for r in sel}
                k = 0
                for line in open(w):
                    x = json.loads(line)
                    if x["index"] in want and x["status"] == "REACHABLE" and x.get("route_polyline") and k < 60:
                        P = np.asarray(x["route_polyline"])
                        ax.plot(P[:, 0], P[:, 1], color=HI_C, lw=.7, alpha=.5)
                        k += 1
            b = F1 / "diag" / "bridges_cylinder.json"
            if b.exists():
                for c in json.loads(b.read_text())["corridors"].values():
                    if "leaf_centres_uv" in c:
                        C = np.asarray(c["leaf_centres_uv"])
                        ax.scatter(C[:, 0], C[:, 1], s=4, marker="s", color="#d62728", label="UNKNOWN corridor leaves")
            box_patch(ax, (-10, -1.35, 4.7, 3.75), ec=HI_C, lw=1, ls="--")
        if "CY-LAMP" in cls:
            ax.add_patch(Rectangle((-1.03, -.08), .36, 2.56, color=LAMP_C, alpha=.5, label="lamp footprint"))
        if any(c in ("SW-EP", "CY-EP") for c in cls):
            d = json.loads((F1 / "diag" / f"endpoints_{robot}.json").read_text())
            seen = {}
            for x in d["rows"]:
                bl = min(x["blockers"], key=lambda b: b["refined_gap_m"])
                seen.setdefault(bl["scene_id"], [bl["mean_route"], 0])[1] += 1
            for sid, (mu, n) in seen.items():
                ax.scatter([mu[0]], [mu[1]], s=60 + 2 * n, marker="*", color=HI_C, edgecolor="k", lw=.5, zorder=5)
                if n >= 9:
                    ax.annotate(f"id {sid}: {n}", (mu[0], mu[1]), xytext=(6, 6), textcoords="offset points", fontsize=8)
        ax.scatter(S[:, 0], S[:, 1], s=9, marker="o", color=START_C, alpha=.6, label=f"start ({len(S)})", zorder=4)
        ax.scatter(G[:, 0], G[:, 1], s=12, marker="x", color=GOAL_C, alpha=.6, label=f"goal ({len(G)})", zorder=4)
        ax.set_title(f"{title}  [{robot}, n={len(sel)}]", fontsize=10)
        h, l = ax.get_legend_handles_labels()
        uniq = dict(zip(l, h))
        ax.legend(uniq.values(), uniq.keys(), loc="upper right", fontsize=7, markerscale=1.5)
    fig.tight_layout()
    fig.savefig(out, dpi=75)
    print("wrote", out)


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "archive"
    if what == "classes":
        plot_class_maps()
    if what == "archive":
        plot_archive()
        plot_archive(F1 / "range" / "archive_raster_west.png", zoom=(-12, -3, -3.5, 5.5))
