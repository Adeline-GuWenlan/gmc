"""bl B1 figure: pilot pairs, cylinder, top-down. Grey = xy shadows (2-sigma, the judge's level) of the judge's
Gaussians that meet the cylinder's slab; black = the pair's A* route; green = SplatNav (frozen config), orange/red =
FOCI (SUCCESS / unsafe claim); purple = floor-level Gaussians whose 2-sigma top is within 8 cm of the chassis
bottom (z_c - h = 0.02 m): sparse bumps that the A* route detours around and that a top view otherwise hides.
Paths are ``route_polyline`` = exactly what was judged (completion segments included). Body radius drawn at the start.

Run (CPU sbatch, gmc-venv, from gmc/): python experiments/bl_b1_fig.py --out results/baselines/pilot/fig_b1_pilot.png
"""
from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import EllipseCollection  # noqa: E402
import numpy as np  # noqa: E402

import bl_harness as H  # noqa: E402


def rows(method, region):
    f = Path(f"results/baselines/pilot/{method}/{region}/cylinder/task_00.jsonl.gz")
    return {r["pair_id"]: r for r in map(json.loads, gzip.open(f, "rt"))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    pairs = {p["pair_id"]: p for p in H.load_pairs("f4")}
    picks = []
    for reg in ("WWEST", "GAPW1", "S"):
        sn, fo = rows("splatnav", reg), rows("foci", reg)
        bad = [k for k in fo if fo[k]["status"].startswith("CLAIMED") and sn[k]["status"] == "SUCCESS"]
        good = [k for k in fo if fo[k]["status"] == "SUCCESS" and sn[k]["status"] == "SUCCESS"]
        picks += [(reg, k, sn[k], fo[k]) for k in (bad[:1] if bad else []) + good[:1]][:2 if reg != "S" else 1]
    fig, axes = plt.subplots(1, len(picks), figsize=(5.2 * len(picks), 5.4))
    for ax, (reg, pid, s, f) in zip(np.atleast_1d(axes), picks):
        with np.load(H.scene_export_path(reg, "cylinder"), allow_pickle=False) as d:
            means, covs, meta = d["means"], d["covs"], json.loads(str(d["meta"]))
        lv, zc, h, m = meta["level"], meta["z_c"], meta["body"]["half_height_m"], meta["margin_m"]
        ext = lv * np.sqrt(covs[:, 2, 2])
        P = [np.asarray(s["route_polyline"]), np.asarray(f["route_polyline"]),
             np.asarray(pairs[pid]["astar_route_uv"], float)]
        lo = np.min([p.min(0) for p in P], axis=0) - .6
        hi = np.max([p.max(0) for p in P], axis=0) + .6
        keep = ((means[:, 2] + ext >= zc - h - m) & (means[:, 2] - ext <= zc + h + m)
                & np.all(means[:, :2] > lo - .5, 1) & np.all(means[:, :2] < hi + .5, 1))
        C = covs[keep][:, :2, :2]
        ev, V = np.linalg.eigh(C)
        ang = np.degrees(np.arctan2(V[:, 1, 1], V[:, 0, 1]))
        w, hh = 2 * lv * np.sqrt(np.maximum(ev[:, 1], 0)), 2 * lv * np.sqrt(np.maximum(ev[:, 0], 0))
        ax.add_collection(EllipseCollection(w, hh, ang, units="xy", offsets=means[keep][:, :2],
                                            offset_transform=ax.transData, facecolors=(.55, .55, .55, .25),
                                            edgecolors="none"))
        ax.scatter(*means[keep][:, :2].T, s=.2, c="0.35", lw=0, rasterized=True)   # centres: small ones are sub-pixel
        low = keep & (means[:, 2] + ext <= zc - h + .08)        # floor-level: 2-sigma top within 8 cm of the chassis bottom
        ax.scatter(*means[low][:, :2].T, s=5, c="tab:purple", lw=0, label=f"floor-level Gaussians ({low.sum()})")
        ax.plot(*P[2].T, "k-", lw=1, label=f"A* route (len ratio {pairs[pid]['len_ratio']:.2f})")
        ax.plot(*P[0].T, "-", color="tab:green", lw=1.6, label=f"SplatNav: {s['status']}")
        col = "tab:orange" if f["status"] == "SUCCESS" else "tab:red"
        ax.plot(*P[1].T, "-", color=col, lw=1.6, label=f"FOCI: {f['status']}")
        if f.get("judge_failed_edge") is not None:
            J = None
            ax.annotate(f"fail on {f.get('judge_fail_location')}\n{f.get('judge_geometry_reason')}",
                        xy=(.02, .02), xycoords="axes fraction", fontsize=7, color=col)
            _ = J
        st = np.asarray(s["start_uv"])
        ax.add_patch(plt.Circle(st, meta["body"]["radius_m"], fill=False, color="b", lw=1))
        ax.plot(*st, "bo", ms=4)
        ax.plot(*np.asarray(s["goal_uv"]), "b*", ms=9)
        ax.set_xlim(lo[0], hi[0])
        ax.set_ylim(lo[1], hi[1])
        ax.set_aspect("equal")
        ax.set_title(f"{reg} {pid} (lateral clearance {pairs[pid]['lateral_clearance_m']} m)", fontsize=9)
        ax.set_xlabel("u [m]")
        ax.set_ylabel("v [m]")
        ax.legend(fontsize=7, loc="upper right")
    fig.suptitle("bl B1 pilot, cylinder (r 0.30 m, blue circle at start): grey = 2σ xy shadows of the judge's "
                 "Gaussians meeting the body slab (dots = centres; purple = floor-level, 2σ top < chassis bottom + 8 cm)", fontsize=9)
    fig.tight_layout()
    fig.savefig(a.out, dpi=130)
    print("wrote", a.out, [p[:2] for p in picks])


if __name__ == "__main__":
    main()
