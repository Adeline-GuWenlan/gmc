"""Figures for experiments A/B/C (reads results/tables/ridge_abc.json).

  ridge_expA.png : grid-seeding ladder — free bilateral seeds vs grid pitch
  ridge_expB.png : corrector recovery basin per width x variant
  ridge_expC.png : oracle-seed bidirectional tracks (xy) for all widths,
                   certified margins annotated (w=0.505 = delta 2.5mm case)

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/ridge_abc_plots.py
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Ellipse
from matplotlib.collections import PatchCollection

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))
from splatc.datasets.g1_gate import (
    make_g1_scene, robot_library, Q_START, GOAL, GOAL_RADIUS)

BASE = os.path.join(os.path.dirname(__file__), "..", "..")
TABDIR = os.path.join(BASE, "results", "tables")
FIGDIR = os.path.join(BASE, "results", "figures")
DY0, TILT_DEG = 0.013, 7.0


def plot_A(res, prov=None):
    fig, ax = plt.subplots(figsize=(7, 5))
    for a in res:
        pitches = [l["pitch_mm"] for l in a["ladder"]]
        frees = [l["n_free_bilateral"] for l in a["ladder"]]
        ax.plot(pitches, frees, "o-", label=f"w={a['w']:.3f}")
    ax.set_xscale("log")
    ax.set_yscale("symlog", linthresh=1)
    ax.invert_xaxis()
    ax.set_xlabel("seed grid pitch [mm]")
    ax.set_ylabel("# FREE bilateral candidates")
    ax.axhline(1, color="gray", ls=":", lw=0.8)
    ax.set_title("exp A: grid seeding needs <=250mm pitch (coarse 500mm grid\n"
                 "has ZERO free bilateral seeds for w<=0.54 -> argmax picks garbage)")
    ax.legend(fontsize=8)
    fig.tight_layout()
    if prov:
        from prov import stamp_figure
        stamp_figure(fig, prov)
    out = os.path.join(FIGDIR, "ridge_expA.png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out


def plot_B(res, prov=None):
    variants = ["single", "iterated", "newton_kkt"]
    fig, ax = plt.subplots(figsize=(7, 5))
    xs = np.arange(len(res))
    width = 0.25
    for j, v in enumerate(variants):
        vals = []
        for b in res:
            rr = [r for r in b["rows"] if r["corr"] == v]
            vals.append(100.0 * sum(r["recovered"] for r in rr) / len(rr))
        ax.bar(xs + (j - 1) * width, vals, width, label=v)
    ax.set_xticks(xs)
    ax.set_xticklabels([f"w={b['w']:.3f}" for b in res])
    ax.set_ylabel("recovered [%]  (perturbations up to ±100mm / ±15°)")
    ax.set_ylim(0, 105)
    ax.axhline(100, color="gray", ls=":", lw=0.8)
    ax.set_title("exp B: corrector recovery from perturbed ORACLE axis poses\n"
                 "(single-pass bisection+ternary is already near-total)")
    ax.legend()
    fig.tight_layout()
    if prov:
        from prov import stamp_figure
        stamp_figure(fig, prov)
    out = os.path.join(FIGDIR, "ridge_expB.png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out


def plot_C(res, prov=None):
    robot = robot_library()["R_long_ellipse"]
    n = len(res)
    fig, axes = plt.subplots(2, 3, figsize=(18, 9))
    for ax, c in zip(axes.ravel(), res):
        w = c["w"]
        scene = make_g1_scene(w, door_offset=DY0,
                              door_tilt=np.radians(TILT_DEG))
        for (R, centers), tags in zip(scene.disc_groups,
                                      scene.meta["group_tags"]):
            col = {1: "#8899aa", -1: "#aa8899", 0: "#666666"}
            pc = PatchCollection([Circle((p[0], p[1]), R) for p in centers],
                                 facecolor=[col.get(int(t), "#999")
                                            for t in tags],
                                 edgecolor="none", alpha=0.5)
            ax.add_collection(pc)
        ax.plot(*Q_START[:2], "g^", ms=10)
        ax.add_patch(Circle(GOAL, GOAL_RADIUS, fill=False, ec="green", lw=2))
        st = np.array(c["stations"])
        ax.plot(st[:, 0], st[:, 1], "r.-", lw=1.2, ms=4)
        for s in st[:: max(1, len(st) // 10)]:
            ax.add_patch(Ellipse((s[0], s[1]), 2 * robot.a, 2 * robot.b,
                                 angle=np.degrees(s[2]), fill=False,
                                 ec="red", lw=0.6, alpha=0.6))
        ok = c["status"] == "CERTIFIED_REACHABLE"
        ax.set_title(f"w={w:.3f}  {c['status']}\n"
                     f"cert_min={c['min_margin_mm']}mm  total={c['total']}q  "
                     f"stations {c['stations_bwd']}+{c['stations_fwd']}",
                     fontsize=10, color=("darkgreen" if ok else
                                         ("darkred" if w > 0.5 else "gray")))
        ax.set_xlim(-2.6, 2.6)
        ax.set_ylim(-1.6, 1.6)
        ax.set_aspect("equal")
    fig.suptitle("exp C: ORACLE mouth seed + bidirectional tracker -> "
                 "certified through-door paths down to delta=2.5mm (w=0.505); "
                 "closed instance w=0.49: SAFE ABSTENTION "
                 "(closure not certified)", fontsize=12)
    fig.tight_layout()
    if prov:
        from prov import stamp_figure
        stamp_figure(fig, prov)
    out = os.path.join(FIGDIR, "ridge_expC.png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out


def main():
    with open(os.path.join(TABDIR, "ridge_abc.json")) as f:
        res = json.load(f)
    os.makedirs(FIGDIR, exist_ok=True)
    prov = res.get("provenance", {})
    print(plot_A(res["A"], prov))
    print(plot_B(res["B"], prov))
    print(plot_C(res["C"], prov))


if __name__ == "__main__":
    main()
