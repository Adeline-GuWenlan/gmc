"""Diagnostic plots for ridge continuation station dumps (review-requested).

Per width, one 4-panel figure from results/tables/ridge_stations.json:
  (1) xy trajectory over wall discs, door, start/goal, robot footprints,
      certificate polyline, and metric-margin argmin;
  (2) x-theta with the ANALYTIC gate band (diagnostic overlay only — the
      tracker never reads door metadata);
  (3) per-station PW values h+, h-, B, m (dimensionless, RHO_CAP=0.25);
  (4) metric-margin profile (mm) along the certificate polyline (checker #2,
      diagnostic-only), zero line and argmin annotated.

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/ridge_diag_plots.py
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Circle
from matplotlib.collections import PatchCollection

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from splatc.datasets.g1_gate import (
    make_g1_scene, robot_library, gate_half_angle, Q_START, GOAL, GOAL_RADIUS)

BASE = os.path.join(os.path.dirname(__file__), "..", "..")
TABDIR = os.path.join(BASE, "results", "tables")
FIGDIR = os.path.join(BASE, "results", "figures")


def draw_scene(ax, scene):
    for (R, centers), tags in zip(scene.disc_groups,
                                  scene.meta["group_tags"]):
        col = {1: "#8899aa", -1: "#aa8899", 0: "#666666"}
        patches = [Circle((c[0], c[1]), R) for c in centers]
        pc = PatchCollection(patches, facecolor=[col.get(int(t), "#999999")
                                                 for t in tags],
                             edgecolor="none", alpha=0.55)
        ax.add_collection(pc)


def plot_case(w, d, robot, dy0=0.013, tilt_deg=7.0, prov=None):
    scene = make_g1_scene(w, door_offset=dy0,
                          door_tilt=np.radians(tilt_deg))
    fig, axes = plt.subplots(2, 2, figsize=(15, 11))
    fig.suptitle(f"ridge continuation diagnostic  w={w:.3f}  "
                 f"({d['terminated']})", fontsize=14)

    st = np.array(d["stations"]) if d["stations"] else np.zeros((0, 3))
    prof = d.get("profile")

    # ---- (1) xy ----------------------------------------------------------
    ax = axes[0, 0]
    draw_scene(ax, scene)
    ax.plot(*Q_START[:2], "g^", ms=12, label="start", zorder=5)
    ax.add_patch(Circle(GOAL, GOAL_RADIUS, fill=False, ec="green", lw=2))
    ax.plot(*GOAL, "g*", ms=12, label="goal", zorder=5)
    if d.get("seed_grid"):
        ax.plot(d["seed_grid"][0], d["seed_grid"][1], "b s", ms=10,
                mfc="none", mew=2, label="seed (grid)", zorder=6)
    if d.get("seed_polished"):
        ax.plot(d["seed_polished"][0], d["seed_polished"][1], "b*", ms=14,
                label="seed (polished)", zorder=6)
    if len(st):
        ax.plot(st[:, 0], st[:, 1], "r.-", lw=1.5, ms=6,
                label=f"stations (n={len(st)})", zorder=4)
        for i, s in enumerate(st):
            if i % max(1, len(st) // 8) == 0 or i == len(st) - 1:
                ax.add_patch(Ellipse((s[0], s[1]), 2 * robot.a, 2 * robot.b,
                                     angle=np.degrees(s[2]), fill=False,
                                     ec="red", lw=0.8, alpha=0.7))
                ax.annotate(str(i), (s[0], s[1]), fontsize=7,
                            textcoords="offset points", xytext=(4, 4))
    if prof:
        wps = np.array(prof["wps"])
        ax.plot(wps[:, 0], wps[:, 1], "-", color="orange", lw=1,
                alpha=0.8, label="cert polyline", zorder=3)
        am = prof["argmin"]
        ax.plot(am["pose"][0], am["pose"][1], "kx", ms=14, mew=3,
                label=f"cert argmin {am['margin_m']*1000:.1f}mm", zorder=7)
        ax.add_patch(Ellipse((am["pose"][0], am["pose"][1]),
                             2 * robot.a, 2 * robot.b,
                             angle=np.degrees(am["pose"][2]), fill=False,
                             ec="black", lw=1.5, ls="--", zorder=7))
    ax.set_xlim(-3.6, 3.6)
    ax.set_ylim(-2.4, 2.4)
    ax.set_aspect("equal")
    ax.legend(fontsize=8, loc="lower right")
    ax.set_title("xy trajectory (walls, door, start/goal)")

    # ---- (2) x - theta ---------------------------------------------------
    ax = axes[0, 1]
    ga = np.degrees(gate_half_angle(robot.a, robot.b, w))
    if ga > 0:
        ax.axhspan(tilt_deg - ga, tilt_deg + ga, color="green", alpha=0.15,
                   label=f"analytic gate band ±{ga:.1f}° (diag overlay)")
    ax.axhline(tilt_deg, color="green", lw=0.8, ls="--")
    ax.axvspan(-0.6, 0.6, color="gray", alpha=0.12, label="wall x-extent")
    if prof:
        smp = np.array(prof["samples"])
        ax.plot(smp[:, 2], np.degrees(((smp[:, 4] + np.pi / 2)
                                       % np.pi) - np.pi / 2),
                "-", color="orange", lw=1, alpha=0.8, label="cert polyline")
    if len(st):
        th_wrapped = np.degrees(((st[:, 2] + np.pi / 2) % np.pi) - np.pi / 2)
        ax.plot(st[:, 0], th_wrapped, "r.-", lw=1.5, ms=6, label="stations")
    # P1b gate chart: section components as vertical theta-interval bars
    wr = lambda t: np.degrees(((t + np.pi / 2) % np.pi) - np.pi / 2)
    trig_col = {"seed": "tab:blue", "multimodal": "tab:purple",
                "reject": "tab:red", "backtrack": "tab:orange"}
    seen_trig = set()
    for sec in d.get("chart", []):
        colr = trig_col.get(sec["trigger"], "gray")
        lbl = None
        if sec["trigger"] not in seen_trig:
            lbl = f"section ({sec['trigger']})"
            seen_trig.add(sec["trigger"])
        if not sec["components"]:
            ax.plot(sec["x"], wr(sec["anchor_th"]), "x", color=colr,
                    ms=7, mew=2, label=lbl)
            continue
        for cc in sec["components"]:
            lo, hi = wr(cc["th_lo"]), wr(cc["th_hi"])
            if lo <= hi:
                ax.plot([sec["x"], sec["x"]], [lo, hi], "-", color=colr,
                        lw=3, alpha=0.6, solid_capstyle="butt", label=lbl)
            else:                              # wrapped interval: split
                ax.plot([sec["x"], sec["x"]], [lo, 90], "-", color=colr,
                        lw=3, alpha=0.6, label=lbl)
                ax.plot([sec["x"], sec["x"]], [-90, hi], "-", color=colr,
                        lw=3, alpha=0.6)
            lbl = None
    n_sw = sum(1 for e in d.get("events", [])
               if e["kind"] == "branch_switch")
    ax.plot(Q_START[0], 90.0, "g^", ms=10)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("theta [deg, wrapped to (-90,90]]")
    ax.legend(fontsize=8)
    ax.set_title("x–theta with gate-chart sections"
                 + (f"  ({n_sw} certified branch switch(es))" if n_sw
                    else ""))

    # ---- (3) per-visit PW values (both marches, sequence order) ----------
    ax = axes[1, 0]
    vis = d["visits"]
    if vis:
        idx = np.arange(len(vis))
        ax.plot(idx, [v["hp"] for v in vis], ".-", label="h+ (upper)")
        ax.plot(idx, [v["hm"] for v in vis], ".-", label="h- (lower)")
        ax.plot(idx, [v["m"] for v in vis], "k.-", lw=2, label="m=min(h+,h-)")
        ax.plot(idx, [v["B"] for v in vis], ".-", color="purple",
                label="B=h+-h-")
        retry = [i for i, v in enumerate(vis) if v["retry"]]
        if retry:
            ax.plot(retry, [vis[i]["m"] for i in retry], "rx", ms=8,
                    label="retry visit")
        marches = [v.get("march", 0) for v in vis]
        for i in range(1, len(vis)):
            if marches[i] != marches[i - 1]:
                ax.axvline(i - 0.5, color="blue", lw=1, ls="--", alpha=0.6)
                ax.text(i, 0.24, "march 2", fontsize=7, color="blue")
    for r in d["rejects"]:
        ax.axvline(r.get("visit_i", r.get("station", 0)), color="red",
                   alpha=0.2, lw=3)
    for b in d["balance_no_bracket"]:
        ax.axvline(b.get("visit_i", b.get("station", 0)), color="orange",
                   alpha=0.4, lw=1, ls=":")
    ax.axhline(0.25, color="gray", ls="--", lw=0.8)
    ax.text(0.1, 0.252, "RHO_CAP", fontsize=7, color="gray")
    ax.axhline(0.22, color="gray", ls=":", lw=0.8)
    ax.axhline(0.0, color="black", lw=0.8)
    ax.set_xlabel("visit sequence  (red band = predictor reject, "
                  "orange dotted = balance no-bracket)")
    ax.set_ylabel("PW value [dimensionless]")
    ax.legend(fontsize=8)
    ax.set_title("per-visit PW h± / B / m (march 1 then march 2)")

    # ---- (4) metric margin profile --------------------------------------
    ax = axes[1, 1]
    if prof:
        smp = np.array(prof["samples"])
        ax.plot(smp[:, 0], smp[:, 5] * 1000, "-", lw=1, color="steelblue")
        ax.axhline(0, color="black", lw=0.8)
        am = prof["argmin"]
        ax.plot(am["s"], am["margin_m"] * 1000, "kx", ms=12, mew=3)
        ax.annotate(f"min {am['margin_m']*1000:.2f}mm @ seg {am['seg']}",
                    (am["s"], am["margin_m"] * 1000), fontsize=9,
                    textcoords="offset points", xytext=(10, -12))
        # waypoint boundaries of the prefix (start, rotate, seed = segs 0-1)
        wps = np.array(prof["wps"])
        s_acc, bounds = 0.0, [0.0]
        for p, qn in zip(wps[:-1], wps[1:]):
            dth = np.arctan2(np.sin(qn[2] - p[2]), np.cos(qn[2] - p[2]))
            s_acc += np.hypot(qn[0] - p[0], qn[1] - p[1]) \
                + robot.a * abs(dth)
            bounds.append(s_acc)
        for b in bounds[:4]:
            ax.axvline(b, color="gray", lw=0.6, ls=":")
        ax.set_title(f"metric margin along {prof['kind']} (checker #2, "
                     f"diagnostic)")
    else:
        ax.set_title("no profile (NO_SEED)")
    ax.set_xlabel("arc parameter s = |dp| + a|dθ| [m]")
    ax.set_ylabel("metric margin [mm]")

    fig.tight_layout()
    if prov:
        from prov import stamp_figure
        stamp_figure(fig, prov)
    out = os.path.join(FIGDIR, f"ridge_diag_w{w:.3f}.png")
    fig.savefig(out, dpi=110)
    if prov:
        from prov import write_sidecar
        write_sidecar(out, prov)
    plt.close(fig)
    return out


def main():
    with open(os.path.join(TABDIR, "ridge_stations.json")) as f:
        diags = json.load(f)
    prov = diags.pop("_provenance", {})
    robot = robot_library()["R_long_ellipse"]
    os.makedirs(FIGDIR, exist_ok=True)
    for wkey, d in diags.items():
        out = plot_case(float(wkey), d, robot, prov=prov)
        print(out)


if __name__ == "__main__":
    main()
