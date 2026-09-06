"""Sprint B decisive early experiment (01 Sprint B / Gate B):
matched-budget probing — uniform vs generic-adaptive vs contact-cues.

All instances are de-aligned (offset 13mm, tilt 7 deg) and analytically
REACHABLE; the measured quantity is the collision-query budget each policy
needs before it stops reporting false-unreachable.

Billing contract: every cell-center evaluation costs 1 query for every policy;
the contact policy's per-side decomposition comes from the same pose
evaluation (pair identities are free by-products of the pairwise h_ij pass).

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/probe_pareto.py
"""
import json
import os
import sys
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from splatc.datasets.g1_gate import (
    make_g1_scene, robot_library, gate_half_angle, Q_START, GOAL, GOAL_RADIUS)
from splatc.baselines.probe_methods import ProbeTree

WS = [0.505, 0.51, 0.52, 0.54, 0.58]
DEALIGN = (0.013, 7.0)  # offset m, tilt deg
BUDGETS = [500, 1000, 2000, 4000, 8000, 16000, 32000, 64000]
POLICIES = ["uniform", "generic", "contact"]
FIGDIR = os.path.join(os.path.dirname(__file__), "..", "..", "results", "figures")
TABDIR = os.path.join(os.path.dirname(__file__), "..", "..", "results", "tables")


def main():
    robot = robot_library()["R_long_ellipse"]
    rows = []
    for w in WS:
        scene = make_g1_scene(w, door_offset=DEALIGN[0],
                              door_tilt=np.radians(DEALIGN[1]))
        for pol in POLICIES:
            t0 = time.time()
            tree = ProbeTree(scene, robot, pol)
            res = tree.run(BUDGETS, Q_START, GOAL, GOAL_RADIUS)
            row = {"w": w, "policy": pol,
                   "gate_half_angle_deg":
                       float(np.degrees(gate_half_angle(robot.a, robot.b, w))),
                   "results": {str(b): r for b, r in res.items()},
                   "seconds": round(time.time() - t0, 1)}
            rows.append(row)
            solved = [b for b, r in sorted(res.items()) if r["reachable"]]
            print(f"w={w:.3f} {pol:8s} first-success budget: "
                  f"{solved[0] if solved else 'NEVER'}  "
                  f"({row['seconds']}s)", flush=True)

    os.makedirs(TABDIR, exist_ok=True)
    with open(os.path.join(TABDIR, "probe_pareto.json"), "w") as f:
        json.dump(rows, f, indent=2)

    # figure 1: queries-to-first-success vs gate width per policy
    fig, ax = plt.subplots(figsize=(8, 4.6))
    colors = {"uniform": "tab:red", "generic": "tab:orange",
              "contact": "tab:green"}
    for pol in POLICIES:
        xs, ys, never_x = [], [], []
        for w in WS:
            row = next(r for r in rows if r["w"] == w and r["policy"] == pol)
            solved = [int(b) for b, r in row["results"].items()
                      if r["reachable"]]
            if solved:
                xs.append(w)
                ys.append(min(solved))
            else:
                never_x.append(w)
        ax.plot(xs, ys, "o-", color=colors[pol], label=pol)
        for w in never_x:
            ax.scatter([w], [BUDGETS[-1] * 1.6], marker="x", s=90,
                       color=colors[pol])
    ax.axhline(BUDGETS[-1], color="0.6", ls=":", lw=1)
    ax.text(WS[0], BUDGETS[-1] * 1.65, "x = never within budget", fontsize=8)
    ax.set_yscale("log")
    ax.set_xlabel("door width w (m)  [all analytically reachable; "
                  "de-aligned offset 13mm + tilt 7°]")
    ax.set_ylabel("collision queries to first correct REACHABLE")
    ax.set_title("matched-budget probing: queries-to-success vs gate width")
    ax.legend()
    fig.tight_layout()
    os.makedirs(FIGDIR, exist_ok=True)
    fig.savefig(os.path.join(FIGDIR, "probe_pareto.png"), dpi=130)

    # figure 2: success curve vs budget aggregated over widths
    fig2, ax2 = plt.subplots(figsize=(7, 4))
    for pol in POLICIES:
        succ = []
        for b in BUDGETS:
            n_ok = sum(1 for r in rows if r["policy"] == pol
                       and r["results"][str(b)]["reachable"])
            succ.append(n_ok / len(WS))
        ax2.plot(BUDGETS, succ, "o-", color=colors[pol], label=pol)
    ax2.set_xscale("log")
    ax2.set_xlabel("collision-query budget")
    ax2.set_ylabel(f"fraction of {len(WS)} widths solved")
    ax2.set_title("gate recall vs query budget (de-aligned critical G1)")
    ax2.legend()
    fig2.tight_layout()
    fig2.savefig(os.path.join(FIGDIR, "probe_success_vs_budget.png"), dpi=130)
    print("probe_pareto.json + 2 figures written")


if __name__ == "__main__":
    main()
