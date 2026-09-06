"""Thin-gate sweep (Sprint A / Gate A criterion 3, and the seed of the
Claim C primary-endpoint curve).

For door widths w sweeping across the critical width 2b:
  1. measured gate angular interval at the door mid-plane (dense theta)
     vs the analytic corridor formula;
  2. full-oracle reachability per resolution -> where does each resolution
     first report false-unreachable while the door is still physically open?

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/thin_gate/gate_sweep.py
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
from splatc.reference.oracle import (
    make_grid, RESOLUTIONS, dense_oracle, reachability, gate_interval_numeric)

WS = [0.45, 0.48, 0.50, 0.505, 0.51, 0.52, 0.54, 0.58, 0.62, 0.70,
      0.85, 1.00, 1.20, 1.30]
RES_FOR_REACH = ["coarse", "medium", "fine"]
FIGDIR = os.path.join(os.path.dirname(__file__), "..", "..", "results", "figures")
TABDIR = os.path.join(os.path.dirname(__file__), "..", "..", "results", "tables")


def measured_half_angle(passable, thetas):
    """Half-width of the passable interval around theta=0 (mod pi)."""
    if not passable.any():
        return 0.0
    if passable.all():
        return np.pi / 2
    # fold to [-pi/2, pi/2) around 0
    th = np.where(thetas >= np.pi, thetas - 2 * np.pi, thetas)
    fold = np.abs(np.where(np.abs(th) > np.pi / 2, np.pi - np.abs(th), th))
    return float(np.max(fold[passable]))


def main():
    os.makedirs(FIGDIR, exist_ok=True)
    os.makedirs(TABDIR, exist_ok=True)
    robot = robot_library()["R_long_ellipse"]
    a, b = robot.a, robot.b
    rows = []
    for w in WS:
        scene = make_g1_scene(w)
        ana = gate_half_angle(a, b, w)
        passable, thetas = gate_interval_numeric(scene, robot, w)
        num = measured_half_angle(passable, thetas)
        row = {"w": w, "analytic_half_angle_deg": np.degrees(ana),
               "numeric_half_angle_deg": np.degrees(num),
               "physically_open": w > 2 * b, "reach": {}}
        for res in RES_FOR_REACH:
            if res == "fine" and not (0.45 <= w <= 0.7):
                continue  # fine resolution only needed near the critical region
            dx, ntheta = RESOLUTIONS[res]
            grid = make_grid(scene.workspace, dx, ntheta)
            t0 = time.time()
            free, _ = dense_oracle(scene, robot, grid)
            reach, _, ncomp, _ = reachability(free, grid, Q_START, GOAL, GOAL_RADIUS)
            row["reach"][res] = {"reachable": bool(reach), "components": int(ncomp),
                                 "seconds": round(time.time() - t0, 1)}
        rows.append(row)
        print(f"w={w:.3f} open={row['physically_open']} "
              f"ana={row['analytic_half_angle_deg']:.2f}° "
              f"num={row['numeric_half_angle_deg']:.2f}° "
              + " ".join(f"{r}:{'R' if v['reachable'] else 'U'}"
                         for r, v in row["reach"].items()), flush=True)

    with open(os.path.join(TABDIR, "gate_sweep.json"), "w") as f:
        json.dump(rows, f, indent=2)

    # -- figure: interval curve + reachability ladder
    ws = np.array([r["w"] for r in rows])
    ana = np.array([r["analytic_half_angle_deg"] for r in rows])
    num = np.array([r["numeric_half_angle_deg"] for r in rows])
    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(8.5, 6.4), sharex=True,
        gridspec_kw={"height_ratios": [3, 1.4]})
    wf = np.linspace(0.42, 1.35, 400)
    af = np.degrees([gate_half_angle(a, b, x) for x in wf])
    ax1.plot(wf, af, "g-", lw=1.5, label="analytic Θ(w) (corridor)")
    ax1.plot(ws, num, "ko", ms=5, mfc="none", label="oracle mid-plane measurement")
    ax1.axvline(2 * b, color="r", ls="--", lw=1, label="physical closure w*=2b")
    ax1.axvline(2 * a, color="b", ls=":", lw=1, label="fully open w=2a")
    ax1.set_ylabel("gate half-angle (deg)")
    ax1.legend(fontsize=8)
    ax1.set_title("G1 thin-gate sweep — R_long_ellipse (a=0.6, b=0.25)")
    for i, res in enumerate(RES_FOR_REACH):
        xs_r, ys_r, cs = [], [], []
        for r in rows:
            if res in r["reach"]:
                xs_r.append(r["w"])
                ys_r.append(i)
                cs.append("tab:green" if r["reach"][res]["reachable"] else "tab:red")
        ax2.scatter(xs_r, ys_r, c=cs, s=42, marker="s")
    ax2.axvline(2 * b, color="r", ls="--", lw=1)
    ax2.set_yticks(range(len(RES_FOR_REACH)))
    ax2.set_yticklabels(RES_FOR_REACH)
    ax2.set_ylim(-0.6, len(RES_FOR_REACH) - 0.4)
    ax2.set_xlabel("door width w (m)")
    ax2.set_title("oracle reachability by resolution (green=R, red=U)", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGDIR, "gate_sweep.png"), dpi=130)
    print("gate_sweep.json + gate_sweep.png written")


if __name__ == "__main__":
    main()
