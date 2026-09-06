"""De-alignment demo: does breaking grid-phase alignment expose
false-unreachable at finite resolution?  (Sprint A finding #2 payoff;
seed of the Claim C measurement.)

All cases are analytically REACHABLE (w > 2b = 0.5).  A resolution that
reports U is producing a false-unreachable.

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/thin_gate/dealign_demo.py
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
    make_grid, RESOLUTIONS, dense_oracle, reachability)

WS = [0.502, 0.505, 0.51, 0.52, 0.54]
CONFIGS = [("aligned", 0.0, 0.0), ("offset", 0.013, 0.0),
           ("tilt", 0.0, 7.0), ("offset+tilt", 0.013, 7.0)]
FIGDIR = os.path.join(os.path.dirname(__file__), "..", "..", "results", "figures")
TABDIR = os.path.join(os.path.dirname(__file__), "..", "..", "results", "tables")


def main():
    robot = robot_library()["R_long_ellipse"]
    rows = []
    for w in WS:
        for cname, dy0, tilt_deg in CONFIGS:
            scene = make_g1_scene(w, door_offset=dy0,
                                  door_tilt=np.radians(tilt_deg))
            row = {"w": w, "config": cname, "door_offset": dy0,
                   "door_tilt_deg": tilt_deg,
                   "analytic_half_angle_deg":
                       float(np.degrees(gate_half_angle(robot.a, robot.b, w))),
                   "reach": {}}
            for res in ("coarse", "medium", "fine"):
                if res == "fine" and w > 0.51:
                    continue
                dx, ntheta = RESOLUTIONS[res]
                grid = make_grid(scene.workspace, dx, ntheta)
                t0 = time.time()
                free, _ = dense_oracle(scene, robot, grid)
                reach, _, ncomp, _ = reachability(free, grid, Q_START,
                                                  GOAL, GOAL_RADIUS)
                row["reach"][res] = {"reachable": bool(reach),
                                     "components": int(ncomp),
                                     "seconds": round(time.time() - t0, 1)}
            rows.append(row)
            print(f"w={w:.3f} {cname:12s} "
                  + " ".join(f"{r}:{'R' if v['reachable'] else 'U'}"
                             for r, v in row["reach"].items()), flush=True)

    os.makedirs(TABDIR, exist_ok=True)
    with open(os.path.join(TABDIR, "dealign_demo.json"), "w") as f:
        json.dump(rows, f, indent=2)

    # figure: config rows x width columns, marker per resolution
    fig, ax = plt.subplots(figsize=(9, 4.2))
    res_names = ["coarse", "medium", "fine"]
    for ci, (cname, _, _) in enumerate(CONFIGS):
        for wi, w in enumerate(WS):
            row = next(r for r in rows if r["w"] == w and r["config"] == cname)
            for ri, res in enumerate(res_names):
                if res not in row["reach"]:
                    continue
                ok = row["reach"][res]["reachable"]
                ax.scatter(wi + ri * 0.22 - 0.22, ci,
                           marker="s", s=120,
                           c="tab:green" if ok else "tab:red",
                           edgecolors="k", linewidths=0.4)
    ax.set_yticks(range(len(CONFIGS)))
    ax.set_yticklabels([c[0] for c in CONFIGS])
    ax.set_xticks(range(len(WS)))
    ax.set_xticklabels([f"w={w}" for w in WS])
    ax.set_title("false-unreachable onset under de-alignment "
                 "(all cases analytically reachable; per width: "
                 "coarse|medium|fine)\nred = false-unreachable")
    fig.tight_layout()
    os.makedirs(FIGDIR, exist_ok=True)
    fig.savefig(os.path.join(FIGDIR, "dealign_demo.png"), dpi=130)
    print("dealign_demo.json + dealign_demo.png written")


if __name__ == "__main__":
    main()
