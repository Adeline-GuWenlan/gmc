"""P3.6 fairness pack (round-5 review §7 / §5):

A. 2x2 positive-control matrix — {aligned, de-aligned(13mm,7deg)} x
   {w=0.90, w=1.10}, R_long_ellipse, full ladder: kills the "baselines
   merely cannot handle rotated doors" attack.
B. Pair-aware ProbeTree — the METHOD's tag-free pair information in the
   volumetric representation, NO continuation (policy "pair"), on the five
   de-aligned critical widths: isolates pair-information vs
   pair-information+continuation.
C. ambiguous_cut localization — for each non-reachable final tree: the
   start-FREE-component's ambiguous frontier, its size, gate-band and
   theta-window fractions, and cell-extent stats (diagnostic, declared).

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/p36_fairness.py
"""
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from splatc.datasets.g1_gate import (
    make_g1_scene, robot_library, gate_half_angle, Q_START, GOAL, GOAL_RADIUS)
from splatc.baselines.probe_methods import ProbeTree, _UF

TABDIR = os.path.join(os.path.dirname(__file__), "..", "..", "results", "tables")
BUDGETS = [500, 1000, 2000, 4000, 8000, 16000, 32000, 65536, 131072]
POLICIES = ["uniform", "generic", "contact", "pair"]
WS_MAIN = [0.505, 0.51, 0.52, 0.54, 0.58]
DEALIGN = (0.013, 7.0)
CONTROLS_2x2 = [
    (0.90, 0.0, 0.0), (0.90, 0.013, 7.0),
    (1.10, 0.0, 0.0), (1.10, 0.013, 7.0),
]


def cut_analysis(tree, scene, robot, w, q_start):
    """DIAGNOSTIC (declared): localize the ambiguous cut blocking the
    conservative connectivity — frontier of the start FREE component."""
    # round-7: exact face-overlap adjacency (was face-center single probe)
    uf = _UF()
    free = {k for k, r in tree.leaves.items() if r["status"] == "FREE"}
    for k in free:
        for nb in tree.face_adjacent_leaves(k):
            if nb in free:
                uf.union(k, nb)
    start = tree.locate(*q_start)
    if start not in free:
        return {"start_in_free": False}
    root = uf.find(start)
    comp = {k for k in free if uf.find(k) == root}
    frontier = set()
    for k in comp:
        for nb in tree.face_adjacent_leaves(k):
            if tree.leaves.get(nb, {}).get("status") == "AMBIG":
                frontier.add(nb)
    tilt = scene.meta["door_tilt"]
    off = scene.meta["door_offset"]
    ct, st = np.cos(tilt), np.sin(tilt)
    ga = gate_half_angle(robot.a, robot.b, w)
    n_band = n_band_th = 0
    dims = []
    for k in frontier:
        c = tree.cell_center(k)
        l = k[0]
        n = np.array(tree.ROOT) * (2 ** l)
        size = tree.span / n
        xf = ct * c[0] + st * (c[1] - off)
        yf = -st * c[0] + ct * (c[1] - off)
        if abs(xf) <= 0.6 and abs(yf) <= w / 2:
            n_band += 1
            dth = abs((c[2] - tilt + np.pi / 2) % np.pi - np.pi / 2)
            if dth <= ga:
                n_band_th += 1
        dims.append(size)
    dims = np.array(dims) if dims else np.zeros((0, 3))
    return {"start_in_free": True,
            "start_component_cells": len(comp),
            "frontier_cells": len(frontier),
            "frontier_in_gate_band": n_band,
            "frontier_in_band_and_theta": n_band_th,
            "frontier_cell_extent_median_mm_deg": (
                [round(float(np.median(dims[:, 0])) * 1000, 1),
                 round(float(np.median(dims[:, 1])) * 1000, 1),
                 round(float(np.degrees(np.median(dims[:, 2]))), 2)]
                if len(dims) else None)}


def run_one(w, dy0, tilt_deg, policy, kind):
    robot = robot_library()["R_long_ellipse"]
    scene = make_g1_scene(w, door_offset=dy0, door_tilt=np.radians(tilt_deg))
    t0 = time.time()
    tree = ProbeTree(scene, robot, policy)
    res = tree.run(BUDGETS, Q_START, GOAL, GOAL_RADIUS)
    solved = [b for b, r in sorted(res.items()) if r["reachable"]]
    row = {"kind": kind, "w": w, "dy0": dy0, "tilt_deg": tilt_deg,
           "policy": policy,
           "first_success_budget": solved[0] if solved else None,
           "queries_final": int(tree.queries),
           "pair_ops": int(scene.pair_ops_total()),
           "bp_hits": int(scene.bp_hits_total()),
           "n_leaves_final": len(tree.leaves),
           "results": {str(b): r for b, r in res.items()},
           "seconds": round(time.time() - t0, 1)}
    if not solved:
        row["cut"] = cut_analysis(tree, scene, robot, w, Q_START)
    print(f"{kind} w={w:.3f} dy={dy0:+.3f} t={tilt_deg:+.1f} {policy:8s} "
          f"first-success: {solved[0] if solved else 'NEVER'} "
          f"({row['seconds']}s)"
          + (f" cut: frontier {row['cut'].get('frontier_cells')} "
             f"band {row['cut'].get('frontier_in_gate_band')}/"
             f"{row['cut'].get('frontier_in_band_and_theta')}"
             if not solved and row["cut"].get("start_in_free") else ""),
          flush=True)
    return row


def main():
    rows = []
    for (w, dy0, tilt) in CONTROLS_2x2:
        for pol in POLICIES:
            rows.append(run_one(w, dy0, tilt, pol, "control_2x2"))
    for w in WS_MAIN:
        rows.append(run_one(w, DEALIGN[0], DEALIGN[1], "pair", "pair_main"))
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from prov import make_provenance
    out = {"provenance": make_provenance(
        __file__, "P3.6 fairness: 2x2 aligned/de-aligned wide controls + "
        "pair-aware ProbeTree mains + ambiguous-cut localization"),
        "budgets": BUDGETS, "rows": rows}
    os.makedirs(TABDIR, exist_ok=True)
    with open(os.path.join(TABDIR, "p36_fairness.json"), "w") as f:
        json.dump(out, f, indent=1,
                  default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print("-> results/tables/p36_fairness.json")


if __name__ == "__main__":
    main()
