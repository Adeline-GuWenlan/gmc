"""P3 matched-budget baselines — the formal Gate B experiment (review
round-3 P3).

Same code tree, same checker, same billing as the v4.1 stack.  Arms:
Uniform SE(2) / Generic Adaptive / contact-v4 (round-1 FROZEN scalar-cue
probe — kept as the historical contact-probe arm, not the method) via the
frozen ProbeTree.run, on the five de-aligned critical widths, budget
ladder extended to 131072 (uncensoring the round-2 curve).  The
continuation-v4.1 reference totals are READ from ridge_continuation.json
(same code version; not re-run here).  The oracle-tube hybrid floor
(lower-bound failure curve) re-runs separately via floor_probe.py.

Gate B criterion (02/01 freeze): the contact-guided method must beat
matched-budget Uniform and Generic Adaptive on at least one critical gate,
under the same evaluator.  Metrics recorded per (w, policy, budget):
reachable verdict, false-unreachable flag (all instances analytically
REACHABLE), final representation size (leaves) and max depth.

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/p3_matched_budget.py
"""
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from splatc.datasets.g1_gate import (
    make_g1_scene, robot_library, gate_half_angle, Q_START, GOAL, GOAL_RADIUS)
from splatc.baselines.probe_methods import ProbeTree
from p36_fairness import cut_analysis

WS = [0.505, 0.51, 0.52, 0.54, 0.58]
DEALIGN = (0.013, 7.0)
BUDGETS = [500, 1000, 2000, 4000, 8000, 16000, 32000, 65536, 131072]
POLICIES = ["uniform", "generic", "contact"]
# P3.5 positive controls (review round-4 §2.1): the evaluator must succeed
# somewhere easy, else all-NEVER results are uninterpretable.  Aligned
# doors, generous widths, small ladder.
POS_CONTROLS = [
    (1.10, 0.0, 0.0, "R_small_circle"),
    (1.10, 0.0, 0.0, "R_long_ellipse"),
    (0.70, 0.0, 0.0, "R_small_circle"),
]
POS_BUDGETS = [500, 1000, 2000, 4000, 8000, 16000, 32000]
TABDIR = os.path.join(os.path.dirname(__file__), "..", "..", "results", "tables")


def continuation_reference():
    """v4.1 totals per width from the canonical sweep (same code tree)."""
    with open(os.path.join(TABDIR, "ridge_continuation.json")) as f:
        d = json.load(f)
    ref = {}
    for r in d["rows"]:
        if r.get("status") == "CERTIFIED_REACHABLE":
            ref[round(r["w"], 4)] = {
                "total": r["total"], "min_margin_mm": r["min_margin_mm"],
                "provenance_script_sha":
                    d["provenance"]["script_sha256"]}
    return ref


def gate_band_stats(tree, scene, robot, w):
    """DIAGNOSTIC (declared): how much of the evaluated representation ever
    visited the door corridor band / the analytic passable theta band."""
    tilt = scene.meta["door_tilt"]
    off = scene.meta["door_offset"]
    ct, st = np.cos(tilt), np.sin(tilt)
    ga = gate_half_angle(robot.a, robot.b, w)
    n_band = n_band_theta = 0
    for k in tree.leaves:
        c = tree.cell_center(k)
        yf = -st * c[0] + ct * (c[1] - off)
        xf = ct * c[0] + st * (c[1] - off)
        if abs(xf) <= 0.6 and abs(yf) <= w / 2:
            n_band += 1
            dth = abs((c[2] - tilt + np.pi / 2) % np.pi - np.pi / 2)
            if dth <= ga:
                n_band_theta += 1
    return {"cells_in_gate_band": n_band,
            "cells_in_band_and_theta": n_band_theta}


def run_one(w, dy0, tilt_deg, robot_id, budgets, ref=None):
    robot = robot_library()[robot_id]
    scene = make_g1_scene(w, door_offset=dy0, door_tilt=np.radians(tilt_deg))
    t0 = time.time()
    tree = ProbeTree(scene, robot, run_one.policy)
    res = tree.run(budgets, Q_START, GOAL, GOAL_RADIUS)
    solved = [b for b, r in sorted(res.items()) if r["reachable"]]
    row = {"w": w, "dy0": dy0, "tilt_deg": tilt_deg, "robot": robot_id,
           "policy": run_one.policy,
           "gate_half_angle_deg": float(np.degrees(
               gate_half_angle(robot.a, robot.b, w))),
           "first_success_budget": solved[0] if solved else None,
           "results": {str(b): r for b, r in res.items()},
           "n_leaves_final": len(tree.leaves),
           "queries_final": int(tree.queries),
           "pair_ops": int(scene.pair_ops_total()),
           "bp_hits": int(scene.bp_hits_total()),
           "seconds": round(time.time() - t0, 1)}
    row.update(gate_band_stats(tree, scene, robot, w))
    if not solved:
        # round-5 §7 P3.6-B: localize the ambiguous cut for every failed
        # main run (frontier of the start FREE component; diagnostic)
        row["cut"] = cut_analysis(tree, scene, robot, w, Q_START)
    return row, solved


def main():
    ref = continuation_reference()
    rows = []
    controls = []
    for (w, dy0, tilt_deg, rid) in POS_CONTROLS:
        for pol in POLICIES:
            run_one.policy = pol
            row, solved = run_one(w, dy0, tilt_deg, rid, POS_BUDGETS)
            row["kind"] = "positive_control"
            controls.append(row)
            print(f"CONTROL w={w:.2f} {rid} {pol:8s} first-success: "
                  f"{solved[0] if solved else 'NEVER'} "
                  f"({row['seconds']}s)", flush=True)
    for w in WS:
        for pol in POLICIES:
            run_one.policy = pol
            row, solved = run_one(w, DEALIGN[0], DEALIGN[1],
                                  "R_long_ellipse", BUDGETS)
            rows.append(row)
            fs = (solved[0] if solved
                  else "NEVER(actual %d)" % row["queries_final"])
            print(f"w={w:.3f} {pol:8s} first-success: {fs} "
                  f"(cont ref {ref.get(round(w,4),{}).get('total')}; "
                  f"{row['seconds']}s, leaves {row['n_leaves_final']}, "
                  f"pair_ops {row['pair_ops']}, "
                  f"band {row['cells_in_gate_band']}/"
                  f"{row['cells_in_band_and_theta']})",
                  flush=True)
    summary = []
    for w in WS:
        cont = ref.get(round(w, 4), {}).get("total")
        line = {"w": w, "continuation_total": cont}
        for pol in POLICIES:
            r = next(x for x in rows if x["w"] == w and x["policy"] == pol)
            line[pol] = r["first_success_budget"]
        summary.append(line)
        print("GATE_B", line, flush=True)
    from prov import make_provenance
    out = {"provenance": make_provenance(
        __file__, "P3.5 matched-budget: uniform/generic/contact-v4 ladder "
        "to nominal 131072 (actual recorded per-run) + positive controls "
        "+ gate-band visitation + three-layer accounting; "
        "continuation-v4.1 reference"),
        "budgets": BUDGETS, "pos_budgets": POS_BUDGETS,
        "summary": summary, "positive_controls": controls, "rows": rows}
    os.makedirs(TABDIR, exist_ok=True)
    with open(os.path.join(TABDIR, "p3_matched_budget.json"), "w") as f:
        json.dump(out, f, indent=1,
                  default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print("-> results/tables/p3_matched_budget.json")


if __name__ == "__main__":
    main()
