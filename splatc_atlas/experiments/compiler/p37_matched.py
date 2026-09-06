"""P3.7 symmetry/domain-matched baselines (round-6 review — the residual
life-or-death experiment).

Round-6 finding: the P3 ProbeTree baselines searched (7.0 x 4.6)m x 2*pi
while the continuation seed box is (6.0 x 3.6)m x pi — a 2.98x domain
volume mismatch, with the [0, pi) theta quotient used by the continuation
only.  This experiment gives every volumetric arm the SAME domain:

  ROI        = (-3, 3) x (-1.8, 1.8)      (the continuation seed box)
  theta      = [0, pi) quotient           (sound: centrally symmetric
                                           robots only — asserted below by
                                           direct pi-periodicity replay)
  arms:
    iso_uniform_q    ProbeTree  uniform policy, matched ROI + quotient
    aniso_uniform_q  AnisoTree  breadth-first by generation
    aniso_generic_q  AnisoTree  |rho| scalar-clearance ordering
    aniso_pair_q     AnisoTree  tag-free pair (ha, hb) ordering (the
                                METHOD's cue family, volumetric, no
                                continuation)
  budgets    = same pose ladder as P3 (500 .. 131072); pair_ops / bp_hits
               snapshotted at every checkpoint (three-currency reporting);
               wall time recorded per run.

All arms share the coarse open-space certification boost (rho > 0.15 at
coarse generations), mirroring the P3 ProbeTree convention.  AnisoTree
scoring uses generation g = (lx+ly+lk)/3 so depth penalties are on the
ProbeTree level scale.  Certification, billing and the conservative
reachable semantics are the frozen ones; `reachable=false` remains a
budgeted false negative, never an unreachable certificate.

KILL CONDITION (accepted, round-6): if a matched anisotropic arm reaches
first-success within ~2x of the continuation's certified totals, thin-gate
query efficiency cannot be claimed as the main residual (the residual
claim shifts to multi-goal reuse / morphology update / topology
representation).

Gate-factored (chart-conditioned) arms are NOT here: they need certified
OpenChart frontiers and belong to the chart-factored Gate B benchmark
(P4a dependency).

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/p37_matched.py
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
from splatc.baselines.aniso_tree import AnisoTree
from p3_matched_budget import gate_band_stats, continuation_reference

ROI = (-3.0, 3.0, -1.8, 1.8)          # continuation seed box, verbatim
THETA_SPAN = float(np.pi)             # [0, pi) quotient
WS = [0.505, 0.51, 0.52, 0.54, 0.58]
DEALIGN = (0.013, 7.0)
BUDGETS = [500, 1000, 2000, 4000, 8000, 16000, 32000, 65536, 131072]
POS_CONTROLS = [
    # full 2 robots x {aligned, de-aligned} grid (round-7: the report said
    # "两机器人 x aligned/de-aligned" while only three combos ran)
    (1.10, 0.0, 0.0, "R_long_ellipse"),
    (1.10, 0.0, 0.0, "R_small_circle"),
    (1.10, 0.013, 7.0, "R_long_ellipse"),
    (1.10, 0.013, 7.0, "R_small_circle"),
]
POS_BUDGETS = [500, 1000, 2000, 4000, 8000, 16000, 32000]
ARMS = ["iso_uniform_q", "aniso_uniform_q", "aniso_generic_q",
        "aniso_pair_q"]
TABDIR = os.path.join(os.path.dirname(__file__), "..", "..",
                      "results", "tables")


def assert_pi_periodic(w, dy0, tilt_deg, robot_id, n=120):
    """The [0, pi) quotient is sound iff collision status is pi-periodic.
    Verified by direct replay on a THROWAWAY scene (billing isolated)."""
    scene = make_g1_scene(w, door_offset=dy0, door_tilt=np.radians(tilt_deg))
    robot = robot_library()[robot_id]
    xs = np.linspace(ROI[0] + 0.05, ROI[1] - 0.05, 8)
    ys = np.linspace(ROI[2] + 0.05, ROI[3] - 0.05, 5)
    ths = np.linspace(0.0, np.pi, 3, endpoint=False)
    for x in xs:
        for y in ys:
            for th in ths:
                a, ra = scene.check_pose(robot, (x, y, th))
                b, rb = scene.check_pose(robot, (x, y, th + np.pi))
                assert a == b and abs(ra - rb) < 1e-9, \
                    f"pi-periodicity violated at {(x, y, th)}: " \
                    f"{(a, ra)} vs {(b, rb)}"


# ---- AnisoTree scoring (new matched arms; generation-scale depth term) ----

def _gen(key):
    return (key[0] + key[1] + key[2]) / 3.0


def _boost(rec, g, score):
    # coarse open-space certification branch, mirroring ProbeTree._push
    if rec["rho"] > 0.15 and g <= 2:
        return min(score, 0.6 + 0.6 * g)
    return score


def score_uniform(key, rec, tree):
    g = _gen(key)
    return _boost(rec, g, g)


def score_generic(key, rec, tree):
    g = _gen(key)
    return _boost(rec, g, abs(rec["rho"]) + 0.25 * g)


def score_pair(key, rec, tree):
    g = _gen(key)
    ha = rec.get("pair_ha", np.inf)
    hb = rec.get("pair_hb", np.inf)
    m = min(ha, hb)
    if max(ha, hb) < 0.2 and m > 0:
        s = abs(ha - hb) + 0.25 * g
    elif max(ha, hb) < 0.2:
        s = 0.5 + abs(m) + 0.25 * g
    else:
        s = 1.0 + abs(rec["rho"]) + 0.25 * g
    return _boost(rec, g, s)


def make_tree(arm, scene, robot):
    if arm == "iso_uniform_q":
        return ProbeTree(scene, robot, "uniform",
                         roi=ROI, theta_span=THETA_SPAN)
    fn = {"aniso_uniform_q": score_uniform,
          "aniso_generic_q": score_generic,
          "aniso_pair_q": score_pair}[arm]
    return AnisoTree(scene, robot, fn, roi=ROI, theta_span=THETA_SPAN,
                     pair_info=(arm == "aniso_pair_q"))


# ---- tree-agnostic cut localization (p36 cut_analysis assumes ProbeTree
# keys; this version asks the tree for its own cell size) ------------------

def _cell_size(tree, key):
    if hasattr(tree, "cell_size"):
        return tree.cell_size(key)
    n = np.array(tree.ROOT) * (2 ** key[0])
    return tree.span / n


def cut_analysis_any(tree, scene, robot, w, q_start):
    # round-7: exact face-overlap adjacency (tree-agnostic)
    from p36_fairness import _UF as UF
    uf = UF()
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
    n_band = n_band_theta = 0
    exts = []
    for k in frontier:
        c = tree.cell_center(k)
        size = _cell_size(tree, k)
        exts.append([float(size[0]), float(size[1]), float(size[2])])
        yf = -st * c[0] + ct * (c[1] - off)
        xf = ct * c[0] + st * (c[1] - off)
        if abs(xf) <= 0.6 and abs(yf) <= w / 2:
            n_band += 1
            dth = abs((c[2] - tilt + np.pi / 2) % np.pi - np.pi / 2)
            if dth <= ga:
                n_band_theta += 1
    med = (np.median(np.array(exts), axis=0).tolist() if exts else None)
    return {"start_in_free": True,
            "start_component_cells": len(comp),
            "frontier_cells": len(frontier),
            "frontier_in_gate_band": n_band,
            "frontier_in_band_and_theta": n_band_theta,
            "frontier_median_extent": med}


def run_one(arm, w, dy0, tilt_deg, robot_id, budgets):
    robot = robot_library()[robot_id]
    scene = make_g1_scene(w, door_offset=dy0, door_tilt=np.radians(tilt_deg))
    t0 = time.time()
    tree = make_tree(arm, scene, robot)
    res = tree.run(budgets, Q_START, GOAL, GOAL_RADIUS)
    solved = [b for b, r in sorted(res.items()) if r["reachable"]]
    row = {"arm": arm, "w": w, "dy0": dy0, "tilt_deg": tilt_deg,
           "robot": robot_id,
           "gate_half_angle_deg": float(np.degrees(
               gate_half_angle(robot.a, robot.b, w))),
           "roi": list(ROI), "theta_span": THETA_SPAN,
           "first_success_budget": solved[0] if solved else None,
           "results": {str(b): r for b, r in res.items()},
           "n_leaves_final": len(tree.leaves),
           "queries_final": int(tree.queries),
           "pair_ops": int(scene.pair_ops_total()),
           "bp_hits": int(scene.bp_hits_total()),
           "seconds": round(time.time() - t0, 1)}
    row.update(gate_band_stats(tree, scene, robot, w))
    if not solved:
        row["cut"] = cut_analysis_any(tree, scene, robot, w, Q_START)
    return row, solved


def main():
    # quotient soundness: verify pi-periodicity on one thin and one wide
    # instance for each robot used (throwaway scenes; billing isolated)
    assert_pi_periodic(0.505, *DEALIGN, "R_long_ellipse")
    assert_pi_periodic(1.10, 0.0, 0.0, "R_small_circle")
    print("pi-periodicity verified for both robots; quotient sound",
          flush=True)
    ref = continuation_reference()
    controls, rows = [], []
    for (w, dy0, tilt_deg, rid) in POS_CONTROLS:
        for arm in ARMS:
            row, solved = run_one(arm, w, dy0, tilt_deg, rid, POS_BUDGETS)
            row["kind"] = "positive_control"
            controls.append(row)
            print(f"CONTROL w={w:.2f} dy={dy0:+.3f} {rid} {arm:16s} "
                  f"first-success: {solved[0] if solved else 'NEVER'} "
                  f"({row['seconds']}s)", flush=True)
    for w in WS:
        for arm in ARMS:
            row, solved = run_one(arm, w, DEALIGN[0], DEALIGN[1],
                                  "R_long_ellipse", BUDGETS)
            row["kind"] = "main"
            rows.append(row)
            fs = (solved[0] if solved
                  else "NEVER(actual %d)" % row["queries_final"])
            cut = row.get("cut", {})
            print(f"w={w:.3f} {arm:16s} first-success: {fs} "
                  f"(cont ref {ref.get(round(w, 4), {}).get('total')}; "
                  f"{row['seconds']}s, leaves {row['n_leaves_final']}, "
                  f"pair_ops {row['pair_ops']}, "
                  f"band {row['cells_in_gate_band']}/"
                  f"{row['cells_in_band_and_theta']}, "
                  f"frontier {cut.get('frontier_cells', '-')} "
                  f"band {cut.get('frontier_in_gate_band', '-')}/"
                  f"{cut.get('frontier_in_band_and_theta', '-')})",
                  flush=True)
    summary = []
    for w in WS:
        cont = ref.get(round(w, 4), {}).get("total")
        line = {"w": w, "continuation_total": cont}
        for arm in ARMS:
            r = next(x for x in rows if x["w"] == w and x["arm"] == arm)
            line[arm] = r["first_success_budget"]
            if cont and r["first_success_budget"]:
                line[arm + "_ratio"] = round(
                    r["first_success_budget"] / cont, 2)
        # kill condition: any matched aniso arm within ~2x of continuation
        line["kill_condition_hit"] = bool(cont and any(
            line.get(a) and line[a] <= 2 * cont
            for a in ARMS))
        summary.append(line)
        print("P37", line, flush=True)
    from prov import make_provenance
    out = {"provenance": make_provenance(
        __file__, "P3.7 symmetry/domain-matched arms: ROI=seed box, "
        "[0,pi) quotient (pi-periodicity replay-verified), "
        "iso-uniform + aniso uniform/generic/pair, pose ladder to 131072, "
        "three-currency snapshots per checkpoint; continuation reference "
        "read from canonical table"),
        "roi": list(ROI), "theta_span": THETA_SPAN,
        "budgets": BUDGETS, "pos_budgets": POS_BUDGETS,
        "summary": summary, "positive_controls": controls, "rows": rows}
    os.makedirs(TABDIR, exist_ok=True)
    with open(os.path.join(TABDIR, "p37_matched.json"), "w") as f:
        json.dump(out, f, indent=1,
                  default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print("-> results/tables/p37_matched.json")


if __name__ == "__main__":
    main()
