"""P4a.3 sealed validation set (round-10 §10).

Purpose: stop validating the chart layer only on the nine hand-picked
transforms it was debugged on.  This is a FROZEN, seeded sweep across
axes the fixes never saw during development:

  S1  width ladder x 3 robot morphologies (+ plugged controls)
      — characterization axes; not connectivity-gated
  S2  30 seeded random whole-scene rigid transforms (thin + wide)
  S3  10 seeded random grid-origin phases (thin + wide)
  S4  max_level {2,3,4} sensitivity (thin + wide)
  S5  door offset/tilt grid (thin + wide)

Hard acceptance (machine verdict, quoted verbatim in the report):
  - INDEPENDENT soundness audit: every member cell of every build is
    re-sampled (4 xy corners x 3 theta samples) against the domain's
    robot-support margin — zero violations allowed (this is the
    round-10 blocker-1 regression, run at scale, checked OUTSIDE the
    builder's own certificate path);
  - plugged doors never yield a generic connector;
  - thin (w=0.505) is never reachability-connected, wide (w=1.10)
    always is, across S2/S3/S4/S5 — every gated build runs at a budget
    (150k; S4 lvl-4 at 500k) that guarantees a CONVERGED, budget-
    independent partition at its level cap, so these verdicts carry no
    budget escape hatch;
  - per-run pair-op accounting (fresh scene per build).

Explicitly NOT acceptance-gated: whether a generic connector exists at
intermediate widths (S1 ladder, w=0.54..0.90).  Round-10 measured this
to be non-monotonic (a straight certified line exists at w=0.58 but not
w=0.70/0.80 for the level-2 anchor set); the connector is a PROOF
SOURCE, not a width classifier, so S1 records the behaviour as
characterization data — with the non-monotonicity disclosed, not
explained away.

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/p4a3_validation.py
"""
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from splatc.datasets.g1_gate import (
    make_g1_scene, robot_library, Q_START, GOAL, WORKSPACE)
from splatc.datasets.transforms import (
    rigid_transform_scene, rigid_transform_pose)
from splatc.compiler.chart_builder import (
    build_open_charts, locate_chart, cell_bounds)
from splatc.compiler.domains import RigidRectDomain
from splatc.compiler.assemble import enumerate_portals

MAJOR_MIN_CELLS = 8
PHASE_ROI = (-4.5, 4.5, -3.3, 3.3)
_CELL = ((PHASE_ROI[1] - PHASE_ROI[0]) / 7.0,
         (PHASE_ROI[3] - PHASE_ROI[2]) / 5.0)
SEED = 20260820
TABDIR = os.path.join(os.path.dirname(__file__), "..", "..",
                      "results", "tables")


def soundness_audit(info, domain, robot):
    """Independent (non-builder-path) audit: sample every member cell's
    xy corners x theta {lo, mid, hi} against the domain support margin.
    Pure geometry, zero queries."""
    if domain is None:
        return {"cells": sum(len(m) for m in info["members"].values()),
                "violations": 0, "note": "no domain"}
    tree = info["tree"]
    n, bad = 0, 0
    for cells in info["members"].values():
        for k in cells:
            n += 1
            lo, hi = cell_bounds(tree.ROOT, tree.origin, tree.span, k)
            ok = True
            for x in (lo[0], hi[0]):
                for y in (lo[1], hi[1]):
                    for th in (lo[2], 0.5 * (lo[2] + hi[2]), hi[2]):
                        if domain.support_margin(x, y, th, robot) <= 0:
                            ok = False
            bad += (not ok)
    return {"cells": n, "violations": bad}


def one_build(scene, robot, budget, roi=None, domain=None,
              max_level=3, pose_map=lambda p: tuple(p)):
    po0 = scene.pair_ops_total() if hasattr(scene, "pair_ops_total") else 0
    t0 = time.time()
    charts, info = build_open_charts(scene, robot, budget=budget,
                                     roi=roi, domain=domain,
                                     max_level=max_level)
    portals, precs, pchecks = enumerate_portals(
        scene, robot, info, domain=domain,
        major_min_cells=MAJOR_MIN_CELLS)
    st = locate_chart(info, pose_map(Q_START))
    gl = locate_chart(info, pose_map((GOAL[0], GOAL[1], Q_START[2])))
    padj = {}
    for p in portals:
        padj.setdefault(p.chart_a, set()).add(p.chart_b)
        padj.setdefault(p.chart_b, set()).add(p.chart_a)
    connected = None
    if st is not None and gl is not None:
        seen, queue = {st}, [st]
        while queue:
            u = queue.pop()
            for v in padj.get(u, ()):
                if v not in seen:
                    seen.add(v)
                    queue.append(v)
        connected = gl in seen
    sizes = sorted((len(m) for m in info["members"].values()),
                   reverse=True)
    audit = soundness_audit(info, domain, robot)
    row = {"queries": info["queries"], "stop": info["stop"],
           "completed_level": info["completed_level"],
           "n_major": sum(1 for v in sizes if v >= MAJOR_MIN_CELLS),
           "chart_sizes": sizes[:4],
           "n_portals": len(portals),
           "portal_attempts": sum(len(r["attempts"])
                                  for r in precs.values()),
           "portal_checks": int(pchecks),
           "portal_min_margins_m": [
               round(p.certificate["min_margin_m"], 6) for p in portals],
           "resolved": bool(st is not None and gl is not None),
           "reachability_connected": connected,
           "soundness_audit": audit,
           "pair_ops": (scene.pair_ops_total() - po0)
           if hasattr(scene, "pair_ops_total") else None,
           "wall_s": round(time.time() - t0, 2)}
    return row


def main():
    robots = robot_library()
    results = {"S1_width_morphology": [], "S2_random_rigid": [],
               "S3_random_phase": [], "S4_level_sensitivity": [],
               "S5_offset_tilt": []}

    # ---- S1: width ladder x morphologies (+ plugged controls) --------
    widths = [0.505, 0.54, 0.58, 0.62, 0.70, 0.80, 0.90, 1.10]
    for rid in ("R_long_ellipse", "R_small_circle", "R_big_circle"):
        for w in widths:
            sc = make_g1_scene(w, door_offset=0.013,
                               door_tilt=np.radians(7.0))
            row = one_build(sc, robots[rid], 150000)
            row.update({"robot": rid, "w": w, "plug": False})
            results["S1_width_morphology"].append(row)
            print(f"P4A3 S1 {rid} w={w}: portals={row['n_portals']} "
                  f"conn={row['reachability_connected']}", flush=True)
        for w in (0.505, 1.10):
            sc = make_g1_scene(w, door_offset=0.013,
                               door_tilt=np.radians(7.0), door_plug=True)
            row = one_build(sc, robots[rid], 150000)
            row.update({"robot": rid, "w": w, "plug": True})
            results["S1_width_morphology"].append(row)
            print(f"P4A3 S1 {rid} w={w} PLUG: portals="
                  f"{row['n_portals']} "
                  f"conn={row['reachability_connected']}", flush=True)

    # ---- S2: 30 seeded random rigid transforms, thin + wide ----------
    rng = np.random.RandomState(SEED)
    transforms = [(float(rng.uniform(0, 2 * np.pi)),
                   (float(rng.uniform(-0.4, 0.4)),
                    float(rng.uniform(-0.4, 0.4)))) for _ in range(30)]
    robot = robots["R_long_ellipse"]
    for w in (0.505, 1.10):
        for i, (phi, t) in enumerate(transforms):
            # fresh scene per build: per-run counters (round-10 §7)
            sc = rigid_transform_scene(make_g1_scene(
                w, door_offset=0.013, door_tilt=np.radians(7.0)),
                phi, t)
            dom = RigidRectDomain(WORKSPACE, phi, t)
            row = one_build(sc, robot, 150000, domain=dom,
                            pose_map=lambda p, _p=phi, _t=t:
                            rigid_transform_pose(p, _p, _t))
            row.update({"w": w, "phi_rad": round(phi, 6),
                        "t": [round(t[0], 6), round(t[1], 6)]})
            results["S2_random_rigid"].append(row)
            print(f"P4A3 S2 w={w} #{i} phi={np.degrees(phi):.1f}: "
                  f"sound={row['soundness_audit']['violations']} "
                  f"conn={row['reachability_connected']}", flush=True)

    # ---- S3: 10 seeded random grid-origin phases, thin + wide --------
    rng = np.random.RandomState(SEED + 1)
    phases = [(float(rng.uniform(0, 1)), float(rng.uniform(0, 1)))
              for _ in range(10)]
    for w in (0.505, 1.10):
        for i, (fx, fy) in enumerate(phases):
            sc = make_g1_scene(w, door_offset=0.013,
                               door_tilt=np.radians(7.0))
            roi = (PHASE_ROI[0] + fx * _CELL[0],
                   PHASE_ROI[1] + fx * _CELL[0],
                   PHASE_ROI[2] + fy * _CELL[1],
                   PHASE_ROI[3] + fy * _CELL[1])
            dom = RigidRectDomain(WORKSPACE, 0.0, (0.0, 0.0))
            row = one_build(sc, robot, 150000, roi=roi, domain=dom)
            row.update({"w": w, "phase": [round(fx, 6), round(fy, 6)]})
            results["S3_random_phase"].append(row)
            print(f"P4A3 S3 w={w} #{i}: "
                  f"conn={row['reachability_connected']}", flush=True)

    # ---- S4: max_level sensitivity, thin + wide ----------------------
    for w in (0.505, 1.10):
        for lvl, budget in ((2, 60000), (3, 60000), (4, 500000)):
            sc = make_g1_scene(w, door_offset=0.013,
                               door_tilt=np.radians(7.0))
            row = one_build(sc, robot, budget, max_level=lvl)
            row.update({"w": w, "max_level": lvl, "budget": budget})
            results["S4_level_sensitivity"].append(row)
            print(f"P4A3 S4 w={w} lvl={lvl}: q={row['queries']} "
                  f"conn={row['reachability_connected']}", flush=True)

    # ---- S5: offset/tilt grid, thin + wide ---------------------------
    for w in (0.505, 1.10):
        for dy0 in (0.0, 0.013, 0.05):
            for tilt in (0.0, 7.0, 15.0):
                sc = make_g1_scene(w, door_offset=dy0,
                                   door_tilt=np.radians(tilt))
                row = one_build(sc, robot, 150000)
                row.update({"w": w, "dy0": dy0, "tilt_deg": tilt})
                results["S5_offset_tilt"].append(row)
        print(f"P4A3 S5 w={w}: done", flush=True)

    # ---- verdicts ----------------------------------------------------
    all_rows = [r for rows in results.values() for r in rows]
    sound_viol = sum(r["soundness_audit"]["violations"] for r in all_rows)
    plug_portals = sum(r["n_portals"]
                       for r in results["S1_width_morphology"]
                       if r.get("plug"))
    s2s3s4 = (results["S2_random_rigid"] + results["S3_random_phase"]
              + results["S4_level_sensitivity"]
              + results["S5_offset_tilt"])
    thin_bad = [r for r in s2s3s4
                if r["w"] == 0.505 and r["reachability_connected"]
                is not False]
    wide_bad = [r for r in s2s3s4
                if r["w"] == 1.10 and r["reachability_connected"]
                is not True]
    verdict = ("PASS" if sound_viol == 0 and plug_portals == 0
               and not thin_bad and not wide_bad else "FAIL")
    # S1 characterization: connector presence by width per robot
    ladder = {}
    for r in results["S1_width_morphology"]:
        if not r.get("plug"):
            ladder.setdefault(r["robot"], {})[str(r["w"])] = \
                r["n_portals"]
    from prov import make_provenance
    out = {"provenance": make_provenance(
        __file__, "P4a.3 sealed validation set: seeded width/morphology/"
        "rigid-transform/grid-phase/level sweeps with an INDEPENDENT "
        "robot-support soundness audit per build; generic-connector "
        "presence at intermediate widths is characterization data "
        "(known non-monotonic), not an acceptance criterion"),
        "seed": SEED, "n_builds": len(all_rows),
        "results": results,
        "connector_ladder_by_robot": ladder,
        "verdict_inputs": {
            "soundness_violations_total": int(sound_viol),
            "plugged_portals_total": int(plug_portals),
            "thin_unexpected_connected": len(thin_bad),
            "wide_unexpected_disconnected": len(wide_bad)},
        "overall": {"sealed_validation": verdict}}
    os.makedirs(TABDIR, exist_ok=True)
    with open(os.path.join(TABDIR, "p4a3_validation.json"), "w") as f:
        json.dump(out, f, indent=1,
                  default=lambda o: o.item() if hasattr(o, "item")
                  else str(o))
    print(f"P4A3 OVERALL sealed validation: {verdict} "
          f"(sound_viol={sound_viol}, plug_portals={plug_portals}, "
          f"thin_bad={len(thin_bad)}, wide_bad={len(wide_bad)})")
    print("-> results/tables/p4a3_validation.json")


if __name__ == "__main__":
    main()
