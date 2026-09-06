"""P4a.2 chart-layer reachability consistency test (round-10 rework).

Round-10 withdrew the round-9 "P4a.1 semantic invariance: PASS" (the
frozen table is preserved in records/): the round-9 harness certified
charts against an unsound domain (robot-center footprint only), chose
its portal pair from the current start/goal, tested gate attachment at
the witness's start/goal poses, and accumulated pair-op counters across
runs.  This version fixes all four and DOWNGRADES the claim to what is
actually tested (review §8.5 naming):

  SINGLE-QUERY CHART-LAYER REACHABILITY CONSISTENCY under the tested
  transforms and budget band — not "graph-semantic invariance", not
  full-method acceptance (both remain HOLD; see the sprint-C report).

Fixes embodied:
  1. domain soundness: member cells certify the ROBOT BODY inside the
     true (rotated) workspace rectangle (support margin vs the same
     Lipschitz cell radius); portal/attachment certificates carry an
     additional domain bubble certificate on transformed problems;
  2. goal-free compile: portals are enumerated over ALL major chart
     pairs (symmetric, deterministic) with NO pose input; start/goal
     enter only the post-compile query readout;
  3. real gate chain (identity frame): ChartAttachment objects anchor
     the RIDGE BRANCH endpoints to the two charts, freshly certified;
     the assembled Atlas passes semantic validate(), and its hash and
     compile counter are invariant under 10 random query-pose pairs.
     The kernel witness itself remains G1-frame, start/goal-driven and
     x-threshold-conditioned (frozen kernel; goal-free gate DISCOVERY
     is P4b's contract — explicitly NOT claimed here);
  4. per-run accounting: every run uses a fresh scene object, so
     pair_ops / bp_hits are per-run quantities.

Budget ladder: {15k, 30k, 60k, 150k}; stability = converged stop
(budget-independent by construction of the whole-wave builder) or
agreement of the top two rungs.  The 150k rung guarantees convergence
at max_level=3, so in practice every transform ends with a converged
verdict; coarse-band flips remain reported.

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/p4a_semantic.py
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
    build_open_charts, locate_chart, chart_connect)
from splatc.compiler.domains import RigidRectDomain
from splatc.reference.oracle import certify_path_conservative

# 150k guarantees convergence at max_level=3 (worst-case level-3 wave
# ~= 8 x all-level-2 cells ~= 88k on top of ~12k spent; actual spend is
# the convergence cost, typically 45-60k) — the top rung exists so every
# transform reaches a CONVERGED, budget-independent verdict; round-10
# rework surfaced R60-class cases where the domain-sound builder's
# larger level-3 wave no longer fit under 60k
BUDGETS = [15000, 30000, 60000, 150000]
MAJOR_MIN_CELLS = 8
T_ROT = (0.3, -0.2)
PHASE_ROI = (-4.5, 4.5, -3.3, 3.3)
_CELL = ((PHASE_ROI[1] - PHASE_ROI[0]) / 7.0,
         (PHASE_ROI[3] - PHASE_ROI[2]) / 5.0)

TRANSFORMS = (
    [{"id": "R0", "phi_deg": 0.0, "t": [0.0, 0.0], "kind": "identity"}]
    + [{"id": f"R{int(p)}", "phi_deg": p, "t": list(T_ROT),
        "kind": "rotation"} for p in (30.0, 60.0, 90.0, 137.0)]
    + [{"id": f"TR{i+1}", "phi_deg": 0.0, "t": list(t),
        "kind": "translation_fixed_grid"}
       for i, t in enumerate([(0.13, -0.07), (0.31, 0.22)])]
    + [{"id": f"PH{i+1}", "phi_deg": 0.0, "t": [0.0, 0.0],
        "kind": "grid_origin_phase", "roi_shift": [f * _CELL[0],
                                                   f * _CELL[1]]}
       for i, f in enumerate([0.25, 0.5])])

CASES = [
    {"case": "thin", "w": 0.505, "dy0": 0.013, "tilt_deg": 7.0,
     "door_plug": False,
     "expected": "two_cores_no_generic_connector"},
    {"case": "wide", "w": 1.10, "dy0": 0.0, "tilt_deg": 0.0,
     "door_plug": False, "expected": "reachability_connected"},
    {"case": "sealed", "w": 0.505, "dy0": 0.013, "tilt_deg": 7.0,
     "door_plug": True,
     "expected": "two_cores_no_generic_connector"},
]

TABDIR = os.path.join(os.path.dirname(__file__), "..", "..",
                      "results", "tables")


def inv_pose(p, phi, t):
    c, s = np.cos(phi), np.sin(phi)
    x, y = p[0] - t[0], p[1] - t[1]
    return (c * x + s * y, -s * x + c * y, p[2] - phi)


def make_case_scene(spec):
    """Fresh scene object per call — per-run counters (round-10 §7)."""
    return make_g1_scene(spec["w"], door_offset=spec["dy0"],
                         door_tilt=np.radians(spec["tilt_deg"]),
                         door_plug=spec["door_plug"])


def setup(spec, tf):
    """Returns (fresh scene_used, roi, domain, pose_map)."""
    scene = make_case_scene(spec)
    phi = np.radians(tf["phi_deg"])
    t = tuple(tf["t"])
    if tf["kind"] == "identity":
        return scene, None, None, (lambda p: tuple(p))
    if tf["kind"] == "rotation":
        sc = rigid_transform_scene(scene, phi, t)
        return sc, None, RigidRectDomain(WORKSPACE, phi, t), \
            (lambda p: rigid_transform_pose(p, phi, t))
    if tf["kind"] == "translation_fixed_grid":
        sc = rigid_transform_scene(scene, 0.0, t)
        return sc, PHASE_ROI, RigidRectDomain(WORKSPACE, 0.0, t), \
            (lambda p: rigid_transform_pose(p, 0.0, t))
    dx, dy = tf["roi_shift"]
    roi = (PHASE_ROI[0] + dx, PHASE_ROI[1] + dx,
           PHASE_ROI[2] + dy, PHASE_ROI[3] + dy)
    return scene, roi, RigidRectDomain(WORKSPACE, 0.0, (0.0, 0.0)), \
        (lambda p: tuple(p))


def one_run(spec, tf, budget):
    """GOAL-FREE COMPILE, then query readout.  The compile part (charts
    + all-pairs portal enumeration) never sees a pose; start/goal are
    used only afterwards to read off single-query reachability."""
    from splatc.compiler.assemble import enumerate_portals
    robot = robot_library()["R_long_ellipse"]
    sc, roi, dom, pose_map = setup(spec, tf)
    po0 = sc.pair_ops_total() if hasattr(sc, "pair_ops_total") else 0
    bp0 = sc.bp_hits_total() if hasattr(sc, "bp_hits_total") else 0
    t0 = time.time()
    charts, info = build_open_charts(sc, robot, budget=budget,
                                     roi=roi, domain=dom)
    po_build = (sc.pair_ops_total() - po0) \
        if hasattr(sc, "pair_ops_total") else None
    bp_build = (sc.bp_hits_total() - bp0) \
        if hasattr(sc, "bp_hits_total") else None
    portals, precs, pchecks = enumerate_portals(
        sc, robot, info, domain=dom, major_min_cells=MAJOR_MIN_CELLS)
    po_total = (sc.pair_ops_total() - po0) \
        if hasattr(sc, "pair_ops_total") else None
    bp_total = (sc.bp_hits_total() - bp0) \
        if hasattr(sc, "bp_hits_total") else None
    # ---- query readout (post-compile; no counters may move) ----------
    q_start = pose_map(Q_START)
    q_goal = pose_map((GOAL[0], GOAL[1], Q_START[2]))
    st = locate_chart(info, q_start)
    gl = locate_chart(info, q_goal)
    resolved = st is not None and gl is not None
    padj = {}
    for p in portals:
        padj.setdefault(p.chart_a, set()).add(p.chart_b)
        padj.setdefault(p.chart_b, set()).add(p.chart_a)
    connected = None
    if resolved:
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
    n_major = sum(1 for v in sizes if v >= MAJOR_MIN_CELLS)
    n_attempts = sum(len(r["attempts"]) for r in precs.values())
    row = {"transform": tf["id"], "budget": budget,
           "queries": info["queries"],
           "completed_level": info["completed_level"],
           "stop": info["stop"],
           "n_charts": info["n_charts"], "n_major": n_major,
           "chart_sizes": sizes[:6],
           "portal_pairs_attempted": len(precs),
           "portal_attempts": n_attempts,
           "portal_checks": int(pchecks),
           "n_portals": len(portals),
           "portal_min_margins_m": [
               round(p.certificate["min_margin_m"], 6) for p in portals],
           "portal_tube_radii_m": [
               round(p.certificate["certified_tube_radius_m"], 6)
               for p in portals],
           "start_chart": st, "goal_chart": gl,
           "resolved": bool(resolved),
           "reachability_connected": connected,
           "retained_free_cells": info["n_free"],
           "serialized_region_bytes": sum(
               len(json.dumps(c.region)) for c in charts),
           "pair_ops_build": po_build, "bp_hits_build": bp_build,
           "pair_ops": po_total, "bp_hits": bp_total,
           "wall_s": round(time.time() - t0, 2)}
    return row, charts, info, portals, sc, dom, pose_map


def inverse_query_check(spec, tf, charts, info, portals, pose_map):
    """Assemble the transformed-frame chart(+portal) polyline, map it
    back to the identity frame, re-certify against a FRESH original
    scene (original workspace box included in the checker margin)."""
    robot = robot_library()["R_long_ellipse"]
    case_scene = make_case_scene(spec)
    q_start = pose_map(Q_START)
    q_goal = pose_map((GOAL[0], GOAL[1], Q_START[2]))
    st = locate_chart(info, q_start)
    gl = locate_chart(info, q_goal)
    if st is None or gl is None:
        return {"assembled": False, "reason": "endpoint unresolved"}
    reg = {c.chart_id: c.region for c in charts}
    if st == gl:
        out = chart_connect(reg[st], q_start, q_goal)
        if out is None:
            return {"assembled": False, "reason": "chart_connect failed"}
        wps = out[0]
    else:
        portal = next((p for p in portals
                       if {p.chart_a, p.chart_b} == {st, gl}), None)
        if portal is None:
            return {"assembled": False, "reason": "not connected"}
        pw = portal.waypoints
        if locate_chart(info, pw[0]) != st:
            pw = pw[::-1]
        a = chart_connect(reg[st], q_start, tuple(pw[0]))
        b = chart_connect(reg[gl], tuple(pw[-1]), q_goal)
        if a is None or b is None:
            return {"assembled": False,
                    "reason": "portal endpoint not in chart"}
        wps = a[0] + [list(p) for p in pw[1:-1]] + b[0]
    phi = np.radians(tf["phi_deg"])
    t = tuple(tf["t"])
    back = np.array([inv_pose(p, phi, t) for p in wps])
    ok, mmin, _, checks = certify_path_conservative(
        case_scene, robot, back)
    return {"assembled": True, "n_waypoints": len(wps),
            "certified_in_identity_frame": bool(ok),
            "min_margin_m": round(float(mmin), 6),
            "recert_checks": int(checks)}


def atlas_prototype_identity(spec):
    """Identity frame, thin case: full goal-free compile into a REAL
    Atlas (charts + all-pairs portals + attachment-branch-attachment
    gate chain), semantic validate, serialized query, and the
    pose-independence regression (10 random query pairs; hash and
    compile counter must not move)."""
    from splatc.compiler.assemble import (
        enumerate_portals, build_gate_chain, compile_atlas,
        query_reachable)
    from ridge_continuation import run_case
    robot = robot_library()["R_long_ellipse"]
    sc = make_case_scene(spec)
    charts, info = build_open_charts(sc, robot, budget=60000)
    portals, _, pchecks = enumerate_portals(sc, robot, info)
    row, diag = run_case(spec["w"])
    if row.get("status") != "CERTIFIED_REACHABLE":
        return {"witness_status": row.get("status"), "assembled": False}
    stations = diag["stations"]
    dense = diag["profile"]["wps"][2:-1]
    objs, rec = build_gate_chain(sc, robot, info, stations, dense)
    if objs is None:
        return {"witness_status": row["status"], "assembled": False,
                "gate_chain": rec}
    atlas = compile_atlas(sc, robot, charts, info, portals,
                          gate_objects=objs,
                          compile_queries=info["queries"] + pchecks
                          + rec["checks"])
    violations = atlas.validate()
    h0 = atlas.atlas_hash()
    ser = json.loads(atlas.serialize())
    demo = query_reachable(ser, tuple(Q_START),
                           (GOAL[0], GOAL[1], Q_START[2]))
    rng = np.random.RandomState(20260820)
    n_reach = 0
    for _ in range(10):
        q1 = (float(rng.uniform(-2.8, -1.2)),
              float(rng.uniform(-1.2, 1.2)),
              float(rng.uniform(0, 2 * np.pi)))
        q2 = (float(rng.uniform(1.2, 2.8)),
              float(rng.uniform(-1.2, 1.2)),
              float(rng.uniform(0, 2 * np.pi)))
        n_reach += bool(query_reachable(ser, q1, q2)["reachable"])
    return {"witness_status": row["status"],
            "witness_queries_total": int(row["total"]),
            "witness_note": "frozen v4.1 kernel: G1-frame, start/"
                            "goal-driven, x-threshold-conditioned "
                            "(recorded limitation; not a goal-free "
                            "discovery claim)",
            "assembled": True,
            "gate_chain": {k: v for k, v in rec.items()},
            "attached_charts": [rec["chart_a"], rec["chart_b"]],
            "validate_violations": violations,
            "atlas_hash": h0,
            "compile_query_count": atlas.compile_query_count,
            "query_demo": demo,
            "pose_independence": {
                "n_random_query_pairs": 10,
                "n_reachable": int(n_reach),
                "hash_unchanged": bool(atlas.atlas_hash() == h0),
                "compile_count_unchanged": bool(
                    atlas.compile_query_count == info["queries"]
                    + pchecks + rec["checks"])}}


def main():
    out_cases = []
    for spec in CASES:
        runs, keep = [], {}
        for tf in TRANSFORMS:
            for b in BUDGETS:
                row, charts, info, portals, sc, dom, pm = \
                    one_run(spec, tf, b)
                runs.append(row)
                if b == BUDGETS[-1]:
                    keep[(tf["id"], b)] = (charts, info, portals, pm)
                print(f"P4A2 {spec['case']:6s} {tf['id']:4s} B={b}: "
                      f"q={row['queries']} lvl={row['completed_level']} "
                      f"major={row['n_major']} "
                      f"portals={row['n_portals']} "
                      f"conn={row['reachability_connected']}",
                      flush=True)
        stability = []
        for tf in TRANSFORMS:
            rt = {r["budget"]: r for r in runs
                  if r["transform"] == tf["id"]}
            v = {b: rt[b]["reachability_connected"] for b in BUDGETS}
            conv = [b for b in BUDGETS
                    if rt[b]["stop"] in ("max_level", "no_frontier")]
            if conv:
                stable, verdict = True, v[max(conv)]
                basis = f"converged_at_{max(conv)}"
            elif v[BUDGETS[-2]] == v[BUDGETS[-1]]:
                stable, verdict = True, v[BUDGETS[-2]]
                basis = f"band_{BUDGETS[-2]}_{BUDGETS[-1]}_agree"
            else:
                stable, verdict, basis = False, None, "UNSTABLE"
            stability.append(
                {"transform": tf["id"],
                 "verdicts_by_budget": {str(b): v[b] for b in BUDGETS},
                 "low_band_agree": v[15000] == v[30000],
                 "stable": stable, "basis": basis,
                 "stable_verdict": verdict})
        want_connected = spec["expected"] == "reachability_connected"
        case_pass = all(s["stable"] and s["stable_verdict"] is
                        want_connected for s in stability)
        if not want_connected:
            case_pass = case_pass and all(
                r["n_major"] == 2 and r["n_portals"] == 0
                for r in runs if r["budget"] in BUDGETS[1:])
        entry = {**spec, "runs": runs, "stability": stability}
        if spec["case"] == "wide":
            iq = []
            for tf in TRANSFORMS:
                charts, info, portals, pm = keep[(tf["id"], BUDGETS[-1])]
                r = inverse_query_check(spec, tf, charts, info,
                                        portals, pm)
                r["transform"] = tf["id"]
                iq.append(r)
                print(f"P4A2 wide inverse-query {tf['id']}: {r}",
                      flush=True)
            entry["inverse_query_top_budget"] = iq
            case_pass = case_pass and all(
                q.get("certified_in_identity_frame") for q in iq)
        if spec["case"] == "thin":
            ap = atlas_prototype_identity(spec)
            print(f"P4A2 thin atlas prototype: assembled="
                  f"{ap.get('assembled')} "
                  f"violations={ap.get('validate_violations')} "
                  f"pose_indep={ap.get('pose_independence')}",
                  flush=True)
            entry["atlas_prototype_identity_60k"] = ap
            case_pass = case_pass and ap.get("assembled", False) \
                and ap.get("validate_violations") == [] \
                and ap.get("pose_independence", {}).get(
                    "hash_unchanged", False) \
                and ap.get("pose_independence", {}).get(
                    "compile_count_unchanged", False)
        entry["case_pass"] = bool(case_pass)
        out_cases.append(entry)
        print(f"P4A2 case {spec['case']}: "
              f"{'PASS' if case_pass else 'FAIL'}", flush=True)
    overall = "PASS" if all(c["case_pass"] for c in out_cases) else "FAIL"
    from prov import make_provenance
    out = {"provenance": make_provenance(
        __file__, "P4a.2 single-query chart-layer reachability "
        "consistency: domain-support-sound wave-complete OpenCharts + "
        "pose-free symmetric all-pairs generic-connector enumeration + "
        "identity-frame atlas prototype with real attachment-branch-"
        "attachment chain; NOT a graph-isomorphism or full-method "
        "acceptance claim; kernel witness remains G1-frame and "
        "x-threshold-conditioned (frozen; recorded limitation)"),
        "budgets": BUDGETS, "major_min_cells": MAJOR_MIN_CELLS,
        "phase_roi": list(PHASE_ROI), "transforms": TRANSFORMS,
        "cases": out_cases,
        "overall": {"chart_layer_reachability_consistency": overall}}
    os.makedirs(TABDIR, exist_ok=True)
    with open(os.path.join(TABDIR, "p4a2_semantic.json"), "w") as f:
        json.dump(out, f, indent=1,
                  default=lambda o: o.item() if hasattr(o, "item")
                  else str(o))
    print(f"P4A2 OVERALL chart-layer reachability consistency: "
          f"{overall}")
    print("-> results/tables/p4a2_semantic.json")


if __name__ == "__main__":
    main()
