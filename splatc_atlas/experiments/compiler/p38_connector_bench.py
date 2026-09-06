"""P3.8 chart-factored connector benchmark (round-7/8/10 mandate — the
paper-level life-or-death experiment).

Task, identical for every arm: given the two certified open-chart
frontiers F_L / F_R of a case (the two largest charts of a converged
goal-free build) plus billed query access to the scene, produce a
CERTIFIED CONNECTOR F_L ~> F_R.  No door axis, no admissible-theta
window, no oracle centerline is handed to any arm; scene.meta is never
read by any arm.

Arms
  contact_continuation   v4.1 kernel witness + build_gate_chain attachment.
                         DISCLOSED ASYMMETRY: the frozen kernel's exit/span
                         logic is G1-frame-conditioned (|x| thresholds — the
                         recorded v4.1 limitation, P4b contract), so this arm
                         carries a door-frame prior the baselines do not get.
                         Baseline ratios below are therefore measured against
                         an ADVANTAGED continuation; a baseline reaching 2x
                         of it kills the claim a fortiori.
  scalar_continuation    declared scalar-clearance ablation + same attachment.
  generic_connector      symmetric certified straight connector
                         (try_certify_portal — the P4a.2 proof source).
  rrt_connect            bidirectional RRT-Connect (sampling baseline).
  lazy_prm_bridge        lazy PRM with bridge-test narrow-passage sampling.
  hrm_se2                HRM-style theta-layer sweep decomposition
                         (SE(2) NavMesh-family representative).

Unified per-run record: success, pose queries (nq), pair_ops, bp_hits,
wall seconds, retained states, certified min margin, first sampled
connection, certification failures.  Sampling arms run SEEDS fixed seeds;
deterministic arms run once.

Cases: w=0.505 de-aligned thin gate (kill case), w=0.62 (generic-connector
regime control), w=0.505 plugged (negative control: every arm must refuse;
any claimed connector is independently audited — false claims must be 0).

KILL CONDITION (round-10 wording): if a non-volumetric baseline
(rrt_connect / lazy_prm_bridge / hrm_se2) reaches success rate >= 0.5 on
the thin case AND is within ~2x of the continuation arm on ANY of pose
queries, pair_ops, or wall time (median over successes), single-query
gate efficiency stops being the paper's main residual and the main line
shifts to compile-once/query-many, morphology update, topology reuse,
local recompilation.  The any-currency rule is deliberately generous to
the baselines.

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/p38_connector_bench.py
"""
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from splatc.datasets.g1_gate import make_g1_scene, robot_library
from splatc.compiler.chart_builder import build_open_charts, cell_bounds
from splatc.compiler.portals import try_certify_portal
from splatc.compiler.assemble import build_gate_chain
from splatc.compiler.atlas_types import ChartAttachment
from splatc.baselines.connector_arms import (
    rrt_connect, lazy_prm_bridge, hrm_se2, in_region)
from splatc.reference.oracle import certify_path_conservative
from ridge_continuation import run_case
from ridge_continuation_scalar import run_case_scalar

TABDIR = os.path.join(os.path.dirname(__file__), "..", "..",
                      "results", "tables")
ROBOT_ID = "R_long_ellipse"
DEALIGN = (0.013, 7.0)
CHART_BUDGET = 150000          # converged build (P4a.2 top rung)
BUDGET = 262144                # per-run pose-query cap for baselines
SEEDS = list(range(10))        # sampling arms, positive cases
SEEDS_NEG = list(range(3))     # sampling arms, plugged case (each run
                               # exhausts the full budget; refusal is
                               # certification-enforced, not seed luck)
CASES = [
    {"case": "thin_0.505", "w": 0.505, "plug": False},
    {"case": "mid_0.62", "w": 0.62, "plug": False},
    {"case": "plug_0.505", "w": 0.505, "plug": True},
]
BASELINE_ARMS = ["rrt_connect", "lazy_prm_bridge", "hrm_se2"]


def case_scene(spec):
    return make_g1_scene(spec["w"], door_offset=DEALIGN[0],
                         door_tilt=np.radians(DEALIGN[1]),
                         door_plug=spec["plug"])


def assert_pi_periodic(spec, robot, n_probe=120):
    """[0, pi) sampling in the baselines is sound iff collision status is
    pi-periodic; verified by direct replay on a throwaway scene."""
    scene = case_scene(spec)
    rng = np.random.RandomState(0)
    for _ in range(n_probe):
        q = (rng.uniform(-3, 3), rng.uniform(-1.8, 1.8),
             rng.uniform(0, np.pi))
        a, ra = scene.check_pose(robot, q)
        b, rb = scene.check_pose(robot, (q[0], q[1], q[2] + np.pi))
        assert a == b and abs(ra - rb) < 1e-9, f"pi-periodicity broken {q}"


def build_frontiers(spec, robot):
    """Shared compile input: converged goal-free chart build; F_L / F_R =
    the two largest charts (L = smaller mean member-cell x; a display
    convention, not an arm input)."""
    scene = case_scene(spec)
    charts, info = build_open_charts(scene, robot, budget=CHART_BUDGET,
                                     max_level=3)
    assert len(charts) >= 2, f"{spec['case']}: fewer than two charts"
    two = charts[:2]
    means = []
    for ch in two:
        g = ch.region["grid"]
        xs = [0.5 * (cell_bounds(g["root"], g["origin"], g["span"],
                                 tuple(k))[0][0]
                     + cell_bounds(g["root"], g["origin"], g["span"],
                                   tuple(k))[1][0])
              for k in ch.region["cells"]]
        means.append(float(np.mean(xs)))
    iL = int(np.argmin(means))
    chL, chR = two[iL], two[1 - iL]
    return {"info": info, "charts": charts,
            "cid_L": chL.chart_id, "cid_R": chR.chart_id,
            "region_L": chL.region, "region_R": chR.region,
            "build_queries": int(info["queries"]),
            "build_stop": info["stop"],
            "build_level": int(info["completed_level"]),
            "n_cells": [len(chL.region["cells"]),
                        len(chR.region["cells"])]}


def audit_connector(spec, robot, wps, region_L, region_R):
    """Independent harness-side audit on a FRESH scene (uncharged):
    endpoint membership + full conservative re-certification.  Guards the
    zero-false-reachable red line."""
    if wps is None:
        return False, None
    if not (in_region(region_L, tuple(wps[0]))
            and in_region(region_R, tuple(wps[-1]))):
        return False, None
    scene = case_scene(spec)
    ok, mmin, _, _ = certify_path_conservative(
        scene, robot, np.asarray(wps, dtype=float))
    return bool(ok), (round(float(mmin) * 1e3, 2) if ok else None)


def continuation_connector(objs, dense_wps):
    """Assemble the full connector polyline from the gate-chain objects:
    attachment A (chart -> ridge endpoint) + dense branch + reversed
    attachment B."""
    atts = [o for o in objs if isinstance(o, ChartAttachment)]
    att_a = next(o for o in atts if o.attachment_id.endswith("aA"))
    att_b = next(o for o in atts if o.attachment_id.endswith("aB"))
    wps = [list(p) for p in att_a.waypoints]
    wps += [list(p) for p in dense_wps[1:]]
    wps += [list(p) for p in reversed(att_b.waypoints[:-1])]
    return wps


def arm_contact(spec, pack, robot, scalar=False):
    t0 = time.time()
    if scalar:
        row = run_case_scalar(spec["w"], door_plug=spec["plug"],
                              return_geometry=True)
        diag = row.pop("_geometry", None)
        stations = diag["stations"] if diag else None
        dense = diag["dense"] if diag else None
        pair_kernel = row.get("pair_ops")      # None on NO_SEED (the
        # scalar kernel's NO_SEED return carries no pair snapshot)
    else:
        row, diag = run_case(spec["w"], door_plug=spec["plug"])
        stations = diag.get("stations")
        # billing boundary: exclude the diagnostic margin_profile
        pair_kernel = row.get("pair_ops_cert",
                              row.get("pair_ops_core",
                                      row.get("pair_ops")))
        dense = None
        if row["status"] == "CERTIFIED_REACHABLE":
            # certificate polyline minus the start/goal template poses
            dense = [p for p in diag["profile"]["wps"][2:-1]]
    nq = int(row.get("total") or (row.get("c_seed", 0)
                                  + row.get("c_track", 0)
                                  + row.get("c_densify", 0)
                                  + row.get("c_connector", 0)))
    out = {"success": False, "nq": nq, "pair_ops": pair_kernel,
           "bp_hits": None, "n_states": None,
           "cert_min_margin_mm": None, "path": None,
           "kernel_status": row["status"],
           "nq_first_sampled": None, "cert_fail_events": 0}
    if row["status"] != "CERTIFIED_REACHABLE":
        out["wall_seconds"] = round(time.time() - t0, 1)
        return out
    scene_att = case_scene(spec)
    objs, rec = build_gate_chain(scene_att, robot, pack["info"],
                                 stations, dense)
    out["nq"] = nq + int(rec.get("checks", 0))
    out["pair_ops"] = pair_kernel + int(scene_att.pair_ops_total())
    out["bp_hits"] = int(scene_att.bp_hits_total())
    out["n_states"] = len(stations)
    if objs is None:
        out["attach_record"] = {k: v for k, v in rec.items()
                                if k != "checks"}
        out["wall_seconds"] = round(time.time() - t0, 1)
        return out
    wps = continuation_connector(objs, dense)
    out.update({"success": True, "path": wps,
                "cert_min_margin_mm": round(
                    rec["branch_min_margin_m"] * 1e3, 2),
                "chart_a": rec["chart_a"], "chart_b": rec["chart_b"]})
    out["wall_seconds"] = round(time.time() - t0, 1)
    return out


def arm_generic(spec, pack, robot):
    t0 = time.time()
    scene = case_scene(spec)
    portal, rec = try_certify_portal(scene, robot, pack["info"],
                                     pack["cid_L"], pack["cid_R"])
    out = {"success": portal is not None, "nq": int(rec["checks"]),
           "pair_ops": int(scene.pair_ops_total()),
           "bp_hits": int(scene.bp_hits_total()),
           "n_states": len(rec["attempts"]),
           "nq_first_sampled": None, "cert_fail_events":
               sum(1 for a in rec["attempts"] if not a.get("certified")),
           "cert_min_margin_mm": None, "path": None}
    if portal is not None:
        out["cert_min_margin_mm"] = round(
            portal.certificate["min_margin_m"] * 1e3, 2)
        out["path"] = [list(p) for p in portal.waypoints]
    out["wall_seconds"] = round(time.time() - t0, 1)
    return out


def arm_baseline(name, spec, pack, robot, seed):
    t0 = time.time()
    scene = case_scene(spec)
    if name == "rrt_connect":
        r = rrt_connect(scene, robot, pack["region_L"], pack["region_R"],
                        seed, BUDGET)
    elif name == "lazy_prm_bridge":
        r = lazy_prm_bridge(scene, robot, pack["region_L"],
                            pack["region_R"], seed, BUDGET)
    else:
        r = hrm_se2(scene, robot, pack["region_L"], pack["region_R"],
                    BUDGET)
    r["pair_ops"] = int(scene.pair_ops_total())
    r["bp_hits"] = int(scene.bp_hits_total())
    r["wall_seconds"] = round(time.time() - t0, 1)
    return r


def median_or_none(vals):
    vals = [v for v in vals if v is not None]
    return float(np.median(vals)) if vals else None


def main():
    robot = robot_library()[ROBOT_ID]
    assert_pi_periodic(CASES[0], robot)
    assert_pi_periodic(CASES[1], robot)
    print("pi-periodicity verified; [0,pi) baseline sampling sound",
          flush=True)
    rows, case_meta, false_claims = [], {}, 0
    for spec in CASES:
        pack = build_frontiers(spec, robot)
        case_meta[spec["case"]] = {k: pack[k] for k in
                                   ("cid_L", "cid_R", "build_queries",
                                    "build_stop", "build_level",
                                    "n_cells")}
        print(f"[{spec['case']}] charts {pack['cid_L']}/{pack['cid_R']} "
              f"cells {pack['n_cells']} build {pack['build_queries']} "
              f"({pack['build_stop']})", flush=True)
        runs = []
        for arm in ("contact_continuation", "scalar_continuation",
                    "generic_connector"):
            if arm == "contact_continuation":
                r = arm_contact(spec, pack, robot)
            elif arm == "scalar_continuation":
                r = arm_contact(spec, pack, robot, scalar=True)
            else:
                r = arm_generic(spec, pack, robot)
            runs.append((arm, None, r))
        seeds = SEEDS_NEG if spec["plug"] else SEEDS
        for arm in BASELINE_ARMS:
            arm_seeds = [None] if arm == "hrm_se2" else seeds
            for sd in arm_seeds:
                r = arm_baseline(arm, spec, pack, robot,
                                 0 if sd is None else sd)
                runs.append((arm, sd, r))
        for arm, sd, r in runs:
            audited, audit_mm = audit_connector(
                spec, robot, r.get("path"),
                pack["region_L"], pack["region_R"])
            if r.get("success") and not audited:
                false_claims += 1
            row = {"case": spec["case"], "arm": arm, "seed": sd,
                   "success": bool(r.get("success")),
                   "audited": bool(audited) if r.get("success") else None,
                   "audit_min_margin_mm": audit_mm,
                   "nq": r.get("nq"), "pair_ops": r.get("pair_ops"),
                   "bp_hits": r.get("bp_hits"),
                   "wall_seconds": r.get("wall_seconds"),
                   "n_states": r.get("n_states"),
                   "cert_min_margin_mm": r.get("cert_min_margin_mm"),
                   "nq_first_sampled": r.get("nq_first_sampled"),
                   "cert_fail_events": r.get("cert_fail_events"),
                   "kernel_status": r.get("kernel_status")}
            rows.append(row)
            print(f"  {arm:22s} seed={sd} success={row['success']} "
                  f"nq={row['nq']} pair_ops={row['pair_ops']} "
                  f"wall={row['wall_seconds']}s "
                  f"margin={row['cert_min_margin_mm']}mm", flush=True)

    # ---- per-case per-arm summary ----------------------------------------
    summary = {}
    for spec in CASES:
        c = spec["case"]
        summary[c] = {}
        for arm in ("contact_continuation", "scalar_continuation",
                    "generic_connector") + tuple(BASELINE_ARMS):
            rs = [r for r in rows if r["case"] == c and r["arm"] == arm]
            succ = [r for r in rs if r["success"]]
            summary[c][arm] = {
                "n_runs": len(rs),
                "success_rate": round(len(succ) / len(rs), 3),
                "median_nq_success": median_or_none(
                    [r["nq"] for r in succ]),
                "median_pair_ops_success": median_or_none(
                    [r["pair_ops"] for r in succ]),
                "median_wall_success": median_or_none(
                    [r["wall_seconds"] for r in succ]),
                "median_nq_all": median_or_none([r["nq"] for r in rs]),
                "median_states": median_or_none(
                    [r["n_states"] for r in rs]),
                "min_margin_mm": median_or_none(
                    [r["cert_min_margin_mm"] for r in succ])}

    # ---- kill condition on the thin case ---------------------------------
    cont = summary["thin_0.505"]["contact_continuation"]
    kill = {"reference": cont, "arms": {}, "hit": False}
    if cont["success_rate"] > 0:
        for arm in BASELINE_ARMS:
            s = summary["thin_0.505"][arm]
            entry = {"success_rate": s["success_rate"]}
            for cur, ref_key in (("nq", "median_nq_success"),
                                 ("pair_ops", "median_pair_ops_success"),
                                 ("wall", "median_wall_success")):
                ref = cont[ref_key]
                val = s[ref_key]
                entry[cur + "_ratio"] = (round(val / ref, 2)
                                         if (val and ref) else None)
            entry["within_2x_any_currency"] = bool(
                s["success_rate"] >= 0.5 and any(
                    entry[c + "_ratio"] is not None
                    and entry[c + "_ratio"] <= 2.0
                    for c in ("nq", "pair_ops", "wall")))
            kill["arms"][arm] = entry
            kill["hit"] = kill["hit"] or entry["within_2x_any_currency"]
    kill_verdict = "HIT" if kill["hit"] else "NOT_HIT"

    # ---- negative control ------------------------------------------------
    neg = [r for r in rows if r["case"] == "plug_0.505"]
    neg_ok = all(not r["success"] for r in neg)
    overall = {
        "false_claims": int(false_claims),
        "negative_control_all_refused": bool(neg_ok),
        "p38_kill_condition": kill_verdict,
        "verdict": ("PASS" if false_claims == 0 and neg_ok else "FAIL"),
    }
    print("P38 overall:", overall, flush=True)
    from prov import make_provenance
    out = {"provenance": make_provenance(
        __file__, "P3.8 chart-factored connector benchmark: shared "
        "converged goal-free frontiers, six arms, three-currency billed "
        "runs, independent connector audit, round-10 2x kill rule "
        "(any-currency, generous to baselines); contact arm's G1-frame "
        "conditioning disclosed as an asymmetry in its own favor"),
        "budget": BUDGET, "chart_budget": CHART_BUDGET,
        "seeds": SEEDS, "seeds_negative": SEEDS_NEG,
        "cases": case_meta, "rows": rows, "summary": summary,
        "kill": kill, "overall": overall}
    os.makedirs(TABDIR, exist_ok=True)
    with open(os.path.join(TABDIR, "p38_connector.json"), "w") as f:
        json.dump(out, f, indent=1,
                  default=lambda o: o.item() if hasattr(o, "item")
                  else str(o))
    print("-> results/tables/p38_connector.json")


if __name__ == "__main__":
    main()
