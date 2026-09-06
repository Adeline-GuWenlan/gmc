"""P2 generalization run (review round-3): FROZEN v4.1 hyperparameters,
dev + validation episode families (offset/tilt x width x morphology),
tracker outcomes compared against the sealed oracle truth records.

No blind episodes are touched.  No hyperparameter may be changed here —
this file only parameterizes run_case over the episode list.

Classification vs fine-resolution oracle truth:
  truth REACH  + CERTIFIED_REACHABLE -> certified_true_positive
  truth REACH  + UNKNOWN/NO_SEED     -> abstain_on_reachable (honest miss)
  truth UNREACH+ CERTIFIED_REACHABLE -> FALSE_REACHABLE (must stay ZERO)
  truth UNREACH+ UNKNOWN/NO_SEED     -> correct_abstention
  oracle ORACLE_UNRESOLVED_GRID      -> reported separately, no verdict

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/p2_generalization.py
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from ridge_continuation import run_case

BASE = os.path.join(os.path.dirname(__file__), "..", "..")
DATA = os.path.join(BASE, "data", "splatc_gates")
RECS = os.path.join(BASE, "outputs", "oracle_records")
TABDIR = os.path.join(BASE, "results", "tables")


def load_episodes():
    eps = []
    for split in ("dev", "validation"):
        with open(os.path.join(DATA, split, "manifest.json")) as f:
            m = json.load(f)
        for e in m["episodes"]:
            e["split"] = split
            eps.append(e)
    return eps


def truth_for(qid):
    p = os.path.join(RECS, f"{qid}.json")
    if not os.path.exists(p):
        return None
    with open(p) as f:
        r = json.load(f)
    fine = r["resolutions"]["fine"]
    # schema guard (review round-4 §7.2): an unresolved oracle grid must
    # read reachable=None, never False — the raw fine-grid False is a
    # discretization artifact and must not leak into any statistics
    reachable = (bool(fine["reachable_per_goal"][0])
                 if r["convergence_status"] == "converged" else None)
    return {"status": r["convergence_status"],
            "reachable": reachable,
            "physically_open": r["analytic"]["physically_open"],
            "gate_half_angle_deg": r["analytic"]["gate_half_angle_deg"]}


def classify(truth, status):
    if truth is None:
        return "no_truth_record"
    if truth["status"] != "converged":
        return "oracle_unresolved"
    cert = status == "CERTIFIED_REACHABLE"
    if truth["reachable"]:
        return "certified_true_positive" if cert else "abstain_on_reachable"
    return "FALSE_REACHABLE" if cert else "correct_abstention"


def main():
    eps = load_episodes()
    rows = []
    counts = {}
    t00 = time.time()
    for e in eps:
        s = e["scene"]
        qid = e["query_id"]
        t0 = time.time()
        try:
            row, diag = run_case(s["door_width"], dy0=s["door_offset"],
                                 tilt_deg=s["door_tilt_deg"],
                                 robot_id=e["robot_id"])
        except Exception as ex:                # a crash is data, not an exit
            row, diag = {"status": f"CRASH:{type(ex).__name__}"}, {}
        truth = truth_for(qid)
        verdict = classify(truth, row.get("status"))
        counts[verdict] = counts.get(verdict, 0) + 1
        out = {"query_id": qid, "split": e["split"],
               "w": s["door_width"], "dy0": s["door_offset"],
               "tilt_deg": s["door_tilt_deg"], "robot": e["robot_id"],
               "truth": truth, "verdict": verdict,
               "seconds": round(time.time() - t0, 1)}
        for k in ("status", "c_seed", "c_track", "c_densify", "c_cert",
                  "c_connector", "total", "stations", "min_margin_mm",
                  "n_sections", "n_events", "x_range"):
            if k in row:
                out[k] = row[k]
        for k in ("terminated", "seed_tries"):
            if k in diag:
                out[k] = diag[k]
        rows.append(out)
        print(f"{qid} {e['robot_id']:15s} w={s['door_width']:.2f} "
              f"dy={s['door_offset']:+.3f} tilt={s['door_tilt_deg']:+.1f} "
              f"-> {row.get('status'):22s} {verdict}", flush=True)
    summary = {"counts": counts,
               "false_reachable": counts.get("FALSE_REACHABLE", 0),
               "n_episodes": len(rows),
               "seconds": round(time.time() - t00, 1)}
    print("SUMMARY", summary, flush=True)
    from prov import make_provenance
    out = {"provenance": make_provenance(
        __file__, "P2 generalization: continuation-v4.1 frozen, "
        "dev+validation, oracle-truth comparison"),
        "summary": summary, "rows": rows}
    os.makedirs(TABDIR, exist_ok=True)
    with open(os.path.join(TABDIR, "p2_generalization.json"), "w") as f:
        json.dump(out, f, indent=1,
                  default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print("-> results/tables/p2_generalization.json")


if __name__ == "__main__":
    main()
