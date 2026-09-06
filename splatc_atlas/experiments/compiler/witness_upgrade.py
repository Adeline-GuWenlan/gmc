"""Witness upgrade protocol for the six ORACLE_UNRESOLVED_GRID episodes
(review round-4 answer #2): the candidate must NOT edit its own ground
truth.  This script writes AMENDMENT files alongside (never over) the
original oracle records:

  outputs/oracle_records_amendments/<qid>.json
    original_status  : ORACLE_UNRESOLVED_GRID (preserved)
    reachable_by_witness : true only if BOTH replay formulations pass
    witness_wps      : the certified polyline
    dual_formulation_replay: checker#1 (PW sign, the probe evaluator) and
            checker#2 (point-to-ellipse metric, the certificate's checker)
            sampled every ~2mm of motion along the witness; min margins +
            any violations recorded.  These are two FORMULATIONS sharing
            one source tree, not independent implementations (round-6
            naming fix; a GJK/interval backend remains future work).
    provenance       : script/src hashes, UTC

These episodes are thereafter EXCLUDED from independent-accuracy
statistics (they are post-hoc resolved validation cases, not untouched
truth).

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/witness_upgrade.py
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from splatc.datasets.g1_gate import make_g1_scene, robot_library
from splatc.reference.oracle import metric_margin
from splatc.common.se2 import wrap_diff
from ridge_continuation import run_case

BASE = os.path.join(os.path.dirname(__file__), "..", "..")
DATA = os.path.join(BASE, "data", "splatc_gates")
OUT = os.path.join(BASE, "outputs", "oracle_records_amendments")
UNRESOLVED = ["dev_003", "dev_004", "val_000", "val_001",
              "val_003", "val_004"]


def episode_params(qid):
    split = "dev" if qid.startswith("dev") else "validation"
    with open(os.path.join(DATA, split, "manifest.json")) as f:
        m = json.load(f)
    e = next(e for e in m["episodes"] if e["query_id"] == qid)
    s = e["scene"]
    return (s["door_width"], s["door_offset"], s["door_tilt_deg"],
            e["robot_id"])


def replay(scene, robot, wps, ds=0.002):
    """Dense dual-formulation replay with both checkers."""
    min_m2 = np.inf
    viol1 = viol2 = n = 0
    for p, qn in zip(wps[:-1], wps[1:]):
        dth = wrap_diff(qn[2] - p[2])
        L = np.hypot(qn[0] - p[0], qn[1] - p[1]) + robot.a * abs(dth)
        k = max(2, int(np.ceil(L / ds)) + 1)
        for t in np.linspace(0.0, 1.0, k):
            pose = (p[0] + t * (qn[0] - p[0]), p[1] + t * (qn[1] - p[1]),
                    p[2] + t * dth)
            ok1, _ = scene.check_pose(robot, pose)       # checker #1 (PW)
            m2 = metric_margin(scene, robot, pose)       # checker #2
            n += 1
            viol1 += (not ok1)
            viol2 += (m2 <= 0)
            min_m2 = min(min_m2, m2)
    return {"samples": n, "checker1_violations": int(viol1),
            "checker2_violations": int(viol2),
            "min_metric_margin_mm": round(min_m2 * 1000, 3)}


def main():
    os.makedirs(OUT, exist_ok=True)
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from prov import make_provenance
    for qid in UNRESOLVED:
        w, dy0, tilt, rid = episode_params(qid)
        row, diag = run_case(w, dy0=dy0, tilt_deg=tilt, robot_id=rid)
        if row["status"] != "CERTIFIED_REACHABLE":
            print(f"{qid}: NOT certified ({row['status']}) — no amendment",
                  flush=True)
            continue
        wps = diag["profile"]["wps"]
        scene = make_g1_scene(w, door_offset=dy0,
                              door_tilt=np.radians(tilt))
        robot = robot_library()[rid]
        rep = replay(scene, robot, wps)
        ok = rep["checker1_violations"] == 0 \
            and rep["checker2_violations"] == 0
        amendment = {
            "query_id": qid,
            "original_status": "ORACLE_UNRESOLVED_GRID",
            "original_record_untouched": True,
            "reachable_by_witness": bool(ok),
            "witness_source": "ridge_continuation v4.1 certified output",
            "witness_wps": [[float(v) for v in p] for p in wps],
            "certificate_min_margin_mm": row["min_margin_mm"],
            "dual_formulation_replay": rep,
            "excluded_from_independent_accuracy_stats": True,
            "provenance": make_provenance(
                __file__, "witness upgrade replay (checker#1 sign + "
                "checker#2 metric, 2mm sampling)"),
        }
        with open(os.path.join(OUT, f"{qid}.json"), "w") as f:
            json.dump(amendment, f, indent=1)
        print(f"{qid}: witness {len(wps)} wps, replay {rep['samples']} "
              f"samples, viol {rep['checker1_violations']}/"
              f"{rep['checker2_violations']}, min "
              f"{rep['min_metric_margin_mm']}mm -> "
              f"reachable_by_witness={ok}", flush=True)


if __name__ == "__main__":
    main()
