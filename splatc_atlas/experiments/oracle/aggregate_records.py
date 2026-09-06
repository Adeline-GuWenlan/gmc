"""Aggregate OracleRecord JSONs (from the HPC labeling array) into the
Sprint B baseline-truth table: per-resolution false-unreachable counts vs
analytic truth, convergence statuses, gate-interval errors, path stats.

Run:  PYTHONPATH=src python experiments/oracle/aggregate_records.py [records_dir]
"""
import json
import os
import sys
from collections import Counter

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT = os.path.join(HERE, "..", "..", "outputs", "oracle_records")


def main(d):
    recs = []
    for fn in sorted(os.listdir(d)):
        if fn.endswith(".json"):
            with open(os.path.join(d, fn)) as f:
                recs.append(json.load(f))
    print(f"{len(recs)} records from {d}\n")

    conv = Counter(r["convergence_status"] for r in recs)
    print("convergence:", dict(conv))

    # false-unreachable vs analytic (single-goal episodes only)
    print("\nfalse-unreachable vs analytic truth (single-goal episodes):")
    for res in ("coarse", "medium", "fine"):
        fu = fr = n = 0
        for r in recs:
            if len(r["resolutions"][res]["reachable_per_goal"]) != 1:
                continue
            n += 1
            reach = r["resolutions"][res]["reachable_per_goal"][0]
            if r["analytic"]["physically_open"] and not reach:
                fu += 1
            if not r["analytic"]["physically_open"] and reach:
                fr += 1
        print(f"  {res:7s}: false-unreachable {fu:2d}/{n}  false-reachable {fr}/{n}")

    # gate interval error where measured
    errs = [abs(r["measured_gate_half_angle_deg"]
                - r["analytic"]["gate_half_angle_deg"])
            for r in recs if r["analytic"]["physically_open"]]
    print(f"\ngate half-angle |measured-analytic|: "
          f"max {max(errs):.3f}°  median {float(np.median(errs)):.3f}°")

    # certified paths
    paths = [r["reference_path"] for r in recs if r["reference_path"]]
    cert = sum(1 for p in paths if p["certified"])
    print(f"\nreference paths: {len(paths)} extracted, {cert} conservatively "
          f"certified; min margin "
          f"{min((p['min_metric_margin_m'] for p in paths)) * 1000:.2f} mm")

    # unresolved episodes listed explicitly (protocol: they leave main eval)
    unres = [r["query_id"] for r in recs
             if r["convergence_status"] != "converged"]
    print(f"\nORACLE_UNRESOLVED_GRID episodes ({len(unres)}): {unres}")

    # runtimes
    for res in ("coarse", "medium", "fine"):
        ts = [r["resolutions"][res]["seconds"] for r in recs]
        print(f"runtime {res}: median {float(np.median(ts)):.1f}s "
              f"max {max(ts):.1f}s")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else DEFAULT)
