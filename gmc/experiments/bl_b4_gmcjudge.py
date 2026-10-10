"""bl B4 audit: GMC's re-judged REACHABLE routes through the baselines' own judge path.

GMC's re-judged column (stage J, ``results/baselines/gmc_rejudged/``) keeps F4's verdict wherever the d757729 fix
cannot change it (the fix only loosens), and re-queried the 538 vetoed rows with the fixed code. This script closes
the remaining gap by evidence instead of argument: every REACHABLE GMC row's stored ``route_polyline`` (plan frame,
0.1 mm-rounded) goes through exactly what every baseline row went through -- ``bl_harness.judge_row``: completion,
GMC's exporter (``api._gs3d_result`` + ``_densify`` 0.20 m), ``replay_plan`` on the SHA-checked F3 compile (gmc/src
at d757729, unchanged since). Expected: all pass, except rows whose 0.1 mm rounding alone pushes the 1 mm margin
(F5 / J (b) saw this on A* routes); those are reported with their clearance, never re-labelled.

Run (CPU sbatch, gmc-venv, from gmc/): python experiments/bl_b4_gmcjudge.py --region R --robot X
Writes results/baselines/b4/gmc_judge/<R>_<robot>.jsonl.gz and .summary.json.
"""
from __future__ import annotations

import argparse
import collections
import gzip
import json
import time
from pathlib import Path

import numpy as np

import bl_harness as H

OUT = Path("results/baselines/b4/gmc_judge")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--region", required=True)
    ap.add_argument("--robot", required=True)
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    src = Path(f"results/baselines/gmc_rejudged/{a.region}/{a.robot}/rows.jsonl")
    rows = [json.loads(x) for x in src.read_text().splitlines() if x.strip()]
    t0 = time.perf_counter()
    J = H.Judge(a.region, a.robot)
    out_rows, cnt = [], collections.Counter()
    for r in rows:
        if r["status"] != "REACHABLE":
            continue
        uv = np.asarray(r["route_polyline"], float)[:, :2]
        q = {"pair_id": r["pair_id"], "start_uv": r["start_uv"], "goal_uv": r["goal_uv"],
             "method_path_uv": uv.tolist(), "claimed_reason": r["reason"]}
        H.judge_row(J, q)
        o = {k: q.get(k) for k in ("pair_id", "status", "judge_reason", "judge_geometry_reason",
                                   "judge_kinematics_reason", "judge_clearance_lower_m", "judge_wall_s",
                                   "judge_fail_location", "prepended_start_segment", "appended_goal_segment",
                                   "start_gap_m", "goal_gap_m", "path_length_m", "vertices", "polyline_sha256")}
        o.update(gmc_rejudge_source=r["rejudge_source"], gmc_clearance_lower_m=r.get("clearance_lower_m"),
                 gmc_polyline_sha256=r["polyline_sha256"],
                 sha_equal_to_gmc=q["polyline_sha256"] == r["polyline_sha256"])
        cnt[o["status"]] += 1
        out_rows.append(o)
    with gzip.open(OUT / f"{a.region}_{a.robot}.jsonl.gz", "wt") as f:
        for o in out_rows:
            f.write(json.dumps(o) + "\n")
    summ = {"schema": "bl.b4_gmc_judge.v1", "region": a.region, "robot": a.robot,
            "source": str(src), "judge_scene": {"a3c": str(J.a3c), "a3c_sha256": J.a3c_sha256},
            "reachable_rows": len(out_rows), "status_counts": dict(cnt),
            "not_success": [{k: o[k] for k in ("pair_id", "status", "judge_reason", "judge_clearance_lower_m",
                                               "gmc_clearance_lower_m", "gmc_rejudge_source")}
                            for o in out_rows if o["status"] != "SUCCESS"],
            "sha_equal_to_gmc": sum(o["sha_equal_to_gmc"] for o in out_rows),
            "judge_wall_s_sum": float(sum(o["judge_wall_s"] for o in out_rows)),
            "wall_s": time.perf_counter() - t0, "host": H.host()}
    (OUT / f"{a.region}_{a.robot}.summary.json").write_text(json.dumps(summ, indent=1))
    print(json.dumps({k: summ[k] for k in ("region", "robot", "reachable_rows", "status_counts", "wall_s")}))


if __name__ == "__main__":
    main()
