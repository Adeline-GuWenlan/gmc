"""F1 Task 2: assign EVERY non-REACHABLE G2 row to exactly one failure class (pure json/csv crunching).

Inputs (all committed under results/aerial3dg/): G2 per-pair rows, F1 diagnostics (diag/), fix probes (fix/),
widened-box runs (widen/), oracle sample (oracle/).  Outputs: f1/classes.csv, f1/classes.json (counts,
index lists per class for the oracle sampler), f1/class_maps.png.

Classes (mechanism, traced in docs/aerial3dg_failures.md):
  SW-EP   sweeper UNKNOWN {start,goal}_not_certified_free: endpoint oracle-free but within the octree buffer
          (clearance in (margin, margin + buffer]) of a captured Gaussian lying wholly under the chassis
          (its 2-sigma top below the chassis bottom); -LAT = the nearest blocker reaches into the body band
  SW-KIN  sweeper UNKNOWN shared_replay_failed: own verifier and replay geometry pass; the exported
          trajectory fails the replay's yaw-rate check by float round-off on a tiny in-place turn
  CY-LAMP cylinder UNREACHABLE possible_space_cut across the lamp + bulkhead (certified)
  CY-GAP  cylinder UNKNOWN safe_graph_disconnected_possible_connected across u~-5.6
  CY-EP   cylinder UNKNOWN {start,goal}_not_certified_free (same mechanism as SW-EP)
A row whose diagnostics do not match its class's mechanism goes to <class>-OTHER (reported, not smoothed).
"""
from __future__ import annotations

import csv
import glob
import json
from pathlib import Path

import numpy as np

F1 = Path("results/aerial3dg/f1")
G2 = Path("results/aerial3dg/g2")
LAMP_U, GAP_U = (-1.03, -.67), -5.6


def rows_of(pattern):
    out = {}
    for f in sorted(glob.glob(str(pattern))):
        for line in open(f):
            r = json.loads(line)
            out[r["index"]] = r
    return out


def load(path):
    p = F1 / path
    return json.loads(p.read_text()) if p.exists() else None


def straddles(r, u):
    lo, hi = sorted((r["start_uv"][0], r["goal_uv"][0]))
    return (lo < u[1] and hi > u[0]) if isinstance(u, tuple) else lo < u < hi


def main():
    g2 = {rb: rows_of(G2 / f"runs/{rb}/task_*.jsonl") for rb in ("sweeper", "cylinder")}
    ep = {rb: {(x["index"]): x for x in (load(f"diag/endpoints_{rb}.json") or {"rows": []})["rows"]}
          for rb in ("sweeper", "cylinder")}
    kin = {x["index"]: x for x in (load("fix/kinematics_sweeper.json") or {"rows": []})["rows"]}
    br = load("diag/bridges_cylinder.json") or {"per_pair": [], "corridors": {}}
    brp = {x["index"]: x for x in br["per_pair"]}
    fix_ep = {rb: {x["index"]: x for x in (load(f"fix/endpoints_buffer0_{rb}.json") or {"rows": []})["rows"]}
              for rb in ("sweeper", "cylinder")}
    widen = {}
    for tag in ("w1", "w2"):
        for rb in ("sweeper", "cylinder"):
            d = rows_of(F1 / f"widen/{tag}/{rb}/fast_*.jsonl") or rows_of(F1 / f"widen/{tag}/{rb}/task_*.jsonl")
            if d:
                widen[(tag, rb)] = d
    g2fast = {rb: rows_of(F1 / f"widen/g2/{rb}/fast_*.jsonl") for rb in ("sweeper", "cylinder")}
    oracle = {}
    for rb in ("sweeper", "cylinder"):
        d = load(f"oracle/astar_{rb}.json")
        for x in (d or {"rows": []})["rows"]:
            oracle[(rb, x["index"])] = x
    out, classes = [], {"sweeper": {}, "cylinder": {}}
    for rb in ("sweeper", "cylinder"):
        for i, r in sorted(g2[rb].items()):
            if r["status"] == "REACHABLE":
                continue
            ev = {}
            reason = r["reason"]
            if reason.endswith("_not_certified_free"):
                cls = "SW-EP" if rb == "sweeper" else "CY-EP"
                d = ep[rb].get(i)
                if d is None:
                    cls += "-UNDIAGNOSED"
                else:
                    b = min(d["blockers"], key=lambda b: b["refined_gap_m"]) if d["blockers"] else None
                    ev.update(endpoint=d["endpoint"], leaf=";".join(d["leaf_status"]), oracle_clearance_m=d["oracle"]["clearance_lower_m"],
                              blocker_scene_id=b and b["scene_id"], blocker_role=b and b["role"],
                              blocker_top_z=b and round(b["top_z_2sigma"], 5),
                              blocker_vertical_gap_m=b and round(b["vertical_gap_to_chassis_bottom_m"], 5),
                              refined_gap_m=b and round(b["refined_gap_m"], 6), n_blockers=d["n_blockers"])
                    if d["cause"] == "domain_boundary" or b is None:
                        cls += "-OTHER:" + d["cause"]
                    elif b["vertical_gap_to_chassis_bottom_m"] < 0:
                        cls += "-LAT"      # the nearest blocker reaches into the body band: side contact
                f = fix_ep[rb].get(i)
                if f:
                    ev["buffer0_status"] = f"{f['status']}:{f['reason']}"
            elif reason == "shared_replay_failed":
                cls = "SW-KIN" if rb == "sweeper" else "CY-KIN"
                k = kin.get(i)
                if k is None:
                    cls += "-UNDIAGNOSED"
                else:
                    v = k["violations"][0] if k["violations"] else None
                    ev.update(n_violations=k["n_violations"], yaw_change_rad=v and v["yaw_change_rad"],
                              yaw_rate_excess=v and v["yaw_rate_excess"], t_s=v and round(v["t_s"], 3),
                              turn_floor_replay_passed=k["snapped_replay"]["passed"])
                    if not (v and v["translation_m"] == 0 and abs(v["yaw_change_rad"]) < 1e-5
                            and v["yaw_rate_excess"] < 1e-7 and k["n_violations"] == 1):
                        cls += "-OTHER"
            elif reason == "possible_space_cut":
                cls = "CY-LAMP" if straddles(r, LAMP_U) and (r.get("cut_pairs_by_role") or {}).get("lamp") else "CY-CUT-OTHER"
                ev.update(cut_pairs=r.get("cut_distinct_pairs"), cut_roles=json.dumps(r.get("cut_pairs_by_role")))
            elif reason == "safe_graph_disconnected_possible_connected":
                cls = "CY-GAP" if straddles(r, GAP_U) else "CY-DISC-OTHER"
                b = brp.get(i)
                if b:
                    ev.update(start_comp=";".join(map(str, b["start_comps"])), goal_comp=";".join(map(str, b["goal_comps"])))
            else:
                cls = f"{rb[:2].upper()}-OTHER:{reason}"
            for tag in ("w1", "w2"):
                w = widen.get((tag, rb), {}).get(i)
                if w:
                    ev[f"{tag}_status"] = f"{w['status']}:{w['reason']}"
            if g2fast[rb].get(i):
                ev["g2fast_status"] = f"{g2fast[rb][i]['status']}:{g2fast[rb][i]['reason']}"
            o = oracle.get((rb, i))
            if o:
                ev["astar"] = o["outcome"]
                ev["astar_len_m"] = o.get("path_length_m") and round(o["path_length_m"], 3)
            row = {"pair_id": r["pair_id"], "index": i, "robot": rb, "status": r["status"], "reason": reason,
                   "class": cls, "dist_m": round(r["dist_m"], 3), "start_u": r["start_uv"][0], "start_v": r["start_uv"][1],
                   "goal_u": r["goal_uv"][0], "goal_v": r["goal_uv"][1], "evidence": json.dumps(ev, sort_keys=True)}
            out.append(row)
            classes[rb].setdefault(cls, []).append(i)
    F1.mkdir(parents=True, exist_ok=True)
    with open(F1 / "classes.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0]))
        w.writeheader()
        w.writerows(out)
    counts = {rb: {k: len(v) for k, v in sorted(c.items())} for rb, c in classes.items()}
    total = {rb: sum(1 for r in g2[rb].values() if r["status"] != "REACHABLE") for rb in g2}
    assert all(sum(counts[rb].values()) == total[rb] for rb in g2), (counts, total)
    (F1 / "classes.json").write_text(json.dumps({"counts": counts, "non_reachable_rows": total,
                                                 "classes": classes}, indent=1) + "\n")
    print(json.dumps({"counts": counts, "total": total}, indent=1))
    xt = {}
    for row in out:
        ev = json.loads(row["evidence"])
        for k in ("w1_status", "w2_status", "g2fast_status", "astar", "buffer0_status", "turn_floor_replay_passed"):
            if k in ev:
                xt.setdefault(row["class"], {}).setdefault(k, {})
                key = str(ev[k])
                xt[row["class"]][k][key] = xt[row["class"]][k].get(key, 0) + 1
    (F1 / "class_crosstabs.json").write_text(json.dumps(xt, indent=1) + "\n")
    print(json.dumps(xt, indent=1))


if __name__ == "__main__":
    main()
