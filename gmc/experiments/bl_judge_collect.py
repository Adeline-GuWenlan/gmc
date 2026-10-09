"""bl J ``collect`` (inline, json only): the (a)-(e) tables of the judge check and GMC's re-judged F4 rows.

Reads results/baselines/j/{tests,astar,straight,requery}/ (written by the sbatch jobs of ``bl_judge_check.py``) and
F4's rows; writes results/baselines/j/judge_check.json and results/baselines/gmc_rejudged/<R>/<robot>/rows.jsonl +
summary.json.  Re-judged rows: F4's row wherever the fix cannot change the verdict (the row never reached the shared
replay, or passed it: the fix only loosens), J's re-query row for the 538 judge-vetoed rows.
"""
from __future__ import annotations

import collections
import csv
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

from bl_judge_check import F4, J, OUT, REGIONS, ROBOTS, VETO, _dump, _jsonl

ALLOWED_FROM = "unknown|unresolved|map_unknown"


# ---------------------------------------------------------------------------------------------- (a) tests
def junit(path):
    out = {}
    for tc in ET.parse(path).getroot().iter("testcase"):
        tid = f"{tc.get('classname')}::{tc.get('name')}"
        kind, msg = "passed", ""
        for k in ("failure", "error", "skipped"):
            e = tc.find(k)
            if e is not None:
                kind, msg = k, (e.get("message") or "").strip()
                break
        out[tid] = (kind, msg)
    return out


def tests_table():
    head, old = J / "tests" / "junit_head.xml", J / "tests" / "junit_old.xml"
    if not (head.exists() and old.exists()):
        return None
    h, o = junit(head), junit(old)
    bad = ("failure", "error")
    hf = {t: v for t, v in h.items() if v[0] in bad}
    regress = [{"test": t, "head": v, "old": o.get(t)} for t, v in hf.items()
               if t not in o or o[t][0] not in bad or o[t][1] != v[1]]
    fixed = [{"test": t, "old": v, "head": h.get(t)} for t, v in o.items() if v[0] in bad and h.get(t, ("",))[0] not in bad]
    return {"head": dict(collections.Counter(v[0] for v in h.values())),
            "old": dict(collections.Counter(v[0] for v in o.values())),
            "only_in_head": sorted(set(h) - set(o)), "only_in_old": sorted(set(o) - set(h)),
            "head_failures": [{"test": t, "kind": v[0], "message": v[1][:300], "old": (o.get(t) or ("absent",))[0],
                               "same_message_old": (o.get(t) or ("", ""))[1] == v[1]} for t, v in sorted(hf.items())],
            "regressions": regress, "failing_on_old_only": fixed, "passed": not regress}


# ---------------------------------------------------------------------------------------------- (b) A*
def astar_table():
    out = {}
    for code in ("new", "old"):
        rows = [r for f in sorted((J / "astar" / code).glob("*.jsonl")) for r in _jsonl(f)]
        if not rows:
            continue
        t = {"n": len(rows), "per_region": {}, "route_stored_passed": 0, "export_stored_passed": 0,
             "final_route_passed": 0, "final_export_passed": 0, "fail_rows": [],
             "export_stored_fail_reasons": collections.Counter(), "route_stored_fail_reasons": collections.Counter(),
             "wall_s": float(sum(r["wall_s"] for r in rows))}
        for r in rows:
            rr, ex, rn = r["route"], r["export"], r.get("rerun") or {}
            fr = rr["passed"] or bool((rn.get("route") or {}).get("passed"))
            fe = ex["passed"] or bool((rn.get("export") or {}).get("passed"))
            t["route_stored_passed"] += rr["passed"]
            t["export_stored_passed"] += ex["passed"]
            t["final_route_passed"] += fr
            t["final_export_passed"] += fe
            if not rr["passed"]:
                t["route_stored_fail_reasons"][rr["reason"]] += 1
            if not ex["passed"]:
                t["export_stored_fail_reasons"][f"{ex['geometry_reason']}|kin={ex['kinematics_passed']}"] += 1
            pr = t["per_region"].setdefault(r["region"], collections.Counter())
            pr["n"] += 1
            pr["final_route_passed"] += fr
            pr["final_export_passed"] += fe
            if not (rr["passed"] and ex["passed"]):
                t["fail_rows"].append({"pair_id": r["pair_id"], "route": rr, "export": ex, "rerun": rn or None,
                                       "final_route_passed": fr, "final_export_passed": fe})
        for k in ("per_region",):
            t[k] = {a: dict(b) for a, b in t[k].items()}
        t["route_stored_fail_reasons"] = dict(t["route_stored_fail_reasons"])
        t["export_stored_fail_reasons"] = dict(t["export_stored_fail_reasons"])
        if code == "old":                   # context only: no re-run under old code
            t["fail_rows"] = [{k: x[k] for k in ("pair_id", "export")} for x in t["fail_rows"]]
        out[code] = t
    if "new" in out:
        out["passed"] = out["new"]["n"] == 5000 and out["new"]["final_route_passed"] == 5000
    return out


# ---------------------------------------------------------------------------------------------- (e) straight
def straight_table():
    data = {}
    for code in ("new", "old"):
        for robot in ROBOTS:
            for f in sorted((J / "straight" / code / robot).glob("*.jsonl*")):
                for r in _jsonl(f):
                    data[(code, robot, r["pair_id"])] = r
    keys = sorted({(rb, p) for c, rb, p in data if c == "new"} & {(rb, p) for c, rb, p in data if c == "old"})
    if not keys:
        return None
    out = {"n_pairs_by_robot": dict(collections.Counter(rb for rb, _ in keys)), "per_robot": {}}
    ok = True
    for robot in ROBOTS:
        ks = [k for k in keys if k[0] == robot]
        ex_t, edge_t, kin_t, over = collections.Counter(), collections.Counter(), collections.Counter(), collections.Counter()
        bad_edges, bad_paths, n_edges, clear_changed = [], [], 0, 0
        for k in ks:
            o, n = data[("old", *k)], data[("new", *k)]
            eo, en = o["export"], n["export"]
            vo = "PASS" if eo["passed"] else f"{eo['geometry_reason']}|kin={eo['kinematics_passed']}"
            vn = "PASS" if en["passed"] else f"{en['geometry_reason']}|kin={en['kinematics_passed']}"
            ex_t[f"{vo} -> {vn}"] += 1
            kin_t[f"{eo['kinematics_passed']} -> {en['kinematics_passed']}"] += 1
            if vo != vn and not vo.startswith("map_unknown"):
                bad_paths.append({"pair_id": k[1], "old": vo, "new": vn})
            if len(o["edges"]) != len(n["edges"]):
                bad_edges.append({"pair_id": k[1], "issue": "edge count differs"})
                continue
            for i, (a, b) in enumerate(zip(o["edges"], n["edges"])):
                n_edges += 1
                edge_t[f"{a} -> {b}"] += 1
                if a != b and a != ALLOWED_FROM:
                    bad_edges.append({"pair_id": k[1], "edge": i, "old": a, "new": b})
                if a == b and o["edge_clearance_lower_m"][i] != n["edge_clearance_lower_m"][i]:
                    clear_changed += 1
            for code, r in (("old", o), ("new", n)):
                for eps, passed in r["kin_overspeed_passed"].items():
                    over[f"{code} overspeed {eps}: {'passed' if passed else 'rejected'}"] += 1
        over_bad = sum(v for kk, v in over.items() if kk.endswith("passed"))
        occ_to_free = sum(v for kk, v in edge_t.items()
                          if kk.split(" -> ")[0].startswith("occupied") and kk.split(" -> ")[1].startswith("free"))
        out["per_robot"][robot] = {
            "pairs": len(ks), "edges": n_edges, "path_verdict_transitions": dict(ex_t.most_common()),
            "kinematics_transitions": dict(kin_t), "edge_verdict_transitions": dict(edge_t.most_common()),
            "edges_same_verdict_but_clearance_changed": clear_changed,
            "occupied_to_free_edges": occ_to_free, "disallowed_edge_transitions": bad_edges[:50],
            "n_disallowed_edge_transitions": len(bad_edges), "disallowed_path_transitions": bad_paths[:50],
            "n_disallowed_path_transitions": len(bad_paths), "overspeed": dict(over), "overspeed_accepted": over_bad}
        ok &= not bad_edges and not bad_paths and over_bad == 0 and occ_to_free == 0
    out["passed"] = ok
    return out


# ---------------------------------------------------------------------------------------------- (c)/(d) re-query
def f4_rows(reg, robot):
    from aerial3dg_fail3_f4 import gmc_rows
    return {r["pair_id"]: r for r in gmc_rows(reg, robot).values()}


def requery_tables(f4):
    vet, reg_ = {"rows": [], "status": collections.Counter()}, {"rows": [], "n": 0, "same": 0}
    fails = {(r["robot"], r["pair_id"]): r for r in csv.DictReader(open(F4 / "failures.csv"))}
    for reg in REGIONS:
        for robot in ROBOTS:
            d = J / "requery" / reg / robot
            for name in ("vetoed", "regression"):
                f = d / f"{name}.jsonl"
                if not f.exists():
                    continue
                for r in _jsonl(f):
                    old = f4[(reg, robot)][r["pair_id"]]
                    if name == "vetoed":
                        vet["status"][f"{r['status']}:{r['reason']}"] += 1
                        vet["rows"].append({"region": reg, "robot": robot, "pair_id": r["pair_id"],
                                            "f4_class": fails[(robot, r["pair_id"])]["class"],
                                            "status": r["status"], "reason": r["reason"],
                                            "shared_replay_passed": r.get("shared_replay_passed"),
                                            "own_verification": r.get("own_verification"),
                                            "outer_wall_s": r["outer_wall_s"], "compile_id": r["compile_id"],
                                            "compile_id_f4": old["compile_id"]})
                    else:
                        same = (r["status"] == "REACHABLE" and r["polyline_sha256"] == old["polyline_sha256"]
                                and r["compile_id"] == old["compile_id"])
                        reg_["n"] += 1
                        reg_["same"] += same
                        reg_["rows"].append({"region": reg, "robot": robot, "pair_id": r["pair_id"],
                                             "status": r["status"], "reason": r["reason"], "same": same,
                                             "polyline_sha256": r["polyline_sha256"],
                                             "f4_polyline_sha256": old["polyline_sha256"],
                                             "outer_wall_s": r["outer_wall_s"], "f4_outer_wall_s": old["outer_wall_s"]})
    vet["n"] = len(vet["rows"])
    vet["reachable"] = sum(r["status"] == "REACHABLE" for r in vet["rows"])
    vet["status"] = dict(vet["status"])
    vet["by_class"] = {c: dict(collections.Counter(r["status"] for r in vet["rows"] if r["f4_class"] == c))
                       for c in VETO}
    vet["not_reachable"] = [r for r in vet["rows"] if r["status"] != "REACHABLE"]
    reg_["passed"] = reg_["n"] >= 300 and reg_["same"] == reg_["n"]
    reg_["mismatches"] = [r for r in reg_["rows"] if not r["same"]]
    reg_["by_stratum"] = dict(collections.Counter(f"{r['region']}/{r['robot']}" for r in reg_["rows"]))
    return vet, reg_


# ---------------------------------------------------------------------------------------------- re-judged GMC rows
def classify_new(r, f4_class):
    if r["status"] == "REACHABLE":
        return "REACHABLE"
    if r["status"] == "TIMEOUT":
        return "METHOD-TIMEOUT"
    if r["reason"] == "shared_replay_failed":
        return "REPLAY-RESIDUAL"
    return f"OTHER:{r['status']}:{r['reason']}"


def rejudged(f4, vet_rows):
    from aerial3dg_fail3_f4 import EP_BANDS, LAT_BANDS, RATIO_BANDS, band_table, ep_min, group_of
    commit = subprocess.run(["git", "log", "-1", "--format=%H", "--", "src"], capture_output=True, text=True).stdout.strip()
    fails = {(r["robot"], r["pair_id"]): r for r in csv.DictReader(open(F4 / "failures.csv"))}
    pairs = {}
    for reg in REGIONS:
        pairs.update({p["pair_id"]: p for p in json.loads((F4 / "sample" / f"{reg}_pairs.json").read_text())["pairs"]})
    rq = {}
    for reg in REGIONS:
        for robot in ROBOTS:
            f = J / "requery" / reg / robot / "vetoed.jsonl"
            if f.exists():
                rq.update({(robot, r["pair_id"]): r for r in _jsonl(f)})
    f4sum = json.loads((F4 / "summary.json").read_text())
    summ = {"schema": "bl.gmc_rejudged.v1", "judge": "gmc.gs3d.trajectory.replay_plan at gmc/src " + commit,
            "rule": __doc__.strip(), "n_pairs": len(pairs), "regions": list(REGIONS), "per_robot": {},
            "per_region_robot": {}, "bands": {}, "before_f4": {r: f4sum["per_robot"][r] for r in ROBOTS},
            "transitions": {}}
    allx = []
    for robot in ROBOTS:
        for reg in REGIONS:
            rows = []
            for pid, r in sorted(f4[(reg, robot)].items(), key=lambda kv: kv[1]["index"]):
                fr = fails.get((robot, pid))
                f4c = fr["class"] if fr else "REACHABLE"
                if f4c in VETO:
                    if (robot, pid) not in rq:
                        raise SystemExit(f"missing re-query row for vetoed {robot} {pid}")
                    new = dict(rq[(robot, pid)])
                    src, cls = "J-requery", classify_new(new, f4c)
                else:
                    new, src, cls = dict(r), "F4", f4c
                new.update(region=reg, robot=robot, rejudge_source=src, f4_status=r["status"], f4_reason=r["reason"],
                           f4_class=f4c, **{"class": cls}, group=group_of(cls) if cls != "REPLAY-RESIDUAL" else "export")
                rows.append(new)
                allx.append({"robot": robot, "region": reg, "class": cls, "f4_class": f4c, "r": new, "p": pairs[pid]})
            d = OUT / reg / robot
            d.mkdir(parents=True, exist_ok=True)
            with open(d / "rows.jsonl", "w") as fh:
                for x in rows:
                    fh.write(json.dumps(x, default=float) + "\n")
    for robot in ROBOTS:
        rr = [x for x in allx if x["robot"] == robot]
        cls = collections.Counter(x["class"] for x in rr)
        summ["per_robot"][robot] = {"n": len(rr), "reachable": cls["REACHABLE"], "failed": len(rr) - cls["REACHABLE"],
                                    "fail_rate": (len(rr) - cls["REACHABLE"]) / len(rr), "class": dict(cls),
                                    "group": dict(collections.Counter(x["r"]["group"] for x in rr)),
                                    "status": dict(collections.Counter(x["r"]["status"] for x in rr)),
                                    "status_reason": dict(collections.Counter(
                                        f"{x['r']['status']}:{x['r']['reason']}" for x in rr)),
                                    "source": dict(collections.Counter(x["r"]["rejudge_source"] for x in rr))}
        summ["transitions"][robot] = dict(collections.Counter(f"{x['f4_class']} -> {x['class']}" for x in rr
                                                              if x["f4_class"] != x["class"]))
        for reg in REGIONS:
            q = [x for x in rr if x["region"] == reg]
            c = collections.Counter(x["class"] for x in q)
            w = np.array([x["r"]["outer_wall_s"] for x in q])
            summ["per_region_robot"][f"{reg}/{robot}"] = {
                "n": len(q), "reachable": c["REACHABLE"], "class": dict(c),
                "group": dict(collections.Counter(x["r"]["group"] for x in q)),
                "query_wall_s": {"sum": float(w.sum()), "mean": float(w.mean()), "p50": float(np.median(w)),
                                 "p90": float(np.percentile(w, 90)), "max": float(w.max())},
                "compile_id": sorted({x["r"]["compile_id"] for x in q}),
                "f4_reachable": sum(x["f4_class"] == "REACHABLE" for x in q)}
        summ["bands"][robot] = {
            "lateral": band_table(rr, lambda x: x["p"]["evidence"]["lateral_m"], LAT_BANDS,
                                  "route lateral clearance (ladder rung passed)"),
            "detour": band_table(rr, lambda x: x["p"]["evidence"]["len_ratio"], RATIO_BANDS,
                                 "A* route length / straight distance"),
            "endpoint": band_table(rr, ep_min, EP_BANDS, "min endpoint clearance ladder of the cylinder (mm)")}
        for reg in REGIONS:
            q = [x for x in rr if x["region"] == reg]
            summ["bands"][robot][f"detour_{reg}"] = band_table(q, lambda x: x["p"]["evidence"]["len_ratio"],
                                                               RATIO_BANDS, f"detour, {reg}")
            summ["bands"][robot][f"lateral_{reg}"] = band_table(q, lambda x: x["p"]["evidence"]["lateral_m"],
                                                                LAT_BANDS, f"lateral, {reg}")
    _dump(OUT / "summary.json", summ)
    return summ


def cmd_collect(a):
    f4 = {(reg, robot): f4_rows(reg, robot) for reg in REGIONS for robot in ROBOTS}
    doc = {"a_tests": tests_table(), "b_astar": astar_table()}
    vet, reg_ = requery_tables(f4)
    doc["c_vetoed"], doc["d_regression"] = vet, reg_
    doc["e_negative_control"] = straight_table()
    _dump(J / "judge_check.json", doc)
    for k, v in doc.items():
        if v is None:
            print(k, "missing")
            continue
        print(k, json.dumps({kk: vv for kk, vv in v.items() if not isinstance(vv, list) or len(vv) < 6},
                            default=float)[:1500])
    if vet["n"] == 538:
        s = rejudged(f4, vet["rows"])
        for robot in ROBOTS:
            print(robot, "before", s["before_f4"][robot]["reachable"], "after", s["per_robot"][robot]["reachable"],
                  s["per_robot"][robot]["class"], s["transitions"][robot])
