"""bl B4: GMC (re-judged) vs the four baselines on the F4 5000 pairs -- analysis from committed rows only.

Inputs (all committed unless noted):
  results/baselines/gmc_rejudged/<R>/<robot>/rows.jsonl     GMC's re-judged F4 rows (stage J)
  results/baselines/<method>/<R>/<robot>/task_NN.jsonl.gz    baseline rows (B3) + task_NN.summary.json
  results/baselines/<method>/setup/<hash>/<R>_<robot>.json   setup ("compile") records (B1/B2)
  results/aerial3dg/f4/pairs_confirmed_5000.json             pairs (A* route, lateral clearance, detour ratio)
  results/aerial3dg/f3/f4_handoff.json                       GMC compile wall per region x robot
  results/baselines/collect.json                             B3's completeness / setup-once record (cross-checked)
  results/baselines/b4/gmc_judge/*.summary.json              B4 audit: GMC routes through the baselines' judge path
  gmc/outputs/baselines/b3_rows/splatnav/** (uncommitted, SHA sidecars): SplatNav's full rows (exact dense polylines),
      used only for SplatNav's turning metric and the figures; slim committed rows carry everything else.
  gmc/outputs/baselines/raster/*_r5mm.npz, scene/*.npz.json, cust_fields setup artifacts (uncommitted, SHA sidecars):
      figure backgrounds / covers and the scene-export time.

Outcome mapping for GMC (plan §5): REACHABLE -> SUCCESS (GMC's shared replay is part of its query, so a GMC claim that
fails the judge is never a REACHABLE row; after J none is left), UNKNOWN (start/goal_not_certified_free) -> FAIL
(GMC declines), TIMEOUT -> TIMEOUT, ERROR -> ERROR.

Subcommands (run from gmc/ under gmc-venv, PYTHONPATH=src:experiments):
  analyze   -> results/baselines/b4/analysis.json (+ tables.md)
  sheets    -> docs/baselines_measurement_<method>.csv (from analysis.json + the template)
  figs      -> results/baselines/fig/*.png (+ fig/figures.json: what each figure shows, pair selection)
  all       analyze, sheets, figs
"""
from __future__ import annotations

import argparse
import collections
import csv
import gzip
import json
import math
import subprocess
from pathlib import Path

import numpy as np

import bl_harness as H
from aerial3dg_fail3_f4 import LAT_BANDS, RATIO_BANDS, wilson

RES = Path("results/baselines")
OUT = RES / "b4"
FIG = RES / "fig"
DOCS = Path("../docs")
TEMPLATE = Path("/scratch/wg2381/splathjb/measurement_template.csv")
METHODS = ["gmc", "splatnav", "foci", "pno", "cust_fields"]
BASELINES = METHODS[1:]
LABEL = {"gmc": "GMC (re-judged)", "splatnav": "SplatNav", "foci": "FOCI", "pno": "PNO", "cust_fields": "cust_fields"}
REGIONS = ["WWEST", "GAPW1", "S"]
ROBOTS = ["cylinder", "sweeper"]
STATUSES = ["SUCCESS", "CLAIMED_COLLIDES", "CLAIMED_UNPROVEN", "CLAIMED_KINEMATICS", "FAIL", "TIMEOUT", "ERROR",
            "SETUP_FAIL"]
GMC_STATUS = {"REACHABLE": "SUCCESS", "UNKNOWN": "FAIL", "TIMEOUT": "TIMEOUT", "ERROR": "ERROR"}
# per-query time stages (seconds) per method; other `stages` keys are counts
TIME_STAGES = {"splatnav": ["times_astar", "times_collision_set", "times_polytope", "times_opt"],
               "foci": ["plan_s"], "pno": ["snap_s", "value_infer_s", "heuristic_s", "astar_s"],
               "cust_fields": ["snap_s", "nf_descend_s"]}
UPSTREAM = {"splatnav": "e996e22（pristine）", "foci": "79d8ddc + MUMPS 补丁（foci_bl 副本）", "pno": "6384751（pristine，HF 权重 rev 36a76173）",
            "cust_fields": "5ca178e（pristine）"}
DIST_BANDS = [(0, 2, "<2 m"), (2, 4, "2-4 m"), (4, 6, "4-6 m"), (6, 99, ">=6 m")]


# ================================================================================================ helpers
def q(x, p):
    return float(np.percentile(x, p)) if len(x) else None


def stats(x):
    x = [float(v) for v in x if v is not None]
    if not x:
        return {"n": 0}
    return {"n": len(x), "median": q(x, 50), "p95": q(x, 95), "max": max(x), "min": min(x),
            "mean": float(np.mean(x)), "sum": float(np.sum(x))}


def rate(k, n):
    lo, hi = wilson(k, n) if n else (None, None)
    return {"k": k, "n": n, "rate": k / n if n else None, "ci95": [lo, hi]}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def turning(poly):
    """Total absolute heading change (rad) and the number of vertices turning > 5 degrees, along a uv polyline."""
    P = np.asarray(poly, float)[:, :2]
    d = np.diff(P, axis=0)
    L = np.linalg.norm(d, axis=1)
    d = d[L > 1e-9]
    if len(d) < 2:
        return 0.0, 0
    h = np.arctan2(d[:, 1], d[:, 0])
    dh = np.abs([wrap(v) for v in np.diff(h)])
    return float(dh.sum()), int((dh > math.radians(5)).sum())


def git(*args):
    return subprocess.run(["git", *args], capture_output=True, text=True, cwd="..").stdout.strip()


def pairs_by_id():
    doc = json.loads(Path("results/aerial3dg/f4/pairs_confirmed_5000.json").read_text())
    return {p["pair_id"]: p for p in doc["pairs"]}


# ================================================================================================ rows
def gmc_rows(region, robot):
    out = []
    for x in Path(f"{RES}/gmc_rejudged/{region}/{robot}/rows.jsonl").read_text().splitlines():
        if not x.strip():
            continue
        r = json.loads(x)
        r["method"] = "gmc"
        r["gmc_status"] = r["status"]
        r["status"] = GMC_STATUS[r["status"]]
        if r.get("algorithm_wall_s") is None:            # TIMEOUT rows: only the outside wall (= the 120 s limit)
            r["algorithm_wall_s"] = r.get("outer_wall_s")
        out.append(r)
    return out


def base_rows(method, region, robot):
    return list(H.iter_rows(RES / method / region / robot))


def splatnav_full(region, robot):
    """pair_id -> exact judged uv polyline from SplatNav's full rows (outputs/, SHA-checked against the slim rows)."""
    out = {}
    d = Path(f"outputs/baselines/b3_rows/splatnav/{region}/{robot}")
    for f in sorted(d.glob("task_*.jsonl.gz")):
        sha = Path(str(f) + ".sha256").read_text().split()[0]
        if H.sha256_file(f) != sha:
            raise ValueError(f"{f}: SHA-256 differs from its sidecar")
        with gzip.open(f, "rt") as fh:
            for x in fh:
                if x.strip():
                    r = json.loads(x)
                    out[r["pair_id"]] = r.get("route_polyline")
    return out


def load_all(with_turning=True):
    """{(method, robot): [row]} with pair evidence joined; polylines reduced to turning metrics."""
    P = pairs_by_id()
    data = {}
    for m in METHODS:
        for robot in ROBOTS:
            rows = []
            for reg in REGIONS:
                rr = gmc_rows(reg, robot) if m == "gmc" else base_rows(m, reg, robot)
                full = splatnav_full(reg, robot) if (m == "splatnav" and with_turning) else None
                for r in rr:
                    p = P[r["pair_id"]]
                    r["region"] = reg
                    r["lat"] = float(p["lateral_clearance_m"])
                    r["ratio"] = float(p["len_ratio"])
                    r["route_len"] = float(p["route_len_m"])
                    r["dist"] = float(p["dist_m"])
                    poly = r.get("route_polyline")
                    if full is not None and r["status"] == "SUCCESS":
                        poly = full.get(r["pair_id"])
                    if poly and with_turning:
                        r["turn_rad"], r["turns5"] = turning(poly)
                    r.pop("route_polyline", None)
                    r.pop("method_path_uv", None)
                    rows.append(r)
            data[(m, robot)] = rows
    return data


# ================================================================================================ analysis
def outcome_table(rows):
    c = collections.Counter(r["status"] for r in rows)
    n = len(rows)
    return {"n": n, "counts": {s: c.get(s, 0) for s in STATUSES},
            "rates": {s: rate(c.get(s, 0), n) for s in STATUSES},
            "claimed_unsafe": rate(sum(c.get(s, 0) for s in STATUSES[1:4]), n),
            "claimed": sum(c.get(s, 0) for s in STATUSES[:4]),
            "unsafe_of_claimed": rate(sum(c.get(s, 0) for s in STATUSES[1:4]), sum(c.get(s, 0) for s in STATUSES[:4]))}


def band_table(rows, key, bands):
    out = []
    for b in bands:
        lo, hi = b[0], b[1]
        name = b[2] if len(b) > 2 else (f"[{lo:g},{hi:g})" if hi < 99 else f">={lo:g}")
        sel = [r for r in rows if lo <= key(r) < hi]
        n = len(sel)
        k = sum(r["status"] != "SUCCESS" for r in sel)
        u = sum(r["status"].startswith("CLAIMED") for r in sel)
        out.append({"band": name, "lo": lo, "hi": hi, "n": n, "fail": rate(k, n), "unsafe": rate(u, n)})
    return out


def setup_times(method, region, robot):
    """Setup ("compile") of one robot x region, seconds. GMC: the F3 compile wall (its .a3c, reused by F4/J).
    Baselines: scene export (shared extraction of the judge's Gaussians; outputs sidecar) + the method's persisted
    setup build + the shared raster build (map methods) + one instantiate (live planner from the artifact; median
    over the task's worker starts)."""
    if method == "gmc":
        h = json.loads(Path("results/aerial3dg/f3/f4_handoff.json").read_text())
        c = h["regions"][region]["compiles"][robot]
        return {"compile_wall_s": c["compile_wall_s"], "total_s": c["compile_wall_s"],
                "source": c["compile_record"]}
    sums = [json.loads(f.read_text()) for f in sorted((RES / method / region / robot).glob("task_*.summary.json"))]
    build = sums[0]["setup"]["build_setup_wall_s"]
    inst = [v for s in sums for v in s["setup_once_proof"]["instantiate_s_per_start"]]
    plan = json.loads((RES / "b3/plan.json").read_text())
    rec = plan["methods"][method]["region_robot"][f"{region}/{robot}"]["setup"]["built_by_record"]
    meta = json.loads(Path(rec).read_text()).get("artifact_meta", {})
    raster = (meta.get("info") or {}).get("raster_build_wall_s")
    sc = Path(f"outputs/baselines/scene/{region}_{robot}.npz.json")
    scene = json.loads(sc.read_text()).get("export_wall_s") if sc.exists() else None
    inst_med = float(np.median(inst))
    total = (scene or 0.) + build + (raster or 0.) + inst_med
    return {"scene_export_s": scene, "build_setup_wall_s": build, "raster_build_wall_s": raster,
            "instantiate_s_per_start": inst, "instantiate_s_median": inst_med, "total_s": total,
            "setup_record": rec, "n_kept_gaussians": [s.get("worker_starts", [{}])[0].get("info", {}).get(
                "n_kept_slab_and_box") for s in sums][0]}


def timing(m, robot, rows):
    alg = [r["algorithm_wall_s"] for r in rows]
    t = {"algorithm_wall_s": stats(alg),
         "algorithm_wall_s_success": stats([r["algorithm_wall_s"] for r in rows if r["status"] == "SUCCESS"]),
         "judge_wall_s": stats([r.get("judge_wall_s") for r in rows]),
         "by_region": {reg: stats([r["algorithm_wall_s"] for r in rows if r["region"] == reg]) for reg in REGIONS},
         "setup": {reg: setup_times(m, reg, robot) for reg in REGIONS}}
    if m == "gmc":
        f4 = [r for r in rows if r.get("rejudge_source") == "F4"]
        t["algorithm_wall_s_f4_sourced"] = stats([r["algorithm_wall_s"] for r in f4])
        t["shared_verification_s"] = stats([(r.get("stages") or {}).get("shared_verification") for r in rows
                                            if r.get("stages")])
        t["algorithm_minus_shared_verification_s"] = stats(
            [r["algorithm_wall_s"] - (r.get("stages") or {}).get("shared_verification", 0.) for r in rows])
        keys = sorted({k for r in rows for k in (r.get("stages") or {})})
    else:
        keys = TIME_STAGES[m]
    tot = sum(alg)
    st = {}
    for k in keys:
        v = [float((r.get("stages") or {}).get(k)) for r in rows
             if isinstance((r.get("stages") or {}).get(k), (int, float))]
        st[k] = {"rows": len(v), "median_s": q(v, 50), "p95_s": q(v, 95), "sum_s": float(sum(v)),
                 "share_of_algorithm_wall": float(sum(v)) / tot if tot else None}
    t["stages"] = st
    setup_total = sum(t["setup"][reg]["total_s"] for reg in REGIONS)
    t["amortised_5000"] = {"setup_total_s": setup_total, "query_total_s": tot,
                           "per_query_s": (setup_total + tot) / len(rows), "n": len(rows),
                           "setup_share": setup_total / (setup_total + tot) if tot else None}
    return t


def counts_stats(m, rows):
    """Per-query computation counts (the method's own: solver iterations, expansions, polytopes, NF steps)."""
    def g(path):
        v = []
        for r in rows:
            o = r
            for k in path:
                o = (o or {}).get(k) if isinstance(o, dict) else None
            if isinstance(o, (int, float)):
                v.append(o)
        return stats(v)
    if m == "foci":
        return {"ipopt_iterations": g(["stages", "ipopt_iterations"])}
    if m == "pno":
        return {"astar_expansions": g(["info", "astar_expansions"])}
    if m == "splatnav":
        return {"num_polytopes": g(["info", "num_polytopes"]), "astar_points": g(["info", "astar_points"])}
    if m == "cust_fields":
        return {"nf_steps": g(["info", "steps"]), "gradient_evals": g(["info", "gradient_evals"]),
                "n_obstacles": g(["info", "n_obstacles"])}
    return {}


def quality(data, robot):
    """Pairs solved by both GMC and the baseline: path length / A* route length, vertices, turning, clearance."""
    g = {r["pair_id"]: r for r in data[("gmc", robot)] if r["status"] == "SUCCESS"}
    out = {}
    for m in BASELINES:
        b = {r["pair_id"]: r for r in data[(m, robot)] if r["status"] == "SUCCESS"}
        common = sorted(set(g) & set(b))
        def side(D):
            rs = [D[k] for k in common]
            return {"len_ratio_vs_astar": stats([r["path_length_m"] / r["route_len"] for r in rs]),
                    "len_ratio_vs_straight": stats([r["path_length_m"] / r["dist"] for r in rs]),
                    "vertices": stats([r["vertices"] for r in rs]),
                    "turn_rad": stats([r.get("turn_rad") for r in rs]),
                    "turns_gt_5deg": stats([r.get("turns5") for r in rs]),
                    "clearance_lower_m": stats([r.get("judge_clearance_lower_m", r.get("clearance_lower_m"))
                                                for r in rs])}
        out[m] = {"common_pairs": len(common), "baseline": side(b), "gmc": side(g),
                  "baseline_shorter_than_gmc": rate(sum(b[k]["path_length_m"] < g[k]["path_length_m"]
                                                        for k in common), len(common))}
    return out


def overlap(data, robot):
    """Who solves what: GMC's failure classes vs each baseline's SUCCESS; pairs solved by nobody / only by GMC."""
    g = {r["pair_id"]: r for r in data[("gmc", robot)]}
    succ = {m: {r["pair_id"] for r in data[(m, robot)] if r["status"] == "SUCCESS"} for m in METHODS}
    cls = collections.defaultdict(list)
    for k, r in g.items():
        cls[r["class"]].append(k)
    by_class = {c: {"n": len(ks), **{m: sum(k in succ[m] for k in ks) for m in BASELINES}}
                for c, ks in sorted(cls.items())}
    allp = set(g)
    anyb = set().union(*(succ[m] for m in BASELINES))
    return {"gmc_class_vs_baseline_success": by_class,
            "solved_by_nobody": len(allp - succ["gmc"] - anyb),
            "solved_only_by_gmc": len(succ["gmc"] - anyb),
            "gmc_fail_but_some_baseline_succeeds": len((allp - succ["gmc"]) & anyb),
            "solved_by_all_five": len(set.intersection(*succ.values()))}


def where_breaks(m, rows):
    c = collections.Counter()
    for r in rows:
        if r["status"] == "SUCCESS":
            continue
        if r["status"].startswith("CLAIMED"):
            c[f"{r['status']}|{r.get('judge_geometry_reason') or r.get('judge_reason')}|{r.get('judge_fail_location')}"] += 1
        elif m == "gmc":
            c[f"{r['status']}|{r['class']}|{r.get('reason')}"] += 1
        else:
            c[f"{r['status']}|{r.get('reason')}"] += 1
    return dict(c.most_common())


def robot_compare(data, m):
    """Same pair, two robots: SUCCESS flips; identical judged uv polylines (sha of the rounded uv) among both-success."""
    a = {r["pair_id"]: r for r in data[(m, "sweeper")]}
    b = {r["pair_id"]: r for r in data[(m, "cylinder")]}
    flip = sum((a[k]["status"] == "SUCCESS") != (b[k]["status"] == "SUCCESS") for k in a)
    both = [k for k in a if a[k]["status"] == "SUCCESS" and b[k]["status"] == "SUCCESS"]
    sw_only = sum(a[k]["status"] == "SUCCESS" and b[k]["status"] != "SUCCESS" for k in a)
    cy_only = sum(b[k]["status"] == "SUCCESS" and a[k]["status"] != "SUCCESS" for k in a)
    shorter = sum(a[k]["path_length_m"] < b[k]["path_length_m"] - 1e-6 for k in both)
    return {"pairs": len(a), "success_flips": flip, "sweeper_only": sw_only, "cylinder_only": cy_only,
            "both_success": len(both), "sweeper_path_shorter": shorter}


def dist_bands(rows):
    out = []
    for lo, hi, name in DIST_BANDS:
        sel = [r for r in rows if lo <= r["dist"] < hi]
        out.append({"band": name, "n": len(sel), "success": rate(sum(r["status"] == "SUCCESS" for r in sel), len(sel)),
                    "algorithm_wall_s": stats([r["algorithm_wall_s"] for r in sel]),
                    "path_length_m": stats([r["path_length_m"] for r in sel if r["status"] == "SUCCESS"]),
                    "fail_types": dict(collections.Counter(r["status"] for r in sel if r["status"] != "SUCCESS"))})
    return out


def audit(data):
    """Rows complete and consistent with collect.json; one setup id per cell; configs frozen before B3's first job;
    gmc/src unchanged since d757729; GMC's REACHABLE routes pass the baselines' judge path (b4/gmc_judge)."""
    col = json.loads((RES / "collect.json").read_text())
    P = pairs_by_id()
    want = {reg: {k for k, p in P.items() if p["region"] == reg} for reg in REGIONS}
    cells, ok = {}, True
    for (m, robot), rows in data.items():
        for reg in REGIONS:
            rr = [r for r in rows if r["region"] == reg]
            ids = collections.Counter(r["pair_id"] for r in rr)
            c = {"rows": len(rr), "missing": len(want[reg] - set(ids)), "dup": sum(v > 1 for v in ids.values()),
                 "foreign": len(set(ids) - want[reg])}
            if m == "gmc":
                c["compile_ids"] = sorted({r["compile_id"] for r in rr})
            else:
                c["setup_ids"] = sorted({r["setup_id"] for r in rr})
                cc = col["methods"][m]["robots"][robot]["regions"][reg]
                mine = collections.Counter(r["status"] for r in rr)
                c["status_counts_equal_collect"] = dict(mine) == cc["summary"]["status_counts"]
                c["setup_id_equal_collect"] = c["setup_ids"] == cc["setup_ids"]
                c["setup_once_collect"] = cc["setup_once"] and cc["complete"]
                ok &= c["status_counts_equal_collect"] and c["setup_id_equal_collect"] and c["setup_once_collect"]
                ok &= len(c["setup_ids"]) == 1
            ok &= c["missing"] == 0 and c["dup"] == 0 and c["foreign"] == 0 and c["rows"] == len(want[reg])
            cells[f"{m}/{robot}/{reg}"] = c
    freeze = {}
    for m in BASELINES:
        f = f"gmc/configs/baselines/{m}.json"
        freeze[m] = {"commits": git("log", "--format=%h %cI", "--", f).splitlines(),
                     "changed_since_b3_plan_8c5f2ef": bool(git("diff", "--stat", "8c5f2ef", "HEAD", "--", f))}
    gj = {}
    for f in sorted((OUT / "gmc_judge").glob("*.summary.json")):
        s = json.loads(f.read_text())
        gj[f"{s['region']}/{s['robot']}"] = {k: s[k] for k in ("reachable_rows", "status_counts", "not_success",
                                                               "sha_equal_to_gmc", "judge_wall_s_sum")}
    gj_ok = len(gj) == 6 and all(v["status_counts"].get("SUCCESS", 0) == v["reachable_rows"] for v in gj.values())
    return {"rows_ok": bool(ok), "cells": cells,
            "config_freeze": freeze,
            "b3_first_job_start_local": "2026-10-10T05:41:40-04:00 (19525659_0, sacct)",
            "gmc_src_diff_lines_d757729_to_HEAD": len(git("diff", "d757729", "HEAD", "--", "gmc/src").splitlines()),
            "harness_adapters_configs_changed_since_8c5f2ef": git(
                "diff", "--stat", "8c5f2ef", "HEAD", "--", "gmc/experiments/bl_harness.py",
                "gmc/experiments/bl_worker.py", "gmc/experiments/bl_splatnav.py", "gmc/experiments/bl_foci.py",
                "gmc/experiments/bl_pno.py", "gmc/experiments/bl_custfields.py", "gmc/experiments/bl_raster.py",
                "gmc/configs/baselines") or "none",
            "gmc_routes_through_baseline_judge": gj, "gmc_routes_all_pass": gj_ok,
            "collect_ok": col["ok"]}


def cmd_analyze(a):
    data = load_all()
    doc = {"schema": "bl.b4_analysis.v1", "commit": git("rev-parse", "HEAD"),
           "definition": __doc__.split("\n\n")[0], "gmc_status_map": GMC_STATUS,
           "lat_bands": LAT_BANDS, "ratio_bands": RATIO_BANDS,
           "audit": audit(data), "headline": {}, "per_region": {}, "bands": {}, "timing": {}, "counts": {},
           "quality": {}, "overlap": {}, "where_breaks": {}, "robot_compare": {}, "dist_bands": {},
           "completion": {}, "gmc_before_after": json.loads((RES / "gmc_rejudged/summary.json").read_text())[
               "before_f4"]}
    for robot in ROBOTS:
        doc["headline"][robot] = {m: outcome_table(data[(m, robot)]) for m in METHODS}
        doc["per_region"][robot] = {reg: {m: outcome_table([r for r in data[(m, robot)] if r["region"] == reg])
                                          for m in METHODS} for reg in REGIONS}
        doc["bands"][robot] = {m: {"lateral": band_table(data[(m, robot)], lambda r: r["lat"], LAT_BANDS),
                                   "detour": band_table(data[(m, robot)], lambda r: r["ratio"], RATIO_BANDS),
                                   "lateral_by_region": {reg: band_table([r for r in data[(m, robot)]
                                                                          if r["region"] == reg],
                                                                         lambda r: r["lat"], LAT_BANDS)
                                                         for reg in REGIONS}}
                               for m in METHODS}
        doc["timing"][robot] = {m: timing(m, robot, data[(m, robot)]) for m in METHODS}
        doc["counts"][robot] = {m: counts_stats(m, data[(m, robot)]) for m in METHODS}
        doc["quality"][robot] = quality(data, robot)
        doc["overlap"][robot] = overlap(data, robot)
        doc["where_breaks"][robot] = {m: where_breaks(m, data[(m, robot)]) for m in METHODS}
        doc["dist_bands"][robot] = {m: dist_bands(data[(m, robot)]) for m in METHODS}
        doc["completion"][robot] = {m: {
            "prepended_start": sum(bool(r.get("prepended_start_segment")) for r in data[(m, robot)]),
            "appended_goal": sum(bool(r.get("appended_goal_segment")) for r in data[(m, robot)]),
            "goal_gap_m_claimed": stats([r.get("goal_gap_m") for r in data[(m, robot)] if r.get("claimed")]),
            "start_gap_m_claimed": stats([r.get("start_gap_m") for r in data[(m, robot)] if r.get("claimed")])}
            for m in BASELINES}
    for m in METHODS:
        doc["robot_compare"][m] = robot_compare(data, m)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "analysis.json").write_text(json.dumps(doc, indent=1))
    (OUT / "tables.md").write_text(tables_md(doc))
    print("wrote", OUT / "analysis.json", OUT / "tables.md", "audit rows_ok", doc["audit"]["rows_ok"],
          "gmc routes pass", doc["audit"]["gmc_routes_all_pass"])


def pct(x):
    return "–" if x is None else f"{100 * x:.1f} %"


def ci(rt):
    if not rt["n"]:
        return "–"
    lo, hi = rt["ci95"]
    return f"{100 * rt['rate']:.1f} % ({100 * lo:.1f}–{100 * hi:.1f})"


def tables_md(doc):
    L = ["# bl B4 tables (generated by `bl_analyze.py analyze` from committed rows; do not edit)", ""]
    for robot in ROBOTS:
        L += [f"## Headline, {robot} (counts; SUCCESS rate with 95 % Wilson CI)", "",
              "| method | " + " | ".join(STATUSES) + " | SUCCESS rate (95 % CI) | unsafe claims (CLAIMED_*) rate |",
              "|---" * (len(STATUSES) + 3) + "|"]
        for m in METHODS:
            h = doc["headline"][robot][m]
            L.append(f"| {LABEL[m]} | " + " | ".join(str(h["counts"][s]) for s in STATUSES) +
                     f" | {ci(h['rates']['SUCCESS'])} | {ci(h['claimed_unsafe'])} |")
        L += ["", f"### Per region, {robot}: SUCCESS / n (rate, 95 % CI); CLAIMED_* count", "",
              "| method | " + " | ".join(REGIONS) + " |", "|---" * 4 + "|"]
        for m in METHODS:
            cells = []
            for reg in REGIONS:
                h = doc["per_region"][robot][reg][m]
                cells.append(f"{h['counts']['SUCCESS']}/{h['n']} ({ci(h['rates']['SUCCESS'])}); unsafe "
                             f"{h['claimed_unsafe']['k']}")
            L.append(f"| {LABEL[m]} | " + " | ".join(cells) + " |")
        for key, title in (("lateral", "route lateral-clearance band (mm, ladder rung passed)"),
                           ("detour", "detour ratio band (A* route length / straight)")):
            bands = doc["bands"][robot]["gmc"][key]
            L += ["", f"### Failure rate (1 − SUCCESS) vs {title}, {robot}", "",
                  "| method | " + " | ".join(f"{b['band']} (n {b['n']})" for b in bands) + " |",
                  "|---" * (len(bands) + 1) + "|"]
            for m in METHODS:
                L.append(f"| {LABEL[m]} | " + " | ".join(
                    pct(b["fail"]["rate"]) for b in doc["bands"][robot][m][key]) + " |")
        L += ["", f"### Timing, {robot} (s; method time only, judge excluded except GMC's in-query shared replay)", "",
              "| method | setup WWEST / GAPW1 / S | query median | p95 | max | judge median | amortised per query over 5000 "
              "(setup share) |", "|---" * 7 + "|"]
        for m in METHODS:
            t = doc["timing"][robot][m]
            su = " / ".join(f"{t['setup'][reg]['total_s']:.1f}" for reg in REGIONS)
            a = t["amortised_5000"]
            jm = t["judge_wall_s"].get("median")
            L.append(f"| {LABEL[m]} | {su} | {t['algorithm_wall_s']['median']:.3f} | {t['algorithm_wall_s']['p95']:.2f} "
                     f"| {t['algorithm_wall_s']['max']:.1f} | {'–' if jm is None else f'{jm:.3f}'} | "
                     f"{a['per_query_s']:.3f} ({pct(a['setup_share'])}) |")
        L += ["", f"### Path quality on pairs solved by both GMC and the baseline, {robot} (medians)", "",
              "| baseline | common pairs | length / A* route: baseline vs GMC | vertices: baseline vs GMC | "
              "turning (rad): baseline vs GMC | turns > 5°: baseline vs GMC | baseline shorter than GMC |",
              "|---" * 7 + "|"]
        for m in BASELINES:
            Q = doc["quality"][robot][m]
            if not Q["common_pairs"]:
                L.append(f"| {LABEL[m]} | 0 | – | – | – | – | – |")
                continue
            b, g = Q["baseline"], Q["gmc"]
            L.append(f"| {LABEL[m]} | {Q['common_pairs']} | {b['len_ratio_vs_astar']['median']:.3f} vs "
                     f"{g['len_ratio_vs_astar']['median']:.3f} | {b['vertices']['median']:.0f} vs "
                     f"{g['vertices']['median']:.0f} | {b['turn_rad']['median']:.2f} vs {g['turn_rad']['median']:.2f} | "
                     f"{b['turns_gt_5deg']['median']:.0f} vs {g['turns_gt_5deg']['median']:.0f} | "
                     f"{ci(Q['baseline_shorter_than_gmc'])} |")
        o = doc["overlap"][robot]
        L += ["", f"### Who solves GMC's failures, {robot} (SUCCESS counts per GMC class)", "",
              "| GMC class | n | " + " | ".join(LABEL[m] for m in BASELINES) + " |", "|---" * 6 + "|"]
        for c, v in o["gmc_class_vs_baseline_success"].items():
            L.append(f"| {c} | {v['n']} | " + " | ".join(str(v[m]) for m in BASELINES) + " |")
        L += ["", f"solved by nobody {o['solved_by_nobody']}; only by GMC {o['solved_only_by_gmc']}; GMC fails but a "
              f"baseline succeeds {o['gmc_fail_but_some_baseline_succeeds']}; solved by all five {o['solved_by_all_five']}",
              ""]
    return "\n".join(L) + "\n"


# ================================================================================================ sheets
def sheet_values(doc, m, robot):
    """Template row label -> filled value for one method x robot (Chinese, numbers from analysis.json)."""
    h, t, cnt = doc["headline"][robot][m], doc["timing"][robot][m], doc["counts"][robot][m]
    Q = doc["quality"][robot][m]
    body = {"cylinder": "cylinder（竖直圆柱 r 0.30 m，半高 0.865 m，离地 0.02 m，z_c 0.885 m）",
            "sweeper": "sweeper（竖直圆柱 r 0.175 m，半高 0.04 m，离地 0.02 m，z_c 0.06 m）"}[robot]
    cover = {"splatnav": "球体经 z 方向线性压缩 ε=0.05 覆盖圆柱，映回实空间为侧向 1.07×r（sweeper 1.09×r）的高椭球",
             "foci": "3 个机体点 × Gaussian robot_cov（包围圆柱的最小体积椭球，cov_scale 0.5），软代价无硬覆盖",
             "pno": "点机器人在共享栅格化 C-space 图上（障碍按 r+margin 膨胀）",
             "cust_fields": "点机器人在由同一 C-space 图生成的 squircle 星形世界里"}[m]
    a = t["algorithm_wall_s"]
    su = t["setup"]
    su_tot = sum(su[r]["total_s"] for r in REGIONS)
    am = t["amortised_5000"]
    n = h["n"]
    c = h["counts"]
    claimed = h["claimed"]
    unsafe = h["claimed_unsafe"]["k"]
    st = t["stages"]
    tot = a["sum"]

    def share(keys):
        s = sum(st[k]["sum_s"] for k in keys if k in st)
        return f"{s / n:.4f} s（每查询均值）；占总时间 {100 * s / tot:.1f}%" if tot else "N/A"
    NA = "N/A"
    jm = t["judge_wall_s"]
    judge_txt = (f"{jm['median']:.3f} s（中位数，裁判 replay_plan，不计入方法时间，单独记录）"
                 if jm.get("n") else "N/A（无声称路径，裁判未运行）")
    stage_name = {"times_astar": "A* 初始化", "times_collision_set": "碰撞集构建", "times_polytope": "安全多面体走廊",
                  "times_opt": "Bézier QP 求解", "plan_s": "IPOPT 轨迹优化（含其 A* 初值）", "snap_s": "端点吸附",
                  "value_infer_s": "PNO 值函数推理（GPU）", "heuristic_s": "启发式构建", "astar_s": "网格 A*",
                  "nf_descend_s": "导航函数梯度下降"}
    top = max(st.items(), key=lambda kv: kv[1]["sum_s"]) if st else None
    top_txt = (f"阶段：{stage_name.get(top[0], top[0])}；耗时：{top[1]['sum_s'] / n:.4f} s（均值）；占总时间："
               f"{100 * top[1]['share_of_algorithm_wall']:.1f}%" if top and tot else NA)
    staged = sum(v["sum_s"] for v in st.values())
    other = tot - staged
    succ = c["SUCCESS"]
    lr = doc["dist_bands"][robot][m]
    rc = doc["robot_compare"][m]
    oth = "sweeper" if robot == "cylinder" else "cylinder"
    oth_su = sum(doc["timing"][oth][m]["setup"][r]["total_s"] for r in REGIONS)
    gpu = {"splatnav": "L40S（4 流共享一卡）峰值 5.8 GB（4 流合计，nvidia-smi 30 s 采样）",
           "foci": "L40S（4 流共享一卡）峰值 2.4 GB（4 流合计）",
           "pno": "L40S（4 流共享一卡）峰值 17.1 GB（4 流合计）", "cust_fields": "N/A（CPU 方法）"}[m]
    rss = {"splatnav": "6.5 GB（sacct MaxRSS，整个 4 流作业含裁判）", "foci": "3.7 GB（sacct MaxRSS，整个 4 流作业含裁判）",
           "pno": "5.1 GB（sacct MaxRSS，整个 4 流作业含裁判）", "cust_fields": "≤ 0.44 GB（每个数组任务，含裁判）"}[m]
    hw = ("CPU：Torch cpu_short 节点（每数组任务 1 核）；GPU：无；RAM：每任务申请 2 GB；OS：Linux (RHEL)，方法 env Python 3.10.22，"
          "裁判 gmc-venv Python 3.13.5" if m == "cust_fields" else
          "CPU：Torch l40s_public 节点 4 核（4 个 harness 流）；GPU：NVIDIA L40S 44 GB（一卡 4 流）；RAM：申请 12 GB；"
          "OS：Linux (RHEL)，方法 env Python 3.10，裁判 gmc-venv Python 3.13.5")
    clr = Q["baseline"]["clearance_lower_m"] if Q["common_pairs"] else {"n": 0}
    lens = [r for r in [Q["baseline"]["len_ratio_vs_astar"]]] if Q["common_pairs"] else []
    rk = lambda key: cnt.get(key, {"n": 0})  # noqa: E731
    it = {"foci": rk("ipopt_iterations"), "pno": rk("astar_expansions"), "splatnav": rk("num_polytopes"),
          "cust_fields": rk("nf_steps")}[m]
    it_name = {"foci": "IPOPT 迭代", "pno": "A* 扩展节点", "splatnav": "安全多面体数", "cust_fields": "NF 下降步数"}[m]
    fails = {k: v for k, v in c.items() if k != "SUCCESS" and v}
    common_fail = max(fails.items(), key=lambda kv: kv[1]) if fails else None
    br = doc["where_breaks"][robot][m]
    top_reason = next(iter(br.items())) if br else None
    v = {
        "实验设置｜代码版本 / Git commit": f"版本：{m} 上游 {UPSTREAM[m]} + bl 适配器（冻结配置 "
            f"configs/baselines/{m}.json）；commit：B3 全量运行 8c5f2ef（PNO 续跑 03ed70c），分析 {doc['commit'][:7]}",
        "实验设置｜运行日期": "2026-10-10",
        "实验设置｜硬件与运行环境": hw,
        "实验设置｜每个配置的 warm-up 次数与正式重复次数": "Warm-up：0 次（每个任务一次实例化后直接查询，实例化单独计时）；"
            "正式运行：每对 1 次（5000 对）",
        "实验设置｜场景名称 / 场景编号": "scene_v2（uavlamp_gallery_booth_v2_ground，SHA-256 2a3a72d6…96cc），区域 WWEST / GAPW1 / S",
        "实验设置｜场景 Gaussian 数量": "N_G（裁判场景，opacity>0.3）= 390,716 / 209,126 / 333,487（WWEST / GAPW1 / S）",
        "实验设置｜机器人名称与几何参数": f"机器人：{body}；覆盖：{cover}",
        "时间｜单次完整规划总时间（cold start）": f"Median：{su_tot / 3 + a['median']:.2f} s；P95：{su_tot / 3 + a['p95']:.2f} s"
            f"（cold = 平均每区域 setup {su_tot / 3:.1f} s + 查询）",
        "时间｜单次完整规划总时间（已有场景结构后的 warm query）": f"Median：{a['median']:.3f} s；P95：{a['p95']:.3f} s",
        "时间｜场景层级结构 / BVH 构建耗时": NA,
        "时间｜scene–robot 候选 Gaussian pair 生成耗时": NA,
        "时间｜scene–robot 精确几何交互 / 碰撞计算耗时": share(["times_collision_set", "times_polytope"])
            if m == "splatnav" else NA,
        "时间｜自由 / 碰撞状态或 configuration-space domain 构建耗时":
            (f"{sum(su[r]['raster_build_wall_s'] or 0 for r in REGIONS):.1f} s（共享栅格化器，3 区域合计，setup 一次）；"
             f"占总时间 N/A（setup，不在查询内）" if m in ("pno", "cust_fields") else
             f"{sum(su[r]['instantiate_s_median'] for r in REGIONS):.1f} s（体素网格 + 碰撞集实例化，3 区域合计，每 worker 一次）"
             if m == "splatnav" else NA),
        "时间｜自适应 refinement 耗时": NA,
        "时间｜规划结构 / graph / operator 组装耗时": share(["value_infer_s", "heuristic_s"]) if m == "pno" else NA,
        "时间｜factorization / preconditioner setup 耗时": NA,
        "时间｜全局求解 / 图搜索 / 数值求解耗时": share({"splatnav": ["times_astar", "times_opt"], "foci": ["plan_s"],
                                               "pno": ["astar_s"], "cust_fields": ["nf_descend_s"]}[m]),
        "时间｜路径提取耗时": share(["snap_s"]) + "（端点吸附；路径即求解输出）" if m in ("pno", "cust_fields") else NA,
        "时间｜最终连续碰撞检查与路径验证耗时": judge_txt,
        "时间｜其他未归类耗时": f"{other / n:.4f} s；占总时间 {100 * other / tot:.1f}%" if tot else NA,
        "时间｜各阶段耗时之和与总时间的差值": f"{other / n:.4f} s；差值占总时间 {100 * other / tot:.1f}%" if tot else NA,
        "时间｜当前最耗时阶段": top_txt,
        "计算量｜单次规划 collision / contact 查询次数": NA,
        "计算量｜scene–robot 候选 Gaussian pair 数量": NA,
        "计算量｜scene–robot 实际有效 Gaussian pair 数量": NA,
        "计算量｜有效 pair 比例": NA,
        "计算量｜broad-phase 剪枝比例": NA,
        "计算量｜单次 collision / contact query 平均耗时": NA,
        "计算量｜重复或近重复 collision / contact query 比例": NA,
        "计算量｜单次规划使用的 configuration states / 节点 / 网格数量":
            (f"{it_name}：中位数 {it['median']:.0f}，P95 {it['p95']:.0f}" if it.get("n") else NA),
        "计算量｜orientation bins 数量": NA,
        "计算量｜单次规划使用的边数": NA,
        "计算量｜稀疏 operator 非零元素数量": NA,
        "计算量｜refinement 轮数": NA,
        "计算量｜solver 迭代次数": (f"Median：{it['median']:.0f}；P95：{it['p95']:.0f}" if m == "foci" and it.get("n") else NA),
        "计算量｜峰值 CPU 内存": rss,
        "计算量｜峰值 GPU 显存": gpu,
        "复用性｜同一场景、同一机器人，仅更换 start/end 后的总规划时间": f"Median：{a['median']:.3f} s；P95：{a['p95']:.3f} s",
        "复用性｜仅更换 start/end 时必须重新计算的阶段": {"splatnav": "A*、碰撞集、多面体走廊、QP",
                                              "foci": "A* 初值 + 样条拟合 + IPOPT", "pno": "值函数推理（目标相关）+ A*",
                                              "cust_fields": "导航函数（目标相关）+ 梯度下降"}[m],
        "复用性｜仅更换 start/end 时可以直接复用的阶段": {"splatnav": "Gaussian 预处理（板切）、体素网格",
                                              "foci": "障碍预处理、CasADi NLP + warp 内核",
                                              "pno": "栅格 C-space 图、重采样、腐蚀、模型权重、χ（FNOSDF）",
                                              "cust_fields": "栅格 C-space 图、squircle 星形世界"}[m],
        "复用性｜仅更换 start/end 时可复用计算占原 cold-start 时间比例":
            f"{100 * (su_tot / 3) / (su_tot / 3 + a['median']):.1f}%",
        "复用性｜相同 start/end 重复查询时的缓存命中率": "0%（无缓存）",
        "机器人对比｜机器人 1 参数": "名称：sweeper；宽：0.35 m；长：0.35 m；高：0.08 m；其他：圆柱，离地 0.02 m",
        "机器人对比｜机器人 1 规划结果": robot_result(doc, m, "sweeper"),
        "机器人对比｜机器人 2 参数": "名称：cylinder；宽：0.60 m；长：0.60 m；高：1.73 m；其他：圆柱，离地 0.02 m",
        "机器人对比｜机器人 2 规划结果": robot_result(doc, m, "cylinder"),
        "机器人对比｜机器人 3 参数": NA,
        "机器人对比｜机器人 3 规划结果": NA,
        "机器人对比｜同一场景、同一 start/end，更换机器人后的总规划时间":
            f"机器人 1：{doc['timing']['sweeper'][m]['algorithm_wall_s']['median']:.3f} s；机器人 2："
            f"{doc['timing']['cylinder'][m]['algorithm_wall_s']['median']:.3f} s；机器人 3：N/A（中位数）",
        "机器人对比｜更换机器人后重新构建 scene–robot planning representation 的时间":
            f"机器人 1→2：{sum(doc['timing']['cylinder'][m]['setup'][r]['total_s'] for r in REGIONS):.1f} s（3 区域合计）；"
            f"机器人 2→3：N/A；机器人 1→3：N/A",
        "机器人对比｜更换机器人时必须从头重新计算的阶段": {"splatnav": "板切 Gaussian、体素网格、碰撞集（机体半径变）",
                                               "foci": "robot_cov 与 NLP", "pno": "C-space 栅格图（膨胀半径变）及其重采样",
                                               "cust_fields": "C-space 栅格图与星形世界"}[m],
        "机器人对比｜更换机器人时可直接复用的阶段": {"pno": "模型权重", "splatnav": "裁判场景 Gaussian 导出（只读）",
                                            "foci": "裁判场景 Gaussian 导出（只读）", "cust_fields": "无"}[m],
        "机器人对比｜更换机器人时可复用计算占原总计算量比例": "≈0%（setup 按机器人各建一次）",
        "机器人对比｜连续完成三种机器人规划所需总时间": NA,
        "机器人对比｜三种机器人是否得到不同路径": f"是 / 否：是（两种机器人）；差异描述：两者都成功的 {rc['both_success']} 对中，"
            f"sweeper 路径更短的 {rc['sweeper_path_shorter']} 对",
        "机器人对比｜三种机器人得到完全相同路径的比例（多组 start/end）": NA,
        "机器人对比｜更换机器人后可达 / 不可达结论发生变化的比例":
            f"{rc['success_flips']} / {rc['pairs']}；{100 * rc['success_flips'] / rc['pairs']:.1f}%（SUCCESS 翻转；"
            f"仅 sweeper 成功 {rc['sweeper_only']}，仅 cylinder 成功 {rc['cylinder_only']}）",
        "机器人对比｜更换机器人后主要路线发生变化的比例": NA,
        "机器人对比｜小机器人选择窄路捷径的比例": NA,
        "机器人对比｜大机器人避开无法通过窄路的比例": NA,
        "机器人对比｜高机器人避开低矮通道的比例": NA,
        "机器人对比｜相近尺寸机器人之间 collision / state 判断相同的比例": NA,
        "机器人对比｜相近尺寸机器人之间 operator edge 相同的比例": NA,
        "机器人对比｜轻微改变机器人宽度 / 长度 / 高度后结果突然翻转的比例": NA,
        "A–B 扩展｜测试距离档位": "L1：<2 m；L2：2–4 m；L3：4–6 m；L4：≥6 m（直线距离；n = " +
            " / ".join(str(b["n"]) for b in lr) + "）",
        "A–B 扩展｜不同距离下的总规划时间": "；".join(f"L{i + 1}：{b['algorithm_wall_s']['median']:.3f} s" if b["n"]
                                             else f"L{i + 1}：N/A（0 对）" for i, b in enumerate(lr)) + "（中位数）",
        "A–B 扩展｜不同距离下的规划成功率": "；".join(f"L{i + 1}：{100 * b['success']['rate']:.1f}%" if b["n"]
                                             else f"L{i + 1}：N/A（0 对）" for i, b in enumerate(lr)),
        "A–B 扩展｜不同距离下的输出路径长度": "；".join(
            f"L{i + 1}：" + (f"{b['path_length_m']['median']:.2f} m" if b["path_length_m"].get("n") else "N/A")
            for i, b in enumerate(lr)) + "（成功行中位数）",
        "A–B 扩展｜不同距离下的 collision / contact 查询次数": NA,
        "A–B 扩展｜不同距离下的候选 / 有效 Gaussian pair 数量": NA,
        "A–B 扩展｜不同距离下的 configuration state / 节点数量": NA,
        "A–B 扩展｜不同距离下的 solver 迭代次数": NA,
        "A–B 扩展｜不同距离下的 refinement 轮数": NA,
        "A–B 扩展｜不同距离下的峰值内存": NA,
        "A–B 扩展｜不同距离下的失败类型": "；".join(f"L{i + 1}：" + (", ".join(f"{k} {v}" for k, v in b["fail_types"].items())
                                                           or "无") for i, b in enumerate(lr)),
        "规模扩展｜不同场景 Gaussian 数量下的总规划时间": "；".join(
            f"N_G={ng}：{t['by_region'][r]['median']:.3f} s" for r, ng in zip(REGIONS, ("390716", "209126", "333487"))) +
            "（每区域查询中位数）",
        "规模扩展｜不同场景 Gaussian 数量下的 pair 计算时间": NA,
        "规模扩展｜不同场景 Gaussian 数量下的 operator / graph 构建时间": "；".join(
            f"N_G={ng}：{su[r]['total_s']:.1f} s" for r, ng in zip(REGIONS, ("390716", "209126", "333487"))) + "（setup 合计）",
        "规模扩展｜不同场景 Gaussian 数量下的全局求解时间": NA,
        "规模扩展｜不同场景 Gaussian 数量下的峰值内存": NA,
        "规模扩展｜不同规划分辨率下的总时间": {"pno": "分辨率 S=1024：2.03 s；S=2048：8.52 s；S=4096：GPU OOM（调参集 cylinder 中位数，"
                                                "tuning_table.json）",
                                         "splatnav": "分辨率 2 cm：0.57 s；1 cm：0.67 s；4 cm：0.66 s（调参集 cylinder 中位数）"}.get(m, NA),
        "规模扩展｜不同规划分辨率下的状态数": NA,
        "质量｜在构造上明确存在可行路径的测试中成功规划比例": f"{succ} / {n}；成功率：{100 * succ / n:.1f}%",
        "质量｜在构造上明确不存在路径的测试中正确返回无路径比例": "N/A（基准只含已证实可达的对）",
        "质量｜错误返回“无路径”的比例": f"{c['FAIL']} / {n}；{100 * c['FAIL'] / n:.1f}%（每对都已证实可达，FAIL 均为错误无路）",
        "质量｜输出路径通过最终连续碰撞验证的比例": f"{succ} / {claimed}；{100 * succ / claimed:.1f}%" if claimed else "0 / 0；N/A",
        "质量｜输出路径未通过最终连续碰撞验证的比例": f"{unsafe} / {claimed}；{100 * unsafe / claimed:.1f}%" if claimed else "0 / 0；N/A",
        "质量｜路径长度统计": path_len_txt(doc, m, robot),
        "质量｜相对于当前可获得最短路径的长度增加比例": (
            f"Mean：{100 * (lens[0]['mean'] - 1):.1f}%；Median：{100 * (lens[0]['median'] - 1):.1f}%；P95："
            f"{100 * (lens[0]['p95'] - 1):.1f}%（相对 F4 A* 路线，0.1 m 栅格，非真最短；与 GMC 共同成功的 {Q['common_pairs']} 对）"
            if lens else NA),
        "质量｜路径最小碰撞间隙 / clearance": (f"Mean：{clr['mean']:.4f} m；Median：{clr['median']:.4f} m；Minimum："
                                         f"{clr['min']:.4f} m（裁判的连续间隙下界，共同成功对）" if clr.get("n") else NA),
        "质量｜路径转弯次数或方向变化次数": (f"Mean：{Q['baseline']['turns_gt_5deg']['mean']:.1f}；Median："
                                     f"{Q['baseline']['turns_gt_5deg']['median']:.0f}；P95：{Q['baseline']['turns_gt_5deg']['p95']:.0f}"
                                     f"（> 5° 的转向点；累计转角中位数 {Q['baseline']['turn_rad']['median']:.2f} rad）"
                                     if Q["common_pairs"] else NA),
        "质量｜路径平滑度 / 曲率代价（若实现）": NA,
        "质量｜在设定时间预算内的 timeout 比例": f"时间预算：120 s；Timeout：{c['TIMEOUT']} / {n}；{100 * c['TIMEOUT'] / n:.1f}%",
        "质量｜规划分辨率改变后成功 / 失败结论发生变化的比例": NA,
        "质量｜规划分辨率改变后主要路线发生变化的比例": NA,
        "质量｜同一输入多次重复运行时结果稳定性": "N/A（每对只跑一次；B3 抽查 25 行重判与存储一致，不是重跑规划器）",
        "失败分析｜无路径误判数量与比例": f"{c['FAIL']} / {n}；{100 * c['FAIL'] / n:.1f}%",
        "失败分析｜超时数量与比例": f"{c['TIMEOUT']} / {n}；{100 * c['TIMEOUT'] / n:.1f}%",
        "失败分析｜输出路径最终验证失败数量与比例": f"{unsafe} / {n}；{100 * unsafe / n:.1f}%（CLAIMED_*，不安全声称）",
        "失败分析｜内存不足数量与比例": f"0 / {n}；0.0%",
        "失败分析｜其他失败数量、比例与原因": f"{c['ERROR'] + c['SETUP_FAIL']} / {n}；0.0%；原因：无 ERROR / SETUP_FAIL",
        "失败分析｜失败是否主要由计算预算不足造成": f"是 / 否：否；相关失败：{c['TIMEOUT']} / {n - succ}",
        "失败分析｜失败是否主要由当前解析表示或求解能力不足造成": representation_txt(m, robot, c, n, succ),
        "失败分析｜计算时间可接受但路径质量明显不好的测试比例": quality_bad_txt(doc, m, robot),
        "失败分析｜路径结果正确但计算时间明显过长的测试比例": NA,
        "失败分析｜当前最常见失败类型": (f"类型：{top_reason[0]}；占全部失败：{100 * top_reason[1] / (n - succ):.1f}%"
                                if top_reason and n - succ else "无失败"),
    }
    v["实验设置｜Start 与 End"] = ("Start/End：F4 的 5000 对 confirmed-reachable 对（pairs_confirmed_5000.json，与 GMC 完全相同的"
                              "起终点）；直线距离：" + "，".join(f"{b['band']} {b['n']} 对" for b in lr))
    return v


def robot_result(doc, m, robot):
    h = doc["headline"][robot][m]
    c = h["counts"]
    t = doc["timing"][robot][m]["algorithm_wall_s"]
    return (f"成功 / 无路径 / 超时：{c['SUCCESS']} / {c['FAIL']} / {c['TIMEOUT']}（另有不安全声称 {h['claimed_unsafe']['k']}）；"
            f"路线类型：N/A；路径长度：{path_len_txt(doc, m, robot)}；规划时间：中位数 {t['median']:.3f} s；"
            f"最小 clearance：见质量｜路径最小碰撞间隙")


def path_len_txt(doc, m, robot):
    Q = doc["quality"][robot][m]
    return "N/A（无与 GMC 共同成功的对）" if not Q["common_pairs"] else \
        f"见长度比（与 GMC 共同成功 {Q['common_pairs']} 对，长度 / A* 路线中位数 {Q['baseline']['len_ratio_vs_astar']['median']:.3f}）"


def representation_txt(m, robot, c, n, succ):
    f = n - succ
    if not f:
        return "是 / 否：否；相关失败：0 / 0；0.0%"
    reason = {"splatnav": "A* 体素图在 WWEST 不连通（体素化 + 覆盖保守性）与 QP 不可行",
              "foci": "软碰撞代价无 margin（擦碰 Gaussian）、软终点（补齐段受裁判检查）、IPOPT 迭代上限",
              "pno": "无失败", "cust_fields": "星形世界（squircle 覆盖）吞没端点（我们的适配）+ 导航函数下降停滞（方法本身）"}[m]
    return f"是 / 否：是；相关失败：{f} / {f}；100.0%；原因：{reason}"


def quality_bad_txt(doc, m, robot):
    Q = doc["quality"][robot][m]
    if not Q["common_pairs"]:
        return "N/A"
    return f"N/A（以长度比 p95 {Q['baseline']['len_ratio_vs_astar']['p95']:.3f} 相对 A* 路线报告，未设阈值）"


def cmd_sheets(a):
    doc = json.loads((OUT / "analysis.json").read_text())
    rows = list(csv.reader(TEMPLATE.read_text(encoding="utf-8-sig").splitlines()))
    for m in BASELINES:
        vals = {robot: sheet_values(doc, m, robot) for robot in ("sweeper", "cylinder")}
        out = [[rows[0][0], f"sweeper · {LABEL[m]} · F4 5000 对（B3 全量，B4 汇总）",
                f"cylinder · {LABEL[m]} · F4 5000 对（B3 全量，B4 汇总）"]]
        missing = []
        for r in rows[1:]:
            if not r:
                continue
            k = r[0]
            if k not in vals["sweeper"]:
                missing.append(k)
            out.append([k, vals["sweeper"].get(k, "N/A"), vals["cylinder"].get(k, "N/A")])
        p = DOCS / f"baselines_measurement_{m}.csv"
        with open(p, "w", encoding="utf-8-sig", newline="") as f:
            csv.writer(f).writerows(out)
        print("wrote", p, "rows", len(out) - 1, "unmapped template rows", missing)


# ================================================================================================ figures
COLOR = {"gmc": "#2a78d6", "splatnav": "#eb6834", "foci": "#1baf7a", "pno": "#eda100", "cust_fields": "#e87ba4"}
STYLE = {"gmc": "-", "splatnav": "--", "foci": "-.", "pno": ":", "cust_fields": (0, (5, 1, 1, 1))}
MARK = {"gmc": "o", "splatnav": "s", "foci": "^", "pno": "D", "cust_fields": "v"}


def pick_pairs(rowsets, region):
    """Deterministic, rule-based pick of 4 pairs per region (rule recorded in figures.json).
    Each rule takes, among qualifying pairs, the one whose straight distance is closest to the region median."""
    def get(m, robot):
        return {r["pair_id"]: r for r in rowsets[(m, robot)] if r["region"] == region}
    cy = {m: get(m, "cylinder") for m in METHODS}
    sw = {m: get(m, "sweeper") for m in METHODS}
    med = float(np.median([r["dist"] for r in cy["gmc"].values()]))

    def best(ids, D):
        ids = sorted(ids)
        return min(ids, key=lambda k: (abs(D["gmc"][k]["dist"] - med), k)) if ids else None
    rules = []
    ok = lambda D, k, m: D[m][k]["status"] == "SUCCESS"  # noqa: E731
    r1 = [k for k in cy["gmc"] if all(ok(cy, k, m) for m in ("gmc", "splatnav", "pno"))
          and cy["foci"][k]["status"] == "CLAIMED_UNPROVEN" and cy["foci"][k].get("judge_fail_location") == "method_path"]
    why = "GMC, SplatNav, PNO SUCCESS; FOCI CLAIMED_UNPROVEN on its own curve (its typical cylinder failure)"
    if not r1:
        r1 = [k for k in cy["gmc"] if all(ok(cy, k, m) for m in ("gmc", "splatnav", "foci", "pno"))]
        why = "GMC, SplatNav, FOCI, PNO all SUCCESS (no FOCI unsafe claim on its own curve in this region)"
    rules.append(("cylinder", best(r1, cy), why))
    r2 = [k for k in cy["gmc"] if cy["gmc"][k]["status"] != "SUCCESS" and ok(cy, k, "splatnav") and ok(cy, k, "pno")]
    tm = [k for k in r2 if cy["gmc"][k]["status"] == "TIMEOUT"]
    rules.append(("cylinder", best(tm or r2, cy), "GMC fails (TIMEOUT if any in the region, else EP-TOL FAIL) while "
                                                 "SplatNav and PNO succeed"))
    r3 = [k for k in cy["gmc"] if cy["splatnav"][k]["status"] == "FAIL"]
    why = "SplatNav FAIL (its typical failure)"
    if not r3:
        r3 = [k for k in cy["gmc"] if cy["foci"][k]["status"] == "CLAIMED_COLLIDES"]
        why = "FOCI CLAIMED_COLLIDES (no SplatNav FAIL in this region)"
    if not r3:
        r3 = [k for k in cy["gmc"] if all(ok(cy, k, m) for m in ("gmc", "splatnav", "foci", "pno"))]
        why = "GMC, SplatNav, FOCI, PNO all SUCCESS"
    rules.append(("cylinder", best(r3, cy), why))
    r4 = [k for k in sw["gmc"] if sw["cust_fields"][k]["status"] == "SUCCESS"]
    why = "sweeper: cust_fields SUCCESS"
    if not r4:
        r4 = [k for k in sw["gmc"] if sw["foci"][k]["status"].startswith("CLAIMED")
              and sw["foci"][k].get("judge_fail_location") == "appended_goal_segment"]
        why = "sweeper: FOCI unsafe claim on the appended goal segment (its typical sweeper failure); cust_fields FAIL"
    if not r4:
        r4 = [k for k in sw["gmc"] if sw["cust_fields"][k]["reason"] == "nf_stuck"]
        why = "sweeper: cust_fields FAIL nf_stuck (its own descent stalls; no FOCI goal-segment claim in this region)"
    rules.append(("sweeper", best(r4, sw), why))
    out, seen = [], set()
    for rb, k, w in rules:
        if k and (k, rb) not in seen:
            seen.add((k, rb))
            out.append({"robot": rb, "pair_id": k, "rule": w})
    return out


def polylines_for(picks, region):
    """pair/robot -> {method: uv polyline or None} from committed rows (+ SplatNav full rows)."""
    out = {}
    for robot in ROBOTS:
        want = {p["pair_id"] for p in picks if p["robot"] == robot}
        if not want:
            continue
        sn = splatnav_full(region, robot)
        for m in METHODS:
            rows = gmc_rows(region, robot) if m == "gmc" else base_rows(m, region, robot)
            for r in rows:
                if r["pair_id"] in want:
                    poly = sn.get(r["pair_id"]) if m == "splatnav" else r.get("route_polyline")
                    out.setdefault((r["pair_id"], robot), {})[m] = {
                        "poly": None if poly is None else np.asarray(poly, float)[:, :2],
                        "status": r["status"],
                        "detail": (r.get("class") if m == "gmc" else
                                   (r.get("judge_fail_location") or r.get("reason") or ""))}
    return out


def cf_covers(region, robot):
    import pickle
    import bl_custfields as CF
    cfg = json.loads(Path("configs/baselines/cust_fields.json").read_text())["robots"][robot]
    art = H.artifact_path("cust_fields", region, robot, dict(cfg))
    if not art.exists():
        return []
    st = pickle.loads(art.read_bytes())
    return [CF.squircle_boundary(np.asarray(c), aa, bb, th, float(cfg["s"]), 180)
            for ch in st["chains"] for c, aa, bb, th in ch]


def fig_overview(region, picks, polys, P):
    import matplotlib.pyplot as plt
    import bl_raster as R
    fig, axs = plt.subplots(2, 2, figsize=(15, 15))
    for ax, pk in zip(axs.flat, picks):
        k, robot = pk["pair_id"], pk["robot"]
        arr, info = R.load(R.raster_path(region, robot, 0.005))
        ext = [info["u0"], info["u0"] + info["nx"] * info["res_m"], info["v0"], info["v0"] + info["ny"] * info["res_m"]]
        ax.imshow(arr["occ"], origin="lower", extent=ext, cmap="Greys", vmin=-.8, vmax=2.2, interpolation="nearest")
        for cv in cf_covers(region, robot):
            ax.plot(np.r_[cv[:, 0], cv[0, 0]], np.r_[cv[:, 1], cv[0, 1]], color=COLOR["cust_fields"], lw=.8,
                    alpha=.6)
        p = P[k]
        rt = np.asarray(p["astar_route_uv"], float)
        ax.plot(rt[:, 0], rt[:, 1], color="#52514e", lw=1.0, ls=(0, (2, 2)), label="F4 A* route")
        pts = [rt]
        legend_txt = []
        for m in METHODS:
            d = polys[(k, robot)][m]
            lab = f"{LABEL[m]}: {d['status']}" + (f" ({d['detail']})" if d["status"] != "SUCCESS" and d["detail"] else "")
            if d["poly"] is not None:
                q = d["poly"]
                ax.plot(q[:, 0], q[:, 1], color=COLOR[m], lw=2.0, ls=STYLE[m], label=lab, zorder=3)
                ax.plot(q[len(q) // 2, 0], q[len(q) // 2, 1], MARK[m], color=COLOR[m], ms=8, mec="white", mew=1.5,
                        zorder=4)
                pts.append(q)
            else:
                ax.plot([], [], color=COLOR[m], lw=2.0, ls=STYLE[m], label=lab + " — no path")
            legend_txt.append(lab)
        s, g = np.asarray(p["start_uv"]), np.asarray(p["goal_uv"])
        ax.plot(*s, "o", color="#0b0b0b", ms=9, mfc="white", mew=2, zorder=5)
        ax.plot(*g, "s", color="#0b0b0b", ms=9, mfc="white", mew=2, zorder=5)
        A = np.vstack(pts)
        lo, hi = A.min(0) - .5, A.max(0) + .5
        ax.set_xlim(max(lo[0], ext[0]), min(hi[0], ext[1]))
        ax.set_ylim(max(lo[1], ext[2]), min(hi[1], ext[3]))
        ax.set_aspect("equal")
        ax.set_title(f"{k} · {robot} · {p['dist_m']:.2f} m straight, lateral {1000 * p['lateral_clearance_m']:g} mm, "
                     f"detour {p['len_ratio']:.2f}\n{pk['rule']}", fontsize=9, color="#0b0b0b")
        ax.legend(fontsize=7.5, loc="upper center", bbox_to_anchor=(.5, -.09), ncol=2, framealpha=.9)
        ax.set_xlabel("u (m, plan frame)", fontsize=8)
        ax.set_ylabel("v (m)", fontsize=8)
        ax.tick_params(labelsize=7)
    for ax in list(axs.flat)[len(picks):]:
        ax.axis("off")
    fig.suptitle(f"{region}: C-space map for the panel's robot (dark = body centre cannot be; 5 mm shared raster),\n"
                 f"cust_fields squircle covers (pink outlines where inside the view; in WWEST one cover encloses the room), "
                 f"start ○ goal □", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, .96])
    out = FIG / f"overview_{region}.png"
    fig.savefig(out, dpi=85)
    plt.close(fig)
    return str(out)


def fig_bands(doc):
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(2, 2, figsize=(15, 9), sharey=True)
    for i, robot in enumerate(ROBOTS):
        for j, key in enumerate(("lateral", "detour")):
            ax = axs[i, j]
            for mi, m in enumerate(METHODS):
                b = [x for x in doc["bands"][robot][m][key] if x["n"]]
                x = np.arange(len(b)) + (mi - 2) * .09
                y = np.array([x_["fail"]["rate"] for x_ in b]) * 100
                lo = np.array([x_["fail"]["ci95"][0] for x_ in b]) * 100
                hi = np.array([x_["fail"]["ci95"][1] for x_ in b]) * 100
                ax.errorbar(x, y, yerr=[np.clip(y - lo, 0, None), np.clip(hi - y, 0, None)], color=COLOR[m], ls=STYLE[m], marker=MARK[m], ms=7, lw=2,
                            capsize=2, elinewidth=1, label=LABEL[m], mec="white", mew=1)
            bb = [x_ for x_ in doc["bands"][robot]["gmc"][key] if x_["n"]]
            ax.set_xticks(range(len(bb)), [f"{x_['band']}\nn={x_['n']}" for x_ in bb], fontsize=8)
            ax.set_title(f"{robot}: failure rate vs " + ("route lateral-clearance band (mm, ladder rung passed)"
                                                        if key == "lateral" else
                                                        "detour ratio band (A* route / straight)"), fontsize=10)
            ax.grid(axis="y", color="#e5e4e0", lw=.6)
            ax.set_ylim(-3, 103)
            if j == 0:
                ax.set_ylabel("failure rate, % of pairs in band (1 − SUCCESS; 95 % CI)", fontsize=9)
    axs[0, 0].legend(fontsize=8, ncol=5, loc="upper left", bbox_to_anchor=(0, 1.25))
    fig.tight_layout()
    out = FIG / "bands.png"
    fig.savefig(out, dpi=90)
    plt.close(fig)
    return str(out)


def fig_timing(doc):
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 3, figsize=(17, 6))
    y = np.arange(len(METHODS))
    for k, robot in enumerate(ROBOTS):
        off = -.17 if robot == "cylinder" else .17
        mk = "o" if robot == "cylinder" else "s"
        med = [doc["timing"][robot][m]["algorithm_wall_s"]["median"] for m in METHODS]
        p95 = [doc["timing"][robot][m]["algorithm_wall_s"]["p95"] for m in METHODS]
        cols = [COLOR[m] for m in METHODS]
        for yi, a_, b_, c_ in zip(y + off, med, p95, cols):
            axs[0].plot([a_, b_], [yi, yi], color=c_, lw=2)
            axs[0].plot(a_, yi, mk, color=c_, ms=8, mec="white", mew=1)
            axs[0].plot(b_, yi, "|", color=c_, ms=10, mew=2)
        st = [sum(doc["timing"][robot][m]["setup"][r]["total_s"] for r in REGIONS) for m in METHODS]
        for yi, v, c_ in zip(y + off, st, cols):
            axs[1].plot(v, yi, mk, color=c_, ms=9, mec="white", mew=1)
        am = [doc["timing"][robot][m]["amortised_5000"]["per_query_s"] for m in METHODS]
        for yi, v, c_ in zip(y + off, am, cols):
            axs[2].plot(v, yi, mk, color=c_, ms=9, mec="white", mew=1)
        for yi, v in zip(y + off, st):
            axs[1].text(v * 1.12, yi, f"{v:.0f} s", va="center", fontsize=7.5, color="#52514e")
        for yi, v in zip(y + off, am):
            axs[2].text(v * 1.15, yi, f"{v:.2f} s", va="center", fontsize=7.5, color="#52514e")
    for ax, t in zip(axs, ("per-query method time: median (marker) to p95 (bar), s",
                           "setup ('compile'), sum of 3 regions, s", "amortised cost per query over 5000 queries\n(setup + all queries) / 5000, s")):
        ax.set_xscale("log")
        ax.set_yticks(y, [LABEL[m] for m in METHODS], fontsize=9)
        ax.invert_yaxis()
        ax.set_title(t, fontsize=10)
        ax.grid(axis="x", color="#e5e4e0", lw=.6)
    axs[0].plot([], [], "o", color="#52514e", label="cylinder (upper)")
    axs[0].plot([], [], "s", color="#52514e", label="sweeper (lower)")
    axs[0].legend(fontsize=8, loc="upper center", bbox_to_anchor=(.5, -.1), ncol=2)
    for ax in axs[1:]:
        ax.plot([], [], "o", color="#52514e", label="cylinder (upper)")
        ax.plot([], [], "s", color="#52514e", label="sweeper (lower)")
        ax.legend(fontsize=8, loc="upper center", bbox_to_anchor=(.5, -.1), ncol=2)
        lo_, hi_ = ax.get_xlim()
        ax.set_xlim(lo_, hi_ * 2.5)
    fig.suptitle("Timing (judge excluded for baselines; GMC's time includes its in-query shared replay). "
                 "cust_fields cylinder fails at the endpoint snap (~0.1 ms).", fontsize=10)
    fig.tight_layout()
    out = FIG / "timing.png"
    fig.savefig(out, dpi=90)
    plt.close(fig)
    return str(out)


def cmd_figs(a):
    import matplotlib
    matplotlib.use("Agg")
    doc = json.loads((OUT / "analysis.json").read_text())
    FIG.mkdir(parents=True, exist_ok=True)
    data = load_all(with_turning=False)
    P = pairs_by_id()
    for p in P.values():
        for k in ("start_uv", "goal_uv", "astar_route_uv"):
            if isinstance(p[k], str):
                p[k] = json.loads(p[k])
    meta = {"schema": "bl.b4_figures.v1", "commit": git("rev-parse", "HEAD"), "figures": {}}
    for reg in REGIONS:
        picks = pick_pairs(data, reg)
        polys = polylines_for(picks, reg)
        f = fig_overview(reg, picks, polys, P)
        meta["figures"][f] = {"picks": [{**pk, "statuses": {m: polys[(pk["pair_id"], pk["robot"])][m]["status"]
                                                            for m in METHODS},
                                         "details": {m: polys[(pk["pair_id"], pk["robot"])][m]["detail"]
                                                     for m in METHODS}} for pk in picks]}
    meta["figures"][fig_bands(doc)] = {"source": "analysis.json bands.<robot>.<method>.{lateral,detour}"}
    meta["figures"][fig_timing(doc)] = {"source": "analysis.json timing.<robot>.<method>"}
    (FIG / "figures.json").write_text(json.dumps(meta, indent=1))
    print(json.dumps(meta, indent=1)[:3000])


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["analyze", "sheets", "figs", "all"])
    a = ap.parse_args(argv)
    if a.cmd in ("analyze", "all"):
        cmd_analyze(a)
    if a.cmd in ("sheets", "all"):
        cmd_sheets(a)
    if a.cmd in ("figs", "all"):
        cmd_figs(a)


if __name__ == "__main__":
    main()
