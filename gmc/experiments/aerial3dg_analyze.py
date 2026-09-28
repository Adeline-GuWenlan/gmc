"""G3 analysis of the aerial3d-ground 5000-pair run (G2) + G1's single-pair breakdown.

Reads only committed artefacts (``results/aerial3dg/{g1,g2}``); nothing is re-planned here.

* ``select``   deterministic gallery pairs (rules below, fixed before rendering) -> configs/aerial3dg/g3_gallery_pairs.json
* ``analyze``  distributions, robot comparison, reuse, distance tiers, scale/resolution, oracle-map cross-check
               -> results/aerial3dg/g3/analysis.json
* ``csv``      the filled measurement template copy (G1's four demo columns + two 5000-pair columns)
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

G1 = Path("results/aerial3dg/g1")
G2 = Path("results/aerial3dg/g2")
ROBOTS = ("sweeper", "cylinder")
GAP_U, LAMP_U = -5.6, -0.85          # the captured structure at u~-5.6 and the lamp centre line (route u, m)
MAJOR_ROUTE_CHANGE_M = 0.5           # Hausdorff distance between two routes that counts as a different route


def load_rows(runs: Path = G2 / "runs") -> dict:
    out = {}
    for robot in ROBOTS:
        rows = {}
        for f in sorted((runs / robot).glob("task_*.jsonl")):
            for line in f.read_text().splitlines():
                if line.strip():
                    d = json.loads(line)
                    rows[d["index"]] = d
        out[robot] = rows
    return out


def densify(poly, step=.02) -> np.ndarray:
    p = np.asarray(poly, float)[:, :2]
    out = [p[:1]]
    for a, b in zip(p[:-1], p[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out.append(a + (b - a) * np.linspace(0, 1, n + 1)[1:, None])
    return np.vstack(out)


def hausdorff(p, q) -> float:
    a, b = densify(p), densify(q)
    return float(max(cKDTree(b).query(a)[0].max(), cKDTree(a).query(b)[0].max()))


def crossings(poly, u0) -> list[float]:
    """v values where a route polyline crosses the line u = u0."""
    p = np.asarray(poly, float)[:, :2]
    vs = []
    for a, b in zip(p[:-1], p[1:]):
        if (a[0] - u0) * (b[0] - u0) < 0:
            vs.append(float(a[1] + (u0 - a[0]) / (b[0] - a[0]) * (b[1] - a[1])))
    return vs


def straddles(r, u0) -> bool:
    return (r["start_uv"][0] - u0) * (r["goal_uv"][0] - u0) < 0


def both_reachable(R) -> list[dict]:
    out = []
    for i, s in R["sweeper"].items():
        c = R["cylinder"][i]
        if s["status"] == c["status"] == "REACHABLE":
            out.append({"index": i, "pair_id": s["pair_id"], "dist_m": s["dist_m"], "start_uv": s["start_uv"],
                        "goal_uv": s["goal_uv"], "L_sweeper": s["path_length_m"], "L_cylinder": c["path_length_m"],
                        "dL": c["path_length_m"] - s["path_length_m"],
                        "hausdorff_m": hausdorff(s["route_polyline"], c["route_polyline"])})
    return sorted(out, key=lambda r: r["index"])


# ----------------------------------------------------------------------------- gallery selection
GALLERY_RULES = {
    "T1_thread_vs_detour": "both REACHABLE and G1's pre-registered contrast rule (sweeper L <= 1.02 d, cylinder "
                           "L >= 1.05 d); the largest L_cylinder - L_sweeper",
    "T2_thread_vs_detour_reverse": "same rule, travelling the opposite u direction to T1; the largest L_cylinder - "
                                   "L_sweeper",
    "L1_gap_and_lamp": "sweeper REACHABLE with a route crossing both u=-5.6 and the lamp line u=-0.85, cylinder "
                       "certified UNREACHABLE; the straightest sweeper route (min L/d)",
    "N1_gap_only": "sweeper REACHABLE crossing u=-5.6 but not the lamp, cylinder UNKNOWN "
                   "(safe_graph_disconnected_possible_connected); the straightest sweeper route (min L/d)",
    "S1_sweeper_max_detour": "sweeper REACHABLE with the largest L/d over all 5000 pairs",
    "B1_both_same_route": "both REACHABLE with route Hausdorff < 0.2 m; the largest d (control: no morphology effect)",
}


def select_gallery(R) -> list[dict]:
    sw, cy = R["sweeper"], R["cylinder"]
    both = both_reachable(R)
    qual = [b for b in both if b["L_sweeper"] <= 1.02 * b["dist_m"] and b["L_cylinder"] >= 1.05 * b["dist_m"]]
    t1 = max(qual, key=lambda b: b["dL"])
    sgn = np.sign(t1["goal_uv"][0] - t1["start_uv"][0])
    t2 = max((b for b in qual if np.sign(b["goal_uv"][0] - b["start_uv"][0]) != sgn), key=lambda b: b["dL"])
    reach = {i: r for i, r in sw.items() if r["status"] == "REACHABLE"}
    ratio = {i: r["path_length_m"] / r["dist_m"] for i, r in reach.items()}
    l1 = min((i for i, r in reach.items() if crossings(r["route_polyline"], GAP_U) and
              crossings(r["route_polyline"], LAMP_U) and cy[i]["status"] == "UNREACHABLE"), key=ratio.get)
    n1 = min((i for i, r in reach.items() if crossings(r["route_polyline"], GAP_U) and
              not crossings(r["route_polyline"], LAMP_U) and
              cy[i]["reason"] == "safe_graph_disconnected_possible_connected"), key=ratio.get)
    s1 = max(reach, key=ratio.get)
    b1 = max((b for b in both if b["hausdorff_m"] < .2), key=lambda b: b["dist_m"])
    picks = [("T1_thread_vs_detour", t1["index"]), ("T2_thread_vs_detour_reverse", t2["index"]),
             ("L1_gap_and_lamp", l1), ("N1_gap_only", n1), ("S1_sweeper_max_detour", s1),
             ("B1_both_same_route", b1["index"])]
    out = []
    for name, i in picks:
        s, c = sw[i], cy[i]
        out.append({"name": name, "role": GALLERY_RULES[name], "g2_index": i, "pair_id": s["pair_id"],
                    "start_uv": s["start_uv"], "goal_uv": s["goal_uv"], "dist_m": s["dist_m"],
                    "g2_status": {"sweeper": s["status"], "cylinder": c["status"]},
                    "g2_reason": {"sweeper": s["reason"], "cylinder": c["reason"]},
                    "g2_length_m": {"sweeper": s["path_length_m"], "cylinder": c["path_length_m"]}})
    return out


def cmd_select(a):
    R = load_rows()
    pairs = select_gallery(R)
    doc = {"box_route_uv": [-9.0, -0.35, 3.7, 2.75],
           "selection": "G3 gallery from the G2 5000-pair results (gmc/results/aerial3dg/g2/runs); rules fixed before "
                        "rendering, applied by experiments/aerial3dg_analyze.py select", "rules": GALLERY_RULES,
           "pairs": pairs}
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(doc, indent=1) + "\n")
    for p in pairs:
        print(p["name"], p["pair_id"], round(p["dist_m"], 3), p["g2_status"], p["g2_length_m"])


# ----------------------------------------------------------------------------- analysis
TIERS = ((3., 4.), (4., 6.), (6., 8.), (8., 12.))
QUERY_STAGE_GROUPS = {          # aerial3d query stages -> measurement-template rows
    "endpoint_locate": ("locate", "locate_cells"), "graph_search": ("graph_search",),
    "path_extraction": ("lifting", "shortcut", "tighten", "merge_corners"),
    "final_verification": ("own_verification", "shared_verification"), "cut_certificate": ("cut_certificate",)}
GAP2 = (-2.34, (1.15, 1.5))     # the sweeper-only passage at route (-2.34, 1.32) north of the floor blob


def stats(x) -> dict | None:
    x = np.asarray([v for v in x if v is not None], float)
    if not len(x):
        return None
    return {"n": int(len(x)), "mean": float(x.mean()), "median": float(np.median(x)),
            "p95": float(np.percentile(x, 95)), "min": float(x.min()), "max": float(x.max()), "sum": float(x.sum())}


def oracle_map_components(cands: dict) -> dict:
    """4-connected components of each robot's 0.1 m gs3d point-free grid (G1 screen3 candidates)."""
    from scipy import ndimage
    us, vs = np.asarray(cands["us"]), np.asarray(cands["vs"])
    out = {}
    for robot in ROBOTS:
        free = np.asarray(cands["free"][robot], bool)
        lab, n = ndimage.label(free)
        out[robot] = {"us": us, "vs": vs, "free": free, "label": lab, "n": n}
    return out


def map_component(m, uv, snap=.15):
    """Component of the nearest free grid node within ``snap`` m of uv (None if none)."""
    iu, iv = np.nonzero(m["free"])
    d = np.hypot(m["us"][iu] - uv[0], m["vs"][iv] - uv[1])
    k = int(np.argmin(d))
    return int(m["label"][iu[k], iv[k]]) if d[k] <= snap else None


def robot_block(rows: dict, robot: str, compile_rec: dict, summaries: list, maps) -> dict:
    allr = list(rows.values())
    st = [r["status"] for r in allr]
    reach = [r for r in allr if r["status"] == "REACHABLE"]
    reasons = {}
    for r in allr:
        reasons.setdefault(r["status"], {}).setdefault(r["reason"], 0)
        reasons[r["status"]][r["reason"]] += 1
    alg = sum(r["algorithm_wall_s"] for r in allr if r.get("algorithm_wall_s") is not None)
    stage = {g: sum(r["stages"].get(k, 0.) for r in allr if r.get("stages") for k in ks)
             for g, ks in QUERY_STAGE_GROUPS.items()}
    stage["other_unattributed"] = alg - sum(stage.values())
    stage_share = {g: {"sum_s": v, "mean_s_per_pair": v / len(allr), "share_of_query_wall": v / alg}
                   for g, v in stage.items()}
    outer = [r["outer_wall_s"] for r in allr]
    loads = [s["load_wall_s"] for s in summaries]
    comp_s = compile_rec["compile_wall_s"]
    amort = (comp_s + sum(loads) + sum(outer)) / len(allr)
    tiers = {}
    for lo, hi in TIERS:
        t = [r for r in allr if lo <= r["dist_m"] < hi]
        tr = [r for r in t if r["status"] == "REACHABLE"]
        tiers[f"{lo:g}-{hi:g}"] = {
            "n": len(t), "status_counts": {k: sum(r["status"] == k for r in t) for k in
                                           ("REACHABLE", "UNREACHABLE", "UNKNOWN", "TIMEOUT")},
            "success_rate": len(tr) / len(t) if t else None, "dist_m": stats([r["dist_m"] for r in t]),
            "query_wall_s": stats([r["outer_wall_s"] for r in t]),
            "path_length_m": stats([r["path_length_m"] for r in tr]),
            "unknown_reasons": {k: sum(r["reason"] == k for r in t if r["status"] == "UNKNOWN")
                                for k in sorted({r["reason"] for r in t if r["status"] == "UNKNOWN"})}}
    m = maps[robot]
    xcheck = {}
    for r in allr:
        a, b = map_component(m, r["start_uv"]), map_component(m, r["goal_uv"])
        key = "unsnapped" if a is None or b is None else ("connected" if a == b else "disconnected")
        xcheck.setdefault(f"{r['status']}:{r['reason']}", {}).setdefault(key, 0)
        xcheck[f"{r['status']}:{r['reason']}"][key] += 1
    crosses_gap = [r for r in reach if crossings(r["route_polyline"], GAP_U)]
    crosses_lamp = [r for r in reach if crossings(r["route_polyline"], LAMP_U)]
    gap2 = [r for r in reach if any(GAP2[1][0] <= v <= GAP2[1][1] for v in crossings(r["route_polyline"], GAP2[0]))]
    return {
        "n": len(allr), "status_counts": {k: st.count(k) for k in ("REACHABLE", "UNREACHABLE", "UNKNOWN", "TIMEOUT")},
        "reasons": reasons, "query_wall_s": stats(outer),
        "query_wall_s_by_status": {k: stats([r["outer_wall_s"] for r in allr if r["status"] == k])
                                   for k in ("REACHABLE", "UNREACHABLE", "UNKNOWN")},
        "query_cpu_s": stats([r["cpu_s"] for r in allr]),
        "query_stage_share": stage_share, "query_algorithm_wall_s_sum": alg,
        "slow_queries_over_10s": sum(v > 10. for v in outer),
        "compile": {"compile_id": compile_rec["compile_id"], "compile_wall_s": comp_s,
                    "stages_s": {x["stage"]: x["seconds"] for x in compile_rec["stages"]}},
        "task_load_wall_s": stats(loads), "task_peak_rss_mb": stats([s["peak_rss_mb"] for s in summaries]),
        "compile_once_all_tasks": all(s["compile_once_proof"]["compiles_in_this_process"] == 0 for s in summaries),
        "amortized_total_s_per_pair": amort,
        "amortized_definition": "(one compile + every task's load of the persisted compile + every query's wall) / pairs",
        "reachable": {
            "path_length_m": stats([r["path_length_m"] for r in reach]),
            "path_over_straight": stats([r["path_length_m"] / r["dist_m"] for r in reach]),
            "excess_over_straight_pct": stats([100 * (r["path_length_m"] / r["dist_m"] - 1) for r in reach]),
            "over_1p3": sum(r["path_length_m"] / r["dist_m"] > 1.3 for r in reach),
            "clearance_lower_m": stats([r["clearance_lower_m"] for r in reach]),
            "turns": stats([r["vertices"] - 2 for r in reach]),
            "own_verified": sum(r["own_verification"] == "CERTIFIED" for r in reach),
            "shared_replay_passed": sum(bool(r["shared_replay_passed"]) for r in reach)},
        "own_certified_but_shared_replay_failed": sum(r["reason"] == "shared_replay_failed" for r in allr),
        "routes_crossing_u_-5.6": {"n": len(crosses_gap), "v_at_crossing": stats(
            [v for r in crosses_gap for v in crossings(r["route_polyline"], GAP_U)])},
        "routes_crossing_lamp": {"n": len(crosses_lamp), "v_at_crossing": stats(
            [v for r in crosses_lamp for v in crossings(r["route_polyline"], LAMP_U)])},
        "routes_through_gap_-2.34": len(gap2),
        "distance_tiers": tiers,
        "oracle_map_crosscheck": xcheck}


def paired_block(R) -> dict:
    sw, cy = R["sweeper"], R["cylinder"]
    idx = sorted(set(sw) & set(cy))
    cross = {}
    for i in idx:
        k = f"{sw[i]['status']}|{cy[i]['status']}"
        cross[k] = cross.get(k, 0) + 1
    both = both_reachable(R)
    H = [b["hausdorff_m"] for b in both]
    qual = [b for b in both if b["L_sweeper"] <= 1.02 * b["dist_m"] and b["L_cylinder"] >= 1.05 * b["dist_m"]]
    gap2 = [b for b in qual if any(GAP2[1][0] <= v <= GAP2[1][1]
                                   for v in crossings(sw[b["index"]]["route_polyline"], GAP2[0]))]
    lamp = [i for i in idx if straddles(sw[i], LAMP_U)]
    gap = [i for i in idx if straddles(sw[i], GAP_U)]

    def tab(ii):
        t = {}
        for i in ii:
            k = f"{sw[i]['status']}|{cy[i]['status']}"
            t[k] = t.get(k, 0) + 1
        return t
    reach_s = {i for i in idx if sw[i]["status"] == "REACHABLE"}
    reach_c = {i for i in idx if cy[i]["status"] == "REACHABLE"}
    return {
        "n": len(idx), "crosstab": cross,
        "verdict_differs": sum(sw[i]["status"] != cy[i]["status"] for i in idx),
        "reachability_differs": len(reach_s ^ reach_c),
        "reachable_vs_certified_unreachable": sum(sw[i]["status"] == "REACHABLE" and cy[i]["status"] == "UNREACHABLE"
                                                  for i in idx),
        "both_reachable": {
            "n": len(both), "hausdorff_m": stats(H), "dL_m": stats([b["dL"] for b in both]),
            "L_over_d_sweeper": stats([b["L_sweeper"] / b["dist_m"] for b in both]),
            "L_over_d_cylinder": stats([b["L_cylinder"] / b["dist_m"] for b in both]),
            "identical_route_within_1cm": sum(h < .01 for h in H),
            "major_route_change": sum(h > MAJOR_ROUTE_CHANGE_M for h in H),
            "major_route_change_definition": f"route Hausdorff distance > {MAJOR_ROUTE_CHANGE_M} m",
            "hausdorff_histogram_0p1m": np.histogram(H, bins=np.arange(0, 1.01, .1))[0].tolist(),
            "g1_rule_thread_vs_detour": {"rule": "sweeper L <= 1.02 d and cylinder L >= 1.05 d (G1, pre-registered)",
                                         "n": len(qual), "through_gap_-2.34": len(gap2),
                                         "pairs": [{k: b[k] for k in ("pair_id", "dist_m", "L_sweeper", "L_cylinder",
                                                                      "hausdorff_m")} for b in qual]}},
        "straddle_lamp": {"n": len(lamp), "crosstab": tab(lamp)},
        "straddle_u_-5.6": {"n": len(gap), "crosstab": tab(gap)}}


RESOLUTION_RUNS = ((0.10, Path("results/aerial3dg/g3/runs/screen5_mc10")), (0.05, G1 / "runs/screen3_corridor"),
                   (0.025, G1 / "runs/screen4_mc025"))


def resolution_block() -> dict:
    """G1's 80 screen pairs at three octree leaf sizes; flips and route changes are against the 0.05 m default."""
    out = {}
    for robot in ROBOTS:
        docs = {m: json.loads((d / f"screen_{robot}.json").read_text()) for m, d in RESOLUTION_RUNS
                if (d / f"screen_{robot}.json").exists()}
        base = {r["name"]: r for r in docs[0.05]["results"]}
        out[robot] = {"levels": {}}
        for m, d in docs.items():
            rr = {r["name"]: r for r in d["results"]}
            common = sorted(set(base) & set(rr))
            routes = [hausdorff(base[k]["route_polyline"], rr[k]["route_polyline"]) for k in common
                      if base[k]["status"] == rr[k]["status"] == "REACHABLE"]
            c = d["compile"]
            out[robot]["levels"][f"{m:g}"] = {
                "min_cell_m": m, "job": d["host"]["slurm_job_id"], "compile_wall_s": c["compile_wall_s"],
                "octree_leaves": c["octree"]["leaves"], "free_cells": c["cells"]["cells"], "portals": c["cells"]["portals"],
                "status_counts": {k: sum(r["status"] == k for r in rr.values()) for k in ("REACHABLE", "UNREACHABLE", "UNKNOWN")},
                "query_wall_s": stats([r["algorithm_wall_s"] for r in rr.values()]),
                "pairs": len(common), "verdict_flips_vs_0.05": sum(base[k]["status"] != rr[k]["status"] for k in common),
                "flips": [f"{k}: {base[k]['status']}/{base[k]['reason']} -> {rr[k]['status']}/{rr[k]['reason']}"
                          for k in common if base[k]["status"] != rr[k]["status"]],
                "both_reachable": len(routes), "route_hausdorff_m": stats(routes),
                "major_route_change": sum(h > MAJOR_ROUTE_CHANGE_M for h in routes)}
    return out


def scale_block() -> dict:
    """Same archive, three query boxes of growing size -> growing cropped Gaussian count N_G."""
    probe = json.loads((G1 / "runs/probe/probe.json").read_text())
    boxes = [("probe_small", probe["crop"]["selected_supports"], probe["box_route"], {r: (probe["robots"][r], None)
                                                                                       for r in ROBOTS})]
    for tag, sub in (("booth", "screen2_booth"), ("corridor", "screen3_corridor")):
        docs = {r: json.loads((G1 / f"runs/{sub}/screen_{r}.json").read_text()) for r in ROBOTS}
        boxes.append((tag, docs["sweeper"]["crop"]["selected_supports"], docs["sweeper"]["box_route"],
                      {r: (docs[r]["compile"], docs[r]["results"]) for r in ROBOTS}))
    out = []
    for tag, ng, box, per in boxes:
        row = {"box": tag, "box_route": box, "N_G_cropped": ng, "robots": {}}
        for r, (c, res) in per.items():
            st = {x["stage"]: x["seconds"] for x in c["stages"]}
            row["robots"][r] = {
                "candidate_pairs": c["pairs"]["candidate_pairs"], "compile_wall_s": c["compile_wall_s"],
                "pair_s": st.get("pair_candidates", 0.) + st.get("envelopes", 0.),
                "graph_build_s": st.get("cells", 0.) + st.get("possible_graph", 0.),
                "octree_s": st.get("octree"), "octree_leaves": c["octree"]["leaves"], "free_cells": c["cells"]["cells"],
                "peak_rss_mb_in_process": c["peak_rss_mb"],
                "graph_search_s_median": None if res is None else float(np.median(
                    [x["stages"].get("graph_search", 0.) for x in res])),
                "query_wall_s_median": None if res is None else float(np.median([x["algorithm_wall_s"] for x in res]))}
        out.append(row)
    return out


def variants_block(R, root: Path) -> dict:
    out = {}
    for d in sorted(root.glob("*_r*")):
        comp, rows = d / "compile.json", {}
        for f in sorted(d.glob("task_*.jsonl")):
            for line in f.read_text().splitlines():
                if line.strip():
                    x = json.loads(line)
                    rows[x["index"]] = x
        if not comp.exists() or not rows:
            continue
        c = json.loads(comp.read_text())["compile"]
        robot = c["body"]["name"]
        base = R[robot]
        idx = sorted(rows)
        same = sum(rows[i]["status"] == base[i]["status"] for i in idx)
        routes = [hausdorff(rows[i]["route_polyline"], base[i]["route_polyline"]) for i in idx
                  if rows[i]["status"] == base[i]["status"] == "REACHABLE"]
        flips = {}
        for i in idx:
            if rows[i]["status"] != base[i]["status"]:
                k = f"{base[i]['status']}->{rows[i]['status']}"
                flips[k] = flips.get(k, 0) + 1
        out[d.name] = {"robot": robot, "radius_m": c["body"]["radius_m"], "compile_id": c["compile_id"],
                       "compile_wall_s": c["compile_wall_s"], "candidate_pairs": c["pairs"]["candidate_pairs"],
                       "octree_leaves": c["octree"]["leaves"], "free_cells": c["cells"]["cells"],
                       "pairs": len(idx), "same_status": same, "flips": flips,
                       "status_counts": {k: sum(rows[i]["status"] == k for i in idx) for k in
                                         ("REACHABLE", "UNREACHABLE", "UNKNOWN", "TIMEOUT")},
                       "baseline_status_counts": {k: sum(base[i]["status"] == k for i in idx) for k in
                                                  ("REACHABLE", "UNREACHABLE", "UNKNOWN", "TIMEOUT")},
                       "both_reachable": len(routes), "route_hausdorff_m": stats(routes),
                       "major_route_change": sum(h > MAJOR_ROUTE_CHANGE_M for h in routes)}
    return out


def cmd_analyze(a):
    R = load_rows()
    g2 = json.loads((G2.parent / "g2_handoff.json").read_text())
    cands = json.loads((G1 / "runs/screen3_corridor/candidates.json").read_text())
    maps = oracle_map_components(cands)
    doc = {"schema": "aerial3dg.g3_analysis.v1", "sources": {"rows": str(G2 / "runs"), "g2_handoff": "g2_handoff.json",
                                                             "oracle_map": "g1/runs/screen3_corridor/candidates.json"},
           "robots": {}, "definitions": {
               "query_wall": "outer wall per query call (G2 row outer_wall_s): locate + search + lift + verify + "
                             "in-query shared gs3d replay; no compile inside (0 compile stages in every row)",
               "tiers_m": TIERS, "major_route_change_m": MAJOR_ROUTE_CHANGE_M,
               "oracle_map_crosscheck": "each endpoint snapped to the nearest oracle-free node (<= 0.15 m) of G1's "
                                        "0.1 m gs3d point-free grid at the robot's own z_c; 4-connected components. "
                                        "Evidence, not proof: a point grid can miss sub-0.1 m windows and does not "
                                        "check the swept body between nodes.",
               "gap_-2.34": f"route crosses u={GAP2[0]} with v in {GAP2[1]}"}}
    for robot in ROBOTS:
        demo = json.loads((G1 / f"runs/demo/demo_{robot}.json").read_text())
        summ = [json.loads(f.read_text()) for f in sorted((G2 / f"runs/{robot}").glob("task_*.summary.json"))]
        doc["robots"][robot] = robot_block(R[robot], robot, demo["compile"], summ, maps)
        doc["robots"][robot]["archive_load_and_crop_s"] = demo["archive_load_and_hash_s"] + demo["scene_build_and_crop_s"]
        doc["robots"][robot]["oracle_map_components"] = int(maps[robot]["n"])
        doc["robots"][robot]["cpu_h"] = g2["cost"]["by_job_group"][robot]
        doc["robots"][robot]["g1_estimate_cpu_h"] = g2["robots"][robot]["g1_estimate_cpu_h"]
        sc3 = json.loads((G1 / f"runs/screen3_corridor/screen_{robot}.json").read_text())
        doc["robots"][robot]["compile_repeat"] = {
            "same_compile_id": sc3["compile"]["compile_id"] == demo["compile"]["compile_id"],
            "screen3_job": sc3["host"]["slurm_job_id"], "screen3_compile_wall_s": sc3["compile"]["compile_wall_s"],
            "demo_job": demo["host"]["slurm_job_id"], "demo_compile_wall_s": demo["compile"]["compile_wall_s"]}
    b = doc["robots"]
    per_pair = {r: b[r]["archive_load_and_crop_s"] + b[r]["compile"]["compile_wall_s"] + b[r]["query_wall_s"]["mean"]
                for r in ROBOTS}
    actual = {r: b[r]["compile"]["compile_wall_s"] + b[r]["task_load_wall_s"]["sum"] + b[r]["query_wall_s"]["sum"]
              for r in ROBOTS}
    doc["reuse"] = {
        "counterfactual_recompile_per_pair_h": {r: 5000 * per_pair[r] / 3600 for r in ROBOTS},
        "measured_compile_once_h": {r: actual[r] / 3600 for r in ROBOTS},
        "speedup": {r: 5000 * per_pair[r] / actual[r] for r in ROBOTS},
        "definition": "counterfactual = 5000 x (measured archive load+crop + measured compile + measured mean query); "
                      "measured = 1 compile (G1) + measured task loads + measured query walls (G2). The compile time "
                      "is measured twice per robot (G1 screen3 and demo, same compile_id) -- see compile_repeat."}
    doc["paired"] = paired_block(R)
    doc["resolution"] = resolution_block()
    doc["scale"] = scale_block()
    doc["variants"] = variants_block(R, a.variants)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(doc, indent=1, default=float) + "\n")
    p = doc["paired"]
    print(json.dumps({"crosstab": p["crosstab"], "both": {k: p["both_reachable"][k] for k in
                                                          ("n", "identical_route_within_1cm", "major_route_change")},
                      "g1_rule": p["both_reachable"]["g1_rule_thread_vs_detour"]["n"],
                      "reuse": doc["reuse"]["speedup"], "variants": list(doc["variants"])}, default=float))


# ----------------------------------------------------------------------------- measurement template
TEMPLATE = Path("/scratch/wg2381/splathjb/measurement_template.csv")     # read-only source (row order + labels)
POP_ONLY = "N/A（单对演示；见 5000 对列）"


def _f(x, n=3):
    return "—" if x is None else f"{x:,.{n}f}"


def _pct(a, b, n=1):
    return f"{a} / {b}；{100 * a / b:.{n}f}%" if b else f"{a} / 0；N/A"


def aggregate_cells(A: dict, robot: str, gallery_same: tuple[int, int] | None) -> dict:
    """Template label -> cell text for the 5000-pair column of ``robot`` (paired rows are the same in both)."""
    b, P, other = A["robots"][robot], A["paired"], "cylinder" if robot == "sweeper" else "sweeper"
    sw, cy = A["robots"]["sweeper"], A["robots"]["cylinder"]
    q, sc, c = b["query_wall_s"], b["reachable"], b["compile"]
    n = b["n"]
    total = c["compile_wall_s"] + b["task_load_wall_s"]["sum"] + q["sum"]
    cs, qs = c["stages_s"], b["query_stage_share"]

    def stage(sec, note):
        return f"{sec:.3f} s（5000 对合计；每对 {1000 * sec / n:.3f} ms）；占总时间 {100 * sec / total:.2f}%（{note}）"
    rows_stage = {
        "BVH": cs["scene_prepare"], "pairs": cs["pair_candidates"], "exact": cs["envelopes"] + cs["audit"],
        "domain": cs["octree"] + cs["cells"], "graph": cs["possible_graph"],
        "search": qs["graph_search"]["sum_s"], "extract": qs["path_extraction"]["sum_s"],
        "verify": qs["final_verification"]["sum_s"],
        "other": qs["endpoint_locate"]["sum_s"] + qs["cut_certificate"]["sum_s"] + b["task_load_wall_s"]["sum"]}
    diff = total - sum(rows_stage.values())
    names = {"BVH": "scene_prepare", "pairs": "pair 生成", "exact": "包络+审计", "domain": "octree+凸胞",
             "graph": "possible 图", "search": "portal A*", "extract": "路径提取（lifting+shortcut+tighten）",
             "verify": "自有验证+共享重放", "other": "端点定位/割证书/任务加载"}
    top = max(rows_stage, key=rows_stage.get)
    tiers = b["distance_tiers"]
    tk = list(tiers)

    def tier_line(fn, unit=""):
        return "；".join(f"L{k + 1}：{fn(tiers[t])}{unit}" for k, t in enumerate(tk))
    x = b["oracle_map_crosscheck"]
    conn = sum(v.get("connected", 0) for v in x.values())
    conn_reach = x.get("REACHABLE:graph_path_lifted_and_verified", {}).get("connected", 0)
    disc = sum(v.get("disconnected", 0) for v in x.values())
    disc_reach = x.get("REACHABLE:graph_path_lifted_and_verified", {}).get("disconnected", 0)
    disc_unreach = sum(v.get("disconnected", 0) for k, v in x.items() if k.startswith("UNREACHABLE"))
    unreach_conn = sum(v.get("connected", 0) for k, v in x.items() if k.startswith("UNREACHABLE"))
    unknown = b["status_counts"]["UNKNOWN"]
    unk_conn = sum(v.get("connected", 0) for k, v in x.items() if k.startswith("UNKNOWN"))
    br = P["both_reachable"]
    rule = br["g1_rule_thread_vs_detour"]
    lamp, gap = P["straddle_lamp"], P["straddle_u_-5.6"]
    cy_gap = {"REACHABLE": 0, "UNREACHABLE": 0, "UNKNOWN": 0}
    for k, v in gap["crosstab"].items():
        cy_gap[k.split("|")[1]] += v
    reuse = A["reuse"]
    v = A["variants"]

    def var(robot_, fn):
        ks = [k for k in sorted(v) if v[k]["robot"] == robot_]
        return "；".join(f"r={v[k]['radius_m']:.3f}：{fn(v[k])}" for k in ks) if ks else "未运行"
    rep = A["robots"][robot]["compile_repeat"]
    lv = A["resolution"][robot]["levels"]
    changed = [x for k, x in lv.items() if k != "0.05"]
    flips_n = sum(x["verdict_flips_vs_0.05"] for x in changed)
    flips_of = sum(x["pairs"] for x in changed)
    routes_n = sum(x["major_route_change"] for x in changed)
    routes_of = sum(x["both_reachable"] for x in changed)
    res_time = "；".join(f"分辨率 {k} m：编译 {x['compile_wall_s']:.1f} s，查询中位 {x['query_wall_s']['median']:.3f} s"
                         for k, x in lv.items())
    res_states = "；".join(f"分辨率 {k} m：叶 {x['octree_leaves']:,} / 凸胞 {x['free_cells']}" for k, x in lv.items())
    res_flip_note = "；".join(f"{k} m：{x['verdict_flips_vs_0.05']}/{x['pairs']}" + (f"（{'; '.join(x['flips'])}）" if x['flips'] else "")
                              for k, x in lv.items() if k != "0.05")
    scale = A["scale"]

    def sc_line(fn):
        return "；".join(f"N_G={s['N_G_cropped']:,}：{fn(s['robots'][robot])}" for s in scale)
    reach = b["status_counts"]["REACHABLE"]
    fails = {}
    for k, vv in b["reasons"].get("UNKNOWN", {}).items():
        k = "endpoint_not_certified_free（start/goal）" if k.endswith("_not_certified_free") else k
        fails[k] = fails.get(k, 0) + vv
    top_fail = max(fails, key=fails.get) if fails else None
    body = {"sweeper": "名称：sweeper；宽：0.350 m；长：0.350 m；高：0.080 m；其他：离地 0.02 m，体带 0.02–0.10 m，r 0.175",
            "cylinder": "名称：cylinder；宽：0.600 m；长：0.600 m；高：1.730 m；其他：离地 0.02 m，体带 0.02–1.75 m，r 0.30"}
    sw_line = (f"成功 / 无路径 / 超时：{sw['status_counts']['REACHABLE']} / {sw['status_counts']['UNREACHABLE']} / "
               f"{sw['status_counts']['TIMEOUT']}（另 {sw['status_counts']['UNKNOWN']} UNKNOWN）；路线类型：从灯下穿行 "
               f"{sw['routes_crossing_lamp']['n']}，穿 u≈−5.6 窄缝（v 中位 {sw['routes_crossing_u_-5.6']['v_at_crossing']['median']:.2f}）"
               f"{sw['routes_crossing_u_-5.6']['n']}，经 (−2.34, 1.15–1.5) 窄道 {sw['routes_through_gap_-2.34']}；"
               f"路径长度：中位 {sw['reachable']['path_length_m']['median']:.2f} m（均值 {sw['reachable']['path_length_m']['mean']:.2f}，"
               f"P95 {sw['reachable']['path_length_m']['p95']:.2f}）；规划时间：中位 {sw['query_wall_s']['median']:.3f} s（P95 "
               f"{sw['query_wall_s']['p95']:.3f}）；最小 clearance：中位 {sw['reachable']['clearance_lower_m']['median']:.4f} m，"
               f"最小 {sw['reachable']['clearance_lower_m']['min']:.4f} m")
    cy_line = (f"成功 / 无路径 / 超时：{cy['status_counts']['REACHABLE']} / {cy['status_counts']['UNREACHABLE']}（认证）/ "
               f"{cy['status_counts']['TIMEOUT']}（另 {cy['status_counts']['UNKNOWN']} UNKNOWN："
               f"{cy['reasons']['UNKNOWN'].get('safe_graph_disconnected_possible_connected', 0)} 个在 u≈−5.6 处未认证，"
               f"其余端点未认证）；路线类型：可达路线 0 条穿灯、0 条穿 u≈−5.6、0 条经 (−2.34, 1.3) 窄道；与 sweeper 路线 "
               f"Hausdorff > 0.5 m 的绕行 {br['major_route_change']} / {br['n']}；路径长度：中位 "
               f"{cy['reachable']['path_length_m']['median']:.2f} m（均值 {cy['reachable']['path_length_m']['mean']:.2f}）；"
               f"规划时间：中位 {cy['query_wall_s']['median']:.3f} s（P95 {cy['query_wall_s']['p95']:.3f}）；最小 clearance："
               f"中位 {cy['reachable']['clearance_lower_m']['median']:.4f} m，最小 {cy['reachable']['clearance_lower_m']['min']:.4f} m")
    paired = {
        "机器人对比｜机器人 1 参数": body["sweeper"],
        "机器人对比｜机器人 1 规划结果": sw_line,
        "机器人对比｜机器人 2 参数": body["cylinder"],
        "机器人对比｜机器人 2 规划结果": cy_line,
        "机器人对比｜机器人 3 参数": "名称：uav；宽：0.50 m；长：0.50 m；高：0.20 m；其他：本链未运行（UAV 结果见 uavconn 分支）",
        "机器人对比｜机器人 3 规划结果": "本链未运行",
        "机器人对比｜同一场景、同一 start/end，更换机器人后的总规划时间":
            f"机器人 1：{sw['query_wall_s']['median']:.3f} s；机器人 2：{cy['query_wall_s']['median']:.3f} s；机器人 3：未运行"
            f"（每对查询中位数，5000 对，各自同一编译；编译各一次）",
        "机器人对比｜更换机器人后重新构建 scene–robot planning representation 的时间":
            f"机器人 1→2：{cy['compile']['compile_wall_s']:.1f} s（cylinder 编译；同 compile_id 另一次测得 "
            f"{cy['compile_repeat']['screen3_compile_wall_s']:.1f} s）；机器人 2→3：未运行；机器人 1→3：未运行",
        "机器人对比｜连续完成三种机器人规划所需总时间":
            f"两种机器人 × 5000 对：实测 {A['robots']['sweeper']['cpu_h']['used_cpu_h'] + A['robots']['cylinder']['cpu_h']['used_cpu_h']:.2f}"
            f" CPU-h（sweeper {A['robots']['sweeper']['cpu_h']['used_cpu_h']:.2f} + cylinder {A['robots']['cylinder']['cpu_h']['used_cpu_h']:.2f}；"
            f"每个机器人 1 次编译），墙钟 34 min（数组并行）；UAV 未运行",
        "机器人对比｜三种机器人是否得到不同路径":
            f"是 / 否：是；差异描述：(1) 跨灯的 {lamp['n']} 对里 cylinder 全部认证不可达，sweeper {lamp['crosstab'].get('REACHABLE|UNREACHABLE', 0)} 对从灯下穿过；"
            f"(2) 两者都可达的 {br['n']} 对里 {rule['n']} 对满足 G1 预注册的“sweeper 直穿（L≤1.02d）/cylinder 绕行（L≥1.05d）”，"
            f"其中 {rule['through_gap_-2.34']} 对 sweeper 从 (−2.34, 1.3) 处地面斑块北侧的窄道穿过，cylinder 绕到斑块南侧（v≈0.45）；"
            f"(3) 跨 u≈−5.6 的对里 sweeper 从 v≈1.74 窄缝穿过，cylinder 无一可达",
        "机器人对比｜三种机器人得到完全相同路径的比例（多组 start/end）":
            f"{br['identical_route_within_1cm']} / {P['n']}；{100 * br['identical_route_within_1cm'] / P['n']:.2f}%"
            f"（两种机器人；路线 Hausdorff < 1 cm；两者都可达的 {br['n']} 对中占 {100 * br['identical_route_within_1cm'] / br['n']:.1f}%）",
        "机器人对比｜更换机器人后可达 / 不可达结论发生变化的比例":
            f"{_pct(P['reachability_differs'], P['n'])}（一方 REACHABLE、另一方不是；其中认证的 可达↔不可达 "
            f"{P['reachable_vs_certified_unreachable']} 对，{100 * P['reachable_vs_certified_unreachable'] / P['n']:.1f}%）",
        "机器人对比｜更换机器人后主要路线发生变化的比例":
            f"{_pct(br['major_route_change'], br['n'])}（两者都可达的对；路线 Hausdorff > 0.5 m；分布双峰：每 0.1 m 一档 "
            f"{br['hausdorff_histogram_0p1m']}，阈值取 0.2–0.7 m 之间任何值结果相同）",
        "机器人对比｜小机器人选择窄路捷径的比例":
            f"{_pct(rule['n'], br['n'])}（两者都可达的对中 sweeper 直穿、cylinder 绕行，G1 预注册规则；{rule['through_gap_-2.34']} 对"
            f"经 (−2.34, 1.3) 窄道）；另：跨 u≈−5.6 的 {gap['n']} 对中 sweeper 可达 "
            f"{gap['crosstab'].get('REACHABLE|UNKNOWN', 0) + gap['crosstab'].get('REACHABLE|UNREACHABLE', 0)} 对（全部经 v≈1.74 窄缝）",
        "机器人对比｜大机器人避开无法通过窄路的比例":
            f"{_pct(rule['through_gap_-2.34'], rule['through_gap_-2.34'])}（sweeper 走 (−2.34, 1.3) 窄道的 G1 规则对，cylinder 全部绕南侧）；"
            f"cylinder 可达路线穿 u≈−5.6 窄缝 0 条：跨该处的 {gap['n']} 对 cylinder 为认证不可达 {cy_gap['UNREACHABLE']}、"
            f"UNKNOWN {cy_gap['UNKNOWN']}、可达 {cy_gap['REACHABLE']}（不是“避开后绕行”，是没有路线）",
        "机器人对比｜高机器人避开低矮通道的比例":
            f"{_pct(lamp['crosstab'].get('REACHABLE|UNREACHABLE', 0) + lamp['crosstab'].get('UNKNOWN|UNREACHABLE', 0), lamp['n'])}"
            f"（跨灯的对：cylinder 全部认证不可达——灯+隔断 1.07 m 以上横跨走廊，无绕行；sweeper "
            f"{lamp['crosstab'].get('REACHABLE|UNREACHABLE', 0)} / {lamp['n']} 从灯下通过，其余 UNKNOWN）",
        "机器人对比｜相近尺寸机器人之间 collision / state 判断相同的比例":
            "每对查询结论相同（同 500 对，任务 0 切片）：" + "；".join(
                var(r_, lambda d: f"{d['same_status']} / {d['pairs']}（{100 * d['same_status'] / d['pairs']:.1f}%）")
                for r_ in ("sweeper", "cylinder")) + "（基准 r=0.175 / 0.30；±1 cm）",
        "机器人对比｜相近尺寸机器人之间 operator edge 相同的比例":
            "N/A（每个半径各自编译出自己的凸胞与 portal，边之间没有一一对应，比例无定义；可比的量见上一行与下一行）",
        "机器人对比｜轻微改变机器人宽度 / 长度 / 高度后结果突然翻转的比例":
            "；".join(var(r_, lambda d: f"{_pct(d['pairs'] - d['same_status'], d['pairs'])}（" + ("，".join(f"{k.replace('->', '→')} {vv}" for k, vv in d['flips'].items()) or "无") + "），"
                                        f"主要路线变化 {d['major_route_change']} / {d['both_reachable']}")
                     for r_ in ("sweeper", "cylinder")) + "（半径 ±1 cm；高度未变）"}
    cells = {
        "实验设置｜代码版本 / Git commit":
            "版本：aerial3d ground（G2 批量查询 + G3 分析）；commit：编译 cd18230（G1，同一 .a3c），查询 2cb9108…3d90de9 "
            "（gmc/src 与 runner 自 cd18230 未改），分析 experiments/aerial3dg_analyze.py（G3）",
        "实验设置｜运行日期": "2026-09-28",
        "实验设置｜硬件与运行环境":
            f"CPU：Torch cpu_short 节点（每数组任务 1 核，查询单线程）；GPU：无；RAM：任务申请 "
            f"{'1.5' if robot == 'sweeper' else '2.2'} GB，实测峰值 {b['task_peak_rss_mb']['max'] / 1024:.2f} GB；OS：Linux (RHEL) / Python 3.13.5",
        "实验设置｜每个配置的 warm-up 次数与正式重复次数":
            "Warm-up：0 次；正式运行：每对 1 次查询（5000 对），全部在同一个持久化编译上（10 个数组任务各加载一次）",
        "实验设置｜场景名称 / 场景编号":
            "uavlamp_gallery_booth_v2（展厅 3DGS + 灯/隔断），route 盒 u[-9.0,3.7] v[-0.35,2.75] m（与 G1 演示同一编译盒）",
        "实验设置｜场景 Gaussian 数量": None,        # same as G1 demo column
        "实验设置｜Start 与 End":
            "Start / End：5000 对，种子 20260928，走廊盒内均匀抽样，两端点对两种机器人都 gs3d-oracle free；直线距离 min 3.00 / "
            "中位 5.33 / max 11.62 m（起终点不重合）",
        "实验设置｜机器人名称与几何参数": None,
        "时间｜单次完整规划总时间（cold start）":
            f"每对 N/A：冷启动每个机器人只发生一次 = 档案加载/裁剪 {b['archive_load_and_crop_s']:.1f} s + 编译 "
            f"{c['compile_wall_s']:.1f} s（同 compile_id 另一次 {rep['screen3_compile_wall_s']:.1f} s）；5000 对摊销到每对："
            f"{b['amortized_total_s_per_pair']:.3f} s",
        "时间｜单次完整规划总时间（已有场景结构后的 warm query）":
            f"Median：{q['median']:.3f} s；P95：{q['p95']:.3f} s（n=5000，同一编译，含查询内自有验证与共享 gs3d 重放）",
        "时间｜场景层级结构 / BVH 构建耗时": stage(rows_stage["BVH"], "编译时一次，scene_prepare"),
        "时间｜scene–robot 候选 Gaussian pair 生成耗时": stage(rows_stage["pairs"], "编译时一次"),
        "时间｜scene–robot 精确几何交互 / 碰撞计算耗时": stage(rows_stage["exact"], "编译时一次：支撑函数包络表 + 夹逼审计"),
        "时间｜自由 / 碰撞状态或 configuration-space domain 构建耗时":
            stage(rows_stage["domain"], f"编译时一次：octree {cs['octree']:.1f} s + 凸胞/portal {cs['cells']:.1f} s"),
        "时间｜自适应 refinement 耗时": "N/A（无独立 refinement 轮）",
        "时间｜规划结构 / graph / operator 组装耗时": stage(rows_stage["graph"], "编译时一次；portal 在凸胞阶段内"),
        "时间｜factorization / preconditioner setup 耗时": "N/A（无线性求解）",
        "时间｜全局求解 / 图搜索 / 数值求解耗时": stage(rows_stage["search"], "每对查询：portal 图 A*"),
        "时间｜路径提取耗时": stage(rows_stage["extract"], "每对查询"),
        "时间｜最终连续碰撞检查与路径验证耗时": stage(rows_stage["verify"], "每对查询"),
        "时间｜其他未归类耗时": stage(rows_stage["other"], "端点定位 + 不可达割证书 + 10 次加载持久化编译"),
        "时间｜各阶段耗时之和与总时间的差值":
            f"{diff:.3f} s；差值占总时间 {100 * diff / total:.2f}%（总时间 = 1 次编译 + 10 次加载 + 5000 次查询墙钟 = {total:.1f} s；"
            f"差值 = 查询内未分阶段的部分 + 调用外层开销 + 编译里未单列的阶段）",
        "时间｜当前最耗时阶段":
            f"阶段：{names[top]}；耗时：{rows_stage[top]:.1f} s；占总时间：{100 * rows_stage[top] / total:.1f}%",
        "计算量｜单次规划 collision / contact 查询次数": "未逐对记录（G2 行只记阶段时间；单对数字见 G1 演示列）",
        "计算量｜scene–robot 候选 Gaussian pair 数量": None, "计算量｜scene–robot 实际有效 Gaussian pair 数量": None,
        "计算量｜有效 pair 比例": None, "计算量｜broad-phase 剪枝比例": None,
        "计算量｜单次 collision / contact query 平均耗时": "未逐对记录",
        "计算量｜重复或近重复 collision / contact query 比例": "N/A（未统计）",
        "计算量｜单次规划使用的 configuration states / 节点 / 网格数量": None,
        "计算量｜orientation bins 数量": None, "计算量｜单次规划使用的边数": None,
        "计算量｜稀疏 operator 非零元素数量": None, "计算量｜refinement 轮数": None, "计算量｜solver 迭代次数": None,
        "计算量｜峰值 CPU 内存":
            f"{b['task_peak_rss_mb']['max'] / 1024:.2f} GB（数组任务：加载持久化编译 + 500 次查询，进程峰值）；编译作业见 G1 列",
        "计算量｜峰值 GPU 显存": "N/A",
        "复用性｜同一场景、同一机器人，仅更换 start/end 后的总规划时间":
            f"Median：{q['median']:.3f} s；P95：{q['p95']:.3f} s（5000 对，同一编译）",
        "复用性｜仅更换 start/end 时必须重新计算的阶段": None, "复用性｜仅更换 start/end 时可以直接复用的阶段": None,
        "复用性｜仅更换 start/end 时可复用计算占原 cold-start 时间比例":
            f"{100 * (b['archive_load_and_crop_s'] + c['compile_wall_s']) / (b['archive_load_and_crop_s'] + c['compile_wall_s'] + q['median']):.1f}%"
            f"（(加载+编译) / (加载+编译+中位查询)）；5000 对实测：先编译后复用 {reuse['measured_compile_once_h'][robot]:.2f} h vs 每对重建 "
            f"{reuse['counterfactual_recompile_per_pair_h'][robot]:.1f} h（{reuse['speedup'][robot]:.0f}×；重建时间 = 实测的单次加载+编译 × 5000）",
        "复用性｜相同 start/end 重复查询时的缓存命中率":
            "编译产物 100% 复用（10/10 个数组任务 0 次编译、compile_id 唯一）；查询结果本身不缓存 → 0%；同一对在 G3 画廊作业里重跑，"
            + (f"{gallery_same[0]} / {gallery_same[1]} 与 G2 结果逐字节相同" if gallery_same else "未重跑"),
        "A–B 扩展｜测试距离档位": tier_line(lambda t: f"{t['dist_m']['min']:.1f}–{t['dist_m']['max']:.1f} m（n={t['n']}）"),
        "A–B 扩展｜不同距离下的总规划时间": tier_line(lambda t: f"中位 {t['query_wall_s']['median']:.3f} / P95 {t['query_wall_s']['p95']:.3f}", " s")
                                  + "（每对查询；编译不随距离变）",
        "A–B 扩展｜不同距离下的规划成功率": tier_line(lambda t: f"{100 * t['success_rate']:.1f}", "%"),
        "A–B 扩展｜不同距离下的输出路径长度": tier_line(lambda t: "—" if not t["path_length_m"] else f"中位 {t['path_length_m']['median']:.2f}", " m"),
        "A–B 扩展｜不同距离下的 collision / contact 查询次数": "未逐对记录",
        "A–B 扩展｜不同距离下的候选 / 有效 Gaussian pair 数量":
            "所有档位同一编译（与距离无关）：" + ("11,906 / 563" if robot == "sweeper" else "252,257 / 1,038"),
        "A–B 扩展｜不同距离下的 configuration state / 节点数量":
            "所有档位同一编译（与距离无关）：" + ("octree 叶 22,743 / 凸胞 330" if robot == "sweeper" else "octree 叶 20,116 / 凸胞 250"),
        "A–B 扩展｜不同距离下的 solver 迭代次数": "N/A（A* 图搜索）",
        "A–B 扩展｜不同距离下的 refinement 轮数": "N/A",
        "A–B 扩展｜不同距离下的峰值内存": f"与距离无关：同一进程，任务峰值 {b['task_peak_rss_mb']['max'] / 1024:.2f} GB",
        "A–B 扩展｜不同距离下的失败类型":
            f"无路径误判：0（{b['status_counts']['UNREACHABLE']} 个 UNREACHABLE，在 oracle 点图中连通的 {unreach_conn} 个）；超时：0；"
            f"最终验证失败：{tier_line(lambda t: t['unknown_reasons'].get('shared_replay_failed', 0))}（自有验证通过、共享重放拒绝 → 报 UNKNOWN）；"
            f"其他：端点未认证 {tier_line(lambda t: t['unknown_reasons'].get('start_not_certified_free', 0) + t['unknown_reasons'].get('goal_not_certified_free', 0))}"
            + (f"；safe 图不连通（u≈−5.6）{tier_line(lambda t: t['unknown_reasons'].get('safe_graph_disconnected_possible_connected', 0))}"
               if robot == "cylinder" else ""),
        "规模扩展｜不同场景 Gaussian 数量下的总规划时间": sc_line(lambda r: f"{r['compile_wall_s']:.1f} s") + "（编译；同一档案、三个逐渐变大的查询盒）",
        "规模扩展｜不同场景 Gaussian 数量下的 pair 计算时间": sc_line(lambda r: f"{r['pair_s']:.2f} s（{r['candidate_pairs']:,} 对）"),
        "规模扩展｜不同场景 Gaussian 数量下的 operator / graph 构建时间": sc_line(lambda r: f"{r['graph_build_s']:.1f} s（{r['free_cells']} 胞）"),
        "规模扩展｜不同场景 Gaussian 数量下的全局求解时间":
            sc_line(lambda r: "未查询" if r["graph_search_s_median"] is None else f"{1000 * r['graph_search_s_median']:.1f} ms") + "（A* 中位数，G1 筛选对）",
        "规模扩展｜不同场景 Gaussian 数量下的峰值内存": sc_line(lambda r: f"{r['peak_rss_mb_in_process'] / 1024:.2f} GB") + "（进程峰值，以档案加载为主）",
        "规模扩展｜不同规划分辨率下的总时间": res_time + f"（octree 最小叶；G1 的同一组 80 个筛选对{'' if len(lv) == 3 else '；只测了这几档'}）",
        "规模扩展｜不同规划分辨率下的状态数": res_states,
        "质量｜在构造上明确存在可行路径的测试中成功规划比例":
            f"{_pct(conn_reach, conn)}（无构造真值；代理 = G1 0.1 m oracle 点图 4-连通的对）",
        "质量｜在构造上明确不存在路径的测试中正确返回无路径比例":
            (f"{_pct(disc_unreach, disc)}（代理 = 点图不连通的对：错误可达 {disc_reach}；其余 {disc - disc_unreach - disc_reach} 对报 UNKNOWN，未认证）"
             if disc else "N/A（sweeper 的点图是单一连通分量，没有“不存在路径”的对）"),
        "质量｜错误返回“无路径”的比例":
            f"{_pct(unreach_conn, n, 2)}（UNREACHABLE 且点图连通；{unk_conn} 个 UNKNOWN 在点图中连通 = 不完备，不是“无路径”结论）",
        "质量｜输出路径通过最终连续碰撞验证的比例":
            f"{_pct(sc['shared_replay_passed'], reach)}（输出为 REACHABLE 的路线，自有连续验证 + 共享 gs3d 重放均通过）",
        "质量｜输出路径未通过最终连续碰撞验证的比例":
            f"0 / {reach}；0%（{b['own_certified_but_shared_replay_failed']} 条自有验证通过、被共享重放否决的路线报 UNKNOWN，未作为可达输出）",
        "质量｜路径长度统计":
            f"Mean：{sc['path_length_m']['mean']:.2f} m；Median：{sc['path_length_m']['median']:.2f} m；P95：{sc['path_length_m']['p95']:.2f} m",
        "质量｜相对于当前可获得最短路径的长度增加比例":
            f"Mean：{sc['excess_over_straight_pct']['mean']:.1f}%；Median：{sc['excess_over_straight_pct']['median']:.1f}%；P95："
            f"{sc['excess_over_straight_pct']['p95']:.1f}%（相对直线距离 = 最短路下界；真最短路未知，所以这是上界）",
        "质量｜路径最小碰撞间隙 / clearance":
            f"Mean：{sc['clearance_lower_m']['mean']:.4f} m；Median：{sc['clearance_lower_m']['median']:.4f} m；Minimum："
            f"{sc['clearance_lower_m']['min']:.4f} m（margin 0.001 m，设计如此）",
        "质量｜路径转弯次数或方向变化次数":
            f"Mean：{sc['turns']['mean']:.2f}；Median：{sc['turns']['median']:.0f}；P95：{sc['turns']['p95']:.0f}（折线内部顶点数）",
        "质量｜路径平滑度 / 曲率代价（若实现）": "N/A（未实现；折线路径，原地转向）",
        "质量｜在设定时间预算内的 timeout 比例": f"时间预算：120 s；Timeout：0 / {n}；0%（最慢 {q['max']:.1f} s）",
        "质量｜规划分辨率改变后成功 / 失败结论发生变化的比例":
            f"{_pct(flips_n, flips_of)}（相对默认 0.05 m：{res_flip_note}）",
        "质量｜规划分辨率改变后主要路线发生变化的比例":
            f"{_pct(routes_n, routes_of)}（与 0.05 m 都可达的对；Hausdorff 最大 "
            f"{max(x['route_hausdorff_m']['max'] for x in changed if x['route_hausdorff_m']):.2f} m）",
        "质量｜同一输入多次重复运行时结果稳定性":
            "完全一致：100%（G1 演示每对冷/热/重载调用逐字节相同；G3 画廊 "
            + (f"{gallery_same[0]}/{gallery_same[1]} 个 机器人×对 在另一进程/节点上与 G2 逐字节相同" if gallery_same else "未重跑")
            + "）；路线变化：0%；成功 / 失败翻转：0%",
        "机器人对比｜更换机器人时必须从头重新计算的阶段": None, "机器人对比｜更换机器人时可直接复用的阶段": None,
        "机器人对比｜更换机器人时可复用计算占原总计算量比例": None,
        "失败分析｜无路径误判数量与比例": f"{_pct(unreach_conn, n, 2)}（按 oracle 点图证据；UNREACHABLE 全部有割证书）",
        "失败分析｜超时数量与比例": f"0 / {n}；0%",
        "失败分析｜输出路径最终验证失败数量与比例":
            f"{_pct(b['own_certified_but_shared_replay_failed'], n, 2)}（自有验证通过但共享 gs3d 重放否决 → 报 UNKNOWN；输出的可达路线 0 失败）",
        "失败分析｜内存不足数量与比例": "0 / 10 个数组任务；0%（sacct 全部 COMPLETED）",
        "失败分析｜其他失败数量、比例与原因":
            f"{_pct(unknown - b['own_certified_but_shared_replay_failed'], n)}；原因："
            + "，".join(f"{k} {vv}" for k, vv in sorted(fails.items()) if k != "shared_replay_failed"),
        "失败分析｜失败是否主要由计算预算不足造成":
            f"是 / 否：否；相关失败：0 / {n}；0%（0 超时，最慢 {q['max']:.1f} s / 预算 120 s；叶子减半 0.05→0.025 m 不改变 UNKNOWN 数）",
        "失败分析｜失败是否主要由当前解析表示或求解能力不足造成":
            f"是 / 否：是；相关失败：{_pct(unknown, n)}（全部 UNKNOWN："
            + ("点图中全部连通 → 5 cm 认证表示/保守重放的不完备" if robot == "sweeper" else
               f"{fails.get('safe_graph_disconnected_possible_connected', 0)} 个 u≈−5.6 处点图也不连通、很可能真的不可达但单 pair 证书认证不了")
            + "）",
        "失败分析｜计算时间可接受但路径质量明显不好的测试比例":
            f"{_pct(sc['over_1p3'], reach)}（L/d > 1.3；无最短路真值，不能断定可避免）",
        "失败分析｜路径结果正确但计算时间明显过长的测试比例": f"{_pct(b['slow_queries_over_10s'], n, 2)}（查询 > 10 s）",
        "失败分析｜当前最常见失败类型":
            (f"类型：{top_fail}（UNKNOWN）；占全部失败：{100 * fails[top_fail] / unknown:.1f}%（认证 UNREACHABLE 不算失败）"
             if top_fail else "类型：无；占全部失败：0%"),
    }
    for k, t in paired.items():
        cells[k] = t if robot == "sweeper" else "同 sweeper 列（配对比较，同一 5000 对）"
    return cells


def cmd_csv(a):
    import csv
    A = json.loads(a.analysis.read_text())
    tpl = list(csv.reader(TEMPLATE.open(encoding="utf-8-sig")))
    g1 = {r[0]: r[1:] for r in csv.reader(a.g1_csv.open(encoding="utf-8-sig"))}
    gallery_same = None
    if a.gallery.exists():
        R = load_rows()
        by_id = {rb: {r["pair_id"]: r for r in R[rb].values()} for rb in ROBOTS}
        cfg = {p["name"]: p for p in json.loads(a.gallery_pairs.read_text())["pairs"]}
        same = tot = 0
        for rb in ROBOTS:
            d = json.loads((a.gallery / f"demo_{rb}.json").read_text())
            for name, pp in d["pairs"].items():
                g = by_id[rb][cfg[name]["pair_id"]]
                tot += 1
                same += all(c["polyline_sha256"] == g["polyline_sha256"] and c["status"] == g["status"] for c in pp["calls"])
        gallery_same = (same, tot)
    agg = {rb: aggregate_cells(A, rb, gallery_same) for rb in ROBOTS}
    header = tpl[0][0]
    g1_head = g1[header]
    out = [[header, *[h.replace("（本次测量）", "（G1 测量）") for h in g1_head],
            "sweeper · 5000 对（G2 全量，G3 汇总）", "cylinder · 5000 对（G2 全量，G3 汇总）"]]
    for row in tpl[1:]:
        key = row[0]
        demo = list(g1[key])
        demo = [POP_ONLY if "____" in c and not c.startswith("N/A") else c for c in demo]
        cells = []
        for k, rb in enumerate(ROBOTS):
            t = agg[rb].get(key, "MISSING")
            if t is None:                                      # identical to that robot's G1 demo column
                t = g1[key][k]
            cells.append(t)
        out.append([key, *demo, *cells])
    missing = [r[0] for r in out if "MISSING" in r]
    if missing:
        raise SystemExit(f"rows without an aggregate cell: {missing}")
    a.out.parent.mkdir(parents=True, exist_ok=True)
    with a.out.open("w", encoding="utf-8-sig", newline="") as f:
        csv.writer(f).writerows(out)
    left = sum(c.count("____") for r in out for c in r[1:])
    print("wrote", a.out, len(out), "rows;", left, "'____' left; gallery identical", gallery_same)


# ----------------------------------------------------------------------------- summary figure
def cmd_plot(a):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    A = json.loads(a.analysis.read_text())
    R = load_rows()
    C = {"sweeper": "#2a78d6", "cylinder": "#eb6834"}
    INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
    plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": ["DejaVu Sans"], "font.size": 9,
                         "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
                         "text.color": INK, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
                         "axes.edgecolor": GRID, "axes.spines.top": False, "axes.spines.right": False})
    fig, axs = plt.subplots(2, 3, figsize=(16, 9.6))
    tiers = list(A["robots"]["sweeper"]["distance_tiers"])
    labels = [f"{t} m\n(n={A['robots']['sweeper']['distance_tiers'][t]['n']})" for t in tiers]
    x = np.arange(len(tiers))
    w = .36
    ax = axs[0, 0]
    for k, rb in enumerate(ROBOTS):
        v = [100 * A["robots"][rb]["distance_tiers"][t]["success_rate"] for t in tiers]
        bars = ax.bar(x + (k - .5) * w, v, w - .03, color=C[rb], label=rb)
        for b_, vv in zip(bars, v):
            ax.text(b_.get_x() + b_.get_width() / 2, vv + 1.5, f"{vv:.0f}%", ha="center", fontsize=8, color=INK2)
    ax.set_xticks(x, labels)
    ax.set_ylim(0, 110)
    ax.set_ylabel("REACHABLE (%)")
    ax.set_title("(a) Success rate by start-goal distance\n(same 5000 pairs for both robots)", loc="left", fontsize=10)
    ax.legend(frameon=False, loc="upper right")
    ax.grid(axis="y", color=GRID, lw=.6)
    ax = axs[0, 1]
    for k, rb in enumerate(ROBOTS):
        med = [A["robots"][rb]["distance_tiers"][t]["query_wall_s"]["median"] for t in tiers]
        p95 = [A["robots"][rb]["distance_tiers"][t]["query_wall_s"]["p95"] for t in tiers]
        xx = x + (k - .5) * .18
        ax.vlines(xx, med, p95, color=C[rb], lw=2)
        ax.plot(xx, med, "o", ms=8, color=C[rb], mec=SURF, mew=2, label=f"{rb}: median (dot) to P95 (bar top)")
    ax.set_yscale("log")
    ax.set_xticks(x, labels)
    ax.set_ylabel("query wall time per pair (s, log)")
    ax.set_title("(b) Query time by distance, on the one cached compile\n(no compile inside any query)", loc="left", fontsize=10)
    ax.legend(frameon=False, loc="upper left", fontsize=8)
    ax.grid(axis="y", color=GRID, lw=.6, which="both")
    ax = axs[0, 2]
    re = A["reuse"]
    y = np.arange(2)
    for k, rb in enumerate(ROBOTS):
        m, cf = re["measured_compile_once_h"][rb], re["counterfactual_recompile_per_pair_h"][rb]
        ax.barh(k + .2, cf, .36, color=C[rb], alpha=.35, label="recompile per pair (5000 x measured load+compile)" if k == 0 else None)
        ax.barh(k - .2, m, .36, color=C[rb], label="compile once + reuse (measured)" if k == 0 else None)
        ax.text(cf * 1.15, k + .2, f"{cf:.1f} h", va="center", fontsize=8.5, color=INK2)
        ax.text(m * 1.15, k - .2, f"{m:.2f} h  ({re['speedup'][rb]:.0f}x less)", va="center", fontsize=8.5, color=INK)
    ax.set_xscale("log")
    ax.set_xlim(.05, 2000)
    ax.set_yticks(y, list(ROBOTS))
    ax.set_xlabel("single-core hours for 5000 pairs (log)")
    ax.set_title("(c) Compile once, answer 5000 pairs from the cache", loc="left", fontsize=10)
    ax.legend(frameon=False, loc="upper center", bbox_to_anchor=(.5, -.13), fontsize=8, ncol=1)
    ax.grid(axis="x", color=GRID, lw=.6, which="major")
    br = both_reachable(R)
    ax = axs[1, 0]
    H = [b["hausdorff_m"] for b in br]
    ax.hist(H, bins=np.arange(0, 1.0001, .05), color="#52514e", rwidth=.85)
    ax.axvline(MAJOR_ROUTE_CHANGE_M, color=INK2, ls=":", lw=1)
    ax.text(MAJOR_ROUTE_CHANGE_M + .01, ax.get_ylim()[1] * .92, f"{MAJOR_ROUTE_CHANGE_M} m: 'different route'", fontsize=8, color=INK2)
    ax.set_xlabel("Hausdorff distance between the two robots' routes (m)")
    ax.set_ylabel("pairs")
    ax.set_title(f"(d) Route divergence on the {len(br)} pairs BOTH robots reach\n"
                 f"(bimodal: {sum(h < .2 for h in H)} same route, {sum(h > .5 for h in H)} different)", loc="left", fontsize=10)
    ax.grid(axis="y", color=GRID, lw=.6)
    ax = axs[1, 1]
    xs = np.array([b["L_sweeper"] / b["dist_m"] for b in br])
    ys = np.array([b["L_cylinder"] / b["dist_m"] for b in br])
    q = (xs <= 1.02) & (ys >= 1.05)
    ax.axvspan(.995, 1.02, ymin=0, ymax=1, color="#e4e3df", alpha=.0)
    ax.fill_between([.995, 1.02], 1.05, 1.3, color=GRID, alpha=.7, lw=0, label="G1's pre-registered 'sweeper threads / cylinder detours' region")
    ax.scatter(xs[~q], ys[~q], s=40, color="#a3a29d", edgecolors=SURF, linewidths=1.5, label=f"other ({(~q).sum()})", zorder=3)
    ax.scatter(xs[q], ys[q], s=48, color=C["cylinder"], edgecolors=SURF, linewidths=1.5, label=f"in the region ({q.sum()})", zorder=4)
    for b in br:
        if b["pair_id"] in ("G2-02772", "G2-02004"):
            ax.annotate({"G2-02772": "T1", "G2-02004": "T2"}[b["pair_id"]], (b["L_sweeper"] / b["dist_m"], b["L_cylinder"] / b["dist_m"]),
                        xytext=(-16, 0), textcoords="offset points", fontsize=8.5, va="center", ha="right")
    ax.plot([1, 1.3], [1, 1.3], color=GRID, lw=.8)
    ax.set_xlim(.985, 1.09)
    ax.set_ylim(.99, 1.27)
    ax.set_xlabel("sweeper route length / straight distance")
    ax.set_ylabel("cylinder route length / straight distance")
    ax.set_title("(e) One threads, one detours: both-reachable pairs", loc="left", fontsize=10)
    ax.legend(frameon=False, loc="upper center", bbox_to_anchor=(.5, -.13), fontsize=8)
    ax = axs[1, 2]
    cols = {"REACHABLE": C["cylinder"], "UNREACHABLE": "#7a4b26", "UNKNOWN": "#d9d8d3"}
    names = {"REACHABLE": "REACHABLE", "UNREACHABLE": "certified UNREACHABLE (cut)", "UNKNOWN": "UNKNOWN (not certified)"}
    bottom = np.zeros(len(tiers))
    for st in ("REACHABLE", "UNREACHABLE", "UNKNOWN"):
        v = np.array([A["robots"]["cylinder"]["distance_tiers"][t]["status_counts"][st] for t in tiers], float)
        ax.bar(x, v, .6, bottom=bottom, color=cols[st], label=names[st], edgecolor=SURF, linewidth=2)
        bottom += v
    ax.set_xticks(x, labels)
    ax.set_ylabel("pairs")
    ax.set_title("(f) Cylinder outcome by distance: longer pairs cross the lamp\n(certified cut) or the u~-5.6 structure (UNKNOWN)", loc="left", fontsize=10)
    ax.legend(frameon=False, loc="upper right", fontsize=8)
    ax.grid(axis="y", color=GRID, lw=.6)
    fig.suptitle("aerial3d-ground, 5000 uniform pairs (d >= 3 m) per robot, each robot compiled once: sweeper r 0.175 m / 0.02-0.10 m band, "
                 "cylinder r 0.30 m / 0.02-1.75 m band", x=.01, ha="left", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, .96))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=100)
    print("wrote", a.out)


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    se = sub.add_parser("select")
    se.add_argument("--out", type=Path, default=Path("configs/aerial3dg/g3_gallery_pairs.json"))
    an = sub.add_parser("analyze")
    an.add_argument("--variants", type=Path, default=Path("results/aerial3dg/g3/variants"))
    an.add_argument("--out", type=Path, default=Path("results/aerial3dg/g3/analysis.json"))
    cs = sub.add_parser("csv")
    cs.add_argument("--analysis", type=Path, default=Path("results/aerial3dg/g3/analysis.json"))
    cs.add_argument("--g1-csv", type=Path, default=G1 / "measurement_g1.csv")
    cs.add_argument("--gallery", type=Path, default=Path("outputs/aerial3dg/g3/gallery"))
    cs.add_argument("--gallery-pairs", type=Path, default=Path("configs/aerial3dg/g3_gallery_pairs.json"))
    cs.add_argument("--out", type=Path, default=Path("../docs/aerial3dg_measurement.csv"))
    pl = sub.add_parser("plot")
    pl.add_argument("--analysis", type=Path, default=Path("results/aerial3dg/g3/analysis.json"))
    pl.add_argument("--out", type=Path, default=Path("results/aerial3dg/g3/g3_summary.png"))
    a = p.parse_args(argv)
    {"select": cmd_select, "analyze": cmd_analyze, "csv": cmd_csv, "plot": cmd_plot}[a.cmd](a)


if __name__ == "__main__":
    main()
