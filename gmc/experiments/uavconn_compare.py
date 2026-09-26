"""C2 comparison of aerial3d vs the uav-lamp lattice baseline: success, smoothness, time.

Reads only result JSONs (no archive): aerial3d runs (``uavconn_run.py``), lattice runs
(``uavlamp_run.py``: L2's recorded benchmark runs, C2's M1 re-time and extended-set runs),
the shared smoothing rows (``uavconn_smooth.py``) and, for lattice jobs that left no
result, Slurm's job state (a TIMEOUT is a failure-by-budget).  Ground truth and success
follow ``docs/uavconn_c2_prereg.md`` §3.  Writes ``comparison.json`` and the time /
smoothness figures.

Usage (from ``gmc/``)::

    python experiments/uavconn_compare.py --out results/uavconn
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess

import numpy as np

from gmc.aerial3d.metrics import polyline_metrics

C2 = Path("outputs/uavconn/c2")
L2 = Path("/scratch/wg2381/splathjb-uavlamp/gmc/outputs/uavlamp/l2")
JOBIDS = Path("/scratch/wg2381/claude_jobs/uavconn/jobids/C2.txt")
BENCH = [  # (label, L2 spec name, aerial3d group)
    ("M1", "M1_main_low_start", "booth"), ("M5", "M5_high_start", "booth"),
    ("N2", "N2_plug_low_start", "plug"), ("N2h", "N2h_plug_high_start", "plug"),
    ("C4h", "C4h_lamp_only_high_start", "lamp")]
EXHAUSTED = {("no_path_on_lattice", "reachable_lattice_exhausted"),
             ("map_unknown", "reachable_frontier_meets_unknown_coverage"),
             ("verification_failed", "reachable_frontier_has_unproven_edges")}
BLUE, ORANGE = "#2a78d6", "#eb6834"      # categorical slots 1-2 (dataviz reference palette)
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def load(path):
    p = Path(path)
    return json.loads(p.read_text()) if p.exists() else None


def wilson(k: int, n: int, z: float = 1.96):
    if n == 0:
        return None
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return [max(0., c - h), min(1., c + h)]


def turning_vertices(poly, eps=1e-6) -> int:
    P = np.asarray(poly, float)
    if len(P) > 1:
        P = P[np.r_[True, np.linalg.norm(np.diff(P, axis=0), axis=1) > 1e-10]]
    if len(P) < 3:
        return 0
    u = np.diff(P, axis=0)
    u /= np.linalg.norm(u, axis=1)[:, None]
    th = np.arccos(np.clip(np.sum(u[:-1] * u[1:], axis=1), -1, 1))
    return int(np.count_nonzero(th > eps))


def raw_block(poly) -> dict:
    m = polyline_metrics(poly)
    m["n_turning_vertices"] = turning_vertices(poly)
    return m


def slurm_states() -> dict:
    """uc_base_<name> -> (job id, sacct state) from this stage's job-id file."""
    out = {}
    if not JOBIDS.exists():
        return out
    for line in JOBIDS.read_text().splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[1].startswith("uc_base_"):
            out[parts[1][len("uc_base_"):]] = parts[0]
    states = {}
    for name, jid in out.items():
        try:
            st = subprocess.run(["sacct", "-j", jid, "-n", "-X", "-o", "State,Elapsed"], capture_output=True,
                                text=True, timeout=30).stdout.split()
        except Exception:
            st = []
        states[name] = {"job_id": jid, "state": st[0] if st else None, "elapsed": st[1] if len(st) > 1 else None}
    return states


def aerial3d_row(doc) -> dict:
    if doc is None:
        return {"outcome": "not_run"}
    r, rep = doc["result"], doc.get("replay") or {}
    st = doc["status"]
    if st == "REACHABLE":
        outcome = "path" if rep.get("passed") else "replay_failed"
    elif st == "UNREACHABLE":
        outcome = "no_path_certified"
    else:
        outcome = "unknown"
    t = doc["timing"]
    row = {"outcome": outcome, "status": st, "reason": doc["reason"],
           "compile_id": doc["compile"]["compile_id"],
           "time": {"compile_s": t["compile_wall_s"], "cold_s": t["cold_algorithm_wall_s"][0],
                    "warm_s": t["warm_algorithm_wall_s"],
                    "warm_median_s": float(np.median(t["warm_algorithm_wall_s"])) if t["warm_algorithm_wall_s"] else None,
                    "time_to_first_path_s": t["time_to_first_path_s"],
                    "archive_load_and_hash_s": t["archive_load_and_hash_s"],
                    "cold_stages": doc["all_calls"][0]["stages"]},
           "identical_routes_across_calls": doc["identical_routes_across_calls"],
           "host": {k: doc["host"].get(k) for k in ("node", "slurm_job_id", "cpus_per_task", "CPUAlloc", "CPUTot",
                                                     "node_shared", "loadavg")}}
    if outcome in ("path", "replay_failed"):
        ev = doc["evidence"]
        row["raw"] = raw_block(r["polyline_world"])
        row["replay"] = {"passed": rep.get("passed"), "clearance_lower_m": ev["clearance_lower_m_replay"]}
        row["evidence"] = {k: ev[k] for k in ("z_range", "passes_under_lamp", "under_precedes_above_table",
                                              "path_length_m")}
        row["evidence"]["under_lamp"] = [(i["s_from_m"], i["s_to_m"]) for i in ev["under_lamp_intervals"]]
        row["evidence"]["above_table"] = [(i["s_from_m"], i["s_to_m"]) for i in ev["above_table_intervals"]]
        row["evidence"]["over_lamp_samples"] = sum(i["sample_to"] - i["sample_from"] + 1
                                                   for i in ev["over_lamp_intervals"])
    if doc.get("certificate_attribution"):
        cert = r["certificate"]
        row["certificate"] = {"kind": cert["kind"], "blocked_leaves_on_cut": cert["blocked_leaves_on_cut"],
                              "cut_distinct_pairs": cert["cut_distinct_pairs"],
                              "cut_traceable_fraction": cert["cut_traceable_fraction"],
                              "cut_pairs_by_role": doc["certificate_attribution"]["cut_pairs_by_role"]}
    return row


def lattice_row(doc, slurm=None, *, source="") -> dict:
    if doc is None:
        state = (slurm or {}).get("state")
        if state is None or state in ("PENDING", "RUNNING"):
            return {"outcome": "not_run", "slurm": slurm, "source": source}
        return {"outcome": "timeout" if state == "TIMEOUT" else "job_failed", "slurm": slurm, "source": source}
    r, rep = doc["result"], doc.get("replay") or {}
    key = (r["status"], r["reason"])
    if r["status"] == "success":
        outcome = "path" if rep.get("passed") else "replay_failed"
    elif key in EXHAUSTED:
        outcome = "exhausted"
    elif r["status"] == "budget_exhausted":
        outcome = "budget"
    else:
        outcome = "other"
    t = doc["timing"]
    calls = doc["all_calls"]
    prim = next(c for c in calls if c["call_id"] == doc["primary_call"])
    row = {"outcome": outcome, "status": r["status"], "reason": r["reason"], "source": source,
           "expansions": r["diagnostics"]["expansions"], "oracle_calls": r["diagnostics"].get("oracle_calls"),
           "time": {"algorithm_s": prim["algorithm_wall_s"], "index_preparation_s": t["index_preparation_wall_s"],
                    "time_to_first_path_s": prim["algorithm_wall_s"] + (t["index_preparation_wall_s"]
                                                                        if prim["mode"] == "warm" else 0.),
                    "all_calls_s": [c["algorithm_wall_s"] for c in calls],
                    "calls_ran_concurrently": t.get("calls_ran_concurrently")},
           "host": doc.get("host"), "slurm": slurm}
    if outcome in ("path", "replay_failed"):
        ev = doc["evidence"]
        row["raw"] = raw_block(np.asarray(r["trajectory"]["poses"], float)[:, :3])
        row["replay"] = {"passed": rep.get("passed"), "clearance_lower_m": ev["clearance_lower_m_replay"]}
        row["evidence"] = {k: ev[k] for k in ("z_range", "passes_under_lamp", "under_precedes_above_table",
                                              "path_length_m")}
        row["evidence"]["under_lamp"] = [(i["s_from_m"], i["s_to_m"]) for i in ev["under_lamp_intervals"]]
        row["evidence"]["above_table"] = [(i["s_from_m"], i["s_to_m"]) for i in ev["above_table_intervals"]]
        row["evidence"]["over_lamp_samples"] = sum(i["sample_to"] - i["sample_from"] + 1
                                                   for i in ev["over_lamp_intervals"])
    return row


def smooth_block(path) -> dict | None:
    s = load(path)
    if s is None or "skipped" in s:
        return None
    sm = s.get("smoothed") or {}
    g = (s.get("gs3d_metrics") or {}).get("smoothed_candidate") or {}
    return {"selected": s["selected"], "reason": s["reason"],
            "fresh_reverification_passed": (s.get("fresh_reverification") or {}).get("passed"),
            "fresh_reverification_clearance_m": (s.get("fresh_reverification") or {}).get("clearance_lower_m"),
            "integrated_squared_jerk": sm.get("integrated_squared_jerk"), "duration_s": sm.get("duration_s"),
            "smooth_path_length_m": sm.get("smooth_path_length_m"), "bezier_segments": sm.get("bezier_segments"),
            "anchor_rows": s.get("anchor_rows"), "gs3d_turn_metric_rad": g.get("turn_metric_rad"),
            "sampled_curvature_max_radpm": g.get("sampled_curvature_max_radpm"),
            "gs3d_raw_turn_metric_rad": ((s.get("gs3d_metrics") or {}).get("raw") or {}).get("turn_metric_rad"),
            "turn_improvement_fraction": (s.get("gs3d_metrics") or {}).get("turn_improvement_fraction"),
            "jerk_dimensionless": (sm["integrated_squared_jerk"] * sm["duration_s"] ** 5 / sm["smooth_path_length_m"] ** 2
                                   if sm.get("integrated_squared_jerk") is not None else None),
            "optimizer_wall_s": s.get("optimizer_wall_s"),
            "passes_under_lamp": (s.get("evidence") or {}).get("passes_under_lamp"),
            "under_precedes_above_table": (s.get("evidence") or {}).get("under_precedes_above_table")}


def judge(row, design_gt=None):
    a, l = row["aerial3d"], row["lattice"]
    has_path = any(m["outcome"] == "path" for m in (a, l))
    row["ground_truth"] = "PATH" if has_path else "NO_PATH_EVIDENCE"
    row["design_expectation"] = design_gt
    for m, no_path in ((a, "no_path_certified"), (l, "exhausted")):
        if has_path:
            m["success"] = m["outcome"] == "path"
        else:
            m["success"] = m["outcome"] == no_path
        m["contradiction"] = m["outcome"] == "replay_failed" or (has_path and m["outcome"] == "no_path_certified")
    return row


def rate(rows, method) -> dict:
    ms = [r[method] for r in rows]
    n = len(ms)
    k = sum(bool(m.get("success")) for m in ms)
    by = {}
    for m in ms:
        by[m["outcome"]] = by.get(m["outcome"], 0) + 1
    return {"n": n, "success": k, "rate": k / n if n else None, "wilson95": wilson(k, n), "by_outcome": by,
            "contradictions": sum(bool(m.get("contradiction")) for m in ms)}


def summary_stats(vals):
    v = np.asarray([x for x in vals if x is not None], float)
    if not len(v):
        return None
    q1, med, q3 = np.percentile(v, [25, 50, 75])
    return {"n": int(len(v)), "median": float(med), "q1": float(q1), "q3": float(q3), "min": float(v.min()),
            "max": float(v.max())}


RAW_KEYS = ("path_length_m", "n_vertices", "n_turning_vertices", "total_turning_rad", "max_turning_rad",
            "bending_energy_per_m", "vertical_travel_m")
SM_KEYS = ("integrated_squared_jerk", "jerk_dimensionless", "duration_s", "smooth_path_length_m",
           "gs3d_turn_metric_rad", "sampled_curvature_max_radpm")


def smoothness(rows) -> dict:
    both = [r for r in rows if r["aerial3d"]["outcome"] == "path" and r["lattice"]["outcome"] == "path"]
    out = {"queries_where_both_have_paths": [r["query"] for r in both], "raw": {}, "smoothed": {}, "paired": {}}
    for k in RAW_KEYS:
        out["raw"][k] = {m: summary_stats([r[m]["raw"][k] for r in both]) for m in ("aerial3d", "lattice")}
        ratios = [r["aerial3d"]["raw"][k] / r["lattice"]["raw"][k] for r in both if r["lattice"]["raw"][k]]
        out["paired"][k] = {"aerial3d_over_lattice": summary_stats(ratios),
                            "aerial3d_lower_count": sum(r["aerial3d"]["raw"][k] < r["lattice"]["raw"][k] - 1e-12
                                                        for r in both),
                            "aerial3d_leq_count": sum(r["aerial3d"]["raw"][k] <= r["lattice"]["raw"][k] + 1e-9
                                                      for r in both), "n": len(both)}
    smb = [r for r in both if (r["aerial3d"].get("smooth") or {}).get("fresh_reverification_passed")
           and (r["lattice"].get("smooth") or {}).get("fresh_reverification_passed")]
    out["queries_where_both_smoothed_and_reverified"] = [r["query"] for r in smb]
    for k in SM_KEYS:
        out["smoothed"][k] = {m: summary_stats([r[m]["smooth"][k] for r in smb]) for m in ("aerial3d", "lattice")}
        out["paired"]["smoothed_" + k] = {
            "aerial3d_lower_count": sum((r["aerial3d"]["smooth"][k] or 0) < (r["lattice"]["smooth"][k] or 0) - 1e-12
                                        for r in smb),
            "aerial3d_leq_count": sum((r["aerial3d"]["smooth"][k] or 0) <= (r["lattice"]["smooth"][k] or 0) + 1e-9
                                      for r in smb), "n": len(smb)}
    return out


def timing(rows, compiles) -> dict:
    per = []
    for r in rows:
        a, l = r["aerial3d"], r.get("lattice_retime") or r["lattice"]
        if "time" not in a or "time" not in l:
            continue
        la = l["time"]["algorithm_s"]
        per.append({"query": r["query"], "set": r["set"], "lattice_s": la, "lattice_source": l.get("source"),
                    "aerial3d_cold_s": a["time"]["cold_s"], "aerial3d_warm_median_s": a["time"]["warm_median_s"],
                    "aerial3d_compile_s": a["time"]["compile_s"],
                    "speedup_cold": la / a["time"]["cold_s"],
                    "speedup_warm": la / a["time"]["warm_median_s"] if a["time"]["warm_median_s"] else None,
                    "speedup_time_to_first_path": l["time"]["time_to_first_path_s"] / a["time"]["time_to_first_path_s"],
                    "lattice_time_to_first_path_s": l["time"]["time_to_first_path_s"],
                    "aerial3d_time_to_first_path_s": a["time"]["time_to_first_path_s"],
                    "lattice_outcome": l["outcome"], "aerial3d_outcome": a["outcome"]})
    return {"per_query": per, "compiles": compiles,
            "definition": "aerial3d cold = first query call in a process forked from the fresh compile; warm = "
                          "median of 3 further calls; both include the in-query gs3d replay. Lattice = its single "
                          "plan() call's algorithm_wall_s (index prepared separately, <1 s). time-to-first-path: "
                          "aerial3d compile + cold query; lattice index preparation + call. Speedup = lattice / "
                          "aerial3d."}


def build(out: Path) -> dict:
    slurm = slurm_states()
    rows = []
    for label, name, group in BENCH:
        a = load(C2 / "aerial3d" / group / f"l2_{name}" / "result.json")
        l = load(L2 / name / "result.json")
        row = {"query": label, "set": "benchmark", "spec": f"configs/uavlamp_l2/{name}.json",
               "aerial3d": aerial3d_row(a),
               "lattice": lattice_row(l, source=f"L2 recorded run (contended node), {L2 / name / 'result.json'}")}
        if label == "M1":
            rt = load(C2 / "baseline" / "M1_retime" / "result.json")
            row["lattice_retime"] = lattice_row(rt, slurm.get("M1"), source="C2 re-time, own 2-CPU job")
        judge(row, "PATH" if label in ("M1", "M5", "C4h") else "NO_PATH (plug closes the only opening)")
        rows.append(row)
    pairs = load("results/uavconn/extended/pairs.json")
    for p in (pairs or {}).get("pairs", []):
        n = p["name"]
        a = load(C2 / "aerial3d" / "ext" / f"c2_ext_{n}" / "result.json")
        l = load(C2 / "baseline" / "ext" / n / "result.json")
        row = {"query": n, "set": "extended", "class": p["class"], "spec": f"configs/uavconn_c2/ext/{n}.json",
               "start_route": p["start_route"], "goal_route": p["goal_route"], "distance_m": p["distance_m"],
               "straight_segment_oracle": p["straight_segment_oracle"]["occupancy"],
               "crosses_lamp_plane": p["crosses_lamp_plane"],
               "aerial3d": aerial3d_row(a), "lattice": lattice_row(l, slurm.get(n), source="C2 extended run")}
        rows.append(judge(row))
    # shared smoothing rows
    for r in rows:
        grp = "ext" if r["set"] == "extended" else ("lamp" if r["query"] == "C4h" else "booth")
        for m in ("aerial3d", "lattice"):
            if r[m]["outcome"] == "path":
                r[m]["smooth"] = smooth_block(C2 / "smooth" / grp / f"{m}__{r['query']}.json")
    compiles = {}
    for group in ("booth", "plug", "lamp", "ext"):
        c = load(C2 / "aerial3d" / group / "compile.json")
        if c:
            comp = c["compile"]
            compiles[group] = {"compile_wall_s": comp["timings"]["compile_wall_s"],
                               "stages": {r["stage"]: r["seconds"] for r in comp["timings"]["records"]},
                               "peak_rss_mb": comp["timings"]["peak_rss_mb"], "compile_cpu_s": c["compile_cpu_s"],
                               "candidate_pairs": comp["pairs"]["candidate_pairs"],
                               "leaves_by_status": comp["octree"]["leaves_by_status"],
                               "cells": comp["cells"]["cells"], "portals": comp["cells"]["portals"],
                               "traceability": comp["traceability"],
                               "free_boundary_traceability": comp["free_boundary_traceability"],
                               "possible_components": comp["possible_components"],
                               "sandwich_audit_passed": comp["audit"].get("passed"),
                               "host": {k: c["host_at_start"].get(k) for k in ("node", "slurm_job_id", "CPUAlloc",
                                                                               "CPUTot", "node_shared", "loadavg")},
                               "queries_served": len(c["specs"])}
    bench = [r for r in rows if r["set"] == "benchmark"]
    ext = [r for r in rows if r["set"] == "extended"]
    success = {}
    for tag, sub in (("benchmark", bench), ("extended", ext), ("extended_A", [r for r in ext if r["class"] == "A"]),
                     ("extended_B", [r for r in ext if r["class"] == "B"]), ("all", rows)):
        success[tag] = {m: rate(sub, m) for m in ("aerial3d", "lattice")}
    doc = {"schema": "uavconn.c2_comparison.v1",
           "methods": {"aerial3d": "gmc.aerial3d pair-certified support-plane cell complex (uav.md U0), default "
                                   "CompileConfig/QueryConfig, one compile per scene variant, query(start, goal)",
                       "lattice": "gs3d.planner.LatticePlanner 26-neighbour xyz A* 0.10 m via unchanged "
                                  "uavlamp_run.py, one plan() call"},
           "shared_final_check": "gs3d.trajectory.replay_plan with a fresh GaussianBodyOracle on a freshly built "
                                 "PreparedScene, same archive / body / margin / tau / level for both",
           "success_definition": "docs/uavconn_c2_prereg.md §3",
           "success": success, "rows": rows,
           "smoothness": {"all": smoothness(rows), "benchmark": smoothness(bench), "extended": smoothness(ext)},
           "time": timing(rows, compiles)}
    out.mkdir(parents=True, exist_ok=True)
    (out / "comparison.json").write_text(json.dumps(doc, indent=1, allow_nan=False, default=float) + "\n")
    return doc


# ----------------------------------------------------------------------------- figures

def _style(ax):
    ax.grid(axis="x", color=GRID, lw=.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelcolor=INK)


def fig_time(doc, dst):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    per = doc["time"]["per_query"]
    if not per:
        return
    fig, ax = plt.subplots(figsize=(10, .32 * len(per) + 1.8))
    y = np.arange(len(per))[::-1]
    for yy, p in zip(y, per):
        ax.plot([p["aerial3d_cold_s"], p["lattice_s"]], [yy, yy], color=GRID, lw=2, zorder=1)
    a_ok = [p["aerial3d_outcome"] in ("path", "no_path_certified") for p in per]
    ax.scatter([p["aerial3d_cold_s"] for p, o in zip(per, a_ok) if o], [yy for yy, o in zip(y, a_ok) if o], s=46,
               color=BLUE, edgecolors="white", linewidths=1.5, zorder=3, label="aerial3d: one query (cold)")
    if not all(a_ok):
        ax.scatter([p["aerial3d_cold_s"] for p, o in zip(per, a_ok) if not o], [yy for yy, o in zip(y, a_ok) if not o],
                   s=46, facecolors="white", edgecolors=BLUE, linewidths=1.8, zorder=3,
                   label="aerial3d: answered UNKNOWN (shared replay rejected a long segment)")
    lat_fail = [p["lattice_outcome"] not in ("path", "exhausted") for p in per]
    ax.scatter([p["lattice_s"] for p, f in zip(per, lat_fail) if not f], [yy for yy, f in zip(y, lat_fail) if not f],
               s=46, color=ORANGE, edgecolors="white", linewidths=1.5, zorder=3, label="lattice: one query")
    if any(lat_fail):
        ax.scatter([p["lattice_s"] for p, f in zip(per, lat_fail) if f], [yy for yy, f in zip(y, lat_fail) if f],
                   s=46, facecolors="white", edgecolors=ORANGE, linewidths=1.8, zorder=3,
                   label="lattice: stopped by budget (no answer)")
    names = {"booth": "booth", "plug": "booth+plug", "lamp": "lamp only", "ext": "booth, ext. job"}
    comps = sorted(doc["time"]["compiles"].items(), key=lambda kv: kv[1]["compile_wall_s"])
    for g, c in comps:
        ax.axvline(c["compile_wall_s"], color=BLUE, lw=1, ls=(0, (4, 3)), alpha=.6, zorder=0)
    note = " · ".join(f"{names.get(g, g)} {c['compile_wall_s']:.0f} s" for g, c in comps)
    ax.set_yticks(y)
    ax.set_yticklabels([f"{p['query']}" + ("" if p["set"] == "benchmark" else "") for p in per], fontsize=8)
    ax.set_xscale("log")
    ax.set_xlabel("wall-clock seconds per query (log scale)\n"
                  f"dashed: aerial3d one-off compile per scene, reused by all its queries ({note})",
                  color=INK, fontsize=9)
    _style(ax)
    ax.legend(loc="upper left", fontsize=8, frameon=True, framealpha=.95, edgecolor="none")
    nb = sum(p["set"] == "benchmark" for p in per)
    if 0 < nb < len(per):
        ax.axhline(y[nb - 1] - .5, color=MUTED, lw=.6)
    ax.set_title("Per-query time: aerial3d (new) vs lattice A* (uav-lamp baseline), same queries",
                 fontsize=10, color=INK, loc="left")
    fig.tight_layout()
    fig.savefig(dst, dpi=120)
    plt.close(fig)


def fig_smooth(doc, dst):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = [r for r in doc["rows"] if r["aerial3d"]["outcome"] == "path" and r["lattice"]["outcome"] == "path"]
    if not rows:
        return
    panels = [("path_length_m", "raw path length (m)", "raw", False),
              ("n_turning_vertices", "turning vertices (raw)", "raw", False),
              ("total_turning_rad", "total turning (rad, raw)", "raw", False),
              ("sampled_curvature_max_radpm", "sampled max curvature (1/m)\nafter the same A6 smoothing (log)",
               "smooth", True),
              ("integrated_squared_jerk", "∫|jerk|² dt after the same\nA6 smoothing (log)", "smooth", True)]
    fig, axes = plt.subplots(1, len(panels), figsize=(16, 3.9))
    rng = np.random.default_rng(0)
    for ax, (k, lab, blk, log) in zip(axes, panels):
        vals = []
        for m in ("aerial3d", "lattice"):
            v = [(r[m].get(blk) or {}).get(k) for r in rows]
            vals.append([x for x in v if x is not None])
        bp = ax.boxplot(vals, positions=[0, 1], widths=.45, patch_artist=True, showfliers=False,
                        medianprops=dict(color=INK, lw=1.6), whiskerprops=dict(color=MUTED),
                        capprops=dict(color=MUTED))
        for patch, c in zip(bp["boxes"], (BLUE, ORANGE)):
            patch.set_facecolor(c + "40"); patch.set_edgecolor(c)
        for i, (v, c) in enumerate(zip(vals, (BLUE, ORANGE))):
            ax.scatter(i + rng.uniform(-.12, .12, len(v)), v, s=22, color=c, edgecolors="white", linewidths=1,
                       zorder=3)
        # pair lines (same query)
        both = [((r["aerial3d"].get(blk) or {}).get(k), (r["lattice"].get(blk) or {}).get(k)) for r in rows]
        for a, b in both:
            if a is not None and b is not None:
                ax.plot([0, 1], [a, b], color=GRID, lw=.8, zorder=1)
        ax.set_xticks([0, 1]); ax.set_xticklabels(["aerial3d", "lattice"])
        if log and any(x > 0 for v in vals for x in v):
            ax.set_yscale("log")
        ax.set_title(lab, fontsize=9, color=INK, loc="left")
        ax.grid(axis="y", color=GRID, lw=.8); ax.set_axisbelow(True)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    fig.suptitle(f"Smoothness on the {len(rows)} queries where both methods returned a verified path "
                 "(grey lines join the same query)", fontsize=10, x=.01, ha="left", color=INK)
    fig.tight_layout()
    fig.savefig(dst, dpi=120)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("results/uavconn"))
    a = ap.parse_args(argv)
    doc = build(a.out)
    (a.out / "final").mkdir(parents=True, exist_ok=True)
    fig_time(doc, a.out / "final" / "time_per_query.png")
    fig_smooth(doc, a.out / "final" / "smoothness.png")
    print(json.dumps(doc["success"], indent=1, default=float))


if __name__ == "__main__":
    main()
