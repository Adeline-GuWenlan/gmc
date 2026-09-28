"""G2: the 5000-pair run for the ground bodies, one compile per robot (compile once, answer every pair).

Subcommands (run from ``gmc/``, ``PYTHONPATH=src:experiments``):

* ``sample``   seeded pair list on the route box: endpoints drawn uniformly (continuous, 0.1 mm
               rounding), straight-line xy distance >= 3 m, each endpoint certified free by the gs3d
               oracle for EVERY robot at that robot's own z_c.  No connectivity filter: the planner
               decides reachability and UNREACHABLE/UNKNOWN rates are reported as they fall.
* ``compile``  compile one robot once on the box and persist it (``save_compiled`` -> .a3c).
* ``task``     one array task: ``load_compiled`` once, answer a contiguous slice of the pair list,
               append one JSON line per pair (fsync'd) so a killed task resumes mid-slice.
* ``collect``  aggregate every task file into per-robot counts / timing and the paired comparison.

Design and choices: docs/aerial3dg_g2.md.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import signal
import time

import numpy as np

import gmc.aerial3d.api as a3api
from gmc.aerial3d.api import load_compiled, query, save_compiled
from gmc.gs3d.planner import _json_finite

from aerial3dg_run import (GROUND_CONFIG, MANIFEST, QCONFIG, ROBOTS, _box, _dump, _sha, brief, compile_record,
                           host, role_of, rss_mb)

STATUSES = ("REACHABLE", "UNREACHABLE", "UNKNOWN", "TIMEOUT")


# ----------------------------------------------------------------------------- sampling
def sample_pairs(n, *, seed, box_uv, is_free, robots, min_dist=3.0):
    """``n`` i.i.d. pairs, uniform over the route box ``(u0, v0, u1, v1)`` conditioned on the rules.

    A draw is (start, goal) both uniform; it is rejected whole, first if the xy distance is below
    ``min_dist`` (cheap), then if any endpoint is not free for any robot (``is_free(name, uv) ->
    (ok, clearance)``).  Rejecting whole draws keeps the accepted pairs i.i.d. from the conditional
    law, so any prefix -- and any contiguous array slice -- is itself an unbiased sample."""
    rng = np.random.default_rng(seed)
    lo, hi = np.asarray(box_uv[:2], float), np.asarray(box_uv[2:], float)
    counts = {"pair_draws": 0, "dist_below_min": 0, **{f"endpoint_not_free_{r}": 0 for r in robots},
              "accepted": 0}
    pairs = []
    while len(pairs) < n:
        counts["pair_draws"] += 1
        s = np.clip(rng.uniform(lo, hi).round(4), lo, hi)
        g = np.clip(rng.uniform(lo, hi).round(4), lo, hi)
        d = float(np.hypot(*(g - s)))
        if d < min_dist:
            counts["dist_below_min"] += 1
            continue
        clear = {}
        for r in robots:
            cs, cg = is_free(r, s), None
            if cs[0]:
                cg = is_free(r, g)
            if not cs[0] or not cg[0]:
                counts[f"endpoint_not_free_{r}"] += 1
                clear = None
                break
            clear[r] = {"start": cs[1], "goal": cg[1]}
        if clear is None:
            continue
        k = len(pairs)
        pairs.append({"index": k, "pair_id": f"G2-{k:05d}", "start_uv": s.tolist(), "goal_uv": g.tolist(),
                      "dist_m": d, "clearance_m": clear})
        counts["accepted"] += 1
    return pairs, counts


def task_slice(n, n_tasks, task) -> range:
    """Contiguous chunk ``task`` of ``n_tasks`` near-equal chunks of ``range(n)``."""
    return range(n * task // n_tasks, n * (task + 1) // n_tasks)


# ----------------------------------------------------------------------------- checkpoint
def read_checkpoint(path: Path) -> dict:
    """{index: row} of a task file; a torn last line (killed mid-write) is cut off the file."""
    path = Path(path)
    if not path.exists():
        return {}
    done, good = {}, 0
    raw = path.read_bytes()
    for line in raw.splitlines(keepends=True):
        try:
            row = json.loads(line)
            if not line.endswith(b"\n"):
                raise ValueError("unterminated")
        except ValueError:
            break
        done[int(row["index"])] = row
        good += len(line)
    if good != len(raw):
        with open(path, "r+b") as f:
            f.truncate(good)
    return done


class _Timeout(Exception):
    pass


@contextmanager
def _alarm(seconds):
    def fire(*_):
        raise _Timeout()
    old = signal.signal(signal.SIGALRM, fire)
    signal.setitimer(signal.ITIMER_REAL, float(seconds))
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0.)
        signal.signal(signal.SIGALRM, old)


class _RouteFrame:
    """aerial3d's plan frame IS the route frame (G1: start_plan == start_uv); ``brief`` wants to_route."""

    def __init__(self, frame):
        self.to_route = frame.to_plan


def _row(p, r, compiled, compile_stages, wall, cpu, manifest):
    route = _RouteFrame(compiled.frame)
    b = brief(r, route)
    ver = r.get("verification") or {}
    cert = r.get("certificate") or {}
    roles = None
    if cert.get("cut_pair_ids") and manifest is not None:
        roles = {}
        for pid in cert["cut_pair_ids"]:
            k = role_of(int(pid), manifest)
            roles[k] = roles.get(k, 0) + 1
    stages = {t["stage"] for t in r["timings"]["records"]}
    return {"index": p["index"], "pair_id": p["pair_id"], "start_uv": p["start_uv"], "goal_uv": p["goal_uv"],
            "dist_m": p["dist_m"], "compile_id": r["compile_id"], **b,
            "own_verification": (ver.get("own") or {}).get("status"),
            "shared_replay_passed": (ver.get("shared") or {}).get("passed"),
            "cut_pairs_by_role": roles,
            "endpoint_certificate_pair": cert.get("pair_id") if cert.get("kind") == "endpoint_inside_inner_polytope" else None,
            "outer_wall_s": wall, "cpu_s": cpu, "compile_stages_in_call": sorted(compile_stages & stages),
            "polyline_sha256": _sha(r.get("polyline_world"))}


def run_task(a3c, pairs, out_jsonl, *, timeout_s=120., manifest=None, qconfig=QCONFIG) -> dict:
    """Load the persisted compile once and answer every pair not yet in ``out_jsonl``."""
    out_jsonl = Path(out_jsonl)
    out_jsonl.parent.mkdir(parents=True, exist_ok=True)
    done = read_checkpoint(out_jsonl)
    todo = [p for p in pairs if p["index"] not in done]
    compiles = {"n": 0}
    real_compile = a3api.compile_complex

    def counted(*a, **k):                     # any compile inside this task would show up here
        compiles["n"] += 1
        return real_compile(*a, **k)
    a3api.compile_complex = counted
    try:
        t0 = time.perf_counter()
        compiled = load_compiled(a3c)
        load_s = time.perf_counter() - t0
        body = compiled.body
        z_c = body.ground_clearance_m + body.half_height_m
        if compiled.domain.ground_z is None or abs(compiled.domain.ground_z - z_c) > 1e-9:
            raise ValueError(f"compiled ground z {compiled.domain.ground_z} is not the body centre height {z_c}")
        before = json.dumps(compiled.timings["records"], sort_keys=True, default=float)
        compile_stages = {x["stage"] for x in compiled.timings["records"]}
        rows = []
        with open(out_jsonl, "a") as f:
            for p in todo:
                s = compiled.frame.to_world([p["start_uv"][0], p["start_uv"][1], z_c])
                g = compiled.frame.to_world([p["goal_uv"][0], p["goal_uv"][1], z_c])
                w0, c0 = time.perf_counter(), time.process_time()
                try:
                    with _alarm(timeout_s):
                        r = query(compiled, s, g, config=qconfig, call_id=p["pair_id"])
                    row = _row(p, r, compiled, compile_stages, time.perf_counter() - w0,
                               time.process_time() - c0, manifest)
                except _Timeout:
                    row = {"index": p["index"], "pair_id": p["pair_id"], "start_uv": p["start_uv"],
                           "goal_uv": p["goal_uv"], "dist_m": p["dist_m"], "compile_id": compiled.compile_id,
                           "status": "TIMEOUT", "reason": f"query_exceeded_{timeout_s:g}s",
                           "outer_wall_s": time.perf_counter() - w0, "cpu_s": time.process_time() - c0,
                           "compile_stages_in_call": [], "polyline_sha256": None}
                f.write(json.dumps(_json_finite(row), default=float) + "\n")
                f.flush()
                os.fsync(f.fileno())
                rows.append(row)
        changed = json.dumps(compiled.timings["records"], sort_keys=True, default=float) != before
    finally:
        a3api.compile_complex = real_compile
    allrows = {**done, **{r["index"]: r for r in rows}}
    ids = {r["compile_id"] for r in allrows.values()}
    st = [r["status"] for r in allrows.values()]
    return {"a3c": str(a3c), "compile_id": compiled.compile_id, "load_wall_s": load_s,
            "pairs_in_slice": len(pairs), "resumed_from_checkpoint": len(done), "answered_this_run": len(rows),
            "complete": len(allrows) == len(pairs), "status_counts": {k: st.count(k) for k in STATUSES},
            "query_wall_s_this_run": float(sum(r["outer_wall_s"] for r in rows)),
            "query_cpu_s_this_run": float(sum(r["cpu_s"] for r in rows)),
            "compile_once_proof": {"compiles_in_this_process": compiles["n"], "loads_in_this_process": 1,
                                   "compile_records_changed_by_queries": changed,
                                   "same_compile_id_all_rows": len(ids) == 1,
                                   "any_compile_stage_inside_a_query_call": any(r["compile_stages_in_call"]
                                                                                for r in allrows.values())},
            "peak_rss_mb": rss_mb()}


# ----------------------------------------------------------------------------- commands
def cmd_sample(a):
    from gmc.gs3d.contracts import Pose3
    from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
    from aerial3dg_run import ground_world, load_booth
    box = _box(a.box)
    ctx = load_booth(box)
    t0 = time.perf_counter()
    oracle = GaussianBodyOracle(PreparedScene(ctx["scene"]))
    prep_s = time.perf_counter() - t0

    def is_free(name, uv):
        body = ROBOTS[name]
        q = Pose3(tuple(map(float, ground_world(ctx["frame"], body, uv))), 0.)
        rep = oracle.pose(q, body, margin_m=GROUND_CONFIG.margin_m)
        ok = rep.occupancy == "free" and rep.safety == "continuous_bound"
        return ok, (float(rep.clearance_lower_m) if ok else None)

    t0 = time.perf_counter()
    pairs, counts = sample_pairs(a.n, seed=a.seed, box_uv=a.box, is_free=is_free, robots=tuple(a.robots),
                                 min_dist=a.min_dist)
    wall = time.perf_counter() - t0
    d = np.array([p["dist_m"] for p in pairs])
    doc = {"schema": "aerial3dg.pairs.v1", "n": a.n, "seed": a.seed, "box_route": box, "box_uv": a.box,
           "robots": a.robots, "min_dist_m": a.min_dist, "margin_m": GROUND_CONFIG.margin_m,
           "rule": sample_pairs.__doc__.strip(),
           "endpoint_check": "gs3d GaussianBodyOracle.pose at the robot's own z_c (floor + clearance + half height), "
                             "free and continuous_bound, margin 0.001 -- the same authority the planner's shared "
                             "replay uses; no connectivity filter",
           "counts": counts, "oracle_calls": oracle.stats["oracle_calls"],
           "acceptance_rate": counts["accepted"] / counts["pair_draws"],
           "dist_m": {"min": float(d.min()), "median": float(np.median(d)), "max": float(d.max())},
           "archive_sha256": ctx["digest"], "crop": ctx["crop"], "host": host(),
           "timing_s": {"archive_load_and_hash": ctx["archive_load_and_hash_s"],
                        "scene_build_and_crop": ctx["scene_build_and_crop_s"], "oracle_prepare": prep_s,
                        "sampling": wall},
           "pairs": pairs}
    _dump(a.out, doc)
    print(json.dumps({k: doc[k] for k in ("counts", "acceptance_rate", "dist_m", "timing_s")}, default=float), flush=True)


def cmd_compile(a):
    from aerial3dg_run import load_booth, timed_compile
    ctx = load_booth(_box(a.box))
    compiled, outer, cpu = timed_compile(ctx["scene"], ROBOTS[a.robot])
    rec = compile_record(compiled, outer, cpu)
    t0 = time.perf_counter()
    meta = save_compiled(compiled, a.a3c)
    rec["persist"] = {**meta, "path": str(a.a3c), "save_wall_s": time.perf_counter() - t0}
    doc = {"schema": "aerial3dg.compile.v1", "robot": a.robot, "box_route": ctx["box_route"], "crop": ctx["crop"],
           "archive_sha256": ctx["digest"], "support": ctx["support_evidence"], "host": host(),
           "archive_load_and_hash_s": ctx["archive_load_and_hash_s"],
           "scene_build_and_crop_s": ctx["scene_build_and_crop_s"], "compile": rec, "peak_rss_mb_end": rss_mb()}
    _dump(a.out, doc)
    print(a.robot, json.dumps({k: rec[k] for k in ("compile_id", "compile_wall_s", "peak_rss_mb")}, default=float),
          meta["bytes"], flush=True)


def cmd_task(a):
    t = a.task if a.task is not None else int(os.environ["SLURM_ARRAY_TASK_ID"])
    doc = json.loads(a.pairs.read_text())
    pairs = [doc["pairs"][i] for i in task_slice(len(doc["pairs"]), a.n_tasks, t)]
    manifest = json.loads(MANIFEST.read_text())
    out = a.out_dir / f"task_{t:02d}.jsonl"
    print(f"{a.robot} task {t}/{a.n_tasks}: pairs {pairs[0]['index']}..{pairs[-1]['index']}", flush=True)
    summary = run_task(a.a3c, pairs, out, timeout_s=a.timeout, manifest=manifest)
    summary.update(robot=a.robot, task=t, n_tasks=a.n_tasks, pair_index_range=[pairs[0]["index"], pairs[-1]["index"]],
                   pairs_file=str(a.pairs), host=host(), timeout_s=a.timeout)
    _dump(a.out_dir / f"task_{t:02d}.summary.json", summary)
    print(json.dumps({k: summary[k] for k in ("status_counts", "resumed_from_checkpoint", "answered_this_run",
                                              "query_wall_s_this_run", "compile_once_proof", "peak_rss_mb")},
                     default=float), flush=True)


def _stats(x):
    x = np.asarray(x, float)
    if not len(x):
        return None
    return {"n": int(len(x)), "sum": float(x.sum()), "mean": float(x.mean()), "median": float(np.median(x)),
            "p95": float(np.percentile(x, 95)), "max": float(x.max())}


def cmd_collect(a):
    doc = json.loads(a.pairs.read_text())
    n = len(doc["pairs"])
    out = {"schema": "aerial3dg.g2_collect.v1", "pairs_file": str(a.pairs), "n_pairs": n, "robots": {}}
    status = {}
    for robot in a.robots:
        rows = {}
        for f in sorted((a.runs / robot).glob("task_*.jsonl")):
            rows.update(read_checkpoint(f))
        st = [r["status"] for r in rows.values()]
        reasons = {}
        for r in rows.values():
            reasons.setdefault(r["status"], {}).setdefault(r["reason"], 0)
            reasons[r["status"]][r["reason"]] += 1
        summaries = [json.loads(f.read_text()) for f in sorted((a.runs / robot).glob("task_*.summary.json"))]
        reach = [r for r in rows.values() if r["status"] == "REACHABLE"]
        out["robots"][robot] = {
            "completed": len(rows), "status_counts": {k: st.count(k) for k in STATUSES}, "reasons": reasons,
            "query_wall_s": _stats([r["outer_wall_s"] for r in rows.values()]),
            "query_wall_s_by_status": {k: _stats([r["outer_wall_s"] for r in rows.values() if r["status"] == k])
                                       for k in STATUSES},
            "path_over_straight": _stats([r["path_length_m"] / r["dist_m"] for r in reach]),
            "clearance_lower_m": _stats([r["clearance_lower_m"] for r in reach if r["clearance_lower_m"] is not None]),
            "shared_replay_passed_all_reachable": all(r.get("shared_replay_passed") for r in reach),
            "compile_ids": sorted({r["compile_id"] for r in rows.values()}),
            "task_summaries": len(summaries),
            "load_wall_s": _stats([s["load_wall_s"] for s in summaries]),
            "task_peak_rss_mb": _stats([s["peak_rss_mb"] for s in summaries]),
            "compile_once_all_tasks": all(s["compile_once_proof"]["compiles_in_this_process"] == 0
                                          and not s["compile_once_proof"]["compile_records_changed_by_queries"]
                                          and s["compile_once_proof"]["same_compile_id_all_rows"]
                                          and not s["compile_once_proof"]["any_compile_stage_inside_a_query_call"]
                                          for s in summaries)}
        status[robot] = {i: r["status"] for i, r in rows.items()}
    if len(a.robots) == 2:
        r0, r1 = a.robots
        both = sorted(set(status[r0]) & set(status[r1]))
        cross = {}
        for i in both:
            k = f"{status[r0][i]}|{status[r1][i]}"
            cross[k] = cross.get(k, 0) + 1
        out["paired"] = {"robots": [r0, r1], "pairs_answered_by_both": len(both), "crosstab": cross,
                         "definition": f"key = '{r0} status|{r1} status' on the same pair"}
    _dump(a.out, out)
    print(json.dumps({r: (v["completed"], v["status_counts"]) for r, v in out["robots"].items()}), flush=True)
    if "paired" in out:
        print(json.dumps(out["paired"]["crosstab"]), flush=True)


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    sa = sub.add_parser("sample")
    sa.add_argument("--box", type=float, nargs=4, required=True, metavar=("U0", "V0", "U1", "V1"))
    sa.add_argument("--n", type=int, default=5000)
    sa.add_argument("--seed", type=int, required=True)
    sa.add_argument("--min-dist", type=float, default=3.0)
    sa.add_argument("--robots", nargs="+", default=["sweeper", "cylinder"])
    sa.add_argument("--out", type=Path, required=True)
    co = sub.add_parser("compile")
    co.add_argument("--robot", required=True, choices=sorted(ROBOTS))
    co.add_argument("--box", type=float, nargs=4, required=True, metavar=("U0", "V0", "U1", "V1"))
    co.add_argument("--a3c", type=Path, required=True)
    co.add_argument("--out", type=Path, required=True)
    ta = sub.add_parser("task")
    ta.add_argument("--robot", required=True, choices=sorted(ROBOTS))
    ta.add_argument("--pairs", type=Path, required=True)
    ta.add_argument("--a3c", type=Path, required=True)
    ta.add_argument("--n-tasks", type=int, required=True)
    ta.add_argument("--task", type=int, default=None, help="default: SLURM_ARRAY_TASK_ID")
    ta.add_argument("--timeout", type=float, default=120.)
    ta.add_argument("--out-dir", type=Path, required=True)
    cl = sub.add_parser("collect")
    cl.add_argument("--pairs", type=Path, required=True)
    cl.add_argument("--runs", type=Path, required=True)
    cl.add_argument("--robots", nargs="+", default=["sweeper", "cylinder"])
    cl.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)
    {"sample": cmd_sample, "compile": cmd_compile, "task": cmd_task, "collect": cmd_collect}[a.cmd](a)


if __name__ == "__main__":
    main()
