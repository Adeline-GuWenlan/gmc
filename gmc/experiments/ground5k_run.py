"""Ground 5000-pair benchmark, stage G1 Task 2: run one planner x robot on a slice of pairs.

Per (pair, robot, planner): crop the query region (``ground5k_common``), plan once from start
to goal, replay the returned path through the shared gs3d replay, write one JSON atomically.
Each query runs in a forked child under a watchdog: wall cap (crop + preparation/projection
+ compile + query + certification) and memory cap; a kill is ``FAIL_BUDGET``.  Outcome set
and mapping: ``docs/ground5k_design.md`` §1.  A finished JSON is never recomputed.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import asdict
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import traceback

import numpy as np

from gmc.gs3d import planner as gs3d_planner
from gmc.gs3d.contracts import GoalRegion, PlannerConfig, SearchBudget
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.planner import LatticePlanner, _limits, _linear_trajectory
from gmc.gs3d.trajectory import replay_plan

import ground5k_common as gc

OUT = Path("outputs/ground5k")
GMC_BUDGET_REASONS = ("query_wall_budget_exhausted", "query_support_budget_exhausted")


# ------------------------------------------------------------------------ watchdog --

def _proc_mem(pid):
    rss = hwm = 0
    try:
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    rss = int(line.split()[1]) * 1024
                elif line.startswith("VmHWM:"):
                    hwm = int(line.split()[1]) * 1024
    except OSError:
        pass
    return rss, hwm


def watchdog(fn, *, wall_cap_s, mem_cap_bytes, grace_s=0.0, post_cap_s=None, poll_s=0.05) -> dict:
    """Run ``fn(progress)`` in a forked child; kill it at the wall or memory cap.

    ``progress(stage, **info)`` reports the stage the child is in.  The stage
    ``capped_done`` ends the capped phase; after it only ``post_cap_s`` applies.
    """
    ctx = mp.get_context("fork")
    rx, tx = ctx.Pipe(duplex=False)

    def child():
        rx.close()

        def progress(stage, **info):
            tx.send(("stage", stage, time.time(), info))
        try:
            tx.send(("result", fn(progress)))
        except BaseException as exc:  # noqa: BLE001 - everything becomes a record
            tx.send(("error", f"{type(exc).__name__}: {exc}", traceback.format_exc()[-6000:]))
        tx.close()
        os._exit(0)

    proc = ctx.Process(target=child)
    t0 = time.time()
    proc.start()
    tx.close()
    out = {"status": None, "result": None, "stages": [], "stage": None, "stage_info": {},
           "peak_rss_bytes": 0, "kill_reason": None, "error": None}
    capped_done_at = None
    eof = False

    def drain():
        nonlocal capped_done_at, eof
        while not eof and rx.poll():
            try:
                msg = rx.recv()
            except EOFError:
                eof = True
                break
            if msg[0] == "stage":
                out["stages"].append((msg[1], msg[2] - t0))
                out["stage"] = msg[1]
                out["stage_info"][msg[1]] = msg[3]
                if msg[1] == "capped_done":
                    capped_done_at = msg[2]
            elif msg[0] == "result":
                out["status"], out["result"] = "ok", msg[1]
            else:
                out["status"], out["error"], out["traceback"] = "error", msg[1], msg[2]

    while True:
        drain()
        rss, hwm = _proc_mem(proc.pid)
        out["peak_rss_bytes"] = max(out["peak_rss_bytes"], rss, hwm)
        if out["status"] is not None or not proc.is_alive():
            proc.join(timeout=30)
            drain()
            break
        now = time.time()
        limit = (t0 + wall_cap_s + grace_s if capped_done_at is None
                 else capped_done_at + (post_cap_s if post_cap_s is not None else math.inf))
        if now > limit:
            out["kill_reason"] = "wall_cap" if capped_done_at is None else "post_cap_wall"
        elif rss > mem_cap_bytes:
            out["kill_reason"] = "memory_cap"
        if out["kill_reason"]:
            proc.kill()
            proc.join()
            drain()
            out["status"] = "killed"
            break
        time.sleep(poll_s)
    if out["status"] is None:
        out["status"] = "died"
        out["exitcode"] = proc.exitcode
    out["wall_s"] = time.time() - t0
    rx.close()
    return out


# ---------------------------------------------------------------- outcome mapping --

def classify_astar(result: dict, oracle_reasons: dict, faces: dict) -> tuple[str, str]:
    """gs3d lattice A* status -> outcome set (design §1.3).

    The query region's box faces bound the search like GMC's workspace boundary: unknown or
    missing-support rejections caused only by them are not map evidence.  Rejections at the
    contact-strip faces (support evidence limit) or unresolved geometry are FAIL_UNKNOWN.
    """
    st, reason = result["status"], str(result.get("reason"))
    if st == "success":
        return "SUCCESS_VERIFIED", "astar_success"
    if st == "budget_exhausted":
        return "FAIL_BUDGET", f"astar_{reason}"
    if st in ("start_invalid", "goal_invalid") or reason.startswith(("start:", "original_goal:")):
        return "FAIL_UNKNOWN", f"astar_endpoint:{st}:{reason}"
    if reason == "post_build_verification_failed":
        return "FAIL_UNKNOWN", "astar_post_build_verification_failed"
    if st in ("map_unknown", "verification_failed", "no_path_on_lattice"):
        if faces.get("contact", 0) + faces.get("contact_hi", 0) + faces.get("contact_lo", 0):
            return "FAIL_UNKNOWN", "astar_search_reached_support_evidence_limit"
        if oracle_reasons.get("geometry_or_margin_unproven", 0):
            return "FAIL_UNKNOWN", "astar_unproven_geometry_on_frontier"
        return "FAIL_NO_PATH", "astar_reachable_lattice_exhausted_in_query_region"
    return "ERROR", f"astar_{st}:{reason}"


def classify_gmc(res: dict) -> tuple[str, str]:
    """GMC status -> outcome set (design §1.3)."""
    st, reason = res["status"], res.get("reason")
    if st == "REACHABLE":
        if res.get("verify", {}).get("certified"):
            return "SUCCESS_VERIFIED", "gmc_reachable_certified"
        return "FAIL_UNKNOWN", "gmc_verify_curve_not_certified"
    if st == "UNREACHABLE":
        return "FAIL_NO_PATH", "gmc_certified_unreachable"
    if st == "UNKNOWN":
        if reason in GMC_BUDGET_REASONS:
            return "FAIL_BUDGET", f"gmc_{reason}"
        if reason == "possible_cut_is_not_a_global_certificate":
            return "FAIL_NO_PATH", "gmc_possible_graph_cut_prototype_mode"
        return "FAIL_UNKNOWN", f"gmc_{reason}"
    if st == "INVALID_GEOMETRY":
        return "FAIL_UNKNOWN", "gmc_endpoint_invalid_on_projection"
    return "ERROR", f"gmc_{st}:{reason}"


def with_replay(outcome: str, replay: dict | None) -> str:
    if outcome == "SUCCESS_VERIFIED" and not (replay or {}).get("passed"):
        return "FAIL_REPLAY"
    return outcome


# ---------------------------------------------------------------- paths + replay --

def gmc_poses(curve: dict, body, floor) -> list:
    """GMC PoseCurve knots -> supported gs3d poses (xy polyline; yaw is set by replay)."""
    knots = [curve["segments"][0]["q0"]]
    for seg in curve["segments"]:
        knots.extend(seg.get("control_points", []))
        knots.append(seg["q1"])
    xy = []
    for k in knots:
        p = (float(k[0]), float(k[1]))
        if not xy or math.hypot(p[0] - xy[-1][0], p[1] - xy[-1][1]) > 1e-12:
            xy.append(p)
    return [gc.ground_pose(x, y, body, floor) for x, y in xy]


def _replay_summary(rep: dict, rows=None) -> dict:
    """Verdict plus, on failure, the rejected edge (poses, implicated Gaussian ids, bound)."""
    geo = rep.get("geometry") or {}
    reports = geo.get("reports", [])
    out = {"passed": bool(rep.get("passed")), "clearance_lb_m": geo.get("clearance_lower_m"),
           "geometry_reason": geo.get("reason"), "n_edges_checked": len(reports),
           "kinematics": {k: v for k, v in (rep.get("kinematics") or {}).items() if k != "samples"},
           "attained_matches": rep.get("attained_matches"), "reason": rep.get("reason")}
    if not geo.get("passed", True) and reports:
        k = len(reports) - 1
        last = reports[-1]
        out["failed_edge"] = {"index": k, **{f: last.get(f) for f in (
            "occupancy", "safety", "clearance_lower_m", "reason", "primitive_ids", "candidate_count",
            "narrowphase_pairs")}}
        if rows is not None and len(rows) > 1:
            out["failed_edge"]["from"] = [float(v) for v in rows[k]]
            out["failed_edge"]["to"] = [float(v) for v in rows[k + 1]]
    return out


def replay_poses(spec, body, poses, goal, cfg) -> dict:
    """Shared independent replay for a pose list (used for GMC paths)."""
    traj_poses, traj = _linear_trajectory(poses, body, poses[0].yaw, goal.yaw)
    result = {"schema_version": "gs3d.v1", "status": "success", "trajectory": traj,
              "attained_goal": [*traj_poses[-1].xyz, traj_poses[-1].yaw],
              "original_goal": [*goal.xyz, goal.yaw], "position_tolerance_m": 0.0,
              "yaw_tolerance_rad": cfg["astar"]["yaw_tolerance_rad"],
              "robot": {**asdict(body), "limits": _limits(body)},
              "diagnostics": {"margin_m": cfg["margin_m"]}}
    return replay_plan(result, GaussianBodyOracle(PreparedScene(spec))), traj["poses"]


def _path_xy(poses_rows) -> list:
    xy = []
    for r in poses_rows:
        p = [round(float(r[0]), 6), round(float(r[1]), 6)]
        if not xy or math.hypot(p[0] - xy[-1][0], p[1] - xy[-1][1]) > 1e-9:
            xy.append(p)
    return xy


# ------------------------------------------------------------------------ A* --

class _Observer:
    """Observation-only record of the A* search: rejection reasons and the expanded pose
    closest to the goal (where a failed search stopped)."""

    def __init__(self, goal):
        self.goal = np.asarray(goal.xyz[:2], float)
        self.reasons: dict[str, int] = {}
        self.closest = None
        self.closest_d = math.inf
        self.last = None


@contextmanager
def _observed_oracle(observer: _Observer):
    base = gs3d_planner.GaussianBodyOracle

    class Observed(base):
        def edge(self, a, b, body, *, margin_m):
            rep = super().edge(a, b, body, margin_m=margin_m)
            if not (rep.occupancy == "free" and rep.safety == "continuous_bound"):
                observer.reasons[rep.reason] = observer.reasons.get(rep.reason, 0) + 1
            if a.xyz != b.xyz:
                xy = np.asarray(a.xyz[:2], float)
                observer.last = xy
                d = float(np.hypot(*(xy - observer.goal)))
                if d < observer.closest_d:
                    observer.closest_d, observer.closest = d, xy
            return rep

    gs3d_planner.GaussianBodyOracle = Observed
    try:
        yield
    finally:
        gs3d_planner.GaussianBodyOracle = base


def _run_astar(spec, region, body, start, goal, cfg, t0, progress) -> dict:
    a = cfg["astar"]
    progress("prepare")
    tp = time.perf_counter()
    prepared = PreparedScene(spec)
    prep_s = time.perf_counter() - tp
    remaining = cfg["per_query"]["wall_cap_s"] - (time.perf_counter() - t0)
    out = {"prepare_s": prep_s, "compile_s": None, "verify_s": None, "prepared": prepared.stats}
    if remaining <= 0:
        return {**out, "outcome": "FAIL_BUDGET", "cause": "wall_cap_before_search", "plan_s": 0.0,
                "poses": None, "planner_status": None}
    counts = {k: int(a["budget"][k]) for k in ("max_expansions", "max_oracle_calls", "max_narrowphase_pairs")}
    conf = PlannerConfig(resolution_m=a["resolution_m"], margin_m=a["margin_m"], seed=a["seed"],
                         budget=SearchBudget(max_wall_s=float(remaining), **counts))
    observer = _Observer(goal)
    before = dict(region.rejections)
    progress("plan", max_wall_s=float(remaining))
    with _observed_oracle(observer):
        result = LatticePlanner(prepared).plan(
            spec, body, start, GoalRegion(goal, a["position_tolerance_m"], a["yaw_tolerance_rad"]), conf)
    faces = {k: region.rejections[k] - before.get(k, 0) for k in region.rejections}
    outcome, cause = classify_astar(result, observer.reasons, faces)
    d = result["diagnostics"]
    stats = {k: d.get(k) for k in ("expansions", "oracle_calls", "narrowphase_pairs", "visited_nodes",
                                   "frontier_peak", "map_unknown_rejections", "unproven_rejections",
                                   "occupied_rejections", "goal_candidates_checked", "candidate_count",
                                   "bvh_nodes_visited", "gjk_iterations", "termination")}
    stats.update(oracle_reasons=observer.reasons, region_face_rejections=faces,
                 budget_max_wall_s=float(remaining))
    stop = None if result["status"] == "success" else {
        "closest_expanded_xy": None if observer.closest is None else observer.closest.round(6).tolist(),
        "closest_expanded_dist_to_goal_m": None if observer.closest is None else observer.closest_d,
        "last_expanded_xy": None if observer.last is None else observer.last.round(6).tolist()}
    return {**out, "outcome": outcome, "cause": cause, "planner_status": result["status"],
            "planner_reason": result["reason"], "plan_s": result["timings"]["algorithm_wall_s"],
            "stats": stats, "stopped_at": stop, "result": result,
            "poses": result["trajectory"]["poses"] if result["status"] == "success" else None}


# ------------------------------------------------------------------------ GMC --

def _run_gmc(spec, region, body, robot_name, start, goal, floor, cfg, t0, progress) -> dict:
    from gmc.budget import WorkLedger
    from gmc.config import load_config
    from gmc.height.pathio import curve_to_dict
    from gmc.height.prism import PrismRobot, robot_table
    from gmc.height.project import project_scene
    from gmc.mobility.graph import compile_mobility
    from gmc.mobility.query import query
    from gmc.orientation.slab_builder import build_slabs
    from gmc.spatial.bvh import query_candidate_pairs
    from gmc.types import Pose2, SceneModel2D
    from gmc.verification.path import verify_curve

    gcfg = load_config(cfg["gmc"]["config"])
    # The shared replay (and A*) demand clearance > margin in every direction.  GMC's band test
    # drops a splat lying wholly outside the band however close it is, so the band is inflated
    # by that margin (horizontally GMC's eps_clear, 2 mm, already exceeds it).  See design §1.4.
    base = robot_table()[robot_name]
    m = float(cfg["margin_m"])
    robot = PrismRobot(base.footprint, base.z_lo - m, base.z_hi + m, base.name)
    ws = region.polygon(erode=body.radius_m + cfg["margin_m"])
    progress("project")
    tp = time.perf_counter()
    s2, pstats = project_scene(spec.gaussians, robot, ws.bounds, z_floor=floor["z_floor"],
                               tau=cfg["tau"], level=cfg["level"])
    s2 = SceneModel2D(supports=s2.supports, workspace=ws, name=s2.name)
    project_s = time.perf_counter() - tp
    stats = {"n_supports": len(s2.supports), "projection": pstats, "workspace_area_m2": float(ws.area),
             "band_above_floor_m": [robot.z_lo, robot.z_hi]}
    progress("compile_pairs", n_supports=len(s2.supports))
    tc = time.perf_counter()
    ledger = WorkLedger()
    oracles = list(query_candidate_pairs(s2, robot.footprint, s2.workspace).oracles)
    for o in oracles:
        o.ledger = ledger
    stats["n_pairs"] = len(oracles)
    progress("compile_slabs", n_pairs=len(oracles))
    dec = build_slabs(s2, robot.footprint, gcfg, oracles, ledger=ledger)
    stats["n_slabs"] = len(dec.slabs)
    progress("compile_mobility", n_slabs=len(dec.slabs))
    mc = compile_mobility(s2, robot.footprint, gcfg, oracles, dec, ledger=ledger)
    compile_s = time.perf_counter() - tc
    stats.update(safe_nodes=mc.M_safe.number_of_nodes(), safe_edges=mc.M_safe.number_of_edges(),
                 possible_nodes=mc.M_possible.number_of_nodes(),
                 possible_edges=mc.M_possible.number_of_edges())
    out = {"prepare_s": project_s, "compile_s": compile_s, "stats": stats}
    remaining = cfg["per_query"]["wall_cap_s"] - (time.perf_counter() - t0)
    if remaining <= 0:
        return {**out, "outcome": "FAIL_BUDGET", "cause": "gmc_wall_cap_before_query", "plan_s": 0.0,
                "verify_s": None, "poses": None, "planner_status": None,
                "stopped_at": {"stage": "compile_done"}}
    # The per-query cap is enforced inside GMC's own deadline: the query's wall budget is the
    # time left.  mc keeps the identical cfg object (its decomposition binding checks identity).
    object.__setattr__(gcfg.query, "max_wall_seconds", float(remaining))
    s, g = Pose2(np.asarray(start.xyz[:2]), float(start.yaw)), Pose2(np.asarray(goal.xyz[:2]), float(goal.yaw))
    progress("query", max_wall_seconds=float(remaining))
    tq = time.perf_counter()
    result = query(s, g, mc)
    query_s = time.perf_counter() - tq
    rep = result.report or {}
    res = {"status": result.status.name, "reason": rep.get("reason"),
           "clearance_lb": result.clearance_lower_bound, "verify": {"ran": False}}
    stats["query_report"] = {k: rep.get(k) for k in ("wall_s", "wall_budget_s", "query_support_calls",
                                                     "query_support_budget", "refinement_rounds",
                                                     "graph_revision", "stage", "paths_tried",
                                                     "refinement_stop_reason", "verify",
                                                     "failed_invariants", "error_type", "error",
                                                     "verify_reason")}
    stats["query_report"] = json.loads(json.dumps(stats["query_report"], default=str))
    verify_s = None
    curve = None
    if result.curve is not None:
        progress("verify_curve")
        tv = time.perf_counter()
        vr = verify_curve(tuple(oracles), s2.workspace, result.curve, gcfg.query.eps_clear,
                          gcfg.orientation.theta_min, expected_start=s, expected_goal=g)
        verify_s = time.perf_counter() - tv
        res["verify"] = {"ran": True, "certified": bool(vr.certified),
                         "min_clearance": float(vr.min_clearance), "reason": vr.reason}
        curve = curve_to_dict(result.curve)
    outcome, cause = classify_gmc(res)
    stop = None
    if result.status.name != "REACHABLE":
        stop = {"stage": rep.get("stage"), "reason": rep.get("reason"),
                "query_support_calls": rep.get("query_support_calls")}
    return {**out, "outcome": outcome, "cause": cause, "planner_status": res["status"],
            "planner_reason": res["reason"], "plan_s": query_s, "verify_s": verify_s,
            "gmc": res, "curve": curve, "stopped_at": stop,
            "poses": gmc_poses(curve, body, floor) if curve is not None else None}


# ---------------------------------------------------------------------- one query --

def run_one(scene3d, floor, pair, combo, cfg, progress, *, scene_name) -> dict:
    planner, robot_name = combo.split("_")
    body = gc.BODIES[robot_name]
    t0 = time.perf_counter()
    progress("crop")
    region = gc.query_region(pair["crop"]["box_xy"], cfg, floor)
    spec, crop = gc.crop_query(scene3d, region, floor, cfg, f"ground5k-{scene_name}-{pair['pair_id']}")
    crop_s = time.perf_counter() - t0
    yaw0 = cfg["gmc"]["start_yaw"] if planner == "gmc" else 0.0
    yaw1 = cfg["gmc"]["goal_yaw"] if planner == "gmc" else 0.0
    start = gc.ground_pose(*pair["start"], body, floor, yaw0)
    goal = gc.ground_pose(*pair["goal"], body, floor, yaw1)
    rec = {"schema_version": "ground5k.run.v1", "pair_id": pair["pair_id"], "combo": combo,
           "planner": planner, "robot": robot_name, "scene": scene_name,
           "start": list(start.xyz), "goal": list(goal.xyz),
           "crop": {**crop, "box_xy": list(region.box), "region_vertices_xy": region.vertices().round(6).tolist(),
                    "area_m2": float(region.polygon().area)}}
    if planner == "astar":
        plan = _run_astar(spec, region, body, start, goal, cfg, t0, progress)
    else:
        plan = _run_gmc(spec, region, body, robot_name, start, goal, floor, cfg, t0, progress)
    total = time.perf_counter() - t0
    poses = plan.pop("poses")
    result = plan.pop("result", None)
    rec.update({k: v for k, v in plan.items() if k not in ("prepare_s", "compile_s", "plan_s", "verify_s")})
    rec["time"] = {"crop_s": crop_s, "prepare_s": plan["prepare_s"], "compile_s": plan["compile_s"],
                   "plan_s": plan["plan_s"], "verify_s": plan["verify_s"], "total_capped_s": total}
    progress("capped_done", record=rec)
    replay = None
    tr = time.perf_counter()
    if poses is not None:
        progress("replay")
        if planner == "astar":
            rows = poses
            replay = _replay_summary(replay_plan(result, GaussianBodyOracle(PreparedScene(spec))), rows)
        else:
            rep, traj_rows = replay_poses(spec, body, poses, goal, cfg)
            replay = _replay_summary(rep, traj_rows)
            rows = [[*p.xyz, p.yaw] for p in poses]
        start_ok = math.hypot(rows[0][0] - start.xyz[0], rows[0][1] - start.xyz[1]) <= 1e-9
        replay["start_matches"] = start_ok
        replay["passed"] = bool(replay["passed"] and start_ok)
        xy = _path_xy(rows)
        rec["path_xy"] = xy
        rec["path_length_m"] = float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(xy, xy[1:])))
    else:
        rec["path_xy"], rec["path_length_m"] = None, None
    rec["time"]["replay_s"] = time.perf_counter() - tr
    rec["replay"] = replay
    rec["outcome"] = with_replay(rec["outcome"], replay)
    return rec


def _killed_record(pair, combo, wd, scene_name) -> dict:
    planner, robot = combo.split("_")
    partial = wd["stage_info"].get("capped_done", {}).get("record")
    if partial is not None:              # planned; the replay did not finish
        rec = dict(partial)
        rec.update(outcome="ERROR", cause=f"replay_{wd['kill_reason'] or wd['status']}",
                   replay=None, path_xy=None, path_length_m=None)
        return rec
    if wd["status"] == "killed":
        outcome, cause = "FAIL_BUDGET", f"{wd['kill_reason']}_kill"
    elif wd["status"] == "died" and wd.get("exitcode") == -signal.SIGKILL:
        outcome, cause = "FAIL_BUDGET", "killed_by_system_likely_oom"
    else:
        outcome, cause = "ERROR", (wd.get("error") or f"child_{wd['status']}_exit{wd.get('exitcode')}")[:300]
    return {"schema_version": "ground5k.run.v1", "pair_id": pair["pair_id"], "combo": combo,
            "planner": planner, "robot": robot, "scene": scene_name, "outcome": outcome, "cause": cause,
            "planner_status": None, "path_xy": None, "path_length_m": None, "replay": None,
            "stopped_at": {"stage": wd["stage"], "stage_info": {k: v for k, v in wd["stage_info"].items()
                                                                 if k != "capped_done"}},
            "traceback": wd.get("traceback"),
            "time": {"total_capped_s": wd["wall_s"]}}


PARENT_RESERVE_GB = 1.5   # the task's own copy of the scene (sacct: 1.3 GB) plus slack


def memory_cap_gb(config_cap_gb, slurm_mem_mb):
    """Child cap: the config cap, but never above the task's Slurm memory minus the parent."""
    if not slurm_mem_mb:
        return float(config_cap_gb)
    return min(float(config_cap_gb), float(slurm_mem_mb) / 1024 - PARENT_RESERVE_GB)


def run_pairs(scene3d, floor, pairs, combo, cfg, outdir, *, scene_name, meta=None, mem_cap_gb=None) -> dict:
    pq = cfg["per_query"]
    cap_gb = float(mem_cap_gb if mem_cap_gb is not None else pq["memory_cap_gb"])
    done = {"ran": 0, "skipped": 0}
    for pair in pairs:
        path = Path(outdir) / combo / f"{pair['pair_id']}.json"
        if path.exists():
            try:
                json.loads(path.read_text())
                done["skipped"] += 1
                continue
            except ValueError:
                pass
        started = time.time()
        wd = watchdog(lambda progress: run_one(scene3d, floor, pair, combo, cfg, progress,
                                               scene_name=scene_name),
                      wall_cap_s=pq["wall_cap_s"], mem_cap_bytes=int(cap_gb * (1 << 30)),
                      grace_s=pq["watchdog_grace_s"], post_cap_s=pq["wall_cap_s"])
        rec = wd["result"] if wd["status"] == "ok" else _killed_record(pair, combo, wd, scene_name)
        rec["peak_rss_bytes"] = wd["peak_rss_bytes"]
        rec["query_wall_s"] = wd["wall_s"]
        rec["stages_s"] = wd["stages"]
        rec["run"] = {**(meta or {}), "host": socket.gethostname(), "child_memory_cap_gb": cap_gb,
                      "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(started))}
        gc.write_json_atomic(path, rec)
        done["ran"] += 1
        print(f"[{time.strftime('%H:%M:%S')}] {combo} {pair['pair_id']} {rec['outcome']} {rec['cause']} "
              f"{wd['wall_s']:.1f}s rss {wd['peak_rss_bytes'] / 2**30:.2f}G", flush=True)
    return done


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="scene_v2")
    ap.add_argument("--combo", required=True, choices=gc.COMBOS)
    ap.add_argument("--first", type=int, default=0, help="first pair index of this array")
    ap.add_argument("--per-task", type=int, default=1)
    ap.add_argument("--task", type=int, default=int(os.environ.get("SLURM_ARRAY_TASK_ID", 0)))
    ap.add_argument("--limit", type=int, default=None, help="exclusive upper pair index (prefix size)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)
    cfg = gc.load_config()
    out = Path(args.out) if args.out else OUT / args.scene / "runs"
    doc = json.loads((OUT / args.scene / f"pairs_{args.scene}.json").read_text())
    lo = args.first + args.task * args.per_task
    hi = lo + args.per_task
    if args.limit is not None:
        hi = min(hi, args.limit)
    pairs = doc["pairs"][lo:hi]
    if not pairs:
        print("no pairs in this slice", flush=True)
        return
    meta = {"slurm_job_id": os.environ.get("SLURM_JOB_ID"), "array_job_id": os.environ.get("SLURM_ARRAY_JOB_ID"),
            "array_task_id": os.environ.get("SLURM_ARRAY_TASK_ID"),
            "git_head": subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip(),
            "config_sha256": gc.sha256(gc.CONFIG), "pairs_sha256": gc.sha256(OUT / args.scene / f"pairs_{args.scene}.json")}
    t0 = time.time()
    scene3d, floor = gc.load_scene(cfg, args.scene)
    print(f"loaded {args.scene} {len(scene3d)} splats in {time.time() - t0:.1f}s; pairs [{lo},{hi})", flush=True)
    cap = memory_cap_gb(cfg["per_query"]["memory_cap_gb"], os.environ.get("SLURM_MEM_PER_NODE"))
    done = run_pairs(scene3d, floor, pairs, args.combo, cfg, out, scene_name=args.scene, meta=meta,
                     mem_cap_gb=cap)
    print(json.dumps(done), flush=True)


if __name__ == "__main__":
    main()
