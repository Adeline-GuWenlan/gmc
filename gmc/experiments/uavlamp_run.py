"""L2 production run: one ``LatticePlanner.plan`` call per query, start + goal + box only.

Thin layer over ``uavlamp_query.build_scene`` (L1) and the ``gs3d`` package:

1. load the archive through the manifest-checked loader, build scene + index once
   (timed as preparation);
2. make the planner calls: an optional *cold* call (unprepared planner, index build
   inside algorithm time) and N *warm* calls (prepared index reused).  With more
   than one call they run in forked processes, one per CPU, so the wall time of a
   ~1.6 h query is paid once; every call is still exactly one ``plan`` call;
3. record the call's arguments verbatim (built from the objects actually passed);
4. independently replay the returned trajectory with a fresh oracle on a freshly
   prepared index (same Gaussians, frozen body and margin);
5. extract ordered evidence from the trajectory alone: under-lamp interval,
   above-table interval, their order, z-range, clearance lower bound.

Usage (from ``gmc/``)::

    python experiments/uavlamp_run.py SPEC.json --cold 1 --warm 3 --output DIR
    python experiments/uavlamp_run.py SPEC.json --smooth-from DIR/result.json --output DIR_SMOOTH

Specs are L1's ``outputs/uavlamp/specs/v2_*.json``; keys outside ``ALLOWED_SPEC_KEYS``
(waypoints, altitude schedules, per-segment limits, ...) are rejected.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import multiprocessing as mp
import os
from pathlib import Path
import platform
import sys
import time

import numpy as np

from gmc.gs3d import scene_uavlamp as su
from gmc.gs3d.contracts import GoalRegion, PlannerConfig, Pose3, SearchBudget
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.planner import LatticePlanner, _limits
from gmc.gs3d.timing import PlanTiming, latency_statistics
from gmc.gs3d.trajectory import replay_plan, sample_linear_trajectory

from uavlamp_query import UAV, build_scene

# Everything a query spec may contain.  Scene variants (``drop_roles``, ``extra_builders``)
# change the *map*, never the query; they are recorded in the result.
ALLOWED_SPEC_KEYS = frozenset({
    "name", "output", "archive", "archive_sha256", "manifest", "frame", "box_route",
    "start_route", "goal_route", "resolution_m", "margin_m", "budget",
    "lamp", "table", "top_band_z", "side_slab_v", "drop_roles", "extra_builders"})


def check_spec(spec: dict) -> dict:
    """Fail closed on anything that could steer the route (waypoints, z schedules, costs)."""
    extra = sorted(set(spec) - ALLOWED_SPEC_KEYS)
    if extra:
        raise ValueError(f"spec keys not allowed for a single start->goal query: {extra}")
    for key in ("start_route", "goal_route"):
        if np.asarray(spec[key], float).shape != (3,):
            raise ValueError(f"{key} must be one xyz point")
    return spec


def single_plan_call(planner: LatticePlanner, scene, start: Pose3, goal: GoalRegion,
                     config: PlannerConfig, *, timer=None) -> tuple[dict, dict]:
    """Exactly one ``plan`` call; returns (result, verbatim argument record).

    The signature has no place for waypoints; the record is built from the very
    objects handed to the planner, so a reader can see nothing else was passed.
    """
    if not isinstance(start, Pose3) or not isinstance(goal, GoalRegion):
        raise TypeError("start must be a Pose3 and goal a GoalRegion")
    known = getattr(scene.known_space, "inner", scene.known_space)
    record = {
        "method": "gmc.gs3d.planner.LatticePlanner.plan",
        "planner_prepared_index": planner.prepared is not None,
        "arguments": {
            "scene": {"scene_id": scene.scene_id, "n_gaussians": int(len(scene.gaussians.ids)),
                      "bounds_min": list(scene.bounds_min), "bounds_max": list(scene.bounds_max),
                      "tau": scene.tau, "level": scene.level,
                      "known_space": {"type": type(known).__name__, **asdict(known)},
                      "support": scene.support},
            "body": asdict(UAV),
            "start": asdict(start),
            "goal": asdict(goal),
            "config": asdict(config)},
        "keyword_arguments": ["timer"] if timer is not None else [],
        "not_passed": "no waypoints, no intermediate poses, no per-segment z bounds, no altitude "
                      "schedule, no cost terms: plan() accepts none of these"}
    result = planner.plan(scene, UAV, start, goal, config, timer=timer)
    return result, record


# ----------------------------------------------------------------------------- evidence

def _intervals(mask: np.ndarray, s: np.ndarray, z: np.ndarray) -> list[dict]:
    out = []
    idx = np.flatnonzero(mask)
    if not len(idx):
        return out
    breaks = np.flatnonzero(np.diff(idx) > 1)
    for a, b in zip(np.r_[idx[0], idx[breaks + 1]], np.r_[idx[breaks], idx[-1]]):
        out.append({"s_from_m": float(s[a]), "s_to_m": float(s[b]), "sample_from": int(a),
                    "sample_to": int(b), "z_min": float(z[a:b + 1].min()), "z_max": float(z[a:b + 1].max())})
    return out


def _dist_to_rect(xy: np.ndarray, lo, hi) -> np.ndarray:
    d = np.maximum(np.maximum(np.asarray(lo) - xy, xy - np.asarray(hi)), 0.)
    return np.hypot(d[:, 0], d[:, 1])


def ordered_evidence(route_xyz, *, lamp: dict, table: dict, radius: float,
                     half_height: float, margin: float) -> dict:
    """Ordered under-lamp / above-table evidence from route-frame centre samples alone.

    *Under the lamp*: the body disc overlaps the lamp's plan footprint and the body top
    plus margin is at or below the underside.  *Above the table*: the body disc overlaps
    the table-top footprint and the body bottom minus margin is at or above the top.
    Any sample whose body overlaps the lamp footprint without being under it (i.e. beside
    in z, or over) is counted in ``lamp_overlap_not_under``.
    """
    p = np.asarray(route_xyz, float)
    s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    z = p[:, 2]
    (l0, l1), (t0, t1) = lamp["footprint_route_uv"], table["top_route_uv"]
    lamp_body = _dist_to_rect(p[:, :2], l0, l1) < radius
    lamp_centre = _dist_to_rect(p[:, :2], l0, l1) == 0.
    table_body = _dist_to_rect(p[:, :2], t0, t1) < radius
    under = lamp_body & (z + half_height + margin <= lamp["underside_z"])
    above = table_body & (z - half_height - margin >= table["top_z"])
    under_iv, above_iv = _intervals(under, s, z), _intervals(above, s, z)
    ordered = bool(under_iv and above_iv and under_iv[0]["s_to_m"] < above_iv[0]["s_from_m"])
    over_lamp = lamp_body & (z - half_height - margin >= lamp["top_z"])
    return {"path_length_m": float(s[-1]), "n_samples": int(len(p)),
            "z_range": [float(z.min()), float(z.max())],
            "z_start": float(z[0]), "z_goal": float(z[-1]),
            "under_lamp_intervals": under_iv,
            "under_lamp_centre_in_footprint_intervals": _intervals(lamp_centre, s, z),
            "above_table_intervals": above_iv,
            "over_lamp_intervals": _intervals(over_lamp, s, z),
            "lamp_overlap_not_under_samples": int((lamp_body & ~under).sum()),
            "under_precedes_above_table": ordered,
            "passes_under_lamp": bool(under_iv) and not bool((lamp_body & ~under).any()),
            "goal_minus_lowest_under_lamp_z": (float(z[-1] - min(i["z_min"] for i in under_iv))
                                               if under_iv else None),
            "definitions": {
                "under": "body disc (r) overlaps lamp footprint and z + half_height + margin <= underside_z",
                "above_table": "body disc overlaps table-top footprint and z - half_height - margin >= top_z",
                "s": "arc length of the centre polyline, from the trajectory samples only"}}


# ----------------------------------------------------------------------------- calls

def _call(kind: str, index: int, scene, prepared, start, goal, config, preparation_s: float) -> dict:
    timing = PlanTiming()
    planner = LatticePlanner(prepared if kind == "warm" else None)
    call_id = f"{kind}{index}"
    faces = getattr(scene.known_space, "face_counts", None)
    if faces is not None:
        faces.clear()
    t0 = time.time()
    result, record = single_plan_call(planner, scene, start, goal, config, timer=timing.for_call(call_id))
    timing.finalize(result, mode=kind, call_id=call_id,
                    preparation_wall_s=preparation_s if kind == "warm" else 0.)
    return {"call_id": call_id, "mode": kind, "call_record": record, "result": result,
            "utc_started": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(t0)),
            "outer_wall_s": time.time() - t0, "pid": os.getpid(),
            "map_unknown_rejections_by_box_face": dict(faces or {})}


def _child(conn_path: str, *args):
    out = _call(*args)
    Path(conn_path).write_text(json.dumps(out, allow_nan=False))
    os._exit(0)


def _knots_sha(result) -> str | None:
    if not result.get("trajectory"):
        return None
    return hashlib.sha256(np.asarray(result["trajectory"]["poses"], float).round(9).tobytes()).hexdigest()


def run(spec_path: Path, output: Path, cold: int, warm: int) -> dict:
    spec = check_spec(json.loads(spec_path.read_text()))
    output.mkdir(parents=True, exist_ok=True)
    t_load = time.perf_counter()
    full, manifest_doc = su.load_uavlamp_derivative(spec["archive"], spec["manifest"])
    load_s = time.perf_counter() - t_load
    t_build = time.perf_counter()
    frame, scene, crop, dropped, extras = build_scene(spec, full, manifest_doc)
    del full
    build_s = time.perf_counter() - t_build
    prep_timer = PlanTiming()
    with prep_timer.preparation():
        prepared = PreparedScene(scene)
    start = Pose3(tuple(map(float, frame.to_world(spec["start_route"]))))
    goal = GoalRegion(Pose3(tuple(map(float, frame.to_world(spec["goal_route"])))))
    config = PlannerConfig(resolution_m=spec.get("resolution_m", .10), margin_m=spec.get("margin_m", .05),
                           seed=0, budget=SearchBudget(**spec.get("budget", {})))
    jobs = [("cold", k) for k in range(cold)] + [("warm", k) for k in range(warm)]
    if not jobs:
        raise ValueError("need at least one planner call")
    calls = []
    if len(jobs) == 1:
        calls.append(_call(*jobs[0], scene, prepared, start, goal, config, prep_timer.preparation_wall_s))
    else:
        ctx = mp.get_context("fork")
        procs = []
        for kind, k in jobs:
            path = output / f"call_{kind}{k}.json"
            proc = ctx.Process(target=_child, args=(str(path), kind, k, scene, prepared, start, goal,
                                                    config, prep_timer.preparation_wall_s))
            proc.start()
            procs.append((proc, path))
        for proc, path in procs:
            proc.join()
            if proc.exitcode != 0 or not path.exists():
                raise RuntimeError(f"planner call process {path.name} failed (exit {proc.exitcode})")
            calls.append(json.loads(path.read_text()))
    primary = next((c for c in calls if c["mode"] == "warm"), calls[0])
    result = primary["result"]

    replay = None
    if result["status"] == "success":
        t_replay = time.perf_counter()
        fresh = PreparedScene(scene)
        replay = replay_plan(result, GaussianBodyOracle(fresh))
        replay = {k: v for k, v in replay.items() if k != "samples"}
        replay["wall_s"] = time.perf_counter() - t_replay
        replay["oracle"] = "fresh GaussianBodyOracle on a freshly built PreparedScene of the same SceneSpec"

    evidence = None
    if result.get("trajectory"):
        samples = sample_linear_trajectory(result["trajectory"], dt_s=.02)
        route = frame.to_route(np.asarray(samples["poses"], float)[:, :3])
        evidence = ordered_evidence(route, lamp=spec["lamp"], table=spec["table"], radius=UAV.radius_m,
                                    half_height=UAV.half_height_m, margin=config.margin_m)
        evidence["knots_route"] = frame.to_route(
            np.asarray(result["trajectory"]["poses"], float)[:, :3]).round(4).tolist()
        evidence["clearance_lower_m_planner"] = result["clearance_lower_m"]
        evidence["clearance_lower_m_replay"] = (replay or {}).get("geometry", {}).get("clearance_lower_m")
        evidence["route_samples"] = route.round(5).tolist()

    by_mode = {m: [c["result"]["timings"]["algorithm_wall_s"] for c in calls if c["mode"] == m]
               for m in ("cold", "warm")}
    doc = {
        "schema": "uavlamp.l2_run.v1", "name": spec["name"], "spec_path": str(spec_path),
        "spec": spec, "archive_sha256": manifest_doc["derivative"]["sha256"],
        "manifest_checked_loader": "gmc.gs3d.scene_uavlamp.load_uavlamp_derivative",
        "scene_variants_test_only": {"dropped_roles": dropped, "extra_rows": extras},
        "crop": crop,
        "planner_calls_per_query": 1,
        "n_repeated_calls_for_timing": len(calls),
        "call_arguments_verbatim": primary["call_record"],
        "status": result["status"], "reason": result["reason"],
        "primary_call": primary["call_id"],
        "result": result,
        "replay": replay,
        "evidence": evidence,
        "all_calls": [{"call_id": c["call_id"], "mode": c["mode"], "status": c["result"]["status"],
                       "reason": c["result"]["reason"],
                       "algorithm_wall_s": c["result"]["timings"]["algorithm_wall_s"],
                       "preparation_wall_s": c["result"]["timings"]["preparation_wall_s"],
                       "expansions": c["result"]["diagnostics"]["expansions"],
                       "oracle_calls": c["result"]["diagnostics"].get("oracle_calls"),
                       "knots_sha256": _knots_sha(c["result"]), "utc_started": c["utc_started"],
                       "same_arguments_as_primary": c["call_record"]["arguments"] ==
                       primary["call_record"]["arguments"]} for c in calls],
        "timing": {
            "definition": "algorithm_wall_s = planner's own perf_counter from plan() entry to assembled "
                          "result (gs3d.timing). Excludes archive load/crop, rendering, replay and the "
                          "trajectory's physical time_s / control_dt_s.",
            "archive_load_and_hash_s": load_s, "scene_build_and_crop_s": build_s,
            "index_preparation_wall_s": prep_timer.preparation_wall_s,
            "cold_algorithm_wall_s": by_mode["cold"], "warm_algorithm_wall_s": by_mode["warm"],
            "warm_stats": latency_statistics(by_mode["warm"]),
            "calls_ran_concurrently": len(calls) > 1,
            "concurrency_note": "one forked process per call, one CPU each, same node" if len(calls) > 1 else None,
            "physical_trajectory_duration_s": (result["trajectory"]["time_s"][-1]
                                               if result.get("trajectory") else None),
            "physical_control_dt_s": (result["trajectory"]["control_dt_s"]
                                      if result.get("trajectory") else None)},
        "identical_routes_across_calls": len({_knots_sha(c["result"]) for c in calls}) == 1,
        "map_unknown_rejections_by_box_face": primary["map_unknown_rejections_by_box_face"],
        "host": {"node": platform.node(), "slurm_job_id": os.environ.get("SLURM_JOB_ID"),
                 "cpus": os.environ.get("SLURM_CPUS_PER_TASK"), "python": sys.version.split()[0]},
    }
    (output / "result.json").write_text(json.dumps(doc, allow_nan=False) + "\n")
    brief = {k: doc[k] for k in ("name", "status", "reason", "identical_routes_across_calls")}
    brief["timing"] = {k: doc["timing"][k] for k in ("cold_algorithm_wall_s", "warm_algorithm_wall_s",
                                                    "index_preparation_wall_s")}
    if evidence:
        brief["evidence"] = {k: evidence[k] for k in ("z_range", "under_precedes_above_table",
                                                      "passes_under_lamp", "path_length_m",
                                                      "clearance_lower_m_replay")}
    brief["replay_passed"] = (replay or {}).get("passed")
    brief["map_unknown_by_face"] = doc["map_unknown_rejections_by_box_face"]
    (output / "summary.json").write_text(json.dumps(brief, indent=1) + "\n")
    print(json.dumps(brief), flush=True)
    return doc


def smooth(spec_path: Path, raw_path: Path, output: Path) -> dict:
    """Optional A6 smoothing *after* the raw route is verified; the raw route stays primary."""
    from gmc.gs3d.smoothing import SmoothingConfig, optimize_trajectory, verify_piecewise_bezier
    spec = check_spec(json.loads(spec_path.read_text()))
    raw = json.loads(raw_path.read_text())
    if raw["status"] != "success" or not (raw.get("replay") or {}).get("passed"):
        raise ValueError("smoothing needs a verified raw route")
    output.mkdir(parents=True, exist_ok=True)
    full, manifest_doc = su.load_uavlamp_derivative(spec["archive"], spec["manifest"])
    frame, scene, *_ = build_scene(spec, full, manifest_doc)
    del full
    margin = float(raw["result"]["diagnostics"]["margin_m"])
    goal = GoalRegion(Pose3(tuple(raw["result"]["original_goal"][:3]), raw["result"]["original_goal"][3]))
    limits = _limits(UAV)
    config = SmoothingConfig(max_shortcut_span=64, max_certificate_depth=14, certificate_deviation_m=.005,
                             min_turn_improvement_fraction=.10)  # A6's UAV setting
    t0 = time.perf_counter()
    out = optimize_trajectory(raw["result"]["trajectory"], GaussianBodyOracle(PreparedScene(scene)), UAV,
                              limits, margin_m=margin, goal=goal, config=config)
    wall = time.perf_counter() - t0
    doc = {"schema": "uavlamp.l2_smooth.v1", "raw_result": str(raw_path), "optimizer_wall_s": wall,
           "selected": out.get("selected"), "success": out.get("success"), "reason": out.get("reason")}
    traj = out.get("trajectory")
    if out.get("selected") == "smoothed" and traj is not None:
        # Independent continuous re-verification with a fresh oracle / fresh index.
        again = verify_piecewise_bezier(traj, GaussianBodyOracle(PreparedScene(scene)), UAV, limits,
                                        margin_m=margin, goal=goal, config=config)
        doc["reverification"] = {k: again.get(k) for k in ("passed", "safety", "reason", "clearance_lower_m")}
        rows = np.asarray(traj["poses"], float)
        route = frame.to_route(rows[:, :3])
        doc["evidence"] = ordered_evidence(route, lamp=spec["lamp"], table=spec["table"], radius=UAV.radius_m,
                                           half_height=UAV.half_height_m, margin=margin)
        doc["evidence"]["route_samples"] = route.round(5).tolist()
    doc["optimizer"] = {k: v for k, v in out.items() if k not in ("trajectory", "candidate_trajectory")}
    doc["trajectory"] = traj
    (output / "smooth.json").write_text(json.dumps(doc, allow_nan=False, default=float) + "\n")
    print(json.dumps({k: doc.get(k) for k in ("selected", "success", "reason", "reverification")}), flush=True)
    return doc


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("spec", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--cold", type=int, default=0)
    p.add_argument("--warm", type=int, default=1)
    p.add_argument("--smooth-from", type=Path)
    a = p.parse_args(argv)
    if a.smooth_from:
        smooth(a.spec, a.smooth_from, a.output)
    else:
        run(a.spec, a.output, a.cold, a.warm)


if __name__ == "__main__":
    main()
