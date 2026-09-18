# gmc/experiments/plane_timing.py
"""Amendment 3 §P4: where the wall clock actually goes.

**Claims boundary:** every hall number here is on the plane-floor scene, a **user-approved manual
scene-definition change** (Amendment 3 P1), not a reconstruction method and not a guaranteed outer
approximation; results are sound with respect to the edited scene only. The case pairs reused here come
from P3's long case and cylinder ladder, which rest on criterion 3 as relaxed by user decision D1
(crit1 ∧ crit3 = 0.9 m² of the hall as built, 1.2 m² with every phantom cell deleted). τ, ρ, the robots,
``configs/height_showcase.yaml``, GMC, ``verify_curve`` and ``replay3d`` are untouched: this script only
*times* them.

Steps:

``sweep --lane L``  the one dedicated scaling sweep (one sbatch job runs the four lanes side by side, one
                    core each). It measures what no P1-P3 run could, because every one of them was one
                    compile followed by one query:

  * ``prep``     per-scene cost: PLY decode -> floor fit/rotate/crop, the Amendment 1 floor rule, the
                 five band rasters + phantom test, the plane-floor replacement, the npz write, the npz
                 load; hall-wide projections; then compile-only on the largest maps (51 k, 422 k,
                 703 k supports) to extend the compile curve past P2/P3's 156 k.
  * ``cyl``      the cylinder on ONE short pair (the ladder's rung 0, 2.43 m) while the window grows
                 7 -> 116 m². P3's ladder grew window and distance together; this separates them.
                 Stops at the first run that is not REACHABLE, as the ladder did.
  * ``sweeper``, ``uav``  one map (the long case's window), seven goals from 2.43 to 11.24 m:
                 **warm** = compile once, query all seven in turn, then all seven again on the same
                 (by now refined) map; **cold** = a fresh compile per goal. Then the rung-0 pair on a
                 growing window, as for the cylinder. The sweeper lane ends with the cylinder warm
                 experiment on rung 2's window (goals of rungs 0-2).

``selftest``        every lane's code path on ``synth3d.table_scene("open")``; the sbatch job runs it first.

Why warm vs cold matters: ``mobility.query.query`` refines the compiled map in place when an attempt
is UNKNOWN (``refine_compiler`` bisects an orientation slab and rebuilds the derived graph). With
``initial_intervals: 1`` a first query therefore carries compile work that a later query on the same map
may not have to redo. Whether "compile once, query cheaply" holds is measured here, not assumed.

Every lane appends one JSON object per measured unit to ``results/height/timing/sweep/<lane>.jsonl`` as
it goes, so a lane cut short by the wall clock still leaves its data.
"""
import argparse
import gc
import json
import math
import os
import socket
import time
from pathlib import Path

import numpy as np

from gmc.budget import WorkLedger
from gmc.config import load_config
from gmc.height.casesearch import d1_precheck, edt_clearance
from gmc.height.prism import robot_table
from gmc.height.project import project_scene
from gmc.height.replay3d import replay_curve
from gmc.height.timing import StageTimer
from gmc.height.timingreport import grow_window
from gmc.mobility.graph import compile_mobility
from gmc.mobility.query import query
from gmc.orientation.slab_builder import build_slabs
from gmc.spatial.bvh import query_candidate_pairs
from gmc.types import Pose2
from gmc.verification.path import verify_curve

RES = Path("results/height/timing")
PLANE_SCENE = Path("/scratch/wg2381/splathjb/gmc/outputs/height/plane/processed_planefloor.npz")
TMP = Path("/scratch/wg2381/splathjb/gmc/outputs/height/timing")
LONG_CASE = Path("results/height/plane/long/case.json")
LADDER = Path("results/height/plane/long/cyl_ladder.json")
CONFIG = "configs/height_showcase.yaml"
HALL = [-4.4, -0.95, 21.05, 32.0]          # P3's search domain: the hall below the y > 32 residual
TAU = 0.3
CLAIMS = ("plane floor = USER-APPROVED MANUAL SCENE EDIT (Amendment 3 P1), sound wrt the edited scene "
          "only; case pairs from P3 rest on criterion 3 relaxed by user decision D1 (crit1 & crit3 = "
          "0.9 m2 as built, 1.2 m2 with every phantom cell deleted)")

# Window-growth targets (m²) around the rung-0 pair. 144 = D2's 12 x 12 cap.
WIN_AREAS = {"sweeper": (0.0, 14.5, 29.0, 58.0, 116.0, 144.0),
             "uav": (0.0, 14.5, 29.0, 58.0, 116.0, 144.0),
             "cylinder": (0.0, 14.5, 29.0, 58.0, 116.0)}
# Guards. Compile is ~4.3 ms/support (P3, 9 runs, 3.8 k-156 k); the query estimate is what a unit may
# cost before it is allowed to start, not a budget -- GMC's own caps are the config's, unchanged.
COMPILE_S_PER_SUPPORT = 0.0065
QUERY_EST_S = {"sweeper": 600.0, "uav": 1500.0, "cylinder": 7200.0}
MEM_GUARD_GB = float(os.environ.get("P4_MEM_GUARD_GB", "18"))

OUT = RES / "sweep"
DEADLINE = float(os.environ.get("P4_DEADLINE_EPOCH", "inf"))


# ------------------------------------------------------------------------------------ plumbing
def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def mem_gb():
    out = {}
    try:
        for line in Path("/proc/self/status").read_text().splitlines():
            k, _, v = line.partition(":")
            if k in ("VmRSS", "VmHWM"):
                out[k] = int(v.split()[0]) / 1024 ** 2
    except OSError:
        pass
    return {"rss_gb": out.get("VmRSS"), "peak_gb": out.get("VmHWM")}


def reset_peak():
    """Reset VmHWM so the next peak is this unit's own, not the PLY decode's (Linux >= 4.0)."""
    try:
        Path("/proc/self/clear_refs").write_text("5")
        return True
    except OSError:
        return False


def host_info():
    model = None
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                model = line.split(":", 1)[1].strip()
                break
    except OSError:
        pass
    return {"host": socket.gethostname(), "cpu_model": model, "job": os.environ.get("SLURM_JOB_ID"),
            "threads": {k: os.environ.get(k) for k in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS")}}


def clean(o):
    if isinstance(o, dict):
        return {str(k): clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    if isinstance(o, (np.floating, float)):
        return None if not math.isfinite(float(o)) else float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o


class Sink:
    """Append-only JSONL per lane: a lane killed by the wall clock keeps every unit it finished."""

    def __init__(self, lane, out_dir):
        self.path = Path(out_dir) / f"{lane}.jsonl"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text("")
        self.lane = lane

    def write(self, row):
        row = clean(dict(row, lane=self.lane, t_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                         mem=mem_gb()))
        with self.path.open("a") as f:
            f.write(json.dumps(row) + "\n")
        log("ROW", json.dumps({k: row.get(k) for k in ("kind", "experiment", "mode", "robot",
                                                         "n_supports", "ab_dist_m", "status",
                                                         "success")}),
            json.dumps(row.get("stages")))


def fits_in_time(est_s):
    return time.time() + est_s <= DEADLINE


def area(w):
    return float((w[2] - w[0]) * (w[3] - w[1]))


def dist(a, b):
    return float(math.hypot(b[0] - a[0], b[1] - a[1]))


def _pose(v):
    return Pose2(np.asarray(v[:2], dtype=float), float(v[2]) if len(v) > 2 else 0.0)


# ------------------------------------------------------------------------------------ the pipeline
def project(tm, scene3d, robot, window, z_floor, label):
    with tm.stage("project", label=label, n_splats=len(scene3d), window=list(window)) as rec:
        s2, stats = project_scene(scene3d, robot, window, z_floor=z_floor, tau=TAU)
        rec["sizes"]["n_supports"] = stats["kept"]
    return s2


def compile_map(tm, s2, robot, cfg):
    """``run.compile_and_query``'s compile half, call for call, keeping the compiled map."""
    body = robot.footprint
    ledger = WorkLedger()
    with tm.stage("compile_pairs", n_supports=len(s2.supports)) as rec:
        oracles = list(query_candidate_pairs(s2, body, s2.workspace).oracles)
        rec["sizes"]["n_pairs"] = len(oracles)
    for o in oracles:
        o.ledger = ledger
    with tm.stage("compile_slabs", n_supports=len(s2.supports), n_pairs=len(oracles)) as rec:
        dec = build_slabs(s2, body, cfg, oracles, ledger=ledger)
        rec["sizes"]["n_slabs"] = len(dec.slabs)
    with tm.stage("compile_mobility", n_supports=len(s2.supports), n_pairs=len(oracles),
                  n_slabs=len(dec.slabs)):
        mc = compile_mobility(s2, body, cfg, oracles, dec, ledger=ledger)
    return {"oracles": oracles, "mc": mc, "workspace": s2.workspace, "n_supports": len(s2.supports)}


def _nslabs(mc):
    try:
        return len(mc.decomposition.slabs)
    except AttributeError:
        return None


def query_map(tm, cm, cfg, start, goal):
    """``run.compile_and_query``'s query + verify half on an already compiled (maybe refined) map."""
    mc = cm["mc"]
    s, g = _pose(start), _pose(goal)
    before = {"graph_revision": int(getattr(mc, "graph_revision", 0)), "n_slabs": _nslabs(mc),
              "n_safe_nodes": mc.M_safe.number_of_nodes(), "n_safe_edges": mc.M_safe.number_of_edges()}
    with tm.stage("query", n_slabs=before["n_slabs"], n_safe_nodes=before["n_safe_nodes"],
                  n_safe_edges=before["n_safe_edges"]) as rec:
        result = query(s, g, mc)
        rec["sizes"]["status"] = result.status.name
    rep = result.report or {}
    out = {"status": result.status.name, "clearance_lb": result.clearance_lower_bound,
           "reason": rep.get("reason"),
           "query_report": {"refinement_rounds": rep.get("refinement_rounds"),
                            "query_support_calls": rep.get("query_support_calls"),
                            "wall_s": rep.get("wall_s"),
                            "graph_revision_before": before["graph_revision"],
                            "graph_revision_after": int(getattr(mc, "graph_revision", 0)),
                            "n_slabs_before": before["n_slabs"], "n_slabs_after": _nslabs(mc)},
           "verify": {"ran": False}, "curve": result.curve}
    if result.curve is not None:
        with tm.stage("verify_curve", n_pairs=len(cm["oracles"]),
                      n_segments=len(result.curve.segments)):
            vr = verify_curve(tuple(cm["oracles"]), cm["workspace"], result.curve, cfg.query.eps_clear,
                              cfg.orientation.theta_min, expected_start=s, expected_goal=g)
        out["verify"] = {"ran": True, "certified": bool(vr.certified),
                         "min_clearance": float(vr.min_clearance), "reason": vr.reason}
    return out


def replay(tm, scene3d, robot, curve, z_floor):
    with tm.stage("replay3d", n_splats=len(scene3d), n_segments=len(curve.segments)):
        return replay_curve(scene3d, robot, curve, z_floor=z_floor, tau=TAU)


def unit_row(tm, *, experiment, mode, robot, window, start, goal, n_supports, q=None, rp=None, **extra):
    row = {"kind": "unit", "source": "P4-sweep", "experiment": experiment, "mode": mode, "robot": robot,
           "window": list(window) if window is not None else None,
           "window_area_m2": area(window) if window is not None else None,
           "start": list(start[:2]) if start is not None else None,
           "goal": list(goal[:2]) if goal is not None else None,
           "ab_dist_m": dist(start, goal) if start is not None and goal is not None else None,
           "n_supports": n_supports, "stages": tm.by_stage(), "timing": tm.to_dict()}
    if q is not None:
        ver = q["verify"]
        row.update(status=q["status"], reason=q["reason"], clearance_lb=q["clearance_lb"],
                   certified=bool(ver.get("certified", False)), verify=ver,
                   query_report=q["query_report"],
                   replay3d_passed=bool(rp and rp.get("passed")),
                   replay3d_lb=(rp or {}).get("min_clearance_lb"))
        row["success"] = bool(row["status"] == "REACHABLE" and row["certified"] and row["replay3d_passed"])
        row["query_is_lower_bound"] = row["status"] == "UNKNOWN"
    row.update(extra)
    return row


def one_pair(ctx, sink, *, experiment, mode, robot_name, s2, window, start, goal, cm=None, extra=None,
             tm=None):
    """Compile (unless ``cm`` is given: a warm query) -> query -> verify -> replay3d, one row.

    ``tm`` lets a caller that projected this map for this unit alone keep ``project`` in the same row.
    """
    robot = ctx["robots"][robot_name]
    tm = tm if tm is not None else StageTimer()
    own = cm is None
    if own:
        cm = compile_map(tm, s2, robot, ctx["cfg"])
    q = query_map(tm, cm, ctx["cfg"], start, goal)
    rp = None
    if q["curve"] is not None:
        rp = replay(tm, ctx["scene"], robot, q["curve"], ctx["z_floor"])
    row = unit_row(tm, experiment=experiment, mode=mode, robot=robot_name, window=window, start=start,
                   goal=goal, n_supports=len(s2.supports), q=q, rp=rp, **(extra or {}))
    sink.write(row)
    return row, (cm if own else None)


# ------------------------------------------------------------------------------------ lanes
def d1_rows(ctx, s2, robot_name, window, start, goals):
    """D1 on this robot's certified map for every goal: which pairs are admissible, recorded not assumed."""
    from showcase_scene import _support_raster
    t0 = time.perf_counter()
    occ = _support_raster(s2, window, ctx["raster"])
    d = edt_clearance(occ, ctx["raster"])
    r = ctx["robots"][robot_name].max_radius()
    out = [d1_precheck(d, window, ctx["raster"], start, g, r) for g in goals]
    return out, time.perf_counter() - t0


def lane_dist(ctx, sink, robot_name):
    """One map, goals at 2.4-11.2 m: warm (compile once, two passes) then cold (compile per goal)."""
    win, start, goals = ctx["dist_window"], ctx["start"], ctx["dist_goals"]
    robot = ctx["robots"][robot_name]
    tm = StageTimer()
    s2 = project(tm, ctx["scene"], robot, win, ctx["z_floor"], f"dist:{robot_name}")
    d1, d1_s = d1_rows(ctx, s2, robot_name, win, start, goals)
    sink.write(unit_row(tm, experiment="dist", mode="project", robot=robot_name, window=win, start=None,
                        goal=None, n_supports=len(s2.supports), d1=d1, d1_raster_seconds=d1_s,
                        goals=goals, goal_dists=[dist(start, g) for g in goals]))
    n = len(s2.supports)
    # warm: one compile, then every goal, twice
    if fits_in_time(COMPILE_S_PER_SUPPORT * n + 2 * len(goals) * QUERY_EST_S[robot_name] / 4):
        tm = StageTimer()
        cm = compile_map(tm, s2, robot, ctx["cfg"])
        sink.write(unit_row(tm, experiment="dist", mode="warm_compile", robot=robot_name, window=win,
                            start=None, goal=None, n_supports=n))
        for p in (1, 2):
            for k, g in enumerate(goals):
                if not fits_in_time(QUERY_EST_S[robot_name]):
                    sink.write({"kind": "skipped", "experiment": "dist", "mode": f"warm{p}",
                                "robot": robot_name, "goal": g, "reason": "deadline"})
                    continue
                one_pair(ctx, sink, experiment="dist", mode=f"warm{p}", robot_name=robot_name, s2=s2,
                         window=win, start=start, goal=g, cm=cm, extra={"order": k, "d1": d1[k]})
        del cm
        gc.collect()
    # cold: a fresh compile per goal; the long goal first, as the calibration against P3's own run
    order = [len(goals) - 1] + list(range(len(goals) - 1))
    for k in order:
        if not fits_in_time(COMPILE_S_PER_SUPPORT * n + QUERY_EST_S[robot_name]):
            sink.write({"kind": "skipped", "experiment": "dist", "mode": "cold", "robot": robot_name,
                        "goal": goals[k], "reason": "deadline"})
            continue
        one_pair(ctx, sink, experiment="dist", mode="cold", robot_name=robot_name, s2=s2, window=win,
                 start=start, goal=goals[k], extra={"d1": d1[k]})
        gc.collect()


def lane_win(ctx, sink, robot_name):
    """One short pair (rung 0, 2.43 m), window grown around it. Stops at the first non-REACHABLE."""
    base, start, goal = ctx["win_base"], ctx["start"], ctx["win_goal"]
    robot = ctx["robots"][robot_name]
    for a in WIN_AREAS[robot_name]:
        w = grow_window(base, a, ctx["bounds"]) if a > 0 else list(base)
        if w is None:
            sink.write({"kind": "skipped", "experiment": "win", "robot": robot_name, "target_area": a,
                        "reason": "bounds cannot hold the area"})
            continue
        tm = StageTimer()
        s2 = project(tm, ctx["scene"], robot, w, ctx["z_floor"], f"win:{robot_name}")
        n = len(s2.supports)
        if not fits_in_time(COMPILE_S_PER_SUPPORT * n + QUERY_EST_S[robot_name] + 300):
            sink.write(unit_row(tm, experiment="win", mode="skipped_deadline", robot=robot_name, window=w,
                                start=start, goal=goal, n_supports=n))
            break
        row, _ = one_pair(ctx, sink, experiment="win", mode="cold", robot_name=robot_name, s2=s2, window=w,
                          start=start, goal=goal, extra={"target_area": a}, tm=tm)
        del s2
        gc.collect()
        if row["status"] != "REACHABLE":
            log(f"win {robot_name}: {row['status']} at {n} supports -> stop (as P3's ladder did)")
            break


def lane_cyl_warm(ctx, sink):
    """The cylinder, where the query dominates: compile rung 2's window once, query rungs 0-2 twice."""
    win, start, goals = ctx["cyl_warm_window"], ctx["start"], ctx["cyl_warm_goals"]
    robot = ctx["robots"]["cylinder"]
    tm = StageTimer()
    s2 = project(tm, ctx["scene"], robot, win, ctx["z_floor"], "cylwarm:cylinder")
    n = len(s2.supports)
    if not fits_in_time(COMPILE_S_PER_SUPPORT * n + 3 * 900):
        sink.write(unit_row(tm, experiment="cyl_warm", mode="skipped_deadline", robot="cylinder",
                            window=win, start=None, goal=None, n_supports=n))
        return
    cm = compile_map(tm, s2, robot, ctx["cfg"])
    sink.write(unit_row(tm, experiment="cyl_warm", mode="warm_compile", robot="cylinder", window=win,
                        start=None, goal=None, n_supports=n))
    for p in (1, 2):
        for k, g in enumerate(goals):
            if not fits_in_time(1500):
                sink.write({"kind": "skipped", "experiment": "cyl_warm", "mode": f"warm{p}", "goal": g,
                            "reason": "deadline"})
                continue
            one_pair(ctx, sink, experiment="cyl_warm", mode=f"warm{p}", robot_name="cylinder", s2=s2,
                     window=win, start=start, goal=g, cm=cm, extra={"order": k})


def compile_only(ctx, sink, robot_name, window, label):
    """Compile a big map and stop: extends the compile curve. Guarded on time and on memory."""
    robot = ctx["robots"][robot_name]
    tm = StageTimer()
    s2 = project(tm, ctx["scene"], robot, window, ctx["z_floor"], f"{label}:{robot_name}")
    n = len(s2.supports)
    est_s = COMPILE_S_PER_SUPPORT * n
    m = mem_gb()
    per = ctx.get("compile_gb_per_support")
    est_gb = (m["rss_gb"] or 0) + (per * n * 1.2 if per else 0)
    if not fits_in_time(est_s) or (per and est_gb > MEM_GUARD_GB):
        sink.write(unit_row(tm, experiment="compile_only", mode="skipped_guard", robot=robot_name,
                            window=window, start=None, goal=None, n_supports=n, label=label,
                            est_seconds=est_s, est_gb=est_gb, mem_guard_gb=MEM_GUARD_GB))
        return
    rss0 = m["rss_gb"] or 0.0
    reset_peak()
    try:
        cm = compile_map(tm, s2, robot, ctx["cfg"])
        err = None
    except MemoryError as e:            # recorded, never retried
        cm, err = None, f"MemoryError: {e}"
    m1 = mem_gb()
    grow = max(0.0, (m1["peak_gb"] or 0) - rss0)
    if n:
        ctx["compile_gb_per_support"] = max(ctx.get("compile_gb_per_support") or 0.0, grow / n)
    sink.write(unit_row(tm, experiment="compile_only", mode="compile", robot=robot_name, window=window,
                        start=None, goal=None, n_supports=n, label=label, error=err,
                        rss_before_gb=rss0, peak_growth_gb=grow))
    del cm, s2
    gc.collect()


def lane_prep(ctx, sink, selftest=False):
    """Per-scene cost, measured once: decode, denoise, write, load; then projections and big compiles."""
    from gmc.height.floor import apply_floor_rule
    from gmc.height.planefloor import build_plane_floor, load_plane_scene, phantom_cell_mask, save_plane_scene
    from showcase_scene import BANDS, CELL, DATA, _occupancy, load_processed

    tm = StageTimer()
    checks = {}
    if not selftest:
        from showcase_scene import _dense_peaks
        from gmc.height.ply3d import crop_box, fit_floor, gravity_rotation, load_3dgs_ply, rotate_scene
        with tm.stage("scene_load", label="ply_decode", path="raw/point_cloud.ply") as rec:
            sc, st = load_3dgs_ply(DATA / "raw" / "point_cloud.ply", name="showcase")
            rec["sizes"]["n_splats"] = len(sc)
        with tm.stage("scene_load", label="floor_fit_rotate_crop") as rec:      # showcase_scene.step_decode
            f0 = fit_floor(sc)
            if f0["tilt_deg"] > 0.5:
                sc = rotate_scene(sc, gravity_rotation(f0["normal"]), f0["centroid"])
            f1 = fit_floor(sc)
            z_f = f1["z_floor"]
            opq = sc.means[sc.opacity > 0.5, 2]
            above = opq[opq > z_f + 1.5]
            ceiling = max(_dense_peaks(above)) if len(above) else float(np.percentile(opq, 99.5))
            sc, _ = crop_box(sc, np.array([-np.inf, -np.inf, z_f - 0.2]),
                             np.array([np.inf, np.inf, ceiling + 0.2]))
            rec["sizes"]["n_splats"] = len(sc)
        checks["decode_n_splats"] = len(sc)
        del sc, opq, above
        gc.collect()
        with tm.stage("scene_load", label="processed_npz") as rec:
            scene, g0 = load_processed(floor_rule=False)
            rec["sizes"]["n_splats"] = len(scene)
        checks["processed_npz_n_splats"] = len(scene)
        checks["decode_matches_processed_npz"] = checks["decode_n_splats"] == len(scene)
        floor = g0["floor"]
        ext = [float(v) for v in np.load(DATA / "maps.npz")["extent"]]
    else:
        scene, floor, ext = ctx["scene"], ctx["floor"], list(ctx["bounds"])
    z_f = floor["z_floor"]
    with tm.stage("floor_replace", label="amendment1_floor_rule", n_splats=len(scene)) as rec:
        scene, st = apply_floor_rule(scene, floor)
        rec["sizes"]["removed"] = st["removed"]
    checks["after_floor_rule_n_splats"] = len(scene)
    with tm.stage("floor_replace", label="band_rasters_and_phantom", n_splats=len(scene),
                  n_bands=len(BANDS), cell=CELL) as rec:
        occ = {b: _occupancy(scene, z_f, b, ext) for b in BANDS}
        phantom = phantom_cell_mask(occ[BANDS[0]], [occ[b] for b in BANDS[1:]])
        rec["sizes"]["n_cells"] = int(phantom.size)
        rec["sizes"]["phantom_cells"] = int(phantom.sum())
    checks["phantom_cells"] = int(phantom.sum())
    from planefloor_build import P1B
    with tm.stage("floor_replace", label="replace_and_plane", n_splats=len(scene),
                  footprint="centre") as rec:
        built, st = build_plane_floor(scene, phantom, ext, CELL, floor=floor, footprint="centre", **P1B)
        rec["sizes"].update(replaced=st["replaced"], inserted=st["inserted"], n_output=st["n_output"])
    checks["planefloor_n_splats"] = st["n_output"]
    tmp = (TMP if not selftest else ctx["out"]) / f"_p4_tmp_planefloor_{os.getpid()}.npz"
    with tm.stage("floor_replace", label="save_npz", n_splats=len(built)) as rec:
        save_plane_scene(tmp, built, {"floor": floor, "z_floor": z_f, "note": "P4 timing copy, deleted"})
        rec["sizes"]["bytes"] = tmp.stat().st_size
    tmp.unlink()
    del scene, built, occ, phantom
    gc.collect()
    if not selftest:
        with tm.stage("scene_load", label="planefloor_npz") as rec:
            scene, _ = load_plane_scene(PLANE_SCENE)
            rec["sizes"]["n_splats"] = len(scene)
        ctx["scene"] = scene
    if not selftest:
        checks["expected"] = {"processed_npz_n_splats": 7319425, "after_floor_rule_n_splats": 7247831,
                              "phantom_cells": 94780, "planefloor_n_splats": 7101868}
        checks["all_match"] = all(checks.get(k) == v for k, v in checks["expected"].items())
    sink.write(unit_row(tm, experiment="prep", mode="per_scene", robot=None, window=None, start=None,
                        goal=None, n_supports=None, checks=checks))
    # hall-wide projections (the project stage at its largest), then compile-only on the biggest maps
    for rn in ("sweeper", "uav", "cylinder"):
        tm = StageTimer()
        s2 = project(tm, ctx["scene"], ctx["robots"][rn], ctx["bounds"], ctx["z_floor"], f"hall:{rn}")
        sink.write(unit_row(tm, experiment="project", mode="hall", robot=rn, window=ctx["bounds"],
                            start=None, goal=None, n_supports=len(s2.supports)))
        del s2
        gc.collect()
    for rn, win, label in ctx["compile_only"]:
        compile_only(ctx, sink, rn, win, label)


# ------------------------------------------------------------------------------------ contexts
def hall_ctx(load=True):
    """The hall. ``load=False`` for the prep lane, which loads (and times) the scene itself."""
    from gmc.height.planefloor import load_plane_scene
    long_case = json.loads(LONG_CASE.read_text())
    lad = json.loads(LADDER.read_text())
    rung = {r["rung"]: r for r in lad["rungs"]}
    scene = None
    if load:
        t0 = time.perf_counter()
        scene, _ = load_plane_scene(PLANE_SCENE)
        log(f"scene {len(scene):,} splats in {time.perf_counter() - t0:.1f}s")
    goals = [rung[k]["goal"] for k in range(6)] + [long_case["goal"][:2]]
    return {"scene": scene, "z_floor": float(long_case["z_floor"]), "robots": robot_table(long_case["z_c"]),
            "cfg": load_config(CONFIG), "raster": 0.025, "bounds": list(HALL),
            "start": long_case["start"][:2],
            "dist_window": list(long_case["window"]), "dist_goals": goals,
            "win_base": list(rung[0]["window"]), "win_goal": rung[0]["goal"],
            "cyl_warm_window": list(rung[2]["window"]), "cyl_warm_goals": [rung[k]["goal"] for k in range(3)],
            "compile_only": [("sweeper", list(HALL), "hall"), ("uav", list(HALL), "hall"),
                             ("cylinder", list(lad["ladder_map_window"]), "ladder_map")],
            "out": OUT}


def synth_ctx():
    """The table scene, scaled down: every lane's code path in seconds (``--step selftest``)."""
    global WIN_AREAS
    from gmc.height.synth3d import GOAL, START, WORKSPACE, Z_FLOOR, table_scene
    scene, _ = table_scene("open")
    WIN_AREAS = {"sweeper": (0.0, 12.0), "uav": (0.0, 12.0), "cylinder": (0.0, 12.0)}
    ws = list(WORKSPACE)
    return {"scene": scene, "z_floor": Z_FLOOR, "robots": robot_table(1.2), "cfg": load_config(CONFIG),
            "raster": 0.025, "bounds": ws, "start": list(START[:2]),
            "floor": {"z_floor": Z_FLOOR, "normal": [0.0, 0.0, 1.0], "centroid": [0.0, 0.0, Z_FLOOR],
                      "tilt_deg": 0.0},
            "dist_window": ws, "dist_goals": [[-0.8, 0.0], list(GOAL[:2])],
            "win_base": [-2.6, -0.6, 0.6, 0.6], "win_goal": [-0.8, 0.0],
            "cyl_warm_window": ws, "cyl_warm_goals": [[-1.2, 1.4], list(GOAL[:2])],
            "compile_only": [("sweeper", ws, "synth")],
            "out": RES / "_selftest"}


def run_lane(ctx, lane, out_dir, selftest=False):
    sink = Sink(lane, out_dir)
    sink.write({"kind": "lane_meta", "claims_boundary": CLAIMS, "host_info": host_info(),
                "deadline_epoch": DEADLINE if math.isfinite(DEADLINE) else None, "config": CONFIG,
                "selftest": selftest})
    t0 = time.time()
    if lane == "prep":
        lane_prep(ctx, sink, selftest=selftest)
    elif lane == "cyl":
        lane_win(ctx, sink, "cylinder")
    elif lane == "sweeper":
        lane_dist(ctx, sink, "sweeper")
        lane_win(ctx, sink, "sweeper")
        lane_cyl_warm(ctx, sink)
    elif lane == "uav":
        lane_dist(ctx, sink, "uav")
        lane_win(ctx, sink, "uav")
    sink.write({"kind": "lane_done", "seconds": time.time() - t0})
    log(f"LANE {lane} DONE in {time.time() - t0:.0f}s -> {sink.path}")
    return sink.path


def step_selftest(args):
    ctx = synth_ctx()
    out = ctx["out"]
    fail = []
    for lane in ("prep", "cyl", "sweeper", "uav"):
        path = run_lane(ctx, lane, out, selftest=True)
        rows = [json.loads(l) for l in path.read_text().splitlines()]
        if not rows or rows[-1].get("kind") != "lane_done":
            fail.append(f"{lane}: no lane_done")
        units = [r for r in rows if r.get("kind") == "unit"]
        if lane != "prep" and not any(r.get("success") for r in units):
            fail.append(f"{lane}: no certified + replayed unit on a scene built to have one")
        for r in units:
            if r.get("status") and not r["stages"].get("query"):
                fail.append(f"{lane}: a query row without a query stage")
        if lane == "sweeper":
            modes = {r.get("mode") for r in units}
            for m in ("warm_compile", "warm1", "warm2", "cold"):
                if m not in modes:
                    fail.append(f"sweeper: mode {m} never ran")
            warm = [r for r in units if r.get("mode", "").startswith("warm") and r.get("status")]
            if any("compile_slabs" in r["stages"] for r in warm):
                fail.append("a warm query row carries a compile stage")
    log("selftest rows in", out)
    if fail:
        raise SystemExit("SELFTEST FAILED: " + "; ".join(fail))
    log("SELFTEST PASSED")


def step_sweep(args):
    run_lane(hall_ctx(load=args.lane != "prep"), args.lane, OUT)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", required=True, choices=["sweep", "selftest"])
    ap.add_argument("--lane", choices=["prep", "cyl", "sweeper", "uav"])
    args = ap.parse_args()
    {"sweep": step_sweep, "selftest": step_selftest}[args.step](args)


if __name__ == "__main__":
    main()
