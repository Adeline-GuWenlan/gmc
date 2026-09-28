"""aerial3d with the ground bodies (sweeper, cylinder) on the real uav-lamp archive (G1).

Subcommands (run from ``gmc/``, ``PYTHONPATH=src:experiments``):

* ``probe``   compile each robot on a (small) route box; per-substep time / peak RSS /
              candidate pairs.  No queries.
* ``screen``  compile one robot on the full booth box once, persist it, then query a
              deterministic candidate-pair list (the same list in every job: both robots'
              endpoints are checked free with the gs3d oracle) -- used to pick the demo pair.
* ``demo``    compile one robot once; query one pair cold + N warm on the same in-memory
              compile; persist, reload, query again; fresh gs3d replay; full timing.

The scene is ``uavlamp_scene.npz`` through the manifest-checked loader, cropped with the
baseline's ``uavlamp_query.build_scene`` to the manifest's route prism, plus the constant
floor support of ``gs3d_run._constant_floor_support`` (z_floor = archive meta = site-frame
origin z).  Ground bodies are ``gs3d.robots.SWEEPER`` / ``CYLINDER`` at margin 0.001
(docs/aerial3dg_design.md).
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import hashlib
import json
import os
from pathlib import Path
import platform
import resource
import sys
import time

import numpy as np

from gmc.aerial3d.api import CompileConfig, QueryConfig, compile_complex, load_compiled, query, save_compiled
from gmc.gs3d import scene_uavlamp as su
from gmc.gs3d.contracts import Pose3
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.planner import _json_finite
from gmc.gs3d.robots import CYLINDER, SWEEPER, BoxKnownSpace, EvidenceBoundedPlaneSupport
from gmc.gs3d.trajectory import replay_plan

from uavlamp_query import build_scene

UAVLAMP = Path("/scratch/wg2381/splathjb-uavlamp/gmc/outputs/uavlamp/scene_v2")
ARCHIVE, MANIFEST = UAVLAMP / "uavlamp_scene.npz", UAVLAMP / "manifest.json"
FROZEN_SHA256 = "2a3a72d610519095381a83fd741a61f32b420b20f39e0c5f60951e463fd596cc"
ROBOTS = {"sweeper": SWEEPER, "cylinder": CYLINDER}
GROUND_CONFIG = CompileConfig(margin_m=.001)
# Only the exported gs3d trajectory is split (collinear, <= 0.20 m, uavconn's post-hoc length): the
# shared oracle's world-AABB swept test otherwise reports map_unknown on long corridor segments.
QCONFIG = QueryConfig(export_max_segment_m=.20)
TABLE_UV = ((-.02, -.03), (2.97, .79))      # manifest query.table.top_route_uv
LAMP_U = (-1.03, -.67)                      # manifest lamp footprint u range


def rss_mb() -> float:
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.


def host() -> dict:
    return {"node": platform.node(), "slurm_job_id": os.environ.get("SLURM_JOB_ID"),
            "cpus_per_task": os.environ.get("SLURM_CPUS_PER_TASK"),
            "affinity_cpus": len(os.sched_getaffinity(0)), "loadavg": list(os.getloadavg()),
            "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "python": sys.version.split()[0]}


def floor_support(meta: dict, bmin, bmax):
    """Constant floor at meta z_floor (gs3d_run._constant_floor_support), evidence = world bounds."""
    floor, z_floor = meta["floor"], float(meta["z_floor"])
    n, c = np.asarray(floor["normal"], float), np.asarray(floor["centroid"], float)
    corners = np.array([[x, y] for x in (bmin[0], bmax[0]) for y in (bmin[1], bmax[1])])
    fitted = c[2] - (n[0] * (corners[:, 0] - c[0]) + n[1] * (corners[:, 1] - c[1])) / n[2]
    dev = float(np.max(np.abs(fitted - z_floor)))
    support = EvidenceBoundedPlaneSupport((0., 0., 1.), (0., 0., z_floor), BoxKnownSpace(tuple(bmin), tuple(bmax)),
                                          height_error_m=dev, max_height_error_m=.05, max_travel_m=.05,
                                          max_slope_deg=5.)
    return support, {"contact_manifold": "constant_legacy_floor_reference", "z_floor_world_m": z_floor,
                     "fitted_plane_max_corner_deviation_m": dev, "allowed_height_error_m": .05,
                     "tile_top_clamp_m": z_floor + .015, "chassis_bottom_m": z_floor + .02}


def load_booth(box_route: dict | None = None) -> dict:
    t0 = time.perf_counter()
    full, man = su.load_uavlamp_derivative(ARCHIVE, MANIFEST)       # hashes the file vs its manifest
    load_s = time.perf_counter() - t0
    digest = man["derivative"]["sha256"]
    if digest != FROZEN_SHA256:
        raise ValueError(f"archive hash {digest} is not the frozen {FROZEN_SHA256}")
    with np.load(ARCHIVE, allow_pickle=False) as d:
        meta = json.loads(str(d["meta"]))
    if abs(float(meta["z_floor"]) - float(man["frame"]["origin_world_m"][2])) > 1e-12:
        raise ValueError("archive z_floor differs from the site frame's origin z (two floor conventions)")
    box = box_route or man["query"]["box_route"]
    spec = {"name": "uavlamp_gallery_booth_v2_ground", "frame": man["frame"], "box_route": box}
    t1 = time.perf_counter()
    frame, scene, crop, _, _ = build_scene(spec, full, man)
    del full
    support, evidence = floor_support(meta, scene.bounds_min, scene.bounds_max)
    scene = replace(scene, support=support,
                    provenance={**scene.provenance, "support_evidence": evidence, "archive_sha256": digest})
    return {"frame": frame, "scene": scene, "crop": crop, "box_route": box, "digest": digest,
            "support_evidence": evidence, "manifest": man, "archive_load_and_hash_s": load_s,
            "scene_build_and_crop_s": time.perf_counter() - t1}


def ground_world(frame, body, uv) -> np.ndarray:
    """World body centre for route (u, v) on the floor: z = clearance + half_height above floor."""
    return frame.to_world([uv[0], uv[1], body.ground_clearance_m + body.half_height_m])


def compile_record(compiled, outer_s, cpu_s) -> dict:
    s = compiled.summary()
    return {"compile_id": compiled.compile_id, "body": asdict(compiled.body),
            "config": s["config"], "domain_bbox_plan": [s["domain"]["bbox_lower"], s["domain"]["bbox_upper"]],
            "ground_z_plan": s["domain"].get("ground_z_plan"),
            "pairs": s["pairs"], "octree": {k: s["octree"].get(k) for k in
                                            ("nodes", "leaves", "leaves_by_status", "volume_m3_by_status",
                                             "root_shape", "leaf_unit_m", "build_wall_s")},
            "cells": {k: s["cells"].get(k) for k in ("cells", "support_plane_cells", "box_cells", "portals",
                                                     "portals_3d", "portals_2d", "planes", "seed_failures",
                                                     "cell_growth_wall_s", "build_wall_s")},
            "possible_components": s["possible_components"], "audit": {k: s["audit"].get(k) for k in
                                                                       ("passed", "pairs_audited")},
            "traceability": s["traceability"], "free_boundary_traceability": s["free_boundary_traceability"],
            "compile_wall_s": s["timings"]["compile_wall_s"], "compile_outer_wall_s": outer_s,
            "compile_cpu_s": cpu_s, "peak_rss_mb": s["timings"]["peak_rss_mb"],
            "stages": [{"stage": r["stage"], "seconds": r["seconds"], "peak_rss_mb": r["sizes"].get("peak_rss_mb"),
                        "sizes": {k: v for k, v in r["sizes"].items() if k != "peak_rss_mb"}}
                       for r in s["timings"]["records"]]}


def timed_compile(scene, body, prepared=None):
    t0, c0 = time.perf_counter(), time.process_time()
    compiled = compile_complex(scene, body, config=GROUND_CONFIG, prepared=prepared)
    return compiled, time.perf_counter() - t0, time.process_time() - c0


def _dump(path: Path, doc):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_json_finite(doc), indent=1, default=float) + "\n")


# ----------------------------------------------------------------------------- probe
def cmd_probe(a):
    box = {"lower": a.box[:2] + [0.], "upper": a.box[2:] + [2.43]}
    ctx = load_booth(box)
    out = {"schema": "aerial3dg.probe.v1", "box_route": box, "crop": ctx["crop"], "host": host(),
           "archive_load_and_hash_s": ctx["archive_load_and_hash_s"],
           "scene_build_and_crop_s": ctx["scene_build_and_crop_s"], "support": ctx["support_evidence"],
           "robots": {}}
    for name in a.robots:
        compiled, outer, cpu = timed_compile(ctx["scene"], ROBOTS[name])   # scene_prepare inside
        out["robots"][name] = compile_record(compiled, outer, cpu)
        print(name, json.dumps({k: out["robots"][name][k] for k in ("compile_wall_s", "peak_rss_mb")},
                               default=float), json.dumps(out["robots"][name]["pairs"], default=float), flush=True)
        _dump(a.out / "probe.json", out)
        del compiled


# ----------------------------------------------------------------------------- map
def free_mask(oracle, frame, body, grid, margin=GROUND_CONFIG.margin_m) -> np.ndarray:
    out = np.zeros(len(grid), dtype=bool)
    for k, uv in enumerate(grid):
        q = Pose3(tuple(map(float, ground_world(frame, body, uv))), 0.)
        r = oracle.edge(q, q, body, margin_m=margin)
        out[k] = r.occupancy == "free" and r.safety == "continuous_bound"
    return out


def cmd_map(a):
    box = {"lower": a.box[:2] + [0.], "upper": a.box[2:] + [2.43]}
    ctx = load_booth(box)
    frame = ctx["frame"]
    oracle = GaussianBodyOracle(PreparedScene(ctx["scene"]))
    us = np.round(np.arange(np.ceil(box["lower"][0] / a.step) * a.step, box["upper"][0], a.step), 6)
    vs = np.round(np.arange(np.ceil(box["lower"][1] / a.step) * a.step, box["upper"][1], a.step), 6)
    grid = np.array([(u, v) for u in us for v in vs])
    masks = {n: free_mask(oracle, frame, b, grid) for n, b in ROBOTS.items()}
    _dump(a.out / "map.json", {"schema": "aerial3dg.map.v1", "box_route": box, "step": a.step, "crop": ctx["crop"],
                               "us": us.tolist(), "vs": vs.tolist(),
                               "free": {n: m.reshape(len(us), len(vs)).tolist() for n, m in masks.items()},
                               "definition": "gs3d oracle point check at each robot's own z_c, margin .001"})
    code = {(True, True): "#", (True, False): "s", (False, True): "c", (False, False): "."}
    S, C = masks["sweeper"].reshape(len(us), len(vs)), masks["cylinder"].reshape(len(us), len(vs))
    print("legend: # both free, s sweeper only, c cylinder only, . neither; rows v (top = max), cols u", flush=True)
    for j in range(len(vs) - 1, -1, -1):
        print(f"{vs[j]:6.2f} " + "".join(code[(bool(S[i, j]), bool(C[i, j]))] for i in range(len(us))), flush=True)
    print(f"u from {us[0]} to {us[-1]} step {a.step}", flush=True)


# ----------------------------------------------------------------------------- candidates
def free_both(oracle, frame, uv, margin=GROUND_CONFIG.margin_m) -> bool:
    for body in ROBOTS.values():
        q = Pose3(tuple(map(float, ground_world(frame, body, uv))), 0.)
        r = oracle.edge(q, q, body, margin_m=margin)
        if r.occupancy != "free" or r.safety != "continuous_bound":
            return False
    return True


def _crosses_rect(p, q, lo, hi, n=200) -> bool:
    t = np.linspace(0, 1, n)[:, None]
    s = p[None] + t * (q - p)[None]
    return bool(np.any(np.all((s >= lo) & (s <= hi), axis=1)))


def candidate_pairs(ctx, *, step=.1, n_table=40, n_lamp=10, n_other=30, seed=0, min_dist=3.0) -> dict:
    """Deterministic: 0.1 m route grid, points free for BOTH robots (gs3d oracle at each own z_c)."""
    frame, scene, box = ctx["frame"], ctx["scene"], ctx["box_route"]
    oracle = GaussianBodyOracle(PreparedScene(scene))
    us = np.arange(np.ceil(box["lower"][0] / step) * step, box["upper"][0], step)
    vs = np.arange(np.ceil(box["lower"][1] / step) * step, box["upper"][1], step)
    grid = np.array([(u, v) for u in us for v in vs]).round(6)
    free = np.array([free_both(oracle, frame, uv) for uv in grid])
    pts = grid[free]
    rng = np.random.default_rng(seed)
    i, j = rng.integers(0, len(pts), (2, 200000))
    d = np.linalg.norm(pts[i] - pts[j], axis=1)
    keep = d >= min_dist
    i, j = i[keep], j[keep]
    buckets = {"table": [], "lamp": [], "other": []}
    want = {"table": n_table, "lamp": n_lamp, "other": n_other}
    seen = set()
    for a, b in zip(i, j):
        key = (min(a, b), max(a, b))
        if key in seen:
            continue
        seen.add(key)
        p, q = pts[a], pts[b]
        if (p[0] - LAMP_U[1]) * (q[0] - LAMP_U[0]) < 0 or (min(p[0], q[0]) < LAMP_U[0] and max(p[0], q[0]) > LAMP_U[1]):
            cat = "lamp"
        elif _crosses_rect(p, q, np.asarray(TABLE_UV[0]), np.asarray(TABLE_UV[1])):
            cat = "table"
        else:
            cat = "other"
        if len(buckets[cat]) < want[cat]:
            buckets[cat].append((p.tolist(), q.tolist()))
        if all(len(buckets[k]) >= want[k] for k in want):
            break
    pairs = [{"name": f"{k[0].upper()}{n:02d}", "category": k, "start_uv": p, "goal_uv": q,
              "dist_m": float(np.linalg.norm(np.subtract(q, p)))}
             for k in ("table", "lamp", "other") for n, (p, q) in enumerate(buckets[k])]
    return {"rule": f"route grid step {step} m over the box; a point is kept if the gs3d oracle says free "
                    f"(continuous_bound) for BOTH robots at their own z_c with margin {GROUND_CONFIG.margin_m}; "
                    f"pairs drawn with numpy default_rng({seed}), xy distance >= {min_dist} m, bucketed by whether "
                    "the straight segment crosses the lamp's u-range, else the table footprint, else other",
            "grid_points": int(len(grid)), "free_for_both": int(free.sum()),
            "free_uv": pts.round(3).tolist(), "pairs": pairs}


# ----------------------------------------------------------------------------- screen
def brief(r, frame) -> dict:
    poly = r.get("polyline_world")
    route = frame.to_route(np.asarray(poly, float)).round(4).tolist() if poly else None
    return {"status": r["status"], "reason": r["reason"],
            "path_length_m": (r.get("metrics") or {}).get("path_length_m"),
            "vertices": len(poly) if poly else None, "route_polyline": route,
            "clearance_lower_m": r.get("clearance_lower_m"),
            "algorithm_wall_s": r["timings"]["algorithm_wall_s"],
            "stages": {t["stage"]: t["seconds"] for t in r["timings"]["records"]},
            "certificate_kind": (r.get("certificate") or {}).get("kind"),
            "cut_distinct_pairs": (r.get("certificate") or {}).get("cut_distinct_pairs")}


def cmd_screen(a):
    ctx = load_booth()
    body = ROBOTS[a.robot]
    frame = ctx["frame"]
    cand_path = a.out / "candidates.json"
    t0 = time.perf_counter()
    cands = candidate_pairs(ctx)
    cands["generation_wall_s"] = time.perf_counter() - t0
    _dump(cand_path, cands)
    print("candidates", {k: sum(p["category"] == k for p in cands["pairs"]) for k in ("table", "lamp", "other")},
          "free points", cands["free_for_both"], flush=True)
    compiled, outer, cpu = timed_compile(ctx["scene"], body)
    rec = compile_record(compiled, outer, cpu)
    t1 = time.perf_counter()
    meta = save_compiled(compiled, a.out / f"{a.robot}.a3c")
    rec["persist"] = {**meta, "save_wall_s": time.perf_counter() - t1}
    doc = {"schema": "aerial3dg.screen.v1", "robot": a.robot, "host": host(), "compile": rec,
           "archive_load_and_hash_s": ctx["archive_load_and_hash_s"],
           "scene_build_and_crop_s": ctx["scene_build_and_crop_s"], "crop": ctx["crop"],
           "support": ctx["support_evidence"], "box_route": ctx["box_route"], "results": []}
    print("compile", json.dumps({k: rec[k] for k in ("compile_wall_s", "peak_rss_mb")}), flush=True)
    _dump(a.out / f"screen_{a.robot}.json", doc)
    for p in cands["pairs"]:
        s, g = ground_world(frame, body, p["start_uv"]), ground_world(frame, body, p["goal_uv"])
        r = query(compiled, s, g, config=QCONFIG, call_id=p["name"])
        b = {"name": p["name"], "category": p["category"], "dist_m": p["dist_m"], **brief(r, frame)}
        doc["results"].append(b)
        print(p["name"], b["status"], b["reason"], b["path_length_m"], round(b["algorithm_wall_s"], 2), flush=True)
        _dump(a.out / f"screen_{a.robot}.json", doc)
    doc["peak_rss_mb_end"] = rss_mb()
    _dump(a.out / f"screen_{a.robot}.json", doc)


# ----------------------------------------------------------------------------- demo
def role_of(pid: int, man: dict) -> str:
    for e in man.get("edits", []):
        if e["id_range"][0] <= pid <= e["id_range"][1]:
            return e["role"]
    return "captured"


def straight_line_evidence(compiled, ctx, s_w, g_w, n=241) -> dict:
    """Along the straight start->goal segment at z_c: octree leaf label (+ the certifying pair for
    BLOCKED leaves) and the gs3d oracle's point report.  Evidence for why a route bends."""
    from gmc.aerial3d.octree import STATUS_NAMES, BLOCKED
    frame, man, body = ctx["frame"], ctx["manifest"], compiled.body
    oracle = GaussianBodyOracle(compiled.prepared)
    t = np.linspace(0., 1., n)
    rows = []
    for ti in t:
        w = s_w + ti * (g_w - s_w)
        p = compiled.frame.to_plan(w)
        leaves = compiled.tree.locate(p)
        st = [int(compiled.tree.status[l]) for l in leaves]
        label = ("BLOCKED" if BLOCKED in st else "SAFE" if st and all(x == 1 for x in st) else
                 "UNKNOWN" if st else "NONE")
        row = {"t": float(ti), "s_m": float(ti * np.linalg.norm(g_w - s_w)),
               "route": frame.to_route(w).round(4).tolist(), "leaf_label": label}
        if label == "BLOCKED":
            l = next(l for l in leaves if compiled.tree.status[l] == BLOCKED)
            k = int(compiled.tree.blocked_pair[l])
            pid = int(compiled.pairs.ids[k])
            mu = frame.to_route(compiled.frame.to_world(compiled.pairs.means[k]))
            row.update(blocked_pair_id=pid, blocked_role=role_of(pid, man),
                       blocked_gaussian_route=mu.round(3).tolist())
        q = Pose3(tuple(map(float, w)), 0.)
        r = oracle.edge(q, q, body, margin_m=compiled.config.margin_m)
        row.update(oracle_occupancy=r.occupancy, oracle_clearance_m=r.clearance_lower_m, oracle_reason=r.reason)
        rows.append(row)
    labels = [r["leaf_label"] for r in rows]
    roles = {}
    for r in rows:
        if "blocked_role" in r:
            roles[r["blocked_role"]] = roles.get(r["blocked_role"], 0) + 1
    return {"samples": n, "definition": "straight segment start->goal at the body-centre height z_c; leaf label = "
                                        "octree certificate at the sample; oracle = gs3d GaussianBodyOracle point check",
            "label_counts": {k: labels.count(k) for k in ("SAFE", "BLOCKED", "UNKNOWN", "NONE")},
            "oracle_counts": {k: sum(r["oracle_occupancy"] == k for r in rows) for k in ("free", "occupied", "unknown")},
            "blocked_samples_by_role": roles,
            "straight_line_free_by_oracle": all(r["oracle_occupancy"] == "free" for r in rows),
            "rows": rows}


def _sha(poly):
    return None if poly is None else hashlib.sha256(np.asarray(poly, float).round(12).tobytes()).hexdigest()


def cmd_demo(a):
    ctx = load_booth()
    body, frame, scene = ROBOTS[a.robot], ctx["frame"], ctx["scene"]
    s = ground_world(frame, body, a.start)
    g = ground_world(frame, body, a.goal)
    compiled, outer, cpu = timed_compile(scene, body)
    rec = compile_record(compiled, outer, cpu)
    compile_records_before = json.dumps(compiled.timings["records"], sort_keys=True, default=float)
    calls = []
    for k in range(1 + a.warm):
        mode, cid = ("cold", "cold0") if k == 0 else ("warm", f"warm{k - 1}")
        t0, c0 = time.perf_counter(), time.process_time()
        r = query(compiled, s, g, config=QCONFIG, call_id=cid)
        calls.append({"call_id": cid, "mode": mode, "source": "in_memory_compile", "result": r,
                      "outer_wall_s": time.perf_counter() - t0, "cpu_s": time.process_time() - c0})
        print(cid, r["status"], r["reason"], round(r["timings"]["algorithm_wall_s"], 3), flush=True)
    t0 = time.perf_counter()
    meta = save_compiled(compiled, a.out / f"{a.robot}.a3c")
    save_s = time.perf_counter() - t0
    t0 = time.perf_counter()
    back = load_compiled(a.out / f"{a.robot}.a3c")
    load_s = time.perf_counter() - t0
    t0, c0 = time.perf_counter(), time.process_time()
    r = query(back, s, g, config=QCONFIG, call_id="reloaded0")
    calls.append({"call_id": "reloaded0", "mode": "reloaded", "source": "persisted_compile_reloaded", "result": r,
                  "outer_wall_s": time.perf_counter() - t0, "cpu_s": time.process_time() - c0})
    compile_rerun = json.dumps(compiled.timings["records"], sort_keys=True, default=float) != compile_records_before
    primary = calls[0]["result"]
    replay = None
    if primary["status"] == "REACHABLE":
        t0 = time.perf_counter()
        rp = replay_plan(primary["gs3d_result"], GaussianBodyOracle(PreparedScene(scene)))
        replay = {k: v for k, v in rp.items() if k != "samples"}
        replay["wall_s"] = time.perf_counter() - t0
        replay["oracle"] = "fresh GaussianBodyOracle on a freshly built PreparedScene of the same SceneSpec"
    t0 = time.perf_counter()
    straight = straight_line_evidence(compiled, ctx, s, g)
    straight["wall_s"] = time.perf_counter() - t0
    print("straight line", straight["label_counts"], straight["oracle_counts"], straight["blocked_samples_by_role"],
          flush=True)
    compile_stage_names = {x["stage"] for x in rec["stages"]}
    doc = {"schema": "aerial3dg.demo.v1", "robot": a.robot, "body": asdict(body), "host": host(),
           "pair": {"start_uv": a.start, "goal_uv": a.goal, "start_world": s.tolist(), "goal_world": g.tolist(),
                    "dist_m": float(np.linalg.norm(np.subtract(a.goal, a.start)))},
           "box_route": ctx["box_route"], "crop": ctx["crop"], "support": ctx["support_evidence"],
           "archive_sha256": ctx["digest"], "compile": rec,
           "persist": {**meta, "save_wall_s": save_s, "load_wall_s": load_s},
           "status": primary["status"], "reason": primary["reason"], "result": primary, "replay": replay,
           "straight_line": straight,
           "calls": [{"call_id": c["call_id"], "mode": c["mode"], "source": c["source"],
                      "status": c["result"]["status"], "reason": c["result"]["reason"],
                      "algorithm_wall_s": c["result"]["timings"]["algorithm_wall_s"],
                      "outer_wall_s": c["outer_wall_s"], "cpu_s": c["cpu_s"],
                      "compile_id": c["result"]["compile_id"],
                      "stages": {t["stage"]: t["seconds"] for t in c["result"]["timings"]["records"]},
                      "compile_stages_in_call": sorted(compile_stage_names &
                                                       {t["stage"] for t in c["result"]["timings"]["records"]}),
                      "polyline_sha256": _sha(c["result"]["polyline_world"])} for c in calls],
           "compile_once_proof": {
               "compiles_in_this_process": 1,
               "compile_records_changed_by_queries": compile_rerun,
               "identical_polylines_all_calls": len({_sha(c["result"]["polyline_world"]) for c in calls}) == 1,
               "identical_status_all_calls": len({c["result"]["status"] for c in calls}) == 1,
               "same_compile_id_all_calls": len({c["result"]["compile_id"] for c in calls}) == 1},
           "timing_definition": "compile_wall_s = compile_complex entry to return (gs3d scene preparation + pairs + "
                                "envelopes + octree + cells/portals + possible graph + audit); algorithm_wall_s = "
                                "query entry to result incl. own verifier + in-query shared gs3d replay; cold = first "
                                "query after the compile; warm = later queries on the same in-memory compile; "
                                "reloaded = query on the compile saved to disk and loaded back (load time separate)",
           "archive_load_and_hash_s": ctx["archive_load_and_hash_s"],
           "scene_build_and_crop_s": ctx["scene_build_and_crop_s"], "peak_rss_mb_end": rss_mb()}
    _dump(a.out / f"demo_{a.robot}.json", doc)
    print(json.dumps(doc["compile_once_proof"]), flush=True)


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    pr = sub.add_parser("probe")
    pr.add_argument("--box", type=float, nargs=4, required=True, metavar=("U0", "V0", "U1", "V1"))
    pr.add_argument("--robots", nargs="+", default=["sweeper", "cylinder"])
    sc = sub.add_parser("screen")
    sc.add_argument("--robot", required=True, choices=sorted(ROBOTS))
    mp_ = sub.add_parser("map")
    mp_.add_argument("--box", type=float, nargs=4, required=True, metavar=("U0", "V0", "U1", "V1"))
    mp_.add_argument("--step", type=float, default=.1)
    de = sub.add_parser("demo")
    de.add_argument("--robot", required=True, choices=sorted(ROBOTS))
    de.add_argument("--start", type=float, nargs=2, required=True)
    de.add_argument("--goal", type=float, nargs=2, required=True)
    de.add_argument("--warm", type=int, default=3)
    for q in (pr, sc, de, mp_):
        q.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)
    {"probe": cmd_probe, "screen": cmd_screen, "demo": cmd_demo, "map": cmd_map}[a.cmd](a)


if __name__ == "__main__":
    main()
