"""C2 production run of the aerial3d connectivity backend: compile once, start + goal queries only.

Mirror of ``uavlamp_run.py`` (same spec files, same manifest-checked loader, same
``uavlamp_query.build_scene``, same fresh-oracle ``replay_plan`` and the same
``ordered_evidence``) so the two methods' result JSONs line up key for key:

1. load the archive through the manifest-checked loader (which hashes the file against
   its manifest) and check that hash against the spec and the frozen value; build the
   scene with the baseline's ``build_scene`` (crop, box, tau, level, test-only variants);
2. compile the complex once (timed, cold).  Every spec given must describe the same
   scene (archive, frame, box, margin, variants), so one compile serves all its queries;
3. per query, in a forked child that starts from the freshly compiled state: one *cold*
   ``query(compiled, start, goal)`` call, then N *warm* calls on the same compile (lazy
   caches warm).  Children run one after another, never concurrently;
4. replay every returned path with a fresh ``GaussianBodyOracle`` on a freshly built
   ``PreparedScene`` of the same SceneSpec -- the baseline's own replay;
5. extract ordered evidence (under-lamp interval before above-table interval, z range,
   clearance) from the replayed trajectory alone.

Usage (from ``gmc/``)::

    python experiments/uavconn_run.py SPEC.json [SPEC.json ...] --output DIR --warm 3 \
        --uavlamp-root /scratch/wg2381/splathjb-uavlamp/gmc

Relative ``archive`` / ``manifest`` paths in a spec are resolved against ``--uavlamp-root``
(the L2 specs are written relative to the baseline worktree's ``gmc/``).
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import inspect
import json
import multiprocessing as mp
import os
from pathlib import Path
import platform
import resource
import subprocess
import sys
import time

import numpy as np

from gmc.aerial3d.api import CompileConfig, QueryConfig, compile_complex, query
from gmc.gs3d import scene_uavlamp as su
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.planner import _json_finite
from gmc.gs3d.timing import latency_statistics
from gmc.gs3d.trajectory import replay_plan, sample_linear_trajectory

from uavlamp_query import UAV, build_scene
from uavlamp_run import check_spec, ordered_evidence

FROZEN_SHA256 = "2a3a72d610519095381a83fd741a61f32b420b20f39e0c5f60951e463fd596cc"
# Keys that define the map (and hence the compile); queries may differ only elsewhere.
SCENE_KEYS = ("archive", "manifest", "archive_sha256", "frame", "box_route", "margin_m",
              "drop_roles", "extra_builders")


def resolve(spec: dict, root: Path | None) -> dict:
    out = dict(spec)
    for key in ("archive", "manifest"):
        p = Path(spec[key])
        out[key] = str(p if p.is_absolute() or root is None else root / p)
    return out


def scene_key(spec: dict) -> str:
    return json.dumps({k: spec.get(k) for k in SCENE_KEYS}, sort_keys=True)


def node_snapshot() -> dict:
    """CPU count and whether the node is shared (other jobs' CPUs allocated on it)."""
    snap = {"node": platform.node(), "slurm_job_id": os.environ.get("SLURM_JOB_ID"),
            "cpus_per_task": os.environ.get("SLURM_CPUS_PER_TASK"),
            "affinity_cpus": len(os.sched_getaffinity(0)), "loadavg": list(os.getloadavg()),
            "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    try:
        txt = subprocess.run(["scontrol", "show", "node", platform.node().split(".")[0]],
                             capture_output=True, text=True, timeout=20).stdout
        fields = dict(tok.split("=", 1) for tok in txt.split() if "=" in tok)
        snap.update({k: fields.get(k) for k in ("CPUAlloc", "CPUTot", "AllocMem", "RealMemory")})
        mine = int(snap["cpus_per_task"] or 0)
        snap["node_shared"] = int(fields.get("CPUAlloc", 0)) > mine if fields.get("CPUAlloc") else None
    except Exception as exc:  # scontrol missing: record, do not fail the run
        snap["scontrol_error"] = repr(exc)
    return snap


def _sha(poly) -> str | None:
    return None if poly is None else hashlib.sha256(np.asarray(poly, float).round(9).tobytes()).hexdigest()


def call_record(compiled, start_w, goal_w, config: QueryConfig, call_id: str) -> dict:
    """Verbatim record of one ``query`` call, built from the very objects passed."""
    known = getattr(compiled.scene.known_space, "inner", compiled.scene.known_space)
    return {"method": "gmc.aerial3d.api.query", "signature": str(inspect.signature(query)),
            "arguments": {
                "compiled": {"compile_id": compiled.compile_id, "scene_id": compiled.scene.scene_id,
                             "n_gaussians": int(len(compiled.scene.gaussians.ids)),
                             "tau": compiled.scene.tau, "level": compiled.scene.level,
                             "known_space": {"type": type(known).__name__, **asdict(known)},
                             "body": asdict(compiled.body), "compile_config": _json_finite(asdict(compiled.config))},
                "start": [float(x) for x in start_w], "goal": [float(x) for x in goal_w]},
            "keyword_arguments": {"config": asdict(config), "call_id": call_id},
            "not_passed": "no waypoints, no intermediate poses, no z schedule, no altitude cost, no per-query "
                          "box or parameter: query() accepts none of these; config is the global default"}


def _query_calls(compiled, start_w, goal_w, warm: int, config: QueryConfig) -> list[dict]:
    calls = []
    for k, mode in enumerate(["cold"] + ["warm"] * warm):
        call_id = "cold0" if mode == "cold" else f"warm{k - 1}"
        t0, c0 = time.time(), time.process_time()
        p0 = time.perf_counter()
        r = query(compiled, start_w, goal_w, config=config, call_id=call_id)
        calls.append({"call_id": call_id, "mode": mode, "result": r, "outer_wall_s": time.perf_counter() - p0,
                      "cpu_s": time.process_time() - c0, "pid": os.getpid(),
                      "utc_started": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(t0)),
                      "call_record": call_record(compiled, start_w, goal_w, config, call_id)})
    return calls


def _child(path: str, compiled, start_w, goal_w, warm, config):
    try:
        out = _query_calls(compiled, start_w, goal_w, warm, config)
        Path(path).write_text(json.dumps(out, allow_nan=False, default=float))
    finally:
        os._exit(0)


def attribute(ids, manifest_doc, full_max_id: int) -> dict:
    """Map certificate pair ids to scene roles (manifest edits, test-only rows, captured)."""
    roles = {}
    ranges = [(e["id_range"][0], e["id_range"][1], e["role"]) for e in manifest_doc.get("edits", [])]
    for i in ids:
        role = "test_only_plug" if i > full_max_id else next(
            (r for lo, hi, r in ranges if lo <= i <= hi), "captured")
        roles[role] = roles.get(role, 0) + 1
    return roles


def run(spec_paths, output: Path, warm: int = 3, *, uavlamp_root: Path | None = None,
        expected_sha256: str | None = FROZEN_SHA256, compile_config: CompileConfig | None = None) -> list[dict]:
    specs = [resolve(check_spec(json.loads(Path(p).read_text())), uavlamp_root) for p in spec_paths]
    if len({scene_key(s) for s in specs}) != 1:
        raise ValueError("all specs of one run must describe the same scene (one compile per scene)")
    spec0 = specs[0]
    output.mkdir(parents=True, exist_ok=True)
    host_start = node_snapshot()
    t_load = time.perf_counter()
    full, manifest_doc = su.load_uavlamp_derivative(spec0["archive"], spec0["manifest"])  # hashes the file
    load_s = time.perf_counter() - t_load
    digest = manifest_doc["derivative"]["sha256"]
    if spec0.get("archive_sha256") and digest != spec0["archive_sha256"]:
        raise ValueError("archive hash differs from the spec")
    if expected_sha256 is not None and digest != expected_sha256:
        raise ValueError(f"archive hash {digest} is not the frozen {expected_sha256}")
    t_build = time.perf_counter()
    full_max_id = int(full.ids.max())
    frame, scene, crop, dropped, extras = build_scene(spec0, full, manifest_doc)
    del full
    build_s = time.perf_counter() - t_build
    config = compile_config or CompileConfig(margin_m=spec0.get("margin_m", .05))
    qconfig = QueryConfig()
    t0, c0 = time.perf_counter(), time.process_time()
    compiled = compile_complex(scene, UAV, config=config)
    compile_outer_s, compile_cpu_s = time.perf_counter() - t0, time.process_time() - c0
    summary = compiled.summary()
    compile_doc = {"schema": "uavconn.c2_compile.v1", "scene_id": scene.scene_id, "archive_sha256": digest,
                   "specs": [str(p) for p in spec_paths], "compile": summary,
                   "compile_outer_wall_s": compile_outer_s, "compile_cpu_s": compile_cpu_s,
                   "peak_rss_mb_after_compile": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.,
                   "host_at_start": host_start, "host_after_compile": node_snapshot()}
    (output / "compile.json").write_text(json.dumps(compile_doc, allow_nan=False, default=float) + "\n")
    print(json.dumps({"compile_wall_s": summary["timings"]["compile_wall_s"],
                      "leaves": summary["octree"].get("leaves_by_status"),
                      "cells": summary["cells"].get("cells")}), flush=True)

    ctx = mp.get_context("fork")
    docs = []
    for spec_path, spec in zip(spec_paths, specs):
        name = spec["name"]
        qdir = output / name
        qdir.mkdir(parents=True, exist_ok=True)
        start_w = frame.to_world(spec["start_route"])
        goal_w = frame.to_world(spec["goal_route"])
        path = qdir / "calls.json"
        proc = ctx.Process(target=_child, args=(str(path), compiled, start_w, goal_w, warm, qconfig))
        proc.start()
        proc.join()
        if proc.exitcode != 0 or not path.exists():
            raise RuntimeError(f"query process for {name} failed (exit {proc.exitcode})")
        calls = json.loads(path.read_text())
        path.unlink()
        primary = calls[0]
        result = primary["result"]
        polys = [c["result"]["polyline_world"] for c in calls]

        replay = None
        if result["status"] == "REACHABLE":
            t_replay = time.perf_counter()
            fresh = PreparedScene(scene)
            replay = replay_plan(result["gs3d_result"], GaussianBodyOracle(fresh))
            replay = {k: v for k, v in replay.items() if k != "samples"}
            replay["wall_s"] = time.perf_counter() - t_replay
            replay["oracle"] = "fresh GaussianBodyOracle on a freshly built PreparedScene of the same SceneSpec"

        evidence = None
        if result["status"] == "REACHABLE":
            samples = sample_linear_trajectory(result["gs3d_result"]["trajectory"], dt_s=.02)
            route = frame.to_route(np.asarray(samples["poses"], float)[:, :3])
            evidence = ordered_evidence(route, lamp=spec["lamp"], table=spec["table"], radius=UAV.radius_m,
                                        half_height=UAV.half_height_m, margin=config.margin_m)
            evidence["knots_route"] = frame.to_route(np.asarray(result["polyline_world"], float)).round(4).tolist()
            evidence["clearance_lower_m_planner"] = result["clearance_lower_m"]
            evidence["clearance_lower_m_replay"] = (replay or {}).get("geometry", {}).get("clearance_lower_m")
            evidence["route_samples"] = route.round(5).tolist()

        attribution = None
        cert = result.get("certificate") or {}
        if cert.get("cut_pair_ids"):
            attribution = {"cut_pairs_by_role": attribute(cert["cut_pair_ids"], manifest_doc, full_max_id),
                           "rule": "manifest edit id ranges by role; ids above the archive's max id are the "
                                   "test-only rows appended by build_scene; the rest are captured Gaussians"}

        by_mode = {m: [c["result"]["timings"]["algorithm_wall_s"] for c in calls if c["mode"] == m]
                   for m in ("cold", "warm")}
        compile_s = summary["timings"]["compile_wall_s"]
        doc = {
            "schema": "uavconn.c2_run.v1", "name": name, "spec_path": str(spec_path), "spec": spec,
            "archive_sha256": digest, "frozen_sha256_checked": expected_sha256,
            "manifest_checked_loader": "gmc.gs3d.scene_uavlamp.load_uavlamp_derivative",
            "scene_variants_test_only": {"dropped_roles": dropped, "extra_rows": extras},
            "crop": crop,
            "compile": {"compile_id": compiled.compile_id, "scene_id": scene.scene_id,
                        "file": str(output / "compile.json"), "shared_by": [s["name"] for s in specs]},
            "planner_calls_per_query": 1,
            "n_repeated_calls_for_timing": len(calls),
            "call_arguments_verbatim": primary["call_record"],
            "status": result["status"], "reason": result["reason"], "primary_call": primary["call_id"],
            "result": result, "replay": replay, "evidence": evidence,
            "certificate_attribution": attribution,
            "all_calls": [{"call_id": c["call_id"], "mode": c["mode"], "status": c["result"]["status"],
                           "reason": c["result"]["reason"],
                           "algorithm_wall_s": c["result"]["timings"]["algorithm_wall_s"],
                           "outer_wall_s": c["outer_wall_s"], "cpu_s": c["cpu_s"],
                           "stages": {r["stage"]: r["seconds"] for r in c["result"]["timings"]["records"]},
                           "polyline_sha256": _sha(c["result"]["polyline_world"]),
                           "utc_started": c["utc_started"], "pid": c["pid"],
                           "same_arguments_as_primary": c["call_record"]["arguments"] ==
                           primary["call_record"]["arguments"] and c["call_record"]["keyword_arguments"]["config"]
                           == primary["call_record"]["keyword_arguments"]["config"]} for c in calls],
            "identical_routes_across_calls": len({_sha(p) for p in polys}) == 1,
            "timing": {
                "definition": "compile_wall_s = compile_complex entry to return (scene preparation + pairs + "
                              "envelopes + octree + cells/portals + possible graph + audit), once per scene; "
                              "algorithm_wall_s = query entry to assembled result incl. the in-query gs3d replay. "
                              "cold = first call in a process forked from the freshly compiled state; warm = "
                              "later calls on the same compile. Both exclude archive load/crop and the "
                              "post-hoc fresh replay below.",
                "archive_load_and_hash_s": load_s, "scene_build_and_crop_s": build_s,
                "compile_wall_s": compile_s, "compile_outer_wall_s": compile_outer_s,
                "compile_cpu_s": compile_cpu_s,
                "compile_stages": [(r["stage"], r["seconds"], r["sizes"].get("peak_rss_mb"))
                                   for r in summary["timings"]["records"]],
                "cold_algorithm_wall_s": by_mode["cold"], "warm_algorithm_wall_s": by_mode["warm"],
                "warm_stats": latency_statistics(by_mode["warm"]) if by_mode["warm"] else None,
                "time_to_first_path_s": compile_s + by_mode["cold"][0],
                "time_to_first_path_incl_load_s": load_s + build_s + compile_s + by_mode["cold"][0],
                "calls_ran_concurrently": False,
                "fresh_replay_wall_s": (replay or {}).get("wall_s")},
            "host": {**node_snapshot(), "python": sys.version.split()[0], "host_at_start": host_start},
        }
        (qdir / "result.json").write_text(json.dumps(doc, allow_nan=False, default=float) + "\n")
        brief = {"name": name, "status": doc["status"], "reason": doc["reason"],
                 "cold_s": by_mode["cold"], "warm_s": by_mode["warm"], "compile_s": compile_s,
                 "identical_routes_across_calls": doc["identical_routes_across_calls"],
                 "replay_passed": (replay or {}).get("passed")}
        if evidence:
            brief["evidence"] = {k: evidence[k] for k in ("z_range", "under_precedes_above_table",
                                                          "passes_under_lamp", "path_length_m",
                                                          "clearance_lower_m_replay")}
        if attribution:
            brief["cut_pairs_by_role"] = attribution["cut_pairs_by_role"]
        (qdir / "summary.json").write_text(json.dumps(brief, indent=1, default=float) + "\n")
        print(json.dumps(brief, default=float), flush=True)
        docs.append(doc)
    return docs


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("specs", nargs="+", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--warm", type=int, default=3)
    p.add_argument("--uavlamp-root", type=Path)
    a = p.parse_args(argv)
    run(a.specs, a.output, a.warm, uavlamp_root=a.uavlamp_root)


if __name__ == "__main__":
    main()
