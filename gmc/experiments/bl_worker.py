"""bl B1: the method side of the baseline harness -- one long-lived process per (method, robot, region) task.

Runs under the METHOD's interpreter (splatnav / foci env, or gmc-venv for the harness's own test methods). Imports
only numpy + the method's adapter module; never ``gmc.gs3d`` (the judge stays in the parent, under gmc-venv).

Protocol (JSON lines). The method's own prints -- Python *and* C level (IPOPT writes to fd 1) -- must not corrupt it,
so at start the worker keeps a private dup of fd 1 for the protocol and points fd 1 at fd 2 (the task's worker log).

  worker -> parent  {"op": "ready", "setup_id", "artifact_sha256", "load_s", "instantiate_s", "instantiate_cpu_s",
                     "pid", "setup_calls": 0, ...adapter info}
  parent -> worker  {"op": "query", "pair_id", "start_uv", "goal_uv"[, "pair"]}       ("pair" only for oracle tests)
  worker -> parent  {"op": "result", "pair_id", "claimed", "claimed_reason", "path_uv" (list [u, v(, z)] or null),
                     "algorithm_wall_s", "cpu_s", "stages", "info"}     or     {"op": "result", ..., "error": "..."}
  parent -> worker  {"op": "exit"}

``--mode setup`` instead builds the method's persisted setup artifact once (``Adapter.build_setup``), writes it with a
SHA-256 sidecar and exits. The per-query wall limit is enforced by the PARENT (it kills this process group); nothing
here uses signals or alarms (F4's in-process SIGALRM surfaced as a TypeError inside numpy, 0a97784).

Adapter interface (``bl_splatnav.Adapter``, ``bl_foci.Adapter``, the test methods below):
  build_setup(scene: dict, body: dict, config: dict) -> state (picklable)    # timed once per robot x region
  instantiate(state, config) -> live planner                               # timed per worker start
  plan(live, start_uv, goal_uv, query) -> {"claimed", "claimed_reason", "path_uv", "stages", "info"}
"""
from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import os
import pickle
import sys
import time
import traceback

sys.dont_write_bytecode = True

ADAPTERS = {"splatnav": "bl_splatnav", "foci": "bl_foci", "astar_replay": "bl_testmethods",
            "gmc_requery": "bl_testmethods", "straight": "bl_testmethods", "sleep": "bl_testmethods",
            "raise": "bl_testmethods", "crash": "bl_testmethods"}


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 22), b""):
            h.update(b)
    return h.hexdigest()


def load_scene(path):
    """The harness's scene export (``bl_harness export``): judge Gaussians in the plan frame + contract + body."""
    import numpy as np
    with np.load(path, allow_pickle=False) as d:
        scene = {k: d[k] for k in d.files if k != "meta"}
        scene["meta"] = json.loads(str(d["meta"]))
    return scene


def adapter_for(method):
    mod = importlib.import_module(ADAPTERS[method])
    return mod.Adapter(method) if getattr(mod, "MULTI", False) else mod.Adapter()


def write_artifact(path, state, meta):
    blob = pickle.dumps(state, protocol=pickle.HIGHEST_PROTOCOL)
    tmp = f"{path}.tmp{os.getpid()}"
    with open(tmp, "wb") as f:
        f.write(blob)
    os.replace(tmp, path)
    meta = dict(meta, bytes=len(blob), sha256=hashlib.sha256(blob).hexdigest())
    with open(f"{path}.json", "w") as f:
        f.write(json.dumps(meta, indent=1, default=float) + "\n")
    return meta


def read_artifact(path):
    """Load a persisted setup; fail closed if its bytes differ from the sidecar hash."""
    with open(f"{path}.json") as f:
        meta = json.load(f)
    with open(path, "rb") as f:
        blob = f.read()
    if hashlib.sha256(blob).hexdigest() != meta["sha256"]:
        raise ValueError(f"setup artifact {path} differs from its sidecar hash")
    return pickle.loads(blob), meta


def cmd_setup(a):
    """Build the method's setup state from the scene export (once per robot x region) and persist it."""
    config = json.loads(a.config)
    scene = load_scene(a.scene)
    ad = adapter_for(a.method)
    w0, c0 = time.perf_counter(), time.process_time()
    state = ad.build_setup(scene, scene["meta"]["body"], config)
    wall, cpu = time.perf_counter() - w0, time.process_time() - c0
    setup_id = hashlib.sha256(json.dumps({"method": a.method, "scene_sha256": sha256_file(a.scene),
                                          "config": config}, sort_keys=True).encode()).hexdigest()[:16]
    meta = write_artifact(a.artifact, state, {
        "setup_id": setup_id, "method": a.method, "scene_export": a.scene, "scene_sha256": sha256_file(a.scene),
        "config": config, "build_setup_wall_s": wall, "build_setup_cpu_s": cpu,
        "region": scene["meta"]["region"], "robot": scene["meta"]["robot"],
        "info": getattr(ad, "setup_info", {}), "python": sys.version.split()[0], "pid": os.getpid()})
    print(json.dumps({k: meta[k] for k in ("setup_id", "build_setup_wall_s", "bytes", "sha256")}), flush=True)


def cmd_serve(a):
    proto = os.fdopen(os.dup(1), "w", buffering=1)
    os.dup2(2, 1)                         # method prints (incl. C-level) -> worker log
    sys.stdout = sys.stderr

    def send(obj):
        proto.write(json.dumps(obj, default=float) + "\n")
        proto.flush()

    config = json.loads(a.config)
    ad = adapter_for(a.method)
    t0 = time.perf_counter()
    state, meta = read_artifact(a.artifact)
    load_s = time.perf_counter() - t0
    w0, c0 = time.perf_counter(), time.process_time()
    live = ad.instantiate(state, config)
    send({"op": "ready", "setup_id": meta["setup_id"], "artifact_sha256": meta["sha256"], "load_s": load_s,
          "instantiate_s": time.perf_counter() - w0, "instantiate_cpu_s": time.process_time() - c0,
          "pid": os.getpid(), "setup_calls": 0, "python": sys.version.split()[0],
          "info": getattr(ad, "instance_info", {})})
    for line in sys.stdin:
        msg = json.loads(line)
        if msg.get("op") == "exit":
            break
        out = {"op": "result", "pair_id": msg["pair_id"]}
        w0, c0 = time.perf_counter(), time.process_time()
        try:
            r = ad.plan(live, msg["start_uv"], msg["goal_uv"], msg)
            out.update(claimed=bool(r["claimed"]), claimed_reason=r.get("claimed_reason"),
                       path_uv=r.get("path_uv"), stages=r.get("stages", {}), info=r.get("info", {}))
        except Exception as exc:
            tb = traceback.extract_tb(exc.__traceback__)[-1]
            out.update(error=f"{type(exc).__name__}: {exc} @ {tb.filename.split('/')[-1]}:{tb.lineno}",
                       traceback=traceback.format_exc()[-3000:])
        out["algorithm_wall_s"] = time.perf_counter() - w0
        out["cpu_s"] = time.process_time() - c0
        send(out)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=("setup", "serve"), required=True)
    ap.add_argument("--method", required=True)
    ap.add_argument("--artifact", required=True)
    ap.add_argument("--scene", help="scene export npz (setup mode)")
    ap.add_argument("--config", default="{}")
    a = ap.parse_args(argv)
    (cmd_setup if a.mode == "setup" else cmd_serve)(a)


if __name__ == "__main__":
    main()
