"""bl B1: the shared baseline harness (plan docs/baselines_f4_plan.md §3, §5). Every baseline runs through it unchanged.

Run from ``gmc/`` under gmc-venv with ``PYTHONPATH=src:experiments MPLBACKEND=Agg``. The judge always runs here (this
process, gmc-venv); a method runs in its own interpreter as a ``bl_worker.py`` child process that the harness can kill.

What it does, per (method, robot, region) task:
  * pairs: the F4 5000 (``results/aerial3dg/f4/pairs_confirmed_5000.json``), the tuning set (plan §3.5: the first
    25 WWEST + 15 GAPW1 + 10 S of F3's ``pilot_pairs.json``), or a committed id list (pilot / sanity samples);
  * judge scene: the persisted F3 compile of that region and robot (SHA-256 = sidecar = F3 handoff = F4
    compile_once, ``bl_judge_check.verified_a3c``) -- its ``prepared`` scene is the object F5 judged A* routes with;
  * scene export (``export``): the judge's own Gaussians (``PreparedScene.means/covs/ids``: opacity > tau, covariances
    as the judge floors them) rotated into the plan (= route) frame, + tau, level, margin, body, z_c, the known-space
    prism. That npz (+ SHA-256) is what a Gaussian-native method reads;
  * setup (``setup``): the method's one-time build from the export, in its own env, persisted with SHA-256
    (``bl_worker --mode setup``). Each task's worker loads it (``load_s``) and instantiates the live planner
    (``instantiate_s``, per worker start); neither is charged to a query. Setup-once proof per task;
  * query: the worker gets only (start_uv, goal_uv); the parent enforces the 120 s wall limit from OUTSIDE by
    killing the worker's process group (no in-process alarms: F4's SIGALRM surfaced as a TypeError, 0a97784), then
    restarts the worker for the next pair;
  * export + judge (``Judge``): a method's planar path is completed (a straight segment from the exact start /
    to the exact goal when the method does not start / end there, recorded), lifted to z_c and converted to a
    ``gs3d.v1`` result by GMC's own exporter -- ``api._gs3d_result(C, api._densify(P, QCONFIG.export_max_segment_m),
    g_w, .)``, i.e. turn in place to each heading, translate at 0.3 m/s, final turn to yaw 0, position tolerance 0,
    yaw tolerance 0.05, margin = the compile's 0.001, segments split to <= 0.20 m, endpoints the exact
    ``frame.to_world([u, v, clearance + half_height])`` as ``aerial3dg_batch.run_task`` computes them and interior
    vertices at ``domain.ground_z`` as ``api.query`` pins them -- then judged by ``replay_plan(result,
    GaussianBodyOracle(C.prepared))`` (§2). ``judge_wall_s`` is never charged to the method;
  * rows (§5) appended + fsync'd per pair (resumable), task summaries with host/commit/setup/config SHA.

Outcome (§5): SUCCESS | CLAIMED_COLLIDES | CLAIMED_UNPROVEN | CLAIMED_KINEMATICS | FAIL | TIMEOUT | ERROR | SETUP_FAIL.

Subcommands:
  export   (sbatch) scene export npz per region x robot -> outputs/baselines/scene/<R>_<robot>.npz (+ .json sidecar)
  setup    (sbatch) one method setup per region x robot -> outputs/baselines/<method>/<cfg>/<R>_<robot>.setup.pkl
  run      (sbatch) one task: rows -> <out>/<R>/<robot>/task_NN.jsonl + .summary.json
  judge    (sbatch) fill deferred judge fields of a task file (``run --judge defer`` keeps GPU time off the CPU judge)
  sample   (inline) stratified F4 pair lists (pilot / sanity), committed
  report   (inline) outcome counts + timing over task files
  project  (inline) full-run cost projection (5000 x 2 robots) from pilot task files
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

import numpy as np

F3 = Path("results/aerial3dg/f3")
F4 = Path("results/aerial3dg/f4")
RES = Path("results/baselines")
OUTB = Path("outputs/baselines")
CONFIGS = Path("configs/baselines")
ENVS = Path("/scratch/wg2381/.conda/envs")
REGIONS = ("WWEST", "GAPW1", "S")
ROBOTS = ("cylinder", "sweeper")
TUNING = {"WWEST": 25, "GAPW1": 15, "S": 10}           # plan §3.5
QUERY_TIMEOUT_S = 120.
READY_TIMEOUT_S = 1800.
STATUSES = ("SUCCESS", "CLAIMED_COLLIDES", "CLAIMED_UNPROVEN", "CLAIMED_KINEMATICS", "FAIL", "TIMEOUT", "ERROR",
            "SETUP_FAIL")
METHOD_PYTHON = {"splatnav": ENVS / "splatnav/bin/python", "foci": ENVS / "foci/bin/python"}
SNAP_M = 1e-9          # a method endpoint this close to the pair's endpoint is the endpoint (roundoff of frames)


def _dump(path, doc):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + f".tmp{os.getpid()}")
    tmp.write_text(json.dumps(doc, indent=1, default=float) + "\n")
    tmp.replace(path)


def sha256_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 22), b""):
            h.update(b)
    return h.hexdigest()


def sha_json(doc):
    return hashlib.sha256(json.dumps(doc, sort_keys=True, default=float).encode()).hexdigest()


def poly_sha(poly):
    """``aerial3dg_run._sha``: SHA-256 of the world polyline rounded to 1e-12."""
    return None if poly is None else hashlib.sha256(np.asarray(poly, float).round(12).tobytes()).hexdigest()


# ================================================================================================ pairs
def _lat_band(x):
    from aerial3dg_fail3_f4 import LAT_BANDS
    for lo, hi, name in LAT_BANDS:
        if lo <= x < hi:
            return name
    return "?"


def load_pairs(source, region=None):
    """List of pair dicts (pair_id, region, index-in-source, start_uv, goal_uv, dist_m, astar_route_uv,
    lateral_clearance_m, len_ratio). ``source`` = 'f4' | 'tuning' | path of a committed id list."""
    if source == "tuning":
        out = []
        for reg, n in TUNING.items():
            doc = json.loads((F3 / "sample" / reg / "pilot_pairs.json").read_text())
            for p in doc["pairs"][:n]:
                ev = p["evidence"]
                out.append({"pair_id": p["pair_id"], "region": reg, "start_uv": p["start_uv"],
                            "goal_uv": p["goal_uv"], "dist_m": p["dist_m"], "astar_route_uv": p["astar_route_uv"],
                            "lateral_clearance_m": ev.get("lateral_m"), "len_ratio": ev.get("len_ratio")})
    else:
        doc = json.loads((F4 / "pairs_confirmed_5000.json").read_text())
        keys = ("pair_id", "region", "start_uv", "goal_uv", "dist_m", "astar_route_uv", "lateral_clearance_m",
                "len_ratio")
        allp = [{k: p[k] for k in keys} for p in doc["pairs"]]
        if source == "f4":
            out = allp
        else:
            ids = json.loads(Path(source).read_text())["pair_ids"]
            by = {p["pair_id"]: p for p in allp}
            out = [by[i] for i in ids]
    if region is not None:
        out = [p for p in out if p["region"] == region]
    for k, p in enumerate(out):
        p["index"] = k
    return out


def task_slice(n, n_tasks, task):
    return range(n * task // n_tasks, n * (task + 1) // n_tasks)


# ================================================================================================ judge + export
def complete_path(path_uv, start_uv, goal_uv, z_c):
    """Planar completion of a method path (§3.3): the exact uv polyline that is exported and judged.

    A method vertex within ``SNAP_M`` of the pair's start / goal is that endpoint (frame round-off only); otherwise a
    straight segment from the exact start / to the exact goal is added and recorded (judged like the rest). A 3-column
    path is (u, v, z): z is dropped (the harness plans the body at z_c) and its largest deviation recorded."""
    info = {"prepended_start_segment": False, "appended_goal_segment": False, "start_gap_m": None,
            "goal_gap_m": None, "max_abs_dz_m": None, "method_vertices": 0}
    P = np.asarray(path_uv, float)
    if P.ndim != 2 or P.shape[1] not in (2, 3) or len(P) == 0:
        raise ValueError(f"method path must be (N, 2|3), got shape {P.shape}")
    if not np.isfinite(P).all():
        raise ValueError("method path has non-finite coordinates")
    info["method_vertices"] = int(len(P))
    if P.shape[1] == 3:
        info["max_abs_dz_m"] = float(np.max(np.abs(P[:, 2] - z_c)))
    uv = P[:, :2].copy()
    s, g = np.asarray(start_uv, float), np.asarray(goal_uv, float)
    info["start_gap_m"], info["goal_gap_m"] = float(np.linalg.norm(uv[0] - s)), float(np.linalg.norm(uv[-1] - g))
    if info["start_gap_m"] <= SNAP_M:
        uv[0] = s
    else:
        uv = np.vstack([s, uv])
        info["prepended_start_segment"] = True
    if info["goal_gap_m"] <= SNAP_M and len(uv) > 1:
        uv[-1] = g
    else:
        uv = np.vstack([uv, g])
        info["appended_goal_segment"] = True
    return uv, info


class Judge:
    """The region's judge scene (persisted F3 compile, SHA-checked) + GMC's exporter + ``replay_plan``."""

    def __init__(self, region, robot):
        from bl_judge_check import verified_a3c
        from gmc.aerial3d.api import load_compiled
        from gmc.gs3d.oracle import GaussianBodyOracle
        t0 = time.perf_counter()
        self.region, self.robot = region, robot
        self.a3c, self.a3c_sha256 = verified_a3c(region, robot)
        self.C = load_compiled(self.a3c)
        self.load_s = time.perf_counter() - t0
        b = self.C.body
        if b.name != robot:
            raise ValueError(f"compile body {b.name} is not {robot}")
        self.z_c = b.ground_clearance_m + b.half_height_m          # aerial3dg_batch.run_task endpoints
        self.ground_z = float(self.C.domain.ground_z)                # api.query interior vertices
        if abs(self.ground_z - self.z_c) > 1e-9:
            raise ValueError(f"compiled ground z {self.ground_z} is not the body centre height {self.z_c}")
        self._oracle_cls = GaussianBodyOracle

    def world(self, uv):
        return self.C.frame.to_world([uv[0], uv[1], self.z_c])

    def complete(self, path_uv, start_uv, goal_uv):
        return complete_path(path_uv, start_uv, goal_uv, self.z_c)

    def export(self, uv, goal_uv):
        """gs3d.v1 result through GMC's exporter (api._gs3d_result + api._densify at QCONFIG's 0.20 m)."""
        import gmc.aerial3d.api as api
        from aerial3dg_run import QCONFIG
        P = self.C.frame.to_world(np.c_[np.asarray(uv, float), np.full(len(uv), self.ground_z)])
        P[0], P[-1] = self.world(uv[0]), self.world(goal_uv)       # exact endpoints, as api.query sets them
        res = api._gs3d_result(self.C, api._densify(P, QCONFIG.export_max_segment_m), P[-1], None)
        return P, res

    def judge(self, res):
        from gmc.gs3d.trajectory import replay_plan
        t0 = time.perf_counter()
        rep = replay_plan(res, self._oracle_cls(self.C.prepared))
        wall = time.perf_counter() - t0
        g, k = rep["geometry"], rep["kinematics"]
        last = g["reports"][-1] if g.get("reports") else None
        return {"judge_passed": bool(rep["passed"]), "judge_geometry_passed": bool(g["passed"]),
                "judge_geometry_reason": g["reason"], "judge_geometry_safety": g["safety"],
                "judge_geometry_occupancy": None if g["passed"] or last is None else last["occupancy"],
                "judge_failed_edge": None if g["passed"] else len(g["reports"]) - 1,
                "judge_clearance_lower_m": g.get("clearance_lower_m"),
                "judge_kinematics_passed": bool(k["passed"]), "judge_kinematics_reason": k.get("reason"),
                "judge_attained_matches": bool(rep["attained_matches"]),
                "judge_reason": "passed" if rep["passed"] else
                ("geometry:" + str(g["reason"]) if not g["passed"] else
                 "kinematics:" + str(k.get("reason")) if not k["passed"] else "attained_goal_mismatch"),
                "judge_wall_s": wall, "export_poses": len(res["trajectory"]["poses"])}


def fail_location(res, edge, info):
    """Where the judge's failing edge lies: on the start segment the harness prepended, on the goal segment it
    appended, or on the method's own path (by arc length along the exported poses; a turn in place at a vertex belongs
    to the method's path unless it is inside a completion segment)."""
    P = np.asarray(res["trajectory"]["poses"], float)[:, :2]
    s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    a, b = s[edge], s[min(edge + 1, len(s) - 1)]
    pre = info["start_gap_m"] if info["prepended_start_segment"] else 0.
    app = info["goal_gap_m"] if info["appended_goal_segment"] else 0.
    tol = 1e-9
    if pre and b <= pre + tol and (b > a or a < pre - tol):
        return "prepended_start_segment"
    if app and a >= s[-1] - app - tol and (b > a or a > s[-1] - app + tol):
        return "appended_goal_segment"
    return "method_path"


def outcome(claimed, j):
    """§5 status from the method's claim and the judge verdict."""
    if not claimed:
        return "FAIL"
    if j["judge_passed"]:
        return "SUCCESS"
    if not j["judge_geometry_passed"]:
        return "CLAIMED_COLLIDES" if j["judge_geometry_occupancy"] == "occupied" else "CLAIMED_UNPROVEN"
    return "CLAIMED_KINEMATICS"


def judge_row(J, row):
    """Complete + export + judge a row whose method claimed a path; fills §5 fields in place."""
    uv, info = J.complete(row["method_path_uv"], row["start_uv"], row["goal_uv"])
    P, res = J.export(uv, row["goal_uv"])
    j = J.judge(res)
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    j["judge_fail_location"] = None if j["judge_geometry_passed"] or j["judge_failed_edge"] is None else \
        fail_location(res, j["judge_failed_edge"], info)
    row.update(info, **j)
    row.update(route_polyline=uv.tolist(), polyline_sha256=poly_sha(P), path_length_m=float(seg.sum()),
               vertices=int(len(P)), status=outcome(True, j))
    row["reason"] = row["claimed_reason"] if row["status"] == "SUCCESS" else row["judge_reason"]
    return res


# ================================================================================================ worker handling
class Worker:
    """A ``bl_worker.py --mode serve`` child in its own session; killed (whole group) on timeout."""

    def __init__(self, method, artifact, config, log_path, python=None):
        self.method, self.artifact, self.config = method, str(artifact), config
        self.python = str(python or METHOD_PYTHON.get(method, sys.executable))
        self.log_path = Path(log_path)
        self.proc, self.buf, self.ready = None, b"", None

    def start(self, timeout=READY_TIMEOUT_S):
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        log = open(self.log_path, "ab")
        log.write(f"\n=== worker start {time.strftime('%Y-%m-%dT%H:%M:%S')} {self.method}\n".encode())
        log.flush()
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", MPLBACKEND="Agg",
                   XDG_CACHE_HOME="/scratch/wg2381/.cache", MPLCONFIGDIR="/scratch/wg2381/.cache/mpl",
                   WARP_CACHE_PATH="/scratch/wg2381/.cache/warp")
        here = str(Path(__file__).resolve().parent)
        env["PYTHONPATH"] = os.pathsep.join([here, str(Path(here).parent / "src")]) \
            if self.python == sys.executable else here
        cwd = self.log_path.parent / f"cwd_{self.method}"         # SplatNav writes infeasible.obj to cwd
        cwd.mkdir(parents=True, exist_ok=True)
        self.proc = subprocess.Popen(
            [self.python, "-B", str(Path(here) / "bl_worker.py"), "--mode", "serve", "--method", self.method,
             "--artifact", str(Path(self.artifact).resolve()), "--config", json.dumps(self.config)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log, cwd=cwd, env=env, start_new_session=True)
        log.close()
        self.buf = b""
        t0 = time.perf_counter()
        msg = self._read(timeout)
        if msg is None or msg.get("op") != "ready":
            tail = self.log_tail()
            self.kill()
            raise RuntimeError(f"worker did not become ready ({'timeout' if msg is None else msg}); log tail: {tail}")
        msg["ready_wall_s"] = time.perf_counter() - t0
        self.ready = msg
        return msg

    def _read(self, timeout):
        """One protocol line within ``timeout`` s; None on timeout; {'op': 'eof'} if the worker died."""
        deadline = time.perf_counter() + timeout
        fd = self.proc.stdout.fileno()
        while b"\n" not in self.buf:
            left = deadline - time.perf_counter()
            if left <= 0:
                return None
            r, _, _ = select.select([fd], [], [], left)
            if not r:
                return None
            chunk = os.read(fd, 1 << 20)
            if not chunk:
                return {"op": "eof", "returncode": self.proc.wait()}
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line)

    def query(self, msg, timeout):
        t0 = time.perf_counter()
        try:
            self.proc.stdin.write((json.dumps(msg) + "\n").encode())
            self.proc.stdin.flush()
        except BrokenPipeError:
            return {"op": "eof", "returncode": self.proc.wait()}, time.perf_counter() - t0
        out = self._read(timeout)
        return out, time.perf_counter() - t0

    def kill(self):
        if self.proc is None:
            return
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            self.proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            pass
        self.proc = None

    def close(self):
        if self.proc is None:
            return
        try:
            self.proc.stdin.write(b'{"op": "exit"}\n')
            self.proc.stdin.flush()
            self.proc.wait(timeout=30)
        except Exception:
            pass
        self.kill()

    def log_tail(self, n=1500):
        try:
            return self.log_path.read_bytes()[-n:].decode(errors="replace")
        except OSError:
            return ""


# ================================================================================================ task runner
def read_checkpoint(path):
    from aerial3dg_batch import read_checkpoint as rc
    return rc(path)


def host():
    from aerial3dg_run import host as h
    return h()


def run_task(method, region, robot, pairs, out_jsonl, *, artifact, config, timeout_s=QUERY_TIMEOUT_S,
             judge_mode="inline", python=None, pass_pair=False, config_file=None):
    """Answer every pair of ``pairs`` not yet in ``out_jsonl`` (resumable); returns the task summary."""
    out_jsonl = Path(out_jsonl)
    out_jsonl.parent.mkdir(parents=True, exist_ok=True)
    done = read_checkpoint(out_jsonl)
    todo = [p for p in pairs if p["index"] not in done]
    art_meta = json.loads(Path(f"{artifact}.json").read_text())
    art_sha = sha256_file(artifact)
    if art_sha != art_meta["sha256"]:
        raise ValueError("setup artifact hash differs from its sidecar")
    J = Judge(region, robot) if judge_mode == "inline" else None
    worker = Worker(method, artifact, config, worker_log_path(out_jsonl), python=python)
    starts, rows = [], []
    gen = -1
    try:
        with open(out_jsonl, "a") as f:
            for p in todo:
                if worker.proc is None:
                    try:
                        starts.append(dict(worker.start(), reason="first" if gen < 0 else "restart"))
                    except RuntimeError as exc:
                        starts.append({"failed": str(exc)[:2000]})
                        row = _base_row(method, region, robot, p, art_meta, gen)
                        row.update(status="SETUP_FAIL", reason=str(exc)[:500])
                        _write(f, row)
                        rows.append(row)
                        continue
                    gen += 1
                msg = {"op": "query", "pair_id": p["pair_id"], "start_uv": p["start_uv"], "goal_uv": p["goal_uv"]}
                if pass_pair:
                    msg["pair"] = p
                c0 = time.process_time()
                out, outer = worker.query(msg, timeout_s)
                row = _base_row(method, region, robot, p, art_meta, gen)
                row.update(outer_wall_s=outer, parent_cpu_s=time.process_time() - c0)
                if out is None:
                    worker.kill()
                    row.update(status="TIMEOUT", reason=f"query_exceeded_{timeout_s:g}s", claimed=None)
                elif out.get("op") == "eof":
                    tail = worker.log_tail(600)
                    worker.kill()
                    row.update(status="ERROR", reason=f"worker_died rc={out.get('returncode')}", claimed=None,
                               error_log_tail=tail)
                elif "error" in out:
                    row.update(status="ERROR", reason=out["error"][:500], claimed=None,
                               traceback=out.get("traceback"), algorithm_wall_s=out["algorithm_wall_s"],
                               cpu_s=out["cpu_s"])
                else:
                    row.update(claimed=out["claimed"], claimed_reason=out.get("claimed_reason"),
                               algorithm_wall_s=out["algorithm_wall_s"], cpu_s=out["cpu_s"],
                               stages=out.get("stages") or {}, info=out.get("info") or {},
                               method_path_uv=out.get("path_uv"))
                    row["info"].pop("gmc_trajectory", None)      # test-only payload, not a row field
                    if not out["claimed"]:
                        row.update(status="FAIL", reason=out.get("claimed_reason"))
                    elif out.get("path_uv") is None:
                        row.update(status="ERROR", reason="claimed_without_path")
                    elif J is not None:
                        try:
                            judge_row(J, row)
                            row.pop("method_path_uv")     # route_polyline holds the exact judged path
                        except ValueError as exc:
                            row.update(status="ERROR", reason=f"unexportable_path: {exc}"[:500])
                    else:
                        row.update(status="CLAIMED_PENDING_JUDGE", reason="judge_deferred")
                _write(f, row)
                rows.append(row)
    finally:
        worker.close()
    allrows = {**done, **{r["index"]: r for r in rows}}
    st = [r["status"] for r in allrows.values()]
    sids = {r.get("setup_id") for r in allrows.values()}
    ok_starts = [s for s in starts if "failed" not in s]
    return {"method": method, "region": region, "robot": robot, "pairs_in_slice": len(pairs),
            "resumed_from_checkpoint": len(done), "answered_this_run": len(rows),
            "complete": len(allrows) == len(pairs),
            "status_counts": {k: st.count(k) for k in STATUSES + ("CLAIMED_PENDING_JUDGE",) if st.count(k)},
            "query_outer_wall_s_this_run": float(sum(r.get("outer_wall_s") or 0. for r in rows)),
            "query_algorithm_wall_s_this_run": float(sum(r.get("algorithm_wall_s") or 0. for r in rows)),
            "judge_wall_s_this_run": float(sum(r.get("judge_wall_s") or 0. for r in rows)),
            "judge_scene": None if J is None else {"a3c": str(J.a3c), "a3c_sha256": J.a3c_sha256,
                                                   "compile_id": J.C.compile_id, "load_s": J.load_s},
            "setup": {"setup_id": art_meta["setup_id"], "artifact": str(artifact), "artifact_sha256": art_sha,
                      "build_setup_wall_s": art_meta.get("build_setup_wall_s"), "config": art_meta.get("config")},
            "setup_once_proof": {
                "setup_builds_in_this_task": 0,
                "worker_starts": len(starts), "worker_start_failures": len(starts) - len(ok_starts),
                "restarts": max(0, len(ok_starts) - 1),
                "every_start_loaded_the_same_artifact": all(s.get("artifact_sha256") == art_sha for s in ok_starts),
                "every_start_setup_calls_zero": all(s.get("setup_calls") == 0 for s in ok_starts),
                "same_setup_id_all_rows": len(sids) == 1,
                "instantiate_s_per_start": [s.get("instantiate_s") for s in ok_starts],
                "load_s_per_start": [s.get("load_s") for s in ok_starts],
                "definition": "the method's setup is built once per robot x region by `setup` and persisted; a task "
                              "never builds it (0), each worker start loads that artifact (SHA checked) and "
                              "instantiates the live planner from it; a restart happens only after a TIMEOUT kill "
                              "or a worker crash, and its instantiate time is reported here, never charged to a "
                              "query"},
            "worker_starts": starts, "config": config, "config_sha256": sha_json(config),
            "config_file": config_file, "config_file_sha256": sha256_file(config_file) if config_file else None,
            "timeout_s": timeout_s, "judge_mode": judge_mode, "worker_log": str(worker.log_path), "host": host()}


def worker_log_path(out_jsonl):
    """Method stdout/stderr (IPOPT is verbose): under outputs/ (uncommitted), mirroring the rows' path."""
    p = Path(out_jsonl).resolve()
    try:
        rel = p.relative_to(Path.cwd().resolve() / "results")
    except ValueError:
        return p.with_suffix(".worker.log")
    return (OUTB / "worker_logs" / rel).with_suffix(".worker.log")


def _base_row(method, region, robot, p, art_meta, gen):
    return {"index": p["index"], "pair_id": p["pair_id"], "region": region, "robot": robot, "method": method,
            "start_uv": p["start_uv"], "goal_uv": p["goal_uv"], "dist_m": p.get("dist_m"),
            "lateral_clearance_m": p.get("lateral_clearance_m"), "len_ratio": p.get("len_ratio"),
            "setup_id": art_meta["setup_id"], "worker_generation": gen, "claimed": None, "claimed_reason": None,
            "appended_goal_segment": None, "prepended_start_segment": None, "judge_passed": None,
            "judge_reason": None, "judge_geometry_reason": None, "judge_kinematics_reason": None,
            "judge_wall_s": None, "path_length_m": None, "vertices": None, "route_polyline": None,
            "polyline_sha256": None, "algorithm_wall_s": None, "cpu_s": None, "outer_wall_s": None, "stages": {}}


def _write(f, row):
    from gmc.gs3d.planner import _json_finite
    f.write(json.dumps(_json_finite(row), default=float) + "\n")
    f.flush()
    os.fsync(f.fileno())


# ================================================================================================ commands
def scene_export_path(region, robot):
    return OUTB / "scene" / f"{region}_{robot}.npz"


def cmd_export(a):
    """Judge Gaussians of each (region, robot) compile in the plan frame + contract, persisted with SHA-256."""
    for region in a.regions:
        for robot in a.robots:
            t0 = time.perf_counter()
            J = Judge(region, robot)
            C = J.C
            prep = C.prepared
            R = np.asarray(C.frame.R, float)
            means = C.frame.to_plan(np.asarray(prep.means, float))
            covs = np.einsum("ij,njk,lk->nil", R, np.asarray(prep.covs, float), R)
            ks = prep.scene.known_space
            ks = getattr(ks, "inner", ks)
            box = json.loads((F4 / "pairs_confirmed_5000.json").read_text())["regions"][region]["box_uv"]
            from dataclasses import asdict
            meta = {"schema": "bl.scene_export.v1", "region": region, "robot": robot, "a3c": str(J.a3c),
                    "a3c_sha256": J.a3c_sha256, "compile_id": C.compile_id, "scene_id": C.scene.scene_id,
                    "tau": float(prep.scene.tau), "level": float(prep.scene.level),
                    "margin_m": float(C.config.margin_m), "body": asdict(C.body), "z_c": J.z_c,
                    "ground_z": J.ground_z, "box_uv": box,
                    "known_route_lower_m": list(map(float, ks.lower_route_m)),
                    "known_route_upper_m": list(map(float, ks.upper_route_m)),
                    "known_space_type": f"{type(ks).__module__}.{type(ks).__name__}",
                    "known_frame_equals_plan_frame": bool(
                        np.allclose(np.asarray(ks.world_to_route, float), R, atol=1e-12)
                        and np.allclose(np.asarray(ks.origin_world_m, float), np.asarray(C.frame.origin), atol=1e-9)),
                    "frame": C.frame.json(), "n_gaussians": int(len(prep.ids)),
                    "prepared_stats": {k: v for k, v in prep.stats.items()},
                    "definition": "the judge's own obstacle set: PreparedScene.means/covs/ids of the persisted "
                                  "compile (opacity > tau, covariances as the judge floors them), rotated into the "
                                  "plan frame (= route frame: u, v, height above the floor); obstacle = the "
                                  "level-sigma ellipsoid {x: (x-m)^T C^-1 (x-m) <= level^2}; body = vertical "
                                  "cylinder centred at z_c, collision-free means clearance > margin_m"}
            out = scene_export_path(region, robot)
            out.parent.mkdir(parents=True, exist_ok=True)
            tmp = out.with_name(out.name + ".tmp.npz")
            np.savez(tmp, means=means, covs=covs, ids=np.asarray(prep.ids), meta=json.dumps(meta))
            tmp.replace(out)
            meta.update(sha256=sha256_file(out), export_wall_s=time.perf_counter() - t0)
            _dump(Path(f"{out}.json"), meta)
            print(region, robot, meta["n_gaussians"], "gaussians, tau", meta["tau"], "level", meta["level"],
                  "known==plan", meta["known_frame_equals_plan_frame"], f"{meta['export_wall_s']:.1f}s", flush=True)


def load_config(method, robot, config_file=None, override=None):
    """Method config = common + robot section of ``configs/baselines/<method>.json`` (or a tuning file)."""
    doc = json.loads(Path(config_file).read_text()) if config_file else {"common": {}, "robots": {}}
    cfg = dict(doc.get("common", {}))
    cfg.update(doc.get("robots", {}).get(robot, {}))
    if override:
        cfg.update(json.loads(override))
    return cfg


def adapter_sha(method):
    """SHA-256 of the method's adapter source: a code change must never reuse an artifact built by older code."""
    from bl_worker import ADAPTERS
    return sha256_file(Path(__file__).resolve().parent / f"{ADAPTERS[method]}.py")


def artifact_path(method, region, robot, config):
    key = {"config": {k: v for k, v in config.items() if not k.startswith("q_")}, "adapter_sha256": adapter_sha(method)}
    return OUTB / method / sha_json(key)[:12] / f"{region}_{robot}.setup.pkl"


def cmd_setup(a):
    """Build + persist the method's setup for each region x robot (in the method's env)."""
    for region in a.regions:
        for robot in a.robots:
            cfg = load_config(a.method, robot, a.config, a.set)
            scene = scene_export_path(region, robot)
            side = json.loads(Path(f"{scene}.json").read_text())
            if sha256_file(scene) != side["sha256"]:
                raise SystemExit(f"scene export {scene} differs from its sidecar")
            art = artifact_path(a.method, region, robot, cfg)
            if art.exists() and Path(f"{art}.json").exists() and not a.force:
                print("exists", art, flush=True)
                continue
            art.parent.mkdir(parents=True, exist_ok=True)
            setup_cfg = {k: v for k, v in cfg.items() if not k.startswith("q_")}
            py = str(METHOD_PYTHON.get(a.method, sys.executable))
            here = Path(__file__).resolve().parent
            env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", MPLBACKEND="Agg",
                       XDG_CACHE_HOME="/scratch/wg2381/.cache", MPLCONFIGDIR="/scratch/wg2381/.cache/mpl",
                       WARP_CACHE_PATH="/scratch/wg2381/.cache/warp",
                       PYTHONPATH=os.pathsep.join([str(here), str(here.parent / "src")]) if py == sys.executable
                       else str(here))
            t0 = time.perf_counter()
            r = subprocess.run([py, "-B", str(here / "bl_worker.py"), "--mode", "setup", "--method", a.method,
                                "--artifact", str(art.resolve()), "--scene", str(scene.resolve()),
                                "--config", json.dumps(setup_cfg)], env=env, cwd=art.parent,
                               capture_output=True, text=True)
            wall = time.perf_counter() - t0
            log = art.with_suffix(".log")
            log.write_text(r.stdout + "\n--- stderr ---\n" + r.stderr)
            rec = {"method": a.method, "region": region, "robot": robot, "config": setup_cfg,
                   "artifact": str(art), "returncode": r.returncode, "outer_wall_s": wall, "host": host(),
                   "scene_export_sha256": side["sha256"], "log": str(log)}
            if r.returncode == 0:
                rec["artifact_meta"] = json.loads(Path(f"{art}.json").read_text())
            else:
                rec["stderr_tail"] = r.stderr[-3000:]
            _dump(RES / a.method / "setup" / art.parent.name / f"{region}_{robot}.json", rec)
            print(region, robot, "rc", r.returncode, f"{wall:.1f}s", art, flush=True)
            if r.returncode != 0:
                print(r.stderr[-3000:], flush=True)


def cmd_run(a):
    pairs = load_pairs(a.pairs, a.region)
    if a.limit:
        pairs = pairs[:a.limit]
    sl = task_slice(len(pairs), a.n_tasks, a.task)
    pairs = [pairs[i] for i in sl]
    cfg = load_config(a.method, a.robot, a.config, a.set)
    art = Path(a.artifact) if a.artifact else artifact_path(a.method, a.region, a.robot, cfg)
    out_dir = Path(a.out) / a.region / a.robot
    out = out_dir / f"task_{a.task:02d}.jsonl"
    print(f"{a.method} {a.region} {a.robot} task {a.task}/{a.n_tasks}: {len(pairs)} pairs, artifact {art}", flush=True)
    s = run_task(a.method, a.region, a.robot, pairs, out, artifact=art, config=cfg, timeout_s=a.timeout,
                 judge_mode=a.judge, pass_pair=a.method == "astar_replay", config_file=a.config)
    s.update(task=a.task, n_tasks=a.n_tasks, pairs_source=a.pairs, pair_ids=[p["pair_id"] for p in pairs])
    _dump(out_dir / f"task_{a.task:02d}.summary.json", s)
    print(json.dumps({k: s[k] for k in ("complete", "status_counts", "answered_this_run",
                                         "query_outer_wall_s_this_run", "judge_wall_s_this_run")}), flush=True)


def cmd_judge(a):
    """Fill deferred judge fields (rows written by ``run --judge defer``), rewriting the task file atomically."""
    for path in a.files:
        path = Path(path)
        rows = [json.loads(x) for x in open(path) if x.strip()]
        todo = [r for r in rows if r["status"] == "CLAIMED_PENDING_JUDGE"]
        if not todo:
            print(path, "nothing pending")
            continue
        J = Judge(rows[0]["region"], rows[0]["robot"])
        for r in todo:
            try:
                judge_row(J, r)
                r.pop("method_path_uv")
            except ValueError as exc:
                r.update(status="ERROR", reason=f"unexportable_path: {exc}"[:500])
        tmp = path.with_name(path.name + ".tmp")
        with open(tmp, "w") as f:
            for r in rows:
                _write(f, r)
        tmp.replace(path)
        print(path, "judged", len(todo), collections.Counter(r["status"] for r in rows), flush=True)


def cmd_sample(a):
    """Stratified F4 pair list: region x lateral-clearance band, proportional with >= 1 per stratum, fixed seed."""
    pairs = load_pairs("f4")
    strata = collections.defaultdict(list)
    for p in pairs:
        strata[(p["region"], _lat_band(p["lateral_clearance_m"]))].append(p["pair_id"])
    rng = np.random.default_rng(a.seed)
    keys = sorted(strata)
    alloc = {k: max(1, int(np.floor(a.n * len(strata[k]) / len(pairs)))) for k in keys}
    while sum(alloc.values()) < a.n:           # largest remainders
        rem = sorted(keys, key=lambda k: -(a.n * len(strata[k]) / len(pairs) - alloc[k]))
        for k in rem:
            if sum(alloc.values()) >= a.n:
                break
            if alloc[k] < len(strata[k]):
                alloc[k] += 1
    while sum(alloc.values()) > a.n:
        k = max(keys, key=lambda k: alloc[k])
        alloc[k] -= 1
    pick = []
    for k in keys:
        pick += sorted(rng.choice(strata[k], min(alloc[k], len(strata[k])), replace=False).tolist())
    order = {p["pair_id"]: i for i, p in enumerate(pairs)}
    pick.sort(key=order.get)
    _dump(a.out, {"schema": "bl.pair_list.v1", "purpose": a.purpose, "source": str(F4 / "pairs_confirmed_5000.json"),
                  "n": len(pick), "seed": a.seed,
                  "rule": "strata = region x F4 lateral-clearance band (aerial3dg_fail3_f4.LAT_BANDS); allocation "
                          "proportional to stratum size, >= 1 per stratum, largest remainders; uniform without "
                          "replacement inside a stratum (numpy default_rng(seed))",
                  "strata": {f"{k[0]}/{k[1]}": {"population": len(strata[k]), "picked": alloc[k]} for k in keys},
                  "pair_ids": pick})
    print(a.out, len(pick), {f"{k[0]}/{k[1]}": alloc[k] for k in keys})


def iter_rows(root):
    """Rows of every task file under ``root`` (``task_NN.jsonl``, or ``.jsonl.gz`` once archived)."""
    import gzip
    for f in sorted(list(Path(root).rglob("task_*.jsonl")) + list(Path(root).rglob("task_*.jsonl.gz"))):
        with (gzip.open(f, "rt") if f.suffix == ".gz" else open(f)) as fh:
            for x in fh:
                if x.strip():
                    yield json.loads(x)


def cmd_archive(a):
    """Compress finished task files for commit: drop the raw ``method_path_uv`` of judged rows (``route_polyline``
    is the exact judged path) and gzip ``task_NN.jsonl`` -> ``.jsonl.gz`` (rows otherwise byte-for-byte)."""
    import gzip
    for f in sorted(Path(a.root).rglob("task_*.jsonl")):
        rows = [json.loads(x) for x in open(f) if x.strip()]
        if any(r["status"] == "CLAIMED_PENDING_JUDGE" for r in rows):
            print("skip (pending judge)", f)
            continue
        with gzip.open(f"{f}.gz", "wt") as g:
            for r in rows:
                if r.get("route_polyline") is not None:
                    r.pop("method_path_uv", None)
                g.write(json.dumps(r, default=float) + "\n")
        f.unlink()
    print("archived", a.root)


def summarize(rows):
    rows = list(rows)
    st = collections.Counter(r["status"] for r in rows)
    t = np.array([r["algorithm_wall_s"] for r in rows if r.get("algorithm_wall_s") is not None])
    o = np.array([r["outer_wall_s"] for r in rows if r.get("outer_wall_s") is not None])
    jw = np.array([r["judge_wall_s"] for r in rows if r.get("judge_wall_s") is not None])
    q = lambda x, p: float(np.percentile(x, p)) if len(x) else None
    return {"n": len(rows), "status_counts": dict(st),
            "success_rate": st["SUCCESS"] / len(rows) if rows else None,
            "appended_goal_segment": sum(bool(r.get("appended_goal_segment")) for r in rows),
            "prepended_start_segment": sum(bool(r.get("prepended_start_segment")) for r in rows),
            "algorithm_wall_s": {"median": q(t, 50), "p95": q(t, 95), "max": float(t.max()) if len(t) else None,
                                 "sum": float(t.sum())},
            "outer_wall_s": {"median": q(o, 50), "p95": q(o, 95), "sum": float(o.sum())},
            "judge_wall_s": {"median": q(jw, 50), "p95": q(jw, 95), "sum": float(jw.sum())}}


def cmd_report(a):
    rows = list(iter_rows(a.root))
    by = collections.defaultdict(list)
    for r in rows:
        by[(r["robot"], r["region"])].append(r)
        by[(r["robot"], "ALL")].append(r)
    doc = {"/".join(k): summarize(v) for k, v in sorted(by.items())}
    if a.out:
        _dump(a.out, doc)
    for k, v in doc.items():
        print(k, v["n"], v["status_counts"], "median alg", v["algorithm_wall_s"]["median"])


def cmd_project(a):
    """Cost projection for the full run (5000 pairs x 2 robots per method) from pilot task files.

    Per method and robot: method time = mean ``outer_wall_s`` per query (parent send -> receive: what the worker holds
    the GPU for; TIMEOUT rows count their full 120 s) x the F4 pair count of each region, + setup (build once per
    region; one worker instantiate per task); judge CPU = mean ``judge_wall_s`` x pairs (CPU, separable with
    ``--judge defer``). GPU-h assume ``--streams`` concurrent harness streams per GPU at the measured per-stream
    speed (the pilot ran that many streams on one L40S), ``--gpus`` GPU jobs at a time (plan: <= 2)."""
    regions = json.loads((F4 / "pairs_confirmed_5000.json").read_text())["regions"]
    n_reg = {r: v["quota"] for r, v in regions.items()}
    doc = {"definition": cmd_project.__doc__.strip(), "pairs_per_region": n_reg, "streams_per_gpu": a.streams,
           "gpu_jobs_at_a_time": a.gpus, "tasks_per_region_robot": a.tasks, "methods": {}}
    for mdir in a.roots:
        rows = list(iter_rows(mdir))
        if not rows:
            continue
        method = rows[0]["method"]
        sums = {}
        for f in sorted(Path(mdir).rglob("task_*.summary.json")):
            s = json.loads(f.read_text())
            sums[(s["region"], s["robot"])] = s
        m = {"pilot_root": str(mdir), "robots": {}}
        tot = {"method_gpu_h": 0., "judge_cpu_h": 0., "setup_h": 0.}
        for robot in ROBOTS:
            rr = [r for r in rows if r["robot"] == robot]
            if not rr:
                continue
            per = {}
            gpu_s = judge_s = setup_s = 0.
            for reg, n in n_reg.items():
                x = [r for r in rr if r["region"] == reg]
                if not x:
                    continue
                ow = [r["outer_wall_s"] for r in x if r.get("outer_wall_s") is not None]
                if not ow:
                    continue
                o = float(np.mean(ow))
                jw = [r["judge_wall_s"] for r in x if r.get("judge_wall_s") is not None]
                jm = float(np.mean(jw)) if jw else 0.
                s = sums.get((reg, robot), {})
                build = (s.get("setup") or {}).get("build_setup_wall_s") or 0.
                inst = [t for t in (s.get("setup_once_proof") or {}).get("instantiate_s_per_start", []) if t]
                inst_m = float(np.mean(inst)) if inst else 0.
                per[reg] = {"pilot_rows": len(x), "mean_outer_wall_s": float(o), "mean_judge_wall_s": jm,
                            "build_setup_wall_s": build, "instantiate_s": inst_m,
                            "projected_method_s": float(o * n + inst_m * a.tasks), "projected_judge_s": jm * n}
                gpu_s += per[reg]["projected_method_s"]
                judge_s += per[reg]["projected_judge_s"]
                setup_s += build
            m["robots"][robot] = {"summary": summarize(rr), "regions": per,
                                  "method_h_one_stream": gpu_s / 3600, "judge_cpu_h": judge_s / 3600,
                                  "setup_build_h": setup_s / 3600}
            tot["method_gpu_h"] += gpu_s / 3600 / a.streams
            tot["judge_cpu_h"] += judge_s / 3600
            tot["setup_h"] += setup_s / 3600
        tot["wall_h_at_concurrency"] = tot["method_gpu_h"] / a.gpus
        tot["cpu_h_harness_parents"] = tot["method_gpu_h"] * a.streams       # one CPU per harness stream
        m["projection_5000x2"] = tot
        doc["methods"][method] = m
        print(method, json.dumps(tot))
    _dump(a.out, doc)


def cmd_tunetable(a):
    """Tuning table per method x robot x candidate + the pre-registered pick (configs/baselines/tuning/RULE.md):
    most SUCCESS, then fewest CLAIMED_*, then lowest median algorithm_wall_s; ``--ineligible`` names are reported only."""
    root = RES / "tuning" / a.method
    table, pick = {}, {}
    for d in sorted(p for p in root.iterdir() if p.is_dir()):
        for robot in ROBOTS:
            rows = [r for r in iter_rows(d / "WWEST" / robot)] + [r for r in iter_rows(d / "GAPW1" / robot)] + \
                [r for r in iter_rows(d / "S" / robot)]
            if not rows:
                continue
            sm = summarize(rows)
            st = sm["status_counts"]
            claimed_bad = sum(v for k, v in st.items() if k.startswith("CLAIMED_"))
            table.setdefault(robot, {})[d.name] = {
                "n": sm["n"], **{k: st.get(k, 0) for k in STATUSES[:-1]}, "claimed_unsafe": claimed_bad,
                "median_alg_s": sm["algorithm_wall_s"]["median"], "p95_alg_s": sm["algorithm_wall_s"]["p95"],
                "appended_goal": sm["appended_goal_segment"], "prepended_start": sm["prepended_start_segment"],
                "median_judge_s": sm["judge_wall_s"]["median"],
                "max_abs_dz_m": max((r.get("max_abs_dz_m") or 0.) for r in rows),
                "eligible": d.name not in a.ineligible}
    for robot, t in table.items():
        el = {k: v for k, v in t.items() if v["eligible"] and v["n"] == 50}
        pick[robot] = min(el, key=lambda k: (-el[k]["SUCCESS"], el[k]["claimed_unsafe"],
                                             el[k]["median_alg_s"] or 1e9)) if el else None
    doc = {"method": a.method, "rule": "configs/baselines/tuning/RULE.md", "ineligible": a.ineligible,
           "table": table, "pick": pick}
    _dump(root / "tuning_table.json", doc)
    for robot, t in table.items():
        print(f"== {a.method} {robot}  pick: {pick[robot]}")
        for k, v in t.items():
            print(f"  {k:24s} n={v['n']:3d} S={v['SUCCESS']:3d} COLL={v['CLAIMED_COLLIDES']:3d} "
                  f"UNPR={v['CLAIMED_UNPROVEN']:3d} KIN={v['CLAIMED_KINEMATICS']:2d} FAIL={v['FAIL']:3d} "
                  f"TO={v['TIMEOUT']:2d} ERR={v['ERROR']:2d} med={v['median_alg_s']} app={v['appended_goal']} "
                  f"pre={v['prepended_start']} dz={v['max_abs_dz_m']:.3f}{'' if v['eligible'] else '  (ineligible)'}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("export")
    e.add_argument("--regions", nargs="+", default=list(REGIONS))
    e.add_argument("--robots", nargs="+", default=list(ROBOTS))
    s = sub.add_parser("setup")
    s.add_argument("--method", required=True)
    s.add_argument("--config")
    s.add_argument("--set", help="json overrides")
    s.add_argument("--regions", nargs="+", default=list(REGIONS))
    s.add_argument("--robots", nargs="+", default=list(ROBOTS))
    s.add_argument("--force", action="store_true")
    r = sub.add_parser("run")
    r.add_argument("--method", required=True)
    r.add_argument("--region", required=True)
    r.add_argument("--robot", required=True)
    r.add_argument("--pairs", default="f4", help="f4 | tuning | committed pair-id list json")
    r.add_argument("--config")
    r.add_argument("--set", help="json overrides")
    r.add_argument("--artifact")
    r.add_argument("--out", required=True)
    r.add_argument("--task", type=int, default=0)
    r.add_argument("--n-tasks", type=int, default=1)
    r.add_argument("--limit", type=int, default=0)
    r.add_argument("--timeout", type=float, default=QUERY_TIMEOUT_S)
    r.add_argument("--judge", choices=("inline", "defer"), default="inline")
    j = sub.add_parser("judge")
    j.add_argument("files", nargs="+")
    sm = sub.add_parser("sample")
    sm.add_argument("--n", type=int, required=True)
    sm.add_argument("--seed", type=int, required=True)
    sm.add_argument("--purpose", required=True)
    sm.add_argument("--out", type=Path, required=True)
    rp = sub.add_parser("report")
    rp.add_argument("root")
    rp.add_argument("--out", type=Path)
    pj = sub.add_parser("project")
    pj.add_argument("roots", nargs="+")
    pj.add_argument("--streams", type=int, default=1)
    pj.add_argument("--gpus", type=int, default=2)
    pj.add_argument("--tasks", type=int, default=1, help="tasks (worker starts) per region x robot in the full run")
    pj.add_argument("--out", type=Path, required=True)
    ar = sub.add_parser("archive")
    ar.add_argument("root")
    tt = sub.add_parser("tunetable")
    tt.add_argument("--method", required=True)
    tt.add_argument("--ineligible", nargs="*", default=[])
    a = ap.parse_args(argv)
    {"export": cmd_export, "setup": cmd_setup, "run": cmd_run, "judge": cmd_judge, "sample": cmd_sample,
     "report": cmd_report, "project": cmd_project,
     "tunetable": cmd_tunetable, "archive": cmd_archive}[a.cmd](a)


if __name__ == "__main__":
    main()
