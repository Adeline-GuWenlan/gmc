"""bl J: verify the replay-judge fix (d757729) at scale, then write GMC's re-judged F4 rows.

Run from ``gmc/`` (the baselines worktree).  ``--code new`` runs with ``PYTHONPATH=src:experiments`` (HEAD);
``--code old`` with ``PYTHONPATH=<old>/src:<old>/experiments`` where ``<old>`` = ``outputs/baselines/j/old/gmc``
(``git archive 2a68d73``), and with ``python -P`` so this file's directory (HEAD experiments) is not put on
``sys.path``.  Every record carries the code actually imported (``code_label``: oracle / validation source files and
whether they contain the fix), and a run aborts if that disagrees with ``--code``.

The judge = ``api._gs3d_result(compiled, api._densify(poly_w, QCONFIG.export_max_segment_m), goal_w, .)`` then
``trajectory.replay_plan(result, GaussianBodyOracle(compiled.prepared))``: exactly the export + shared replay GMC's
``query`` runs (``api.py:413-417``), so every method is judged by it in later stages.

  plan      (inline) shard lists under results/baselines/j/plan/ + plan_*.txt (one line per array task)
  astar     (sbatch) (b) per pair of a shard, on the region's CYLINDER compile:
              ``route``  F5 (a) verbatim (``aerial3dg_fail5_trace.Ctx.route_poses`` + ``verify_path`` with the
                         compile oracle, margin 0.001, goal region): the stored A* route (0.1 mm rounded) as unicycle
                         poses; if it fails, the A* is re-run (``Ctx.rerun_astar``) and its exact poses verified.
              ``export`` the same route as a polyline through the judge above (what a baseline's path faces).
  straight  (sbatch) (e) negative control, per pair and robot (row robot's compile): the straight start -> goal
              segment (``export``: judge verdict; ``edges``: every densified 0.20 m edge's own oracle verdict, so a
              change on an edge after the first failure is seen too; ``overspeed``: the export trajectory with its
              times shrunk by 1e-6 / 1e-8 (relative) must fail kinematics).  Run once per code.
  widecheck (sbatch) (e+) every straight edge that left map_unknown, re-judged on a 1 m wider crop of the archive
  requery   (sbatch) (c)/(d) GMC re-run with ``aerial3dg_batch.run_task`` (G2 QCONFIG, 120 s, compile-once proof) on
              the persisted F3 compile after checking its SHA-256 against the sidecar and F3's handoff.
  collect   (inline) tables for (b)-(e), gmc_rejudged/<R>/<robot>/rows.jsonl + summary.json, judge_check.json
"""
from __future__ import annotations

import argparse
import collections
import csv
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import time

import numpy as np

F3 = Path("results/aerial3dg/f3")
F4 = Path("results/aerial3dg/f4")
J = Path("results/baselines/j")
OUT = Path("results/baselines/gmc_rejudged")
A3C_ROOT = Path("/scratch/wg2381/splathjb-aerial3dg-fail/gmc/outputs/aerial3dg/f3")
REGIONS = ("WWEST", "GAPW1", "S")
ROBOTS = ("cylinder", "sweeper")
MARGIN = .001
VETO = ("EXPORT-DOMAIN", "EXPORT-KIN")


def _jsonl(p):
    return [json.loads(x) for x in open(p) if x.strip()]


def _dump(path, doc):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=1, default=float) + "\n")


def code_label():
    import gmc.gs3d.oracle as o
    import gmc.gs3d.validation as v
    swept = "contains_swept_cylinder" in inspect.getsource(o.GaussianBodyOracle.edge)
    ulp = "t_ulp" in inspect.getsource(v.verify_linear_trajectory)
    label = "new" if swept and ulp else "old" if not (swept or ulp) else "mixed"
    return {"label": label, "oracle_file": o.__file__, "validation_file": v.__file__}


def check_code(want):
    c = code_label()
    if c["label"] != want:
        raise SystemExit(f"--code {want} but imported {c}")
    return c


def sha256_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 22), b""):
            h.update(b)
    return h.hexdigest()


def verified_a3c(region, robot):
    """Path of F3's persisted compile after checking its SHA-256 = sidecar = F3 handoff = F4 compile_once."""
    p = A3C_ROOT / region / f"{robot}.a3c"
    side = json.loads(Path(f"{p}.json").read_text())["sha256"]
    hand = json.loads((F3 / "f4_handoff.json").read_text())["regions"][region]["compiles"][robot]["sha256"]
    once = json.loads((F4 / "compile_once.json").read_text())["per_region_robot"][f"{region}/{robot}"]["a3c_sha256"]
    got = sha256_file(p)
    if not got == side == hand == once:
        raise SystemExit(f"SHA-256 mismatch for {p}: file {got} sidecar {side} handoff {hand} compile_once {once}")
    return p, got


# ---------------------------------------------------------------------------------------------- plan (inline)
def cmd_plan(a):
    doc = json.loads((F4 / "pairs_confirmed_5000.json").read_text())
    by = collections.defaultdict(list)
    for p in doc["pairs"]:
        by[p["region"]].append(p)
    d = J / "plan"
    d.mkdir(parents=True, exist_ok=True)
    lines = collections.defaultdict(list)
    for reg in REGIONS:
        ps = by[reg]
        for k in range(0, len(ps), a.shard):
            f = d / f"pairs_{reg}_{k // a.shard:02d}.json"
            _dump(f, [{x: p[x] for x in ("pair_id", "region", "start_uv", "goal_uv", "astar_route_uv")}
                      for p in ps[k:k + a.shard]])
            for code in ("new", "old"):
                lines[f"astar_{code}"].append(f"{code} astar --code {code} --region {reg} --shard {f} "
                                              f"--out {J}/astar/{code}/{f.stem}.jsonl")
                for robot in ROBOTS:
                    lines[f"straight_{code}"].append(
                        f"{code} straight --code {code} --region {reg} --robot {robot} --shard {f} "
                        f"--out {J}/straight/{code}/{robot}/{f.stem}.jsonl")
    # (c) vetoed rows + (d) stratified REACHABLE sample, as run_task pair lists (F4's sample entries)
    fails = list(csv.DictReader(open(F4 / "failures.csv")))
    sample = {}
    for reg in REGIONS:
        sample.update({p["pair_id"]: p for p in json.loads((F4 / "sample" / f"{reg}_pairs.json").read_text())["pairs"]})
    ratio = {p["pair_id"]: p["len_ratio"] for p in doc["pairs"]}
    from aerial3dg_fail3_f4 import RATIO_BANDS, gmc_rows
    rng = np.random.default_rng(20261009)
    strata = {}
    for reg in REGIONS:
        for robot in ROBOTS:
            rows = [r for r in gmc_rows(reg, robot).values() if r["status"] == "REACHABLE"]
            for lo, hi in RATIO_BANDS:
                s = sorted(r["pair_id"] for r in rows if lo <= ratio[r["pair_id"]] < hi)
                if s:
                    strata[(reg, robot, f"[{lo:g},{hi:g})")] = s
    total = sum(len(v) for v in strata.values())
    pick = {}
    for k, s in strata.items():                    # proportional to a.n_regression, >= a.min_stratum per stratum
        n = min(len(s), max(a.min_stratum, round(a.n_regression * len(s) / total)))
        pick[k] = sorted(rng.choice(s, n, replace=False).tolist())
    _dump(d / "regression_strata.json", {"seed": 20261009, "n_target": a.n_regression, "min_stratum": a.min_stratum,
                                         "strata": {"/".join(k): {"population": len(strata[k]), "picked": v}
                                                    for k, v in pick.items()}})
    for reg in REGIONS:
        for robot in ROBOTS:
            vet = sorted((r["pair_id"] for r in fails if r["region"] == reg and r["robot"] == robot
                          and r["class"] in VETO), key=lambda x: sample[x]["index"])
            reg_ids = sorted((x for k, v in pick.items() if k[0] == reg and k[1] == robot for x in v),
                             key=lambda x: sample[x]["index"])
            for name, ids in (("vetoed", vet), ("regression", reg_ids)):
                if not ids:
                    continue
                f = d / f"requery_{reg}_{robot}_{name}.json"
                _dump(f, [sample[x] for x in ids])
                lines["requery"].append(f"new requery --region {reg} --robot {robot} --pairs {f} "
                                        f"--out {J}/requery/{reg}/{robot}/{name}.jsonl")
    for k, v in lines.items():
        (d / f"plan_{k}.txt").write_text("\n".join(v) + "\n")
        print(k, len(v), "tasks")
    print("vetoed", sum(r["class"] in VETO for r in fails), "regression", sum(len(v) for v in pick.values()),
          "strata", len(pick))


# ---------------------------------------------------------------------------------------------- helpers (sbatch)
def make_ctx(region, robot):
    """F5's ``Ctx`` (frames, A* poses, A* re-run) on the verified persisted compile, without F5's npz side output."""
    from aerial3dg_fail5_trace import Ctx
    from gmc.aerial3d.api import load_compiled
    from gmc.gs3d.oracle import GaussianBodyOracle
    path, sha = verified_a3c(region, robot)
    X = Ctx.__new__(Ctx)
    t0 = time.perf_counter()
    X.region, X.robot, X.a3c = region, robot, path
    X.C = load_compiled(path)
    X.load_s = time.perf_counter() - t0
    X.oracle = GaussianBodyOracle(X.C.prepared)
    X.z_c = float(X.C.domain.ground_z)
    b = X.C.body
    assert b.name == robot and abs(X.z_c - (b.ground_clearance_m + b.half_height_m)) < 1e-6, (X.z_c, b)
    ks = X.C.prepared.scene.known_space
    X.ks = getattr(ks, "inner", ks)
    X._planner = None
    X.sha256 = sha
    X.known_space_type = f"{type(ks).__module__}.{type(ks).__name__}"
    return X


def judge(X, poly_w):
    """GMC's export + shared replay on a world polyline (exact endpoints): the judge every method faces."""
    import gmc.aerial3d.api as api
    from gmc.gs3d.oracle import GaussianBodyOracle
    from gmc.gs3d.trajectory import replay_plan
    from aerial3dg_run import QCONFIG
    P = np.asarray(poly_w, float)
    res = api._gs3d_result(X.C, api._densify(P, QCONFIG.export_max_segment_m), P[-1], 0.)
    rep = replay_plan(res, GaussianBodyOracle(X.C.prepared))
    g, k = rep["geometry"], rep["kinematics"]
    return res, {"passed": rep["passed"], "geometry_passed": g["passed"], "geometry_reason": g["reason"],
                 "geometry_safety": g["safety"], "failed_edge": None if g["passed"] else len(g["reports"]) - 1,
                 "n_poses": len(res["trajectory"]["poses"]), "clearance_lower_m": g.get("clearance_lower_m"),
                 "kinematics_passed": k["passed"], "kinematics_reason": k.get("reason"),
                 "attained_matches": rep["attained_matches"]}


def uv_world(X, uv_list, body):
    W = np.asarray([X.world(p, body) for p in uv_list], float)
    keep = [0] + [i for i in range(1, len(W)) if np.linalg.norm(W[i] - W[i - 1]) > 1e-12]
    return W[keep]


def _run_shard(a, robot, fn):
    code = check_code(a.code)
    items = json.loads(Path(a.shard).read_text())
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    done = {json.loads(x)["pair_id"] for x in open(out) if x.strip()} if out.exists() else set()
    X = make_ctx(a.region, robot)
    print(f"{a.region} {X.robot} compile {X.C.compile_id} sha {X.sha256[:16]} load {X.load_s:.1f}s "
          f"known space {X.known_space_type} code {code}", flush=True)
    meta = {"code": code, "compile_id": X.C.compile_id, "a3c_sha256": X.sha256, "load_s": X.load_s,
            "known_space": X.known_space_type, "node": os.uname().nodename, "job": os.environ.get("SLURM_JOB_ID")}
    t_all = time.perf_counter()
    n = 0
    with open(out, "a") as f:
        for it in items:
            if it["pair_id"] in done:
                continue
            t0 = time.perf_counter()
            rec = {"pair_id": it["pair_id"], "region": it["region"], **fn(X, it)}
            rec["wall_s"] = time.perf_counter() - t0
            f.write(json.dumps(rec, default=float) + "\n")
            f.flush()
            n += 1
    meta.update(answered=n, resumed=len(done), wall_s=time.perf_counter() - t_all, peak_rss_mb=_rss())
    _dump(out.with_suffix(".meta.json"), meta)
    print(json.dumps(meta), flush=True)


def _rss():
    import resource
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.


# ---------------------------------------------------------------------------------------------- (b) A* routes
def _astar(X, it, rerun=True):
    from gmc.gs3d.validation import verify_path
    from aerial3dg_run import ROBOTS as RB
    cyl = RB["cylinder"]
    s_uv, g_uv = it["start_uv"], it["goal_uv"]
    poses = X.route_poses(it["astar_route_uv"], cyl)
    v = verify_path(X.oracle, poses, cyl, margin_m=MARGIN, goal=X.goal_region(g_uv, cyl))
    rec = {"route": {"passed": v["passed"], "reason": v["reason"], "n_poses": len(poses),
                     "failed_edge": None if v["passed"] else len(v["reports"]) - 1,
                     "clearance_lower_m": v.get("clearance_lower_m"), "replayed_from": "stored_route_uv_0.1mm"}}
    poly = uv_world(X, it["astar_route_uv"], cyl)
    poly[0], poly[-1] = X.world(s_uv, cyl), X.world(g_uv, cyl)
    _, rec["export"] = judge(X, poly)
    rec["export"]["replayed_from"] = "stored_route_uv_0.1mm"
    if rerun and not (v["passed"] and rec["export"]["passed"]):
        P = X.rerun_astar(s_uv, g_uv)
        rec["rerun"] = {"found": P is not None}
        if P is not None:
            w = verify_path(X.oracle, P, cyl, margin_m=MARGIN, goal=X.goal_region(g_uv, cyl))
            rec["rerun"]["route"] = {"passed": w["passed"], "reason": w["reason"], "n_poses": len(P),
                                     "clearance_lower_m": w.get("clearance_lower_m")}
            pw = np.asarray([p.xyz for p in P], float)
            keep = [0] + [i for i in range(1, len(pw)) if np.linalg.norm(pw[i] - pw[i - 1]) > 1e-12]
            pw = pw[keep]
            pw[0], pw[-1] = X.world(s_uv, cyl), X.world(g_uv, cyl)
            _, rec["rerun"]["export"] = judge(X, pw)
    return rec


def cmd_astar(a):
    _run_shard(a, "cylinder", lambda X, it: _astar(X, it, rerun=a.code == "new"))


# ---------------------------------------------------------------------------------------------- (e) straight segment
def _straight(X, it):
    from gmc.gs3d.contracts import Pose3
    from gmc.gs3d.validation import verify_linear_trajectory
    from gmc.gs3d.trajectory import _body_from_result
    body = X.C.body
    poly = np.asarray([X.world(it["start_uv"], body), X.world(it["goal_uv"], body)], float)
    res, rec = judge(X, poly)
    rec = {"export": rec}
    # every densified edge's own verdict (translation edges only; turns in place are single poses)
    P = np.asarray(res["trajectory"]["poses"], float)
    codes, clear = [], []
    for p, q in zip(P[:-1], P[1:]):
        if np.linalg.norm(q[:3] - p[:3]) <= 1e-12:
            continue
        r = X.oracle.edge(Pose3(tuple(p[:3]), float(p[3])), Pose3(tuple(q[:3]), float(q[3])), body, margin_m=MARGIN)
        codes.append(f"{r.occupancy}|{r.safety}|{r.reason}")
        clear.append(r.clearance_lower_m)
    rec["edges"] = codes
    rec["edge_clearance_lower_m"] = clear
    # kinematics: the export as is must pass; times shrunk by a relative 1e-6 / 1e-8 must fail
    b, limits = _body_from_result(res)
    traj = res["trajectory"]
    over = {}
    for eps in (1e-6, 1e-8):
        t = dict(traj, time_s=[x * (1 - eps) for x in traj["time_s"]])
        over[f"{eps:g}"] = verify_linear_trajectory(t, b, limits)["passed"]
    rec["kin_overspeed_passed"] = over
    return rec


def cmd_straight(a):
    _run_shard(a, a.robot, _straight)


# ---------------------------------------------------------------------------------------------- (e+) wide-crop check
def cmd_widecheck(a):
    """Every straight-segment edge whose verdict went map_unknown (old) -> anything (new), re-judged by an oracle on a
    scene cropped from the archive with the region box widened by ``--pad`` m in u and v (z unchanged: route z =
    world z, so the z faces were tested exactly by both codes).  The fix is sound only if no such edge that the
    region's crop calls free is occupied or margin-unproven against the wider scene."""
    from gmc.gs3d.contracts import Pose3
    from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
    from aerial3dg_run import load_booth
    code = check_code("new")
    todo = collections.defaultdict(list)
    for robot in ROBOTS:
        new, old = {}, {}
        for d, code_ in ((new, "new"), (old, "old")):
            for f in sorted((J / "straight" / code_ / robot).glob(f"pairs_{a.region}_*.jsonl")):
                d.update({r["pair_id"]: r for r in _jsonl(f)})
        for pid in sorted(set(new) & set(old)):
            for i, (o, n) in enumerate(zip(old[pid]["edges"], new[pid]["edges"])):
                if o.endswith("|map_unknown") and n != o:
                    todo[robot].append((pid, i, o, n, new[pid]["edge_clearance_lower_m"][i]))
    rec = json.loads((F3 / "probe" / a.region / "compile_cylinder.json").read_text())["box_route"]
    box = {"lower": [rec["lower"][0] - a.pad, rec["lower"][1] - a.pad, rec["lower"][2]],
           "upper": [rec["upper"][0] + a.pad, rec["upper"][1] + a.pad, rec["upper"][2]]}
    t0 = time.perf_counter()
    ctx = load_booth(box)
    wide = GaussianBodyOracle(PreparedScene(ctx["scene"]))
    load_s = time.perf_counter() - t0
    shards = {}
    for f in sorted((J / "plan").glob(f"pairs_{a.region}_*.json")):
        shards.update({p["pair_id"]: p for p in json.loads(f.read_text())})
    out = {"region": a.region, "code": code, "box_route_region": rec, "box_route_wide": box, "pad_m": a.pad,
           "wide_crop": ctx["crop"], "load_s": load_s, "per_robot": {}}
    for robot in ROBOTS:
        X = make_ctx(a.region, robot)
        rows, cnt = [], collections.Counter()
        cache = {}
        for pid, i, o, n, c_new in todo[robot]:
            if pid not in cache:
                body = X.C.body
                poly = np.asarray([X.world(shards[pid]["start_uv"], body), X.world(shards[pid]["goal_uv"], body)])
                res, _ = judge(X, poly)
                P = np.asarray(res["trajectory"]["poses"], float)
                cache[pid] = [(p, q) for p, q in zip(P[:-1], P[1:]) if np.linalg.norm(q[:3] - p[:3]) > 1e-12]
            p, q = cache[pid][i]
            A, B = Pose3(tuple(p[:3]), float(p[3])), Pose3(tuple(q[:3]), float(q[3]))
            w = wide.edge(A, B, X.C.body, margin_m=MARGIN)
            again = X.oracle.edge(A, B, X.C.body, margin_m=MARGIN)
            wv = f"{w.occupancy}|{w.safety}|{w.reason}"
            ok = not (n.startswith("free") and not wv.startswith("free"))
            cnt[f"{n} || wide {wv}"] += 1
            rows.append({"pair_id": pid, "edge": i, "old": o, "new": n, "new_again": f"{again.occupancy}|{again.safety}|{again.reason}",
                         "wide": wv, "new_clearance_m": c_new, "wide_clearance_m": w.clearance_lower_m, "ok": ok})
        out["per_robot"][robot] = {"edges": len(rows), "verdicts": dict(cnt), "not_ok": [r for r in rows if not r["ok"]],
                                   "rows": rows}
    out["passed"] = all(not v["not_ok"] for v in out["per_robot"].values())
    _dump(J / "widecheck" / f"{a.region}.json", out)
    print(a.region, {r: (v["edges"], v["verdicts"]) for r, v in out["per_robot"].items()}, "passed", out["passed"],
          "rss", _rss(), flush=True)


# ---------------------------------------------------------------------------------------------- (c)/(d) re-query
def cmd_requery(a):
    from aerial3dg_batch import run_task
    from aerial3dg_run import MANIFEST, host
    code = check_code("new")
    path, sha = verified_a3c(a.region, a.robot)
    pairs = json.loads(Path(a.pairs).read_text())
    out = Path(a.out)
    s = run_task(path, pairs, out, timeout_s=120., manifest=json.loads(MANIFEST.read_text()))
    s.update(region=a.region, robot=a.robot, pairs_file=a.pairs, a3c_sha256_verified=sha, code=code, host=host(),
             timeout_s=120.)
    _dump(out.with_suffix(".summary.json"), s)
    print(json.dumps({k: s[k] for k in ("status_counts", "answered_this_run", "compile_once_proof", "peak_rss_mb")}),
          flush=True)


# ---------------------------------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("plan")
    p.add_argument("--shard", type=int, default=250)
    p.add_argument("--n-regression", type=int, default=360)
    p.add_argument("--min-stratum", type=int, default=3)
    for name in ("astar", "straight"):
        p = sub.add_parser(name)
        p.add_argument("--code", choices=("new", "old"), required=True)
        p.add_argument("--region", required=True)
        p.add_argument("--shard", required=True)
        p.add_argument("--out", required=True)
        if name == "straight":
            p.add_argument("--robot", choices=ROBOTS, required=True)
    p = sub.add_parser("requery")
    p.add_argument("--region", required=True)
    p.add_argument("--robot", choices=ROBOTS, required=True)
    p.add_argument("--pairs", required=True)
    p.add_argument("--out", required=True)
    p = sub.add_parser("widecheck")
    p.add_argument("--region", required=True)
    p.add_argument("--pad", type=float, default=1.)
    sub.add_parser("collect")
    a = ap.parse_args(argv)
    if a.cmd == "collect":
        from bl_judge_collect import cmd_collect
        return cmd_collect(a)
    globals()[f"cmd_{a.cmd}"](a)


if __name__ == "__main__":
    main()
