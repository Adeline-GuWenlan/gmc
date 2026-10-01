"""F1 Task 2 truth check: the gs3d lattice A* baseline on a stratified sample of each failure class.

The baseline is the one ground5k used (``gmc.gs3d.planner.LatticePlanner``, ``experiments/ground5k_run.py
_run_astar``: margin 0.001, position tolerance 0, yaw tolerance 0.05, same budget counts) on the SAME scene /
route box the G2 compile used, at the 0.1 m lattice of the uav-lamp ground runs.  One PreparedScene per job,
reused by every query (the planner's own warm mode).

Outcome (ground5k ``classify_astar`` semantics, the box faces being the query domain as for aerial3d):
  ROUTE        A* found a route and its own post-build verification passed  -> a route exists
  NO_ROUTE     reachable 0.1 m lattice exhausted, only occupied / outside-box rejections -> evidence (not proof)
               that none exists: windows narrower than the lattice can be missed
  UNSURE       budget exhausted, or the frontier had unproven edges (geometry_or_margin / support), or an
               endpoint the A* itself does not accept

  sample   pick the stratified sample (seeded) from G2 rows + F1 classes  -> configs/aerial3dg/f1_oracle_sample.json
  run      run one robot's share of the sample (sbatch)
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np

from aerial3dg_run import ROBOTS, _box, ground_world, load_booth
from aerial3dg_fail_diag import _dump

G2_BOX = (-9.0, -0.35, 3.7, 2.75)
BUDGET = {"max_expansions": 500000, "max_oracle_calls": 5000000, "max_narrowphase_pairs": 20000000}


def classify(res) -> str:
    st, d = res["status"], res["diagnostics"]
    if st == "success":
        return "ROUTE"
    if st in ("map_unknown", "no_path_on_lattice", "verification_failed") and \
            res["reason"] in ("reachable_frontier_meets_unknown_coverage", "reachable_lattice_exhausted",
                              "reachable_frontier_has_unproven_edges"):
        return "NO_ROUTE" if d.get("unproven_rejections", 0) == 0 else "UNSURE"
    return "UNSURE"


def cmd_sample(a):
    classes = json.loads(Path(a.classes).read_text())   # {robot: {class: [index,...]}}
    rng = np.random.default_rng(a.seed)
    pairs = json.loads(Path("results/aerial3dg/g2/pairs_5000.json").read_text())["pairs"]
    out = {"seed": a.seed, "per_class": a.n, "rule": "per (robot, class): if <= n rows take all, else n rows "
           "stratified over straight-line-distance terciles (equal share, seeded)", "robots": {}}
    for robot, cl in classes.items():
        out["robots"][robot] = {}
        for name, idx in cl.items():
            idx = np.asarray(sorted(idx))
            if len(idx) <= a.n:
                pick = idx
            else:
                d = np.array([pairs[i]["dist_m"] for i in idx])
                edges = np.quantile(d, [0, 1 / 3, 2 / 3, 1])
                bins = np.clip(np.searchsorted(edges, d, side="right") - 1, 0, 2)
                pick = []
                for b in range(3):
                    pool = idx[bins == b]
                    k = a.n // 3 + (1 if b < a.n % 3 else 0)
                    pick += rng.choice(pool, size=min(k, len(pool)), replace=False).tolist()
                pick = np.asarray(sorted(pick))
            out["robots"][robot][name] = [int(i) for i in pick]
    Path(a.out).write_text(json.dumps(out, indent=1) + "\n")
    print({r: {k: len(v) for k, v in c.items()} for r, c in out["robots"].items()})


def cmd_run(a):
    from gmc.gs3d.contracts import GoalRegion, PlannerConfig, Pose3, SearchBudget
    from gmc.gs3d.oracle import PreparedScene
    from gmc.gs3d.planner import LatticePlanner
    sample = json.loads(Path(a.sample).read_text())["robots"][a.robot]
    pairs = json.loads(Path("results/aerial3dg/g2/pairs_5000.json").read_text())["pairs"]
    ctx = load_booth(_box(G2_BOX))
    frame, scene, body = ctx["frame"], ctx["scene"], ROBOTS[a.robot]
    t0 = time.perf_counter()
    prepared = PreparedScene(scene)
    prep_s = time.perf_counter() - t0
    planner = LatticePlanner(prepared)
    out_path = Path(a.out)
    done = {}
    if out_path.exists():
        done = {(r["class"], r["index"]): r for r in json.loads(out_path.read_text())["rows"]}
    rows = list(done.values())
    for cls, idx in sample.items():
        for i in idx:
            if (cls, i) in done:
                continue
            p = pairs[i]
            s = Pose3(tuple(map(float, ground_world(frame, body, p["start_uv"]))), 0.)
            g = Pose3(tuple(map(float, ground_world(frame, body, p["goal_uv"]))), 0.)
            conf = PlannerConfig(resolution_m=a.resolution, margin_m=.001, seed=0,
                                 budget=SearchBudget(max_wall_s=a.max_wall, **BUDGET))
            w0 = time.perf_counter()
            res = planner.plan(scene, body, s, GoalRegion(g, 0., .05), conf)
            d = res["diagnostics"]
            poly = None
            if res["status"] == "success":
                poly = frame.to_route(np.asarray(res["trajectory"]["poses"], float)[:, :3])[:, :2].round(3).tolist()
            row = {"class": cls, "index": i, "pair_id": p["pair_id"], "dist_m": p["dist_m"],
                   "outcome": classify(res), "status": res["status"], "reason": res["reason"],
                   "path_length_m": d.get("path_length_m"), "wall_s": time.perf_counter() - w0,
                   **{k: d.get(k) for k in ("expansions", "oracle_calls", "map_unknown_rejections",
                                            "unproven_rejections", "occupied_rejections")},
                   "route_uv": poly}
            rows.append(row)
            print(a.robot, cls, p["pair_id"], row["outcome"], res["status"], res["reason"], round(row["wall_s"], 1),
                  flush=True)
            _dump(out_path, {"robot": a.robot, "resolution_m": a.resolution, "margin_m": .001,
                             "box_uv": G2_BOX, "prepare_s": prep_s, "budget": {**BUDGET, "max_wall_s": a.max_wall},
                             "rows": rows})
    summ = {}
    for r in rows:
        summ.setdefault(r["class"], {}).setdefault(r["outcome"], 0)
        summ[r["class"]][r["outcome"]] += 1
    _dump(out_path, {"robot": a.robot, "resolution_m": a.resolution, "margin_m": .001, "box_uv": G2_BOX,
                     "prepare_s": prep_s, "budget": {**BUDGET, "max_wall_s": a.max_wall}, "summary": summ,
                     "rows": rows})
    print(json.dumps(summ), flush=True)


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("sample")
    s.add_argument("--classes", required=True)
    s.add_argument("--n", type=int, default=30)
    s.add_argument("--seed", type=int, default=20261001)
    s.add_argument("--out", required=True)
    r = sub.add_parser("run")
    r.add_argument("--robot", required=True)
    r.add_argument("--sample", required=True)
    r.add_argument("--resolution", type=float, default=.1)
    r.add_argument("--max-wall", type=float, default=300.)
    r.add_argument("--out", required=True)
    a = p.parse_args(argv)
    {"sample": cmd_sample, "run": cmd_run}[a.cmd](a)


if __name__ == "__main__":
    main()
