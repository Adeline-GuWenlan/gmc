"""F2 Task 1b: a cylinder pair list in which EVERY pair is independently confirmed reachable, with clearance.

Run from ``gmc/`` with ``PYTHONPATH=src:experiments`` (sbatch).

Why "clearance" needs a robust body, not a bigger oracle margin.  The shared oracle's clearance is the 3-D
distance body <-> 2-sigma ellipsoid, and the chassis rides 0.02 m above the floor, so floor splats cap it below
0.02 m almost everywhere (F2 region survey: oracle clearance >= 0.02 m on 0 % of every candidate box).  A route
"with >= 5 cm clearance" in the oracle's own sense therefore does not exist anywhere; what can be required is
5 cm LATERAL / overhead clearance plus a vertical margin above the shared 1 mm.  So the confirmation runs the
same oracle and A* on a ROBUST body: the cylinder grown by ``--lat`` (0.05 m) in radius and at the top, chassis
bottom unchanged (still 0.02 m over the floor), at margin ``--vmargin`` (0.003 m = 3x the shared margin, 1 mm
above aerial3d's margin + buffer).  The robust body contains the real cylinder, so a route the robust body
passes at margin X keeps the real cylinder >= lat + X from every Gaussian sideways / above and > X from every
Gaussian below; its world AABB also contains the real one, so the route stays inside the real body's domain
(no CY-GAP-style box artefact).

A draw is (start, goal), both uniform over the route box (G2's rule: continuous, 0.1 mm rounding).  It is
rejected whole at the first failing test (cheap tests first), so the accepted pairs are i.i.d. from the
conditional law and any prefix of a stream is itself an unbiased sample:

  0. straight-line xy distance >= 3 m                                               (G2's convention)
  1. both endpoints oracle-free for the real cylinder at its own z_c, margin 0.001  (G2's endpoint test)
  2. both endpoints oracle-free for the robust body at margin --vmargin
  3. cheap pre-filter: both endpoints attach (one robust-body oracle edge) to a node of ONE connected component
     of a shared 0.1 m route-frame lattice of the robust body, built once per region (``lattice``)
  4. the gs3d lattice A* (``LatticePlanner``; F1/ground5k settings: 0.1 m lattice anchored at the start,
     position tolerance 0, yaw tolerance 0.05, 500k expansions) on the robust body at margin --vmargin
     returns ROUTE (its own post-build verification -- every closed edge + kinematics -- passed)
  5. that route, shifted onto the real cylinder's z_c, re-verified edge by edge with the real cylinder at the
     shared margin 0.001 (``verify_path`` with the goal) passes

Step 3 can only reject (never accept); its false-rejection rate is measured in the pilot (``--audit`` runs
step 4 on pre-filter rejects too).

  lattice  build the shared robust-body lattice for a box once -> npz (nodes, components) + json (cost)
  run      one stream: draw / test / append one JSON line per distance-passing candidate (fsync'd, resumable)
  collect  merge stream files -> pairs json in G2's schema (+ the A* evidence per pair) and per-stage stats
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
import os
from pathlib import Path
import time

import numpy as np

from aerial3dg_run import GROUND_CONFIG, ROBOTS, _box, _dump, ground_world, host, load_booth, rss_mb
from aerial3dg_fail_oracle import BUDGET, classify

MIN_DIST = 3.0
STEP = .1
NBR = ((1, 0), (0, 1), (1, 1), (1, -1))


def robust_body(lat):
    c = ROBOTS["cylinder"]
    return replace(c, name=f"cylinder_robust_{lat:g}", radius_m=c.radius_m + lat, half_height_m=c.half_height_m + lat / 2)


def draws(seed, box_uv):
    """The seeded draw sequence (index, start, goal, dist), exactly G2's ``sample_pairs`` draw rule."""
    rng = np.random.default_rng(seed)
    lo, hi = np.asarray(box_uv[:2], float), np.asarray(box_uv[2:], float)
    k = 0
    while True:
        s = np.clip(rng.uniform(lo, hi).round(4), lo, hi)
        g = np.clip(rng.uniform(lo, hi).round(4), lo, hi)
        yield k, s, g, float(np.hypot(*(g - s)))
        k += 1


def read_jsonl(path: Path) -> list:
    if not path.exists():
        return []
    rows, good = [], 0
    raw = path.read_bytes()
    for line in raw.splitlines(keepends=True):
        try:
            if not line.endswith(b"\n"):
                raise ValueError("unterminated")
            rows.append(json.loads(line))
        except ValueError:
            break
        good += len(line)
    if good != len(raw):
        with open(path, "r+b") as f:
            f.truncate(good)
    return rows


class Tester:
    def __init__(self, box_uv, lat, vmargin, max_wall, resolution):
        from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
        from gmc.gs3d.planner import LatticePlanner
        t0 = time.perf_counter()
        self.box = tuple(box_uv)
        self.ctx = load_booth(_box(box_uv))
        self.load_s = time.perf_counter() - t0
        t0 = time.perf_counter()
        self.prepared = PreparedScene(self.ctx["scene"])
        self.prepare_s = time.perf_counter() - t0
        self.oracle = GaussianBodyOracle(self.prepared)
        self.planner = LatticePlanner(self.prepared)
        self.body, self.robust = ROBOTS["cylinder"], robust_body(lat)
        self.lat, self.vmargin, self.max_wall, self.resolution = lat, vmargin, max_wall, resolution
        self.dz = self.robust.half_height_m - self.body.half_height_m
        self.lattice = None

    def pose(self, uv, body):
        from gmc.gs3d.contracts import Pose3
        return Pose3(tuple(map(float, ground_world(self.ctx["frame"], body, uv))), 0.)

    def free(self, q, body, margin):
        rep = self.oracle.pose(q, body, margin_m=margin)
        ok = rep.occupancy == "free" and rep.safety == "continuous_bound"
        return ok, (float(rep.clearance_lower_m) if ok else None), rep.reason

    def edge_ok(self, a, b):
        rep = self.oracle.edge(a, b, self.robust, margin_m=self.vmargin)
        return rep.occupancy == "free" and rep.safety == "continuous_bound"

    # ------------------------------------------------------------------ shared lattice (pre-filter)
    def grid(self):
        u0, v0, u1, v1 = self.box
        us = np.round(np.arange(np.ceil(u0 / STEP) * STEP, u1 + 1e-9, STEP), 6)
        vs = np.round(np.arange(np.ceil(v0 / STEP) * STEP, v1 + 1e-9, STEP), 6)
        return us, vs

    def build_lattice(self):
        t0, c0 = time.perf_counter(), self.oracle.stats["oracle_calls"]
        us, vs = self.grid()
        nu, nv = len(us), len(vs)
        free = np.zeros((nu, nv), bool)
        poses = {}
        for i, u in enumerate(us):
            for j, v in enumerate(vs):
                q = self.pose((u, v), self.robust)
                if self.free(q, self.robust, self.vmargin)[0]:
                    free[i, j] = True
                    poses[i, j] = q
        parent = np.arange(nu * nv)

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x
        n_edges = n_ok = 0
        for (i, j), q in poses.items():
            for di, dj in NBR:
                k = (i + di, j + dj)
                if k in poses:
                    n_edges += 1
                    if self.edge_ok(q, poses[k]):
                        n_ok += 1
                        a, b = find(i * nv + j), find(k[0] * nv + k[1])
                        if a != b:
                            parent[a] = b
        comp = np.full((nu, nv), -1, np.int64)
        for (i, j) in poses:
            comp[i, j] = find(i * nv + j)
        labels, sizes = np.unique(comp[comp >= 0], return_counts=True)
        remap = {int(l): r for r, l in enumerate(labels[np.argsort(-sizes)])}
        comp = np.vectorize(lambda x: remap.get(int(x), -1))(comp)
        self.lattice = {"us": us, "vs": vs, "free": free, "comp": comp}
        meta = {"step_m": STEP, "nodes": int(nu * nv), "free_nodes": int(free.sum()), "edges_tested": n_edges,
                "edges_free": n_ok, "components": int(len(labels)), "component_sizes": sorted(sizes.tolist())[::-1][:20],
                "wall_s": time.perf_counter() - t0, "oracle_calls": self.oracle.stats["oracle_calls"] - c0}
        return meta

    def load_lattice(self, path):
        with np.load(path) as d:
            self.lattice = {k: d[k] for k in ("us", "vs", "free", "comp")}

    def attach(self, uv):
        """Components of the lattice nodes within 1.5 steps that a single robust-body oracle edge reaches."""
        L = self.lattice
        q = self.pose(uv, self.robust)
        i0 = int(np.floor((uv[0] - L["us"][0]) / STEP))
        j0 = int(np.floor((uv[1] - L["vs"][0]) / STEP))
        cand = []
        for i in range(i0 - 1, i0 + 3):
            for j in range(j0 - 1, j0 + 3):
                if 0 <= i < len(L["us"]) and 0 <= j < len(L["vs"]) and L["comp"][i, j] >= 0:
                    d = np.hypot(L["us"][i] - uv[0], L["vs"][j] - uv[1])
                    cand.append((d, i, j))
        comps = set()
        for d, i, j in sorted(cand):
            c = int(L["comp"][i, j])
            if c in comps:
                continue
            if self.edge_ok(q, self.pose((L["us"][i], L["vs"][j]), self.robust)):
                comps.add(c)
        return comps

    # ------------------------------------------------------------------ the per-candidate test
    def test(self, s_uv, g_uv, audit=False) -> dict:
        from gmc.gs3d.contracts import GoalRegion, PlannerConfig, Pose3, SearchBudget
        from gmc.gs3d.validation import verify_path
        row = {"stage_failed": None}
        t0 = time.perf_counter()
        for tag, body, margin in (("m001", self.body, GROUND_CONFIG.margin_m), ("robust", self.robust, self.vmargin)):
            for e, uv in (("start", s_uv), ("goal", g_uv)):
                ok, c, why = self.free(self.pose(uv, body), body, margin)
                row[f"{e}_free_{tag}"], row[f"{e}_clear_{tag}"] = ok, c
                if not ok:
                    row.update(stage_failed=f"endpoint_{tag}", endpoint_reason=why,
                               t_endpoint_s=time.perf_counter() - t0)
                    return row
        row["t_endpoint_s"] = time.perf_counter() - t0
        t0 = time.perf_counter()
        cs, cg = self.attach(s_uv), self.attach(g_uv)
        row.update(prefilter_start_comps=sorted(cs), prefilter_goal_comps=sorted(cg),
                   prefilter_passed=bool(cs & cg), t_prefilter_s=time.perf_counter() - t0)
        if not row["prefilter_passed"]:
            row["stage_failed"] = "prefilter"
            if not audit:
                return row
        s, g = self.pose(s_uv, self.robust), self.pose(g_uv, self.robust)
        conf = PlannerConfig(resolution_m=self.resolution, margin_m=self.vmargin, seed=0,
                             budget=SearchBudget(max_wall_s=self.max_wall, **BUDGET))
        t0 = time.perf_counter()
        res = self.planner.plan(self.ctx["scene"], self.robust, s, GoalRegion(g, 0., .05), conf)
        d = res["diagnostics"]
        row.update(astar_outcome=classify(res), astar_status=res["status"], astar_reason=res["reason"],
                   t_astar_s=time.perf_counter() - t0, astar_clearance_lower_m=res["clearance_lower_m"],
                   path_length_m=d.get("path_length_m"),
                   **{f"astar_{k}": d.get(k) for k in ("expansions", "oracle_calls", "map_unknown_rejections",
                                                       "unproven_rejections", "occupied_rejections")})
        if row["stage_failed"] == "prefilter":         # audit only: never accept a pre-filter reject
            return row
        if row["astar_outcome"] != "ROUTE":
            row["stage_failed"] = "astar_robust"
            return row
        t0 = time.perf_counter()
        poses = [Pose3((p[0], p[1], p[2] - self.dz), p[3]) for p in res["trajectory"]["poses"]]
        goal = GoalRegion(self.pose(g_uv, self.body), 0., .05)
        ver = verify_path(self.oracle, poses, self.body, margin_m=GROUND_CONFIG.margin_m, goal=goal)
        row.update(reverify_m001_passed=ver["passed"], reverify_m001_reason=ver["reason"],
                   reverify_m001_clearance_lower_m=ver.get("clearance_lower_m"), t_reverify_s=time.perf_counter() - t0,
                   route_uv=self.ctx["frame"].to_route(np.asarray([p.xyz for p in poses]))[:, :2].round(4).tolist())
        if not ver["passed"]:
            row["stage_failed"] = "reverify_m001"
        return row


def _tester(a):
    return Tester(a.box, a.lat, a.vmargin, a.max_wall, a.resolution)


def cmd_lattice(a):
    t = _tester(a)
    meta = t.build_lattice()
    a.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(a.out, **t.lattice)
    meta.update(box_uv=a.box, lat_m=a.lat, vmargin_m=a.vmargin, robust_body=t.robust.__dict__,
                crop=t.ctx["crop"], load_s=t.load_s, prepare_s=t.prepare_s, host=host(), peak_rss_mb=rss_mb())
    _dump(a.out.with_suffix(".json"), meta)
    print(json.dumps({k: meta[k] for k in ("free_nodes", "nodes", "edges_free", "components", "component_sizes",
                                           "wall_s")}), flush=True)


def cmd_run(a):
    out = Path(a.out_dir) / f"stream_{a.stream:02d}.jsonl"
    out.parent.mkdir(parents=True, exist_ok=True)
    done = read_jsonl(out)
    seen = {r["draw"] for r in done}
    accepted = sum(r["accepted"] for r in done)
    n_cand = len(done)
    t0 = time.perf_counter()
    tester = _tester(a)
    if a.lattice and Path(a.lattice).exists():
        tester.load_lattice(a.lattice)
        lat_meta = {"path": str(a.lattice), "built_here": False}
    else:
        lat_meta = {**tester.build_lattice(), "built_here": True}
    seed = a.seed + 1000 * a.stream
    print(f"stream {a.stream} seed {seed}: resumed {n_cand} candidates / {accepted} accepted; "
          f"prepare {tester.prepare_s:.1f} s; crop {tester.ctx['crop']['selected_supports']}; lattice {lat_meta}",
          flush=True)
    meta = {"schema": "aerial3dg_fail2.sample_stream.v2", "stream": a.stream, "seed": seed, "box_uv": a.box,
            "lat_m": a.lat, "vmargin_m": a.vmargin, "robust_body": tester.robust.__dict__,
            "resolution_m": a.resolution, "astar_budget": {**BUDGET, "max_wall_s": a.max_wall}, "audit": a.audit,
            "min_dist_m": MIN_DIST, "crop": tester.ctx["crop"], "archive_sha256": tester.ctx["digest"],
            "load_s": tester.load_s, "prepare_s": tester.prepare_s, "lattice": lat_meta, "host": host()}
    _dump(out.with_suffix(".meta.json"), meta)
    n_draws = 0
    with open(out, "a") as f:
        for k, s, g, d in draws(seed, a.box):
            if accepted >= a.quota or n_cand >= a.max_candidates:
                break
            if time.perf_counter() - t0 > a.max_hours * 3600:
                print("wall-clock guard reached; resubmit to resume", flush=True)
                break
            n_draws = k + 1
            if d < MIN_DIST or k in seen:
                continue
            w0, c0 = time.perf_counter(), time.process_time()
            row = {"draw": k, "start_uv": s.tolist(), "goal_uv": g.tolist(), "dist_m": d}
            row.update(tester.test(s, g, audit=a.audit))
            row["accepted"] = row["stage_failed"] is None
            row["wall_s"], row["cpu_s"] = time.perf_counter() - w0, time.process_time() - c0
            f.write(json.dumps(row, default=float) + "\n")
            f.flush()
            os.fsync(f.fileno())
            n_cand += 1
            accepted += row["accepted"]
            if n_cand % 25 == 0:
                print(f"cand {n_cand} accepted {accepted} last {row['stage_failed']} {row['wall_s']:.1f}s "
                      f"rss {rss_mb():.0f} MB", flush=True)
    meta["draws_consumed"] = n_draws
    _dump(out.with_suffix(".meta.json"), meta)
    print(f"stream {a.stream} done: {n_cand} candidates, {accepted} accepted, {time.perf_counter() - t0:.0f} s, "
          f"peak rss {rss_mb():.0f} MB", flush=True)


def _sum(rows, key):
    x = np.array([r[key] for r in rows if r.get(key) is not None], float)
    if not len(x):
        return None
    return {"n": int(len(x)), "sum": float(x.sum()), "mean": float(x.mean()), "median": float(np.median(x)),
            "p95": float(np.percentile(x, 95)), "max": float(x.max())}


def stage_stats(rows) -> dict:
    """Pass counts and cost of each stage, on the candidates that reached it."""
    order = ["endpoint_m001", "endpoint_robust", "prefilter", "astar_robust", "reverify_m001"]
    out, alive = {}, len(rows)
    for st in order:
        failed = sum(r["stage_failed"] == st for r in rows)
        out[st] = {"entered": alive, "failed": failed, "pass_rate": (alive - failed) / max(alive, 1)}
        alive -= failed
    ok = [r for r in rows if r.get("prefilter_passed")]
    rej = [r for r in rows if r.get("prefilter_passed") is False]
    out["cost_s"] = {"endpoint": _sum(rows, "t_endpoint_s"), "prefilter": _sum(rows, "t_prefilter_s"),
                     "astar_on_prefilter_pass": _sum(ok, "t_astar_s"),
                     "astar_on_prefilter_reject_audit": _sum(rej, "t_astar_s"),
                     "reverify": _sum(rows, "t_reverify_s"), "candidate_wall": _sum(rows, "wall_s")}
    out["astar_outcomes_on_prefilter_pass"] = {o: sum(r.get("astar_outcome") == o for r in ok)
                                               for o in ("ROUTE", "NO_ROUTE", "NO_ROUTE_MARGIN", "UNSURE")}
    out["astar_outcomes_on_prefilter_reject_audit"] = {o: sum(r.get("astar_outcome") == o for r in rej)
                                                       for o in ("ROUTE", "NO_ROUTE", "NO_ROUTE_MARGIN", "UNSURE")}
    out["accepted"] = sum(r["accepted"] for r in rows)
    return out


def cmd_collect(a):
    rows, metas = [], []
    for p in sorted(Path(a.out_dir).glob("stream_*.jsonl")):
        metas.append(json.loads(p.with_suffix(".meta.json").read_text()))
        st = metas[-1]["stream"]
        rows += [{**r, "stream": st} for r in read_jsonl(p)]
    acc = sorted([r for r in rows if r["accepted"]], key=lambda r: (r["stream"], r["draw"]))
    if a.n is not None:
        acc = acc[:a.n]                                # each stream prefix is an unbiased sample
    pairs = []
    for i, r in enumerate(acc):
        pairs.append({"index": i, "pair_id": f"F2-{i:05d}", "start_uv": r["start_uv"], "goal_uv": r["goal_uv"],
                      "dist_m": r["dist_m"], "stream": r["stream"], "draw": r["draw"],
                      "clearance_m": {"cylinder": {"start": r["start_clear_m001"], "goal": r["goal_clear_m001"]},
                                      "robust": {"start": r["start_clear_robust"], "goal": r["goal_clear_robust"]}},
                      "astar": {k: r.get(k) for k in ("astar_outcome", "astar_clearance_lower_m", "path_length_m",
                                                      "reverify_m001_passed", "reverify_m001_clearance_lower_m",
                                                      "astar_expansions", "t_astar_s")},
                      "astar_route_uv": r["route_uv"]})
    m0 = metas[0]
    d = np.array([p["dist_m"] for p in pairs]) if pairs else np.zeros(1)
    doc = {"schema": "aerial3dg.pairs.v1+f2_confirmed", "n": len(pairs), "box_uv": m0["box_uv"],
           "box_route": _box(m0["box_uv"]), "robots": ["cylinder", "sweeper"], "min_dist_m": MIN_DIST,
           "margin_m": GROUND_CONFIG.margin_m, "lat_m": m0["lat_m"], "vmargin_m": m0["vmargin_m"],
           "robust_body": m0["robust_body"], "rule": __doc__.strip(),
           "streams": [{k: m.get(k) for k in ("stream", "seed", "draws_consumed", "audit")} for m in metas],
           "stages": stage_stats(rows), "distance_passing_candidates": len(rows),
           "dist_m": {"min": float(d.min()), "median": float(np.median(d)), "max": float(d.max())},
           "crop": m0["crop"], "archive_sha256": m0["archive_sha256"], "pairs": pairs}
    _dump(Path(a.out), doc)
    print(json.dumps({k: doc[k] for k in ("n", "distance_passing_candidates", "stages", "dist_m")}, default=float),
          flush=True)


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    la = sub.add_parser("lattice")
    r = sub.add_parser("run")
    for q in (la, r):
        q.add_argument("--box", type=float, nargs=4, required=True, metavar=("U0", "V0", "U1", "V1"))
        q.add_argument("--lat", type=float, default=.05)
        q.add_argument("--vmargin", type=float, default=.003)
        q.add_argument("--resolution", type=float, default=.1)
        q.add_argument("--max-wall", type=float, default=120.)
    la.add_argument("--out", type=Path, required=True)
    r.add_argument("--lattice", type=Path, default=None, help="reuse this lattice npz (built in-process if absent)")
    r.add_argument("--stream", type=int, default=0)
    r.add_argument("--seed", type=int, default=20261002)
    r.add_argument("--quota", type=int, default=5000, help="stop after this many accepted pairs in this stream")
    r.add_argument("--max-candidates", type=int, default=10 ** 9, help="stop after this many distance-passing draws")
    r.add_argument("--audit", action="store_true", help="also run the A* on pre-filter rejects (pilot: false-rejection rate)")
    r.add_argument("--max-hours", type=float, default=7.5)
    r.add_argument("--out-dir", type=Path, required=True)
    c = sub.add_parser("collect")
    c.add_argument("--out-dir", type=Path, required=True)
    c.add_argument("--n", type=int, default=None)
    c.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)
    {"lattice": cmd_lattice, "run": cmd_run, "collect": cmd_collect}[a.cmd](a)


if __name__ == "__main__":
    main()
