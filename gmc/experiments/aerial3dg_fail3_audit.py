"""F3 Task 0: an independent audit of every G2 non-REACHABLE row (cylinder 4927, sweeper 301).

Run from ``gmc/`` with ``PYTHONPATH=src:experiments`` (sbatch for everything but ``rows`` / ``verdicts``).

Question (user): does G2 check with A* (or any other algorithm) that the cylinder really is unreachable on so
many pairs?  It does not (``g2/collect.json`` holds GMC's own verdict counts only); F1 checked a sample of 30
rows per class, and the outcome there is mostly NO_ROUTE_MARGIN, which proves little.  This script checks every
row with algorithms other than GMC:

  rows      the row list: all G2 non-REACHABLE rows with their F1 class + a seeded, distance-stratified
            control of G2-REACHABLE rows per robot  -> configs/aerial3dg/f3_audit_rows.json
  astar     per-row gs3d lattice A* (``LatticePlanner``, F1 oracle settings: position tolerance 0, yaw
            tolerance 0.05, 500k expansions) for one body variant / margin / lattice step / box, one shard of
            the rows (array task), resumable jsonl.  Also records which Gaussians the "unproven" (within-margin)
            rejections came from and their 2-sigma tops, so NO_ROUTE_MARGIN can be read.
  grid      exhaustive alternative to per-row A* for the expensive negatives: label the connected components
            of a fixed route-frame lattice (8-connected, every node a free pose, every edge one oracle edge
            test -- exactly the A*'s graph, but anchored on a global grid instead of at the start), then attach
            every row's start / goal to it with one oracle edge each.  Same start + goal in one component =
            a constructive route (lattice path of verified edges, re-verified with ``verify_path`` on a sample);
            different components = the A* would exhaust (modulo the anchoring).  One build per (body, margin,
            step, box) answers all rows.
  lampwidth independent, non-search geometry of the lamp crossing: 1 cm route-frame occupancy of the 2-sigma
            cross-sections of every Gaussian in the cylinder's body band [0.02, 1.75] m (sampled z planes, an
            UNDER-estimate of the obstacle), its distance transform, and the max-min clearance of any
            west -> east crossing of the lamp line.  Twice that clearance is the widest disk that can cross.
  verdicts  merge everything into the per-row verdict table (inline, json only).

Body variants (strict subsets of the real body, chosen from the geometry, see ``variant``):
  real   the G2 body (sweeper r 0.175 hh 0.04 / cylinder r 0.30 hh 0.865, chassis 0.02 m over the floor)
  s2     radius -2 mm, bottom +2 mm, top -2 mm  (half-height -2 mm, chassis clearance +2 mm, z_c unchanged)
  s10    radius -10 mm, bottom +3 mm, top -10 mm (half-height -6.5 mm, chassis clearance +3 mm, z_c -3.5 mm)
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
import os
from pathlib import Path
import time

import numpy as np

from aerial3dg_run import ROBOTS, _box, ground_world, host, load_booth, rss_mb
from aerial3dg_fail_oracle import BUDGET, G2_BOX, classify
from aerial3dg_fail2_sample import read_jsonl
from aerial3dg_fail_diag import _dump

W1_BOX = (-10.0, -1.35, 4.7, 3.75)
BOXES = {"G2": G2_BOX, "W1": W1_BOX}
NBR8 = ((1, 0), (0, 1), (1, 1), (1, -1))


def variant(robot: str, name: str):
    """Strict-subset bodies.  ``dr`` shrinks the radius, ``db`` raises the chassis bottom, ``dt`` lowers the top.
    Ground pose z_c = clearance + half_height, so (db, dt) -> clearance + db, half_height - (db + dt) / 2."""
    b = ROBOTS[robot]
    if name == "real":
        return b
    dr, db, dt = {"s2": (.002, .002, .002), "s10": (.010, .003, .010)}[name]
    return replace(b, name=f"{b.name}_{name}", radius_m=b.radius_m - dr,
                   half_height_m=b.half_height_m - (db + dt) / 2, ground_clearance_m=b.ground_clearance_m + db)


def _box_arg(s):
    if s in BOXES:
        return BOXES[s]
    return tuple(float(x) for x in s.split(","))


# ----------------------------------------------------------------------------- rows
def _g2_rows(robot):
    rows = {}
    for p in sorted(Path(f"results/aerial3dg/g2/runs/{robot}").glob("task_*.jsonl")):
        for line in p.read_text().splitlines():
            r = json.loads(line)
            rows[r["index"]] = r
    return rows


def cmd_rows(a):
    import csv
    cls = {(r["robot"], int(r["index"])): r["class"] for r in csv.DictReader(open("results/aerial3dg/f1/classes.csv"))}
    pairs = json.loads(Path("results/aerial3dg/g2/pairs_5000.json").read_text())["pairs"]
    rng = np.random.default_rng(a.seed)
    out = []
    for robot in ("cylinder", "sweeper"):
        g2 = _g2_rows(robot)
        assert len(g2) == 5000, (robot, len(g2))
        reach = []
        for i, r in sorted(g2.items()):
            c = cls.get((robot, i))
            if r["status"] == "REACHABLE":
                assert c is None
                reach.append(i)
                continue
            assert c is not None, (robot, i)
            out.append({"robot": robot, "index": i, "class": c, "g2_status": r["status"], "g2_reason": r["reason"]})
        reach = np.asarray(reach)
        d = np.array([pairs[i]["dist_m"] for i in reach])
        edges = np.quantile(d, [0, 1 / 3, 2 / 3, 1])
        bins = np.clip(np.searchsorted(edges, d, side="right") - 1, 0, 2)
        n = min(a.n_ctrl, len(reach))
        pick = reach.tolist() if n == len(reach) else []      # cylinder: only 73 G2-REACHABLE rows exist -> all
        for b in range(3 if n < len(reach) else 0):
            pool = reach[bins == b]
            k = n // 3 + (1 if b < n % 3 else 0)
            pick += rng.choice(pool, size=min(k, len(pool)), replace=False).tolist()
        for i in sorted(pick):
            out.append({"robot": robot, "index": int(i), "class": "CTRL-REACHABLE", "g2_status": "REACHABLE",
                        "g2_reason": g2[i]["reason"]})
    for r in out:
        p = pairs[r["index"]]
        r.update(pair_id=p["pair_id"], start_uv=p["start_uv"], goal_uv=p["goal_uv"], dist_m=p["dist_m"])
    doc = {"seed": a.seed, "n_ctrl_per_robot": a.n_ctrl,
           "rule": "every G2 non-REACHABLE row (F1 class from classes.csv) + per robot n_ctrl G2-REACHABLE rows, "
                   "stratified over straight-distance terciles (seeded)", "rows": out}
    Path(a.out).write_text(json.dumps(doc, indent=0) + "\n")
    import collections
    print(collections.Counter((r["robot"], r["class"]) for r in out))


def _select(rows_path, robot, classes, shard, nshards, stride=1, offset=0):
    rows = [r for r in json.loads(Path(rows_path).read_text())["rows"] if r["robot"] == robot
            and (not classes or r["class"] in classes)]
    rows = rows[offset::stride]
    return rows[shard::nshards]


# ----------------------------------------------------------------------------- per-row A*
class _Ctx:
    def __init__(self, box):
        from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
        t0 = time.perf_counter()
        self.box = tuple(box)
        self.ctx = load_booth(_box(box))
        self.frame, self.scene = self.ctx["frame"], self.ctx["scene"]
        self.load_s = time.perf_counter() - t0
        t0 = time.perf_counter()
        self.prepared = PreparedScene(self.scene)
        self.prepare_s = time.perf_counter() - t0
        self.oracle = GaussianBodyOracle(self.prepared)
        g = self.scene.gaussians
        z_floor = float(self.ctx["support_evidence"]["z_floor_world_m"])
        sz = np.sqrt(np.maximum(g.covs[:, 2, 2], 0.))
        self.top_by_id = dict(zip(g.ids.tolist(), (g.means[:, 2] + self.scene.level * sz - z_floor).tolist()))
        self.uv_by_id = None

    def pose(self, uv, body):
        from gmc.gs3d.contracts import Pose3
        return Pose3(tuple(map(float, ground_world(self.frame, body, uv))), 0.)

    def free(self, uv, body, margin):
        rep = self.oracle.pose(self.pose(uv, body), body, margin_m=margin)
        ok = rep.occupancy == "free" and rep.safety == "continuous_bound"
        return ok, rep

    def edge_rep(self, qa, qb, body, margin):
        rep = self.oracle.edge(qa, qb, body, margin_m=margin)
        return rep.occupancy == "free" and rep.safety == "continuous_bound", rep


def _recording_planner(prepared):
    """LatticePlanner whose oracle records the Gaussian ids behind every 'unproven' (within-margin) rejection."""
    import gmc.gs3d.planner as pl
    base = pl.GaussianBodyOracle
    seen = {}

    class Rec(base):
        def edge(self, a, b, body, *, margin_m):
            rep = super().edge(a, b, body, margin_m=margin_m)
            if rep.occupancy != "free" and rep.reason == "geometry_or_margin_unproven":
                for i in rep.primitive_ids:
                    seen[int(i)] = seen.get(int(i), 0) + 1
            return rep
    pl.GaussianBodyOracle = Rec
    return pl.LatticePlanner(prepared), seen


def cmd_astar(a):
    from gmc.gs3d.contracts import GoalRegion, PlannerConfig, SearchBudget
    body = variant(a.robot, a.body)
    rows = _select(a.rows, a.robot, a.classes, a.shard, a.nshards, a.stride, a.offset)
    out = Path(a.out_dir) / f"{a.robot}_{a.body}_m{a.margin:g}_r{a.resolution:g}_{a.box_name}{a.tag}_{a.shard:03d}.jsonl"
    out.parent.mkdir(parents=True, exist_ok=True)
    done = {(r["index"]) for r in read_jsonl(out)}
    t0 = time.perf_counter()
    C = _Ctx(_box_arg(a.box_name))
    planner, seen = _recording_planner(C.prepared)
    meta = {"robot": a.robot, "body": asdict(body), "variant": a.body, "margin_m": a.margin,
            "resolution_m": a.resolution, "box_name": a.box_name, "box_uv": C.box, "shard": a.shard,
            "nshards": a.nshards, "classes": a.classes, "budget": {**BUDGET, "max_wall_s": a.max_wall},
            "crop": C.ctx["crop"], "load_s": C.load_s, "prepare_s": C.prepare_s, "host": host(),
            "n_rows": len(rows)}
    _dump(out.with_suffix(".meta.json"), meta)
    print(f"{len(rows)} rows, {len(done)} done; load {C.load_s:.0f}s prepare {C.prepare_s:.1f}s "
          f"crop {C.ctx['crop']['selected_supports']}", flush=True)
    with open(out, "a") as f:
        for r in rows:
            if r["index"] in done:
                continue
            if time.perf_counter() - t0 > a.max_hours * 3600:
                print("wall-clock guard; resubmit to resume", flush=True)
                break
            seen.clear()
            s, g = C.pose(r["start_uv"], body), C.pose(r["goal_uv"], body)
            conf = PlannerConfig(resolution_m=a.resolution, margin_m=a.margin, seed=0,
                                 budget=SearchBudget(max_wall_s=a.max_wall, **BUDGET))
            w0, c0 = time.perf_counter(), time.process_time()
            res = planner.plan(C.scene, body, s, GoalRegion(g, 0., .05), conf)
            d = res["diagnostics"]
            tops = sorted(((C.top_by_id.get(i, float("nan")), i, n) for i, n in seen.items()))
            row = {"robot": a.robot, "index": r["index"], "pair_id": r["pair_id"], "class": r["class"],
                   "outcome": classify(res), "status": res["status"], "reason": res["reason"],
                   "wall_s": time.perf_counter() - w0, "cpu_s": time.process_time() - c0,
                   "path_length_m": d.get("path_length_m"), "clearance_lower_m": res["clearance_lower_m"],
                   **{k: d.get(k) for k in ("expansions", "visited_nodes", "oracle_calls", "map_unknown_rejections",
                                            "unproven_rejections", "occupied_rejections")},
                   "unproven_gaussians": len(seen),
                   "unproven_top_z_range": [tops[0][0], tops[-1][0]] if tops else None,
                   "unproven_under_chassis": sum(t < body.ground_clearance_m for t, _, _ in tops),
                   "unproven_ids_top": [[i, round(t, 6), n] for t, i, n in sorted(tops, key=lambda x: -x[2])[:12]]}
            if res["status"] == "success":
                row["route_uv"] = C.frame.to_route(np.asarray(res["trajectory"]["poses"], float)[:, :3])[:, :2] \
                    .round(4).tolist()
            f.write(json.dumps(row, default=float) + "\n")
            f.flush()
            os.fsync(f.fileno())
            print(r["class"], r["pair_id"], row["outcome"], row["reason"], f"{row['wall_s']:.1f}s",
                  row["expansions"], row["unproven_rejections"], row["unproven_gaussians"], flush=True)
    meta.update(wall_s=time.perf_counter() - t0, peak_rss_mb=rss_mb())
    _dump(out.with_suffix(".meta.json"), meta)
    print(f"done {time.perf_counter() - t0:.0f}s peak rss {rss_mb():.0f} MB", flush=True)


# ----------------------------------------------------------------------------- global lattice labelling
def cmd_grid(a):
    from gmc.gs3d.contracts import GoalRegion
    from gmc.gs3d.validation import verify_path
    body = variant(a.robot, a.body)
    t0 = time.perf_counter()
    C = _Ctx(_box_arg(a.box_name))
    u0, v0, u1, v1 = C.box
    st = a.step
    us = np.round(np.arange(np.ceil(u0 / st) * st, u1 + 1e-9, st), 6)
    vs = np.round(np.arange(np.ceil(v0 / st) * st, v1 + 1e-9, st), 6)
    nu, nv = len(us), len(vs)
    reasons = {}
    poses = {}
    for i, u in enumerate(us):
        for j, v in enumerate(vs):
            ok, rep = C.free((u, v), body, a.margin)
            if ok:
                poses[i, j] = C.pose((u, v), body)
            else:
                k = rep.occupancy if rep.occupancy == "occupied" else rep.reason
                reasons[k] = reasons.get(k, 0) + 1
    t_nodes = time.perf_counter() - t0
    parent = np.arange(nu * nv)

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    adj = {}
    erej = {}
    n_edges = 0
    for (i, j), q in poses.items():
        for di, dj in NBR8:
            k = (i + di, j + dj)
            if k in poses:
                n_edges += 1
                ok, rep = C.edge_rep(q, poses[k], body, a.margin)
                if ok:
                    adj.setdefault((i, j), []).append(k)
                    adj.setdefault(k, []).append((i, j))
                    ra, rb = find(i * nv + j), find(k[0] * nv + k[1])
                    if ra != rb:
                        parent[ra] = rb
                else:
                    kk = rep.occupancy if rep.occupancy == "occupied" else rep.reason
                    erej[kk] = erej.get(kk, 0) + 1
    comp = np.full((nu, nv), -1, np.int64)
    for (i, j) in poses:
        comp[i, j] = find(i * nv + j)
    labels, sizes = np.unique(comp[comp >= 0], return_counts=True)
    remap = {int(l): r for r, l in enumerate(labels[np.argsort(-sizes)])}
    comp = np.vectorize(lambda x: remap.get(int(x), -1))(comp) if len(labels) else comp
    t_build = time.perf_counter() - t0
    print(f"grid {a.robot}/{a.body} m{a.margin} step {st} {a.box_name}: {len(poses)}/{nu * nv} free nodes, "
          f"{n_edges} edges, {sum(len(v) for v in adj.values()) // 2} free, {len(labels)} comps "
          f"(top {sorted(sizes.tolist())[::-1][:6]}), {t_build:.0f}s; node rej {reasons}; edge rej {erej}", flush=True)

    def attach(uv):
        q = C.pose(uv, body)
        i0, j0 = int(np.floor((uv[0] - us[0]) / st)), int(np.floor((uv[1] - vs[0]) / st))
        cand = sorted((float(np.hypot(us[i] - uv[0], vs[j] - uv[1])), i, j)
                      for i in range(i0 - 1, i0 + 3) for j in range(j0 - 1, j0 + 3)
                      if 0 <= i < nu and 0 <= j < nv and comp[i, j] >= 0)
        got = {}
        for _, i, j in cand:
            c = int(comp[i, j])
            if c not in got and C.edge_rep(q, poses[i, j], body, a.margin)[0]:
                got[c] = (i, j)
        return got

    def bfs(src, dst):
        from collections import deque
        prev, dq = {src: None}, deque([src])
        while dq:
            x = dq.popleft()
            if x == dst:
                break
            for y in adj.get(x, ()):
                if y not in prev:
                    prev[y] = x
                    dq.append(y)
        path, x = [], dst
        while x is not None:
            path.append(x)
            x = prev[x]
        return path[::-1]

    rows = json.loads(Path(a.rows).read_text())["rows"]
    rows = [r for r in rows if r["robot"] == a.robot and (not a.classes or r["class"] in a.classes)]
    res_rows, n_ver = [], 0
    for r in rows:
        w0 = time.perf_counter()
        row = {"index": r["index"], "pair_id": r["pair_id"], "class": r["class"]}
        oks, rs = C.free(r["start_uv"], body, a.margin)
        okg, rg = C.free(r["goal_uv"], body, a.margin)
        row.update(start_free=oks, goal_free=okg,
                   start_reason=None if oks else rs.reason, goal_reason=None if okg else rg.reason)
        if not (oks and okg):
            row["outcome"] = "ENDPOINT_NOT_FREE"
        else:
            cs, cg = attach(r["start_uv"]), attach(r["goal_uv"])
            row.update(start_comps=sorted(cs), goal_comps=sorted(cg))
            common = sorted(set(cs) & set(cg))
            if not cs or not cg:
                row["outcome"] = "NO_ATTACH"
            elif not common:
                row["outcome"] = "SEPARATED"
            else:
                row["outcome"] = "CONNECTED"
                c = common[0]
                path = bfs(cs[c], cg[c])
                pts = [r["start_uv"]] + [(float(us[i]), float(vs[j])) for i, j in path] + [r["goal_uv"]]
                row["lattice_path_len_m"] = float(np.sum(np.hypot(*np.diff(np.asarray(pts), axis=0).T)))
                if n_ver < a.verify_n:
                    from gmc.gs3d.planner import _linear_trajectory
                    gq = C.pose(r["goal_uv"], body)
                    vposes, _ = _linear_trajectory([C.pose(p, body) for p in pts], body, 0., 0.)   # turn, then drive
                    ver = verify_path(C.oracle, vposes, body, margin_m=a.margin, goal=GoalRegion(gq, 0., .05))
                    row.update(verify_passed=ver["passed"], verify_reason=ver["reason"],
                               verify_clearance_lower_m=ver.get("clearance_lower_m"))
                    n_ver += 1
                    if a.store_paths:
                        row["route_uv"] = np.round(pts, 4).tolist()
        row["wall_s"] = time.perf_counter() - w0
        res_rows.append(row)
    import collections
    summ = {}
    for r in res_rows:
        summ.setdefault(r["class"], collections.Counter())[r["outcome"]] += 1
    doc = {"robot": a.robot, "variant": a.body, "body": asdict(body), "margin_m": a.margin, "step_m": st,
           "box_name": a.box_name, "box_uv": C.box, "crop": C.ctx["crop"],
           "lattice": {"nodes": nu * nv, "free_nodes": len(poses), "edges_tested": n_edges,
                       "edges_free": sum(len(v) for v in adj.values()) // 2, "components": int(len(labels)),
                       "component_sizes": sorted(sizes.tolist())[::-1][:30], "node_rejections": reasons,
                       "edge_rejections": erej, "node_wall_s": t_nodes, "build_wall_s": t_build},
           "summary": {k: dict(v) for k, v in summ.items()}, "verified_routes": n_ver,
           "wall_s": time.perf_counter() - t0, "peak_rss_mb": rss_mb(), "host": host(), "rows": res_rows}
    out = Path(a.out_dir) / f"grid_{a.robot}_{a.body}_m{a.margin:g}_s{st:g}_{a.box_name}.json"
    _dump(out, doc)
    np.savez_compressed(out.with_suffix(".npz"), us=us, vs=vs, comp=comp)
    print(json.dumps(doc["summary"]), f"{doc['wall_s']:.0f}s rss {rss_mb():.0f} MB", flush=True)


# ----------------------------------------------------------------------------- lamp passage width
def cmd_lampwidth(a):
    from scipy import ndimage
    body = ROBOTS["cylinder"]
    t0 = time.perf_counter()
    C = _Ctx(_box_arg(a.box_name))
    g = C.scene.gaussians
    z_floor = float(C.ctx["support_evidence"]["z_floor_world_m"])
    R = C.frame.R
    mu = C.frame.to_route(g.means)                       # route z = height above z_floor (site origin)
    cov = np.einsum("ij,njk,lk->nil", R, g.covs, R)
    lev = float(C.scene.level)
    zlo, zhi = body.ground_clearance_m, body.ground_clearance_m + 2 * body.half_height_m   # above z_floor
    zr = mu[:, 2]
    sz = lev * np.sqrt(np.maximum(cov[:, 2, 2], 0.))
    inband = (zr + sz > zlo) & (zr - sz < zhi)
    u0, u1, v0, v1 = a.window
    st = a.step
    us = np.arange(u0, u1 + 1e-9, st)
    vs = np.arange(v0, v1 + 1e-9, st)
    zs = np.arange(zlo, zhi + 1e-9, a.zstep)
    occz = np.zeros((len(zs), len(us), len(vs)), bool)
    sel = np.flatnonzero(inband & (mu[:, 0] > u0 - 1) & (mu[:, 0] < u1 + 1) & (mu[:, 1] > v0 - 1) & (mu[:, 1] < v1 + 1))
    U, V = np.meshgrid(us, vs, indexing="ij")
    for n in sel:
        m, S = mu[n], cov[n]
        P = np.linalg.inv(S)
        # cross-section at z: (x - c(z))^T A (x - c(z)) <= lev^2 - k(z), A = P_xy, c from the Schur complement
        A = P[:2, :2]
        Ai = np.linalg.inv(A)
        bvec = P[:2, 2]
        for kz, z in enumerate(zs):
            dz = z - m[2]
            c = m[:2] - Ai @ bvec * dz
            rhs = lev ** 2 - dz * dz * (P[2, 2] - bvec @ Ai @ bvec)
            if rhs <= 0:
                continue
            ext = np.sqrt(rhs * np.diag(Ai))
            i0, i1 = np.searchsorted(us, c[0] - ext[0]), np.searchsorted(us, c[0] + ext[0], "right")
            j0, j1 = np.searchsorted(vs, c[1] - ext[1]), np.searchsorted(vs, c[1] + ext[1], "right")
            if i1 <= i0 or j1 <= j0:
                continue
            du, dv = U[i0:i1, j0:j1] - c[0], V[i0:i1, j0:j1] - c[1]
            q = A[0, 0] * du * du + 2 * A[0, 1] * du * dv + A[1, 1] * dv * dv
            occz[kz, i0:i1, j0:j1] |= q <= rhs
    west, east = us < a.cross_u - .6, us > a.cross_u + .6

    def analyse(occ):
        occ = occ.copy()
        occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True   # window border = domain wall (geometric, not AABB)
        edt = ndimage.distance_transform_edt(~occ) * st     # distance of each free cell centre to the nearest occupied

        def crosses(r):
            lab, _ = ndimage.label(edt >= r, structure=np.ones((3, 3)))
            w = set(np.unique(lab[west][lab[west] > 0]).tolist())
            e = set(np.unique(lab[east][lab[east] > 0]).tolist())
            return bool(w & e)
        lo, hi = 0., 1.
        for _ in range(30):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if crosses(mid) else (lo, mid)
        runs = []          # per-u-column widest free v-run near the lamp line -- a simpler necessary condition
        for i, u in enumerate(us):
            if abs(u - a.cross_u) > a.cross_half:
                continue
            best = cur = 0
            for f in ~occ[i]:
                cur = cur + 1 if f else 0
                best = max(best, cur)
            runs.append((float(u), best * st))
        return occ, edt, lo, runs
    occ, edt, lo, runs = analyse(occz.any(0))
    # which heights close the crossing: band [lo_z, hi_z] sub-bands
    by_band = []
    for zb0, zb1 in [(zlo, zhi), (zlo + .003, zhi - .01), (zlo, .5), (.5, 1.), (1., 1.5), (1.5, zhi), (1.6, zhi),
                     (1.7, zhi), (zlo, 1.7), (zlo, 1.6), (zlo, 1.5)]:
        k = (zs >= zb0 - 1e-9) & (zs <= zb1 + 1e-9)
        _, _, r_, ru = analyse(occz[k].any(0))
        by_band.append({"band_z": [float(zb0), float(zb1)], "max_min_clearance_crossing_m": r_,
                        "column_widest_free_run_min_m": min(x for _, x in ru) if ru else None})
        print("band", by_band[-1], flush=True)
    out = {"window_uv": a.window, "step_m": st, "zstep_m": a.zstep, "band_above_floor_m": [zlo, zhi],
           "gaussians_in_band_near": int(len(sel)), "level": lev, "cross_u": a.cross_u,
           "max_min_clearance_crossing_m": lo, "widest_disk_diameter_crossing_m": 2 * lo,
           "cylinder_diameter_m": 2 * body.radius_m,
           "column_widest_free_run_min_m": min(r for _, r in runs) if runs else None,
           "column_widest_free_run": runs, "by_z_band": by_band,
           "definition": "occupancy = union over z planes (every zstep in the body band) of each Gaussian's 2-sigma "
                         "ellipse cross-section: an under-estimate of each ellipsoid's band shadow.  edt = Euclidean "
                         "distance (cell centres) to the nearest occupied cell.  The cylinder's centre path must keep "
                         "edt > r = 0.30 m everywhere; max_min_clearance_crossing_m is the largest r for which some "
                         "8-connected cell path joins the region west of cross_u - 0.6 to east of cross_u + 0.6.",
           "wall_s": time.perf_counter() - t0}
    p = Path(a.out)
    _dump(p, out)
    np.savez_compressed(p.with_suffix(".npz"), us=us, vs=vs, occ=occ, edt=edt.astype(np.float32))
    print(json.dumps({k: out[k] for k in ("max_min_clearance_crossing_m", "widest_disk_diameter_crossing_m",
                                          "column_widest_free_run_min_m", "gaussians_in_band_near", "wall_s")}),
          flush=True)


# ----------------------------------------------------------------------------- per-row verdicts
V_BLOCKED = "blocked (location picking: pair is genuinely unreachable)"
V_BOX = "box artefact (location picking)"
V_TOL = "reachable within <= 2 mm tolerance (by-tolerance incompleteness)"
V_ALG = "reachable, GMC UNKNOWN (algorithm incompleteness)"
V_SOUND = "reachable, GMC UNREACHABLE (SOUNDNESS BUG - investigate)"
V_UNRES = "unresolved"
V_AGREE = "control: G2 REACHABLE, A* agrees"
V_CTRL_DIS = "control: G2 REACHABLE, A* disagrees"


def _astar(raw, pattern):
    out = {}
    for p in sorted(Path(raw).glob(pattern)):
        for r in read_jsonl(p):
            out[(r["robot"], r["index"])] = r
    return out


def verdict(r, real, w1, grids, ep_clear, s2row=None):
    cls, st = r["class"], r["g2_status"]
    ro = (real or {}).get("outcome")
    w = (w1 or {}).get("outcome")
    g = {k: v.get("outcome") for k, v in grids.items()}
    tol_route = g.get("real_m0") == "CONNECTED" or g.get("s2_m0") == "CONNECTED"
    s2_neg = g.get("s2_m0", "SEPARATED") == "SEPARATED" or (
        g.get("s2_m0") == "NO_ATTACH" and (s2row or {}).get("outcome") in ("NO_ROUTE", "NO_ROUTE_MARGIN"))
    blocked = g.get("s10_m0") == "SEPARATED" and s2_neg
    if cls == "CTRL-REACHABLE":
        return V_AGREE if ro == "ROUTE" else V_CTRL_DIS, ""
    if st == "UNREACHABLE":
        if ro == "ROUTE":
            return V_SOUND, "real-body A* ROUTE in the G2 box against a certified cut"
        if blocked:
            return V_BLOCKED, "no lattice route even for the 2 mm and 1 cm subset bodies at margin 0"
        if tol_route:
            return V_TOL, "route only for a <= 2 mm subset body / margin 0 (cut certificate vs tolerance: check)"
        return V_UNRES, ""
    # UNKNOWN rows
    if r["g2_reason"].endswith("_not_certified_free"):
        c = ep_clear
        note = f"endpoint oracle clearance {c * 1e3:.2f} mm (F1)" if c is not None else "endpoint clearance unknown"
        under = ("; pair itself: real-body route in G2 box" if ro == "ROUTE" else
                 "; pair itself: real-body route only in W1 (box artefact underneath)" if w == "ROUTE" else
                 "; pair itself: route only within tolerance in G2 box" if tol_route else "; pair itself: no route found")
        if c is not None and c <= .002 + 1e-6:
            return V_TOL, note + under
        if ro == "ROUTE":
            return V_ALG, note + under
        return V_UNRES, note + under
    if r["g2_reason"] == "shared_replay_failed":
        return (V_ALG, "route exists (A*); GMC's own verifier certified one, the replay veto is export round-off "
                       "(F1 3.3)") if ro == "ROUTE" else (V_UNRES, "")
    if r["g2_reason"].startswith("safe_graph_disconnected"):
        if ro == "ROUTE":
            return V_ALG, "real-body A* route inside the G2 box"
        if w == "ROUTE":
            return V_BOX, "real-body A* ROUTE in W1 (+1 m); in the G2 box only " + (
                "a <= 2 mm-tolerance route" if tol_route else "no route")
        if tol_route:
            return V_TOL, "no real-body route in G2 or W1; subset-body route in G2"
        if blocked:
            return V_BLOCKED, ""
        return V_UNRES, ""
    return V_UNRES, "unhandled G2 reason"


def cmd_verdicts(a):
    import csv
    import collections
    rows = json.loads(Path(a.rows).read_text())["rows"]
    raw = Path(a.raw)
    real = _astar(raw, "*_real_m0.001_r0.1_G2_*.jsonl")
    w1 = _astar(raw, "*_real_m0.001_r0.1_W1_*.jsonl")
    s2 = _astar(Path(a.raw_s2), "*_s2_m0_r0.05_G2_*.jsonl")
    grids = {}
    for p in sorted(Path(a.grid_dir).glob("grid_*_G2.json")):
        d = json.loads(p.read_text())
        key = f"{d['variant']}_m{d['margin_m']:g}" + ("" if d["step_m"] == .05 else f"_s{d['step_m']:g}")
        for r in d["rows"]:
            grids.setdefault((d["robot"], r["index"]), {})[key] = r
    f1 = {(r["robot"], int(r["index"])): json.loads(r["evidence"]) for r in csv.DictReader(open(a.classes))}
    out = []
    for r in rows:
        k = (r["robot"], r["index"])
        ev = f1.get(k, {})
        v, note = verdict(r, real.get(k), w1.get(k), grids.get(k, {}), ev.get("oracle_clearance_m"), s2.get(k))
        g = grids.get(k, {})
        out.append({"robot": r["robot"], "index": r["index"], "pair_id": r["pair_id"], "f1_class": r["class"],
                    "g2_status": r["g2_status"], "g2_reason": r["g2_reason"], "dist_m": round(r["dist_m"], 3),
                    "astar_real_g2": (real.get(k) or {}).get("outcome"),
                    "astar_real_g2_reason": (real.get(k) or {}).get("reason"),
                    "astar_real_g2_expansions": (real.get(k) or {}).get("expansions"),
                    "astar_real_g2_unproven": (real.get(k) or {}).get("unproven_rejections"),
                    "astar_real_g2_unproven_gaussians": (real.get(k) or {}).get("unproven_gaussians"),
                    "astar_real_g2_unproven_under_chassis": (real.get(k) or {}).get("unproven_under_chassis"),
                    "astar_real_g2_wall_s": round((real.get(k) or {}).get("wall_s") or 0, 2) or None,
                    "astar_real_w1": (w1.get(k) or {}).get("outcome"),
                    "astar_s2_g2": (s2.get(k) or {}).get("outcome"),
                    **{f"grid_{n}": (g.get(n) or {}).get("outcome") for n in
                       ("real_m0.001", "real_m0", "s2_m0", "s10_m0")},
                    "f1_endpoint_clearance_mm": round(ev["oracle_clearance_m"] * 1e3, 3)
                    if ev.get("oracle_clearance_m") is not None else None,
                    "verdict": v, "note": note})
    p = Path(a.out_csv)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0]))
        w.writeheader()
        w.writerows(out)
    tab = collections.defaultdict(collections.Counter)
    for r in out:
        tab[f"{r['robot']}:{r['f1_class']}"][r["verdict"]] += 1
    cov = {n: sum(r[n] is not None for r in out) for n in ("astar_real_g2", "astar_real_w1", "astar_s2_g2",
                                                            "grid_real_m0.001", "grid_real_m0", "grid_s2_m0", "grid_s10_m0")}
    agree = collections.Counter((r["f1_class"], r["astar_s2_g2"], r["grid_s2_m0"]) for r in out if r["astar_s2_g2"])
    summ = {"n_rows": len(out), "coverage": cov, "class_x_verdict": {k: dict(v) for k, v in tab.items()},
            "astar_real_g2_by_class": {c: dict(collections.Counter(r["astar_real_g2"] for r in out if
                                                                    f"{r['robot']}:{r['f1_class']}" == c))
                                       for c in tab},
            "s2_perrow_vs_grid": {f"{a_}|{b}|{c}": n for (a_, b, c), n in agree.items()}}
    _dump(p.with_name("summary.json"), summ)
    print(json.dumps(summ, indent=1))


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("rows")
    r.add_argument("--n-ctrl", type=int, default=120)
    r.add_argument("--seed", type=int, default=20261006)
    r.add_argument("--out", default="configs/aerial3dg/f3_audit_rows.json")
    for name in ("astar", "grid"):
        q = sub.add_parser(name)
        q.add_argument("--robot", required=True)
        q.add_argument("--body", default="real", choices=["real", "s2", "s10"])
        q.add_argument("--margin", type=float, default=.001)
        q.add_argument("--box-name", default="G2")
        q.add_argument("--rows", default="configs/aerial3dg/f3_audit_rows.json")
        q.add_argument("--classes", nargs="*", default=None)
        q.add_argument("--out-dir", default="results/aerial3dg/f3/g2_audit/raw")
    q = sub.choices["astar"]
    q.add_argument("--resolution", type=float, default=.1)
    q.add_argument("--max-wall", type=float, default=300.)
    q.add_argument("--shard", type=int, default=int(os.environ.get("SLURM_ARRAY_TASK_ID", 0)))
    q.add_argument("--nshards", type=int, default=1)
    q.add_argument("--stride", type=int, default=1)
    q.add_argument("--offset", type=int, default=0)
    q.add_argument("--max-hours", type=float, default=7.5)
    q.add_argument("--tag", default="", help="file-name tag (e.g. the class subset)")
    q = sub.choices["grid"]
    q.add_argument("--step", type=float, default=.05)
    q.add_argument("--verify-n", type=int, default=300)
    q.add_argument("--store-paths", action="store_true")
    w = sub.add_parser("lampwidth")
    w.add_argument("--box-name", default="-2.6,-0.8,1.0,3.2")
    w.add_argument("--window", type=float, nargs=4, default=[-2.5, 0.0, -0.35, 2.75], metavar=("U0", "U1", "V0", "V1"))
    w.add_argument("--step", type=float, default=.01)
    w.add_argument("--zstep", type=float, default=.02)
    w.add_argument("--cross-u", type=float, default=-.85)
    w.add_argument("--cross-half", type=float, default=.3)
    w.add_argument("--out", default="results/aerial3dg/f3/g2_audit/lampwidth.json")
    v = sub.add_parser("verdicts")
    v.add_argument("--rows", default="configs/aerial3dg/f3_audit_rows.json")
    v.add_argument("--raw", default="results/aerial3dg/f3/g2_audit/raw")
    v.add_argument("--raw-s2", default="results/aerial3dg/f3/g2_audit/raw_s2")
    v.add_argument("--grid-dir", default="results/aerial3dg/f3/g2_audit/raw")
    v.add_argument("--classes", default="results/aerial3dg/f1/classes.csv")
    v.add_argument("--out-csv", default="results/aerial3dg/f3/g2_audit/verdicts.csv")
    a = p.parse_args(argv)
    {"rows": cmd_rows, "astar": cmd_astar, "grid": cmd_grid, "lampwidth": cmd_lampwidth,
     "verdicts": cmd_verdicts}[a.cmd](a)


if __name__ == "__main__":
    main()
