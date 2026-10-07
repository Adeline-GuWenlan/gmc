"""F5 Task 1: per-row verification facts + mechanism trace of every GMC failure on confirmed-reachable pairs.

Run from ``gmc/`` with ``PYTHONPATH=src:experiments``.  Inputs: F4's ``failures.csv`` (1567 rows, both robots) and
F3's verified genuine-failure candidates (``f3/f4_handoff.json -> f5_candidate_cases``, group genuine / borderline;
kept separate as source ``F3-targeted``).  Nothing in ``gmc/src`` is changed: the trace wraps module globals of
``gmc.aerial3d.api`` (``PortalGraph``, ``replay_plan``, ``simplify``, ``shortcut`` / ``tighten`` /
``merge_corners``, ``grow_cell``) for the duration of one query and restores them.

  plan    (inline) shard files under results/aerial3dg/f5/plan/ + plan.txt (one shard per array task)
  trace   (sbatch) one shard: per row
            (a) ``astar``: the stored A* route (route frame, 0.1 mm rounding) rebuilt as unicycle poses (start yaw 0,
                turn in place, translate, final turn to yaw 0) and re-replayed with ``verify_path`` through the
                compile's own gs3d oracle (``GaussianBodyOracle(compiled.prepared)``: the oracle GMC's shared
                replay uses) on the real cylinder and on the row's robot, margin 0.001, with the goal region.
                If the 0.1 mm rounding breaks it, the A* is re-run (deterministic, F2/F3 settings) and its exact
                poses are verified instead (``replayed_from``).
            (b) ``query``: GMC re-run on the same persisted compile with G2's QCONFIG and G2's 120 s limit for
                TIMEOUT rows (300 s otherwise), instrumented: cells + portals of the safe-graph path (polygons at
                z_c), endpoint cells, the lifted / certified / exported polylines, the shared replay's first failing
                edge (geometry) or step (kinematics), the own verifier's failing segment, post-processing stage
                times and accept() counts (kept even when the 120 s alarm fires), the ``simplify`` input on error.
                ``reproduced`` = same status and reason (TIMEOUT: again >= 120 s).
            (c) ``clearance``: fine clearances by bisection with the same oracle: both endpoints (row robot, margin
                ladder 0.01 mm), and the route's lateral clearance (body grown by d, chassis bottom raised 3 mm,
                margin 0 -- F3's ``lateral_body``) to 0.01 mm + its tightest edge.  ``tolerance`` = the failure
                locus clearance <= margin + buffer (2 mm).
            plus endpoint stage facts (octree leaf status, possible-graph components) for both endpoints, and for
            ``*_not_certified_free`` the replay of ``grow_cell``'s None branch (F1's ``endpoint_trace``: blocking
            pairs, refined gap vs buffer + slack, 2-sigma tops).
            Probes (fix tests, never src changes):
              verdict-only (``aerial3dg_fail_widen.FAST``) for TIMEOUT rows: the certified, unshortened route;
              ``budget``: QCONFIG with a post-processing deadline (accept() refuses after ``--post-budget`` s from
                the first shortcut/tighten call; the polyline is always the last certified one), 120 s limit, and on a
                replay veto the 1 ms turn + translation floors (``aerial3dg_fail2_kin``);
              ``guard``: ``simplify`` with a guard for a == c (drops the a -> b -> a spike) for METHOD-ERROR rows;
              ``b0``: the same, with the endpoint query cell grown at buffer 0 (F3's ``bufzero``) where flagged.
          Also writes outputs/aerial3dg/f5/<R>_<robot>_pairs.npz (ids, means, covs of the compile's pairs; for figures).
"""
from __future__ import annotations

import argparse
import collections
from dataclasses import asdict
import csv
import json
import math
from pathlib import Path
import time
import traceback

import numpy as np

from aerial3dg_run import MANIFEST, QCONFIG, ROBOTS, _dump

F3 = Path("results/aerial3dg/f3")
F4 = Path("results/aerial3dg/f4")
F5 = Path("results/aerial3dg/f5")
OUT_BIG = Path("outputs/aerial3dg/f5")
A3C = Path("outputs/aerial3dg/f3")
MARGIN, BUFFER = .001, .001
TOL = MARGIN + BUFFER


# ---------------------------------------------------------------------------------------------- plan (inline)
def _jsonl(p):
    out = []
    for line in open(p):
        line = line.strip()
        if line:
            out.append(json.loads(line))
    return out


def cmd_plan(a):
    rows = list(csv.DictReader(open(F4 / "failures.csv")))
    pairs, gmc = {}, {}
    for reg in ("WWEST", "GAPW1", "S"):
        pairs.update({p["pair_id"]: p for p in json.loads((F4 / "sample" / f"{reg}_pairs.json").read_text())["pairs"]})
        for robot in ("cylinder", "sweeper"):
            for r in _jsonl(F4 / "diag" / reg / robot / "rows_all.jsonl"):
                gmc[(robot, r["pair_id"])] = r
    b0_ids = {"F4W-01351", "F4X-02245", "F4X-00667", "F4X-01092", "F4X-00914", "F4X-01885"}
    items = []
    for r in rows:
        g, p = gmc[(r["robot"], r["pair_id"])], pairs[r["pair_id"]]
        items.append({"case_id": f"{r['region']}-{r['pair_id']}", "source": "F4", "region": r["region"],
                      "robot": r["robot"], "pair_id": r["pair_id"], "class_f4": r["class"],
                      "gmc": {k: g[k] for k in ("status", "reason", "start_uv", "goal_uv", "outer_wall_s")},
                      "astar_route_uv": p["astar_route_uv"], "pairs_file": f"gmc/{F4}/sample/{r['region']}_pairs.json",
                      "b0_probe": r["pair_id"] in b0_ids, "guard_probe": r["class"] == "METHOD-ERROR"})
    ho = json.loads((F3 / "f4_handoff.json").read_text())
    for c in ho["f5_candidate_cases"]:
        if not (c["group"].startswith("genuine") or c["group"].startswith("borderline")):
            continue
        reg, run, pid = c["case_id"].split("-", 2)
        pf = Path(c["pairs_file"].removeprefix("gmc/"))
        p = {q["pair_id"]: q for q in json.loads(pf.read_text())["pairs"]}[pid]
        g = {x["pair_id"]: x for x in _jsonl(F3 / "gmc" / reg / run / c["robot"] / "task_00.jsonl")}[pid]
        items.append({"case_id": c["case_id"], "source": "F3-targeted", "region": reg, "run": run,
                      "robot": c["robot"], "pair_id": pid, "class_f4": c["class"], "f3_group": c["group"],
                      "gmc": {k: g[k] for k in ("status", "reason", "start_uv", "goal_uv", "outer_wall_s")},
                      "astar_route_uv": p["astar_route_uv"], "pairs_file": c["pairs_file"],
                      "b0_probe": c["group"] != "genuine", "guard_probe": pid == "F3X-00163"})
    # shards: per (region, robot, kind); TIMEOUT rows are the expensive ones
    by = collections.defaultdict(list)
    for it in items:
        kind = "timeout" if it["gmc"]["status"] == "TIMEOUT" or it["b0_probe"] and it["source"] == "F3-targeted" \
            else "fast"
        by[(it["region"], it["robot"], kind)].append(it)
    d = F5 / "plan"
    d.mkdir(parents=True, exist_ok=True)
    for old in d.glob("*.jsonl"):
        old.unlink()
    lines = []
    for (reg, robot, kind), its in sorted(by.items()):
        size = a.timeout_shard if kind == "timeout" else a.fast_shard
        for k in range(0, len(its), size):
            f = d / f"{reg}_{robot}_{kind}_{k // size:02d}.jsonl"
            with open(f, "w") as fh:
                for it in its[k:k + size]:
                    fh.write(json.dumps(it) + "\n")
            lines.append(f"{reg} {robot} {f}")
    (d / "plan.txt").write_text("\n".join(lines) + "\n")
    print(len(items), "rows;", collections.Counter(it["source"] for it in items), ";", len(lines), "shards")
    for k, v in sorted(by.items()):
        print(k, len(v))


# ---------------------------------------------------------------------------------------------- helpers (sbatch)
def poly_at_z(cell, z, pad=.05):
    """2-D polygon (route u, v) of a convex cell {A x <= b} on the plane z (Sutherland-Hodgman on its bbox)."""
    A, b = np.asarray(cell.A, float), np.asarray(cell.b, float)
    lo, hi = np.asarray(cell.bbox_lower, float), np.asarray(cell.bbox_upper, float)
    P = [np.array([lo[0] - pad, lo[1] - pad]), np.array([hi[0] + pad, lo[1] - pad]),
         np.array([hi[0] + pad, hi[1] + pad]), np.array([lo[0] - pad, hi[1] + pad])]
    for n, c in zip(A[:, :2], b - A[:, 2] * z):
        if not P:
            break
        if np.linalg.norm(n) < 1e-15:
            if c < 0:
                P = []
            continue
        Q = []
        for i in range(len(P)):
            p, q = P[i], P[(i + 1) % len(P)]
            fp, fq = n @ p - c, n @ q - c
            if fp <= 0:
                Q.append(p)
            if fp * fq < 0:
                Q.append(p + (q - p) * (fp / (fp - fq)))
        P = Q
    if len(P) < 3:
        return None, 0.
    P = np.asarray(P)
    area = .5 * abs(np.dot(P[:, 0], np.roll(P[:, 1], -1)) - np.dot(P[:, 1], np.roll(P[:, 0], -1)))
    return P.round(4).tolist(), float(area)


def guarded_simplify(points, tol: float = 1e-9) -> np.ndarray:
    """``query.simplify`` + the guard F3/F4 proposed: if a == c (ac . ac == 0) the vertex b is an a -> b -> a spike;
    drop b and c (the polyline then goes a -> next, which is the old c -> next segment, already certified)."""
    P = [np.asarray(points[0], float)]
    for p in np.asarray(points, float)[1:]:
        if np.linalg.norm(p - P[-1]) > tol:
            P.append(p)
    changed = True
    while changed and len(P) > 2:
        changed = False
        for i in range(1, len(P) - 1):
            a, b, c = P[i - 1], P[i], P[i + 1]
            ab, ac = b - a, c - a
            den = float(ac @ ac)
            if den <= tol * tol:
                del P[i:i + 2]
                changed = True
                break
            t = float(ab @ ac) / den
            if 0 <= t <= 1 and np.linalg.norm(ab - t * ac) <= tol:
                del P[i]
                changed = True
                break
    return np.asarray(P)


class Ctx:
    def __init__(self, region, robot):
        from gmc.aerial3d.api import load_compiled
        from gmc.gs3d.oracle import GaussianBodyOracle
        t0 = time.perf_counter()
        self.region, self.robot = region, robot
        self.a3c = A3C / region / f"{robot}.a3c"
        self.C = load_compiled(self.a3c)
        self.load_s = time.perf_counter() - t0
        self.oracle = GaussianBodyOracle(self.C.prepared)
        self.z_c = float(self.C.domain.ground_z)
        self.man = json.loads(MANIFEST.read_text())
        b = self.C.body
        assert abs(self.z_c - (b.ground_clearance_m + b.half_height_m)) < 1e-6, (self.z_c, b)
        ks = self.C.prepared.scene.known_space
        self.ks = getattr(ks, "inner", ks)          # uavlamp_query.FaceLoggingKnownSpace wraps the route prism
        self._planner = None
        self.save_pairs()

    def save_pairs(self):
        OUT_BIG.mkdir(parents=True, exist_ok=True)
        f = OUT_BIG / f"{self.region}_{self.robot}_pairs.npz"
        if f.exists():
            return
        P, d = self.C.pairs, self.C.domain
        tmp = f.with_suffix(f".tmp{np.random.randint(1 << 30)}.npz")
        np.savez_compressed(tmp, ids=P.ids, means=P.means.astype(np.float32), covs=P.covs.astype(np.float32),
                            level=P.level, z_c=self.z_c, dom_A=d.A, dom_b=d.b, compile_id=self.C.compile_id,
                            body=json.dumps(asdict(self.C.body)),
                            ks_lower=np.asarray(self.ks.lower_route_m), ks_upper=np.asarray(self.ks.upper_route_m))
        tmp.rename(f)

    # -- frames / poses
    def world(self, uv, body):
        return self.C.frame.to_world([uv[0], uv[1], body.ground_clearance_m + body.half_height_m])

    def plan_uv(self, xyz_w):
        return self.C.frame.to_plan(np.atleast_2d(np.asarray(xyz_w, float)))[:, :2]

    def route_poses(self, uv, body):
        from gmc.gs3d.contracts import Pose3
        from gmc.gs3d.validation import angle_delta
        W = [self.world(p, body) for p in uv]
        poses = [Pose3(tuple(map(float, W[0])), 0.)]
        for q in W[1:]:
            prev = poses[-1]
            d = np.asarray(q) - prev.xyz
            if float(np.linalg.norm(d[:2])) <= 1e-12:
                continue
            yaw = prev.yaw + angle_delta(math.atan2(d[1], d[0]), prev.yaw)
            if yaw != prev.yaw:
                poses.append(Pose3(prev.xyz, yaw))
            poses.append(Pose3(tuple(map(float, q)), yaw))
        last = poses[-1]
        turn = angle_delta(0., last.yaw)
        if turn != 0.:
            poses.append(Pose3(last.xyz, last.yaw + turn))
        return poses

    def goal_region(self, uv, body):
        from gmc.gs3d.contracts import GoalRegion, Pose3
        return GoalRegion(Pose3(tuple(map(float, self.world(uv, body))), 0.), 0., .05)

    def free(self, uv, body, margin):
        from gmc.gs3d.contracts import Pose3
        rep = self.oracle.pose(Pose3(tuple(map(float, self.world(uv, body))), 0.), body, margin_m=margin)
        return rep.occupancy == "free" and rep.safety == "continuous_bound", rep

    def rerun_astar(self, s_uv, g_uv):
        from gmc.gs3d.contracts import PlannerConfig, Pose3, SearchBudget
        from gmc.gs3d.planner import LatticePlanner
        from aerial3dg_fail_oracle import BUDGET
        if self._planner is None:
            self._planner = LatticePlanner(self.C.prepared)
        body = ROBOTS["cylinder"]
        s = Pose3(tuple(map(float, self.world(s_uv, body))), 0.)
        conf = PlannerConfig(resolution_m=.1, margin_m=MARGIN, seed=0, budget=SearchBudget(max_wall_s=120., **BUDGET))
        res = self._planner.plan(self.C.prepared.scene, body, s, self.goal_region(g_uv, body), conf)
        if res["status"] != "success":
            return None
        return [Pose3((p[0], p[1], p[2]), p[3]) for p in res["trajectory"]["poses"]]


def lateral_body(body, d):
    from dataclasses import replace
    top = body.ground_clearance_m + 2 * body.half_height_m + d
    bot = body.ground_clearance_m + .003
    return replace(body, name=f"lat_probe_{d:g}", radius_m=body.radius_m + d, ground_clearance_m=bot,
                   half_height_m=(top - bot) / 2)


def shift(poses, body_from, body_to):
    from gmc.gs3d.contracts import Pose3
    dz = (body_to.ground_clearance_m + body_to.half_height_m) - (body_from.ground_clearance_m + body_from.half_height_m)
    return [Pose3((p.xyz[0], p.xyz[1], p.xyz[2] + dz), p.yaw) for p in poses]


def bisect(ok, lo, hi, res):
    """largest x in [lo, hi] with ok(x) (ok monotone decreasing), to ``res``; None if not ok(lo)."""
    if not ok(lo):
        return None
    if ok(hi):
        return hi
    while hi - lo > res:
        mid = (lo + hi) / 2
        if ok(mid):
            lo = mid
        else:
            hi = mid
    return lo


# ---------------------------------------------------------------------------------------------- (a) + (c)
def fact_astar(X: Ctx, it):
    from gmc.gs3d.validation import verify_path
    cyl = ROBOTS["cylinder"]
    rb = X.C.body
    uv = it["astar_route_uv"]
    s_uv, g_uv = it["gmc"]["start_uv"], it["gmc"]["goal_uv"]
    t0 = time.perf_counter()
    poses = X.route_poses(uv, cyl)
    src = "stored_route_uv_0.1mm"
    out = {}
    v = verify_path(X.oracle, poses, cyl, margin_m=MARGIN, goal=X.goal_region(g_uv, cyl))
    if not v["passed"]:
        out["stored_route_replay"] = {"passed": False, "reason": v["reason"], "failed_edge": len(v["reports"]) - 1}
        P = X.rerun_astar(s_uv, g_uv)
        if P is not None:
            poses, src = P, "astar_rerun_exact_poses"
            v = verify_path(X.oracle, poses, cyl, margin_m=MARGIN, goal=X.goal_region(g_uv, cyl))
    rec = {"replayed_from": src, "n_poses": len(poses), "oracle": "GaussianBodyOracle(compiled.prepared)",
           "margin_m": MARGIN, "cylinder": {"passed": v["passed"], "reason": v["reason"],
                                            "clearance_lower_m": v.get("clearance_lower_m"),
                                            "failed_edge": None if v["passed"] else len(v["reports"]) - 1}}
    rposes = poses
    if rb.name != cyl.name:
        rposes = shift(poses, cyl, rb)
        w = verify_path(X.oracle, rposes, rb, margin_m=MARGIN, goal=X.goal_region(g_uv, rb))
        rec[rb.name] = {"passed": w["passed"], "reason": w["reason"], "clearance_lower_m": w.get("clearance_lower_m")}
    rec["passed"] = all(rec[k]["passed"] for k in ("cylinder", rb.name) if k in rec)
    # (c) lateral clearance of the row robot along the route (F3's lateral body, margin 0), fine + tightest edge
    def lat_ok(d):
        b = lateral_body(rb, d)
        return verify_path(X.oracle, shift(rposes, rb, b), b, margin_m=0.)["passed"]
    lat = bisect(lat_ok, 0., .1, 1e-5)
    # tightest point: the first edge that fails just above the fine clearance (the oracle's per-edge clearance_lower_m
    # is an AABB bound, too loose to rank edges)
    k_best, best = None, None
    if lat is not None and lat < .1:
        b1 = lateral_body(rb, lat + 2e-5)
        v1 = verify_path(X.oracle, shift(rposes, rb, b1), b1, margin_m=0.)
        if not v1["passed"]:
            k_best, best = len(v1["reports"]) - 1, v1["reason"]
    tight = None
    if k_best is not None:
        k_best = min(max(k_best, 0), len(rposes) - 2)
        tight = X.plan_uv([rposes[k_best].xyz, rposes[k_best + 1].xyz]).mean(0).round(4).tolist()
    rec["lateral_fine_m"] = lat
    rec["lateral_tightest_edge"] = {"edge": k_best, "uv": tight, "fails_at_m": None if k_best is None else lat + 2e-5,
                                    "reason": best}
    rec["route_uv_used"] = X.plan_uv([p.xyz for p in rposes]).round(4).tolist() if src != "stored_route_uv_0.1mm" else None
    rec["wall_s"] = time.perf_counter() - t0
    return rec


def fact_endpoints(X: Ctx, it):
    """Endpoint stage facts for both endpoints + fine oracle clearance of the row robot (margin bisection)."""
    from gmc.aerial3d import api as A
    from gmc.aerial3d.octree import STATUS_NAMES
    from gmc.aerial3d.query import cells_containing
    from aerial3dg_fail_diag import endpoint_trace
    rb = X.C.body
    out = {}
    for e in ("start", "goal"):
        uv = it["gmc"][f"{e}_uv"]
        p = np.array([uv[0], uv[1], X.z_c])
        ep = A._endpoint(X.C, p, e, None)
        rec = {"uv": uv, "ok": ep["ok"], "reason": ep.get("reason"),
               "possible_components": sorted(ep.get("components") or []),
               "leaf_status": sorted({STATUS_NAMES[int(X.C.tree.status[l])] for l in (ep.get("leaves") or [])}),
               "in_compiled_cell": bool(cells_containing(X.C.cells.cells, p))}
        clear = bisect(lambda m: X.free(uv, rb, m)[0], 0., .02, 1e-5)
        rec["clearance_fine_m"] = clear
        rec["oracle_clearance_lower_m001"] = X.free(uv, rb, MARGIN)[1].clearance_lower_m
        if it["gmc"]["reason"] == f"{e}_not_certified_free":
            tr = endpoint_trace(X.C, p, X.man)
            rec["grow_cell_trace"] = {k: tr[k] for k in ("domain_box_status", "domain_row_slack_m", "tightest_domain_row",
                                                         "pairs_failing_fixed_directions", "n_blockers", "refined_ok",
                                                         "min_refined_gap_m", "cause", "blockers")}
        out[e] = rec
    out["shared_possible_component"] = bool(set(out["start"]["possible_components"])
                                            & set(out["goal"]["possible_components"]))
    return out


# ---------------------------------------------------------------------------------------------- (b) instrumented query
def run_query(X: Ctx, it, *, config, timeout, budget_s=None, guard=False, buffer0=False, tag="full"):
    from gmc.aerial3d import api as A
    from gmc.aerial3d.query import PortalGraph, cells_containing
    from aerial3dg_batch import _Timeout, _alarm
    CAP = {"post": {}, "deadline": None, "stage": "pre_post"}
    saved = {k: getattr(A, k) for k in ("PortalGraph", "replay_plan", "simplify", "shortcut", "tighten",
                                        "merge_corners", "grow_cell")}

    class CapGraph(PortalGraph):
        def __init__(self, cx):
            super().__init__(cx)
            CAP["graph"] = self

    def cap_replay(result, oracle):
        rep = saved["replay_plan"](result, oracle)
        CAP["replay"], CAP["gs3d"] = rep, result
        return rep

    def cap_simplify(points, tol=1e-9):
        CAP["simplify_in"] = np.asarray(points, float).copy()
        return (guarded_simplify if guard else saved["simplify"])(points, tol)

    def wrap(name):
        real = saved[name]

        def f(points, accept, *args, **kw):
            st = CAP["post"].setdefault(name, {"calls": 0, "accept_calls": 0, "accept_s": 0., "wall_s": 0.,
                                               "refused_after_deadline": 0, "vertices_in": []})
            if budget_s is not None and CAP["deadline"] is None:
                CAP["deadline"] = time.perf_counter() + budget_s
            CAP["stage"] = name
            st["vertices_in"].append(len(points))
            t0 = time.perf_counter()

            def acc(x, y):
                st["accept_calls"] += 1
                if CAP["deadline"] is not None and time.perf_counter() > CAP["deadline"]:
                    st["refused_after_deadline"] += 1
                    return False
                t1 = time.perf_counter()
                try:
                    return accept(x, y)
                finally:
                    st["accept_s"] += time.perf_counter() - t1
            try:
                res = real(points, acc, *args, **kw)
            except BaseException:
                CAP["stage"] = name                     # interrupted inside this stage (alarm / exception)
                raise
            else:
                CAP["stage"] = "after_" + name
                return res
            finally:
                st["calls"] += 1
                st["wall_s"] += time.perf_counter() - t0
        return f

    def grow(table, domain, centre, half, *, buffer_m, kind="support_plane", **kw):
        if kind == "query" and buffer0:
            buffer_m = 0.
        return saved["grow_cell"](table, domain, centre, half, buffer_m=buffer_m, kind=kind, **kw)

    A.PortalGraph, A.replay_plan, A.simplify, A.grow_cell = CapGraph, cap_replay, cap_simplify, grow
    for n in ("shortcut", "tighten", "merge_corners"):
        setattr(A, n, wrap(n))
    s = X.C.frame.to_world([*it["gmc"]["start_uv"], X.z_c])
    g = X.C.frame.to_world([*it["gmc"]["goal_uv"], X.z_c])
    w0 = time.perf_counter()
    q, err = None, None
    try:
        with _alarm(timeout):
            q = A.query(X.C, s, g, config=config, call_id=f"{it['pair_id']}-{tag}")
        st, rs = q["status"], q["reason"]
    except _Timeout:
        st, rs = "TIMEOUT", f"query_exceeded_{timeout:g}s"
    except Exception as exc:
        if isinstance(exc.__cause__, _Timeout) or isinstance(exc.__context__, _Timeout):
            st, rs = "TIMEOUT", f"query_exceeded_{timeout:g}s (alarm surfaced as {type(exc).__name__})"
        else:
            tb = traceback.extract_tb(exc.__traceback__)
            st = "ERROR"
            rs = f"{type(exc).__name__}: {exc} @ {tb[-1].filename.split('/')[-1]}:{tb[-1].lineno}"
            err = {"traceback": [f"{t.filename.split('src/')[-1]}:{t.lineno} {t.name}" for t in tb[-6:]]}
    finally:
        for k, v in saved.items():
            setattr(A, k, v)
    wall = time.perf_counter() - w0
    rec = {"tag": tag, "status": st, "reason": rs, "wall_s": wall, "timeout_s": timeout, "budget_s": budget_s,
           "guard": guard, "buffer0": buffer0, "stage_at_end": CAP["stage"],
           "post": {k: {**v, "vertices_in": v["vertices_in"][:4]} for k, v in CAP["post"].items()}}
    if err:
        rec["error"] = err
        if "simplify_in" in CAP:
            P = CAP["simplify_in"]
            rec["simplify_in_uv"] = P[:, :2].round(5).tolist()
            Q = [P[0]]
            for p in P[1:]:
                if np.linalg.norm(p - Q[-1]) > 1e-9:
                    Q.append(p)
            spikes = [i for i in range(1, len(Q) - 1) if float((Q[i + 1] - Q[i - 1]) @ (Q[i + 1] - Q[i - 1])) == 0.]
            rec["simplify_spikes"] = [{"i": i, "a_uv": Q[i - 1][:2].round(5).tolist(), "b_uv": Q[i][:2].round(5).tolist(),
                                       "ab_m": float(np.linalg.norm(Q[i] - Q[i - 1]))} for i in spikes]
            rec["simplify_n_in"], rec["simplify_n_dedup"] = len(P), len(Q)
    graph = CAP.get("graph")
    if q is not None:
        tm = q.get("timings") or {}
        rec["stage_s"] = {t["stage"]: round(t["seconds"], 3) for t in tm.get("records", [])}
        for k in ("polyline_plan", "cell_polyline_plan", "lifted_polyline_plan"):
            if q.get(k) is not None:
                rec[k.replace("_plan", "_uv")] = np.asarray(q[k])[:, :2].round(4).tolist()
        gp = q.get("graph_path")
        if gp and graph is not None:
            cells = []
            for c in gp["cell_ids"]:
                poly, area = poly_at_z(graph.cells[c], X.z_c)
                cells.append({"id": int(c), "kind": graph.cells[c].kind, "poly_uv": poly, "area_m2": area})
            rec["path_cells"] = cells
            rec["n_path_cells"] = len(cells)
            rec["n_portals"] = len(gp["portals"])
        own = (q.get("verification") or {}).get("own")
        if own:
            rec["own"] = {"status": own["status"], "reason": own["reason"], "clearance_lower_m": own["clearance_lower_m"]}
            bad = [i for i, sgm in enumerate(own["segments"]) if sgm["status"] != "CERTIFIED"]
            if bad and q.get("polyline_plan") is not None:
                i = bad[0]
                P = np.asarray(q["polyline_plan"])
                rec["own"]["failed_segment"] = {"i": i, **own["segments"][i],
                                                "a_uv": P[i, :2].round(4).tolist(), "b_uv": P[i + 1, :2].round(4).tolist()}
        if "replay" in CAP:
            rp, gs = CAP["replay"], CAP["gs3d"]
            geo = rp["geometry"]
            rr = {"passed": rp["passed"], "geometry_passed": geo.get("passed"), "geometry_reason": geo.get("reason"),
                  "kinematics": {k: v for k, v in rp["kinematics"].items() if k != "acceleration_verified"},
                  "attained_matches": rp["attained_matches"], "n_exported_poses": len(gs["trajectory"]["poses"])}
            T = np.asarray(gs["trajectory"]["poses"], float)
            if not geo.get("passed"):
                k = len(geo.get("reports") or []) - 1
                k = max(k, 0)
                a_, b_ = T[k, :3], T[min(k + 1, len(T) - 1), :3]
                rr["failed_edge"] = {"k": k, "a_uv": X.plan_uv(a_)[0].round(4).tolist(),
                                     "b_uv": X.plan_uv(b_)[0].round(4).tolist()}
                body = X.C.body
                half = np.array([body.radius_m, body.radius_m, body.half_height_m])
                lo, hi = np.minimum(a_, b_) - half, np.maximum(a_, b_) + half
                from itertools import product
                corners = np.asarray(list(product(*zip(lo, hi))), float)
                ks = X.ks
                R = (corners - np.asarray(ks.origin_world_m)) @ np.asarray(ks.world_to_route).T
                lo_r, hi_r = np.asarray(ks.lower_route_m), np.asarray(ks.upper_route_m)
                ex = np.maximum(lo_r - R.min(0), R.max(0) - hi_r)
                rr["failed_edge"].update(aabb_corners_route_uv=R[:, :2].round(4).tolist(),
                                         excess_beyond_prism_m=ex.round(6).tolist(),
                                         prism_lower=lo_r.tolist(), prism_upper=hi_r.tolist())
            if not rp["kinematics"].get("passed", True):
                from aerial3dg_fail_fixprobe import kin_steps
                bad = kin_steps(gs["trajectory"], X.C.body)
                rr["kin_violations"] = bad[:3]
                rr["n_kin_violations"] = len(bad)
                if bad:
                    rr["kin_locus_uv"] = X.plan_uv(T[bad[0]["step"], :3])[0].round(4).tolist()
            if not rp["passed"] and q.get("polyline_world") is not None:
                from aerial3dg_fail2_kin import _replay, floored_trajectory
                from gmc.aerial3d.api import _densify
                from gmc.gs3d.contracts import Pose3
                dens = _densify(np.asarray(q["polyline_world"], float), config.export_max_segment_m)
                path = [Pose3(tuple(map(float, x))) for x in dens]
                rr["both_floors"] = _replay(gs, floored_trajectory(path, X.C.body, 0., 0., 1e-3, 1e-3), X.oracle)
            rec["replay"] = rr
    if graph is not None:
        ends = {}
        for e in ("start", "goal"):
            p = np.array([*it["gmc"][f"{e}_uv"], X.z_c])
            cs = cells_containing(graph.cells, p)
            if cs:
                poly, area = poly_at_z(graph.cells[cs[0]], X.z_c)
                ends[e] = {"cell": int(cs[0]), "query_cell": bool(cs[0] >= len(X.C.cells.cells)), "poly_uv": poly,
                           "area_m2": area, "n_containing": len(cs)}
        rec["endpoint_cells"] = ends
    return rec


def trace_row(X: Ctx, it, a):
    from aerial3dg_fail_widen import FAST
    t0 = time.perf_counter()
    g = it["gmc"]
    out = {"case_id": it["case_id"], "source": it["source"], "region": it["region"], "robot": it["robot"],
           "pair_id": it["pair_id"], "class_f4": it["class_f4"], "gmc_orig": f"{g['status']}:{g['reason']}",
           "compile_id": X.C.compile_id, "a3c": str(X.a3c)}
    out["astar"] = fact_astar(X, it)
    out["endpoints"] = fact_endpoints(X, it)
    timeout = 120. if g["status"] == "TIMEOUT" else a.timeout
    out["query"] = run_query(X, it, config=QCONFIG, timeout=timeout)
    qn = out["query"]
    out["reproduced"] = (qn["status"] == "TIMEOUT") if g["status"] == "TIMEOUT" else \
        (qn["status"], qn["reason"]) == (g["status"], g["reason"]) or \
        (g["status"] == "ERROR" and qn["status"] == "ERROR" and qn["reason"].split(":")[0] in g["reason"])
    probes = {}
    if g["status"] == "TIMEOUT":
        probes["verdict_only"] = run_query(X, it, config=FAST, timeout=a.timeout, tag="verdict_only")
        probes["budget"] = run_query(X, it, config=QCONFIG, timeout=120., budget_s=a.post_budget, tag="budget")
    if it.get("guard_probe") and not it.get("b0_probe"):
        probes["guard"] = run_query(X, it, config=QCONFIG, timeout=a.timeout, guard=True, tag="guard")
    if it.get("b0_probe"):
        probes["b0"] = run_query(X, it, config=QCONFIG, timeout=120., buffer0=True, tag="b0")
        b0 = probes["b0"]
        if b0["status"] == "TIMEOUT":
            probes["b0_verdict_only"] = run_query(X, it, config=FAST, timeout=a.timeout, buffer0=True,
                                                  tag="b0_verdict_only")
            probes["b0_budget"] = run_query(X, it, config=QCONFIG, timeout=120., budget_s=a.post_budget,
                                            buffer0=True, tag="b0_budget")
        if b0["status"] == "ERROR" or it.get("guard_probe"):
            probes["b0_guard"] = run_query(X, it, config=QCONFIG, timeout=120., guard=True, buffer0=True,
                                           tag="b0_guard")
    out["probes"] = probes
    out["wall_s"] = time.perf_counter() - t0
    return out


def cmd_trace(a):
    items = _jsonl(a.shard)
    out = Path(a.out) if a.out else F5 / "trace" / Path(a.shard).name
    out.parent.mkdir(parents=True, exist_ok=True)
    done = {r["case_id"] + r["robot"] for r in _jsonl(out)} if out.exists() else set()
    todo = [it for it in items if it["case_id"] + it["robot"] not in done]
    print(f"shard {a.shard}: {len(items)} rows, {len(done)} done, {len(todo)} to do", flush=True)
    if not todo:
        return
    X = Ctx(todo[0]["region"], todo[0]["robot"])
    print("compile", X.C.compile_id, "load", round(X.load_s, 1), "s", flush=True)
    for it in todo:
        assert (it["region"], it["robot"]) == (X.region, X.robot)
        r = trace_row(X, it, a)
        with open(out, "a") as f:
            f.write(json.dumps(r, default=float) + "\n")
        print(r["case_id"], r["robot"], r["gmc_orig"], "->", r["query"]["status"], r["query"]["reason"][:50],
              "repro", r["reproduced"], "astar", r["astar"]["passed"], r["astar"]["replayed_from"][:6],
              {k: v["status"] for k, v in r["probes"].items()}, f"{r['wall_s']:.0f}s", flush=True)


def cmd_segprobe(a):
    """POST-TIMEOUT mechanism probe: G2's query (QCONFIG, 120 s) with every ``verify_segment`` call logged -- the
    stage it ran in, segment length, the padded segment AABB's area, pairs_checked (pairs whose AABB meets it),
    subdivisions, status, seconds.  Rows: ``--ids`` from the shard."""
    from gmc.aerial3d import api as A
    from aerial3dg_batch import _Timeout, _alarm
    items = [it for it in _jsonl(a.shard) if not a.ids or it["pair_id"] in a.ids]
    X = Ctx(items[0]["region"], items[0]["robot"])
    out = Path(a.out)
    real_vs = A.verify_segment
    saved = {k: getattr(A, k) for k in ("shortcut", "tighten", "merge_corners")}
    for it in items:
        log, stage = [], {"s": "pre"}

        def vs(table, domain, p, q, **kw):
            t0 = time.perf_counter()
            r = real_vs(table, domain, p, q, **kw)
            d = np.abs(np.asarray(q, float) - np.asarray(p, float))
            log.append([stage["s"], round(float(np.linalg.norm(d[:2])), 4),
                        round(float((d[0] + .1) * (d[1] + .1)), 4), r["pairs_checked"], r["subdivisions"],
                        r["status"][:3], round(time.perf_counter() - t0, 4)])
            return r

        def wrap(name):
            def f(*args, **kw):
                stage["s"] = name
                return saved[name](*args, **kw)
            return f
        A.verify_segment = vs
        for n in saved:
            setattr(A, n, wrap(n))
        s = X.C.frame.to_world([*it["gmc"]["start_uv"], X.z_c])
        g = X.C.frame.to_world([*it["gmc"]["goal_uv"], X.z_c])
        w0 = time.perf_counter()
        try:
            with _alarm(a.timeout):
                q = A.query(X.C, s, g, config=QCONFIG, call_id=it["pair_id"] + "-seg")
            st = q["status"]
        except BaseException as exc:
            st = "TIMEOUT" if isinstance(exc, _Timeout) or isinstance(exc.__context__, _Timeout) else type(exc).__name__
        finally:
            A.verify_segment = real_vs
            for k, v in saved.items():
                setattr(A, k, v)
        rec = {"case_id": it["case_id"], "pair_id": it["pair_id"], "status": st, "wall_s": time.perf_counter() - w0,
               "pairs_in_compile": int(len(X.C.pairs)),
               "columns": ["stage", "len_xy_m", "aabb_area_m2", "pairs_checked", "subdivisions", "status", "s"],
               "calls": log}
        with open(out, "a") as f:
            f.write(json.dumps(rec) + "\n")
        L = np.asarray([c[1] for c in log]) if log else np.zeros(0)
        print(it["case_id"], st, f"{rec['wall_s']:.0f}s", len(log), "calls", flush=True)


class CylinderKnownSpace:
    """Fix probe for REPLAY-AABB (shared replay only, never GMC): the route prism tested against the swept *vertical
    cylinder* instead of its world AABB.  ``contains_aabb(lower, upper)`` gets the oracle's world box of the body over
    one edge; its centre box [lower + half, upper - half] holds both edge endpoints at two opposite corners.  A vertical
    cylinder is rotation invariant in xy, and the swept hull of two such cylinders extends, along any route axis, no
    further than one of its endpoint cylinders: so it suffices (conservatively, all 4 xy corners) that every centre-box
    corner +- (r, r, half_height) in the route frame lies in the prism."""

    def __init__(self, inner, body):
        self.inner = inner
        self.half = np.array([body.radius_m, body.radius_m, body.half_height_m])
        self.R = np.asarray(inner.world_to_route, float)
        assert abs(abs(self.R[2, 2]) - 1.) < 1e-9, "route z must be world z"
        self.o = np.asarray(inner.origin_world_m, float)
        self.lo, self.hi = np.asarray(inner.lower_route_m, float), np.asarray(inner.upper_route_m, float)
        self.calls, self.aabb_rejects, self.cyl_rejects = 0, 0, 0

    def contains_aabb(self, lower, upper):
        from itertools import product
        self.calls += 1
        if self.inner.contains_aabb(lower, upper):
            return True
        self.aabb_rejects += 1
        lo, hi = np.asarray(lower, float) + self.half, np.asarray(upper, float) - self.half
        if np.any(lo > hi + 1e-12):
            self.cyl_rejects += 1
            return False
        C = (np.asarray(list(product(*zip(lo, hi))), float) - self.o) @ self.R.T
        ok = bool(np.all(C - self.half >= self.lo - 1e-12) and np.all(C + self.half <= self.hi + 1e-12))
        self.cyl_rejects += not ok
        return ok


def cmd_aabbprobe(a):
    """Every REPLAY-AABB row of the shard (F4 class EXPORT-DOMAIN): GMC re-queried on the same compile with G2's
    QCONFIG, but the shared replay's coverage test is ``CylinderKnownSpace``.  GMC itself is untouched."""
    from gmc.aerial3d import api as A
    from aerial3dg_batch import _Timeout, _alarm
    items = [it for it in _jsonl(a.shard) if it["class_f4"] == "EXPORT-DOMAIN"]
    if not items:
        return
    X = Ctx(items[0]["region"], items[0]["robot"])
    scene = X.C.prepared.scene
    ks0 = scene.known_space
    cks = CylinderKnownSpace(X.ks, X.C.body)
    object.__setattr__(scene, "known_space", cks)
    out = Path(a.out)
    done = {json.loads(l)["case_id"] for l in open(out)} if out.exists() else set()
    try:
        for it in items:
            if it["case_id"] in done:
                continue
            c0 = (cks.calls, cks.aabb_rejects, cks.cyl_rejects)
            s = X.C.frame.to_world([*it["gmc"]["start_uv"], X.z_c])
            g = X.C.frame.to_world([*it["gmc"]["goal_uv"], X.z_c])
            w0 = time.perf_counter()
            try:
                with _alarm(a.timeout):
                    q = A.query(X.C, s, g, config=QCONFIG, call_id=it["pair_id"] + "-cylks")
                st, rs = q["status"], q["reason"]
                geo = ((q.get("verification") or {}).get("shared") or {}).get("geometry") or {}
            except _Timeout:
                st, rs, geo = "TIMEOUT", "timeout", {}
            rec = {"case_id": it["case_id"], "pair_id": it["pair_id"], "status": st, "reason": rs,
                   "replay_geometry": geo.get("reason"), "wall_s": time.perf_counter() - w0,
                   "coverage_calls": cks.calls - c0[0], "aabb_rejects": cks.aabb_rejects - c0[1],
                   "cylinder_rejects": cks.cyl_rejects - c0[2]}
            with open(out, "a") as f:
                f.write(json.dumps(rec) + "\n")
            print(rec, flush=True)
    finally:
        object.__setattr__(scene, "known_space", ks0)


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    pl = sub.add_parser("plan")
    pl.add_argument("--fast-shard", type=int, default=120)
    pl.add_argument("--timeout-shard", type=int, default=12)
    t = sub.add_parser("trace")
    t.add_argument("--shard", required=True)
    t.add_argument("--out", default=None)
    t.add_argument("--timeout", type=float, default=300.)
    t.add_argument("--post-budget", type=float, default=60.)
    sp = sub.add_parser("segprobe")
    sp.add_argument("--shard", required=True)
    sp.add_argument("--ids", nargs="*", default=None)
    sp.add_argument("--timeout", type=float, default=120.)
    sp.add_argument("--out", required=True)
    ap = sub.add_parser("aabbprobe")
    ap.add_argument("--shard", required=True)
    ap.add_argument("--timeout", type=float, default=300.)
    ap.add_argument("--out", required=True)
    a = p.parse_args(argv)
    {"plan": cmd_plan, "trace": cmd_trace, "segprobe": cmd_segprobe, "aabbprobe": cmd_aabbprobe}[a.cmd](a)


if __name__ == "__main__":
    main()
