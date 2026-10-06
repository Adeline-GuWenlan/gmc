"""F3 Task 1: confirmed-reachable pairs for the REAL body (no robust inflation), with difficulty evidence.

Run from ``gmc/`` with ``PYTHONPATH=src:experiments`` (sbatch).

The rule is F2's (``aerial3dg_fail2_sample.py``) with the robust body replaced by the real cylinder at the shared
margin 0.001 (brief, "confirmed reachable"): F2's ``Tester`` with ``lat = 0`` and ``vmargin = 0.001`` makes the
"robust" body the real cylinder, so its stages become

  0. straight xy distance >= ``--min-dist`` (3 m; G2's convention)
  1. both endpoints oracle-free for the real cylinder, margin 0.001          (stage 2 repeats it: always passes)
  3. cheap pre-filter: both endpoints attach by one real-body oracle edge to one component of a shared 0.1 m
     route-frame lattice of the real body at margin 0.001 (built once per region; only ever rejects; its false-
     rejection rate is measured in the pilot with ``--audit``)
  4. gs3d lattice A* (``LatticePlanner``, 0.1 m, position tol 0, yaw tol 0.05, 500k expansions) on the real
     cylinder at margin 0.001 returns ROUTE (its post-build verification of every edge + kinematics passed)
  5. that route re-verified edge by edge with ``verify_path`` (real cylinder, margin 0.001, with the goal)

Rejection is whole-draw, so any stream prefix is an unbiased sample of the conditional law.  A cylinder route is a
sweeper route too (the sweeper is inside the cylinder: same axis, same chassis bottom, r 0.175 < 0.30, top 0.10 <
1.75), so the same pairs are confirmed for both robots.

Targeted discovery (``--mode``; never mixed into the uniform rates):
  detour  extra stage 0b: the straight segment start -> goal is NOT oracle-free for the real cylinder (endpoints on
          opposite sides of clutter); everything else identical.
  tight   extra stage 1b: at least one endpoint is within 3 cm laterally of an obstacle (the probe body r + 3 cm,
          chassis bottom raised 1 cm, top + 3 cm, is not free at margin 0 there) AND the straight segment is blocked.

Every accepted pair also gets difficulty evidence along its A* route (real body):
  clear3d_m   largest margin in LADDER at which ``verify_path`` still passes (oracle 3-D clearance, floor included)
  lateral_m   largest radius / top growth in LADDER at which the body with its chassis bottom raised 1 cm (so floor
              splats below 0.03 m are ignored) still passes at margin 0: lateral + overhead clearance
  vertical_m  geometric under-chassis gap: min over Gaussians whose 2-sigma top is below the chassis bottom + 1 cm
              and whose xy 2-sigma disk (max axis) comes within the body radius of the route, of (bottom - top).
              (The xy disk over-covers the ellipse, so this is a lower bound.)
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
import os
from pathlib import Path
import time

import numpy as np

from aerial3dg_run import GROUND_CONFIG, ROBOTS, _dump, host, rss_mb
from aerial3dg_fail2_sample import MIN_DIST, Tester, draws, read_jsonl, stage_stats, _sum
import aerial3dg_fail2_sample as f2s

LADDER = (.001, .002, .003, .005, .01, .02, .05, .1)


class RealTester(Tester):
    def __init__(self, box_uv, max_wall, resolution, mode="uniform"):
        super().__init__(box_uv, 0., GROUND_CONFIG.margin_m, max_wall, resolution)
        self.mode = mode
        b = self.body
        self.probe_tight = replace(b, name="cylinder_tight_probe", radius_m=b.radius_m + .03,
                                   ground_clearance_m=b.ground_clearance_m + .01,
                                   half_height_m=b.half_height_m + .01)          # bottom +1 cm, top +3 cm
        g = self.ctx["scene"].gaussians
        lev = float(self.ctx["scene"].level)
        L = self.ctx["frame"].to_route(g.means)
        R = self.ctx["frame"].R
        cov = np.einsum("ij,njk,lk->nil", R, g.covs, R)
        top = L[:, 2] + lev * np.sqrt(np.maximum(cov[:, 2, 2], 0.))
        low = top < b.ground_clearance_m + .01
        self.low_uv = L[low, :2]
        self.low_top = top[low]
        self.low_rxy = lev * np.sqrt(np.maximum(np.maximum(cov[low, 0, 0], cov[low, 1, 1]), 0.))
        # max-axis bound of the xy ellipse: sqrt of the largest eigenvalue of the xy block
        ev = np.linalg.eigvalsh(cov[low][:, :2, :2])[:, -1]
        self.low_rxy = lev * np.sqrt(np.maximum(ev, 0.))

    # ------------------------------------------------------------------ difficulty evidence
    def _passes(self, route_uv, body, margin):
        from gmc.gs3d.validation import verify_path
        return verify_path(self.oracle, [self.pose(uv, body) for uv in route_uv], body, margin_m=margin)["passed"]

    def _ladder(self, ok):
        lo, hi = -1, len(LADDER)            # invariant: LADDER[lo] passes (lo=-1: unknown), LADDER[hi] fails
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ok(LADDER[mid]):
                lo = mid
            else:
                hi = mid
        return LADDER[lo] if lo >= 0 else 0.

    def lateral_body(self, d):
        b = self.body
        top = b.ground_clearance_m + 2 * b.half_height_m + d
        bot = b.ground_clearance_m + .01
        return replace(b, name=f"lat_probe_{d:g}", radius_m=b.radius_m + d, ground_clearance_m=bot,
                       half_height_m=(top - bot) / 2)

    def vertical_gap(self, route_uv):
        r = self.body.radius_m
        P = np.asarray(route_uv, float)
        best = np.inf
        lo, hi = P.min(0) - r - .5, P.max(0) + r + .5
        m = np.all((self.low_uv >= lo) & (self.low_uv <= hi), axis=1)
        X, T, RX = self.low_uv[m], self.low_top[m], self.low_rxy[m]
        if not len(X):
            return None
        for a, b in zip(P[:-1], P[1:]):
            ab = b - a
            t = np.clip(((X - a) @ ab) / max(ab @ ab, 1e-18), 0, 1)
            d = np.linalg.norm(X - (a + t[:, None] * ab), axis=1)
            hit = d <= r + RX
            if hit.any():
                best = min(best, float(np.min(self.body.ground_clearance_m - T[hit])))
        return None if not np.isfinite(best) else best

    def evidence(self, row):
        t0 = time.perf_counter()
        R = row["route_uv"]
        s, g = np.asarray(row["start_uv"]), np.asarray(row["goal_uv"])
        length = float(np.sum(np.linalg.norm(np.diff(np.asarray(R), axis=0), axis=1)))
        row.update(route_len_m=length, len_ratio=length / max(float(np.linalg.norm(g - s)), 1e-9),
                   clear3d_m=self._ladder(lambda m: self._passes(R, self.body, m)),
                   lateral_m=self._ladder(lambda d: self._passes(R, self.lateral_body(d), 0.)),
                   vertical_m=self.vertical_gap(R),
                   start_lateral_m=self._ladder(lambda d: self._passes([R[0]], self.lateral_body(d), 0.)),
                   goal_lateral_m=self._ladder(lambda d: self._passes([R[-1]], self.lateral_body(d), 0.)),
                   t_evidence_s=time.perf_counter() - t0)

    # ------------------------------------------------------------------ targeted stages + evidence
    def test(self, s_uv, g_uv, audit=False) -> dict:
        if self.mode in ("detour", "tight"):
            t0 = time.perf_counter()
            pre = {}
            for e, uv in (("start", s_uv), ("goal", g_uv)):
                ok, c, why = self.free(self.pose(uv, self.body), self.body, self.vmargin)
                if not ok:
                    return {"stage_failed": "endpoint_m001", "endpoint_reason": why, f"{e}_free_m001": False,
                            "t_endpoint_s": time.perf_counter() - t0}
            if self.mode == "tight":
                near = [not self.free(self.pose(uv, self.probe_tight), self.probe_tight, 0.)[0] for uv in (s_uv, g_uv)]
                pre["tight_start"], pre["tight_goal"] = near
                if not any(near):
                    return {**pre, "stage_failed": "targeted_tight", "t_target_s": time.perf_counter() - t0}
            rep = self.oracle.edge(self.pose(s_uv, self.body), self.pose(g_uv, self.body), self.body,
                                   margin_m=self.vmargin)
            pre["straight_free"] = rep.occupancy == "free" and rep.safety == "continuous_bound"
            pre["t_target_s"] = time.perf_counter() - t0
            if pre["straight_free"]:
                return {**pre, "stage_failed": "targeted_detour"}
            row = super().test(s_uv, g_uv, audit)
            row.update(pre)
        else:
            row = super().test(s_uv, g_uv, audit)
        if row["stage_failed"] is None:
            self.evidence(row)
        return row


def cmd_lattice(a):
    t = RealTester(a.box, a.max_wall, a.resolution)
    meta = t.build_lattice()
    a.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(a.out, **t.lattice)
    meta.update(box_uv=a.box, body="cylinder (real)", margin_m=t.vmargin, crop=t.ctx["crop"], load_s=t.load_s,
                prepare_s=t.prepare_s, support=t.ctx["support_evidence"], host=host(), peak_rss_mb=rss_mb())
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
    tester = RealTester(a.box, a.max_wall, a.resolution, a.mode)
    if a.lattice and Path(a.lattice).exists():
        tester.load_lattice(a.lattice)
        lat_meta = {"path": str(a.lattice), "built_here": False}
    else:
        lat_meta = {**tester.build_lattice(), "built_here": True}
        np.savez_compressed(out.with_suffix(".lattice.npz"), **tester.lattice)
    seed = a.seed + 1000 * a.stream
    print(f"stream {a.stream} seed {seed} mode {a.mode}: resumed {n_cand} / {accepted}; prepare "
          f"{tester.prepare_s:.1f} s; crop {tester.ctx['crop']['selected_supports']}; lattice {lat_meta}", flush=True)
    meta = {"schema": "aerial3dg_fail3.sample_stream.v1", "stream": a.stream, "seed": seed, "box_uv": a.box,
            "mode": a.mode, "body": "cylinder (real)", "margin_m": tester.vmargin, "resolution_m": a.resolution,
            "astar_budget": {**f2s.BUDGET, "max_wall_s": a.max_wall}, "audit": a.audit, "min_dist_m": a.min_dist,
            "crop": tester.ctx["crop"], "support": tester.ctx["support_evidence"],
            "archive_sha256": tester.ctx["digest"], "load_s": tester.load_s, "prepare_s": tester.prepare_s,
            "lattice": lat_meta, "host": host(), "ladder": LADDER}
    _dump(out.with_suffix(".meta.json"), meta)
    with open(out, "a") as f:
        n_draws = 0
        for k, s, g, d in draws(seed, a.box):
            if accepted >= a.quota or n_cand >= a.max_candidates:
                break
            if time.perf_counter() - t0 > a.max_hours * 3600:
                print("wall-clock guard reached; resubmit to resume", flush=True)
                break
            n_draws = k + 1
            if d < a.min_dist or k in seen:
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
            if n_cand % 25 == 0 or row["accepted"]:
                print(f"cand {n_cand} acc {accepted} last {row['stage_failed']} {row.get('astar_outcome')} "
                      f"{row['wall_s']:.1f}s lat {row.get('lateral_m')} ratio {row.get('len_ratio')} "
                      f"rss {rss_mb():.0f} MB", flush=True)
    meta.update(draws_consumed=n_draws, wall_s=time.perf_counter() - t0, peak_rss_mb=rss_mb())
    _dump(out.with_suffix(".meta.json"), meta)
    print(f"stream {a.stream} done: {n_cand} candidates, {accepted} accepted, {time.perf_counter() - t0:.0f} s, "
          f"peak rss {rss_mb():.0f} MB", flush=True)


def funnel(rows, mode="uniform"):
    """Stage table on the distance-passing candidates (+ targeted stages first), A* reject split, costs."""
    order = (["targeted_tight", "targeted_detour"] if mode != "uniform" else []) + \
        ["endpoint_m001", "endpoint_robust", "prefilter", "astar_robust", "reverify_m001"]
    out, alive = {}, len(rows)
    for st in order:
        failed = sum(r["stage_failed"] == st for r in rows)
        out[st.replace("_robust", "_real" if st == "astar_robust" else "_real_dup")] = {
            "entered": alive, "failed": failed, "pass_rate": (alive - failed) / max(alive, 1)}
        alive -= failed
    ast = [r for r in rows if r.get("astar_outcome") and r.get("prefilter_passed")]
    rej = [r for r in rows if r.get("prefilter_passed") is False]
    out["astar_split_on_prefilter_pass"] = {o: sum(r["astar_outcome"] == o for r in ast)
                                            for o in ("ROUTE", "NO_ROUTE", "NO_ROUTE_MARGIN", "UNSURE")}
    out["astar_split_on_prefilter_reject_audit"] = {o: sum(r.get("astar_outcome") == o for r in rej)
                                                    for o in ("ROUTE", "NO_ROUTE", "NO_ROUTE_MARGIN", "UNSURE")}
    out["cost_s"] = {"endpoint": _sum(rows, "t_endpoint_s"), "targeted": _sum(rows, "t_target_s"),
                     "prefilter": _sum(rows, "t_prefilter_s"),
                     "astar_route": _sum([r for r in ast if r["astar_outcome"] == "ROUTE"], "t_astar_s"),
                     "astar_reject": _sum([r for r in ast if r["astar_outcome"] != "ROUTE"], "t_astar_s"),
                     "astar_audit": _sum(rej, "t_astar_s"), "reverify": _sum(rows, "t_reverify_s"),
                     "evidence": _sum(rows, "t_evidence_s"), "candidate_cpu": _sum(rows, "cpu_s")}
    acc = [r for r in rows if r["accepted"]]
    out["accepted"] = len(acc)
    cpu = sum(r.get("cpu_s", 0.) for r in rows)
    out["cpu_s_total"] = cpu
    out["cpu_s_per_accepted"] = cpu / max(len(acc), 1)
    for k in ("len_ratio", "lateral_m", "clear3d_m", "vertical_m", "dist_m"):
        x = np.array([r[k] for r in acc if r.get(k) is not None], float)
        out[f"accepted_{k}"] = None if not len(x) else {
            "min": float(x.min()), "p10": float(np.percentile(x, 10)), "median": float(np.median(x)),
            "p90": float(np.percentile(x, 90)), "max": float(x.max())}
    return out


def cmd_collect(a):
    rows, metas = [], []
    for p in sorted(Path(a.out_dir).glob("stream_*.jsonl")):
        m = json.loads(p.with_suffix(".meta.json").read_text())
        metas.append(m)
        rows += [{**r, "stream": m["stream"]} for r in read_jsonl(p)]
    mode = metas[0]["mode"]
    acc = sorted([r for r in rows if r["accepted"]], key=lambda r: (r["stream"], r["draw"]))
    if a.n is not None:
        acc = acc[:a.n]
    pairs = []
    for i, r in enumerate(acc):
        pairs.append({"index": i, "pair_id": f"{a.prefix}-{i:05d}", "start_uv": r["start_uv"], "goal_uv": r["goal_uv"],
                      "dist_m": r["dist_m"], "stream": r["stream"], "draw": r["draw"], "region": a.region,
                      "clearance_m": {"cylinder": {"start": r["start_clear_m001"], "goal": r["goal_clear_m001"]}},
                      "evidence": {k: r.get(k) for k in ("route_len_m", "len_ratio", "clear3d_m", "lateral_m",
                                                         "vertical_m", "start_lateral_m", "goal_lateral_m",
                                                         "straight_free", "tight_start", "tight_goal")},
                      "astar": {k: r.get(k) for k in ("astar_outcome", "astar_clearance_lower_m", "path_length_m",
                                                      "reverify_m001_passed", "reverify_m001_clearance_lower_m",
                                                      "astar_expansions", "t_astar_s")},
                      "astar_route_uv": r["route_uv"]})
    from aerial3dg_run import _box
    doc = {"schema": "aerial3dg.pairs.v1+f3_confirmed_real", "n": len(pairs), "region": a.region, "mode": mode,
           "box_uv": metas[0]["box_uv"], "box_route": _box(metas[0]["box_uv"]), "robots": ["cylinder", "sweeper"],
           "min_dist_m": metas[0]["min_dist_m"], "margin_m": GROUND_CONFIG.margin_m, "rule": __doc__.strip(),
           "streams": [{k: m.get(k) for k in ("stream", "seed", "draws_consumed", "audit", "mode", "wall_s")}
                       for m in metas],
           "funnel": funnel(rows, mode), "distance_passing_candidates": len(rows),
           "crop": metas[0]["crop"], "archive_sha256": metas[0]["archive_sha256"], "pairs": pairs}
    _dump(Path(a.out), doc)
    print(json.dumps({k: doc[k] for k in ("n", "distance_passing_candidates", "funnel")}, default=float), flush=True)


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    la = sub.add_parser("lattice")
    r = sub.add_parser("run")
    for q in (la, r):
        q.add_argument("--box", type=float, nargs=4, required=True, metavar=("U0", "V0", "U1", "V1"))
        q.add_argument("--resolution", type=float, default=.1)
        q.add_argument("--max-wall", type=float, default=120.)
    la.add_argument("--out", type=Path, required=True)
    r.add_argument("--lattice", type=Path, default=None)
    r.add_argument("--mode", default="uniform", choices=["uniform", "detour", "tight"])
    r.add_argument("--stream", type=int, default=0)
    r.add_argument("--seed", type=int, default=20261006)
    r.add_argument("--quota", type=int, default=200)
    r.add_argument("--max-candidates", type=int, default=10 ** 9)
    r.add_argument("--min-dist", type=float, default=MIN_DIST)
    r.add_argument("--audit", action="store_true")
    r.add_argument("--max-hours", type=float, default=7.5)
    r.add_argument("--out-dir", type=Path, required=True)
    c = sub.add_parser("collect")
    c.add_argument("--out-dir", type=Path, required=True)
    c.add_argument("--n", type=int, default=None)
    c.add_argument("--region", required=True)
    c.add_argument("--prefix", required=True)
    c.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)
    {"lattice": cmd_lattice, "run": cmd_run, "collect": cmd_collect}[a.cmd](a)


if __name__ == "__main__":
    main()
