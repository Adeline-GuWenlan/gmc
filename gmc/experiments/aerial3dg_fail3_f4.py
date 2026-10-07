"""F4: 5000 confirmed-reachable pairs in F3's hard regions, GMC on all of them (both robots), first-pass classification.

Run from ``gmc/`` with ``PYTHONPATH=src:experiments``.  Builds on F3 (``aerial3dg_fail3_sample.py`` sampler and
``collect``, ``aerial3dg_fail3_probe.py`` probes and ``classify_row``, F3's persisted compiles); the regions, quotas,
seeds and pair-id prefixes are in ``REGIONS`` (from ``results/aerial3dg/f3/f4_handoff.json``).

Layout under results/aerial3dg/f4/:
  sample/<R>/stream_1NN.jsonl        sampler streams (``f4_samp.sbatch``, one array task per stream)
  sample/<R>_pairs.json              ``collect``: the region's confirmed pairs (GMC input)
  sample/<R>/prefilter_audit.json    real-body A* on recorded pre-filter rejects (``aerial3dg_fail3_sample.py audit``)
  pairs_confirmed_5000.json          ``merge``: all regions, per pair the A* route + evidence, + the funnel per region
  gmc/<R>/<robot>/task_NN.jsonl      GMC rows (``f3_gmc.sbatch`` on the F3 compile, array over tasks)
  diag/<R>/<robot>/...               probes (``f4_probe.sbatch``): kin.json, bufzero.json, requery_*.json, witness.json,
                                     epclear.json
  summary.json, failures.csv, fig/   ``summary`` / ``plots``

Subcommands (inline = json only, < 2 min; sbatch = loads the scene or a compile):
  collect   (sbatch, big S stream) per region ``aerial3dg_fail3_sample.collect`` with ``--n quota``; checks quotas
  merge     (inline) pairs_confirmed_5000.json + funnel table
  check     (inline) GMC completeness + compile-once proof (one compile id per robot per region, = F3's compile)
  prep      (inline) probe row files: rows_all.jsonl per region/robot, requery subsets (every non-TIMEOUT failure, a
            stratified <= ``--n-timeout`` TIMEOUT sample, every UNREACHABLE/ERROR), witness / epclear id lists
  epclear   (sbatch) endpoint oracle clearance ladder of the robot's OWN real body for ``*_not_certified_free`` rows.
            F3 classified sweeper EP rows with the cylinder's ladder; the sweeper is a sub-body of the cylinder, so its
            clearance can only be larger, and the cylinder ladder cannot show the sweeper's endpoint is within
            margin + buffer.  F4 uses the sweeper's own ladder (``classify``).
  summary   (inline) one class per non-REACHABLE row (F3's ``classify_row`` + the refinement above), failures.csv,
            summary.json, headline / class / band tables (markdown on stdout)
  plots     (inline) failure rate vs lateral-clearance band, vs detour ratio, vs endpoint clearance
"""
from __future__ import annotations

import argparse
import collections
import csv
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from aerial3dg_fail2_sample import read_jsonl
from aerial3dg_fail3_probe import BUFFER, EXPORT, GENUINE, MARGIN, TOLERANCE, UNVERIFIED, classify_row
from aerial3dg_run import _dump

F3 = Path("results/aerial3dg/f3")
F4 = Path("results/aerial3dg/f4")
ROBOTS = ("cylinder", "sweeper")
HANDOFF = json.loads((F3 / "f4_handoff.json").read_text())
PREFIX = {"WWEST": "F4X", "GAPW1": "F4W", "S": "F4S", "G2MID": "F4G"}
REGIONS = {r: {"box": v["box_uv"], "quota": v["quota"], "seed_base": v["seed_base"], "prefix": PREFIX[r],
               "compiles": v["compiles"]}
           for r, v in HANDOFF["regions"].items() if v["quota"] > 0}
LAT_BANDS = [(0, .0005, "0"), (.0005, .002, "0.5-1.5"), (.002, .005, "2-3"), (.005, .01, "5"), (.01, .02, "10"),
             (.02, .05, "20"), (.05, .1, "50"), (.1, 1., ">=100")]          # ladder values (m): rung passed
RATIO_BANDS = [(1., 1.05), (1.05, 1.1), (1.1, 1.2), (1.2, 1.35), (1.35, 1.5), (1.5, 2.), (2., 99.)]
EP_BANDS = [(0, .0015, "<1.5"), (.0015, .002, "1.5"), (.002, .003, "2"), (.003, .01, "3-5"), (.01, 1., ">=10")]


def _load(p):
    p = Path(p)
    return json.loads(p.read_text()) if p.exists() else None


def group_of(c):
    return ("reachable" if c == "REACHABLE" else "genuine" if c in GENUINE else "tolerance" if c in TOLERANCE
            else "export" if c in EXPORT else "unverified" if c in UNVERIFIED else "other")


# ---------------------------------------------------------------------------------------------- sampling
def cmd_collect(a):
    import aerial3dg_fail3_sample as f3s
    for reg in a.regions:
        R = REGIONS[reg]
        d = F4 / "sample" / reg
        for m in sorted(d.glob("stream_*.meta.json")):
            meta = json.loads(m.read_text())
            n_acc = sum(r["accepted"] for r in read_jsonl(m.with_suffix("").with_suffix(".jsonl")))
            print(reg, meta["stream"], "accepted", n_acc, "wall", round(meta.get("wall_s", 0)), flush=True)
        f3s.cmd_collect(SimpleNamespace(out_dir=d, n=R["quota"], region=reg, prefix=R["prefix"],
                                        out=F4 / "sample" / f"{reg}_pairs.json"))


def cmd_merge(a):
    """pairs_confirmed_5000.json: per pair the A* route, length ratio, min lateral / vertical clearance along the
    route, endpoint clearances, region, stream / seed; per region the funnel (counts, pass rates, cost per stage, A*
    split on pre-filter passes + on the audited pre-filter rejects)."""
    pairs, funnels = [], {}
    for reg, R in REGIONS.items():
        doc = _load(F4 / "sample" / f"{reg}_pairs.json")
        assert doc is not None and doc["n"] == R["quota"], (reg, doc and doc["n"], R["quota"])
        f = doc["funnel"]
        aud = _load(F4 / "sample" / reg / "prefilter_audit.json")
        funnels[reg] = {"box_uv": doc["box_uv"], "quota": R["quota"], "accepted": doc["n"],
                        "distance_passing_candidates": doc["distance_passing_candidates"],
                        "streams": doc["streams"],
                        "stages": {k: v for k, v in f.items() if isinstance(v, dict) and "pass_rate" in v},
                        "astar_split_on_prefilter_pass": f["astar_split_on_prefilter_pass"],
                        "prefilter_reject_audit": None if aud is None else {"n": aud["n"], "split": aud["summary"],
                                                                            "file": str(F4 / "sample" / reg /
                                                                                        "prefilter_audit.json")},
                        "cost_s": f["cost_s"], "cpu_s_total": f["cpu_s_total"],
                        "cpu_s_per_accepted": f["cpu_s_per_accepted"],
                        "accepted_stats": {k: f[k] for k in f if k.startswith("accepted_")}}
        for p in doc["pairs"]:
            e = p["evidence"]
            pairs.append({"pair_id": p["pair_id"], "region": reg, "stream": p["stream"],
                          "seed": R["seed_base"] + 1000 * p["stream"], "draw": p["draw"],
                          "start_uv": p["start_uv"], "goal_uv": p["goal_uv"], "dist_m": p["dist_m"],
                          "astar_route_uv": p["astar_route_uv"], "route_len_m": e["route_len_m"],
                          "len_ratio": e["len_ratio"], "lateral_clearance_m": e["lateral_m"],
                          "vertical_clearance_m": e["vertical_m"], "clear3d_m": e["clear3d_m"],
                          "endpoint_clearance_m": {"start": p["clearance_m"]["cylinder"]["start"],
                                                   "goal": p["clearance_m"]["cylinder"]["goal"],
                                                   "start_ladder": e["start_clear3d_m"],
                                                   "goal_ladder": e["goal_clear3d_m"],
                                                   "start_lateral_ladder": e["start_lateral_m"],
                                                   "goal_lateral_ladder": e["goal_lateral_m"]},
                          "astar": p["astar"]})
    doc = {"schema": "aerial3dg_fail3.f4_pairs_confirmed.v1", "n": len(pairs),
           "rule": HANDOFF["confirmation_rule"],
           "evidence_definitions": "lateral_clearance_m / vertical_clearance_m / clear3d_m / *_ladder: "
                                   "experiments/aerial3dg_fail3_sample.py docstring (ladder value = largest rung "
                                   "that passes; endpoint start/goal = oracle clearance_lower_m at margin 0.001)",
           "regions": {r: {"box_uv": v["box"], "quota": v["quota"], "prefix": v["prefix"],
                           "seed_base": v["seed_base"], "pairs_file": str(F4 / "sample" / f"{r}_pairs.json")}
                       for r, v in REGIONS.items()},
           "funnel": funnels, "pairs": pairs}
    _dump(F4 / "pairs_confirmed_5000.json", doc)
    print("pairs", len(pairs), {r: v["accepted"] for r, v in funnels.items()})
    print_funnel(funnels)


def print_funnel(funnels):
    print("| region | n | distance-passing cand. | endpoint pass | pre-filter pass | A* ROUTE / NRM / NR / UNSURE "
          "(pre-filter pass) | pre-filter rejects audited: ROUTE / NRM / NR / UNSURE | CPU s per accepted | "
          "CPU h total |")
    print("|---|---|---|---|---|---|---|---|---|")
    for reg, f in funnels.items():
        s = f["stages"]
        sp = f["astar_split_on_prefilter_pass"]
        au = (f["prefilter_reject_audit"] or {}).get("split") or {}
        aud = " / ".join(str(au.get(k, 0)) for k in ("ROUTE", "NO_ROUTE_MARGIN", "NO_ROUTE", "UNSURE")) \
            + f" (of {f['prefilter_reject_audit']['n']})" if f["prefilter_reject_audit"] else "-"
        print(f"| {reg} | {f['accepted']} | {f['distance_passing_candidates']:,} | "
              f"{s['endpoint_m001']['pass_rate']:.1%} | {s['prefilter']['pass_rate']:.1%} | "
              f"{sp['ROUTE']} / {sp['NO_ROUTE_MARGIN']} / {sp['NO_ROUTE']} / {sp['UNSURE']} | {aud} | "
              f"{f['cpu_s_per_accepted']:.1f} | {f['cpu_s_total'] / 3600:.2f} |")
    print()
    print("| region | stage | entered | failed | pass rate | CPU s in stage |")
    print("|---|---|---|---|---|---|")
    cost_key = {"endpoint_m001": "endpoint", "endpoint_real_dup": None, "prefilter": "prefilter",
                "astar_real": "astar_route", "reverify_m001": "reverify"}
    def cs(f, k):
        return (f["cost_s"].get(k) or {}).get("sum", 0.)
    for reg, f in funnels.items():
        for st, v in f["stages"].items():
            k = cost_key.get(st)
            c = None if k is None else cs(f, k)
            if st == "astar_real":
                c = cs(f, "astar_route") + cs(f, "astar_reject")
            print(f"| {reg} | {st} | {v['entered']:,} | {v['failed']:,} | {v['pass_rate']:.1%} | "
                  f"{'' if c is None else f'{c:,.0f}'} |")
        print(f"| {reg} | evidence (accepted pairs) | {f['accepted']:,} | | | {cs(f, 'evidence'):,.0f} |")


# ---------------------------------------------------------------------------------------------- GMC
def gmc_rows(reg, robot):
    rows = {}
    for t in sorted((F4 / "gmc" / reg / robot).glob("task_*.jsonl")):
        for r in read_jsonl(t):
            rows[r["index"]] = r
    return rows


def cmd_check(a):
    """Every pair answered once per robot; compile-once proof: one compile id per robot per region across all rows,
    equal to F3's persisted compile (whose SHA-256 the handoff records), and no compile stage inside any call."""
    out, ok = {}, True
    for reg, R in REGIONS.items():
        pairs = _load(F4 / "sample" / f"{reg}_pairs.json")
        for robot in ROBOTS:
            rows = gmc_rows(reg, robot)
            ids = {r["compile_id"] for r in rows.values()}
            sums = [_load(s) for s in sorted((F4 / "gmc" / reg / robot).glob("task_*.summary.json"))]
            in_call = sum(bool(r.get("compile_stages_in_call")) for r in rows.values())
            rec = {"rows": len(rows), "expected": pairs["n"] if pairs else None, "compile_ids": sorted(ids),
                   "expected_compile_id": R["compiles"][robot]["compile_id"],
                   "a3c_sha256": R["compiles"][robot]["sha256"],
                   "rows_with_compile_stage_in_call": in_call, "task_summaries": len(sums),
                   "task_compile_once_proofs": [s.get("compile_once_proof") for s in sums if s],
                   "status": dict(collections.Counter(r["status"] for r in rows.values()))}
            rec["ok"] = (pairs is not None and len(rows) == pairs["n"] and ids == {rec["expected_compile_id"]}
                         and in_call == 0)
            ok &= rec["ok"]
            out[f"{reg}/{robot}"] = rec
            print(reg, robot, {k: rec[k] for k in ("rows", "expected", "compile_ids", "expected_compile_id",
                                                  "rows_with_compile_stage_in_call", "status", "ok")})
    _dump(F4 / "compile_once.json", {"all_ok": ok, "per_region_robot": out})
    print("ALL OK" if ok else "NOT OK")


def cmd_prep(a):
    """Probe inputs per region/robot under diag/<R>/<robot>/: rows_all.jsonl (every GMC row; kin / bufzero pick their
    own rows), requery_fail.jsonl (every non-REACHABLE non-TIMEOUT row), requery_timeout_K.jsonl (stratified TIMEOUT
    sample, sharded), and the id lists for the sweeper endpoint ladder."""
    pairs = {}
    for reg in REGIONS:
        pairs.update({p["pair_id"]: p for p in _load(F4 / "sample" / f"{reg}_pairs.json")["pairs"]})
    timeouts = []
    for reg in REGIONS:
        for robot in ROBOTS:
            rows = gmc_rows(reg, robot)
            d = F4 / "diag" / reg / robot
            d.mkdir(parents=True, exist_ok=True)
            with open(d / "rows_all.jsonl", "w") as f:
                for r in sorted(rows.values(), key=lambda r: r["index"]):
                    f.write(json.dumps(r) + "\n")
            fail = [r for r in rows.values() if r["status"] not in ("REACHABLE", "TIMEOUT")]
            with open(d / "requery_fail.jsonl", "w") as f:
                for r in sorted(fail, key=lambda r: r["index"]):
                    f.write(json.dumps(r) + "\n")
            timeouts += [(reg, robot, r) for r in rows.values() if r["status"] == "TIMEOUT"]
            ep = [r["pair_id"] for r in rows.values() if r["reason"].endswith("_not_certified_free")]
            (d / "ep_ids.txt").write_text("\n".join(sorted(ep)) + ("\n" if ep else ""))
            print(reg, robot, "rows", len(rows), "fail(non-TIMEOUT)", len(fail), "EP", len(ep),
                  "replay", sum(r["reason"] == "shared_replay_failed" for r in rows.values()),
                  "TIMEOUT", sum(r["status"] == "TIMEOUT" for r in rows.values()),
                  "UNREACHABLE", sum(r["status"] == "UNREACHABLE" for r in rows.values()))
    # stratified TIMEOUT sample: strata = region x robot x detour-ratio tercile; proportional, >= 1 per non-empty stratum
    rng = np.random.default_rng(20261007)
    n_t = len(timeouts)
    pick = []
    if n_t:
        lr = np.array([pairs[r["pair_id"]]["evidence"]["len_ratio"] for _, _, r in timeouts])
        cuts = np.percentile(lr, [100 / 3, 200 / 3])
        strata = collections.defaultdict(list)
        for (reg, robot, r), x in zip(timeouts, lr):
            strata[(reg, robot, int(np.searchsorted(cuts, x)))].append((reg, robot, r))
        want = min(a.n_timeout, n_t)
        for k, v in sorted(strata.items()):
            m = max(1, round(want * len(v) / n_t))
            idx = rng.permutation(len(v))[:m]
            pick += [v[i] for i in sorted(idx)]
        pick = pick[:max(want, len(strata))]
    shards = collections.defaultdict(list)
    for i, (reg, robot, r) in enumerate(sorted(pick, key=lambda x: (x[0], x[1], x[2]["index"]))):
        shards[(reg, robot)].append(r)
    plan = []
    for (reg, robot), rr in shards.items():
        d = F4 / "diag" / reg / robot
        for old in d.glob("requery_timeout_*.jsonl"):
            old.unlink()
        for k in range(0, len(rr), a.shard):
            p = d / f"requery_timeout_{k // a.shard:02d}.jsonl"
            with open(p, "w") as f:
                for r in rr[k:k + a.shard]:
                    f.write(json.dumps(r) + "\n")
            plan.append(str(p))
    _dump(F4 / "diag" / "timeout_sample.json", {"n_timeout_rows": n_t, "n_sampled": len(pick),
                                                 "strata": "region x robot x len_ratio tercile (all TIMEOUT rows)",
                                                 "seed": 20261007, "files": plan,
                                                 "pair_ids": [[g, b, r["pair_id"]] for g, b, r in pick]})
    print("TIMEOUT rows", n_t, "sampled", len(pick), "files", plan)


def cmd_epclear(a):
    """Endpoint oracle clearance of ``--robot``'s own real body at the given pairs' endpoints: ladder (largest rung of
    LADDER + 0.0021 at which the pose is free) and the oracle's clearance_lower_m at margin 0.001."""
    from aerial3dg_fail3_sample import LADDER, RealTester
    from aerial3dg_run import ROBOTS as RB
    doc = _load(F4 / "sample" / f"{a.region}_pairs.json")
    want = set(Path(a.ids_file).read_text().split())
    t = RealTester(doc["box_uv"], 120., .1)
    body = RB[a.robot]
    rungs = sorted(set(LADDER) | {.0021})
    out = []
    for p in doc["pairs"]:
        if p["pair_id"] not in want:
            continue
        rec = {"pair_id": p["pair_id"], "robot": a.robot}
        for e in ("start", "goal"):
            q = t.pose(p[f"{e}_uv"], body)
            ok, c, why = t.free(q, body, MARGIN)
            passed = [m for m in rungs if t.free(q, body, m)[0]]
            rec[e] = {"free_m001": ok, "clearance_lower_m001": c, "reason_m001": why,
                      "ladder_m": max(passed) if passed else 0., "free_m0021": .0021 in passed}
        out.append(rec)
        print(rec, flush=True)
    _dump(Path(a.out), {"region": a.region, "robot": a.robot, "rungs": rungs, "rows": out})


# ---------------------------------------------------------------------------------------------- classification
def domain_check(reg, robot, rows):
    """F3's ``aerial3dg_fail3_analyze.domain_check`` on F4's rows: max exported-pose vs swept-segment excess."""
    from aerial3dg_fail2_diag import excess
    from gmc.aerial3d.api import _densify
    from aerial3dg_run import MANIFEST, QCONFIG, ROBOTS as RB
    man = json.loads(MANIFEST.read_text())
    origin = np.asarray(man["frame"]["origin_world_m"], float)
    R = np.asarray(man["frame"]["world_to_route"], float)
    box = REGIONS[reg]["box"]
    lo, hi = np.array([*box[:2], 0.]), np.array([*box[2:], 2.43])
    body = RB[robot]
    half = np.array([body.radius_m, body.radius_m, body.half_height_m])
    out = {}
    for r in rows:
        if r["reason"] != "shared_replay_failed" or not r.get("route_polyline"):
            continue
        dens = _densify(np.asarray(r["route_polyline"], float) @ R + origin, QCONFIG.export_max_segment_m)
        pose = max(excess(p - half, p + half, origin, R, lo, hi) for p in dens)
        swept = max(excess(np.minimum(x, y) - half, np.maximum(x, y) + half, origin, R, lo, hi)
                    for x, y in zip(dens[:-1], dens[1:]))
        out[r["pair_id"]] = {"max_pose_excess_m": pose, "max_swept_excess_m": swept}
    return out


def classify(r, p, robot, kin, b0, wit, epc):
    """F3's ``classify_row``; for the sweeper's ``*_not_certified_free`` rows the endpoint ladder is the sweeper's
    own (``epclear``), never the cylinder's (module docstring).  No sweeper ladder -> EP-UNVERIFIED."""
    ev = {**p["evidence"], "clearance_m": p["clearance_m"]["cylinder"]}
    if robot == "sweeper" and r["status"] == "UNKNOWN" and r["reason"].endswith("_not_certified_free"):
        end = "start" if r["reason"].startswith("start") else "goal"
        if epc is None:
            return "EP-UNVERIFIED"
        lad = epc[end]["ladder_m"]
        if lad < MARGIN + BUFFER - 1e-9:
            return "EP-TOL"
        if not epc[end]["free_m0021"]:
            return "EP-TOL"            # clearance in (2.0, 2.1] mm
        return "EP-GENUINE"
    if robot == "cylinder" and r["status"] == "UNKNOWN" and r["reason"].endswith("_not_certified_free") and epc:
        end = "start" if r["reason"].startswith("start") else "goal"
        wit = {**(wit or {}), f"{end}_free_at_margin": epc[end]["free_m0021"]}
    return classify_row(r, ev, kin, b0, wit)


def _by_pid(path):
    d = _load(path)
    return {r["pair_id"]: r for r in (d or {}).get("rows", [])}


def load_all():
    pairs, rows = {}, []
    for reg in REGIONS:
        pairs.update({p["pair_id"]: p for p in _load(F4 / "sample" / f"{reg}_pairs.json")["pairs"]})
    for reg in REGIONS:
        for robot in ROBOTS:
            d = F4 / "diag" / reg / robot
            g = gmc_rows(reg, robot)
            kin, b0 = {}, {}
            for f in sorted(d.glob("kin*.json")):             # probe shards: kin.json / kin_0.json ...
                kin.update(_by_pid(f))
            for f in sorted(d.glob("bufzero*.json")):
                b0.update(_by_pid(f))
            wit = _by_pid(d / "witness.json")
            epc = _by_pid(d / "epclear.json")
            rq = {}
            for f in sorted(d.glob("requery_*.json")):
                rq.update(_by_pid(f))
            dom = domain_check(reg, robot, g.values())
            _dump(d / "domain.json", dom)
            for pid, x in dom.items():          # EXPORT-DOMAIN only when no exported pose leaves the prism
                if pid in kin and x["max_pose_excess_m"] > 1e-9:
                    kin[pid] = {**kin[pid], "original": {**kin[pid].get("original", {}),
                                                         "geometry_reason": "pose_exits_domain"}}
            for r in g.values():
                pid = r["pair_id"]
                c = classify(r, pairs[pid], robot, kin.get(pid), b0.get(pid), wit.get(pid), epc.get(pid))
                rows.append({"region": reg, "robot": robot, "r": r, "p": pairs[pid], "class": c,
                             "kin": kin.get(pid), "b0": b0.get(pid), "wit": wit.get(pid), "epc": epc.get(pid),
                             "rq": rq.get(pid), "dom": dom.get(pid)})
    return pairs, rows


def fail_record(x):
    r, p, e = x["r"], x["p"], x["p"]["evidence"]
    rq, kin, b0, epc = x["rq"] or {}, x["kin"] or {}, x["b0"] or {}, x["epc"] or {}
    reg = x["region"]
    return {"robot": x["robot"], "pair_id": r["pair_id"], "region": reg, "stream": p["stream"], "draw": p["draw"],
            "gmc_status": r["status"], "gmc_reason": r["reason"], "class": x["class"], "group": group_of(x["class"]),
            "gmc_wall_s": round(r["outer_wall_s"], 2), "own_verification": r.get("own_verification"),
            "dist_m": round(p["dist_m"], 3), "len_ratio": round(e["len_ratio"], 4),
            "lateral_mm": round(e["lateral_m"] * 1e3, 2), "clear3d_mm": round(e["clear3d_m"] * 1e3, 2),
            "vertical_mm": None if e["vertical_m"] is None else round(e["vertical_m"] * 1e3, 3),
            "start_clear_mm": round((p["clearance_m"]["cylinder"]["start"] or 0) * 1e3, 3),
            "goal_clear_mm": round((p["clearance_m"]["cylinder"]["goal"] or 0) * 1e3, 3),
            "start_ladder_mm": e["start_clear3d_m"] * 1e3, "goal_ladder_mm": e["goal_clear3d_m"] * 1e3,
            "own_body_ep_ladder_mm": None if not epc else
            min(epc["start"]["ladder_m"], epc["goal"]["ladder_m"]) * 1e3,
            "kin_both_floors_passed": (kin.get("both_floors") or {}).get("passed") if kin else None,
            "replay_geometry": (kin.get("original") or {}).get("geometry_reason") if kin else None,
            "swept_excess_mm": None if not x["dom"] else round(x["dom"]["max_swept_excess_m"] * 1e3, 3),
            "pose_excess_mm": None if not x["dom"] else round(x["dom"]["max_pose_excess_m"] * 1e3, 3),
            "buffer0": b0.get("buffer0"),
            "requery_now": rq.get("now"),
            # TIMEOUT rows are re-queried with a 300 s limit: reproduced = the re-query again needs >= 120 s
            "reproduced": (rq["requery_wall_s"] >= 120. - 1e-6) if r["status"] == "TIMEOUT" and rq else rq.get("reproduced"),
            "requery_wall_s": None if rq.get("requery_wall_s") is None else round(rq["requery_wall_s"], 1),
            "verdict_only": rq.get("verdict_only"),
            "verdict_only_wall_s": None if rq.get("verdict_only_wall_s") is None else round(rq["verdict_only_wall_s"], 2),
            "witness_m0021": (x["wit"] or {}).get("astar_outcome"),
            "astar_route_file": f"gmc/results/aerial3dg/f4/sample/{reg}_pairs.json#{r['pair_id']}",
            "cut_min_xy_dist_to_astar_m": rq.get("astar_route_min_xy_dist_to_cut_gaussian_m"),
            "probe_files": f"gmc/results/aerial3dg/f4/diag/{reg}/{x['robot']}/"}


def wilson(k, n, z=1.96):
    if n == 0:
        return (0., 0.)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0., c - h), min(1., c + h))


def band_table(rows, key, bands, label):
    out = []
    for b in bands:
        lo, hi = b[0], b[1]
        name = b[2] if len(b) > 2 else f"[{lo:g}, {hi:g})"
        sel = [x for x in rows if lo <= key(x) < hi]
        n = len(sel)
        cnt = collections.Counter(group_of(x["class"]) for x in sel)
        k = n - cnt["reachable"]
        out.append({"band": name, "lo": lo, "hi": hi, "n": n, "fail": k, "rate": k / n if n else None,
                    "ci95": wilson(k, n), **{g: cnt[g] for g in ("genuine", "tolerance", "export", "unverified")},
                    "genuine_rate": cnt["genuine"] / n if n else None,
                    "genuine_ci95": wilson(cnt["genuine"], n)})
    return {"by": label, "bands": out}


def ep_min(x):
    e = x["p"]["evidence"]
    if x["robot"] == "sweeper" and x["epc"]:
        return min(x["epc"]["start"]["ladder_m"], x["epc"]["goal"]["ladder_m"])
    return min(e["start_clear3d_m"], e["goal_clear3d_m"])


def cmd_summary(a):
    pairs, rows = load_all()
    fails = [fail_record(x) for x in rows if x["class"] != "REACHABLE"]
    with open(F4 / "failures.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(fail_record(rows[0]).keys()))
        w.writeheader()
        w.writerows(sorted(fails, key=lambda r: (r["robot"], r["pair_id"])))
    summ = {"n_pairs": len(pairs), "regions": list(REGIONS), "per_robot": {}, "per_region_robot": {},
            "bands": {}, "soundness_flags": [], "timeout_probe": {}}
    for robot in ROBOTS:
        rr = [x for x in rows if x["robot"] == robot]
        cls = collections.Counter(x["class"] for x in rr)
        grp = collections.Counter(group_of(x["class"]) for x in rr)
        summ["per_robot"][robot] = {"n": len(rr), "reachable": cls["REACHABLE"], "failed": len(rr) - cls["REACHABLE"],
                                    "fail_rate": (len(rr) - cls["REACHABLE"]) / len(rr), "class": dict(cls),
                                    "group": dict(grp),
                                    "status": dict(collections.Counter(x["r"]["status"] for x in rr)),
                                    "status_reason": dict(collections.Counter(
                                        f"{x['r']['status']}:{x['r']['reason']}" for x in rr))}
        for reg in REGIONS:
            q = [x for x in rr if x["region"] == reg]
            c = collections.Counter(x["class"] for x in q)
            w = np.array([x["r"]["outer_wall_s"] for x in q])
            summ["per_region_robot"][f"{reg}/{robot}"] = {
                "n": len(q), "reachable": c["REACHABLE"], "class": dict(c),
                "group": dict(collections.Counter(group_of(x["class"]) for x in q)),
                "query_wall_s": {"sum": float(w.sum()), "mean": float(w.mean()), "p50": float(np.median(w)),
                                 "p90": float(np.percentile(w, 90)), "max": float(w.max())},
                "compile_id": sorted({x["r"]["compile_id"] for x in q})}
        summ["bands"][robot] = {
            "lateral": band_table(rr, lambda x: x["p"]["evidence"]["lateral_m"], LAT_BANDS, "route lateral clearance (ladder rung passed)"),
            "detour": band_table(rr, lambda x: x["p"]["evidence"]["len_ratio"], RATIO_BANDS, "A* route length / straight distance"),
            "endpoint": band_table(rr, ep_min, EP_BANDS, "min endpoint clearance ladder (own body for sweeper EP rows)")}
        for reg in REGIONS:
            q = [x for x in rr if x["region"] == reg]
            summ["bands"][robot][f"detour_{reg}"] = band_table(q, lambda x: x["p"]["evidence"]["len_ratio"],
                                                               RATIO_BANDS, f"detour, {reg}")
            summ["bands"][robot][f"lateral_{reg}"] = band_table(q, lambda x: x["p"]["evidence"]["lateral_m"],
                                                                LAT_BANDS, f"lateral, {reg}")
    for x in rows:
        if x["r"]["status"] == "UNREACHABLE":
            rq = x["rq"] or {}
            summ["soundness_flags"].append({"robot": x["robot"], "pair_id": x["r"]["pair_id"],
                                            "region": x["region"], "reason": x["r"]["reason"],
                                            "certificate_kind": x["r"].get("certificate_kind"),
                                            "cut_distinct_pairs": x["r"].get("cut_distinct_pairs"),
                                            "requery": rq.get("now"),
                                            "cut_gaussians": rq.get("cut_gaussians"),
                                            "astar_route_min_xy_dist_to_cut_gaussian_m":
                                                rq.get("astar_route_min_xy_dist_to_cut_gaussian_m"),
                                            "astar_route_min_xy_dist_to_cut_leaf_centre_m":
                                                rq.get("astar_route_min_xy_dist_to_cut_leaf_centre_m")})
    tp = [x for x in rows if x["r"]["status"] == "TIMEOUT" and x["rq"]]
    if tp:
        fin = [x for x in tp if not x["rq"]["now"].startswith("TIMEOUT")]
        vo = [x["rq"].get("verdict_only_wall_s") for x in tp if x["rq"].get("verdict_only_wall_s") is not None]
        summ["timeout_probe"] = {
            "n_probed": len(tp), "requery_timeout_s": tp[0]["rq"].get("requery_timeout_s"),
            "requery_outcome": dict(collections.Counter(x["rq"]["now"].split(":")[0] for x in tp)),
            "finished_wall_s": [round(x["rq"]["requery_wall_s"], 1) for x in fin],
            "verdict_only": dict(collections.Counter((x["rq"].get("verdict_only") or "?").split(":")[0] for x in tp)),
            "verdict_only_wall_s": {"min": min(vo), "max": max(vo), "median": float(np.median(vo))} if vo else None,
            "stage_share": _stage_share(tp)}
    summ["probes"] = {robot: {k: sum(1 for x in rows if x["robot"] == robot and x[k] is not None)
                              for k in ("kin", "b0", "rq", "wit", "epc")} for robot in ROBOTS}
    summ["buffer0"] = {robot: dict(collections.Counter((x["b0"]["buffer0"] or "?").split(":")[0] for x in rows
                                                       if x["robot"] == robot and x["b0"]))
                       for robot in ROBOTS}
    summ["replay_probe"] = {robot: dict(collections.Counter(
        ("floors_clear" if (x["kin"].get("both_floors") or {}).get("passed") else
         (x["kin"].get("original") or {}).get("geometry_reason") or "other")
        for x in rows if x["robot"] == robot and x["kin"])) for robot in ROBOTS}
    summ["reproduced"] = {robot: dict(collections.Counter(str(fail_record(x)["reproduced"]) for x in rows
                                                          if x["robot"] == robot and x["rq"]))
                          for robot in ROBOTS}
    _dump(F4 / "summary.json", summ)
    print_summary(summ)


def _stage_share(tp):
    tot = collections.Counter()
    for x in tp:
        for k, v in (x["rq"].get("full_stage_s") or {}).items():
            tot[k] += v
    s = sum(tot.values())
    return {k: round(v / s, 3) for k, v in tot.most_common()} if s else None


def print_summary(s):
    print("| robot | n | REACHABLE | failed (rate) | genuine | by-tolerance | export | unverified | classes |")
    print("|---|---|---|---|---|---|---|---|---|")
    for robot, v in s["per_robot"].items():
        g = v["group"]
        cl = ", ".join(f"{c} {n}" for c, n in sorted(v["class"].items(), key=lambda t: -t[1]) if c != "REACHABLE")
        print(f"| {robot} | {v['n']} | {v['reachable']} | {v['failed']} ({v['fail_rate']:.2%}) | {g.get('genuine', 0)} "
              f"| {g.get('tolerance', 0)} | {g.get('export', 0)} | {g.get('unverified', 0)} | {cl} |")
    print()
    print("| region / robot | n | REACHABLE | genuine | by-tolerance | export | unverified | classes | wall mean / p90 / max (s) |")
    print("|---|---|---|---|---|---|---|---|---|")
    for k, v in s["per_region_robot"].items():
        g = v["group"]
        cl = ", ".join(f"{c} {n}" for c, n in sorted(v["class"].items(), key=lambda t: -t[1]) if c != "REACHABLE")
        w = v["query_wall_s"]
        print(f"| {k} | {v['n']} | {v['reachable']} | {g.get('genuine', 0)} | {g.get('tolerance', 0)} | "
              f"{g.get('export', 0)} | {g.get('unverified', 0)} | {cl} | {w['mean']:.1f} / {w['p90']:.1f} / {w['max']:.0f} |")
    for robot, b in s["bands"].items():
        for key in ("lateral", "detour", "endpoint"):
            t = b[key]
            print(f"\n{robot}: failure rate vs {t['by']}")
            print("| band | n | failed | rate (95 % CI) | genuine | by-tolerance | export | unverified |")
            print("|---|---|---|---|---|---|---|---|")
            for r in t["bands"]:
                if not r["n"]:
                    continue
                print(f"| {r['band']} | {r['n']} | {r['fail']} | {r['rate']:.1%} ({r['ci95'][0]:.1%}-{r['ci95'][1]:.1%}) "
                      f"| {r['genuine']} | {r['tolerance']} | {r['export']} | {r['unverified']} |")
    print("\nsoundness flags:", len(s["soundness_flags"]))
    print("timeout probe:", json.dumps(s.get("timeout_probe"), default=float)[:800])
    print("buffer0:", s["buffer0"], "\nreplay probe:", s["replay_probe"], "\nreproduced:", s["reproduced"],
          "\nprobes:", s["probes"])


def cmd_gmctask(a):
    """``aerial3dg_batch.run_task`` (G2's runner, unchanged) with ``query`` wrapped so that one pair cannot kill a task:
      * an exception raised while G2's 120 s SIGALRM was being handled (``_Timeout`` as ``__cause__``/``__context__``;
        numpy's ``norm`` turns it into ``TypeError: 'axis' must be ...``) -> re-raised as ``_Timeout`` -> TIMEOUT row
      * any other exception from the method -> an ERROR row (reason = exception type, message, file:line)
    Every such event is listed in ``task_NN.notes.json``.  Resumes from the task JSONL like ``run_task``."""
    import time as _t
    import traceback
    import aerial3dg_batch as B
    from aerial3dg_run import host as _host
    doc = _load(F4 / "sample" / f"{a.region}_pairs.json")
    pairs = [doc["pairs"][i] for i in B.task_slice(len(doc["pairs"]), a.n_tasks, a.task)]
    real = B.query
    notes = []

    def safe(compiled, s, g, *, config, call_id):
        w0 = _t.perf_counter()
        try:
            return real(compiled, s, g, config=config, call_id=call_id)
        except B._Timeout:
            raise
        except Exception as exc:
            tb = traceback.extract_tb(exc.__traceback__)[-1]
            where = f"{type(exc).__name__}: {exc} @ {tb.filename.split('/')[-1]}:{tb.lineno}"
            if isinstance(exc.__cause__, B._Timeout) or isinstance(exc.__context__, B._Timeout):
                notes.append({"pair_id": call_id, "event": "timeout_alarm_surfaced_as_exception", "exception": where,
                              "wall_s": _t.perf_counter() - w0})
                raise B._Timeout() from exc
            notes.append({"pair_id": call_id, "event": "method_exception", "exception": where,
                          "traceback": traceback.format_exc()[-3000:], "wall_s": _t.perf_counter() - w0})
            return {"status": "ERROR", "reason": where, "compile_id": compiled.compile_id,
                    "timings": {"algorithm_wall_s": _t.perf_counter() - w0, "records": []}}
    B.query = safe
    out_dir = F4 / "gmc" / a.region / a.robot
    a3c = Path("outputs/aerial3dg/f3") / a.region / f"{a.robot}.a3c"
    print(f"{a.robot} task {a.task}/{a.n_tasks}: pairs {pairs[0]['index']}..{pairs[-1]['index']} (gmctask)", flush=True)
    summary = B.run_task(a3c, pairs, out_dir / f"task_{a.task:02d}.jsonl", timeout_s=a.timeout,
                         manifest=json.loads(B.MANIFEST.read_text()) if hasattr(B, "MANIFEST") else None)
    summary.update(robot=a.robot, task=a.task, n_tasks=a.n_tasks, pair_index_range=[pairs[0]["index"], pairs[-1]["index"]],
                   pairs_file=str(F4 / "sample" / f"{a.region}_pairs.json"), host=_host(), timeout_s=a.timeout,
                   runner="aerial3dg_fail3_f4.py gmctask (run_task + exception-safe query)", events=len(notes))
    _dump(out_dir / f"task_{a.task:02d}.summary.json", summary)
    prev = _load(out_dir / f"task_{a.task:02d}.notes.json") or {"events": []}
    _dump(out_dir / f"task_{a.task:02d}.notes.json", {"events": prev["events"] + notes})
    print(json.dumps({k: summary[k] for k in ("complete", "status_counts", "answered_this_run")}), notes, flush=True)


# ---------------------------------------------------------------------------------------------- plots
def cmd_plots(a):
    import matplotlib.pyplot as plt
    s = _load(F4 / "summary.json")
    (F4 / "fig").mkdir(parents=True, exist_ok=True)
    # categorical slots 1-4 of the dataviz reference palette, fixed order (validated: validate_palette.js, light)
    groups = [("genuine", "#2a78d6", "genuine (method)"), ("tolerance", "#eb6834", "by-tolerance"),
              ("export", "#1baf7a", "export artefact"), ("unverified", "#eda100", "unverified")]
    for key, xlabel in (("lateral", "route lateral clearance band (mm; largest ladder rung passed)"),
                        ("detour", "detour ratio band (A* route length / straight distance)"),
                        ("endpoint", "min endpoint clearance band (mm)")):
        fig, axes = plt.subplots(1, 2, figsize=(12, 4.2), sharey=True)
        for ax, robot in zip(axes, ROBOTS):
            bands = [b for b in s["bands"][robot][key]["bands"] if b["n"]]
            x = np.arange(len(bands))
            bottom = np.zeros(len(bands))
            for g, col, lab in groups:
                v = np.array([b[g] / b["n"] for b in bands]) * 100
                if v.any():
                    ax.bar(x, v, bottom=bottom, color=col, label=lab, width=.7, edgecolor="white", linewidth=1.5)
                bottom += v
            lo = np.array([b["ci95"][0] for b in bands]) * 100
            hi = np.array([b["ci95"][1] for b in bands]) * 100
            ax.errorbar(x, bottom, yerr=[bottom - lo, hi - bottom], fmt="none", ecolor="#333", capsize=3, lw=.8)
            for xi, b, top in zip(x, bands, hi):
                ax.text(xi, top + 1., f"{b['fail']}/{b['n']}", ha="center", va="bottom", fontsize=7, color="#333")
            ax.set_xticks(x, [b["band"] for b in bands], fontsize=8)
            ax.set_title(f"{robot}: {s['per_robot'][robot]['failed']}/{s['per_robot'][robot]['n']} failed", fontsize=10)
            ax.set_xlabel(xlabel, fontsize=8)
            ax.spines[["top", "right"]].set_visible(False)
            ax.grid(axis="y", color="#ddd", lw=.5)
            ax.set_axisbelow(True)
        axes[0].set_ylabel("failure rate (% of confirmed pairs in band)")
        hl = {}
        for ax in axes:
            for h, lab in zip(*ax.get_legend_handles_labels()):
                hl.setdefault(lab, h)
        order = [lab for _, _, lab in groups if lab in hl]
        fig.legend([hl[k] for k in order], order, fontsize=8, frameon=False, loc="upper center", bbox_to_anchor=(.5, .89),
                   ncol=len(order))
        fig.suptitle(f"GMC failures on {s['n_pairs']} confirmed-reachable pairs vs {key}\n(bars: share by group; "
                     f"whiskers: Wilson 95 % CI of the total)", fontsize=9)
        fig.tight_layout(rect=(0, 0, 1, .83))
        fig.savefig(F4 / "fig" / f"rate_vs_{key}.png", dpi=130)
        plt.close(fig)
    # per-region detour curves (cylinder): genuine rate and total rate
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.2), sharey=False)
    cols = {"WWEST": "#2a78d6", "GAPW1": "#eb6834", "S": "#1baf7a"}
    for ax, (what, ttl) in zip(axes, (("rate", "all failures"), ("genuine_rate", "genuine failures only"))):
        for reg in REGIONS:
            bands = s["bands"]["cylinder"][f"detour_{reg}"]["bands"]
            pts = [(i, b[what] * 100, b["n"]) for i, b in enumerate(bands) if b["n"] >= 10]
            if pts:
                ax.plot([p[0] for p in pts], [p[1] for p in pts], "o-", color=cols.get(reg), label=reg, lw=1.5, ms=4)
        ax.set_xticks(range(len(RATIO_BANDS)), [f"[{lo:g},{hi:g})" if hi < 99 else f">={lo:g}" for lo, hi in RATIO_BANDS],
                      fontsize=8)
        ax.set_xlabel("detour ratio band (bands with n >= 10)", fontsize=8)
        ax.set_ylabel(f"cylinder {ttl} (%)")
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="y", color="#ddd", lw=.5)
        ax.legend(fontsize=8, frameon=False)
    fig.suptitle("Cylinder failure rate vs detour ratio, per region", fontsize=10)
    fig.tight_layout()
    fig.savefig(F4 / "fig" / "cylinder_rate_vs_detour_by_region.png", dpi=130)
    plt.close(fig)
    print(sorted(str(p) for p in (F4 / "fig").glob("*.png")))


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("collect")
    c.add_argument("--regions", nargs="+", default=list(REGIONS))
    sub.add_parser("merge")
    sub.add_parser("check")
    pr = sub.add_parser("prep")
    pr.add_argument("--n-timeout", type=int, default=50)
    pr.add_argument("--shard", type=int, default=10)
    e = sub.add_parser("epclear")
    e.add_argument("--region", required=True)
    e.add_argument("--robot", required=True)
    e.add_argument("--ids-file", required=True)
    e.add_argument("--out", required=True)
    gt = sub.add_parser("gmctask")
    gt.add_argument("--region", required=True)
    gt.add_argument("--robot", required=True)
    gt.add_argument("--n-tasks", type=int, required=True)
    gt.add_argument("--task", type=int, required=True)
    gt.add_argument("--timeout", type=float, default=120.)
    sub.add_parser("summary")
    sub.add_parser("plots")
    a = p.parse_args(argv)
    {"collect": cmd_collect, "merge": cmd_merge, "check": cmd_check, "prep": cmd_prep, "epclear": cmd_epclear,
     "summary": cmd_summary, "plots": cmd_plots, "gmctask": cmd_gmctask}[a.cmd](a)


if __name__ == "__main__":
    main()
