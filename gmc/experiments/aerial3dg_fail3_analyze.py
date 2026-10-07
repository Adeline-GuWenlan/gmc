"""F3 Task 1: merge the per-region pilot (uniform) and targeted runs into tables (inline: json/csv only).

Run from ``gmc/`` with ``PYTHONPATH=src:experiments``.

Layout per region R and run kind K in {pilot, targeted}:
  results/aerial3dg/f3/sample/R/K/stream_*.jsonl          sampler streams        -> collect -> R/K_pairs.json
  results/aerial3dg/f3/gmc/R/K/{cylinder,sweeper}/task_00.jsonl   GMC rows (one persisted compile per robot / region)
  results/aerial3dg/f3/diag/R/K/kin_{robot}.json           1 ms floor probe on shared_replay_failed rows
  results/aerial3dg/f3/diag/R/K/bufzero_{robot}.json       buffer-0 endpoint probe on *_not_certified_free rows
  results/aerial3dg/f3/diag/R/K/requery_{robot}.json       re-query (reproduction + certificate) of every failure

``summary`` writes results/aerial3dg/f3/pilot_summary.json and results/aerial3dg/f3/failures_pilot.csv.
"""
from __future__ import annotations

import argparse
import collections
import csv
import json
from pathlib import Path

import numpy as np

from aerial3dg_fail2_sample import read_jsonl
from aerial3dg_fail3_probe import EXPORT, GENUINE, TOLERANCE, classify_row
from aerial3dg_run import _dump

F3 = Path("results/aerial3dg/f3")


def _load(p):
    p = Path(p)
    return json.loads(p.read_text()) if p.exists() else None


def domain_check(reg, kind, robot, box):
    """Per shared_replay_failed row (no archive): F2's segment test (``aerial3dg_fail2_diag.excess``) -- does any
    exported pose's own body AABB leave the prism (a real domain exit), or only the swept-segment world AABB (the
    replay's conservative coverage test)?"""
    from aerial3dg_fail2_diag import excess
    from gmc.aerial3d.api import _densify
    from aerial3dg_run import MANIFEST, QCONFIG, ROBOTS
    man = json.loads(MANIFEST.read_text())
    origin = np.asarray(man["frame"]["origin_world_m"], float)
    R = np.asarray(man["frame"]["world_to_route"], float)
    lo, hi = np.array([*box[:2], 0.]), np.array([*box[2:], 2.43])
    body = ROBOTS[robot]
    half = np.array([body.radius_m, body.radius_m, body.half_height_m])
    out = {}
    for t in sorted((F3 / "gmc" / reg / kind / robot).glob("task_*.jsonl")):
        for r in read_jsonl(t):
            if r["reason"] != "shared_replay_failed" or not r.get("route_polyline"):
                continue
            dens = _densify(np.asarray(r["route_polyline"], float) @ R + origin, QCONFIG.export_max_segment_m)
            pose = max(excess(p - half, p + half, origin, R, lo, hi) for p in dens)
            swept = max(excess(np.minimum(a, b) - half, np.maximum(a, b) + half, origin, R, lo, hi)
                        for a, b in zip(dens[:-1], dens[1:]))
            out[r["pair_id"]] = {"max_pose_excess_m": pose, "max_swept_excess_m": swept}
    return out


def region_kind(reg, kind):
    pairs = _load(F3 / "sample" / reg / f"{kind}_pairs.json")
    if pairs is None:
        return None
    ev = {p["pair_id"]: p for p in pairs["pairs"]}
    out = {"region": reg, "kind": kind, "box_uv": pairs["box_uv"], "n_pairs": pairs["n"],
           "funnel": pairs["funnel"], "distance_passing_candidates": pairs["distance_passing_candidates"],
           "robots": {}}
    fails = []
    for robot in ("cylinder", "sweeper"):
        gm = {}
        for t in sorted((F3 / "gmc" / reg / kind / robot).glob("task_*.jsonl")):
            gm.update({r["index"]: r for r in read_jsonl(t)})
        if not gm:
            continue
        d = F3 / "diag" / reg / kind
        kin = {r["pair_id"]: r for r in (_load(d / f"kin_{robot}.json") or {}).get("rows", [])}
        b0 = {r["pair_id"]: r for r in (_load(d / f"bufzero_{robot}.json") or {}).get("rows", [])}
        rq = {r["pair_id"]: r for r in (_load(d / f"requery_{robot}.json") or {}).get("rows", [])}
        wit = {r["pair_id"]: r for r in (_load(d / "witness.json") or {}).get("rows", [])}
        dom = domain_check(reg, kind, robot, pairs["box_uv"])
        _dump(d / f"domain_{robot}.json", dom)
        for pid, x in dom.items():          # EXPORT-DOMAIN only when no exported pose leaves the prism
            if pid in kin and x["max_pose_excess_m"] > 1e-9:
                kin[pid] = {**kin[pid], "original": {**kin[pid].get("original", {}), "geometry_reason": "pose_exits_domain"}}
        cls = collections.Counter()
        ids = {r["compile_id"] for r in gm.values()}
        for r in gm.values():
            p = ev[r["pair_id"]]
            e = {**p["evidence"], "clearance_m": p["clearance_m"]["cylinder"]}
            c = classify_row(r, e, kin.get(r["pair_id"]), b0.get(r["pair_id"]), wit.get(r["pair_id"]))
            cls[c] += 1
            if c != "REACHABLE":
                fails.append({"region": reg, "kind": kind, "robot": robot, "pair_id": r["pair_id"],
                              "status": r["status"], "reason": r["reason"], "class": c,
                              "group": "genuine" if c in GENUINE else "tolerance" if c in TOLERANCE else
                              "export" if c in EXPORT else "unverified",
                              "dist_m": round(p["dist_m"], 3), "len_ratio": round(e["len_ratio"], 3),
                              "clear3d_mm": e["clear3d_m"] * 1e3, "lateral_mm": e["lateral_m"] * 1e3,
                              "vertical_mm": None if e["vertical_m"] is None else round(e["vertical_m"] * 1e3, 3),
                              "start_clear_mm": round((p["clearance_m"]["cylinder"]["start"] or 0) * 1e3, 3),
                              "goal_clear_mm": round((p["clearance_m"]["cylinder"]["goal"] or 0) * 1e3, 3),
                              "kin_both_floors_passed": (kin.get(r["pair_id"]) or {}).get("both_floors", {}).get("passed"),
                              "replay_geometry": ((kin.get(r["pair_id"]) or {}).get("original") or {}).get("geometry_reason"),
                              "swept_excess_mm": None if r["pair_id"] not in dom else
                              round(dom[r["pair_id"]]["max_swept_excess_m"] * 1e3, 3),
                              "buffer0": (b0.get(r["pair_id"]) or {}).get("buffer0"),
                              "reproduced": (rq.get(r["pair_id"]) or {}).get("reproduced"),
                              "witness_m0021": (wit.get(r["pair_id"]) or {}).get("astar_outcome"),
                              "ep_clear3d_mm": None if e.get("start_clear3d_m") is None else
                              min(e["start_clear3d_m"], e["goal_clear3d_m"]) * 1e3,
                              "own_verification": r.get("own_verification"), "wall_s": round(r["outer_wall_s"], 2)})
        w = np.array([r["outer_wall_s"] for r in gm.values()])
        out["robots"][robot] = {"n": len(gm), "status": dict(collections.Counter(r["status"] for r in gm.values())),
                                "status_reason": dict(collections.Counter(f"{r['status']}:{r['reason']}"
                                                                          for r in gm.values())),
                                "class": dict(cls), "genuine": sum(v for k, v in cls.items() if k in GENUINE),
                                "tolerance": sum(v for k, v in cls.items() if k in TOLERANCE),
                                "export": sum(v for k, v in cls.items() if k in EXPORT),
                                "compile_ids": sorted(ids), "query_wall_s": {"sum": float(w.sum()),
                                                                            "mean": float(w.mean()),
                                                                            "max": float(w.max())}}
    return out, fails


def cmd_summary(a):
    res, fails = {}, []
    for reg in a.regions:
        for kind in ("pilot", "targeted", "tight"):
            r = region_kind(reg, kind)
            if r is None:
                continue
            res[f"{reg}/{kind}"], f = r
            fails += f
    _dump(F3 / "pilot_summary.json", res)
    if fails:
        with open(F3 / "failures_pilot.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(fails[0]))
            w.writeheader()
            w.writerows(fails)
    for k, v in res.items():
        print(k, v["n_pairs"], {r: (x["status"], x["class"]) for r, x in v["robots"].items()})


def cmd_cases(a):
    """Figure inputs per region: every genuine / unverified failure + up to ``--per-class`` examples of each other class
    (tightest lateral clearance first) -> results/aerial3dg/f3/cases/<REGION>.json (input of aerial3dg_fail3_fig.py)."""
    rows = list(csv.DictReader(open(F3 / "failures_pilot.csv")))
    by = collections.defaultdict(list)
    for r in rows:
        by[r["region"]].append(r)
    for reg, rr in by.items():
        pick, seen = [], collections.Counter()
        for r in sorted(rr, key=lambda r: float(r["lateral_mm"])):
            if r["group"] in ("genuine", "unverified") or seen[(r["robot"], r["class"])] < a.per_class:
                pick.append(r)
                seen[(r["robot"], r["class"])] += 1
        cases, box = [], None
        for r in pick:
            pairs = _load(F3 / "sample" / reg / f"{r['kind']}_pairs.json")
            box = pairs["box_uv"]
            p = next(x for x in pairs["pairs"] if x["pair_id"] == r["pair_id"])
            rq = {x["pair_id"]: x for x in (_load(F3 / "diag" / reg / r["kind"] / f"requery_{r['robot']}.json") or {})
                  .get("rows", [])}.get(r["pair_id"], {})
            note = (f"class {r['class']} ({r['group']}); A* ratio {r['len_ratio']}, clear3d {r['clear3d_mm']} mm, "
                    f"lateral {r['lateral_mm']} mm, ep clear3d {r.get('ep_clear3d_mm')} mm; buffer0 {r['buffer0'] or '-'}; "
                    f"replay {r['replay_geometry'] or '-'} swept excess {r['swept_excess_mm'] or '-'} mm")
            cases.append({"case_id": f"{reg}-{r['kind']}-{r['pair_id']}", "robot": r["robot"],
                          "start_uv": p["start_uv"], "goal_uv": p["goal_uv"], "astar_route_uv": p["astar_route_uv"],
                          "gmc_status": r["status"], "gmc_reason": r["reason"], "gmc_route_uv": rq.get("gmc_route_uv"),
                          "cut_gaussians": rq.get("cut_gaussians"), "note": note, "class": r["class"],
                          "group": r["group"]})
        _dump(F3 / "cases" / f"{reg}.json", {"region": reg, "box_uv": box, "cases": cases})
        print(reg, len(cases), collections.Counter(c["class"] for c in cases))


def cmd_tables(a):
    """Markdown tables for docs/aerial3dg_failures_f3.md from pilot_summary.json + probe/compile records."""
    res = _load(F3 / "pilot_summary.json")
    surv = _load(F3 / "region" / "survey.json")
    print("| region | box u0 v0 u1 v1 | supports | cyl free | compile cyl / sw (s) | cand. pairs cyl / sw | "
          "peak RSS (MB) |")
    print("|---|---|---|---|---|---|---|")
    for reg in a.regions:
        c = {r: _load(F3 / "probe" / reg / f"compile_{r}.json") for r in ("cylinder", "sweeper")}
        if not c["cylinder"]:
            continue
        sv = (surv["boxes"].get(reg) or {}) if surv else {}
        fr = (sv.get("robots", {}).get("cylinder", {}).get("status_share", {}).get("free"))
        cc, cs = c["cylinder"]["compile"], c["sweeper"]["compile"]
        print(f"| {reg} | {' '.join(f'{x:g}' for x in c['cylinder']['box_route']['lower'][:2] + c['cylinder']['box_route']['upper'][:2])} "
              f"| {c['cylinder']['crop']['selected_supports']:,} | {'' if fr is None else f'{fr:.0%}'} "
              f"| {cc['compile_wall_s']:.0f} / {cs['compile_wall_s']:.0f} "
              f"| {cc['pairs'].get('candidate_pairs', cc['pairs'].get('candidates', '?'))} / "
              f"{cs['pairs'].get('candidate_pairs', cs['pairs'].get('candidates', '?'))} "
              f"| {max(cc['peak_rss_mb'], cs['peak_rss_mb']):.0f} |")
    print()
    print("| region / run | n | distance-passing cand. | endpoint pass | pre-filter pass | A* ROUTE / NRM / NR / UNSURE "
          "| CPU s per accepted | len ratio p50 / p90 | lateral p10 / p50 (mm) |")
    print("|---|---|---|---|---|---|---|---|---|")
    for k, v in res.items():
        f = v["funnel"]
        sp = f["astar_split_on_prefilter_pass"]
        lr, la = f["accepted_len_ratio"], f["accepted_lateral_m"]
        print(f"| {k} | {v['n_pairs']} | {v['distance_passing_candidates']:,} | {f['endpoint_m001']['pass_rate']:.1%} "
              f"| {f['prefilter']['pass_rate']:.1%} | {sp['ROUTE']} / {sp['NO_ROUTE_MARGIN']} / {sp['NO_ROUTE']} / "
              f"{sp['UNSURE']} | {f['cpu_s_per_accepted']:.1f} | {lr['median']:.2f} / {lr['p90']:.2f} "
              f"| {la['p10'] * 1e3:g} / {la['median'] * 1e3:g} |")
    print()
    print("| region / run | robot | REACHABLE | genuine | by-tolerance | export | unverified | classes | query wall mean / max (s) |")
    print("|---|---|---|---|---|---|---|---|---|")
    for k, v in res.items():
        for r, x in v["robots"].items():
            cl = {c: n for c, n in x["class"].items() if c != "REACHABLE"}
            unv = x["n"] - x["class"].get("REACHABLE", 0) - x["genuine"] - x["tolerance"] - x["export"]
            print(f"| {k} | {r} | {x['class'].get('REACHABLE', 0)} / {x['n']} | {x['genuine']} | {x['tolerance']} "
                  f"| {x['export']} | {unv} | {', '.join(f'{c} {n}' for c, n in sorted(cl.items()))} "
                  f"| {x['query_wall_s']['mean']:.2f} / {x['query_wall_s']['max']:.1f} |")


SEEDS = {"S": 20261200, "G2MID": 20261400, "GAPW1": 20261500, "WWEST": 20261600, "E": 20261100, "NMID": 20261300}


def cmd_handoff(a):
    """results/aerial3dg/f3/f4_handoff.json: region quotas, sampler / GMC command lines, compiles (+SHA-256),
    measured per-stage costs, the projection for F4, the backup region, candidate cases for F5."""
    res = _load(F3 / "pilot_summary.json")
    regions, proj_total = {}, 0.
    quotas = dict(x.split(":") for x in a.quota)
    for reg in list(quotas) + [a.backup]:
        pil = res[f"{reg}/pilot"]
        box = pil["box_uv"]
        f = pil["funnel"]
        comp = {}
        for r in ("cylinder", "sweeper"):
            side = _load(Path("outputs/aerial3dg/f3") / reg / f"{r}.a3c.json")
            rec = _load(F3 / "probe" / reg / f"compile_{r}.json")
            comp[r] = {"a3c": f"gmc/outputs/aerial3dg/f3/{reg}/{r}.a3c", "compile_id": side["compile_id"],
                       "sha256": side["sha256"], "bytes": side["bytes"],
                       "compile_record": f"gmc/results/aerial3dg/f3/probe/{reg}/compile_{r}.json",
                       "compile_wall_s": rec["compile"]["compile_wall_s"],
                       "supports": rec["crop"]["selected_supports"]}
        q = int(quotas.get(reg, 0))
        samp = f["cpu_s_per_accepted"]
        gm = {r: pil["robots"][r]["query_wall_s"]["mean"] for r in ("cylinder", "sweeper")}
        per_pair = samp + gm["cylinder"] + gm["sweeper"]
        n_streams = max(1, int(np.ceil(q * samp / 3600 / 5.0))) if q else 0     # <= ~5 h per stream
        regions[reg] = {
            "role": "benchmark" if q else "backup", "quota": q, "box_uv": box,
            "box_route": {"lower": [box[0], box[1], 0.], "upper": [box[2], box[3], 2.43]},
            "seed_base": SEEDS[reg], "stream_layout": f"streams 101..{100 + n_streams} (seed = seed_base + 1000*stream; "
                                                     f"pilot used stream 1, targeted 21, tight 31: do not reuse)",
            "n_streams": n_streams, "quota_per_stream": int(np.ceil(q / n_streams)) if q else 0,
            "sampler": f"sbatch --job-name=a3f4_samp_{reg} --array=101-{100 + n_streams} --time=06:00:00 --mem=2200M "
                       f"gmc/hpc/aerial3dg/f3_py.sbatch experiments/aerial3dg_fail3_sample.py run --box "
                       f"{' '.join(f'{x:g}' for x in box)} --stream $SLURM_ARRAY_TASK_ID --seed {SEEDS[reg]} "
                       f"--quota {int(np.ceil(q / n_streams)) if q else 0} --max-hours 5.8 --lattice "
                       f"results/aerial3dg/f3/sample/{reg}/pilot/stream_01.lattice.npz --out-dir results/aerial3dg/f4/sample/{reg}"
                       + " (array task id == stream; collect with --n <quota>, prefix per region)",
            "gmc": f"gmc/hpc/aerial3dg/f3_gmc.sbatch <robot> {' '.join(f'{x:g}' for x in box)} <pairs.json> "
                   f"outputs/aerial3dg/f3/{reg}/<robot>.a3c results/aerial3dg/f4/gmc/{reg}/<robot> N_TASKS TASK",
            "compiles": comp,
            "measured_pilot": {"pairs": pil["n_pairs"], "distance_passing_candidates": pil["distance_passing_candidates"],
                               "funnel": {k: v for k, v in f.items() if isinstance(v, dict) and "pass_rate" in v},
                               "astar_split_on_prefilter_pass": f["astar_split_on_prefilter_pass"],
                               "sampling_cpu_s_per_accepted": samp, "gmc_query_wall_s_mean": gm,
                               "gmc_query_wall_s_max": {r: pil["robots"][r]["query_wall_s"]["max"]
                                                        for r in ("cylinder", "sweeper")},
                               "cylinder_classes": pil["robots"]["cylinder"]["class"],
                               "sweeper_classes": pil["robots"]["sweeper"]["class"]},
            "projection_cpu_h": {"per_pair_s": per_pair, "sampling": q * samp / 3600,
                                 "gmc_cylinder": q * gm["cylinder"] / 3600, "gmc_sweeper": q * gm["sweeper"] / 3600,
                                 "total": q * per_pair / 3600}}
        proj_total += q * per_pair / 3600
    cases = []
    for p in sorted((F3 / "cases").glob("*.json")):
        for c in _load(p)["cases"]:
            cases.append({"case_id": c["case_id"], "robot": c["robot"], "class": c["class"], "group": c["group"],
                          "figure": f"gmc/results/aerial3dg/f3/cases/fig/{c['case_id']}_{c['robot']}.png",
                          "pairs_file": f"gmc/results/aerial3dg/f3/sample/{c['case_id'].split('-')[0]}/"
                                        f"{c['case_id'].split('-')[1]}_pairs.json"})
    doc = {"schema": "aerial3dg_fail3.f4_handoff.v1", "n_total": sum(int(q) for q in quotas.values()),
           "regions": regions, "backup_region": a.backup, "choice_rationale": a.why,
           "confirmation_rule": "real cylinder, margin 0.001, 0.1 m lattice A* (LatticePlanner; pos tol 0, yaw tol 0.05, "
                                "500k expansions, 120 s wall) ROUTE + verify_path re-check; straight distance >= 3 m; "
                                "no robust inflation (experiments/aerial3dg_fail3_sample.py docstring)",
           "gmc_config": "aerial3dg_run.QCONFIG (G2's full QueryConfig, export_max_segment_m 0.20), timeout 120 s",
           "projection_cpu_h_total": proj_total, "budget_cpu_h": 200,
           "probes": {"shared_replay_failed": "experiments/aerial3dg_fail2_kin.py --full (1 ms floors) + "
                                              "aerial3dg_fail3_analyze.py domain check",
                      "not_certified_free": "experiments/aerial3dg_fail3_probe.py bufzero",
                      "all failures": "experiments/aerial3dg_fail3_probe.py requery (TIMEOUT rows: 300 s re-run + verdict-only mode; F4: probe a stratified <= 50 of them, ~6 min each)",
                      "genuine-vs-tolerance": "experiments/aerial3dg_fail3_sample.py witness (margin 0.0021)",
                      "sbatch": "gmc/hpc/aerial3dg/f3_probe.sbatch REGION KIND ROBOT"},
           "f5_candidate_cases": cases}
    _dump(F3 / "f4_handoff.json", doc)
    print(json.dumps({k: (v["quota"], round(v["projection_cpu_h"]["total"], 1)) for k, v in regions.items()}),
          "total", round(proj_total, 1))


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("summary")
    s.add_argument("--regions", nargs="+", default=["E", "S", "NMID", "G2MID", "GAPW1"])
    t = sub.add_parser("tables")
    t.add_argument("--regions", nargs="+", default=["E", "NMID", "S", "G2MID", "GAPW1", "WWEST"])
    h = sub.add_parser("handoff")
    h.add_argument("--quota", nargs="+", required=True, help="REGION:N ...")
    h.add_argument("--backup", required=True)
    h.add_argument("--why", required=True)
    c = sub.add_parser("cases")
    c.add_argument("--per-class", type=int, default=2)
    a = p.parse_args(argv)
    {"summary": cmd_summary, "cases": cmd_cases, "tables": cmd_tables, "handoff": cmd_handoff}[a.cmd](a)


if __name__ == "__main__":
    main()
