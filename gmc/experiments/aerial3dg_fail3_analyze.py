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
        cls = collections.Counter()
        ids = {r["compile_id"] for r in gm.values()}
        for r in gm.values():
            p = ev[r["pair_id"]]
            e = {**p["evidence"], "clearance_m": p["clearance_m"]["cylinder"]}
            c = classify_row(r, e, kin.get(r["pair_id"]), b0.get(r["pair_id"]))
            cls[c] += 1
            if c != "REACHABLE":
                fails.append({"region": reg, "kind": kind, "robot": robot, "pair_id": r["pair_id"],
                              "status": r["status"], "reason": r["reason"], "class": c,
                              "group": "genuine" if c in GENUINE else "tolerance" if c in TOLERANCE else
                              "export" if c in EXPORT else "unprobed",
                              "dist_m": round(p["dist_m"], 3), "len_ratio": round(e["len_ratio"], 3),
                              "clear3d_mm": e["clear3d_m"] * 1e3, "lateral_mm": e["lateral_m"] * 1e3,
                              "vertical_mm": None if e["vertical_m"] is None else round(e["vertical_m"] * 1e3, 3),
                              "start_clear_mm": round((p["clearance_m"]["cylinder"]["start"] or 0) * 1e3, 3),
                              "goal_clear_mm": round((p["clearance_m"]["cylinder"]["goal"] or 0) * 1e3, 3),
                              "kin_both_floors_passed": (kin.get(r["pair_id"]) or {}).get("both_floors", {}).get("passed"),
                              "buffer0": (b0.get(r["pair_id"]) or {}).get("buffer0"),
                              "reproduced": (rq.get(r["pair_id"]) or {}).get("reproduced"),
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
        for kind in ("pilot", "targeted"):
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


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("summary")
    s.add_argument("--regions", nargs="+", default=["E", "S", "NMID", "G2MID", "GAPW1"])
    a = p.parse_args(argv)
    {"summary": cmd_summary}[a.cmd](a)


if __name__ == "__main__":
    main()
