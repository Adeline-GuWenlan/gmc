"""Assemble gmc/results/uavconn/c1_handoff.json from C1's committed artefacts (run from gmc/).

Every number is read from a result file: probe JSONs (results/uavconn/probe), the
pytest JUnit XML, the synthetic summaries, and job accounting passed on the command
line (``--job NAME=JOBID,ELAPSED,MAXRSS``, from ``sacct``).
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import glob
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from gmc.aerial3d.api import CompileConfig, QueryConfig


def pick(d, *keys):
    for k in keys:
        d = (d or {}).get(k)
    return d


def probe_row(path: Path) -> dict:
    d = json.loads(path.read_text())
    c = d["compile"]
    row = {"file": str(path), "spec": d["spec"], "slurm_job_id": d["host"]["slurm_job_id"],
           "node": d["host"]["node"], "archive_sha256": d["archive_sha256"],
           "archive_load_and_hash_s": d["preparation"]["archive_load_and_hash_s"],
           "scene_build_and_crop_s": d["preparation"]["scene_build_and_crop_s"],
           "cropped_gaussians": d["crop"]["selected_supports"],
           "candidate_pairs": c["pairs"]["candidate_pairs"], "pruned_pairs": c["pairs"]["pruned_pairs"],
           "compile_wall_s": c["timings"]["compile_wall_s"],
           "compile_stages": [{"stage": r["stage"], "seconds": r["seconds"],
                               "peak_rss_mb_after": r["sizes"]["peak_rss_mb"]}
                              for r in c["timings"]["records"]],
           "octree_nodes": c["octree"]["nodes"], "leaves": c["octree"]["leaves_by_status"],
           "leaf_volume_m3": c["octree"]["volume_m3_by_status"],
           "cells": c["cells"]["cells"], "support_plane_cells": c["cells"]["support_plane_cells"],
           "box_cells": c["cells"]["box_cells"], "portals": c["cells"]["portals"],
           "possible_components": c["possible_components"],
           "traceability": c["traceability"], "free_boundary_traceability": c["free_boundary_traceability"],
           "sandwich_audit_passed": c["audit"]["passed"],
           "endpoints_same_possible_component": d["endpoints"]["same_possible_component"]}
    q = d.get("query")
    if q:
        row["query"] = {"status": q["status"], "reason": q["reason"],
                        "algorithm_wall_s": q["timings"]["algorithm_wall_s"],
                        "stages": {r["stage"]: r["seconds"] for r in q["timings"]["records"]},
                        "clearance_lower_m": q["clearance_lower_m"],
                        "metrics": q["metrics"],
                        "certificate": {k: v for k, v in (q["certificate"] or {}).items()
                                        if k not in ("cut_pair_ids", "cut_leaf_centres_plan", "claim")} or None}
    return row


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("results/uavconn/c1_handoff.json"))
    ap.add_argument("--junit", type=Path, required=True)
    ap.add_argument("--job", action="append", default=[], help="NAME=JOBID,ELAPSED,MAXRSS")
    ap.add_argument("--extra", type=Path, help="JSON with commits/limitations/predictions/notes")
    a = ap.parse_args(argv)
    ts = ET.parse(a.junit).getroot()
    ts = ts if ts.tag == "testsuite" else ts.find("testsuite")
    per_file = {}
    for tc in ts.iter("testcase"):
        f = tc.get("classname").split(".")[-1]
        per_file[f] = per_file.get(f, 0) + 1
    rows = []
    for f in sorted(glob.glob("results/uavconn/synthetic/summary_*.json")):
        rows += json.loads(Path(f).read_text())["rows"]
    synth = []
    for r in rows:
        a3, lt = r["aerial3d"], r["lattice"]
        synth.append({"case": r["case"], "query": r["query"], "body": r["body"],
                      "aerial3d": {"status": a3["status"], "reason": a3["reason"],
                                   "compile_wall_s": a3["compile_wall_s"], "query_wall_s": a3["wall_s"],
                                   "length_m": pick(a3, "raw", "path_length_m"),
                                   "vertices": pick(a3, "raw", "n_vertices"),
                                   "total_turning_rad": pick(a3, "raw", "total_turning_rad"),
                                   "smoothed_jerk": pick(a3, "smoothing", "smoothed", "integrated_squared_jerk"),
                                   "smoothed_duration_s": pick(a3, "smoothing", "smoothed", "duration_s"),
                                   "replay_fresh_passed": pick(a3, "replay_fresh", "passed")},
                      "lattice": {"status": lt["status"], "reason": lt["reason"], "wall_s": lt["wall_s"],
                                  "expansions": lt.get("expansions"),
                                  "length_m": pick(lt, "raw", "path_length_m"),
                                  "vertices": pick(lt, "raw", "n_vertices"),
                                  "total_turning_rad": pick(lt, "raw", "total_turning_rad"),
                                  "smoothed_jerk": pick(lt, "smoothing", "smoothed", "integrated_squared_jerk"),
                                  "smoothed_duration_s": pick(lt, "smoothing", "smoothed", "duration_s"),
                                  "replay_fresh_passed": pick(lt, "replay_fresh", "passed")},
                      "verdicts_agree": r["verdicts_agree"]})
    jobs = {}
    for j in a.job:
        name, rest = j.split("=", 1)
        jid, elapsed, rss = rest.split(",", 2)
        jobs[name] = {"job_id": jid, "elapsed": elapsed, "max_rss": rss}
    doc = {"schema": "uavconn.c1_handoff.v1",
           "compile_config_defaults": asdict(CompileConfig()),
           "query_config_defaults": asdict(QueryConfig()),
           "probes": {Path(p).stem: probe_row(Path(p)) for p in sorted(glob.glob("results/uavconn/probe/*.json"))},
           "job_accounting": jobs,
           "unit_tests": {"junit": str(a.junit), "tests": int(ts.get("tests")), "failures": int(ts.get("failures")),
                          "errors": int(ts.get("errors")), "skipped": int(ts.get("skipped") or 0),
                          "seconds": float(ts.get("time")), "per_file": per_file},
           "synthetic_comparison": synth}
    if a.extra:
        doc.update(json.loads(a.extra.read_text()))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(doc, indent=1, allow_nan=False, default=float) + "\n")
    print(f"wrote {a.out}: {len(doc['probes'])} probes, {len(synth)} synthetic rows, "
          f"{doc['unit_tests']['tests']} tests / {doc['unit_tests']['failures']} failures")


if __name__ == "__main__":
    main()
