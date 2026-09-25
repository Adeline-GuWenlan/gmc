"""C1 scale probe: compile the aerial3d complex for a uav-lamp query box on the real archive.

Reads the archive only through the manifest-checked loader
(``gmc.gs3d.scene_uavlamp.load_uavlamp_derivative``), builds the SceneSpec with
the baseline's own ``uavlamp_query.build_scene`` (same crop, box, tau, level,
known space), compiles once, and reports per-stage time and peak RSS, pair counts,
leaves/cells/portals, and whether the spec's start and goal share a component.
With ``--query`` it also runs the single start+goal query once (including the
shared gs3d replay) to measure query cost.  No comparison, no rendering.

Usage (from gmc/):
    python experiments/aerial3d_probe.py configs/uavlamp_l2/M1_main_low_start.json \
        --uavlamp-root /scratch/wg2381/splathjb-uavlamp/gmc --output outputs/uavconn/probe/M1 [--query]
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import platform
import sys
from time import perf_counter

import numpy as np

from gmc.aerial3d.api import CompileConfig, QueryConfig, compile_complex, query
from gmc.aerial3d.cells import CellConfig
from gmc.aerial3d.octree import OctreeConfig
from gmc.aerial3d.query import cells_containing
from gmc.gs3d import scene_uavlamp as su
from gmc.gs3d.timing import PlanTiming


def endpoint_report(compiled, s_w, g_w) -> dict:
    tree, lab = compiled.tree, compiled.possible_label
    rep = {}
    for name, p_w in (("start", s_w), ("goal", g_w)):
        p = compiled.frame.to_plan(p_w)
        leaves = tree.locate(p)
        rep[name] = {"plan": p.tolist(), "leaves": [int(l) for l in leaves],
                     "leaf_status": [int(tree.status[l]) for l in leaves],
                     "possible_components": sorted({int(lab[l]) for l in leaves if lab[l] >= 0}),
                     "cells": cells_containing(compiled.cells.cells, p)[:16]}
    rep["same_possible_component"] = bool(set(rep["start"]["possible_components"])
                                          & set(rep["goal"]["possible_components"]))
    return rep


def probe(scene, body, start_w, goal_w, config: CompileConfig, *, run_query: bool,
          query_config: QueryConfig = QueryConfig()) -> dict:
    timer = PlanTiming()
    compiled = compile_complex(scene, body, config=config, timer=timer)
    doc = {"compile": compiled.summary(), "endpoints": endpoint_report(compiled, start_w, goal_w)}
    if run_query:
        r = query(compiled, start_w, goal_w, config=query_config)
        doc["query"] = {k: r[k] for k in ("status", "reason", "clearance_lower_m", "metrics", "timings",
                                          "verification", "certificate", "polyline_world", "diagnostics")}
        doc["query_result"] = r
    return doc


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("spec", type=Path)
    ap.add_argument("--uavlamp-root", type=Path, required=True,
                    help="gmc/ dir of the baseline worktree holding outputs/uavlamp/scene_v2 (read-only)")
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--min-cell", type=float, default=.05)
    ap.add_argument("--root-levels", type=int, default=3)
    ap.add_argument("--max-cells", type=int, default=5000)
    ap.add_argument("--cell-wall-s", type=float, default=None)
    ap.add_argument("--query", action="store_true")
    a = ap.parse_args(argv)
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from uavlamp_query import UAV, build_scene

    spec = json.loads(a.spec.read_text())
    archive, manifest = a.uavlamp_root / spec["archive"], a.uavlamp_root / spec["manifest"]
    a.output.mkdir(parents=True, exist_ok=True)
    t0 = perf_counter()
    full, doc = su.load_uavlamp_derivative(archive, manifest)
    load_s = perf_counter() - t0
    if doc["derivative"]["sha256"] != spec["archive_sha256"]:
        raise ValueError("archive hash differs from the spec")
    t0 = perf_counter()
    frame, scene, crop, dropped, extras = build_scene(spec, full, doc)
    del full
    build_s = perf_counter() - t0
    start_w = frame.to_world(spec["start_route"])
    goal_w = frame.to_world(spec["goal_route"])
    config = CompileConfig(margin_m=spec.get("margin_m", .05),
                           octree=OctreeConfig(min_cell_m=a.min_cell, root_levels=a.root_levels),
                           cells=CellConfig(max_cells=a.max_cells, max_wall_s=a.cell_wall_s))
    out = probe(scene, UAV, start_w, goal_w, config, run_query=a.query)
    out.update(spec=str(a.spec), archive_sha256=doc["derivative"]["sha256"],
               manifest_checked_loader="gmc.gs3d.scene_uavlamp.load_uavlamp_derivative",
               crop=crop, scene_variants_test_only={"dropped_roles": dropped, "extra_rows": extras},
               preparation={"archive_load_and_hash_s": load_s, "scene_build_and_crop_s": build_s},
               host={"node": platform.node(), "slurm_job_id": os.environ.get("SLURM_JOB_ID"),
                     "cpus": os.environ.get("SLURM_CPUS_PER_TASK"), "python": sys.version.split()[0]})
    (a.output / "probe.json").write_text(json.dumps(out, allow_nan=False, default=float) + "\n")
    brief = {"stages": [(r["stage"], round(r["seconds"], 2), round(r["sizes"]["peak_rss_mb"], 1))
                        for r in out["compile"]["timings"]["records"]],
             "compile_wall_s": out["compile"]["timings"]["compile_wall_s"],
             "pairs": out["compile"]["pairs"]["candidate_pairs"],
             "leaves": out["compile"]["octree"]["leaves_by_status"],
             "cells": {k: out["compile"]["cells"][k] for k in ("cells", "support_plane_cells", "box_cells", "portals")},
             "same_possible_component": out["endpoints"]["same_possible_component"],
             "start_cells": out["endpoints"]["start"]["cells"], "goal_cells": out["endpoints"]["goal"]["cells"]}
    if a.query:
        q = out["query"]
        brief["query"] = {"status": q["status"], "reason": q["reason"],
                          "algorithm_wall_s": q["timings"]["algorithm_wall_s"],
                          "stages": [(r["stage"], round(r["seconds"], 2)) for r in q["timings"]["records"]],
                          "length": (q["metrics"] or {}).get("path_length_m")}
    (a.output / "brief.json").write_text(json.dumps(brief, indent=1, default=float) + "\n")
    print(json.dumps(brief, default=float), flush=True)


if __name__ == "__main__":
    main()
