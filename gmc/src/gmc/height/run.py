"""Run the unchanged GMC pipeline on one projected map (mirrors k2_door_gate Stage 4)."""
import json
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

from ..budget import WorkLedger
from ..mobility.graph import compile_mobility
from ..mobility.lineage import component_slice, slab_has_safe_components
from ..mobility.query import query
from ..orientation.slab_builder import build_slabs
from ..spatial.bvh import query_candidate_pairs
from ..types import Pose2
from ..verification.path import verify_curve
from .pathio import curve_to_dict, save_path_json


def with_overrides(cfg, *, initial_intervals=None, max_depth=None,
                   max_refinement_rounds=None, max_wall_seconds=None,
                   max_support_calls=None):
    orient = cfg.orientation
    if initial_intervals is not None:
        orient = replace(orient, initial_intervals=int(initial_intervals))
    if max_depth is not None:
        orient = replace(orient, max_depth=int(max_depth))
    q = cfg.query
    changes = {k: v for k, v in (("max_refinement_rounds", max_refinement_rounds),
                                 ("max_wall_seconds", max_wall_seconds),
                                 ("max_support_calls", max_support_calls)) if v is not None}
    if changes:
        q = replace(q, **changes)
    return replace(cfg, orientation=orient, query=q)


def _pose(v):
    return Pose2(np.asarray(v[:2], dtype=float), float(v[2]))


def compile_and_query(scene2d, robot, cfg, start, goal, *, out_dir=None):
    body = robot.footprint
    ledger = WorkLedger()
    t0 = time.time()
    oracles = list(query_candidate_pairs(scene2d, body, scene2d.workspace).oracles)
    for o in oracles:
        o.ledger = ledger
    dec = build_slabs(scene2d, body, cfg, oracles, ledger=ledger)
    mc = compile_mobility(scene2d, body, cfg, oracles, dec, ledger=ledger)
    compile_s = time.time() - t0
    s, g = _pose(start), _pose(goal)
    t1 = time.time()
    result = query(s, g, mc)
    query_s = time.time() - t1
    verify = {"ran": False}
    if result.curve is not None:
        rep = verify_curve(tuple(oracles), scene2d.workspace, result.curve,
                           cfg.query.eps_clear, cfg.orientation.theta_min,
                           expected_start=s, expected_goal=g)
        verify = {"ran": True, "certified": bool(rep.certified),
                  "min_clearance": float(rep.min_clearance), "reason": rep.reason}
    res = {"status": result.status.name,
           "clearance_lb": result.clearance_lower_bound,
           "reason": result.report.get("reason"),
           "n_supports": len(scene2d.supports), "n_pairs": len(oracles),
           "n_slabs": len(dec.slabs), "compile_seconds": compile_s,
           "query_seconds": query_s,
           "safe_nodes": mc.M_safe.number_of_nodes(),
           "safe_edges": mc.M_safe.number_of_edges(),
           "possible_nodes": mc.M_possible.number_of_nodes(),
           "possible_edges": mc.M_possible.number_of_edges(),
           "verify": verify,
           "curve": curve_to_dict(result.curve) if result.curve is not None else None,
           "start": list(map(float, start)), "goal": list(map(float, goal))}
    if out_dir is not None:
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        if result.curve is not None:
            save_path_json(result.curve, out / "path.json")
        (out / "result.json").write_text(json.dumps(res, indent=2, default=str))
    return res, mc


def safe_areas(mc):
    out = []
    for slab in mc.decomposition.slabs:
        area = 0.0
        if slab_has_safe_components(slab):
            area = float(sum(c.geometry.area for c in component_slice(slab).D_safe))
        out.append({"lo": float(slab.interval.lo), "hi": float(slab.interval.hi),
                    "area": area})
    return out
