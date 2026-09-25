"""C2: the SAME gs3d A6 smoothing post-process on both methods' raw paths, each curve re-verified.

For every given run JSON (aerial3d from ``uavconn_run.py``, lattice from ``uavlamp_run.py``)
whose raw path passed the shared fresh replay:

1. ``gs3d.smoothing.optimize_trajectory`` on the raw linear trajectory with A6's UAV
   ``SmoothingConfig`` (identical to ``uavlamp_run.smooth`` and ``aerial3d_synthetic.SMOOTH``),
   with a fresh oracle on a freshly prepared index;
2. the candidate curve is re-verified independently (``verify_piecewise_bezier`` with another
   fresh oracle / fresh index) -- a smoothed curve never inherits the raw path's safety flag;
3. ``aerial3d.metrics.smooth_segments_metrics`` (exact integrated squared jerk, duration,
   length) and gs3d's own smoothed-candidate turn metrics; ordered evidence on the curve.

All runs given must share one scene (same spec scene keys); the scene is built once.

Usage (from ``gmc/``)::

    python experiments/uavconn_smooth.py --template SPEC --out DIR \
        --run aerial3d:M1=PATH/result.json --run lattice:M1=PATH/result.json ... [--uavlamp-root R]
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import platform
from time import perf_counter

import numpy as np

from gmc.aerial3d.metrics import polyline_metrics, smooth_segments_metrics
from gmc.gs3d import scene_uavlamp as su
from gmc.gs3d.contracts import GoalRegion, Pose3
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.planner import _limits
from gmc.gs3d.smoothing import SmoothingConfig, optimize_trajectory, verify_piecewise_bezier

from uavlamp_query import UAV, build_scene
from uavlamp_run import check_spec, ordered_evidence
from uavconn_run import resolve, scene_key

SMOOTH = SmoothingConfig(max_shortcut_span=64, max_certificate_depth=14, certificate_deviation_m=.005,
                         min_turn_improvement_fraction=.10)  # A6's UAV setting (uavlamp_run.smooth)


def raw_of(method: str, doc: dict):
    """(raw gs3d-style result, replay passed?) for either runner's result JSON."""
    if method == "aerial3d":
        res = doc["result"].get("gs3d_result")
    else:
        res = doc["result"] if doc["result"].get("status") == "success" else None
    return res, bool((doc.get("replay") or {}).get("passed"))


def smooth_one(scene, frame, spec, res) -> dict:
    margin = float(spec.get("margin_m", .05))
    goal = GoalRegion(Pose3(tuple(res["original_goal"][:3]), res["original_goal"][3]))
    limits = _limits(UAV)
    t0 = perf_counter()
    out = optimize_trajectory(res["trajectory"], GaussianBodyOracle(PreparedScene(scene)), UAV, limits,
                              margin_m=margin, goal=goal, config=SMOOTH)
    wall = perf_counter() - t0
    cand, cver = out.get("candidate_trajectory"), out.get("candidate_verification") or {}
    block = {"selected": out.get("selected"), "reason": out.get("reason"), "optimizer_wall_s": wall,
             "candidate_verified_by_optimizer": bool(cver.get("passed")),
             "candidate_clearance_lower_m": cver.get("clearance_lower_m"),
             "anchor_rows": (out.get("optimization") or {}).get("anchor_rows"),
             "input_position_rows": (out.get("optimization") or {}).get("input_position_rows"),
             "spline_tension": (out.get("optimization") or {}).get("spline_tension"),
             "gs3d_metrics": out.get("metrics"),
             "raw_polyline": polyline_metrics(np.asarray(res["trajectory"]["poses"], float)[:, :3])}
    if cand is not None:
        t1 = perf_counter()
        again = verify_piecewise_bezier(cand, GaussianBodyOracle(PreparedScene(scene)), UAV, limits,
                                        margin_m=margin, goal=goal, config=SMOOTH)
        block["fresh_reverification"] = {k: again.get(k) for k in ("passed", "safety", "reason",
                                                                   "clearance_lower_m")}
        block["fresh_reverification"]["wall_s"] = perf_counter() - t1
        block["smoothed"] = smooth_segments_metrics(cand["segments"])
        route = frame.to_route(np.asarray(cand["poses"], float)[:, :3])
        ev = ordered_evidence(route, lamp=spec["lamp"], table=spec["table"], radius=UAV.radius_m,
                              half_height=UAV.half_height_m, margin=margin)
        block["evidence"] = {k: ev[k] for k in ("z_range", "passes_under_lamp", "under_precedes_above_table",
                                                "under_lamp_intervals", "above_table_intervals",
                                                "over_lamp_intervals", "path_length_m")}
        block["route_samples"] = route[::max(1, len(route) // 400)].round(4).tolist()
    return block


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--template", type=Path, required=True)
    ap.add_argument("--uavlamp-root", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--run", action="append", required=True, help="method:name=path/to/result.json")
    a = ap.parse_args(argv)
    template = resolve(check_spec(json.loads(a.template.read_text())), a.uavlamp_root)
    runs = []
    for item in a.run:
        head, path = item.split("=", 1)
        method, name = head.split(":", 1)
        doc = json.loads(Path(path).read_text())
        spec = resolve(check_spec(doc["spec"]), a.uavlamp_root)
        if scene_key(spec) != scene_key(template):
            raise ValueError(f"{item}: not the template's scene")
        runs.append((method, name, path, doc, spec))
    full, mdoc = su.load_uavlamp_derivative(template["archive"], template["manifest"])
    if mdoc["derivative"]["sha256"] != template["archive_sha256"]:
        raise ValueError("archive hash differs from the spec")
    frame, scene, *_ = build_scene(template, full, mdoc)
    del full
    a.out.mkdir(parents=True, exist_ok=True)
    summary = []
    for method, name, path, doc, spec in runs:
        res, passed = raw_of(method, doc)
        row = {"method": method, "name": name, "run_json": path, "raw_replay_passed": passed}
        if res is not None and passed:
            row.update(smooth_one(scene, frame, spec, res))
        else:
            row["skipped"] = "no replay-passing raw path"
        (a.out / f"{method}__{name}.json").write_text(json.dumps(row, allow_nan=False, default=float) + "\n")
        brief = {k: row.get(k) for k in ("method", "name", "selected", "reason", "optimizer_wall_s")}
        brief["reverified"] = (row.get("fresh_reverification") or {}).get("passed")
        brief["jerk"] = (row.get("smoothed") or {}).get("integrated_squared_jerk")
        print(json.dumps(brief, default=float), flush=True)
        summary.append(brief)
    (a.out / "summary.json").write_text(json.dumps({"schema": "uavconn.c2_smooth.v1", "config": SMOOTH.__dict__,
                                                     "rows": summary, "host": {"node": platform.node(),
                                                     "slurm_job_id": os.environ.get("SLURM_JOB_ID")}},
                                                    indent=1, default=float) + "\n")


if __name__ == "__main__":
    main()
