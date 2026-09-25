"""C1 synthetic acceptance run with a free preview of C2's comparison.

For every synthetic case/query (``aerial3d_fixtures``): compile the aerial3d complex
once per (scene, body), run one start+goal query, run the baseline
``LatticePlanner.plan`` once (0.10 m, margin 0.05, bounded budget), then for both
methods: verdict, raw polyline metrics (``aerial3d.metrics``), the SAME
``gs3d.smoothing.optimize_trajectory`` post-process (A6's UAV setting, as in
``uavlamp_run.smooth``) with its smoothed-curve jerk/duration, an independent
replay with a fresh oracle on a fresh index, and wall times.  Writes
``<out>/summary.json`` and one top+side PNG per case.

Usage (from gmc/):  python experiments/aerial3d_synthetic.py --out results/uavconn/synthetic
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import platform
from time import perf_counter

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import EllipseCollection
import numpy as np

from gmc.aerial3d.api import compile_complex, query
from gmc.aerial3d.metrics import polyline_metrics, smooth_segments_metrics
from gmc.aerial3d.octree import BLOCKED
from gmc.gs3d.contracts import GoalRegion, PlannerConfig, Pose3, SearchBudget
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.planner import LatticePlanner, _limits
from gmc.gs3d.smoothing import SmoothingConfig, optimize_trajectory
from gmc.gs3d.trajectory import replay_plan
import aerial3d_fixtures as F

CASES = [  # (case id, builder, kwargs, body, query names)
    ("open_ascent", F.open_ascent, {}, F.UAV, None),
    ("low_wall", F.low_wall, {}, F.UAV, None),
    ("full_wall_closed", F.full_wall, {"gap": False}, F.UAV, None),
    ("full_wall_gap", F.full_wall, {"gap": True}, F.UAV, None),
    ("overhang", F.overhang, {}, F.UAV, None),
    ("bridge", F.bridge, {}, F.UAV, ["under", "over"]),
    ("shaft", F.shaft, {}, F.UAV, None),
    ("window_small_uav", F.window_wall, {}, F.SMALL_UAV, None),
    ("window_big_uav", F.window_wall, {}, F.BIG_UAV, None),
    ("f1_under_over", F.under_over_f1, {}, F.UAV, None),
    ("mini_booth", F.mini_booth, {}, F.UAV, ["low_start", "high_start"]),
    ("mini_booth_plug", F.mini_booth, {"plug": True}, F.UAV, ["low_start", "high_start"]),
]
SMOOTH = SmoothingConfig(max_shortcut_span=64, max_certificate_depth=14, certificate_deviation_m=.005,
                         min_turn_improvement_fraction=.10)  # A6's UAV setting (uavlamp_run.smooth)
LATTICE = PlannerConfig(resolution_m=.10, margin_m=.05, seed=0,
                        budget=SearchBudget(max_wall_s=1200., max_expansions=400_000,
                                            max_oracle_calls=20_000_000, max_narrowphase_pairs=500_000_000))


def smooth_block(scene, body, trajectory, goal_xyz) -> dict:
    t0 = perf_counter()
    out = optimize_trajectory(trajectory, GaussianBodyOracle(PreparedScene(scene)), body, _limits(body),
                              margin_m=.05, goal=GoalRegion(Pose3(tuple(goal_xyz))), config=SMOOTH)
    cand = out.get("candidate_trajectory")
    cver = out.get("candidate_verification") or {}
    block = {"selected": out.get("selected"), "reason": out.get("reason"), "wall_s": perf_counter() - t0,
             "candidate_verified": bool(cver.get("passed")),
             "candidate_clearance_lower_m": cver.get("clearance_lower_m")}
    if cand is not None and cver.get("passed"):
        block["smoothed"] = smooth_segments_metrics(cand["segments"])
    return block


def method_row(status, poly, wall_s, scene, body, result_gs3d, goal) -> dict:
    row = {"status": status, "wall_s": wall_s}
    if poly is not None and result_gs3d is not None:
        row["raw"] = polyline_metrics(poly)
        fresh = replay_plan(result_gs3d, GaussianBodyOracle(PreparedScene(scene)))
        row["replay_fresh"] = {"passed": fresh["passed"],
                               "clearance_lower_m": fresh["geometry"].get("clearance_lower_m")}
        row["smoothing"] = smooth_block(scene, body, result_gs3d["trajectory"], goal)
    return row


def run_case(cid, builder, kwargs, body, names, out: Path) -> list[dict]:
    scene, queries = builder(**kwargs)
    t0 = perf_counter()
    compiled = compile_complex(scene, body)
    compile_s = perf_counter() - t0
    rows, results = [], {}
    for name in names or list(queries):
        start, goal = queries[name]
        r = query(compiled, start, goal)
        results[name] = r
        poly = np.asarray(r["polyline_world"]) if r["polyline_world"] else None
        a3 = method_row(r["status"], poly, r["timings"]["algorithm_wall_s"], scene, body, r["gs3d_result"], goal)
        a3.update(reason=r["reason"], compile_wall_s=compile_s,
                  query_stages={t["stage"]: t["seconds"] for t in r["timings"]["records"]},
                  clearance_lower_m=r["clearance_lower_m"],
                  certificate={k: v for k, v in (r["certificate"] or {}).items()
                               if k not in ("cut_pair_ids", "cut_leaf_centres_plan")} or None,
                  graph_cells=len(r["graph_path"]["cell_ids"]) if r["graph_path"] else None,
                  cell_certified_path=(polyline_metrics(compiled.frame.to_world(np.asarray(r["cell_polyline_plan"])))
                                       if r["cell_polyline_plan"] else None))
        t0 = perf_counter()
        lat = LatticePlanner().plan(scene, body, Pose3(tuple(start)), GoalRegion(Pose3(tuple(goal))), LATTICE)
        lat_s = perf_counter() - t0
        lpoly = np.asarray(lat["trajectory"]["poses"])[:, :3] if lat["trajectory"] else None
        lt = method_row(lat["status"], lpoly, lat["timings"]["algorithm_wall_s"], scene, body,
                        lat if lat["status"] == "success" else None, goal)
        lt.update(reason=lat["reason"], expansions=lat["diagnostics"]["expansions"], outer_wall_s=lat_s,
                  clearance_lower_m=lat["clearance_lower_m"])
        results[name + "__lattice"] = lat
        rows.append({"case": cid, "query": name, "body": body.name, "start": list(start), "goal": list(goal),
                     "n_gaussians": int(len(scene.gaussians.ids)), "aerial3d": a3, "lattice": lt,
                     "verdicts_agree": (r["status"] == "REACHABLE") == (lat["status"] == "success")})
    compile_doc = compiled.summary()
    (out / "runs").mkdir(parents=True, exist_ok=True)
    (out / "runs" / f"{cid}.json").write_text(json.dumps(
        {"case": cid, "compile": compile_doc, "rows": rows,
         "aerial3d_results": {k: v for k, v in results.items() if "__lattice" not in k}},
        allow_nan=False, default=float) + "\n")
    figure(cid, scene, compiled, results, out / f"{cid}.png")
    return rows


def figure(cid, scene, compiled, results, path: Path):
    g = scene.gaussians
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))
    for ax, (i, j, lab) in zip(axes, [(0, 1, "top (x-y)"), (0, 2, "side (x-z)")]):
        cov2 = g.covs[:, [i, j]][:, :, [i, j]]
        w, v = np.linalg.eigh(cov2)
        ang = np.degrees(np.arctan2(v[:, 1, 1], v[:, 0, 1]))
        ec = EllipseCollection(4 * np.sqrt(w[:, 1]), 4 * np.sqrt(w[:, 0]), ang, units="xy",
                               offsets=g.means[:, [i, j]], offset_transform=ax.transData,
                               facecolor=(.55, .55, .6, .25), edgecolor="none")
        ax.add_collection(ec)
        lo, hi = np.asarray(scene.bounds_min), np.asarray(scene.bounds_max)
        ax.add_patch(plt.Rectangle((lo[i], lo[j]), hi[i] - lo[i], hi[j] - lo[j], fill=False, ls=":", color="k"))
        for n, (name, r) in enumerate((k, v) for k, v in results.items() if "__lattice" not in k):
            col = ["tab:blue", "tab:green"][n % 2]
            s, gl = np.asarray(r["start_world"]), np.asarray(r["goal_world"])
            ax.plot(s[i], s[j], "o", color=col, ms=7)
            ax.plot(gl[i], gl[j], "*", color=col, ms=12)
            if r["polyline_world"]:
                P = np.asarray(r["polyline_world"])
                ax.plot(P[:, i], P[:, j], "-", color=col, lw=2.2, marker=".", ms=9,
                        label=f"aerial3d {name}: {r['status']} {r['metrics']['path_length_m']:.2f} m")
            else:
                ax.plot([], [], color=col, label=f"aerial3d {name}: {r['status']} ({r['reason']})")
            lat = results[name + "__lattice"]
            if lat["trajectory"]:
                L = np.asarray(lat["trajectory"]["poses"])
                ax.plot(L[:, i], L[:, j], "--", color="tab:orange", lw=1.3,
                        label=f"lattice {name}: {lat['diagnostics']['path_length_m']:.2f} m")
            else:
                ax.plot([], [], "--", color="tab:orange", label=f"lattice {name}: {lat['status']}")
        if any(r["status"] == "UNREACHABLE" for k, r in results.items() if "__lattice" not in k):
            tree = compiled.tree
            b = np.flatnonzero(tree.status == BLOCKED)
            c, _ = tree.boxes(b)
            cw = compiled.frame.to_world(c)
            ax.plot(cw[:, i], cw[:, j], "s", color="tab:red", ms=1.2, alpha=.25, label="BLOCKED leaves (inner-polytope)")
        ax.set_aspect("equal")
        ax.set_xlim(lo[i] - .1, hi[i] + .1); ax.set_ylim(lo[j] - .1, hi[j] + .1)
        ax.set_title(f"{cid} - {lab}"); ax.set_xlabel("xyz"[i] + " (m)"); ax.set_ylabel("xyz"[j] + " (m)")
    axes[1].legend(loc="upper left", bbox_to_anchor=(1.01, 1.), fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--only", nargs="*")
    a = ap.parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)
    rows = []
    for cid, builder, kw, body, names in CASES:
        if a.only and cid not in a.only:
            continue
        t0 = perf_counter()
        rows += run_case(cid, builder, kw, body, names, a.out)
        print(f"{cid}: {perf_counter() - t0:.1f} s", flush=True)
    doc = {"schema": "uavconn.c1_synthetic.v1", "rows": rows,
           "host": {"node": platform.node(), "slurm_job_id": os.environ.get("SLURM_JOB_ID"),
                    "cpus": os.environ.get("SLURM_CPUS_PER_TASK")},
           "lattice_config": {"resolution_m": .10, "margin_m": .05, "budget": LATTICE.budget.__dict__},
           "smoothing_config": SMOOTH.__dict__}
    name = "summary.json" if not a.only else f"summary_{'_'.join(a.only)}.json"
    (a.out / name).write_text(json.dumps(doc, indent=1, allow_nan=False, default=float) + "\n")


if __name__ == "__main__":
    main()
