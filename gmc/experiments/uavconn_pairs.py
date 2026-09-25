"""C2 extended query set: seeded start/goal pairs in the booth scene's query box.

Pre-registered in ``docs/uavconn_c2_prereg.md`` (committed before this runs).  Two classes:

* ``A`` corridor -> table: start uniform in the approach corridor, goal centre uniform
  over the table-top footprint at a height where the body is above the table;
* ``B`` random free pairs: start and goal uniform over the C-space domain's bounding
  box (the box every configuration of both methods lives in), at least ``min_dist_m`` apart.

A pose is kept only if the **shared gs3d oracle** certifies it free (``pose`` with the frozen
body and margin on the same SceneSpec both planners get).  Rejections are counted by reason.
For each pair the oracle's verdict on the straight segment is recorded (a stratum, not a filter).

Writes ``<out>/pairs.json`` and one spec per pair (``<spec_dir>/<name>.json``): the template
spec with only ``name``/``output``/``start_route``/``goal_route``/``budget`` replaced and
absolute archive paths, so both methods read identical inputs.

Usage (from ``gmc/``)::

    python experiments/uavconn_pairs.py configs/uavlamp_l2/M1_main_low_start.json \
        --uavlamp-root /scratch/wg2381/splathjb-uavlamp/gmc --out results/uavconn/extended \
        --spec-dir configs/uavconn_c2/ext
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import platform
import time

import numpy as np

from gmc.aerial3d.pairs import domain_from_scene
from gmc.gs3d import scene_uavlamp as su
from gmc.gs3d.contracts import Pose3
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene

from uavlamp_query import UAV, build_scene
from uavlamp_run import check_spec
from uavconn_run import resolve

SEED = 20260925
# Route-frame regions for the booth (docs/uavconn_c2_prereg.md §2).  u < -1.3 is the approach
# corridor (the lamp slab's C-obstacle starts at u ~ -1.16); the table top is L1's footprint.
BOOTH_REGIONS = {
    "A_start": {"lower": [-2.6, .35, .20], "upper": [-1.3, 2.05, 2.20]},
    "A_goal": {"lower": [-.02, -.03, 1.15], "upper": [2.97, .79, 2.25]},
}
BASELINE_BUDGET = {"max_wall_s": 18000.0, "max_expansions": 2000000, "max_oracle_calls": 50000000,
                   "max_narrowphase_pairs": 1000000000}


def free_pose(oracle, frame, p_route, margin) -> tuple[bool, str]:
    rep = oracle.pose(Pose3(tuple(map(float, frame.to_world(p_route)))), UAV, margin_m=margin)
    return rep.occupancy == "free", rep.reason


def draw(rng, box, oracle, frame, margin, stats, max_tries=5000):
    lo, hi = np.asarray(box["lower"], float), np.asarray(box["upper"], float)
    for _ in range(max_tries):
        p = lo + rng.random(3) * (hi - lo)
        ok, reason = free_pose(oracle, frame, p, margin)
        stats["drawn"] += 1
        if ok:
            stats["accepted"] += 1
            return p
        stats["rejected_by_reason"][reason] = stats["rejected_by_reason"].get(reason, 0) + 1
    raise RuntimeError(f"no free pose in {box} after {max_tries} draws")


def generate(scene, frame, *, n_a: int, n_b: int, seed: int = SEED, margin: float = .05,
             regions: dict = BOOTH_REGIONS, min_dist_m: float = 1.0, domain_box: dict | None = None,
             slab_u: float | None = -.85) -> dict:
    oracle = GaussianBodyOracle(PreparedScene(scene))
    if domain_box is None:
        _, dom = domain_from_scene(scene, UAV, margin_m=margin)
        domain_box = {"lower": list(dom.bbox_lower), "upper": list(dom.bbox_upper)}
    rng = np.random.default_rng(seed)
    stats = {"drawn": 0, "accepted": 0, "rejected_by_reason": {}}
    pairs = []
    for k in range(n_a):
        s = draw(rng, regions["A_start"], oracle, frame, margin, stats)
        g = draw(rng, regions["A_goal"], oracle, frame, margin, stats)
        pairs.append(("A", k, s, g))
    for k in range(n_b):
        while True:
            s = draw(rng, domain_box, oracle, frame, margin, stats)
            g = draw(rng, domain_box, oracle, frame, margin, stats)
            if np.linalg.norm(g - s) >= min_dist_m:
                break
            stats["pairs_rejected_too_close"] = stats.get("pairs_rejected_too_close", 0) + 1
        pairs.append(("B", k, s, g))
    rows = []
    for cls, k, s, g in pairs:
        t0 = time.perf_counter()
        sw, gw = frame.to_world(s), frame.to_world(g)
        rep = oracle.edge(Pose3(tuple(map(float, sw))), Pose3(tuple(map(float, gw))), UAV, margin_m=margin)
        rows.append({"name": f"E{cls}{k:02d}", "class": cls,
                     "start_route": [round(float(x), 4) for x in s], "goal_route": [round(float(x), 4) for x in g],
                     "distance_m": float(np.linalg.norm(g - s)),
                     "straight_segment_oracle": {"occupancy": rep.occupancy, "reason": rep.reason,
                                                 "wall_s": time.perf_counter() - t0},
                     "crosses_lamp_plane": (None if slab_u is None else bool((s[0] < slab_u) != (g[0] < slab_u)))})
    # re-check the rounded poses (the specs carry rounded values)
    for r in rows:
        for key in ("start_route", "goal_route"):
            ok, reason = free_pose(oracle, frame, np.asarray(r[key]), margin)
            if not ok:
                raise RuntimeError(f"{r['name']} {key} not free after rounding ({reason})")
    return {"seed": seed, "n_a": n_a, "n_b": n_b, "margin_m": margin, "min_dist_m": min_dist_m,
            "regions_route": regions, "domain_box_route": domain_box, "sampling_stats": stats,
            "pairs": rows}


def write_specs(template: dict, pairs: list[dict], spec_dir: Path, out_root: str) -> list[str]:
    spec_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for r in pairs:
        spec = {**template, "name": f"c2_ext_{r['name']}", "output": f"{out_root}/{r['name']}",
                "start_route": r["start_route"], "goal_route": r["goal_route"], "budget": dict(BASELINE_BUDGET)}
        check_spec(spec)
        dst = spec_dir / f"{r['name']}.json"
        dst.write_text(json.dumps(spec, indent=1) + "\n")
        paths.append(str(dst))
    return paths


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("template", type=Path)
    ap.add_argument("--uavlamp-root", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--spec-dir", type=Path, required=True)
    ap.add_argument("--n-a", type=int, default=12)
    ap.add_argument("--n-b", type=int, default=12)
    ap.add_argument("--seed", type=int, default=SEED)
    a = ap.parse_args(argv)
    template = resolve(check_spec(json.loads(a.template.read_text())), a.uavlamp_root)
    t0 = time.perf_counter()
    full, doc = su.load_uavlamp_derivative(template["archive"], template["manifest"])
    if doc["derivative"]["sha256"] != template["archive_sha256"]:
        raise ValueError("archive hash differs from the spec")
    frame, scene, *_ = build_scene(template, full, doc)
    del full
    out = generate(scene, frame, n_a=a.n_a, n_b=a.n_b, seed=a.seed, margin=template.get("margin_m", .05))
    out.update(schema="uavconn.c2_pairs.v1", template=str(a.template), archive_sha256=doc["derivative"]["sha256"],
               oracle="gmc.gs3d.oracle.GaussianBodyOracle.pose (shared collision authority), same SceneSpec as both "
                      "planners (uavlamp_query.build_scene of the template)",
               baseline_budget=BASELINE_BUDGET, wall_s=time.perf_counter() - t0,
               host={"node": platform.node(), "slurm_job_id": os.environ.get("SLURM_JOB_ID")})
    tmpl = {k: v for k, v in json.loads(a.template.read_text()).items()}
    tmpl = {**tmpl, "archive": template["archive"], "manifest": template["manifest"]}
    out["spec_files"] = write_specs(tmpl, out["pairs"], a.spec_dir, "outputs/uavconn/c2/ext")
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "pairs.json").write_text(json.dumps(out, indent=1, default=float) + "\n")
    print(json.dumps({"pairs": [(r["name"], r["start_route"], r["goal_route"], r["straight_segment_oracle"]["occupancy"],
                                 r["crosses_lamp_plane"]) for r in out["pairs"]], "stats": out["sampling_stats"]},
                     default=float), flush=True)


if __name__ == "__main__":
    main()
