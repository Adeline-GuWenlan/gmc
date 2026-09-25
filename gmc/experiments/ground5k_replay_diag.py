"""Diagnose a FAIL_REPLAY record: which edge, which Gaussians, how close really.

For the rejected edge of the shared replay, each implicated Gaussian is re-bounded (i) by the
gs3d oracle's own pair routine with 10x the GJK iterations and (ii) by a dense direction search:
the separating-support gap of the swept upright cylinder against the level-2 ellipsoid, maximised
over 20k directions and refined with Nelder-Mead.  (ii) only ever reports a gap it exhibits, so a
positive value is a valid lower bound; a value <= 0 is evidence of contact at the sampled resolution.
Diagnostic only: nothing here feeds an outcome.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

from gmc.gs3d.geometry import cylinder_ellipsoid_bound
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene

import ground5k_common as gc
import ground5k_run as gr


def sweep_gap(n, a, b, radius, half, mu, cov, level):
    n = n / np.linalg.norm(n)
    body = min(a @ n, b @ n) - radius * np.linalg.norm(n[:2]) - half * abs(n[2])
    return body - (mu @ n + level * np.sqrt(max(n @ cov @ n, 0.0)))


def best_gap(a, b, radius, half, mu, cov, level, k=20000):
    """max over unit n of the separating gap (n ranges over the whole sphere, so both sides)."""
    dirs = np.random.default_rng(0).normal(size=(k, 3))
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    g = np.array([sweep_gap(d, a, b, radius, half, mu, cov, level) for d in dirs])
    i = int(np.argmax(g))
    r = minimize(lambda v: -sweep_gap(v, a, b, radius, half, mu, cov, level), dirs[i],
                 method="Nelder-Mead", options={"xatol": 1e-10, "fatol": 1e-12, "maxiter": 4000})
    n = r.x / np.linalg.norm(r.x)
    return max(float(g[i]), sweep_gap(n, a, b, radius, half, mu, cov, level)), n.tolist()


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("records", nargs="+")
    ap.add_argument("--out", default="outputs/ground5k/diag")
    args = ap.parse_args(argv)
    cfg = gc.load_config()
    scene3d, floor = gc.load_scene(cfg, "scene_v2")
    pairs = {p["pair_id"]: p for p in json.loads(
        (Path("outputs/ground5k/scene_v2") / "pairs_scene_v2.json").read_text())["pairs"]}
    for path in args.records:
        rec = json.loads(Path(path).read_text())
        pair = pairs[rec["pair_id"]]
        body = gc.BODIES[rec["robot"]]
        region = gc.query_region(pair["crop"]["box_xy"], cfg, floor)
        spec, _ = gc.crop_query(scene3d, region, floor, cfg, "diag")
        prepared = PreparedScene(spec)
        poses = gr.gmc_poses(rec["curve"], body, floor) if rec["planner"] == "gmc" else None
        goal = gc.ground_pose(*pair["goal"], body, floor, 0.0)
        rep, rows = gr.replay_poses(spec, body, poses, goal, cfg)
        summ = gr._replay_summary(rep, rows)
        out = {"pair_id": rec["pair_id"], "combo": rec["combo"], "replay": summ, "primitives": []}
        fe = summ.get("failed_edge")
        if fe:
            a, b = np.asarray(fe["from"][:3]), np.asarray(fe["to"][:3])
            id_to_row = {int(i): k for k, i in enumerate(prepared.ids)}
            oracle = GaussianBodyOracle(prepared)
            wide = oracle.edge(*(gc.ground_pose(*a[:2], body, floor), gc.ground_pose(*b[:2], body, floor)),
                               body, margin_m=cfg["margin_m"])
            for pid in fe["primitive_ids"] or list(wide.primitive_ids):
                k = id_to_row[int(pid)]
                mu, cov = prepared.means[k], prepared.covs[k]
                pb = cylinder_ellipsoid_bound(a, b, body.radius_m, body.half_height_m, mu, cov, cfg["level"],
                                              margin_m=cfg["margin_m"], max_iterations=400)
                gap, arg = best_gap(a, b, body.radius_m, body.half_height_m, mu, cov, cfg["level"])
                ell_z = [float(mu[2] - cfg["level"] * np.sqrt(cov[2, 2])), float(mu[2] + cfg["level"] * np.sqrt(cov[2, 2]))]
                out["primitives"].append({
                    "id": int(pid), "mean": mu.tolist(), "ellipsoid_z_extent_above_floor": [z - floor["z_floor"] for z in ell_z],
                    "sqrt_eig": np.sqrt(np.linalg.eigvalsh(cov)).tolist(),
                    "gjk400": {"lower_m": pb.clearance_lower_m, "overlap": pb.overlap, "reason": pb.reason},
                    "dense_direction_gap_m": gap, "direction": arg})
        gc.write_json_atomic(Path(args.out) / f"{rec['combo']}_{rec['pair_id']}.json", out)
        print(json.dumps({"pair": rec["pair_id"], "failed_edge": {k: fe.get(k) for k in ("index", "reason", "clearance_lower_m", "primitive_ids")} if fe else None,
                          "prims": [(p["id"], round(p["gjk400"]["lower_m"], 6), p["gjk400"]["overlap"], round(p["dense_direction_gap_m"], 6),
                                     [round(v, 3) for v in p["ellipsoid_z_extent_above_floor"]]) for p in out["primitives"]]}), flush=True)


if __name__ == "__main__":
    main()
