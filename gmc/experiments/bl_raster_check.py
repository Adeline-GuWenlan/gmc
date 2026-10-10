"""bl B2: the shared rasteriser measured against the judge (gmc-venv, sbatch: loads the persisted compiles).

Per region x robot x resolution (builds + persists the raster if missing):
  * uniform: N random points in the region box -> the judge's own pose check (``GaussianBodyOracle.pose`` on the
    SHA-checked compile's prepared scene, body at z_c on the support, margin 0.001) vs the map's cell.  Confusion:
    map FREE but judge not free (must be 0: an unsafe map; every case is listed with its reason) and map OCCUPIED but
    judge free (the map's conservatism; it can close real passages).
  * boundary: M free cells that touch an occupied cell (8-neighbours) -- where an unsafe map would show first; the
    judge is asked at the centre, the 4 corners and a random point of each (a free cell must be free everywhere).
  * routes: every pair of the tuning set and of F4's 5000: is the start / goal cell occupied (the endpoints are
    confirmed free, so this is conservatism at the endpoint), snap distance to the nearest free cell, does the stored
    A* route (0.1 mm rounded) cross a map-occupied cell, and are the snapped endpoints in the same 8-connected free
    component (= a grid planner on this map can connect them at all).  Per F4 lateral-clearance band.
Output: ``results/baselines/raster/check_<R>_<robot>.json`` (+ a per-pair csv.gz for the route part).
"""
from __future__ import annotations

import argparse
import gzip
import json
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np

sys.dont_write_bytecode = True
import bl_raster as R  # noqa: E402

OUT = Path("results/baselines/raster")


def judge_points(J, oracle, uv):
    from gmc.gs3d.contracts import Pose3
    out = []
    m = J.C.config.margin_m
    for p in np.atleast_2d(uv):
        rep = oracle.pose(Pose3(tuple(map(float, J.world(p)))), J.C.body, margin_m=m)
        out.append((rep.occupancy, rep.reason, rep.clearance_lower_m))
    return out


def route_stats(arrays, info, pairs, feat=None):
    occ, lab = arrays["occ"], arrays["free_label"]
    rows = []
    for p in pairs:
        s, g = np.asarray(p["start_uv"], float), np.asarray(p["goal_uv"], float)
        so, go = (bool(x) for x in R.occupied_at(arrays, info, [s, g]))
        ns, ng = R.nearest_free(arrays, info, s), R.nearest_free(arrays, info, g)
        route = p["astar_route_uv"]
        route = json.loads(route) if isinstance(route, str) else route
        hits, total = R.polyline_hits(arrays, info, np.asarray(route, float))
        conn = (ns is not None and ng is not None and lab[ns[0], ns[1]] == lab[ng[0], ng[1]])
        rows.append({"pair_id": p["pair_id"], "lat": p.get("lateral_clearance_m"), "start_occ": so, "goal_occ": go,
                     "start_snap_m": None if ns is None else ns[3], "goal_snap_m": None if ng is None else ng[3],
                     "route_samples_occ": hits, "route_samples": total, "route_crosses_occ": hits > 0,
                     "connected": bool(conn)})
    return rows


def summarise_routes(rows, band_fn=None):
    n = len(rows)
    if not n:
        return {}

    def frac(k):
        return sum(1 for r in rows if r[k]) / n
    snaps = [max(r["start_snap_m"] or 0, r["goal_snap_m"] or 0) for r in rows]
    out = {"n": n, "start_cell_occupied": frac("start_occ"), "goal_cell_occupied": frac("goal_occ"),
           "route_crosses_occupied": frac("route_crosses_occ"),
           "route_samples_occupied_frac_mean": float(np.mean([r["route_samples_occ"] / max(1, r["route_samples"])
                                                              for r in rows])),
           "snapped_endpoints_connected": frac("connected"),
           "max_snap_m_p50_p95_max": [float(np.percentile(snaps, 50)), float(np.percentile(snaps, 95)),
                                      float(np.max(snaps))]}
    if band_fn:
        by = {}
        for r in rows:
            by.setdefault(band_fn(r["lat"]), []).append(r)
        out["by_lateral_band"] = {b: {"n": len(v), "route_crosses_occupied": sum(r["route_crosses_occ"] for r in v)
                                      / len(v), "connected": sum(r["connected"] for r in v) / len(v)}
                                  for b, v in by.items()}
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--region", required=True)
    ap.add_argument("--robot", required=True)
    ap.add_argument("--res", nargs="+", type=float, required=True)
    ap.add_argument("--n-uniform", type=int, default=3000)
    ap.add_argument("--n-boundary", type=int, default=1000)
    ap.add_argument("--seed", type=int, default=20261012)
    a = ap.parse_args(argv)
    from bl_harness import Judge, load_pairs, _lat_band, scene_export_path
    from gmc.gs3d.oracle import GaussianBodyOracle
    rng = np.random.default_rng(a.seed)
    t0 = time.perf_counter()
    J = Judge(a.region, a.robot)
    oracle = GaussianBodyOracle(J.C.prepared)
    scene = R.load_scene(scene_export_path(a.region, a.robot))
    meta = scene["meta"]
    if meta["a3c_sha256"] != J.a3c_sha256:
        raise ValueError("scene export was not made from the judge's compile")
    lo, hi = meta["known_route_lower_m"], meta["known_route_upper_m"]
    # the judge's scene bounds must be the world AABB of the known prism (bl_raster.world_bounds)
    wb = R.world_bounds(meta)
    bounds_match = bool(np.allclose(J.C.prepared.lower, wb[0], atol=1e-9)
                        and np.allclose(J.C.prepared.upper, wb[1], atol=1e-9))
    U = np.c_[rng.uniform(lo[0], hi[0], a.n_uniform), rng.uniform(lo[1], hi[1], a.n_uniform)]
    tj = time.perf_counter()
    JU = judge_points(J, oracle, U)
    uniform_judge_s = time.perf_counter() - tj
    ju_free = np.array([o == "free" for o, _, _ in JU])
    tuning = load_pairs("tuning", a.region)
    f4 = load_pairs("f4", a.region)
    doc = {"region": a.region, "robot": a.robot, "judge_a3c_sha256": J.a3c_sha256,
           "scene_export_sha256": scene["_sha256"], "judge_bounds_equal_raster_world_bounds": bounds_match,
           "uniform": {"n": a.n_uniform, "judge_free": int(ju_free.sum()),
                       "judge_verdicts": dict(Counter(f"{o}:{r}" for o, r, _ in JU)),
                       "judge_wall_s": uniform_judge_s},
           "by_res": {}}
    csv_rows = []
    for res in a.res:
        path = R.raster_path(a.region, a.robot, res)
        if path.exists():
            arrays, info = R.load(path)
        else:
            arrays, info = R.build(scene, meta["body"], res)
            info = R.save(path, arrays, info)
            arrays, info = R.load(path)
        occ = arrays["occ"].astype(bool)
        m_occ = R.occupied_at(arrays, info, U)
        unsafe = [(i, JU[i]) for i in np.nonzero(~m_occ & ~ju_free)[0]]
        # boundary free cells (8-neighbour of an occupied cell)
        pad = np.pad(occ, 1, constant_values=True)
        nb = np.zeros_like(occ)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy or dx:
                    nb |= pad[1 + dy:1 + dy + occ.shape[0], 1 + dx:1 + dx + occ.shape[1]]
        bcells = np.argwhere(~occ & nb)
        pick = bcells[rng.permutation(len(bcells))[:a.n_boundary]]
        c = R.centre(info, pick[:, 0], pick[:, 1])
        h = info["res_m"] / 2
        offs = np.array([[0, 0], [-h, -h], [-h, h], [h, -h], [h, h]], float)
        pts = np.concatenate([c + o for o in offs] + [c + rng.uniform(-h, h, c.shape)])
        tb = time.perf_counter()
        JB = judge_points(J, oracle, pts)
        bnd_s = time.perf_counter() - tb
        bad_b = [(pts[i].tolist(), JB[i]) for i in range(len(pts)) if JB[i][0] != "free"]
        clear_b = np.array([cl for o, _, cl in JB if o == "free" and cl is not None])
        tr = time.perf_counter()
        rt = route_stats(arrays, info, tuning)
        rf = route_stats(arrays, info, f4)
        route_s = time.perf_counter() - tr
        for r in rt:
            csv_rows.append(dict(r, set="tuning", res_m=res))
        for r in rf:
            csv_rows.append(dict(r, set="f4", res_m=res))
        n_jf = int(ju_free.sum())
        doc["by_res"][f"{res * 1000:g}mm"] = {
            "raster": str(path), "raster_sha256": info["sha256"], "build_wall_s": info["build_wall_s"],
            "grid": [info["nx"], info["ny"]], "occupied_frac": info["occupied_cells"] / info["cells"],
            "occupied_components_8conn": info["occupied_components_8conn"],
            "free_components_8conn": info["free_components_8conn"],
            "n_in_slab_and_near": info["n_in_slab_and_near"], "n_cut_by_slab": info["n_cut_by_slab"],
            "uniform": {"map_free_judge_not_free": len(unsafe),
                        "map_free_judge_not_free_cases": [{"uv": U[i].tolist(), "judge": list(j)} for i, j in unsafe],
                        "map_occupied_judge_free": int((m_occ & ju_free).sum()),
                        "conservatism_frac_of_judge_free": float((m_occ & ju_free).sum() / max(1, n_jf)),
                        "map_free": int((~m_occ).sum()), "judge_free": n_jf},
            "boundary": {"free_cells_touching_occupied": int(len(bcells)), "cells_sampled": int(len(pick)),
                         "points_judged": int(len(pts)), "points_not_free": len(bad_b),
                         "not_free_cases": bad_b[:50], "judge_wall_s": bnd_s,
                         "clearance_lower_m_p0_p5_p50": None if not len(clear_b) else
                         [float(np.min(clear_b)), float(np.percentile(clear_b, 5)), float(np.median(clear_b))]},
            "routes_tuning": summarise_routes(rt), "routes_f4": summarise_routes(rf, _lat_band),
            "route_wall_s": route_s}
        print(a.region, a.robot, f"{res * 1000:g}mm", json.dumps({k: doc["by_res"][f"{res * 1000:g}mm"][k] for k in
              ("build_wall_s", "grid", "occupied_frac")}), "unsafe uniform", len(unsafe), "boundary not free",
              len(bad_b), "conservatism", doc["by_res"][f"{res * 1000:g}mm"]["uniform"]["conservatism_frac_of_judge_free"],
              "f4 connected", doc["by_res"][f"{res * 1000:g}mm"]["routes_f4"]["snapped_endpoints_connected"], flush=True)
    doc["wall_s"] = time.perf_counter() - t0
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"check_{a.region}_{a.robot}.json").write_text(json.dumps(doc, indent=1, default=float) + "\n")
    with gzip.open(OUT / f"check_{a.region}_{a.robot}_pairs.jsonl.gz", "wt") as f:
        for r in csv_rows:
            f.write(json.dumps(r, default=float) + "\n")


if __name__ == "__main__":
    main()
