"""F2 Task 1a: choose a new, open sub-region of the same scene_v2 archive for a clean cylinder benchmark.

Run from ``gmc/`` with ``PYTHONPATH=src:experiments`` (sbatch: loads the ~0.8 GB archive per box).

* ``survey``  for each candidate route box (u0 v0 u1 v1, z 0..2.43 as G2):
              - G2's crop rule -> number of selected supports (size of the compile input),
              - floor-support contract value (fitted plane's max corner deviation, limit 0.05 m),
              - a gs3d oracle clearance map for each robot on a ``--step`` (u, v) grid at the robot's own z_c,
                margin 0.001 (G2's endpoint authority): status code + ``clearance_lower_m`` (the oracle caps
                it at 0.10 m), so the share of the box with clearance >= 1 / 2 / 5 cm is measured, not guessed.
              Plus, over the archive-raster window, a raster of "near-floor" Gaussians: opacity > 0.3, route
              2-sigma top z in [0, 0.07] m.  They are inside the crop (top >= floor) and within 5 cm below the
              chassis bottom (0.02 m) -- the F1 endpoint / CY-GAP mechanism -- so they bound any clearance
              requirement from below.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np

from aerial3dg_run import ARCHIVE, GROUND_CONFIG, MANIFEST, ROBOTS, _box, _dump, ground_world, host, load_booth

TAU, LEVEL = .3, 2.
WINDOW = (-13.0, -5.0, 7.7, 8.0)
CODES = {"free": 0, "geometry_or_margin_unproven": 1, "occupied": 2, "map_unknown": 3, "other": 4}


def near_floor_raster(step=.05, top=(0., .07)):
    man = json.loads(MANIFEST.read_text())
    R = np.asarray(man["frame"]["world_to_route"], float)
    o = np.asarray(man["frame"]["origin_world_m"], float)
    with np.load(ARCHIVE, allow_pickle=False) as d:
        means, covs, op = d["means"], d["covs"], d["opacity"]
    keep = op > TAU
    mu = (means[keep] - o) @ R.T
    sz = LEVEL * np.sqrt(np.maximum(np.einsum("j,njk,k->n", R[2], covs[keep], R[2]), 0.))
    del means, covs
    tz = mu[:, 2] + sz
    sel = (tz >= top[0]) & (tz <= top[1])
    u0, v0, u1, v1 = WINDOW
    nu, nv = int(round((u1 - u0) / step)), int(round((v1 - v0) / step))
    x = mu[sel]
    iu = np.floor((x[:, 0] - u0) / step).astype(int)
    iv = np.floor((x[:, 1] - v0) / step).astype(int)
    ok = (iu >= 0) & (iu < nu) & (iv >= 0) & (iv < nv)
    cnt = np.zeros((nu, nv), np.int32)
    np.add.at(cnt, (iu[ok], iv[ok]), 1)
    return cnt, {"window_uv": list(WINDOW), "step": step, "top_z_range": list(top), "n_selected": int(sel.sum()),
                 "n_in_window": int(ok.sum())}


def clearance_map(ctx, oracle, body, box_uv, step):
    u0, v0, u1, v1 = box_uv
    us = np.round(np.arange(u0 + step / 2, u1, step), 4)
    vs = np.round(np.arange(v0 + step / 2, v1, step), 4)
    from gmc.gs3d.contracts import Pose3
    code = np.zeros((len(us), len(vs)), np.int8)
    clr = np.full((len(us), len(vs)), np.nan, np.float32)
    for i, u in enumerate(us):
        for j, v in enumerate(vs):
            rep = oracle.pose(Pose3(tuple(map(float, ground_world(ctx["frame"], body, (u, v)))), 0.), body,
                              margin_m=GROUND_CONFIG.margin_m)
            k = "free" if rep.occupancy == "free" else ("occupied" if rep.occupancy == "occupied" else rep.reason)
            code[i, j] = CODES.get(k, 4)
            if rep.occupancy == "free":
                clr[i, j] = rep.clearance_lower_m
    return us, vs, code, clr


def cmd_survey(a):
    from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
    t0 = time.perf_counter()
    a.out.mkdir(parents=True, exist_ok=True)
    arrays = {}
    cnt, meta = near_floor_raster()
    arrays["near_floor_count"] = cnt
    out = {"schema": "aerial3dg_fail2.region_survey.v1", "host": host(), "near_floor": meta, "boxes": {}}
    print("near-floor", meta, flush=True)
    for spec in a.boxes:
        name, b = spec.split(":")
        box_uv = tuple(float(x) for x in b.split(","))
        w0 = time.perf_counter()
        try:
            ctx = load_booth(_box(box_uv))
        except ValueError as exc:                        # e.g. the floor-support contract refuses the box
            out["boxes"][name] = {"box_uv": box_uv, "error": str(exc)}
            print(name, "ERROR", exc, flush=True)
            continue
        oracle = GaussianBodyOracle(PreparedScene(ctx["scene"]))
        rec = {"box_uv": box_uv, "crop": ctx["crop"], "support": ctx["support_evidence"],
               "area_m2": (box_uv[2] - box_uv[0]) * (box_uv[3] - box_uv[1]), "robots": {}}
        for rname in a.robots:
            us, vs, code, clr = clearance_map(ctx, oracle, ROBOTS[rname], box_uv, a.step)
            arrays[f"{name}_{rname}_code"], arrays[f"{name}_{rname}_clr"] = code, clr
            arrays[f"{name}_us"], arrays[f"{name}_vs"] = us, vs
            n = code.size
            c = np.nan_to_num(clr, nan=-1.)
            rec["robots"][rname] = {"grid_points": int(n),
                                    "status_share": {k: float((code == v).mean()) for k, v in CODES.items()},
                                    "clearance_ge_share": {f"{t:g}": float((c >= t).mean())
                                                           for t in (.001, .002, .01, .02, .05, .099)}}
            print(name, rname, json.dumps(rec["robots"][rname]), flush=True)
        rec["wall_s"] = time.perf_counter() - w0
        out["boxes"][name] = rec
        print(name, "crop", ctx["crop"].get("selected_supports", ctx["crop"]), round(rec["wall_s"], 1), flush=True)
        del ctx, oracle
        np.savez_compressed(a.out / "survey.npz", **arrays)
        _dump(a.out / "survey.json", out)
    out["wall_s"] = time.perf_counter() - t0
    np.savez_compressed(a.out / "survey.npz", **arrays)
    _dump(a.out / "survey.json", out)


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("survey")
    s.add_argument("--boxes", nargs="+", required=True, help="NAME:U0,V0,U1,V1")
    s.add_argument("--robots", nargs="+", default=["cylinder"])
    s.add_argument("--step", type=float, default=.05)
    s.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)
    {"survey": cmd_survey}[a.cmd](a)


if __name__ == "__main__":
    main()
