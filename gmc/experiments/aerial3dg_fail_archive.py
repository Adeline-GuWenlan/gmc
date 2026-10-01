"""F1 Task 1: what lies around the G2 compile box in the FULL uav-lamp archive (no planner, no compile).

Run from ``gmc/`` with ``PYTHONPATH=src:experiments`` (sbatch: the archive is ~0.8 GB).

* ``raster``  per robot body band, a 5 cm route-frame (u, v) raster of opacity>tau Gaussians whose
              2-sigma z-extent overlaps the band (+margin): ``centres`` = count of means per cell,
              ``cover`` = count of 2-sigma route-frame xy AABBs covering the cell.  Over a window much
              wider than the G2 box, so walls / structures outside v in [-0.35, 2.75] are visible.
              Also: crop sizes (G2's crop rule, ``crop_by_support_aabb``) for widened boxes, to size
              the widened compiles before running them.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np

from aerial3dg_run import ARCHIVE, MANIFEST, ROBOTS, GROUND_CONFIG
from gmc.gs3d.integration import RouteBoxKnownSpace

TAU, LEVEL = .3, 2.
G2_BOX = (-9.0, -0.35, 3.7, 2.75)
Z_TOP = 2.43
WINDOW = (-13.0, -5.0, 7.7, 8.0)          # u0 v0 u1 v1 of the raster
STEP = .05
PADS = [(0., 0.), (0., 1.), (0., 2.), (1., 1.), (2., 2.), (1., 2.)]   # (pad_u, pad_v) each side


def padded(box, pu, pv):
    u0, v0, u1, v1 = box
    return (u0 - pu, v0 - pv, u1 + pu, v1 + pv)


def cmd_raster(a):
    t0 = time.perf_counter()
    man = json.loads(MANIFEST.read_text())
    R = np.asarray(man["frame"]["world_to_route"], float)
    o = np.asarray(man["frame"]["origin_world_m"], float)
    with np.load(ARCHIVE, allow_pickle=False) as d:
        means, covs, op = d["means"], d["covs"], d["opacity"]
    load_s = time.perf_counter() - t0
    keep = op > TAU
    n_all, n_op = len(op), int(keep.sum())
    # crop sizes for widened boxes: G2's rule = 2-sigma WORLD AABB overlaps the prism's world AABB
    sd_w = LEVEL * np.sqrt(np.maximum(np.diagonal(covs, axis1=1, axis2=2), 0.))
    lo_w, hi_w = means - sd_w, means + sd_w
    crops = []
    for pu, pv in PADS:
        b = padded(G2_BOX, pu, pv)
        ks = RouteBoxKnownSpace(tuple(o), tuple(map(tuple, R)), (b[0], b[1], 0.), (b[2], b[3], Z_TOP))
        bmin, bmax = map(np.asarray, ks.world_bounds())
        sel = keep & np.all(hi_w >= bmin, axis=1) & np.all(lo_w <= bmax, axis=1)
        crops.append({"pad_u": pu, "pad_v": pv, "box_uv": list(b), "selected_supports": int(sel.sum())})
        print("crop", crops[-1], flush=True)
    del sd_w, lo_w, hi_w
    mu = (means[keep] - o) @ R.T
    C = covs[keep]
    del means, covs
    # route-frame variances: diag(R C R^T)
    var = np.empty((len(C), 3))
    for k in range(3):
        var[:, k] = np.einsum("j,njk,k->n", R[k], C, R[k])
    del C
    sd = LEVEL * np.sqrt(np.maximum(var, 0.))
    u0, v0, u1, v1 = WINDOW
    nu, nv = int(round((u1 - u0) / STEP)), int(round((v1 - v0) / STEP))
    out = {"window_uv": list(WINDOW), "step": STEP, "nu": nu, "nv": nv}
    arrays = {}
    for name, body in ROBOTS.items():
        m = GROUND_CONFIG.margin_m
        zlo, zhi = body.ground_clearance_m - m, body.ground_clearance_m + 2 * body.half_height_m + m
        inband = (mu[:, 2] + sd[:, 2] >= zlo) & (mu[:, 2] - sd[:, 2] <= zhi)
        x, s = mu[inband], sd[inband]
        iu = np.floor((x[:, 0] - u0) / STEP).astype(int)
        iv = np.floor((x[:, 1] - v0) / STEP).astype(int)
        ok = (iu >= 0) & (iu < nu) & (iv >= 0) & (iv < nv)
        cen = np.zeros((nu, nv), np.int32)
        np.add.at(cen, (iu[ok], iv[ok]), 1)
        # 2-sigma xy AABB footprint coverage via a 2-D difference array
        a0 = np.clip(np.floor((x[:, 0] - s[:, 0] - u0) / STEP).astype(int), 0, nu)
        a1 = np.clip(np.floor((x[:, 0] + s[:, 0] - u0) / STEP).astype(int) + 1, 0, nu)
        b0 = np.clip(np.floor((x[:, 1] - s[:, 1] - v0) / STEP).astype(int), 0, nv)
        b1 = np.clip(np.floor((x[:, 1] + s[:, 1] - v0) / STEP).astype(int) + 1, 0, nv)
        g = (a1 > a0) & (b1 > b0)
        D = np.zeros((nu + 1, nv + 1), np.int64)
        np.add.at(D, (a0[g], b0[g]), 1)
        np.add.at(D, (a1[g], b0[g]), -1)
        np.add.at(D, (a0[g], b1[g]), -1)
        np.add.at(D, (a1[g], b1[g]), 1)
        cover = D.cumsum(0).cumsum(1)[:nu, :nv].astype(np.int32)
        arrays[f"{name}_centres"], arrays[f"{name}_cover"] = cen, cover
        out[name] = {"band_z": [zlo, zhi], "gaussians_in_band": int(inband.sum()),
                     "in_band_in_window": int(ok.sum())}
        print(name, out[name], flush=True)
    # full-height (floor to soffit) for context: walls
    inband = (mu[:, 2] + sd[:, 2] >= .2) & (mu[:, 2] - sd[:, 2] <= 2.0)
    x = mu[inband]
    iu = np.floor((x[:, 0] - u0) / STEP).astype(int)
    iv = np.floor((x[:, 1] - v0) / STEP).astype(int)
    ok = (iu >= 0) & (iu < nu) & (iv >= 0) & (iv < nv)
    wall = np.zeros((nu, nv), np.int32)
    np.add.at(wall, (iu[ok], iv[ok]), 1)
    arrays["z020_200_centres"] = wall
    # the u~-5.6 structure: every in-window Gaussian with u in [-6.2, -5.0] (route xyz + 2-sigma)
    sel = (mu[:, 0] > -6.2) & (mu[:, 0] < -5.0) & (mu[:, 1] > v0) & (mu[:, 1] < v1) & (mu[:, 2] < 2.5)
    arrays["gap_mu"], arrays["gap_sd"] = mu[sel].astype(np.float32), sd[sel].astype(np.float32)
    out.update(archive_supports=n_all, opacity_selected=n_op, crops=crops, load_s=load_s,
               wall_s=time.perf_counter() - t0,
               definition="route frame = manifest site frame; raster of opacity>0.3 Gaussians whose 2-sigma z-extent "
                          "overlaps the robot band [clearance - margin, clearance + 2 half_height + margin]")
    a.out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(a.out / "archive_raster.npz", **arrays)
    (a.out / "archive_raster.json").write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps({k: out[k] for k in ("archive_supports", "opacity_selected", "wall_s")}), flush=True)


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("raster")
    r.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)
    {"raster": cmd_raster}[a.cmd](a)


if __name__ == "__main__":
    main()
