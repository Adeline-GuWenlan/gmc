"""bl B2: the shared rasteriser -- one conservative 2D C-space map per robot x region for the map-based baselines.

Plan §3.1. Input: the harness's scene export (``outputs/baselines/scene/<R>_<robot>.npz``) = the judge's own obstacle
set (opacity > tau, covariances as the judge floors them) in the plan frame (u, v, height above the floor), level 2,
margin 0.001, body, z_c, the known route box and the plan->world frame.

A grid cell is FREE only if every point p of the (closed) cell is a pose the judge can pass, for three reasons that
mirror ``GaussianBodyOracle.edge`` at a pose (``oracle.py``):

1. Gaussians.  The judge's body is the vertical cylinder B(p) = disk(p, r) x [z_c - h, z_c + h]; it is free of a
   Gaussian iff its clearance to the level-sigma ellipsoid E exceeds the margin m.  dist(B(p), E) <= m implies that
   E meets B(p) (+) ball(m), which lies inside disk(p, r + m) x Z with the slab Z = [z_c - h - m, z_c + h + m]; so
   it implies dist(p, S) <= r + m with S = xy-projection of (E n Z).  We mark occupied every cell that meets
   S (+) disk(rho), rho = r + m + pad_m -- a superset of every pose the judge could call occupied/unproven.
   S is convex.  Its support function in a planar direction d is exact in closed form: with E = mu + L A w,
   |w| <= 1 (A A^T = Sigma), the slab is t_lo <= n.w <= t_hi with n = A_z / |A_z|, and
       h_S(d) = d.mu_xy + max_{t in [t_lo, t_hi]} ( alpha t + beta sqrt(1 - t^2) ),
       |c|^2 = L^2 d^T Sigma_xy d,  alpha = L (d.Sigma_xy,z) / sqrt(Sigma_zz),  beta^2 = |c|^2 - alpha^2,
   maximised at t = clip(alpha / |c|, t_lo, t_hi) (the function is concave in t).  The slab cut is what keeps a
   floor splat whose 2-sigma top just reaches the 2 cm chassis clearance (judge: z_c - h - m = 0.019 m) from
   blocking its whole footprint: only its cap above 0.019 m counts, exactly as the judge counts it, and a splat
   whose top is below 0.019 m is not an obstacle at all (it can never be within the margin of the chassis bottom).
   The cell test is the intersection of K half-planes d_k.x <= h_S(d_k) + rho + delta_k, delta_k = res/2 (|d_kx| +
   |d_ky|) (the cell's own support): a cell is marked if its centre satisfies all K, i.e. if the square meets every
   half-plane of the outer K-gon of S (+) disk(rho) -- a superset of "the square meets S (+) disk(rho)".  So a FREE
   cell has some k with d_k.c > h_S(d_k) + rho + delta_k, hence d_k.p > h_S(d_k) + rho for every p in the cell, hence
   dist(p, S) > rho.  Conservatism added on top of the judge: the K-gon's corners (<= (1/cos(pi/K) - 1) x the local
   radius), delta_k (<= res/sqrt(2)), the rounded-vs-square cylinder edge (the (r+m) x (h+m) box around B (+) ball(m)),
   and pad_m.
2. Known space.  ``RouteBoxKnownSpace.contains_swept_cylinder``: route-frame centre +- (r, r, h) inside the known
   prism (no margin).  For a cell: all four corners (the condition is linear in p).
3. Workspace bounds.  The judge also needs the body's WORLD AABB (centre +- (r, r, h)) to stay more than the margin
   (+ its numerical slack) inside the scene bounds = the world AABB of the known prism
   (``RouteBoxKnownSpace.world_bounds``).  In a rotated frame that bites near the prism's corners.  The condition is
   a min of linear functions of p (concave), so checking the four cell corners is exact for the cell.
   The floor support's evidence box is the same world box (no margin) and is implied by 3.

``build`` runs under gmc-venv (numpy + scipy for the component labels); ``load`` (SHA-checked) is numpy only, so every
method env can read the same bytes.  The artifact ``outputs/baselines/raster/<R>_<robot>_r<res_mm>.npz`` (+ ``.json``
with SHA-256 and the build time, which is part of each map-based method's setup ("compile") time).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import time
from itertools import product
from pathlib import Path

import numpy as np

SCHEMA = "bl.raster.v1"
EPS_M = 1e-9                  # absolute guard on every analytic support value (float round-off)
DEFAULTS = {"res_m": 0.005, "n_dirs": 16, "pad_m": 0.0}    # n_dirs: sampled angles per half-circle before refining


# ============================================================================================ geometry (numpy only)
def directions(k):
    """K unit directions offset by half a step, so no direction has a zero x component."""
    th = (np.arange(k) + .5) * 2 * np.pi / k
    return np.c_[np.cos(th), np.sin(th)]


def support_params(means, covs, level, zlo, zhi):
    """Per-Gaussian constants of the closed-form support of S = xy-projection of (E n slab); ``keep`` = E meets it."""
    means, covs = np.asarray(means, float), np.asarray(covs, float)
    L = float(level)
    szz = covs[:, 2, 2]
    if np.any(szz <= 0):
        raise ValueError("non-positive vertical variance")
    sz = np.sqrt(szz)
    tlo = (zlo - means[:, 2]) / (L * sz)
    thi = (zhi - means[:, 2]) / (L * sz)
    keep = (tlo <= 1.) & (thi >= -1.)
    P = {"mx": means[:, 0], "my": means[:, 1], "sxx": L * L * covs[:, 0, 0], "sxy": L * L * covs[:, 0, 1],
         "syy": L * L * covs[:, 1, 1], "axz": L * covs[:, 0, 2] / sz, "ayz": L * covs[:, 1, 2] / sz,
         "tlo": np.clip(tlo, -1., 1.), "thi": np.clip(thi, -1., 1.)}
    return P, keep


def support_eval(P, c, s):
    """h_S(d) for d = (c, s); P entries broadcast against c, s (e.g. P (n, 1), c (n, m))."""
    c2 = c * c * P["sxx"] + 2 * c * s * P["sxy"] + s * s * P["syy"]          # |L A_xy^T d|^2
    alpha = c * P["axz"] + s * P["ayz"]
    c2 = np.maximum(c2, alpha * alpha)                 # |c| >= |alpha| (Cauchy-Schwarz; round-off only)
    beta = np.sqrt(np.maximum(c2 - alpha * alpha, 0.))
    cn = np.sqrt(c2)
    ts = np.divide(alpha, cn, out=np.zeros_like(cn), where=cn > 0)
    t = np.clip(ts, P["tlo"], P["thi"])
    H = c * P["mx"] + s * P["my"] + alpha * t + beta * np.sqrt(np.maximum(1. - t * t, 0.))
    return H + EPS_M + 64 * np.finfo(float).eps * (np.abs(c * P["mx"]) + np.abs(s * P["my"]) + cn)


def slab_support(means, covs, level, zlo, zhi, dirs):
    """Support h_S(d_k) of S = xy-projection of ({x: (x-mu)^T C^-1 (x-mu) <= level^2} n {zlo <= z <= zhi}).

    Returns (H (n, K), keep (n,)): rows of Gaussians whose ellipsoid misses the slab are ``keep = False``."""
    P, keep = support_params(means, covs, level, zlo, zhi)
    dirs = np.asarray(dirs, float)
    return support_eval({k: v[:, None] for k, v in P.items()}, dirs[:, 0][None, :], dirs[:, 1][None, :]), keep


def _hc(P, th, rho, res):
    """Support of C = S (+) disk(rho) (+) cell square [-res/2, res/2]^2 at angle(s) th."""
    c, s = np.cos(th), np.sin(th)
    return support_eval(P, c, s) + rho + res / 2 * (np.abs(c) + np.abs(s))


GOLDEN_ITERS = 16


def _chord_end(P, y, rho, res, side, k):
    """Rigorous bound on the right (side=+1) / left (side=-1) end of the chord {x: (x, y) in C} for every row y
    (P (n, 1), y (n, m)).  Every angle th with side * cos th > 0 gives the bound (h_C(th) - y sin th) / cos th
    (the support line meets the row there); the exact end is the extremum over th.  We take the best of k sampled
    angles, then golden-section refine in the bracket around it, keeping the best value ever evaluated -- so the
    bound is sound whatever the refinement does, and tight when the bound is unimodal in th (convex C)."""
    base = 0. if side > 0 else np.pi
    step = np.pi / k
    th_k = base - np.pi / 2 + (np.arange(k) + .5) * step                         # strictly inside the half-circle
    th = np.broadcast_to(th_k, y.shape[:-1] + (y.shape[-1], k))

    def f(t):
        return side * (_hc({kk: v[..., None] for kk, v in P.items()}, t, rho, res) - y[..., None] * np.sin(t)) / np.cos(t)

    F = f(th)                                                                     # (n, m, k): side*x bounds
    j = np.argmin(F, axis=-1)
    best = np.take_along_axis(F, j[..., None], -1)[..., 0]
    lo = np.maximum(base - np.pi / 2 + 1e-9, th_k[j] - step)
    hi = np.minimum(base + np.pi / 2 - 1e-9, th_k[j] + step)
    g = (math.sqrt(5) - 1) / 2
    a, b = hi - g * (hi - lo), lo + g * (hi - lo)
    fa, fb = f(a[..., None])[..., 0], f(b[..., None])[..., 0]
    for _ in range(GOLDEN_ITERS):
        left = fa < fb
        hi = np.where(left, b, hi)
        lo = np.where(left, lo, a)
        best = np.minimum(best, np.minimum(fa, fb))
        a_new = hi - g * (hi - lo)
        b_new = lo + g * (hi - lo)
        a, b = np.where(left, a_new, b), np.where(left, a, b_new)
        fa_new = f(np.where(left, a_new, b_new)[..., None])[..., 0]
        fa, fb = np.where(left, fa_new, fb), np.where(left, fa, fa_new)
    best = np.minimum(best, np.minimum(fa, fb))
    return side * best


def mark_gaussians(occ_count, P, u0, v0, res, rho, k=16, chunk_rows=60_000):
    """Add 1 to every cell (iy, ix) that meets S (+) disk(rho) for each Gaussian in P (support params, kept only):
    the cell centre lies in C = S (+) disk(rho) (+) square, row by row between the two chord ends.  Returns the number
    of Gaussians that marked at least one cell."""
    ny, nx = occ_count.shape
    n = len(P["mx"])
    if not n:
        return 0
    Pc = {kk: v[:, None] for kk, v in P.items()}
    ymax = _hc(Pc, np.full((n, 1), np.pi / 2), rho, res)[:, 0] + 1e-7          # exact y extent of C (+ guard)
    ymin = -_hc(Pc, np.full((n, 1), -np.pi / 2), rho, res)[:, 0] - 1e-7
    iy_lo = np.maximum(np.ceil((ymin - v0) / res - .5 - 1e-9).astype(np.int64), 0)
    iy_hi = np.minimum(np.floor((ymax - v0) / res - .5 + 1e-9).astype(np.int64), ny - 1)
    span = iy_hi - iy_lo + 1
    live = np.nonzero(span > 0)[0]
    diff = np.zeros((ny, nx + 1), np.int32)
    touched = 0
    for smax in np.unique(span[live]):
        grp = live[span[live] == smax]
        step = max(1, chunk_rows // int(smax))
        for c0 in range(0, len(grp), step):
            idx = grp[c0:c0 + step]
            rows = iy_lo[idx][:, None] + np.arange(int(smax))[None, :]
            y = v0 + (rows + .5) * res
            Pi = {kk: v[idx][:, None] for kk, v in P.items()}
            xmax = _chord_end(Pi, y, rho, res, +1, k)
            xmin = _chord_end(Pi, y, rho, res, -1, k)
            ix_lo = np.maximum(np.ceil((xmin - u0) / res - .5 - 1e-9), 0)
            ix_hi = np.minimum(np.floor((xmax - u0) / res - .5 + 1e-9), nx - 1)
            ok = ix_lo <= ix_hi
            if ok.any():
                r_ = rows[ok]
                np.add.at(diff, (r_, ix_lo[ok].astype(np.int64)), 1)
                np.add.at(diff, (r_, ix_hi[ok].astype(np.int64) + 1), -1)
                touched += int(ok.any(axis=1).sum())
    occ_count += np.cumsum(diff, axis=1)[:, :nx]
    return touched


def world_bounds(meta):
    """``RouteBoxKnownSpace.world_bounds``: world AABB of the known route prism's 8 corners."""
    R = np.asarray(meta["frame"]["world_to_plan"], float)
    o = np.asarray(meta["frame"]["origin_world_m"], float)
    lo, hi = np.asarray(meta["known_route_lower_m"], float), np.asarray(meta["known_route_upper_m"], float)
    corners = np.asarray(list(product(*zip(lo, hi))), float)
    world = corners @ R + o
    return world.min(axis=0), world.max(axis=0)


def numerical_slack(*values):
    """``gmc.gs3d.geometry.numerical_slack`` (ABS_SLACK_M 1e-8), re-stated so this module stays numpy-only."""
    scale = max((float(np.max(np.abs(v))) for v in values if np.size(v)), default=1.)
    return 1e-8 + 128 * np.finfo(float).eps * max(1., scale)


def workspace_margin(meta, body, uv):
    """The judge's ``boundary`` at body centres ``uv`` (plan frame, at z_c): min distance of the body's world AABB to
    the scene's world bounds, minus the judge's numerical slack.  Free requires > margin."""
    R = np.asarray(meta["frame"]["world_to_plan"], float)
    o = np.asarray(meta["frame"]["origin_world_m"], float)
    blo, bhi = world_bounds(meta)
    uv = np.atleast_2d(np.asarray(uv, float))
    P = np.c_[uv, np.full(len(uv), float(meta["z_c"]))] @ R + o
    half = np.array([body["radius_m"], body["radius_m"], body["half_height_m"]], float)
    lower, upper = P - half, P + half
    b = np.minimum((lower - blo).min(axis=1), (bhi - upper).min(axis=1))
    slack = np.array([numerical_slack(lower[i], upper[i], blo, bhi) for i in range(len(uv))])
    return b - slack


def known_ok(meta, body, uv):
    """``RouteBoxKnownSpace.contains_swept_cylinder`` at a pose (route frame == plan frame, checked at export)."""
    uv = np.atleast_2d(np.asarray(uv, float))
    lo, hi = np.asarray(meta["known_route_lower_m"], float), np.asarray(meta["known_route_upper_m"], float)
    r, h, z = float(body["radius_m"]), float(body["half_height_m"]), float(meta["z_c"])
    return ((uv[:, 0] - r >= lo[0] - 1e-12) & (uv[:, 0] + r <= hi[0] + 1e-12)
            & (uv[:, 1] - r >= lo[1] - 1e-12) & (uv[:, 1] + r <= hi[1] + 1e-12)
            & (z - h >= lo[2] - 1e-12) & (z + h <= hi[2] + 1e-12))


def grid_for(meta, res):
    """Grid over the region box (= the known route box in u, v): cell (iy, ix) has centre
    (u0 + (ix + .5) res, v0 + (iy + .5) res).  Cells that stick out of the box are occupied by the known layer."""
    lo, hi = meta["known_route_lower_m"], meta["known_route_upper_m"]
    u0, v0 = float(lo[0]), float(lo[1])
    nx, ny = int(math.ceil((hi[0] - u0) / res - 1e-9)), int(math.ceil((hi[1] - v0) / res - 1e-9))
    return u0, v0, nx, ny


def cell_centres(u0, v0, res, nx, ny):
    return u0 + (np.arange(nx) + .5) * res, v0 + (np.arange(ny) + .5) * res


def corner_layer(fn, u0, v0, res, nx, ny):
    """A cell passes iff ``fn`` passes at all four of its corners (exact for linear / concave conditions)."""
    ue, ve = u0 + np.arange(nx + 1) * res, v0 + np.arange(ny + 1) * res
    U, V = np.meshgrid(ue, ve)
    ok = fn(np.c_[U.ravel(), V.ravel()]).reshape(ny + 1, nx + 1)
    return ok[:-1, :-1] & ok[1:, :-1] & ok[:-1, 1:] & ok[1:, 1:]


def build(scene, body, res_m=DEFAULTS["res_m"], n_dirs=DEFAULTS["n_dirs"], pad_m=DEFAULTS["pad_m"]):
    """The C-space map (dict of arrays + meta).  ``occ`` (ny, nx) uint8: 1 = occupied for the body's centre."""
    meta = scene["meta"]
    t0 = time.perf_counter()
    level, m = float(meta["level"]), float(meta["margin_m"])
    r, h, z_c = float(body["radius_m"]), float(body["half_height_m"]), float(meta["z_c"])
    zlo, zhi = z_c - h - m, z_c + h + m
    rho = r + m + float(pad_m)
    u0, v0, nx, ny = grid_for(meta, res_m)
    means, covs = np.asarray(scene["means"], float), np.asarray(scene["covs"], float)
    # cheap prefilter: z extent meets the slab and the xy extent (+ rho) meets the grid
    ext = level * np.sqrt(np.einsum("nii->ni", covs))
    in_slab = (means[:, 2] + ext[:, 2] >= zlo) & (means[:, 2] - ext[:, 2] <= zhi)
    reach = rho + res_m
    near = ((means[:, 0] + ext[:, 0] >= u0 - reach) & (means[:, 0] - ext[:, 0] <= u0 + nx * res_m + reach)
            & (means[:, 1] + ext[:, 1] >= v0 - reach) & (means[:, 1] - ext[:, 1] <= v0 + ny * res_m + reach))
    sel = np.nonzero(in_slab & near)[0]
    count = np.zeros((ny, nx), np.int32)
    touched, cut_partial = 0, 0
    t1 = time.perf_counter()
    for a in range(0, len(sel), 50_000):
        ids = sel[a:a + 50_000]
        P, keep = support_params(means[ids], covs[ids], level, zlo, zhi)
        mz, ez = means[ids, 2], ext[ids, 2]
        cut_partial += int(np.count_nonzero(keep & ((mz - ez < zlo) | (mz + ez > zhi))))
        touched += mark_gaussians(count, {k: v[keep] for k, v in P.items()}, u0, v0, res_m, rho, int(n_dirs))
    gauss_s = time.perf_counter() - t1
    gauss = count > 0
    known = corner_layer(lambda p: known_ok(meta, body, p), u0, v0, res_m, nx, ny)
    wsp = corner_layer(lambda p: workspace_margin(meta, body, p) > m + EPS_M, u0, v0, res_m, nx, ny)
    occ = (gauss | ~known | ~wsp).astype(np.uint8)
    out = {"occ": occ, "gauss": gauss.astype(np.uint8), "known_ok": known.astype(np.uint8),
           "workspace_ok": wsp.astype(np.uint8)}
    lab_s = None
    try:
        from scipy import ndimage
        t2 = time.perf_counter()
        lab, n = ndimage.label(occ, structure=np.ones((3, 3), int))     # 8-connected occupied components
        # 8-connected free components: the segment between the centres of two diagonal free cells only crosses
        # their shared corner, which belongs to both closed cells, so diagonal grid moves are judge-free too.
        free_lab, nf = ndimage.label(1 - occ, structure=np.ones((3, 3), int))
        out["occ_label"], out["free_label"] = lab.astype(np.int32), free_lab.astype(np.int32)
        lab_s = time.perf_counter() - t2
    except ImportError:
        n = nf = None
    info = {"schema": SCHEMA, "region": meta["region"], "robot": meta["robot"], "res_m": float(res_m),
            "n_dirs": int(n_dirs), "pad_m": float(pad_m), "rho_m": rho, "slab_z": [zlo, zhi], "u0": u0, "v0": v0,
            "nx": nx, "ny": ny, "level": level, "margin_m": m, "tau": float(meta["tau"]),
            "body": dict(body), "z_c": z_c, "world_bounds": [list(map(float, b)) for b in world_bounds(meta)],
            "n_gaussians_input": int(len(means)), "n_in_slab_and_near": int(len(sel)),
            "n_cut_by_slab": cut_partial, "n_touching_grid": touched,
            "cells": int(nx * ny), "occupied_cells": int(occ.sum()), "gauss_cells": int(gauss.sum()),
            "known_fail_cells": int((~known).sum()), "workspace_fail_cells": int((~wsp).sum()),
            "occupied_components_8conn": None if n is None else int(n),
            "free_components_8conn": None if nf is None else int(nf),
            "gauss_wall_s": gauss_s, "label_wall_s": lab_s, "build_wall_s": time.perf_counter() - t0,
            "scene_export_sha256": scene.get("_sha256")}
    return out, info


# ============================================================================================ persistence
def raster_path(region, robot, res_m, root="outputs/baselines/raster", n_dirs=None, pad_m=None):
    tag = f"r{res_m * 1000:g}mm"
    if n_dirs not in (None, DEFAULTS["n_dirs"]):
        tag += f"_k{n_dirs}"
    if pad_m not in (None, 0., DEFAULTS["pad_m"]):
        tag += f"_p{pad_m * 1000:g}mm"
    return Path(root) / f"{region}_{robot}_{tag}.npz"


def _sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 22), b""):
            h.update(b)
    return h.hexdigest()


def save(path, arrays, info):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + f".tmp{os.getpid()}.npz")
    np.savez_compressed(tmp, info=json.dumps(info), **arrays)
    os.replace(tmp, path)
    info = dict(info, sha256=_sha(path), bytes=path.stat().st_size,
                bl_raster_sha256=_sha(Path(__file__).resolve()))
    Path(f"{path}.json").write_text(json.dumps(info, indent=1) + "\n")
    return info


def load(path, expect_sha256=None):
    """Raster arrays + info; fail closed if the bytes differ from the sidecar (or from ``expect_sha256``)."""
    path = Path(path)
    side = json.loads(Path(f"{path}.json").read_text())
    sha = _sha(path)
    if sha != side["sha256"] or (expect_sha256 and sha != expect_sha256):
        raise ValueError(f"raster {path} differs from its recorded SHA-256")
    with np.load(path, allow_pickle=False) as d:
        arrays = {k: d[k] for k in d.files if k != "info"}
    return arrays, side


def to_index(info, uv):
    """(iy, ix) of the cell containing plan point(s) ``uv`` (may be out of range)."""
    uv = np.atleast_2d(np.asarray(uv, float))
    ix = np.floor((uv[:, 0] - info["u0"]) / info["res_m"]).astype(np.int64)
    iy = np.floor((uv[:, 1] - info["v0"]) / info["res_m"]).astype(np.int64)
    return iy, ix


def centre(info, iy, ix):
    return np.c_[info["u0"] + (np.asarray(ix) + .5) * info["res_m"], info["v0"] + (np.asarray(iy) + .5) * info["res_m"]]


def occupied_at(arrays, info, uv):
    """Map occupancy at plan points; out-of-grid points are occupied."""
    iy, ix = to_index(info, uv)
    occ = arrays["occ"]
    inside = (iy >= 0) & (iy < occ.shape[0]) & (ix >= 0) & (ix < occ.shape[1])
    out = np.ones(len(iy), bool)
    out[inside] = occ[iy[inside], ix[inside]] > 0
    return out


def nearest_free(arrays, info, uv, max_m=1.0):
    """(iy, ix, centre, distance) of the free cell whose centre is nearest to ``uv``, searching a window of growing
    half-width up to ``max_m``; None if there is none that close.  A window hit at half-width w is final once the
    best distance is <= w * res (no cell outside the window can be nearer)."""
    occ = arrays["occ"]
    ny, nx = occ.shape
    res = info["res_m"]
    p = np.asarray(uv, float)
    iy0, ix0 = (int(v[0]) for v in to_index(info, p))
    w = 4
    while True:
        y0, y1, x0, x1 = max(iy0 - w, 0), min(iy0 + w + 1, ny), max(ix0 - w, 0), min(ix0 + w + 1, nx)
        if y0 < y1 and x0 < x1:
            fy, fx = np.nonzero(occ[y0:y1, x0:x1] == 0)
            if len(fy):
                c = centre(info, fy + y0, fx + x0)
                d = np.linalg.norm(c - p[None, :], axis=1)
                k = int(np.argmin(d))
                if d[k] <= w * res or w * res >= max_m:
                    return (int(fy[k] + y0), int(fx[k] + x0), c[k], float(d[k])) if d[k] <= max_m else None
        if w * res >= max_m:
            return None
        w *= 2


def segment_cells(info, a, b):
    """All (iy, ix) cells a straight segment a -> b passes through (supercover, via dense sampling at res/8 plus the
    exact endpoints; used for reporting only)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(2, int(math.ceil(np.linalg.norm(b - a) / (info["res_m"] / 8))) + 1)
    t = np.linspace(0., 1., n)[:, None]
    return to_index(info, a + t * (b - a))


def polyline_hits(arrays, info, P):
    """Fraction-free stats of a polyline against the map: number of sample points in occupied cells (res/8)."""
    P = np.asarray(P, float)
    hits, total = 0, 0
    occ = arrays["occ"]
    for a, b in zip(P[:-1], P[1:]):
        iy, ix = segment_cells(info, a, b)
        inside = (iy >= 0) & (iy < occ.shape[0]) & (ix >= 0) & (ix < occ.shape[1])
        o = np.ones(len(iy), bool)
        o[inside] = occ[iy[inside], ix[inside]] > 0
        hits += int(o.sum())
        total += len(o)
    return hits, total


# ============================================================================================ CLI (gmc-venv)
def load_scene(path):
    with np.load(path, allow_pickle=False) as d:
        scene = {k: d[k] for k in d.files if k != "meta"}
        scene["meta"] = json.loads(str(d["meta"]))
    scene["_sha256"] = _sha(path)
    side = json.loads(Path(f"{path}.json").read_text())
    if side["sha256"] != scene["_sha256"]:
        raise ValueError(f"scene export {path} differs from its sidecar hash")
    return scene


def cmd_build(a):
    for region in a.regions:
        for robot in a.robots:
            for res in a.res:
                out = raster_path(region, robot, res, n_dirs=a.n_dirs, pad_m=a.pad)
                if out.exists() and not a.force:
                    print("exists", out, flush=True)
                    continue
                scene = load_scene(Path("outputs/baselines/scene") / f"{region}_{robot}.npz")
                arrays, info = build(scene, scene["meta"]["body"], res, a.n_dirs, a.pad)
                info = save(out, arrays, info)
                print(region, robot, f"res {res * 1000:g} mm", f"{info['nx']}x{info['ny']}",
                      f"occ {info['occupied_cells'] / info['cells']:.3f}", "comp", info["occupied_components_8conn"],
                      f"{info['build_wall_s']:.1f}s", out, flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build")
    b.add_argument("--regions", nargs="+", default=["WWEST", "GAPW1", "S"])
    b.add_argument("--robots", nargs="+", default=["cylinder", "sweeper"])
    b.add_argument("--res", nargs="+", type=float, default=[DEFAULTS["res_m"]])
    b.add_argument("--n-dirs", type=int, default=DEFAULTS["n_dirs"])
    b.add_argument("--pad", type=float, default=DEFAULTS["pad_m"])
    b.add_argument("--force", action="store_true")
    b.set_defaults(fn=cmd_build)
    a = ap.parse_args(argv)
    a.fn(a)


if __name__ == "__main__":
    main()
