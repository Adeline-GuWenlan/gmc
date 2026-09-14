# gmc/experiments/percase_search_cylinder.py
"""Amendment 2 case search for the cylinder robot (disc r = 0.30 m, band 0.02-1.75 m above the floor).

One sbatch job (gmc/hpc/percase_search.sbatch), run from gmc/ with PYTHONPATH=src:experiments.

1. Scene-wide certified raster of the cylinder map: project_scene on an exact pre-subset (opaque splats whose
   rho-AABB meets the extent + 2 m and the band +- 0.2 m), showcase_scene._support_raster at 2.5 cm, distance
   transform.  A window's raster is this raster cropped with its border marked occupied, so its distance map is
   min(cropped distance, distance to the border).
2. Window scan (selection aid only): windows of 3-6 m on a 0.5 m grid inside the building (floor and ceiling splat
   coverage), away from the glass doors, in ascending order of estimated support count. In each window, find clear
   lattice points (dist >= 0.40) and pairs in one free component whose straight segment is blocked (the disc overlaps
   occupied cells by >= 0.10 m), with bottleneck clearance from thresholded components and 3D heights of the blocker.
3. Distinct candidates (one per blocker location): geodesic detour on the raster, tightened window, then the
   amendment pre-check on the window's own project_scene map. The top candidates get a full-scene recheck by
   primitive ids.
4. showcase_run.py's probe (2 x 2 m at the start-goal midpoint, because the case has no overhang) for the top
   candidates. Each probe runs in a spawned child that is killed after --probe-timeout.

Writes results/height/percase/cylinder/search.json (partial after every phase) and figs/."""
import argparse
import json
import multiprocessing as mp
import os
import time
import traceback
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
from scipy import ndimage
from scipy.spatial import cKDTree

from gmc.height.ply3d import GaussianScene3D
from gmc.height.prism import robot_table
from gmc.height.project import project_scene
from showcase_scene import RHO, TAU, _section, _support_raster, load_processed

OUT_DEFAULT = "results/height/percase/cylinder"
CELL = 0.025
EXTENT = [-4.5, -1.0, 21.5, 38.0]
BAND = (0.02, 1.75)
R = 0.30
CLEAR_MIN = R + 0.05                 # amendment pre-check: start/goal clearance >= r + 0.05
LATTICE_CLEAR = 0.40                 # scan lattice points: a little more than the pre-check needs
BLOCK_DEPTH = 0.10                   # straight segment "blocked": disc overlaps occupied cells by >= 0.10 m
THRESH = (0.30, 0.325, 0.35, 0.375, 0.40, 0.45, 0.50)
XY_MARGIN, Z_MARGIN = 2.0, 0.2
GLASS_ZONE = (-4.9, -0.1, -2.9, 3.6)  # glass doors x ~ -3.9, y 0.9-2.6, grown by 1 m (as the UAV search)
SIZES = [(w, h) for w in (3.0, 3.5, 4.0, 5.0, 6.0) for h in (3.0, 3.5, 4.0, 5.0, 6.0)]
STEP = 0.5
COV = 0.5                            # coverage / density cells
FLOOR_MIN, CEIL_MIN = 5, 5           # opaque splats per 0.5 m cell: |h| < 0.20 (floor), h > 3.0 (ceiling)
LMIN = 2.0
MAXP = 70
MIN_COMP_AREA = 1.0
PAD = 0.75
SUP_CAP = 400_000
PROBE_HALF, ABORT_HOURS = 1.0, 20.0
CLAIMS = ("Per-robot showcase case chosen for success: not a morphology comparison at one place, and no "
          "'body shape changes the route' claim may rest on it; floor-surface rule (Amendment 1) on.")
TABLES = {  # A4 table_frames.py: centre, long-axis angle (deg), length, width (2-98 % extents)
    "A": ((8.96, 7.29), -22.9, 2.55, 0.72), "C": ((13.28, 14.09), -21.4, 1.74, 0.68),
    "B": ((-0.45, 10.81), 60.9, 2.35, 0.93), "G": ((3.35, 22.53), 31.5, 1.64, 0.62)}
OBJECTS = [("SW-hall bench", (2.1, 10.5), 1.3), ("hall bench", (10.0, 16.0), 1.3), ("north bench", (7.8, 28.5), 1.3),
           ("plinth box", (-1.2, 9.0), 0.8), ("plinth box", (3.8, 6.5), 0.8), ("plinth box", (8.8, 4.2), 0.8),
           ("plinth box", (12.3, 11.5), 0.8), ("plinth box", (2.2, 16.5), 0.8), ("L-shaped column", (2.9, 16.2), 1.3),
           ("small table D", (5.8, 18.0), 0.9), ("object E", (11.0, 28.0), 1.0), ("object F", (15.5, 10.0), 1.0),
           ("round tables", (12.0, 34.0), 1.5), ("SE platform", (12.7, 5.7), 1.2),
           ("reception counter", (-0.2, 1.25), 2.0)]


def _j(o):
    if isinstance(o, np.bool_):
        return bool(o)
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


def dump(path, obj):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(obj, indent=1, default=_j))
    tmp.replace(path)


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def _cell(p, win, shape):
    return (min(max(int((p[0] - win[0]) / CELL), 0), shape[0] - 1),
            min(max(int((p[1] - win[1]) / CELL), 0), shape[1] - 1))


def _r(x, k=3):
    return [round(float(v), k) for v in x]


def name_blocker(xy):
    xy = np.asarray(xy, float)
    for tag, (c, ang, ln, wd) in TABLES.items():
        u = np.array([np.cos(np.radians(ang)), np.sin(np.radians(ang))])
        v = np.array([-u[1], u[0]])
        rel = xy - np.asarray(c)
        du = max(abs(rel @ u) - ln / 2 - 0.2, 0.0)
        dv = max(abs(rel @ v) - wd / 2 - 0.2, 0.0)
        if np.hypot(du, dv) <= 0.5:
            return f"table {tag}"
    for name, c, rad in OBJECTS:
        if np.hypot(*(xy - np.asarray(c))) <= rad:
            return f"{name} at ({c[0]:g}, {c[1]:g})"
    return None


def glass_hit(win):
    g = GLASS_ZONE
    return bool(win[0] < g[2] and win[2] > g[0] and win[1] < g[3] and win[3] > g[1])


# ----------------------------------------------------------------------------------------------------------------
# context: scene-wide certified raster and support arrays

def presub_mask(lo, hi, win, z_f):
    x0, y0, x1, y1 = win
    zlo, zhi = z_f + BAND[0] - Z_MARGIN, z_f + BAND[1] + Z_MARGIN
    return ((hi[:, 0] >= x0 - XY_MARGIN) & (lo[:, 0] <= x1 + XY_MARGIN) & (hi[:, 1] >= y0 - XY_MARGIN)
            & (lo[:, 1] <= y1 + XY_MARGIN) & (hi[:, 2] >= zlo) & (lo[:, 2] <= zhi))


def presub(ctx, win):
    """Exact pre-subset for project_scene on win: every opaque splat that can survive its opacity, band and
    window tests (the rho-AABB is project_scene's own superset box; the margins exceed r)."""
    return ctx["G"].subset(presub_mask(ctx["GLO"], ctx["GHI"], win, ctx["z_f"]))


def build_context(scene, LO, HI, robot, z_f, extent, S):
    opq = scene.opacity > TAU
    gm = opq & presub_mask(LO, HI, extent, z_f)
    G = scene.subset(gm)
    ctx = dict(scene=scene, LO=LO, HI=HI, G=G, GLO=LO[gm], GHI=HI[gm], robot=robot, z_f=z_f, extent=extent,
               bcache={})
    t = time.time()
    s2, st = project_scene(G, robot, extent, z_floor=z_f)
    t_proj = time.time() - t
    log("global projection", json.dumps(st), f"{t_proj:.0f}s")
    t = time.time()
    occ = _support_raster(s2, extent, CELL)
    t_ras = time.time() - t
    dist = ndimage.distance_transform_edt(~occ) * CELL
    n = len(s2.supports)
    mean = np.array([sp.mean for sp in s2.supports], dtype=np.float64).reshape(-1, 2)
    cov = np.array([sp.covariance for sp in s2.supports], dtype=np.float64).reshape(-1, 2, 2)
    lev = np.array([sp.level for sp in s2.supports], dtype=np.float64)
    pid = np.array([sp.primitive_id for sp in s2.supports], dtype=np.int64)
    del s2
    idx = np.searchsorted(scene.ids, pid)
    assert n == 0 or np.array_equal(scene.ids[idx], pid)
    ctx.update(occ=occ, dist=dist, sup_mean=mean,
               sup_hb=lev[:, None] * np.sqrt(np.stack([cov[:, 0, 0], cov[:, 1, 1]], 1)) if n else np.zeros((0, 2)),
               sup_lo3=LO[idx, :2], sup_hi3=HI[idx, :2], sup_zc=scene.means[idx, 2] - z_f,
               sup_top=HI[idx, 2] - z_f, kd=cKDTree(mean) if n else None)
    x0, y0, x1, y1 = extent
    nbx, nby = int(round((x1 - x0) / COV)), int(round((y1 - y0) / COV))
    ex, ey = x0 + COV * np.arange(nbx + 1), y0 + COV * np.arange(nby + 1)
    h = scene.means[:, 2] - z_f
    fl, ce = opq & (np.abs(h) < 0.20), opq & (h > 3.0)
    F = np.histogram2d(scene.means[fl, 0], scene.means[fl, 1], bins=[ex, ey])[0]
    C = np.histogram2d(scene.means[ce, 0], scene.means[ce, 1], bins=[ex, ey])[0]
    D = np.histogram2d(mean[:, 0], mean[:, 1], bins=[ex, ey])[0]
    ctx.update(F=F, C=C, D=D)
    S["global"] = {"presubset_splats": int(gm.sum()), "projection": st, "n_supports": n,
                   "projection_seconds": t_proj, "raster_seconds": t_ras, "raster_shape": list(occ.shape),
                   "occupied_frac": float(occ.mean()), "disc_free_frac": float((dist > R).mean()),
                   "floor_cells_ge_min": int((F >= FLOOR_MIN).sum()), "ceiling_cells_ge_min": int((C >= CEIL_MIN).sum()),
                   "interior_cells": int(((F >= FLOOR_MIN) & (C >= CEIL_MIN)).sum()), "cov_cells": int(F.size),
                   "floor_count_percentiles": {q: float(np.percentile(F, q)) for q in (10, 25, 50, 75, 90)},
                   "ceiling_count_percentiles": {q: float(np.percentile(C, q)) for q in (10, 25, 50, 75, 90)}}
    log("global raster", json.dumps({k: v for k, v in S["global"].items() if k != "projection"}))
    return ctx


def exact_count(ctx, win):
    """Supports project_scene keeps for win (both of its box tests; dedup was 0 on this scene)."""
    x0, y0, x1, y1 = win
    m, hb, lo, hi = ctx["sup_mean"], ctx["sup_hb"], ctx["sup_lo3"], ctx["sup_hi3"]
    inside = ((m[:, 0] - hb[:, 0] <= x1 + R) & (m[:, 0] + hb[:, 0] >= x0 - R)
              & (m[:, 1] - hb[:, 1] <= y1 + R) & (m[:, 1] + hb[:, 1] >= y0 - R))
    near = (lo[:, 0] <= x1 + R) & (hi[:, 0] >= x0 - R) & (lo[:, 1] <= y1 + R) & (hi[:, 1] >= y0 - R)
    return int((inside & near).sum())


# ----------------------------------------------------------------------------------------------------------------
# window scan (selection aid on the cropped scene-wide raster)

def integral(a):
    out = np.zeros((a.shape[0] + 1, a.shape[1] + 1))
    out[1:, 1:] = a.cumsum(0).cumsum(1)
    return out


def box_sum(I, i0, j0, i1, j1):
    return I[i1, j1] - I[i0, j1] - I[i1, j0] + I[i0, j0]


def enumerate_windows(extent, sizes, step):
    out = []
    for w, h in sizes:
        xs = np.arange(extent[0], extent[2] - w + 1e-9, step)
        ys = np.arange(extent[1], extent[3] - h + 1e-9, step)
        if not len(xs) or not len(ys):
            continue
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        out.append(np.stack([X.ravel(), Y.ravel(), X.ravel() + w, Y.ravel() + h], 1))
    return np.round(np.concatenate(out), 6)


def window_stats(ctx, W):
    ext = ctx["extent"]
    nbx, nby = ctx["F"].shape
    gx, gy = ctx["dist"].shape

    def ci(v, o, c, nmax):
        return np.clip(np.rint((v - o) / c).astype(np.int64), 0, nmax)
    a0, b0 = ci(W[:, 0], ext[0], COV, nbx), ci(W[:, 1], ext[1], COV, nby)
    a1, b1 = ci(W[:, 2], ext[0], COV, nbx), ci(W[:, 3], ext[1], COV, nby)
    nc = np.maximum((a1 - a0) * (b1 - b0), 1)
    floor_frac = box_sum(integral((ctx["F"] >= FLOOR_MIN).astype(float)), a0, b0, a1, b1) / nc
    ceil_frac = box_sum(integral((ctx["C"] >= CEIL_MIN).astype(float)), a0, b0, a1, b1) / nc
    est = box_sum(integral(ctx["D"]), np.maximum(a0 - 1, 0), np.maximum(b0 - 1, 0),
                  np.minimum(a1 + 1, nbx), np.minimum(b1 + 1, nby))
    c0, d0 = ci(W[:, 0], ext[0], CELL, gx), ci(W[:, 1], ext[1], CELL, gy)
    c1, d1 = ci(W[:, 2], ext[0], CELL, gx), ci(W[:, 3], ext[1], CELL, gy)
    free = box_sum(integral((ctx["dist"] > R).astype(float)), c0, d0, c1, d1) / np.maximum((c1 - c0) * (d1 - d0), 1)
    glass = np.array([glass_hit(w) for w in W], dtype=bool)
    return floor_frac, ceil_frac, est, free, glass


_BORDER = {}


def crop(ctx, win):
    ext = ctx["extent"]
    i0, j0 = int(round((win[0] - ext[0]) / CELL)), int(round((win[1] - ext[1]) / CELL))
    nx, ny = int(round((win[2] - win[0]) / CELL)), int(round((win[3] - win[1]) / CELL))
    if (nx, ny) not in _BORDER:
        bi = np.minimum(np.arange(nx), nx - 1 - np.arange(nx))
        bj = np.minimum(np.arange(ny), ny - 1 - np.arange(ny))
        _BORDER[nx, ny] = np.minimum(bi[:, None], bj[None, :]) * CELL
    return np.minimum(ctx["dist"][i0:i0 + nx, j0:j0 + ny], _BORDER[nx, ny])


def blocker_stats(ctx, pts, key=None):
    if key is not None and key in ctx["bcache"]:
        return ctx["bcache"][key]
    pts = np.asarray(pts, float).reshape(-1, 2)
    pts = pts[:: max(1, len(pts) // 25)]
    u = np.zeros(0, dtype=np.int64)
    if ctx["kd"] is not None and len(pts):
        lists = ctx["kd"].query_ball_point(pts, r=R + 0.05, return_sorted=False)
        if len(lists):
            u = np.unique(np.concatenate([np.asarray(x, dtype=np.int64) for x in lists]))
    zc, top = ctx["sup_zc"][u], ctx["sup_top"][u]
    st = {"n_supports_near": int(len(u)), "n_centre_above_0.30": int((zc > 0.30).sum()),
          "top_p95": float(np.percentile(top, 95)) if len(u) else 0.0,
          "top_max": float(top.max()) if len(u) else 0.0,
          "centre_height_p50": float(np.median(zc)) if len(u) else 0.0}
    st["real_object"] = bool(st["n_centre_above_0.30"] >= 20 and st["top_p95"] >= 0.35)
    if key is not None:
        ctx["bcache"][key] = st
    return st


def scan_window(ctx, win):
    d = crop(ctx, win)
    nx, ny = d.shape
    lab, n = ndimage.label(d > R)
    if n == 0:
        return None
    area = np.bincount(lab.ravel(), minlength=n + 1) * CELL * CELL
    area[0] = 0.0
    for step in (6, 8, 10, 12, 16, 20, 24, 32):
        ii, jj = np.meshgrid(np.arange(step // 2, nx, step), np.arange(step // 2, ny, step), indexing="ij")
        ii, jj = ii.ravel(), jj.ravel()
        ok = (d[ii, jj] >= LATTICE_CLEAR) & (area[lab[ii, jj]] >= MIN_COMP_AREA)
        if ok.sum() <= MAXP:
            break
    ii, jj = ii[ok], jj[ok]
    if len(ii) > MAXP:
        k = np.linspace(0, len(ii) - 1, MAXP).astype(int)
        ii, jj = ii[k], jj[k]
    if len(ii) < 2:
        return {"free_frac": float((d > R).mean())}
    pa, pb = np.triu_indices(len(ii), 1)
    L = np.hypot((ii[pa] - ii[pb]).astype(float), (jj[pa] - jj[pb]).astype(float)) * CELL
    keep = (lab[ii[pa], jj[pa]] == lab[ii[pb], jj[pb]]) & (L >= LMIN)
    pa, pb, L = pa[keep], pb[keep], L[keep]
    out = {"free_frac": float((d > R).mean())}
    if not len(pa):
        return out
    ai, aj, bi, bj = ii[pa], jj[pa], ii[pb], jj[pb]
    K = int(np.ceil(L.max() / CELL)) + 1
    t = np.linspace(0.0, 1.0, K)
    si = np.rint(ai[:, None] + (bi - ai)[:, None] * t).astype(np.int32)
    sj = np.rint(aj[:, None] + (bj - aj)[:, None] * t).astype(np.int32)
    vals = d[si, sj]
    mind = vals.min(axis=1)
    bott = np.zeros(len(pa))
    for th in THRESH:
        lt, _ = ndimage.label(d > th)
        x, y = lt[ai, aj], lt[bi, bj]
        bott = np.where((x > 0) & (x == y), th, bott)
    cs, cg = d[ai, aj], d[bi, bj]
    score = bott + 0.5 * np.minimum(np.minimum(cs, cg), 0.8) + 0.25 * np.minimum(L, 4.0) / 4.0

    def pair(j, extra=None):
        p = {"start": _r([win[0] + (ai[j] + 0.5) * CELL, win[1] + (aj[j] + 0.5) * CELL], 4),
             "goal": _r([win[0] + (bi[j] + 0.5) * CELL, win[1] + (bj[j] + 0.5) * CELL], 4),
             "length": float(L[j]), "start_clear": float(cs[j]), "goal_clear": float(cg[j]),
             "bottleneck": float(bott[j]), "line_min_dist": float(mind[j]), "score": float(score[j])}
        if extra:
            p.update(extra)
        return p
    fb = np.flatnonzero(bott >= CLEAR_MIN)
    if len(fb):
        out["fallback"] = pair(fb[np.argmax(score[fb])])
    pref = np.flatnonzero((mind <= R - BLOCK_DEPTH) & (bott >= CLEAR_MIN))
    seen, tried = set(), 0
    for j in pref[np.argsort(-score[pref])]:
        core = np.flatnonzero(vals[j] <= R - BLOCK_DEPTH)
        px = win[0] + (si[j, core] + 0.5) * CELL
        py = win[1] + (sj[j, core] + 0.5) * CELL
        bxy = (float(px.mean()), float(py.mean()))
        key = (round(bxy[0] / 0.25), round(bxy[1] / 0.25))
        if key in seen:
            continue
        seen.add(key)
        st = blocker_stats(ctx, np.stack([px, py], 1), key)
        tried += 1
        if st["real_object"]:
            out["pref"] = pair(j, {"blocker_xy": _r(bxy), "blocker": st})
            break
        if tried >= 6:
            break
    return out


def run_scan(ctx, W, est, floor_frac, ceil_frac, free, budget, S, J):
    order = np.argsort(est, kind="stable")
    t0, last = time.time(), time.time()
    pref, fallbacks = {}, []
    n_done = n_pref_windows = 0
    for k in order:
        if time.time() - t0 > budget:
            S["scan"]["budget_exhausted"] = True
            break
        win = [float(v) for v in W[k]]
        try:
            r = scan_window(ctx, win)
        except Exception as e:  # noqa: BLE001
            S["errors"].append({"phase": "scan", "window": win, "error": repr(e)})
            continue
        n_done += 1
        if r is None:
            continue
        base = {"window": win, "est_supports": int(est[k]), "floor_frac": float(floor_frac[k]),
                "ceil_frac": float(ceil_frac[k]), "interior": bool(floor_frac[k] >= 0.75 and ceil_frac[k] >= 0.75),
                "free_frac": r["free_frac"]}
        if "pref" in r:
            n_pref_windows += 1
            p = r["pref"]
            key = (round(p["blocker_xy"][0] / 0.5), round(p["blocker_xy"][1] / 0.5))
            cur = pref.get(key)
            if cur is None or (base["est_supports"] <= 1.15 * cur["est_supports"]
                               and p["score"] >= cur["pair"]["score"] + 0.1):
                pref[key] = {**base, "pair": p}
        if "fallback" in r and base["interior"] and len(fallbacks) < 2000:
            fallbacks.append({**base, "pair": r["fallback"]})
        if time.time() - last > 120:
            last = time.time()
            S["scan"].update(n_scanned=n_done, n_pref_windows=n_pref_windows, n_pref_clusters=len(pref),
                             seconds=time.time() - t0, last_est_supports=int(est[k]))
            dump(J, S)
            log("scan progress", n_done, "windows, est", int(est[k]), "pref clusters", len(pref))
    S["scan"].update(n_scanned=n_done, n_pref_windows=n_pref_windows, n_pref_clusters=len(pref),
                     n_fallback_windows=len(fallbacks), seconds=time.time() - t0)
    return list(pref.values()), fallbacks


# ----------------------------------------------------------------------------------------------------------------
# refinement: geodesic detour, tightened window, amendment pre-check on the window's own map

def geodesic(d, a, b, thr, full):
    from skimage.graph import MCP_Geometric
    cost = np.where(d > thr, 1.0, -1.0)
    if cost[a] < 0 or cost[b] < 0:
        return None, None
    m = MCP_Geometric(cost, fully_connected=full)
    cum, _ = m.find_costs([a], [b])
    if not np.isfinite(cum[b]):
        return None, None
    return np.array(m.traceback(b)), float(cum[b] * CELL)


def snap_out(lo, hi, g=0.25):
    return (float(np.floor(round(lo / g, 6)) * g), float(np.ceil(round(hi / g, 6)) * g))


def tighten(ctx, cand):
    win, p = cand["window"], cand["pair"]
    d = crop(ctx, win)
    a, b = _cell(p["start"], win, d.shape), _cell(p["goal"], win, d.shape)
    path, _ = geodesic(d, a, b, max(p["bottleneck"], R), False)
    if path is None:
        return None
    xs = win[0] + (path[:, 0] + 0.5) * CELL
    ys = win[1] + (path[:, 1] + 0.5) * CELL
    x0, x1 = snap_out(xs.min() - PAD, xs.max() + PAD)
    y0, y1 = snap_out(ys.min() - PAD, ys.max() + PAD)
    tw = [max(win[0], x0), max(win[1], y0), min(win[2], x1), min(win[3], y1)]
    return [round(v, 4) for v in tw]


def bottleneck(dist, a, b):
    best = 0.0
    for th in THRESH:
        lab, _ = ndimage.label(dist > th)
        if lab[a] != 0 and lab[a] == lab[b]:
            best = th
        else:
            break
    return best


def precheck(ctx, win, start, goal):
    """Amendment 2 pre-check on the certified map of this window (case_certified_check.py pattern)."""
    t = time.time()
    sub = presub(ctx, win)
    s2, st = project_scene(sub, ctx["robot"], win, z_floor=ctx["z_f"])
    occ = _support_raster(s2, win, CELL)
    dist = ndimage.distance_transform_edt(~occ) * CELL
    lab, ncomp = ndimage.label(dist > R)
    a, b = _cell(start, win, dist.shape), _cell(goal, win, dist.shape)
    cs, cg = float(dist[a]), float(dist[b])
    same = bool(lab[a] != 0 and lab[a] == lab[b])
    p0, p1 = np.asarray(start[:2], float), np.asarray(goal[:2], float)
    L = float(np.linalg.norm(p1 - p0))
    tt = np.linspace(0, 1, int(np.ceil(L / (CELL / 2))) + 1)[:, None]
    pts = p0 + tt * (p1 - p0)
    li = np.clip(((pts[:, 0] - win[0]) / CELL).astype(int), 0, dist.shape[0] - 1)
    lj = np.clip(((pts[:, 1] - win[1]) / CELL).astype(int), 0, dist.shape[1] - 1)
    vals = dist[li, lj]
    bott = bottleneck(dist, a, b) if same else 0.0
    path, glen = geodesic(dist, a, b, max(bott, R), True) if same else (None, None)
    core = vals <= R - BLOCK_DEPTH
    blk = None
    if core.any():
        cxy = pts[core].mean(axis=0)
        ids = np.array([sp.primitive_id for sp in s2.supports], dtype=np.int64)
        means = np.array([sp.mean for sp in s2.supports]).reshape(-1, 2)
        tree = cKDTree(means)
        cp = pts[core][:: max(1, int(core.sum()) // 25)]
        u = np.unique(np.concatenate([np.asarray(x, dtype=np.int64) for x in
                                      tree.query_ball_point(cp, r=R + 0.05, return_sorted=False)]))
        idx = np.searchsorted(ctx["scene"].ids, ids[u])
        zc = ctx["scene"].means[idx, 2] - ctx["z_f"]
        top = ctx["HI"][idx, 2] - ctx["z_f"]
        blk = {"centroid": _r(cxy), "name": name_blocker(cxy), "n_supports_near": int(len(u)),
               "n_centre_above_0.30": int((zc > 0.30).sum()),
               "top_p95": float(np.percentile(top, 95)) if len(u) else 0.0,
               "top_max": float(top.max()) if len(u) else 0.0,
               "centre_height_p50": float(np.median(zc)) if len(u) else 0.0}
        blk["real_object"] = bool(blk["n_centre_above_0.30"] >= 20 and blk["top_p95"] >= 0.35)
    ext = ctx["extent"]
    i0, j0 = int(round((win[0] - ext[0]) / CELL)), int(round((win[1] - ext[1]) / CELL))
    g = ctx["occ"][i0:i0 + occ.shape[0], j0:j0 + occ.shape[1]]
    diff = int((g[1:-1, 1:-1] != occ[1:g.shape[0] - 1, 1:g.shape[1] - 1]).sum()) if g.shape == occ.shape else None
    res = {"window": [float(v) for v in win], "window_size": _r([win[2] - win[0], win[3] - win[1]], 3),
           "start": _r(start, 4), "goal": _r(goal, 4), "n_supports": len(s2.supports), "projection": st,
           "start_clear": cs, "goal_clear": cg, "same_component": same, "n_components": int(ncomp),
           "precheck_pass": bool(cs >= CLEAR_MIN and cg >= CLEAR_MIN and same),
           "straight_length": L, "line_min_dist": float(vals.min()), "line_blocked": bool(vals.min() <= R),
           "line_blocked_strong": bool(vals.min() <= R - BLOCK_DEPTH),
           "line_blocked_frac": float((vals <= R).mean()), "bottleneck": bott,
           "geodesic_length": glen, "detour_ratio": (glen / L) if glen else None,
           "disc_free_frac": float((dist > R).mean()), "occupied_frac": float(occ.mean()),
           "blocker": blk, "raster_vs_global_crop_diff_cells": diff, "glass_zone_hit": glass_hit(win),
           "seconds": time.time() - t}
    arrays = {"occ": occ, "dist": dist, "lab": lab, "path": path, "ids": np.sort(
        np.array([sp.primitive_id for sp in s2.supports], dtype=np.int64))}
    return res, arrays


def hard_pass(rec):
    b = rec.get("blocker") or {}
    return bool(rec["precheck_pass"] and rec["line_blocked_strong"] and b.get("real_object")
                and rec["bottleneck"] >= CLEAR_MIN and not rec["glass_zone_hit"]
                and rec["window_size"][0] <= 8.0 and rec["window_size"][1] <= 8.0 and rec.get("interior", True))


def quality(rec):
    b = rec.get("blocker") or {}
    return (rec["bottleneck"] + 0.5 * min(rec["start_clear"], rec["goal_clear"], 0.8)
            + (0.2 if b.get("name") else 0.0) + 0.1 * min((rec.get("detour_ratio") or 1.0) - 1.0, 0.5))


def rank_key(rec):
    return (int(np.floor(np.log(max(rec["n_supports"], 1)) / np.log(1.5))), -quality(rec))


# ----------------------------------------------------------------------------------------------------------------
# probe (showcase_run.py logic) in a spawned child with a timeout

def _probe_child(conn, sp, robot, pw, cy):
    try:
        from gmc.config import load_config
        from gmc.height.run import compile_and_query, with_overrides
        cfg = load_config("configs/height_showcase.yaml")
        t0 = time.time()
        pcfg = with_overrides(cfg, initial_intervals=1, max_depth=0)
        res, _ = compile_and_query(sp, robot, pcfg, (pw[0] + 0.3, cy, 0.0), (pw[2] - 0.3, cy, 0.0))
        dt = time.time() - t0
        conn.send({"ok": True, "probe_compile_seconds": dt, "probe_status": res["status"],
                   "compile_seconds_inner": res["compile_seconds"], "n_pairs": res["n_pairs"],
                   "n_slabs": res["n_slabs"], "initial_intervals": cfg.orientation.initial_intervals})
    except BaseException as e:  # noqa: BLE001
        conn.send({"ok": False, "error": repr(e), "traceback": traceback.format_exc()})
    finally:
        conn.close()


def probe_input(ctx, rec, half=PROBE_HALF):
    st, gl = np.array(rec["start"][:2], float), np.array(rec["goal"][:2], float)
    mid_s = 0.5 * np.linalg.norm(gl - st)                      # showcase_run: no overhang in the case
    c = st + (gl - st) / np.linalg.norm(gl - st) * mid_s
    pw = [float(c[0] - half), float(c[1] - half), float(c[0] + half), float(c[1] + half)]
    sp, _ = project_scene(presub(ctx, pw), ctx["robot"], pw, z_floor=ctx["z_f"])
    return sp, pw, float(c[1])


def run_probes(ctx, jobs, timeout, S, J):
    """jobs: list of (label, rec, half). Runs all in parallel spawned children; kills any after timeout."""
    mpc = mp.get_context("spawn")
    live, results = [], []
    for label, rec, half in jobs:
        try:
            sp, pw, cy = probe_input(ctx, rec, half)
        except Exception as e:  # noqa: BLE001
            results.append({"label": label, "ok": False, "error": repr(e)})
            continue
        meta = {"label": label, "window": rec["window"], "n_supports_full": rec["n_supports"], "probe_half": half,
                "probe_window": pw, "n_supports_probe": len(sp.supports)}
        rd, wr = mpc.Pipe(duplex=False)
        p = mpc.Process(target=_probe_child, args=(wr, sp, ctx["robot"], pw, cy))
        p.start()
        wr.close()
        live.append([p, rd, time.time(), meta])
        log("probe started", json.dumps(meta))
    while live:
        for item in list(live):
            p, rd, t0, meta = item
            msg = None
            if rd.poll():
                try:
                    msg = rd.recv()
                except EOFError:
                    msg = {"ok": False, "error": f"child closed pipe, exitcode {p.exitcode}"}
            elif not p.is_alive():
                p.join(1)
                msg = None if rd.poll() else {"ok": False, "error": f"child died, exitcode {p.exitcode}"}
                if msg is None:
                    continue
            elif time.time() - t0 > timeout:
                p.kill()
                p.join(30)
                msg = {"ok": False, "timeout": True, "error": f"killed after {timeout:.0f} s"}
            if msg is None:
                continue
            p.join(30)
            r = {**meta, **msg, "wall_seconds": time.time() - t0}
            ratio = meta["n_supports_full"] / max(1, meta["n_supports_probe"])
            r["ratio"] = ratio
            if msg.get("ok"):
                r["projected_hours"] = msg["probe_compile_seconds"] * ratio * msg["initial_intervals"] / 3600.0
                r["abort"] = r["projected_hours"] > ABORT_HOURS
                r["seconds_per_probe_support"] = msg["probe_compile_seconds"] / max(1, meta["n_supports_probe"])
            elif msg.get("timeout"):
                r["projected_hours_lower_bound"] = timeout * ratio / 3600.0
                r["abort"] = r["projected_hours_lower_bound"] > ABORT_HOURS
            results.append(r)
            live.remove(item)
            S.setdefault("probes", []).append(r)
            dump(J, S)
            log("probe done", json.dumps({k: v for k, v in r.items() if k != "traceback"}))
        time.sleep(2)
    return results


# ----------------------------------------------------------------------------------------------------------------
# figures

def fig_candidate(ctx, rec, arr, path, label):
    win, st, gl = rec["window"], rec["start"], rec["goal"]
    occ, dist, lab = arr["occ"], arr["dist"], arr["lab"]
    ext = [win[0], win[2], win[1], win[3]]
    fig, ax = plt.subplots(1, 3, figsize=(27, 9))
    img = np.ones(occ.shape[::-1] + (3,))
    img[(dist > R).T] = (0.75, 0.95, 0.75)
    img[occ.T] = (0.8, 0.1, 0.1)
    ax[0].imshow(img, origin="lower", extent=ext, interpolation="nearest")
    a = _cell(st, win, dist.shape)
    comp = (lab == lab[a]) if lab[a] else np.zeros_like(occ)
    ax[0].imshow(np.ma.masked_where(~comp.T, comp.T), origin="lower", extent=ext, cmap="winter", alpha=0.3)
    ax[0].plot([st[0], gl[0]], [st[1], gl[1]], "b--", lw=1.2, label="straight start-goal")
    if arr["path"] is not None:
        px = win[0] + (arr["path"][:, 0] + 0.5) * CELL
        py = win[1] + (arr["path"][:, 1] + 0.5) * CELL
        ax[0].plot(px, py, "-", color="orange", lw=2, label="raster detour (selection aid)")
    ax[0].plot(*st[:2], "go", ms=9, label="start")
    ax[0].plot(*gl[:2], "bs", ms=9, label="goal")
    b = rec.get("blocker") or {}
    if b.get("centroid"):
        ax[0].plot(*b["centroid"], "kx", ms=12, mew=2, label="blocker")
    ax[0].legend(loc="upper right", fontsize=8)
    ax[0].set_title(f"{label}: certified cylinder map, {rec['n_supports']:,} supports (red); green disc r=0.30 fits; "
                    f"blue = start component\nclear s/g {rec['start_clear']:.3f}/{rec['goal_clear']:.3f}, same comp "
                    f"{rec['same_component']}, bottleneck {rec['bottleneck']:.3f}, line min dist "
                    f"{rec['line_min_dist']:.3f}, detour x{(rec.get('detour_ratio') or 0):.2f}", fontsize=9)
    ax[0].set_xticks(np.arange(np.ceil(win[0]), win[2] + 1e-9, 0.5))
    ax[0].set_yticks(np.arange(np.ceil(win[1]), win[3] + 1e-9, 0.5))
    ax[0].tick_params(labelsize=7)
    ax[0].grid(alpha=0.3, lw=0.4)
    sc = ctx["scene"]
    m = ((sc.means[:, 0] >= win[0] - 0.5) & (sc.means[:, 0] <= win[2] + 0.5)
         & (sc.means[:, 1] >= win[1] - 0.5) & (sc.means[:, 1] <= win[3] + 0.5))
    loc = sc.subset(m)
    s, zb, zt, L = _section(loc, ctx["z_f"], st[:2], gl[:2], half=R)
    ax[1].vlines(s, zb, zt, lw=0.3, color="k", alpha=0.3)
    ax[1].axhspan(BAND[0], BAND[1], color="r", alpha=0.08, label="cylinder band 0.02-1.75")
    ax[1].set_ylim(-0.15, 2.6)
    ax[1].set_xlim(0, L)
    ax[1].set_xlabel("distance along straight start->goal (m)")
    ax[1].set_ylabel("height above floor (m)")
    ax[1].legend(loc="upper right")
    ax[1].set_title(f"section +-{R} m along the straight line; blocker: {b.get('name')} "
                    f"(top p95 {b.get('top_p95', 0):.2f} m, {b.get('n_centre_above_0.30', 0)} supports centred > 0.30 m)",
                    fontsize=9)
    opq = loc.opacity > TAU
    lo3, hi3 = loc.aabb(RHO)
    z_f = ctx["z_f"]
    inb = opq & (hi3[:, 2] >= z_f + BAND[0]) & (lo3[:, 2] <= z_f + BAND[1])
    hc = 0.05
    nx, ny = int(np.ceil((win[2] - win[0]) / hc)), int(np.ceil((win[3] - win[1]) / hc))
    hm = np.full((nx, ny), np.nan)
    ii = ((loc.means[inb, 0] - win[0]) / hc).astype(int)
    jj = ((loc.means[inb, 1] - win[1]) / hc).astype(int)
    ok = (ii >= 0) & (ii < nx) & (jj >= 0) & (jj < ny)
    top = np.clip(hi3[inb, 2] - z_f, 0, 2.5)[ok]
    hm0 = np.full((nx, ny), -1.0)
    np.maximum.at(hm0, (ii[ok], jj[ok]), top)
    hm[hm0 >= 0] = hm0[hm0 >= 0]
    im = ax[2].imshow(hm.T, origin="lower", extent=ext, cmap="viridis", vmin=0, vmax=2.0, interpolation="nearest")
    plt.colorbar(im, ax=ax[2], shrink=0.7, label="highest opaque in-band splat top above floor (m), 5 cm cells")
    ax[2].plot([st[0], gl[0]], [st[1], gl[1]], "w--", lw=1)
    if arr["path"] is not None:
        ax[2].plot(px, py, "-", color="orange", lw=1.5)
    ax[2].plot(*st[:2], "go", ms=8)
    ax[2].plot(*gl[:2], "rs", ms=8)
    ax[2].set_title("3D context: height of splats in the cylinder band (selection aid)", fontsize=9)
    plt.tight_layout()
    plt.savefig(path, dpi=70)
    plt.close()


def fig_overview(ctx, ranked, fallbacks, path):
    ext = ctx["extent"]
    occ, dist = ctx["occ"][::4, ::4], ctx["dist"][::4, ::4]
    img = np.ones(occ.shape[::-1] + (3,))
    img[(dist > R).T] = (0.75, 0.95, 0.75)
    img[occ.T] = (0.8, 0.1, 0.1)
    E = [ext[0], ext[2], ext[1], ext[3]]
    fig, ax = plt.subplots(1, 2, figsize=(22, 17))
    ax[0].imshow(img, origin="lower", extent=E, interpolation="nearest")
    interior = (ctx["F"] >= FLOOR_MIN) & (ctx["C"] >= CEIL_MIN)
    gx = ext[0] + COV * (np.arange(interior.shape[0]) + 0.5)
    gy = ext[1] + COV * (np.arange(interior.shape[1]) + 0.5)
    ax[0].contour(gx, gy, interior.T.astype(float), levels=[0.5], colors="k", linewidths=1)
    g = GLASS_ZONE
    ax[0].add_patch(Rectangle((g[0], g[1]), g[2] - g[0], g[3] - g[1], fill=False, ec="m", lw=2, ls=":"))
    for k, rec in enumerate(ranked[:10]):
        w = rec["window"]
        ax[0].add_patch(Rectangle((w[0], w[1]), w[2] - w[0], w[3] - w[1], fill=False, ec="b", lw=1.5))
        ax[0].plot([rec["start"][0], rec["goal"][0]], [rec["start"][1], rec["goal"][1]], "b-", lw=1)
        ax[0].text(w[0] + 0.1, w[3] - 0.5, f"P{k + 1}", color="b", fontsize=11, weight="bold")
    for k, rec in enumerate(fallbacks[:4]):
        w = rec["window"]
        ax[0].add_patch(Rectangle((w[0], w[1]), w[2] - w[0], w[3] - w[1], fill=False, ec="c", lw=1.5, ls="--"))
        ax[0].text(w[0] + 0.1, w[1] + 0.2, f"F{k + 1}", color="c", fontsize=11, weight="bold")
    ax[0].set_title("scene-wide certified cylinder raster (red occupied, green disc r=0.30 fits); black: floor+ceiling "
                    "coverage; blue P = preferred candidates, cyan F = fallback; magenta: glass zone", fontsize=10)
    ax[1].imshow(np.log10(1 + ctx["D"]).T, origin="lower", extent=E, cmap="magma")
    ax[1].set_title("log10(1 + cylinder supports per 0.5 m cell)")
    for a_ in ax:
        a_.set_xticks(np.arange(np.ceil(ext[0]), ext[2], 1))
        a_.set_yticks(np.arange(np.ceil(ext[1]), ext[3], 1))
        a_.tick_params(labelsize=6)
        a_.grid(alpha=0.3, lw=0.4)
    plt.tight_layout()
    plt.savefig(path, dpi=70)
    plt.close()


# ----------------------------------------------------------------------------------------------------------------

def synthetic_scene():
    """Tiny synthetic room for the login-node smoke test: floor, ceiling, one 0.8 m box, a speck."""
    pts, sig = [], []
    g = np.arange(0.05, 6.0, 0.1)
    X, Y = np.meshgrid(g, g)
    flat = np.c_[X.ravel(), Y.ravel()]
    for z in (0.0, 4.5):
        pts.append(np.c_[flat, np.full(len(flat), z)])
        sig.append(np.tile([0.05, 0.05, 0.001], (len(flat), 1)))
    box = []
    for z in np.arange(0.1, 1.21, 0.2):
        for u in np.arange(-0.4, 0.41, 0.2):
            box += [(3 + u, 2.6, z), (3 + u, 3.4, z), (2.6, 3 + u, z), (3.4, 3 + u, z)]
    box = np.unique(np.round(np.array(box), 6), axis=0)
    box[:, :2] += np.random.default_rng(0).normal(0.0, 0.01, (len(box), 2))   # distinct shadows (no dedup)
    pts.append(box)
    sig.append(np.tile([0.06, 0.06, 0.06], (len(box), 1)))
    pts.append(np.array([[1.2, 4.8, 0.03]]))
    sig.append(np.array([[0.02, 0.02, 0.02]]))
    M, Sg = np.concatenate(pts), np.concatenate(sig)
    covs = np.zeros((len(M), 3, 3))
    covs[:, [0, 1, 2], [0, 1, 2]] = Sg ** 2
    return GaussianScene3D(M, covs, np.full(len(M), 0.9), np.arange(len(M)), "synthetic")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=OUT_DEFAULT)
    ap.add_argument("--scan-budget", type=float, default=2700.0)
    ap.add_argument("--probe-timeout", type=float, default=2400.0)
    ap.add_argument("--n-refine", type=int, default=14)
    ap.add_argument("--n-probe", type=int, default=3)
    ap.add_argument("--synthetic", action="store_true")
    a = ap.parse_args()
    T0 = time.time()
    out = Path(a.out)
    figs = out / "figs"
    figs.mkdir(parents=True, exist_ok=True)
    J = out / "search.json"
    S = {"meta": {"script": "experiments/percase_search_cylinder.py", "job": os.environ.get("SLURM_JOB_ID"),
                  "host": os.uname().nodename, "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "args": vars(a),
                  "robot": "cylinder", "disc_r": R, "band_above_floor": BAND, "tau": TAU, "rho": RHO,
                  "raster_cell": CELL, "clear_min": CLEAR_MIN, "lattice_clear": LATTICE_CLEAR,
                  "block_depth": BLOCK_DEPTH, "glass_zone": GLASS_ZONE, "claims_boundary": CLAIMS,
                  "note": "window scan, blocker heights and raster detours are selection aids; the case is judged "
                          "on each window's own certified raster (amendment pre-check)"},
         "errors": [], "timing": {}, "scan": {}}

    def tick(name, t):
        S["timing"][name] = round(time.time() - t, 1)
        dump(J, S)
        log("phase", name, S["timing"][name], "s")

    t = time.time()
    if a.synthetic:
        scene, z_f, extent, sizes = synthetic_scene(), 0.0, [0.0, 0.0, 6.0, 6.0], [(3.0, 3.0), (4.0, 4.0)]
    else:
        scene, g0 = load_processed()
        assert "floor_rule" in g0, "floor rule must be on"
        z_f, extent, sizes = float(g0["floor"]["z_floor"]), EXTENT, SIZES
        S["meta"]["floor_rule"] = g0["floor_rule"]
    assert np.all(np.diff(scene.ids) > 0)
    robot = robot_table(1.20)["cylinder"]
    assert abs(robot.max_radius() - R) < 1e-9 and (robot.z_lo, robot.z_hi) == BAND
    S["meta"].update(z_floor=z_f, n_splats=len(scene), extent=extent)
    LO, HI = scene.aabb(RHO)
    tick("load", t)

    t = time.time()
    ctx = build_context(scene, LO, HI, robot, z_f, extent, S)
    tick("global_raster", t)

    t = time.time()
    W = enumerate_windows(extent, sizes, STEP)
    floor_frac, ceil_frac, est, free, glass = window_stats(ctx, W)
    keep = (floor_frac >= 0.5) & (ceil_frac >= 0.5) & (free >= 0.12) & ~glass & (est <= SUP_CAP)
    S["scan"].update(n_windows=int(len(W)), n_prefilter=int(keep.sum()),
                     n_interior_strict=int(((floor_frac >= 0.75) & (ceil_frac >= 0.75)).sum()),
                     n_glass=int(glass.sum()), budget_s=a.scan_budget,
                     est_supports_prefilter_percentiles={q: float(np.percentile(est[keep], q)) if keep.any() else None
                                                         for q in (5, 25, 50, 75, 95)})
    log("windows", len(W), "after prefilter", int(keep.sum()))
    Wk = W[keep]
    pref, fallbacks = run_scan(ctx, Wk, est[keep], floor_frac[keep], ceil_frac[keep], free[keep],
                               a.scan_budget, S, J)
    tick("scan", t)

    t = time.time()
    for c in pref:
        try:
            tw = tighten(ctx, c)
            c["tight_window"] = tw
            c["tight_exact_supports"] = exact_count(ctx, tw) if tw else None
            c["window_exact_supports"] = exact_count(ctx, c["window"])
        except Exception as e:  # noqa: BLE001
            S["errors"].append({"phase": "tighten", "window": c["window"], "error": repr(e)})
    pref = [c for c in pref if c.get("window_exact_supports") is not None]
    pref.sort(key=lambda c: min(c["window_exact_supports"], c.get("tight_exact_supports") or 1 << 60))
    S["pref_clusters"] = pref[:60]
    fallbacks.sort(key=lambda c: c["est_supports"])
    fb_sel = []
    def centre(c):
        return np.array([(c["window"][0] + c["window"][2]) / 2, (c["window"][1] + c["window"][3]) / 2])

    for c in fallbacks:
        if all(float(np.linalg.norm(centre(c) - centre(f))) >= 3.0 for f in fb_sel):
            c["window_exact_supports"] = exact_count(ctx, c["window"])
            fb_sel.append(c)
        if len(fb_sel) >= 4:
            break
    S["fallback_windows"] = fb_sel
    tick("tighten", t)

    t = time.time()
    refined, arrays = [], {}
    chosen_keys = []
    todo = [c for c in pref if c["interior"]] or pref
    for c in todo:
        if len(refined) >= a.n_refine:
            break
        bxy = np.array(c["pair"]["blocker_xy"])
        if any(np.hypot(*(bxy - k)) < 1.0 for k in chosen_keys):
            continue
        chosen_keys.append(bxy)
        for tag, win in (("tight", c.get("tight_window")), ("scan", c["window"])):
            if not win:
                continue
            try:
                rec, arr = precheck(ctx, win, c["pair"]["start"], c["pair"]["goal"])
            except Exception as e:  # noqa: BLE001
                S["errors"].append({"phase": "precheck", "window": win, "error": repr(e),
                                    "traceback": traceback.format_exc()})
                continue
            rec.update(kind="preferred", window_from=tag, scan_window=c["window"], interior=c["interior"],
                       floor_frac=c["floor_frac"], ceil_frac=c["ceil_frac"])
            rec["hard_pass"] = hard_pass(rec)
            log("precheck", tag, json.dumps({k: rec[k] for k in ("window", "n_supports", "start_clear", "goal_clear",
                                                                 "same_component", "line_min_dist", "bottleneck",
                                                                 "hard_pass")}), (rec["blocker"] or {}).get("name"))
            refined.append(rec)
            arrays[len(refined) - 1] = arr
            if rec["hard_pass"]:
                break
        S["refined"] = refined
        dump(J, S)
    for c in fb_sel:
        try:
            tw = tighten(ctx, c) or c["window"]
            rec, arr = precheck(ctx, tw, c["pair"]["start"], c["pair"]["goal"])
            rec.update(kind="fallback", window_from="tight", scan_window=c["window"], interior=c["interior"],
                       floor_frac=c["floor_frac"], ceil_frac=c["ceil_frac"])
            rec["hard_pass"] = False
            rec["fallback_pass"] = bool(rec["precheck_pass"] and rec["bottleneck"] >= CLEAR_MIN
                                        and not rec["glass_zone_hit"])
            refined.append(rec)
            arrays[len(refined) - 1] = arr
        except Exception as e:  # noqa: BLE001
            S["errors"].append({"phase": "precheck_fallback", "window": c["window"], "error": repr(e)})
    S["refined"] = refined
    tick("precheck", t)

    t = time.time()
    order = sorted([i for i, r in enumerate(refined) if r["hard_pass"]], key=lambda i: rank_key(refined[i]))
    fb_order = sorted([i for i, r in enumerate(refined) if r.get("fallback_pass")],
                      key=lambda i: (refined[i]["n_supports"], -quality(refined[i])))
    S["ranking"] = {"preferred": [{"refined_index": i, "n_supports": refined[i]["n_supports"],
                                   "quality": quality(refined[i]), "blocker": (refined[i]["blocker"] or {}).get("name"),
                                   "window": refined[i]["window"]} for i in order],
                    "fallback": [{"refined_index": i, "n_supports": refined[i]["n_supports"],
                                  "window": refined[i]["window"]} for i in fb_order]}
    S["figures"] = []
    for k, i in enumerate(order[:3]):
        p = figs / f"precheck_P{k + 1}.png"
        try:
            fig_candidate(ctx, refined[i], arrays[i], p, f"P{k + 1}")
            S["figures"].append(str(p))
        except Exception as e:  # noqa: BLE001
            S["errors"].append({"phase": "figure", "index": i, "error": repr(e), "traceback": traceback.format_exc()})
    for k, i in enumerate(fb_order[:1] if len(order) < 3 else []):
        p = figs / f"precheck_F{k + 1}.png"
        try:
            fig_candidate(ctx, refined[i], arrays[i], p, f"F{k + 1}")
            S["figures"].append(str(p))
        except Exception as e:  # noqa: BLE001
            S["errors"].append({"phase": "figure", "index": i, "error": repr(e)})
    try:
        fig_overview(ctx, [refined[i] for i in order], [refined[i] for i in fb_order], figs / "overview.png")
        S["figures"].append(str(figs / "overview.png"))
    except Exception as e:  # noqa: BLE001
        S["errors"].append({"phase": "overview", "error": repr(e), "traceback": traceback.format_exc()})
    tick("figures", t)

    t = time.time()
    S["full_scene_check"] = []
    for i in (order[:3] + fb_order[:1]):
        rec = refined[i]
        try:
            s_full, _ = project_scene(scene, robot, rec["window"], z_floor=z_f)
            ids_full = np.sort(np.array([s.primitive_id for s in s_full.supports], dtype=np.int64))
            S["full_scene_check"].append({"refined_index": i, "window": rec["window"], "n_full": int(len(ids_full)),
                                          "n_presubset": rec["n_supports"],
                                          "ids_equal": bool(np.array_equal(ids_full, arrays[i]["ids"]))})
            del s_full
        except Exception as e:  # noqa: BLE001
            S["errors"].append({"phase": "full_scene_check", "index": i, "error": repr(e)})
        log("full scene check", json.dumps(S["full_scene_check"][-1:]))
    tick("full_scene_check", t)

    t = time.time()
    jobs = [(f"P{k + 1}", refined[i], PROBE_HALF) for k, i in enumerate(order[:a.n_probe])]
    if order:
        jobs.append(("P1_half_scaling", refined[order[0]], 0.5 * PROBE_HALF))
    res = run_probes(ctx, jobs, a.probe_timeout, S, J) if jobs else []
    main_probes = [r for r in res if r["label"] != "P1_half_scaling"]
    if fb_order and not any(r.get("ok") and not r.get("abort") for r in main_probes):
        run_probes(ctx, [(f"F{k + 1}", refined[i], PROBE_HALF) for k, i in enumerate(fb_order[:2])],
                   a.probe_timeout, S, J)
    tick("probes", t)

    probes = {r["label"]: r for r in S.get("probes", [])}
    sugg = None
    for lim in (3.0, ABORT_HOURS):
        for k, i in enumerate(order[:a.n_probe]):
            r = probes.get(f"P{k + 1}")
            if r and r.get("ok") and r["projected_hours"] <= lim:
                sugg = {"label": f"P{k + 1}", "refined_index": i, "behaviour": "around", "projected_hours_limit": lim}
                break
        if sugg:
            break
    if sugg is None:
        for k, i in enumerate(fb_order[:2]):
            r = probes.get(f"F{k + 1}")
            if r and r.get("ok") and r["projected_hours"] <= ABORT_HOURS:
                sugg = {"label": f"F{k + 1}", "refined_index": i, "behaviour": "fallback"}
                break
    S["suggestion"] = sugg
    S["timing"]["total"] = round(time.time() - T0, 1)
    dump(J, S)
    log("suggestion", json.dumps(sugg), "total", S["timing"]["total"], "s; errors", len(S["errors"]))


if __name__ == "__main__":
    main()
