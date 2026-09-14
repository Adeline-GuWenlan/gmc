# gmc/experiments/percase_search_sweeper.py
"""Amendment 2 case search for the sweeper (disc r = 0.175 m, band 0.02-0.10 m above the floor).

This is a selection aid, not a result.
- Preferred behaviour: the straight start->goal segment passes under an overhang (bench seat, tabletop). The overhang's
  underside is >= 0.15 m above the floor (sweeper top + 0.05) and it reaches into the cylinder band (< 1.75 m).
- Fallback: any connected pair.

Every number that decides a case comes from the amendment's certified pre-check:
project_scene for the sweeper on the window -> showcase_scene._support_raster(s2, window, 0.025) -> distance transform
-> start/goal clearance >= r + 0.05, and start and goal in one connected component of dist > r.
The scene (load_processed() with its floor rule), tau, rho, the robot and the GMC config are unchanged.

Stages (search.json is rewritten after each):
  0  Load the scene once. Pre-subset per region by the rho-AABB + 2.5 m, and check that the subset is identical to the
     full scene on the A2/A4 SW-bench window.
  1  Per region:
     - certified sweeper raster of the whole region;
     - endpoints on a 0.25 m lattice;
     - pairs whose segment crosses sweeper-free cells under overhang mass;
     - section check (showcase_scene._section logic);
     - windows at several margins, cropped from the region raster (screening only).
  2  Shortlist: raster shortest path in dist > r + margin, and its length under the overhang.
  3  Finalists:
     - the exact pre-check on each window;
     - figures for the top 3;
     - showcase_run's timing probe;
     - a time-boxed trial compile+query on the top candidates (selection aid only; the planning job is the result).

Run from gmc/: PYTHONPATH=src:experiments python experiments/percase_search_sweeper.py [--selftest]
"""
import argparse
import json
import math
import os
import signal
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from showcase_scene import RASTER, RHO, TAU, _section, _support_raster, load_processed
from gmc.height.prism import robot_table

ROBOT = "sweeper"
OUT = Path("results/height/percase/sweeper")
FIGS = OUT / "figs"
G0_FILE = Path("results/height/showcase/g0.json")
Z_C = 1.20
C = RASTER                          # 0.025 m raster of the amendment's pre-check
SEC_BIN = 0.02                      # step_case's section bin
LOW_TOP = 0.15                      # "free low": nothing with rho-bottom <= 0.15 and rho-top >= 0.02 in the strip
UNDER_LO, UNDER_HI = 0.15, 1.75     # overhang: rho-bottom in (0.15, 1.75) m above the floor
ROB = robot_table(Z_C)[ROBOT]
CYL = robot_table(Z_C)["cylinder"]
R = ROB.max_radius()                # 0.175
CLEAR_MIN = R + 0.05                # amendment pre-check
MAX_SIDE = 8.0
MIN_SIDE = 2.0
SUBSET_MARGIN = 2.5
LATTICE = 10                        # endpoint lattice in raster cells (0.25 m)
EDGE_CELLS = 20                     # endpoints >= 0.5 m inside the region
L_MIN, L_MAX = 2.0, 6.0
MARGINS = (0.5, 0.75, 1.0, 1.5, 2.0)
TOP_PAIRS = 300
SHORTLIST_PAIRS = 40
FINALISTS = 8
GLASS_BOX = (-4.4, 0.4, -3.4, 3.1)  # glass doors x ~ -3.9, y 0.9-2.6 (A2), dilated by 0.5 m
JOB_S = 4 * 3600
T0 = time.time()
CLAIMS = ("Per-robot showcase case chosen for success, not a morphology comparison at one place; no 'body shape "
          "changes the route' claim rests on it; floor-surface rule (Amendment 1) on.")

# Regions, from A2/A4 observations (worklog height_bands.md). All are far from the glass doors.
REGIONS = [
    ("SW", [-2.5, 5.5, 6.5, 16.0], "SW bench (2.1,10.5) seat 0.43; table B (-0.45,10.81) top ~0.8; SW clear blob"),
    ("A", [4.5, 2.5, 13.5, 11.5], "table A (8.96,7.29) top 0.80 with a bench along its NE side"),
    ("E", [6.5, 10.5, 16.5, 19.5], "hall bench (10,16) seat 0.43; table C (13.28,14.09) top 0.93"),
    ("G", [-0.5, 18.5, 7.5, 26.5], "table G (3.35,22.53) top 0.95; object D (5.8,18)"),
    ("N", [3.5, 24.5, 12.5, 32.5], "north bench (7.8,28.5); object E (11,28)"),
]
REFERENCE = {"id": "ref-SWbench-A2", "region": "SW", "window": [-1.5, 6.5, 5.5, 14.2],
             "start": [2.12, 7.72, 0.0], "goal": [1.93, 13.02, 0.0]}


# ----------------------------------------------------------------------------------------------- bookkeeping
def now_utc():
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def elapsed():
    return time.time() - T0


def log(*a):
    print(f"[{elapsed():8.1f}s]", *a, flush=True)


def clean(o):
    """JSON-safe copy: numpy scalars/arrays to Python, non-finite floats to None (A4 crashed on a numpy bool)."""
    if isinstance(o, dict):
        return {str(k): clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    if isinstance(o, np.ndarray):
        return clean(o.tolist())
    if isinstance(o, (bool, np.bool_)):
        return bool(o)
    if isinstance(o, (int, np.integer)):
        return int(o)
    if isinstance(o, (float, np.floating)):
        f = float(o)
        return f if math.isfinite(f) else None
    if o is None or isinstance(o, str):
        return o
    return str(o)


class Store:
    def __init__(self, path):
        self.path = Path(path)
        self.d = {"robot": ROBOT, "status": "running", "started_utc": now_utc(),
                  "job": os.environ.get("SLURM_JOB_ID"), "host": os.uname().nodename, "errors": []}

    def save(self):
        self.d["updated_utc"] = now_utc()
        self.d["elapsed_s"] = round(elapsed(), 1)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_name(self.path.name + ".tmp")
        tmp.write_text(json.dumps(clean(self.d), indent=1))
        os.replace(tmp, self.path)

    def error(self, where, exc):
        tb = traceback.format_exc()
        log("ERROR", where, repr(exc))
        print(tb, flush=True)
        self.d["errors"].append({"where": where, "error": repr(exc), "traceback": tb[-3000:]})
        self.save()


class StageTimeout(BaseException):
    """BaseException, so no `except Exception` inside GMC can swallow it."""


def _on_alarm(signum, frame):
    raise StageTimeout()


def with_timeout(seconds, fn, *a, **kw):
    old = signal.signal(signal.SIGALRM, _on_alarm)
    signal.alarm(max(1, int(seconds)))
    try:
        return fn(*a, **kw)
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old)


# ----------------------------------------------------------------------------------------------- raster helpers
def raster_shape(win):
    return int(np.ceil((win[2] - win[0]) / C)), int(np.ceil((win[3] - win[1]) / C))


def cell_of(p, win, shape):
    i = int((p[0] - win[0]) / C)
    j = int((p[1] - win[1]) / C)
    return min(max(i, 0), shape[0] - 1), min(max(j, 0), shape[1] - 1)


def crop(arr, reg, win, fill):
    """Window cut of a region raster on the same 0.025 m grid; pads with `fill` if rounding runs off the region."""
    nx, ny = raster_shape(win)
    i0 = int(round((win[0] - reg[0]) / C))
    j0 = int(round((win[1] - reg[1]) / C))
    out = np.full((nx, ny), fill, dtype=arr.dtype)
    a = arr[max(i0, 0):i0 + nx, max(j0, 0):j0 + ny]
    out[max(0, -i0):max(0, -i0) + a.shape[0], max(0, -j0):max(0, -j0) + a.shape[1]] = a
    return out


def with_border(occ):
    occ = occ.copy()
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    return occ


def connected(dist, cs, cg, t):
    lab, _ = ndimage.label(dist > t)
    return bool(lab[cs] != 0 and lab[cs] == lab[cg])


def bottleneck(dist, cs, cg, hi=0.60, step=0.0125):
    """Largest t on a 0.0125 m grid in [r, min(clear_s, clear_g, hi)) with start and goal 4-connected in dist > t."""
    ts = np.arange(R, min(float(dist[cs]), float(dist[cg]), hi), step)
    if len(ts) == 0 or not connected(dist, cs, cg, ts[0]):
        return None
    a, b = 0, len(ts) - 1
    while a < b:
        m = (a + b + 1) // 2
        if connected(dist, cs, cg, ts[m]):
            a = m
        else:
            b = m - 1
    return float(ts[a])


def raster_metrics(occ, win, st, gl):
    """The amendment's pre-check numbers on a bordered occupancy raster (4-connected labels, as in A4)."""
    dist = ndimage.distance_transform_edt(~occ) * C
    cs, cg = cell_of(st, win, occ.shape), cell_of(gl, win, occ.shape)
    met = {"start_clear_m": float(dist[cs]), "goal_clear_m": float(dist[cg]),
           "connected_r": connected(dist, cs, cg, R),
           "connected_r_plus_0.025": connected(dist, cs, cg, R + 0.025),
           "connected_r_plus_0.05": connected(dist, cs, cg, R + 0.05),
           "bottleneck_m": bottleneck(dist, cs, cg),
           "disc_free_frac": float((dist > R).mean()), "occupied_frac": float(occ.mean())}
    return dist, cs, cg, met


def raster_path(dist, cs, cg, thr):
    """8-connected shortest path through cells with dist > thr (Dijkstra, step lengths). Returns (ij, length) or None."""
    free = dist > thr
    if not (free[cs] and free[cg]):
        return None
    nx, ny = free.shape
    flat = np.flatnonzero(free.ravel())
    idx = np.full(nx * ny, -1, dtype=np.int64)
    idx[flat] = np.arange(len(flat))
    idx = idx.reshape(nx, ny)
    rows, cols, wts = [], [], []
    for di, dj in ((1, 0), (0, 1), (1, 1), (1, -1)):
        i_s, i_t = slice(0, nx - di), slice(di, nx)
        j_s, j_t = slice(max(0, -dj), ny - max(0, dj)), slice(max(0, dj), ny + min(0, dj))
        m = free[i_s, j_s] & free[i_t, j_t]
        rows.append(idx[i_s, j_s][m])
        cols.append(idx[i_t, j_t][m])
        wts.append(np.full(int(m.sum()), C * math.hypot(di, dj)))
    n = len(flat)
    G = coo_matrix((np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))), shape=(n, n)).tocsr()
    s, g = int(idx[cs]), int(idx[cg])
    d, pred = dijkstra(G, directed=False, indices=s, return_predecessors=True)
    if not np.isfinite(d[g]):
        return None
    seq = [g]
    while seq[-1] != s:
        seq.append(int(pred[seq[-1]]))
    seq = np.array(seq[::-1])
    ij = np.stack(np.unravel_index(flat[seq], (nx, ny)), axis=1)
    return ij, float(d[g])


def path_stats(ij, win, over, dist):
    steps = np.hypot(*np.diff(ij, axis=0).T) * C
    under = over[ij[1:, 0], ij[1:, 1]]
    xy = np.stack([win[0] + (ij[:, 0] + 0.5) * C, win[1] + (ij[:, 1] + 0.5) * C], axis=1)
    return {"path_len_m": float(steps.sum()), "path_under_m": float(steps[under].sum()),
            "path_min_dist_m": float(dist[ij[:, 0], ij[:, 1]].min())}, xy


def polyline_under(xy, win, over, step=0.0125):
    tot = und = 0.0
    for a, b in zip(xy[:-1], xy[1:]):
        L = float(np.hypot(*(b - a)))
        if L == 0:
            continue
        n = max(1, math.ceil(L / step))
        t = (np.arange(n) + 0.5) / n
        pts = a + t[:, None] * (b - a)
        i = np.clip(((pts[:, 0] - win[0]) / C).astype(int), 0, over.shape[0] - 1)
        j = np.clip(((pts[:, 1] - win[1]) / C).astype(int), 0, over.shape[1] - 1)
        tot += L
        und += L * over[i, j].mean()
    return tot, und


def longest_run(mask):
    if not mask.any():
        return 0, None
    d = np.diff(np.concatenate([[0], mask.astype(np.int8), [0]]))
    s, e = np.flatnonzero(d == 1), np.flatnonzero(d == -1)
    k = int(np.argmax(e - s))
    return int(e[k] - s[k]), (int(s[k]), int(e[k]))


def overhang_on_line(s, zb, zt, L):
    """step_case's criterion-1 bins (0.02 m, strip half-width 0.2 m).
    Sweeper version: free low, and overhang rho-bottom in (0.15, 1.75). step_case's z_c = 1.20 count is kept for reference."""
    nb = int(np.ceil(L / SEC_BIN)) + 1
    k = np.clip((s / SEC_BIN).astype(int), 0, nb - 1)
    free_low = np.ones(nb, dtype=bool)
    free_low[k[(zb <= LOW_TOP) & (zt >= 0.02)]] = False
    over_m = (zb > UNDER_LO) & (zb < UNDER_HI)
    over = np.zeros(nb, dtype=bool)
    over[k[over_m]] = True
    under = free_low & over
    runs = np.flatnonzero(under)
    n_long, iv = longest_run(under)
    over_sc = np.zeros(nb, dtype=bool)
    over_sc[k[(zb > 0.15) & (zb < Z_C - 0.10)]] = True
    uav = np.zeros(nb, dtype=bool)
    uav[k[(zb <= Z_C + 0.10) & (zt >= Z_C - 0.10)]] = True
    out = {"runs": int(len(runs)), "longest_run_m": n_long * SEC_BIN,
           "longest_run_s": [iv[0] * SEC_BIN, iv[1] * SEC_BIN] if iv else None,
           "low_blocked_bins": int((~free_low).sum()), "over_bins": int(over.sum()), "bins": nb,
           "step_case_runs_zc1.2": int((free_low & over_sc & ~uav).sum()),
           "top": None, "underside": None, "s_interval": None}
    if len(runs):
        a, b = runs.min() * SEC_BIN, runs.max() * SEC_BIN
        seg = over_m & (s >= a) & (s <= b)
        if seg.any():
            out.update(top=float(zt[seg].max()), underside=float(zb[seg].min()))
        out["s_interval"] = [float(a), float(b)]
    return out


def aligned_window(reg, lo, hi, margin):
    q = 0.05
    x0 = reg[0] + math.floor((lo[0] - margin - reg[0]) / q + 1e-9) * q
    y0 = reg[1] + math.floor((lo[1] - margin - reg[1]) / q + 1e-9) * q
    x1 = reg[0] + math.ceil((hi[0] + margin - reg[0]) / q - 1e-9) * q
    y1 = reg[1] + math.ceil((hi[1] + margin - reg[1]) / q - 1e-9) * q
    if x1 - x0 < MIN_SIDE:
        pad = math.ceil((MIN_SIDE - (x1 - x0)) / 2 / q - 1e-9) * q
        x0, x1 = x0 - pad, x1 + pad
    if y1 - y0 < MIN_SIDE:
        pad = math.ceil((MIN_SIDE - (y1 - y0)) / 2 / q - 1e-9) * q
        y0, y1 = y0 - pad, y1 + pad
    x0, y0, x1, y1 = max(x0, reg[0]), max(y0, reg[1]), min(x1, reg[2]), min(y1, reg[3])
    w = [round(x0, 3), round(y0, 3), round(x1, 3), round(y1, 3)]
    if w[2] - w[0] > MAX_SIDE + 1e-9 or w[3] - w[1] > MAX_SIDE + 1e-9:
        return None
    return w


def glass_clear(win):
    g = GLASS_BOX
    return not (win[0] <= g[2] and win[2] >= g[0] and win[1] <= g[3] and win[3] >= g[1])


def supports_in_window(cen, eh, win):
    """project_scene's final window test (outer shadow box vs window dilated by r) on region supports."""
    return int(((cen[:, 0] - eh[:, 0] <= win[2] + R) & (cen[:, 0] + eh[:, 0] >= win[0] - R)
                & (cen[:, 1] - eh[:, 1] <= win[3] + R) & (cen[:, 1] + eh[:, 1] >= win[1] - R)).sum())


class Sections:
    """Inline showcase_scene._section on pre-extracted opaque arrays (checked equal to _section once per region)."""

    def __init__(self, opq, z_f):
        self.opq = opq
        self.M = opq.means
        self.SZ = RHO * np.sqrt(opq.covs[:, 2, 2])
        self.z_f = z_f
        self.use_plan = False

    def section(self, p0, p1, half=0.2):
        if self.use_plan:
            return _section(self.opq, self.z_f, p0, p1, half=half)
        p0, p1 = np.asarray(p0[:2], float), np.asarray(p1[:2], float)
        d = p1 - p0
        L = float(np.linalg.norm(d))
        u = d / L
        rel = self.M[:, :2] - p0
        s = rel @ u
        off = np.abs(rel @ np.array([-u[1], u[0]]))
        sel = (s >= 0) & (s <= L) & (off <= half)
        return s[sel], self.M[sel, 2] - self.SZ[sel] - self.z_f, self.M[sel, 2] + self.SZ[sel] - self.z_f, L

    def check_against_plan(self, p0, p1):
        a = self.section(p0, p1)
        b = _section(self.opq, self.z_f, p0[:2], p1[:2], half=0.2)
        ok = len(a[0]) == len(b[0]) and all(np.allclose(np.sort(x), np.sort(y)) for x, y in zip(a[:3], b[:3]))
        if not ok:
            self.use_plan = True
        return ok


# ----------------------------------------------------------------------------------------------- ranking
def sup_bucket(n):
    return 0 if n <= 600 else 1 if n <= 1200 else 2 if n <= 2000 else 3 if n <= 3000 else 4


def is_valid(c):
    return (c["start_clear_m"] >= CLEAR_MIN and c["goal_clear_m"] >= CLEAR_MIN and c["connected_r"]
            and c["glass_clear"])


def is_under(c):
    return (c.get("overhang") or {}).get("runs", 0) >= 5


def tier(c):
    b = c.get("bottleneck_m") or 0.0
    clear = min(c["start_clear_m"], c["goal_clear_m"])
    lr = (c.get("overhang") or {}).get("longest_run_m", 0.0)
    pu = c.get("path_under_m")
    if b >= R + 0.05 and clear >= CLEAR_MIN + 0.05 and lr >= 0.3 and (pu is None or pu >= 0.3):
        return 0
    if b >= R + 0.025 and clear >= CLEAR_MIN and lr >= 0.1:
        return 1
    return 2


def n_sup(c):
    return c.get("n_supports", c.get("n_supports_est", 10 ** 9))


def rank_key(c):
    b = c.get("bottleneck_m") or 0.0
    return (0 if is_valid(c) else 1, 0 if is_under(c) else 1, tier(c), sup_bucket(n_sup(c)),
            -round(min(b, R + 0.15), 4), n_sup(c))


def pair_key(c):
    return (c["region"], tuple(np.round(c["start"][:2], 3)), tuple(np.round(c["goal"][:2], 3)))


def case_draft(c, z_f):
    ov = c.get("overhang") or {}
    pre = {k: c.get(k) for k in ("start_clear_m", "goal_clear_m", "connected_r", "connected_r_plus_0.025",
                                 "connected_r_plus_0.05", "bottleneck_m", "disc_free_frac", "occupied_frac",
                                 "path_len_m", "path_under_m", "path_min_dist_m", "precheck_scene")}
    pre.update(raster_cell=C, clearance_required_m=CLEAR_MIN, disc_r=R)
    return {"robot": ROBOT, "window": c["window"], "start": list(c["start"]), "goal": list(c["goal"]),
            "z_floor": z_f, "z_c": Z_C, "behaviour": "under" if is_under(c) else "fallback",
            "overhang": ({"top": ov.get("top"), "underside": ov.get("underside"), "s_interval": ov.get("s_interval")}
                         if is_under(c) else None),
            "precheck": pre, "n_supports": n_sup(c), "claims_boundary": CLAIMS, "candidate_id": c["id"]}


# ----------------------------------------------------------------------------------------------- stage 1
def region_data(name, box, scene, lo_xy, hi_xy, z_f, subset):
    t = time.time()
    if subset:
        m = SUBSET_MARGIN
        sel = ((lo_xy[:, 0] <= box[2] + m) & (hi_xy[:, 0] >= box[0] - m)
               & (lo_xy[:, 1] <= box[3] + m) & (hi_xy[:, 1] >= box[1] - m))
        sub = scene.subset(sel)
    else:
        sub = scene
    s2, pst = project_scene_(sub, ROB, box, z_f)
    occ = _support_raster(s2, box, C)
    dist = ndimage.distance_transform_edt(~occ) * C
    cen = np.array([sp.mean for sp in s2.supports], dtype=float).reshape(-1, 2)
    eh = np.array([sp.level * np.sqrt(np.diag(sp.covariance)) for sp in s2.supports], dtype=float).reshape(-1, 2)
    opq = sub.subset(sub.opacity > TAU)
    sec = Sections(opq, z_f)
    nx, ny = occ.shape
    zb = sec.M[:, 2] - sec.SZ - z_f
    inb = ((sec.M[:, 0] >= box[0]) & (sec.M[:, 0] < box[0] + nx * C)
           & (sec.M[:, 1] >= box[1]) & (sec.M[:, 1] < box[1] + ny * C) & (zb > UNDER_LO) & (zb < UNDER_HI))
    H = np.zeros((nx, ny), dtype=np.int64)
    np.add.at(H, (((sec.M[inb, 0] - box[0]) / C).astype(int), ((sec.M[inb, 1] - box[1]) / C).astype(int)), 1)
    overhang = ndimage.convolve(H, np.ones((3, 3), dtype=np.int64), mode="constant") >= 3
    info = {"box": box, "n_subset": len(sub), "n_opaque_subset": len(opq), "n_supports_region": len(s2.supports),
            "projection": pst, "overhang_cells": int(overhang.sum()),
            "region_disc_free_frac": float((dist > R).mean()), "seconds_maps": round(time.time() - t, 1)}
    return {"name": name, "box": box, "sub": sub, "occ": occ, "dist": dist, "cen": cen, "eh": eh, "sec": sec,
            "overhang": overhang}, info


def project_scene_(scene3d, robot, win, z_f):
    from gmc.height.project import project_scene
    return project_scene(scene3d, robot, win, z_floor=z_f, tau=TAU)


def screen_region(rd, info):
    box, occ, dist, overhang, sec = rd["box"], rd["occ"], rd["dist"], rd["overhang"], rd["sec"]
    t = time.time()
    nx, ny = occ.shape
    under_mask = overhang & (dist > R)
    lab_r, _ = ndimage.label(dist > R)
    over_near = ndimage.binary_dilation(overhang, iterations=10)
    ii, jj = np.arange(EDGE_CELLS, nx - EDGE_CELLS, LATTICE), np.arange(EDGE_CELLS, ny - EDGE_CELLS, LATTICE)
    I, J = [a.ravel() for a in np.meshgrid(ii, jj, indexing="ij")]
    ok = (dist[I, J] >= CLEAR_MIN) & (lab_r[I, J] != 0) & ~over_near[I, J]
    I, J = I[ok], J[ok]
    P = np.stack([box[0] + (I + 0.5) * C, box[1] + (J + 0.5) * C], axis=1)
    lab_e, clear_e = lab_r[I, J], dist[I, J]
    info.update(lattice_points=int(len(ii) * len(jj)), endpoints=int(len(P)))
    NS = int(np.ceil(L_MAX / 0.05)) + 1
    tt = np.linspace(0.0, 1.0, NS)
    pairs = []
    n_geom = 0
    for a in range(len(P)):
        b = np.arange(a + 1, len(P))
        dxy = P[b] - P[a]
        L = np.hypot(dxy[:, 0], dxy[:, 1])
        keep = ((L >= L_MIN) & (L <= L_MAX) & (lab_e[b] == lab_e[a])
                & (np.abs(dxy[:, 0]) <= MAX_SIDE - 1.0) & (np.abs(dxy[:, 1]) <= MAX_SIDE - 1.0))
        if not keep.any():
            continue
        b, dxy, L = b[keep], dxy[keep], L[keep]
        n_geom += len(b)
        X = P[a, 0] + tt[None, :] * dxy[:, :1]
        Y = P[a, 1] + tt[None, :] * dxy[:, 1:]
        ci = np.clip(((X - box[0]) / C).astype(int), 0, nx - 1)
        cj = np.clip(((Y - box[1]) / C).astype(int), 0, ny - 1)
        und = under_mask[ci, cj].sum(axis=1) * (L / (NS - 1))
        blk = (dist[ci, cj] <= R).sum(axis=1) * (L / (NS - 1))
        good = und >= 0.3
        for bb, LL, uu, kk in zip(b[good], L[good], und[good], blk[good]):
            pairs.append((a, int(bb), float(LL), float(uu), float(kk)))
    info.update(pairs_connected_len_ok=int(n_geom), pairs_line_under_raster=int(len(pairs)))

    def pscore(p):
        return min(p[3], 0.8) - 0.08 * p[2] + 0.5 * min(clear_e[p[0]], clear_e[p[1]], 0.6)

    pairs.sort(key=pscore, reverse=True)
    seen, top = set(), []
    for p in pairs:
        ka = (round(P[p[0], 0] / 0.5), round(P[p[0], 1] / 0.5))
        kb = (round(P[p[1], 0] / 0.5), round(P[p[1], 1] / 0.5))
        key = tuple(sorted([ka, kb]))
        if key in seen:
            continue
        seen.add(key)
        top.append(p)
        if len(top) >= TOP_PAIRS:
            break
    info["pairs_top_distinct"] = len(top)

    cands, n_sec, checked = [], 0, False
    for pi, (a, b, L, u, k) in enumerate(top):
        pa, pb = P[a], P[b]
        st, gl = (pa, pb) if (pa[1], pa[0]) <= (pb[1], pb[0]) else (pb, pa)   # start = southern end
        if not checked:
            info["section_inline_equals_plan"] = sec.check_against_plan(st, gl)
            checked = True
        s, zb, zt, LL = sec.section(st, gl)
        ov = overhang_on_line(s, zb, zt, LL)
        if ov["runs"] < 5:
            continue
        n_sec += 1
        for M in MARGINS:
            win = aligned_window(box, np.minimum(st, gl), np.maximum(st, gl), M)
            if win is None:
                continue
            occ_w = with_border(crop(occ, box, win, True))
            dist_w, cs, cg, met = raster_metrics(occ_w, win, st, gl)
            cands.append({"id": f"{rd['name']}-p{pi}-m{M}", "region": rd["name"], "source": "lattice",
                          "start": [round(float(st[0]), 4), round(float(st[1]), 4), 0.0],
                          "goal": [round(float(gl[0]), 4), round(float(gl[1]), 4), 0.0],
                          "L": round(LL, 3), "margin": M, "window": win,
                          "win_size": [round(win[2] - win[0], 3), round(win[3] - win[1], 3)],
                          "n_supports_est": supports_in_window(rd["cen"], rd["eh"], win),
                          "line_under_raster_m": round(u, 3), "line_blocked_raster_m": round(k, 3),
                          "overhang": ov, "glass_clear": glass_clear(win), **met})
    info.update(pairs_section_under=n_sec, windows_evaluated=len(cands),
                windows_valid=sum(is_valid(c) for c in cands),
                windows_valid_tier0=sum(is_valid(c) and tier(c) == 0 for c in cands),
                seconds_screen=round(time.time() - t, 1))
    return cands


def reference_candidates(rd):
    out = []
    st, gl = np.array(REFERENCE["start"][:2]), np.array(REFERENCE["goal"][:2])
    s, zb, zt, L = rd["sec"].section(st, gl)
    ov = overhang_on_line(s, zb, zt, L)
    wins = [("A2", REFERENCE["window"])] + [(f"m{M}", aligned_window(rd["box"], np.minimum(st, gl),
                                                                   np.maximum(st, gl), M)) for M in MARGINS]
    for tag, win in wins:
        if win is None:
            continue
        occ_w = with_border(crop(rd["occ"], rd["box"], win, True))
        _, _, _, met = raster_metrics(occ_w, win, st, gl)
        out.append({"id": f"{REFERENCE['id']}-{tag}", "region": "SW", "source": "reference",
                    "start": list(REFERENCE["start"]), "goal": list(REFERENCE["goal"]), "L": round(L, 3),
                    "margin": tag, "window": win,
                    "win_size": [round(win[2] - win[0], 3), round(win[3] - win[1], 3)],
                    "n_supports_est": supports_in_window(rd["cen"], rd["eh"], win),
                    "overhang": ov, "glass_clear": glass_clear(win), **met})
    return out


def fallback_candidates(rd, n=60):
    """Any connected pair: 2.5-4.5 m apart, widest clearances, margin 0.75/1.0 m."""
    box, occ, dist = rd["box"], rd["occ"], rd["dist"]
    nx, ny = occ.shape
    lab, _ = ndimage.label(dist > R)
    ii, jj = np.arange(EDGE_CELLS, nx - EDGE_CELLS, LATTICE), np.arange(EDGE_CELLS, ny - EDGE_CELLS, LATTICE)
    I, J = [a.ravel() for a in np.meshgrid(ii, jj, indexing="ij")]
    ok = dist[I, J] >= CLEAR_MIN + 0.1
    I, J = I[ok], J[ok]
    order = np.argsort(-dist[I, J])[:400]
    I, J = I[order], J[order]
    P = np.stack([box[0] + (I + 0.5) * C, box[1] + (J + 0.5) * C], axis=1)
    rows = []
    for a in range(len(P)):
        for b in range(a + 1, len(P)):
            L = float(np.hypot(*(P[b] - P[a])))
            if 2.5 <= L <= 4.5 and lab[I[a], J[a]] == lab[I[b], J[b]] != 0:
                rows.append((min(dist[I[a], J[a]], dist[I[b], J[b]]), a, b, L))
    rows.sort(reverse=True)
    out = []
    for k, (_, a, b, L) in enumerate(rows[:n]):
        st, gl = P[a], P[b]
        s, zb, zt, LL = rd["sec"].section(st, gl)
        ov = overhang_on_line(s, zb, zt, LL)
        for M in (0.75, 1.0):
            win = aligned_window(box, np.minimum(st, gl), np.maximum(st, gl), M)
            if win is None:
                continue
            occ_w = with_border(crop(occ, box, win, True))
            _, _, _, met = raster_metrics(occ_w, win, st, gl)
            out.append({"id": f"{rd['name']}-fb{k}-m{M}", "region": rd["name"], "source": "fallback",
                        "start": [round(float(st[0]), 4), round(float(st[1]), 4), 0.0],
                        "goal": [round(float(gl[0]), 4), round(float(gl[1]), 4), 0.0], "L": round(LL, 3),
                        "margin": M, "window": win, "win_size": [round(win[2] - win[0], 3), round(win[3] - win[1], 3)],
                        "n_supports_est": supports_in_window(rd["cen"], rd["eh"], win), "overhang": ov,
                        "glass_clear": glass_clear(win), **met})
    return out


# ----------------------------------------------------------------------------------------------- stages 2-3
def add_raster_path(c, rd, dist=None, occ=None):
    win = c["window"]
    if dist is None:
        occ = with_border(crop(rd["occ"], rd["box"], win, True))
        dist = ndimage.distance_transform_edt(~occ) * C
    over = crop(rd["overhang"], rd["box"], win, False)
    cs, cg = cell_of(c["start"], win, dist.shape), cell_of(c["goal"], win, dist.shape)
    b = c.get("bottleneck_m")
    c["path_len_m"] = c["path_under_m"] = c["path_min_dist_m"] = None
    if b is None:
        return None
    thr = min(b, R + 0.05) - 1e-9
    rp = raster_path(dist, cs, cg, thr)
    if rp is None:
        return None
    ps, xy = path_stats(rp[0], win, over, dist)
    c.update(ps)
    c["path_threshold_m"] = thr
    return xy


def exact_precheck(c, scene_used, rd, z_f, tag):
    t = time.time()
    win = c["window"]
    s2, stats = project_scene_(scene_used, ROB, win, z_f)
    occ = _support_raster(s2, win, C)
    dist, cs, cg, met = raster_metrics(occ, win, c["start"], c["goal"])
    e = dict(c)
    e.update(met)
    e["n_supports"] = int(stats["kept"])
    e["projection"] = stats
    e["precheck_scene"] = tag
    e["raster_crop_mismatch_frac"] = float((with_border(crop(rd["occ"], rd["box"], win, True)) != occ).mean())
    xy = add_raster_path(e, rd, dist=dist, occ=occ)
    over = crop(rd["overhang"], rd["box"], win, False)
    L_line = float(np.hypot(*(np.array(c["goal"][:2]) - np.array(c["start"][:2]))))
    n = max(2, int(L_line / 0.0125))
    pts = np.array(c["start"][:2]) + np.linspace(0, 1, n)[:, None] * (np.array(c["goal"][:2]) - np.array(c["start"][:2]))
    ci = np.clip(((pts[:, 0] - win[0]) / C).astype(int), 0, occ.shape[0] - 1)
    cj = np.clip(((pts[:, 1] - win[1]) / C).astype(int), 0, occ.shape[1] - 1)
    e["line_disc_free_frac"] = float((dist[ci, cj] > R).mean())
    e["line_under_overhang_and_free_m"] = float(((dist[ci, cj] > R) & over[ci, cj]).mean() * L_line)
    e["precheck_seconds"] = round(time.time() - t, 1)
    return e, (s2, occ, dist, xy)


def draw_map(ax, occ, dist, r, win, st, gl, title, comp_cell=None, over=None, path_xy=None, trial_xy=None):
    ext = [win[0], win[0] + occ.shape[0] * C, win[1], win[1] + occ.shape[1] * C]
    img = np.ones(occ.shape[::-1] + (3,))
    img[(dist > r).T] = (0.75, 0.95, 0.75)
    img[occ.T] = (0.8, 0.1, 0.1)
    ax.imshow(img, origin="lower", extent=ext, interpolation="nearest")
    if comp_cell is not None:
        lab, _ = ndimage.label(dist > r)
        if lab[comp_cell]:
            comp = lab == lab[comp_cell]
            ax.imshow(np.ma.masked_where(~comp.T, comp.T), origin="lower", extent=ext, cmap="winter", alpha=0.35,
                      interpolation="nearest")
    if over is not None and over.any():
        xs = win[0] + (np.arange(occ.shape[0]) + 0.5) * C
        ys = win[1] + (np.arange(occ.shape[1]) + 0.5) * C
        ax.contour(xs, ys, over.T.astype(float), levels=[0.5], colors="orange", linewidths=0.9)
    ax.plot([st[0], gl[0]], [st[1], gl[1]], "b-", lw=1)
    if path_xy is not None:
        ax.plot(path_xy[:, 0], path_xy[:, 1], "m--", lw=1.2, label="raster shortest path")
    if trial_xy is not None:
        ax.plot(trial_xy[:, 0], trial_xy[:, 1], "k-", lw=1.8, label="trial GMC path")
    ax.plot(st[0], st[1], "go", ms=8)
    ax.plot(gl[0], gl[1], "bs", ms=8)
    ax.set_xticks(np.arange(math.ceil(ext[0] * 2) / 2, ext[1], 0.5))
    ax.set_yticks(np.arange(math.ceil(ext[2] * 2) / 2, ext[3], 0.5))
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.3, lw=0.4)
    ax.set_title(title, fontsize=9)
    if path_xy is not None or trial_xy is not None:
        ax.legend(fontsize=7, loc="upper right")


def figure(e, maps, rd, scene_used, z_f, rank, trial_xy=None):
    s2, occ, dist, path_xy = maps
    win, st, gl = e["window"], e["start"], e["goal"]
    over = crop(rd["overhang"], rd["box"], win, False)
    fig, axs = plt.subplots(1, 3, figsize=(28, 10), gridspec_kw={"width_ratios": [1, 1.3, 1]})
    cs = cell_of(st, win, occ.shape)
    b = e.get("bottleneck_m")
    draw_map(axs[0], occ, dist, R, win, st, gl,
             f"#{rank} {e['id']} sweeper certified map: {e['n_supports']} supports (red); green = disc r={R:g} fits;\n"
             f"blue tint = start component; orange = overhang mass; clear start {e['start_clear_m']:.2f} "
             f"goal {e['goal_clear_m']:.2f}; bottleneck {b if b is None else round(b, 3)}; "
             f"raster path under {e.get('path_under_m')}", comp_cell=cs, over=over, path_xy=path_xy, trial_xy=trial_xy)
    s, zb, zt, L = rd["sec"].section(st, gl)
    ax = axs[1]
    ax.vlines(s, zb, zt, lw=0.3, color="k", alpha=0.3)
    ax.axhspan(0.02, 1.75, color="r", alpha=0.07, label="cylinder band 0.02-1.75")
    ax.axhspan(0.02, 0.10, color="g", alpha=0.35, label="sweeper band 0.02-0.10")
    ov = e.get("overhang") or {}
    if ov.get("s_interval"):
        ax.axvspan(*ov["s_interval"], color="gold", alpha=0.25, label="free low + overhang bins (span)")
    if ov.get("longest_run_s"):
        ax.axvspan(*ov["longest_run_s"], color="orange", alpha=0.25, label="longest contiguous run")
    ax.set_xlim(0, L)
    ax.set_ylim(-0.15, 2.0)
    ax.set_xlabel("distance along start->goal (m)")
    ax.set_ylabel("height above floor (m)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title(f"section (opaque splats, strip +-0.2 m, rho-extent): runs {ov.get('runs')}, longest "
                 f"{ov.get('longest_run_m')} m, top {ov.get('top')}, underside {ov.get('underside')}", fontsize=9)
    try:
        s2c, pc = project_scene_(scene_used, CYL, win, z_f)
        occc = _support_raster(s2c, win, C)
        distc = ndimage.distance_transform_edt(~occc) * C
        rc = CYL.max_radius()
        n = max(2, int(L / 0.0125))
        pts = np.array(st[:2]) + np.linspace(0, 1, n)[:, None] * (np.array(gl[:2]) - np.array(st[:2]))
        ci = np.clip(((pts[:, 0] - win[0]) / C).astype(int), 0, occc.shape[0] - 1)
        cj = np.clip(((pts[:, 1] - win[1]) / C).astype(int), 0, occc.shape[1] - 1)
        e["cylinder_diag"] = {"n_supports": int(pc["kept"]), "line_blocked_m": float((distc[ci, cj] <= rc).mean() * L),
                              "note": "diagnostic of the overhang only; not a morphology comparison"}
        draw_map(axs[2], occc, distc, rc, win, st, gl,
                 f"cylinder map, same window (diagnostic only): {pc['kept']} supports; green = disc r={rc:g} fits;\n"
                 f"straight line blocked for {e['cylinder_diag']['line_blocked_m']:.2f} m",
                 comp_cell=cell_of(st, win, occc.shape))
    except Exception as exc:  # noqa: BLE001 - the figure must not kill the search
        axs[2].set_title(f"cylinder diagnostic failed: {exc!r}")
    fig.suptitle(f"sweeper case pre-check (selection aid). {CLAIMS}", fontsize=10)
    plt.tight_layout()
    path = FIGS / f"precheck_top{rank}_{e['id']}.png"
    plt.savefig(path, dpi=65)
    plt.close()
    return str(path)


def region_figure(rd, cands):
    box, occ, dist = rd["box"], rd["occ"], rd["dist"]
    fig, ax = plt.subplots(figsize=(14, 14 * (box[3] - box[1]) / (box[2] - box[0])))
    draw_map(ax, occ, dist, R, box, [box[0], box[1]], [box[0], box[1]],
             f"region {rd['name']} {box}: sweeper certified map ({len(rd['cen'])} supports), orange = overhang mass; "
             f"lines = top valid candidates", over=rd["overhang"])
    for c in cands[:25]:
        ax.plot([c["start"][0], c["goal"][0]], [c["start"][1], c["goal"][1]], "-", lw=0.8,
                color="navy" if tier(c) == 0 else "purple")
    plt.tight_layout()
    path = FIGS / f"region_{rd['name']}.png"
    plt.savefig(path, dpi=60)
    plt.close()
    return str(path)


def probe(c, scene_used, z_f, cfg):
    """showcase_run.py's timing probe, verbatim in logic."""
    from gmc.height.run import compile_and_query, with_overrides
    from showcase_run import ABORT_HOURS, PROBE_HALF
    ov = c.get("overhang") or {}
    st, gl = np.array(c["start"][:2]), np.array(c["goal"][:2])
    mid_s = np.mean(ov["s_interval"]) if ov.get("s_interval") else 0.5 * np.linalg.norm(gl - st)
    cc = st + (gl - st) / np.linalg.norm(gl - st) * mid_s
    pw = [cc[0] - PROBE_HALF, cc[1] - PROBE_HALF, cc[0] + PROBE_HALF, cc[1] + PROBE_HALF]
    sp, _ = project_scene_(scene_used, ROB, pw, z_f)
    t0 = time.time()
    pcfg = with_overrides(cfg, initial_intervals=1, max_depth=0)
    res, _ = compile_and_query(sp, ROB, pcfg, (pw[0] + 0.3, cc[1], 0.0), (pw[2] - 0.3, cc[1], 0.0))
    dt = time.time() - t0
    n_int = cfg.orientation.initial_intervals
    ratio = n_sup(c) / max(1, len(sp.supports))
    hours = dt * ratio * n_int / 3600.0
    return {"candidate": c["id"], "n_supports_full": n_sup(c), "n_supports_probe": len(sp.supports),
            "probe_window": pw, "probe_compile_seconds": dt, "projected_hours": hours, "abort": hours > ABORT_HOURS,
            "probe_status": res["status"], "extrapolation": "linear in supports x initial_intervals (rough)"}


def trial(c, scene_used, rd, z_f, cfg):
    from gmc.height.pathio import curve_from_dict, polyline_xy
    from gmc.height.run import compile_and_query
    t = time.time()
    s2, _ = project_scene_(scene_used, ROB, c["window"], z_f)
    res, _ = compile_and_query(s2, ROB, cfg, c["start"], c["goal"])
    out = {k: res.get(k) for k in ("status", "clearance_lb", "reason", "n_supports", "n_pairs", "n_slabs",
                                   "compile_seconds", "query_seconds", "safe_nodes", "safe_edges",
                                   "possible_nodes", "possible_edges", "verify")}
    out.update(candidate=c["id"], note="selection aid; the planning job (percase_run) is the result",
               wall_seconds=round(time.time() - t, 1))
    xy = None
    if res.get("curve") is not None:
        xy = polyline_xy(curve_from_dict(res["curve"]))
        over = crop(rd["overhang"], rd["box"], c["window"], False)
        tot, und = polyline_under(xy, c["window"], over)
        out.update(path_len_m=tot, path_under_overhang_m=und, n_vertices=len(xy),
                   polyline=np.round(xy, 4).tolist() if len(xy) <= 2000 else None)
    return out, xy


# ----------------------------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--regions", default=",".join(r[0] for r in REGIONS))
    ap.add_argument("--trial-minutes", type=float, default=40.0)
    ap.add_argument("--max-trials", type=int, default=2)
    a = ap.parse_args()
    if a.selftest:
        selftest()
        return
    FIGS.mkdir(parents=True, exist_ok=True)
    store = Store(OUT / "search.json")
    D = store.d
    D["constants"] = {"z_c": Z_C, "raster_cell": C, "disc_r": R, "clearance_required": CLEAR_MIN,
                      "band": [ROB.z_lo, ROB.z_hi], "tau": TAU, "rho": RHO, "overhang_underside_range": [UNDER_LO, UNDER_HI],
                      "low_top": LOW_TOP, "section_bin": SEC_BIN, "lattice_m": LATTICE * C, "L_range": [L_MIN, L_MAX],
                      "margins": MARGINS, "max_side": MAX_SIDE, "subset_margin": SUBSET_MARGIN,
                      "claims_boundary": CLAIMS}
    store.save()

    # ---- stage 0
    t = time.time()
    scene, g0 = load_processed()
    assert "floor_rule" in g0, "floor rule not applied"
    z_f = float(g0["floor"]["z_floor"])
    z_file = float(json.loads(G0_FILE.read_text())["floor"]["z_floor"])
    assert z_f == z_file, (z_f, z_file)
    hxy = RHO * np.sqrt(np.stack([scene.covs[:, 0, 0], scene.covs[:, 1, 1]], axis=1))
    lo_xy, hi_xy = scene.means[:, :2] - hxy, scene.means[:, :2] + hxy
    del hxy
    D["scene"] = {"n": len(scene), "floor_rule": g0["floor_rule"], "z_floor": z_f, "load_seconds": round(time.time() - t, 1)}
    log("scene", D["scene"])
    store.save()

    use_subset = True
    try:
        rd_sw, info_sw = region_data("SW", REGIONS[0][1], scene, lo_xy, hi_xy, z_f, subset=True)
        s2f, pf = project_scene_(scene, ROB, REFERENCE["window"], z_f)
        s2s, ps = project_scene_(rd_sw["sub"], ROB, REFERENCE["window"], z_f)
        idf = np.array([sp.primitive_id for sp in s2f.supports])
        ids = np.array([sp.primitive_id for sp in s2s.supports])
        same = (pf["kept"] == ps["kept"] and np.array_equal(idf, ids)
                and np.allclose(np.array([sp.mean for sp in s2f.supports]), np.array([sp.mean for sp in s2s.supports]))
                and np.allclose(np.array([sp.covariance for sp in s2f.supports]),
                                np.array([sp.covariance for sp in s2s.supports])))
        D["subset_check"] = {"window": REFERENCE["window"], "full_kept": pf["kept"], "subset_kept": ps["kept"],
                             "identical": bool(same), "a4_reported_kept": 2796}
        use_subset = bool(same)
        log("subset_check", D["subset_check"])
        del s2f, s2s
    except Exception as exc:  # noqa: BLE001
        store.error("subset_check", exc)
        use_subset = False
        rd_sw = None
    store.save()

    # ---- stage 1
    wanted = set(a.regions.split(","))
    RD, all_cands = {}, []
    D["regions"] = {}
    for name, box, note in REGIONS:
        if name not in wanted:
            continue
        if name != "SW" and elapsed() > 75 * 60:
            D["regions"][name] = {"skipped": "time guard (75 min)"}
            continue
        try:
            t = time.time()
            if name == "SW" and rd_sw is not None and use_subset:
                rd, info = rd_sw, info_sw
            else:
                rd, info = region_data(name, box, scene, lo_xy, hi_xy, z_f, subset=use_subset)
            info["note"] = note
            D["regions"][name] = info
            store.save()
            cands = screen_region(rd, info)
            if name == "SW":
                cands += reference_candidates(rd)
            cands.sort(key=rank_key)
            info["seconds_total"] = round(time.time() - t, 1)
            info["figure"] = region_figure(rd, [c for c in cands if is_valid(c)])
            RD[name] = rd
            all_cands += cands
            log("region", name, json.dumps(clean({k: v for k, v in info.items() if k != "projection"})))
        except Exception as exc:  # noqa: BLE001
            store.error(f"region {name}", exc)
        all_cands.sort(key=rank_key)
        D["screened_counts"] = {"windows": len(all_cands), "valid": sum(map(is_valid, all_cands)),
                                "valid_under": sum(is_valid(c) and is_under(c) for c in all_cands)}
        D["screened_top"] = all_cands[:150]
        store.save()

    valid_under = [c for c in all_cands if is_valid(c) and is_under(c)]
    D["behaviour_search"] = "under" if valid_under else "fallback"
    if not valid_under:
        log("no valid 'under' candidate; fallback search in SW")
        try:
            fb = fallback_candidates(RD["SW"]) if "SW" in RD else []
            fb.sort(key=rank_key)
            D["fallback_screened"] = fb[:60]
            pool = [c for c in fb if is_valid(c)]
        except Exception as exc:  # noqa: BLE001
            store.error("fallback", exc)
            pool = []
    else:
        pool = valid_under
    store.save()

    # ---- stage 2: shortlist with raster paths
    short = []
    try:
        pairs_seen = {}
        for c in pool:
            pk = pair_key(c)
            if pairs_seen.get(pk, 0) >= 2:
                continue
            if len(pairs_seen) >= SHORTLIST_PAIRS and pk not in pairs_seen:
                continue
            pairs_seen[pk] = pairs_seen.get(pk, 0) + 1
            add_raster_path(c, RD[c["region"]])
            short.append(c)
        refs = [c for c in all_cands if c["source"] == "reference" and c not in short]
        for c in refs:
            add_raster_path(c, RD["SW"])
        short.sort(key=rank_key)
        D["shortlist"] = short
        D["reference_screen"] = refs
        log("shortlist", len(short), [(c["id"], tier(c), n_sup(c), c.get("bottleneck_m"), c.get("path_under_m"))
                                     for c in short[:10]])
    except Exception as exc:  # noqa: BLE001
        store.error("shortlist", exc)
    store.save()

    # ---- stage 3: exact pre-check on finalists
    finals = []
    try:
        chosen, seen = [], set()
        for c in short:
            if pair_key(c) in seen:
                continue
            seen.add(pair_key(c))
            chosen.append(c)
            if len(chosen) >= FINALISTS:
                break
        if "SW" in RD:
            refs = [c for c in all_cands if c["source"] == "reference"]
            refs.sort(key=rank_key)
            chosen += [r for r in refs if r["id"].endswith("-A2")] + [r for r in refs[:1] if not r["id"].endswith("-A2")]
        for c in chosen:
            try:
                rd = RD[c["region"]]
                e, _ = exact_precheck(c, rd["sub"] if use_subset else scene, rd, z_f,
                                      "region subset (identical)" if use_subset else "full scene")
                finals.append(e)
                log("precheck", e["id"], e["n_supports"], "est", c["n_supports_est"], e["start_clear_m"], e["goal_clear_m"],
                    e["connected_r"], e["bottleneck_m"], e.get("path_under_m"), "mismatch", e["raster_crop_mismatch_frac"])
            except Exception as exc:  # noqa: BLE001
                store.error(f"precheck {c['id']}", exc)
            D["finalists"] = sorted(finals, key=rank_key)
            store.save()
    except Exception as exc:  # noqa: BLE001
        store.error("finalists", exc)
    finals.sort(key=rank_key)
    finals_valid = [e for e in finals if is_valid(e)]

    # ---- top 3: full-scene pre-check, figures, probe
    top3, maps3 = [], []
    top_src, seen3 = [], set()
    for e in finals_valid:
        if pair_key(e) not in seen3:
            seen3.add(pair_key(e))
            top_src.append(e)
    for rank, e in enumerate(top_src[:3], start=1):
        try:
            rd = RD[e["region"]]
            f, maps = exact_precheck(e, scene, rd, z_f, "full scene")
            f["subset_vs_full_same_supports"] = f["n_supports"] == e["n_supports"]
            f["figure"] = figure(f, maps, rd, scene, z_f, rank)
            top3.append(f)
            maps3.append(maps)
            log("top", rank, f["id"], f["n_supports"], f["figure"])
        except Exception as exc:  # noqa: BLE001
            store.error(f"top3 {e['id']}", exc)
        D["top3"] = top3
        store.save()

    cfg = None
    try:
        from gmc.config import load_config
        cfg = load_config("configs/height_showcase.yaml")
    except Exception as exc:  # noqa: BLE001
        store.error("load_config", exc)
    D["probe"] = []
    if cfg is not None:
        for f in top3:
            try:
                p = with_timeout(15 * 60, probe, f, scene, z_f, cfg)
                D["probe"].append(p)
                log("probe", json.dumps(clean(p)))
            except StageTimeout:
                D["probe"].append({"candidate": f["id"], "timeout_s": 900})
            except Exception as exc:  # noqa: BLE001
                store.error(f"probe {f['id']}", exc)
            store.save()

    # ---- trials (time-boxed)
    D["trials"] = []
    if cfg is not None:
        for k, f in enumerate(top3[:max(0, a.max_trials)]):
            budget = min(a.trial_minutes * 60, JOB_S - elapsed() - 20 * 60)
            if budget < 5 * 60:
                D["trials"].append({"candidate": f["id"], "skipped": "time guard"})
                break
            try:
                log("trial start", f["id"], "budget_s", round(budget))
                tr, xy = with_timeout(budget, trial, f, scene, RD[f["region"]], z_f, cfg)
                D["trials"].append(tr)
                log("trial", json.dumps(clean({kk: v for kk, v in tr.items() if kk != "polyline"})))
                if xy is not None:
                    try:
                        f["figure_trial"] = figure(dict(f), maps3[k], RD[f["region"]], scene, z_f, f"{k + 1}_trial",
                                                   trial_xy=xy)
                    except Exception as exc:  # noqa: BLE001
                        store.error(f"trial figure {f['id']}", exc)
                ok = tr["status"] == "REACHABLE" and (tr.get("verify") or {}).get("certified")
                store.save()
                if ok:
                    break
            except StageTimeout:
                D["trials"].append({"candidate": f["id"], "status": "TRIAL_TIMEOUT", "budget_s": round(budget)})
                log("trial timeout", f["id"])
            except Exception as exc:  # noqa: BLE001
                store.error(f"trial {f['id']}", exc)
            store.save()

    # ---- recommendation
    rec = None
    for f in top3:
        tr = next((t for t in D["trials"] if t.get("candidate") == f["id"] and "status" in t), None)
        if tr and tr["status"] == "REACHABLE" and (tr.get("verify") or {}).get("certified"):
            rec = (f, "top-3 candidate whose trial compile+query was REACHABLE and verify-certified")
            break
    if rec is None and top3:
        rec = (top3[0], "best-ranked full-scene pre-check (no successful trial)")
    if rec is not None:
        D["recommendation"] = {"why": rec[1], "case_draft": case_draft(rec[0], z_f)}
    D["status"] = "done"
    store.save()
    log("done", json.dumps(clean(D.get("recommendation")), indent=1))


def selftest():
    """Synthetic checks of the raster/section helpers (no scene, a few seconds)."""
    win = [0.0, 0.0, 4.0, 3.0]
    occ = np.zeros(raster_shape(win), dtype=bool)
    occ[78:82, :] = True                 # a wall at x ~ 1.95-2.05 ...
    occ[78:82, 50:70] = False            # ... with a 0.5 m gap at y 1.25-1.75
    occ = with_border(occ)
    st, gl = [0.9875, 1.4875, 0.0], [3.0125, 1.4875, 0.0]
    dist, cs, cg, met = raster_metrics(occ, win, st, gl)
    assert met["connected_r"] and met["start_clear_m"] >= CLEAR_MIN, met
    assert met["bottleneck_m"] is not None and R <= met["bottleneck_m"] <= 0.25 + 1e-9, met
    rp = raster_path(dist, cs, cg, R + 0.025)
    assert rp is not None and 2.0 <= rp[1] <= 2.2, rp[1] if rp else None
    over = np.zeros_like(occ)
    over[60:100, 40:80] = True
    ps, xy = path_stats(rp[0], win, over, dist)
    assert 0.9 <= ps["path_under_m"] <= 1.05 and ps["path_min_dist_m"] > R + 0.025, ps
    tot, und = polyline_under(np.array([st[:2], gl[:2]]), win, over)
    assert abs(tot - 2.025) < 1e-6 and abs(und - 1.0) < 0.03, (tot, und)
    occ2 = occ.copy()
    occ2[78:82, :] = True
    _, _, _, met2 = raster_metrics(occ2, win, st, gl)
    assert not met2["connected_r"] and met2["bottleneck_m"] is None, met2
    reg = [-1.0, -1.0, 6.0, 5.0]
    big = np.zeros(raster_shape(reg), dtype=bool)
    big[40 + 78:40 + 82, :] = True
    cw = crop(big, reg, win, True)
    assert cw.shape == occ.shape and cw[78:82, 10].all() and not cw[10, 10], cw.shape
    w = aligned_window(reg, np.array([1.0, 1.0]), np.array([1.2, 4.0]), 0.5)
    assert w == [0.1, 0.5, 2.1, 4.5], w
    assert aligned_window([-10, -10, 10, 10], np.array([0.0, 0.0]), np.array([7.5, 0.0]), 0.5) is None
    s = np.array([0.5, 1.0, 1.02, 1.5, 2.5])
    zb = np.array([-0.01, 0.40, 0.41, 0.42, -0.02])
    zt = np.array([0.05, 0.45, 0.46, 0.45, 0.03])
    ov = overhang_on_line(s, zb, zt, 3.0)
    assert ov["runs"] == 3 and ov["top"] == 0.46 and ov["underside"] == 0.40 and ov["s_interval"] == [1.0, 1.5], ov
    assert ov["longest_run_m"] == 0.04, ov
    txt = json.dumps(clean({"b": np.bool_(True), "i": np.int64(3), "f": np.float32(0.5), "n": float("inf"),
                            "a": np.arange(3), "t": (1, 2)}))
    assert json.loads(txt) == {"b": True, "i": 3, "f": 0.5, "n": None, "a": [0, 1, 2], "t": [1, 2]}, txt
    c = {"start_clear_m": 0.5, "goal_clear_m": 0.4, "connected_r": True, "glass_clear": True, "bottleneck_m": 0.3,
         "overhang": {"runs": 20, "longest_run_m": 0.4}, "n_supports_est": 900, "region": "SW",
         "start": [0, 0, 0], "goal": [1, 1, 0], "id": "x", "window": [0, 0, 3, 3]}
    assert is_valid(c) and is_under(c) and tier(c) == 0 and rank_key(c)[:4] == (0, 0, 0, 1)
    assert glass_clear([-1.5, 6.5, 5.5, 14.2]) and not glass_clear([-5.0, 0.0, 0.0, 4.0])
    d = case_draft(c, -1.2271749593107995)
    assert d["behaviour"] == "under" and d["overhang"]["s_interval"] is None and json.dumps(clean(d))
    signal_ok = False
    try:
        with_timeout(1, time.sleep, 3)
    except StageTimeout:
        signal_ok = True
    assert signal_ok
    print("selftest ok", json.dumps(clean(met)), json.dumps(clean(ps)))


if __name__ == "__main__":
    main()
