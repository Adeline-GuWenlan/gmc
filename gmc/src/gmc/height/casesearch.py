"""Case-search machinery for Amendment 3: the pure part, on rasters and sections (P2a/P2b, reused by P3).

Everything here operates on arrays, so it is testable without loading the 7.3 M-splat scene. The heavy
end — ``project_scene`` -> ``showcase_scene._support_raster(s2, window, 0.025)`` -> EDT — stays in the
experiment scripts, which run under sbatch. The **certified** raster is the only map these functions are
meant to be fed; ``showcase_scene._occupancy`` (the xy-AABB map) is a selection aid and is far more
pessimistic.

Two criteria live here because P2 must not paraphrase them:

* ``overhang_on_line`` is spec §5.3 **criterion 1** ("the straight start->goal segment passes under an
  overhang"), extracted bin for bin from ``showcase_scene.step_case`` and pinned against it by a test.
  It is **unchanged** by Amendment 3.
* ``d1_precheck`` is spec §5.3 criterion 3 **as relaxed by user decision D1**: per robot, start and goal
  clearance >= ``robot.max_radius() + 0.05`` m on that robot's own certified map, and start and goal in
  one connected component of ``dist > r``. The original 0.5 m form is not a data problem — crit1 and
  crit3 intersect in **0.9 m²** of the 990 m² hall as built and **1.2 m²** with every phantom cell
  deleted (``docs/worklog/height_map_diagnosis.md`` §3) — so D1 is a user-approved criterion change and
  every report that uses it must say so and quote both numbers.
"""
import math

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

SEC_BIN = 0.02          # step_case's section bin, metres along the segment
LOW_TOP = 0.15          # "free low": nothing with rho-bottom <= 0.15 m and rho-top >= 0.02 m in the strip
D1_MARGIN = 0.05        # D1: clearance must reach robot.max_radius() + this


# ------------------------------------------------------------------------------------ raster helpers
def with_border(occ):
    """A copy of ``occ`` with the window edge marked occupied (``_support_raster`` does this too)."""
    out = np.array(occ, dtype=bool, copy=True)
    out[0, :] = out[-1, :] = out[:, 0] = out[:, -1] = True
    return out


def edt_clearance(occ, cell):
    """Distance in metres from each cell centre to the nearest occupied cell."""
    return ndimage.distance_transform_edt(~np.asarray(occ, dtype=bool)) * float(cell)


def cell_of(p, window, shape, cell):
    """The raster index of world point ``p``, clamped into ``shape``."""
    i = int(np.clip(int((float(p[0]) - window[0]) / cell), 0, shape[0] - 1))
    j = int(np.clip(int((float(p[1]) - window[1]) / cell), 0, shape[1] - 1))
    return i, j


def clearance_at(dist, window, cell, p):
    return float(dist[cell_of(p, window, dist.shape, cell)])


def free_components(dist, r):
    """4-connected components of ``dist > r`` (the A4/Amendment 2 convention)."""
    lab, n = ndimage.label(dist > float(r))
    return lab, int(n)


def same_component(lab, ci, cj):
    return bool(lab[ci] != 0 and lab[ci] == lab[cj])


# ------------------------------------------------------------------------------------ D1 (criterion 3)
def d1_precheck(dist, window, cell, start, goal, r, margin=D1_MARGIN):
    """Criterion 3 as user decision D1 relaxes it, on one robot's own certified map.

    ``r`` is that robot's ``max_radius()``. Returns the two clearances, whether each reaches
    ``r + margin``, whether start and goal share a component of ``dist > r``, and the conjunction.
    """
    need = float(r) + float(margin)
    cs = cell_of(start, window, dist.shape, cell)
    cg = cell_of(goal, window, dist.shape, cell)
    lab, _ = free_components(dist, r)
    out = {"start_clear_m": float(dist[cs]), "goal_clear_m": float(dist[cg]),
           "clearance_required_m": need, "disc_r": float(r),
           "start_clear_ok": bool(dist[cs] >= need), "goal_clear_ok": bool(dist[cg] >= need),
           "connected": same_component(lab, cs, cg),
           "start_cell": list(cs), "goal_cell": list(cg),
           "criterion": "D1 (user-approved): spec crit3's 0.5 m -> r + 0.05 m on the certified map, "
                        "plus start/goal in one component of dist > r"}
    out["ok"] = bool(out["start_clear_ok"] and out["goal_clear_ok"] and out["connected"])
    return out


# ------------------------------------------------------------------------------------ the straight line
def _samples(p0, p1, step):
    p0, p1 = np.asarray(p0, float)[:2], np.asarray(p1, float)[:2]
    L = float(np.hypot(*(p1 - p0)))
    n = max(2, int(math.ceil(L / step)) + 1)
    t = np.linspace(0.0, 1.0, n)
    return p0 + t[:, None] * (p1 - p0), L


def line_clearance(dist, window, cell, p0, p1, r=0.0, step=None):
    """Clearance profile of the straight start->goal segment on one robot's certified map.

    ``blocked_len_m`` is how much of the segment a disc of radius ``r`` cannot occupy; ``free`` says the
    whole segment is usable by it. This is what makes a case a *morphology* case: the same segment is
    ``free`` for the sweeper and blocked for the cylinder.
    """
    step = float(cell) / 2 if step is None else float(step)
    pts, L = _samples(p0, p1, step)
    i = np.clip(((pts[:, 0] - window[0]) / cell).astype(int), 0, dist.shape[0] - 1)
    j = np.clip(((pts[:, 1] - window[1]) / cell).astype(int), 0, dist.shape[1] - 1)
    d = dist[i, j]
    seg = L / max(1, len(pts) - 1)
    blocked = d <= float(r)
    return {"length_m": L, "min_clearance_m": float(d.min()),
            "blocked_len_m": float(blocked.sum() * seg), "blocked_frac": float(blocked.mean()),
            "free": bool(not blocked.any()), "disc_r": float(r)}


# ------------------------------------------------------------------------------------ geodesics
def free_graph(dist, r, cell):
    """The 8-connected graph over ``dist > r``, with true step lengths in metres.

    Returned once and reused for every source in a region, which is why the P2 search can afford a
    geodesic for hundreds of candidate pairs: ``(G, idx, flat)`` where ``idx`` maps a raster cell to its
    node id (-1 when the cell is not free) and ``flat`` maps a node id back to a raveled cell.
    """
    free = np.asarray(dist, float) > float(r)
    nx, ny = free.shape
    flat = np.flatnonzero(free.ravel())
    idx = np.full(nx * ny, -1, dtype=np.int64)
    idx[flat] = np.arange(len(flat))
    idx = idx.reshape(nx, ny)
    rows, cols, wts = [], [], []
    for di, dj in ((1, 0), (0, 1), (1, 1), (1, -1)):
        i_s, i_t = slice(0, nx - di), slice(di, nx)
        j_s = slice(max(0, -dj), ny - max(0, dj))
        j_t = slice(max(0, dj), ny + min(0, dj))
        m = free[i_s, j_s] & free[i_t, j_t]
        rows.append(idx[i_s, j_s][m])
        cols.append(idx[i_t, j_t][m])
        wts.append(np.full(int(m.sum()), float(cell) * math.hypot(di, dj)))
    G = coo_matrix((np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))),
                   shape=(len(flat), len(flat))).tocsr()
    return G, idx, flat


def path_from_predecessors(pred, flat, shape, s, g):
    """Walk a scipy ``dijkstra`` predecessor row back from node ``g`` to node ``s`` -> raster cells."""
    seq = [int(g)]
    while seq[-1] != int(s):
        nxt = int(pred[seq[-1]])
        if nxt < 0:
            return None
        seq.append(nxt)
    return np.stack(np.unravel_index(flat[np.array(seq[::-1])], shape), axis=1)


def geodesic_path(dist, r, cs, cg, cell):
    """8-connected shortest path through ``dist > r`` (Dijkstra, true step lengths in metres).

    Returns ``(ij, length_m)`` or ``None`` if either endpoint is not free or they are disconnected.
    This is the route a reader compares between robots; it is a *selection aid*, not GMC's certified
    curve.
    """
    cs, cg = tuple(cs), tuple(cg)
    G, idx, flat = free_graph(dist, r, cell)
    if idx[cs] < 0 or idx[cg] < 0:
        return None
    s, g = int(idx[cs]), int(idx[cg])
    d, pred = dijkstra(G, directed=False, indices=s, return_predecessors=True)
    if not np.isfinite(d[g]):
        return None
    ij = path_from_predecessors(pred, flat, dist.shape, s, g)
    return ij, float(d[g])


def path_polyline(ij, window, cell):
    """Raster cells -> world xy at cell centres."""
    ij = np.asarray(ij)
    return np.stack([window[0] + (ij[:, 0] + 0.5) * cell,
                     window[1] + (ij[:, 1] + 0.5) * cell], axis=1)


def _resample(xy, step):
    xy = np.asarray(xy, float)
    seg = np.hypot(*np.diff(xy, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 0:
        return xy
    t = np.arange(0.0, s[-1] + step, step)
    return np.stack([np.interp(t, s, xy[:, 0]), np.interp(t, s, xy[:, 1])], axis=1)


def path_separation(xy_a, xy_b, step=0.01):
    """Symmetric Hausdorff distance in metres between two routes — how far apart they actually run.

    This is the number that answers "are the routes *visibly* different", so it is the primary term of
    :func:`contrast`.
    """
    a, b = _resample(xy_a, step), _resample(xy_b, step)
    d = np.hypot(a[:, 0, None] - b[None, :, 0], a[:, 1, None] - b[None, :, 1])
    return float(max(d.min(axis=1).max(), d.min(axis=0).max()))


def contrast(sweeper_len, cylinder_len, separation_m):
    """Expected morphological contrast of a candidate: how far apart the routes run, scaled by detour.

    ``separation_m`` (metres between the two routes) leads, because that is what a reader sees; the
    detour ratio ``cylinder_len / sweeper_len`` scales it, so of two candidates whose routes separate
    equally the one that costs the cylinder more distance wins. Zero when the routes coincide.
    """
    if sweeper_len is None or cylinder_len is None or separation_m is None:
        return 0.0
    if sweeper_len <= 0:
        return 0.0
    return float(separation_m) * float(cylinder_len) / float(sweeper_len)


def first_affordable(candidates, cylinder_supports, exact_check, cap, max_checks=None):
    """The first candidate, in the order given, whose cylinder map fits under ``cap`` and passes exactly.

    ``candidates`` is a ranked list (P2's own order). ``cylinder_supports(c)`` is called on every candidate
    tried; ``exact_check(c)`` -- the unchanged per-window verdict, a dict with ``pass`` and
    ``failed_criteria`` -- only on those with at most ``cap`` supports. The filter can only remove
    candidates, never admit one the exact check rejects. Returns ``(verdict_or_None, tried)``; ``tried``
    records every candidate walked, so a reader sees what was skipped and why.
    """
    tried = []
    for c in candidates:
        if max_checks is not None and len(tried) >= max_checks:
            break
        n = int(cylinder_supports(c))
        row = {"window": list(c["window"]), "start": list(c["start"]), "goal": list(c["goal"]),
               "cylinder_supports": n}
        tried.append(row)
        if n > cap:
            row["verdict"] = "over_cap"
            continue
        ev = exact_check(c)
        row["verdict"] = "pass" if ev.get("pass") else "fails:" + ",".join(ev.get("failed_criteria", []))
        if ev.get("pass"):
            return ev, tried
    return None, tried


# ------------------------------------------------------------------------------------ criterion 1
def overhang_on_line(s, zb, zt, L, *, z_c, ceiling_height_m, bin_m=SEC_BIN, low_top=LOW_TOP):
    """Spec §5.3 criterion 1, bin for bin as ``showcase_scene.step_case`` computes it. Unchanged.

    ``(s, zb, zt, L)`` is what ``showcase_scene._section`` returns: distance along the segment and the
    rho-AABB bottom/top of each opaque splat in the +-0.2 m strip, heights relative to the floor.

    A bin counts when the low band is free (nothing with rho-bottom <= ``low_top`` and rho-top >= 0.02),
    something hangs over it (rho-bottom in ``(low_top, z_c - 0.10)``) and the uav band is clear.
    """
    s, zb, zt = np.asarray(s, float), np.asarray(zb, float), np.asarray(zt, float)
    nb = int(np.ceil(L / bin_m)) + 1
    free_low = np.ones(nb, dtype=bool)
    over = np.zeros(nb, dtype=bool)
    uav_hit = np.zeros(nb, dtype=bool)
    k = np.clip((s / bin_m).astype(int), 0, nb - 1)
    free_low[k[(zb <= low_top) & (zt >= 0.02)]] = False
    over[k[(zb > low_top) & (zb < z_c - 0.10)]] = True
    uav_hit[k[(zb <= z_c + 0.10) & (zt >= z_c - 0.10)]] = True
    runs = np.flatnonzero(free_low & over & ~uav_hit)
    out = {"runs": int(len(runs)), "length_m": float(L), "z_c": float(z_c),
           "straight_line_under_overhang": bool(len(runs) >= 5),
           "top": None, "underside": None, "s_interval": None,
           "longest_run_m": float(_longest_run(free_low & over & ~uav_hit) * bin_m),
           "ceiling_side_obstacle_above_floor": float(ceiling_height_m),
           "zc_above_overhang_top_by_0.25": False, "zc_band_below_ceiling_side_by_0.30": False}
    if len(runs):
        seg = ((zb > low_top) & (zb < z_c - 0.10)
               & (s >= runs.min() * bin_m) & (s <= runs.max() * bin_m))
        top = float(zt[seg].max()) if seg.any() else None
        out["top"] = top
        out["underside"] = float(zb[seg].min()) if seg.any() else None
        out["s_interval"] = [runs.min() * bin_m, runs.max() * bin_m]
        higher = zb[zb > (top or 0) + 0.5]
        ceil_side = float(higher.min()) if len(higher) else float(ceiling_height_m)
        out["ceiling_side_obstacle_above_floor"] = ceil_side
        out["zc_above_overhang_top_by_0.25"] = bool(top is not None and z_c >= top + 0.25)
        out["zc_band_below_ceiling_side_by_0.30"] = bool(z_c + 0.10 <= ceil_side - 0.30)
    out["ok"] = bool(out["straight_line_under_overhang"] and out["zc_above_overhang_top_by_0.25"]
                     and out["zc_band_below_ceiling_side_by_0.30"])
    return out


def _longest_run(mask):
    if not mask.any():
        return 0
    d = np.diff(np.concatenate([[0], mask.astype(np.int8), [0]]))
    return int((np.flatnonzero(d == -1) - np.flatnonzero(d == 1)).max())
