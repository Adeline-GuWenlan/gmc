# gmc/experiments/percase_search_uav.py
"""Amendment 2 case search for the uav (disc r = 0.25 m, band z_c +- 0.10 m above the floor).

Preferred behaviour: the straight start->goal passes OVER a table, z_c by the spec §3.3 rule
(z_c >= overhang top + 0.25 m; z_c + 0.10 <= lowest ceiling-side obstacle - 0.30 m). Fallback: any connected
pair at z_c = 1.20. This is selection only; showcase_run.py re-projects the chosen case from the full scene.

Pre-check on the certified map (amendment): project_scene -> showcase_scene._support_raster(s2, window, 0.025)
-> distance transform; start and goal clearance >= r + 0.05 and start, goal in one component of dist > r.

z_c rule, per straight line and z_c in 1.20..1.60:
  * step_case's criterion-1 block verbatim (section half-width 0.2 m, 2 cm bins) gives its overhang top;
  * a conservative tabletop top: highest rho-top of splats with underside in (0.15, z_c - 0.10) over the
    geometric table crossing grown by r, in a section of half-width r + 0.05;
  * top = max of the two; zc_ok: z_c >= top + 0.25;
  * ceil_ok (step_case's definition, threshold min(top) + 0.5): z_c + 0.10 <= ceiling-side - 0.30;
  * ceil_strict_ok (preference tier only): anything whose underside is above the band top, within r + 0.30 of
    the line, is >= 0.30 m above the band top;
  * no splat overlaps the band over the crossing (+- r).

Speed: projection runs on a pre-subset (splats whose rho-AABB meets the window grown by 2 m in xy and the band
grown by 0.2 m in z). project_scene's window test uses the same rho-AABB grown by r <= 0.25, so the kept supports
are identical; this is re-checked on the full scene for the top candidates (ids compared).

Run from gmc/: python experiments/percase_search_uav.py [--synthetic --out DIR]
"""
import argparse
import json
import time
import traceback
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage

from gmc.height.ply3d import GaussianScene3D
from gmc.height.prism import robot_table
from gmc.height.project import project_scene
from showcase_scene import RHO, TAU, _section, _support_raster, load_processed

RASTER = 0.025
R_UAV = 0.25
HALF_BAND = 0.10
CLEAR_MIN = R_UAV + 0.05
ZCS = [round(float(z), 2) for z in np.arange(1.20, 1.601, 0.05)]
BIN = 0.02
SEC_HALF = 0.2
TOP_HALF = R_UAV + 0.05
CEIL_CORRIDOR_HALF = R_UAV + 0.30
XY_MARGIN = 2.0
Z_MARGIN = 0.2
REGION_HALF = 7.5
SEC_REGION_HALF = 5.5
MAX_WIN = 8.0
GLASS_ZONE = (-4.9, -0.1, -2.9, 3.6)   # glass doors x ~ -3.9, y 0.9-2.6, grown by 1 m
WALL_BUDGET_S = 3.3 * 3600
CLAIMS = ("Per-robot showcase case chosen for success: not a morphology comparison at one place, and no "
          "'body shape changes the route' claim may rest on it; floor-surface rule (Amendment 1) on.")

# A4 table_frames.py (flat tabletop splats, 2-98 % extents): centre, long-axis angle (deg), length, width.
TABLES = {
    "A": ((8.96, 7.29), -22.9, 2.55, 0.72),
    "C": ((13.28, 14.09), -21.4, 1.74, 0.68),
    "B": ((-0.45, 10.81), 60.9, 2.35, 0.93),
    "G": ((3.35, 22.53), 31.5, 1.64, 0.62),
}


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


def frame(spec):
    (cx, cy), deg, L, W = spec
    t = np.radians(deg)
    return np.array([cx, cy]), np.array([np.cos(t), np.sin(t)]), np.array([-np.sin(t), np.cos(t)]), L, W


def crossing(st, gl, fr):
    """Metres along st->gl inside the table rectangle (Liang-Barsky), or None."""
    c, v, u, L, W = fr
    p = np.asarray(st[:2], float) - c
    d = np.asarray(gl[:2], float) - np.asarray(st[:2], float)
    Ls = float(np.linalg.norm(d))
    t0, t1 = 0.0, 1.0
    for axis, half in ((v, L / 2), (u, W / 2)):
        pa, da = float(p @ axis), float(d @ axis)
        if abs(da) < 1e-12:
            if abs(pa) > half:
                return None
            continue
        ta, tb = sorted(((-half - pa) / da, (half - pa) / da))
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 >= t1:
            return None
    return [t0 * Ls, t1 * Ls]


def section(sec, p0, p1, half):
    """showcase_scene._section on precomputed opaque arrays (asserted equal once per table)."""
    p0, p1 = np.asarray(p0[:2], float), np.asarray(p1[:2], float)
    d = p1 - p0
    L = float(np.linalg.norm(d))
    u = d / L
    rel = sec["xy"] - p0
    s = rel @ u
    off = np.abs(rel @ np.array([-u[1], u[0]]))
    sel = (s >= 0) & (s <= L) & (off <= half)
    return s[sel], sec["zb"][sel], sec["zt"][sel], L


def zc_rows(sec, st, gl, fr, ceil_h):
    s, zb, zt, L = section(sec, st, gl, SEC_HALF)
    s3, zb3, zt3, _ = section(sec, st, gl, TOP_HALF)
    _, zb5, _, _ = section(sec, st, gl, CEIL_CORRIDOR_HALF)
    cross = crossing(st, gl, fr) if fr is not None else None
    nb = int(np.ceil(L / BIN)) + 1
    k = np.clip((s / BIN).astype(int), 0, nb - 1)
    rows = []
    for z_c in ZCS:
        # --- step_case criterion-1 block (verbatim logic) ---
        free_low = np.ones(nb, dtype=bool)
        over = np.zeros(nb, dtype=bool)
        uav_hit = np.zeros(nb, dtype=bool)
        free_low[k[(zb <= 0.15) & (zt >= 0.02)]] = False
        over[k[(zb > 0.15) & (zb < z_c - 0.10)]] = True
        uav_hit[k[(zb <= z_c + 0.10) & (zt >= z_c - 0.10)]] = True
        runs = np.flatnonzero(free_low & over & ~uav_hit)
        r = {"z_c": z_c, "runs": int(len(runs)), "uav_hit_bins_line": int(uav_hit.sum()),
             "top_step_case": None, "under_step_case": None, "s_interval_step_case": None}
        if len(runs):
            seg = (zb > 0.15) & (zb < z_c - 0.10) & (s >= runs.min() * BIN) & (s <= runs.max() * BIN)
            if seg.any():
                r["top_step_case"] = float(zt[seg].max())
                r["under_step_case"] = float(zb[seg].min())
            r["s_interval_step_case"] = [float(runs.min() * BIN), float(runs.max() * BIN)]
        # --- conservative tabletop over the crossing +- r ---
        r.update(crossing=cross, crossing_len=0.0 if cross is None else cross[1] - cross[0],
                 top_table=None, under_table=None, band_hits_over_table=None)
        if cross is not None:
            a0, a1 = cross[0] - R_UAV, cross[1] + R_UAV
            ins = (s3 >= a0) & (s3 <= a1)
            m3 = ins & (zb3 > 0.15) & (zb3 < z_c - 0.10)
            if m3.any():
                r["top_table"] = float(zt3[m3].max())
                r["under_table"] = float(zb3[m3].min())
            r["band_hits_over_table"] = int((ins & (zb3 <= z_c + 0.10) & (zt3 >= z_c - 0.10)).sum())
        tops = [t for t in (r["top_step_case"], r["top_table"]) if t is not None]
        r["top"] = max(tops) if tops else None
        r["underside"] = r["under_table"] if r["under_table"] is not None else r["under_step_case"]
        if tops:
            higher = zb[zb > min(tops) + 0.5]
            r["ceiling_side"] = float(higher.min()) if len(higher) else float(ceil_h)
        else:
            r["ceiling_side"] = None
        above = zb5[zb5 > z_c + HALF_BAND]
        r["ceiling_strict"] = float(above.min()) if len(above) else float(ceil_h)
        r["zc_ok"] = bool(r["top"] is not None and z_c >= r["top"] + 0.25)
        r["ceil_ok"] = bool(r["ceiling_side"] is not None and z_c + 0.10 <= r["ceiling_side"] - 0.30)
        r["ceil_strict_ok"] = bool(z_c + 0.10 <= r["ceiling_strict"] - 0.30)
        over_ok = (r["crossing_len"] >= 0.3 and r["zc_ok"] and r["ceil_ok"]
                   and r["band_hits_over_table"] == 0)
        r["tier"] = (0 if r["ceil_strict_ok"] else 1) if over_ok else None
        rows.append(r)
    return rows, L


def gen_lines(tag, fr):
    c, v, u, L, W = fr
    out = []
    n = int(round(0.8 * L / 0.2)) + 1
    for i, a in enumerate(np.linspace(-0.4 * L, 0.4 * L, n)):
        P = c + a * v
        for d0, d1 in ((0.9, 0.9), (1.3, 1.3), (0.9, 1.3), (1.3, 0.9)):
            out.append(("short", {"a": float(a), "d": [d0, d1]}, P - (W / 2 + d0) * u, P + (W / 2 + d1) * u))
    for b in (-0.25 * W, 0.0, 0.25 * W):
        P = c + b * u
        for d in (0.8, 1.2):
            out.append(("long", {"b": float(b), "d": d}, P - (L / 2 + d) * v, P + (L / 2 + d) * v))
    for ang in (40.0, -40.0):
        t = np.radians(ang)
        w = np.array([np.cos(t) * u[0] - np.sin(t) * u[1], np.sin(t) * u[0] + np.cos(t) * u[1]])
        for a in (-0.3 * L, 0.0, 0.3 * L):
            P = c + a * v
            for D in (1.6, 2.1):
                out.append(("diag", {"ang": ang, "a": float(a), "D": D}, P - D * w, P + D * w))
    for b in (W / 2 + 0.9, -(W / 2 + 0.9), W / 2 + 1.4, -(W / 2 + 1.4)):
        P = c + b * u
        out.append(("parallel", {"b": float(b)}, P - 1.4 * v, P + 1.4 * v))
    return [{"id": f"{tag}-{fam}-{i}", "table": tag, "family": fam, "params": prm,
             "start": [round(float(st[0]), 3), round(float(st[1]), 3)],
             "goal": [round(float(gl[0]), 3), round(float(gl[1]), 3)]}
            for i, (fam, prm, st, gl) in enumerate(out)]


def _f05(x):
    return round(float(np.floor(x / 0.05) * 0.05), 2)


def _c05(x):
    return round(float(np.ceil(x / 0.05) * 0.05), 2)


def gen_windows(st, gl):
    st, gl = np.asarray(st, float), np.asarray(gl, float)
    lo, hi = np.minimum(st, gl), np.maximum(st, gl)
    wins = [(f"bbox{m:.1f}", [_f05(lo[0] - m), _f05(lo[1] - m), _c05(hi[0] + m), _c05(hi[1] + m)])
            for m in (0.7, 1.0, 1.4)]
    mid = (st + gl) / 2
    side = _c05(max(hi - lo) + 2.0)
    wins.append(("square", [round(mid[0] - side / 2, 2), round(mid[1] - side / 2, 2),
                            round(mid[0] + side / 2, 2), round(mid[1] + side / 2, 2)]))
    out, seen = [], set()
    for tag, w in wins:
        if w[2] - w[0] > MAX_WIN + 1e-9 or w[3] - w[1] > MAX_WIN + 1e-9 or tuple(w) in seen:
            continue
        g = GLASS_ZONE
        if not (w[2] < g[0] or w[0] > g[2] or w[3] < g[1] or w[1] > g[3]):
            continue
        seen.add(tuple(w))
        out.append((tag, w))
    return out


class Prechecker:
    """Per-table projection pre-subset and certified-raster cache."""

    def __init__(self, scene, LO, HI, z_f, centre):
        self.z_f = z_f
        c = np.asarray(centre, float)
        self.box = [c[0] - REGION_HALF, c[1] - REGION_HALF, c[0] + REGION_HALF, c[1] + REGION_HALF]
        zlo = z_f + ZCS[0] - HALF_BAND - Z_MARGIN
        zhi = z_f + ZCS[-1] + HALF_BAND + Z_MARGIN
        m = ((HI[:, 0] >= self.box[0]) & (LO[:, 0] <= self.box[2]) & (HI[:, 1] >= self.box[1])
             & (LO[:, 1] <= self.box[3]) & (HI[:, 2] >= zlo) & (LO[:, 2] <= zhi))
        self.sub = scene.subset(m)
        self.lo, self.hi = LO[m], HI[m]
        self.cache = {}

    def window_subset(self, win, z_c):
        x0, y0, x1, y1 = win
        b = self.box
        if not (x0 - XY_MARGIN >= b[0] and y0 - XY_MARGIN >= b[1] and x1 + XY_MARGIN <= b[2]
                and y1 + XY_MARGIN <= b[3]):
            raise ValueError(f"window {win} not inside the pre-subset region {b} with margin")
        zlo, zhi = self.z_f + z_c - HALF_BAND - Z_MARGIN, self.z_f + z_c + HALF_BAND + Z_MARGIN
        m = ((self.hi[:, 0] >= x0 - XY_MARGIN) & (self.lo[:, 0] <= x1 + XY_MARGIN)
             & (self.hi[:, 1] >= y0 - XY_MARGIN) & (self.lo[:, 1] <= y1 + XY_MARGIN)
             & (self.hi[:, 2] >= zlo) & (self.lo[:, 2] <= zhi))
        return self.sub.subset(m)

    def project(self, win, z_c):
        return project_scene(self.window_subset(win, z_c), robot_table(z_c)["uav"], win, z_floor=self.z_f)

    def maps(self, win, z_c):
        key = (tuple(win), z_c)
        if key not in self.cache:
            t = time.time()
            s2, st = self.project(win, z_c)
            occ = _support_raster(s2, win, RASTER)
            dist = ndimage.distance_transform_edt(~occ) * RASTER
            lab, ncomp = ndimage.label(dist > R_UAV)
            self.cache[key] = {"dist": dist.astype(np.float32), "lab": lab.astype(np.int32),
                               "ncomp": int(ncomp), "n_supports": len(s2.supports), "stats": st,
                               "free_frac": float((dist > R_UAV).mean()), "occ_frac": float(occ.mean()),
                               "seconds": time.time() - t}
        return self.cache[key]


def _cell(p, win, shape):
    return (min(max(int((p[0] - win[0]) / RASTER), 0), shape[0] - 1),
            min(max(int((p[1] - win[1]) / RASTER), 0), shape[1] - 1))


def evaluate(pc, line, row, win_tag, win, behaviour):
    z_c = row["z_c"]
    mp = pc.maps(win, z_c)
    dist, lab = mp["dist"], mp["lab"]
    st, gl = np.asarray(line["start"], float), np.asarray(line["goal"], float)
    cs, cg = _cell(st, win, dist.shape), _cell(gl, win, dist.shape)
    L = float(np.linalg.norm(gl - st))
    ts = np.linspace(0.0, 1.0, max(2, int(L / (RASTER / 2)) + 1))
    pts = st[None, :] + ts[:, None] * (gl - st)[None, :]
    ix = np.clip(((pts[:, 0] - win[0]) / RASTER).astype(int), 0, dist.shape[0] - 1)
    iy = np.clip(((pts[:, 1] - win[1]) / RASTER).astype(int), 0, dist.shape[1] - 1)
    lmin = float(dist[ix, iy].min())
    sc, gc = float(dist[cs]), float(dist[cg])
    same = bool(lab[cs] != 0 and lab[cs] == lab[cg])
    return {"line_id": line["id"], "table": line["table"], "family": line["family"], "params": line["params"],
            "start": line["start"], "goal": line["goal"], "length": L, "behaviour": behaviour,
            "tier": row["tier"] if behaviour == "over" else 2, "z_c": z_c, "window_tag": win_tag,
            "window": win, "window_size": [round(win[2] - win[0], 2), round(win[3] - win[1], 2)],
            "n_supports": mp["n_supports"], "projection_subset_stats": mp["stats"],
            "start_clear": sc, "goal_clear": gc, "same_component": same, "n_components": mp["ncomp"],
            "free_frac": mp["free_frac"], "occ_frac": mp["occ_frac"], "line_min_dist": lmin,
            "line_clear": bool(lmin > R_UAV + RASTER), "precheck_seconds": mp["seconds"],
            "precheck_pass": bool(sc >= CLEAR_MIN and gc >= CLEAR_MIN and same),
            "rule": {k: row[k] for k in ("runs", "top", "top_step_case", "top_table", "underside", "crossing",
                                         "crossing_len", "s_interval_step_case", "ceiling_side",
                                         "ceiling_strict", "zc_ok", "ceil_ok", "ceil_strict_ok",
                                         "band_hits_over_table", "uav_hit_bins_line")}}


def _bucket(n):
    return 0 if n <= 300 else 1 if n <= 1500 else 2 if n <= 5000 else 3


def rank_key(c):
    return (c["tier"], not c["line_clear"], _bucket(c["n_supports"]),
            -round(min(c["start_clear"], c["goal_clear"], 1.0), 1), c["z_c"],
            -round(c["rule"]["crossing_len"], 1), c["n_supports"],
            c["window_size"][0] * c["window_size"][1])


def table_rect(fr):
    c, v, u, L, W = fr
    return np.array([c + sa * L / 2 * v + sb * W / 2 * u
                     for sa, sb in ((-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1))])


def height_map(sec, win, cell=0.05, zmax=2.5):
    x0, y0, x1, y1 = win
    nx, ny = int(np.ceil((x1 - x0) / cell)), int(np.ceil((y1 - y0) / cell))
    xy, zt = sec["xy"], sec["zt"]
    m = (xy[:, 0] >= x0) & (xy[:, 0] < x1) & (xy[:, 1] >= y0) & (xy[:, 1] < y1) & (zt < zmax)
    hm = np.full((nx, ny), np.nan)
    ix = np.clip(((xy[m, 0] - x0) / cell).astype(int), 0, nx - 1)
    iy = np.clip(((xy[m, 1] - y0) / cell).astype(int), 0, ny - 1)
    np.fmax.at(hm, (ix, iy), zt[m])
    return hm


def fig_candidate(cand, pc, sec, fr, ceil_h, path, label):
    win, z_c = cand["window"], cand["z_c"]
    s2, _ = pc.project(win, z_c)
    occ = _support_raster(s2, win, RASTER)
    dist = ndimage.distance_transform_edt(~occ) * RASTER
    lab, _ = ndimage.label(dist > R_UAV)
    st, gl = np.asarray(cand["start"]), np.asarray(cand["goal"])
    ext = [win[0], win[2], win[1], win[3]]
    fig, ax = plt.subplots(1, 3, figsize=(27, 9))
    img = np.ones(occ.shape[::-1] + (3,))
    img[(dist > R_UAV).T] = (0.75, 0.95, 0.75)
    img[occ.T] = (0.8, 0.1, 0.1)
    ax[0].imshow(img, origin="lower", extent=ext, interpolation="nearest")
    cs = _cell(st, win, occ.shape)
    if lab[cs]:
        comp = lab == lab[cs]
        ax[0].imshow(np.ma.masked_where(~comp.T, comp.T), origin="lower", extent=ext, cmap="winter",
                     alpha=0.35, interpolation="nearest")
    hm = height_map(sec, win)
    h = ax[1].imshow(hm.T, origin="lower", extent=ext, cmap="viridis", vmin=0, vmax=2.5, interpolation="nearest")
    plt.colorbar(h, ax=ax[1], shrink=0.7, label="highest rho-top above floor, splats below 2.5 m (m)")
    for a in ax[:2]:
        a.plot([st[0], gl[0]], [st[1], gl[1]], "b-", lw=1.2)
        a.plot(*st, "go", ms=8)
        a.plot(*gl, "bs", ms=8)
        if fr is not None:
            a.plot(*table_rect(fr).T, "k--", lw=1)
        a.set_xlim(win[0], win[2]); a.set_ylim(win[1], win[3])
        a.set_xticks(np.arange(np.ceil(win[0] * 2) / 2, win[2], 0.5))
        a.set_yticks(np.arange(np.ceil(win[1] * 2) / 2, win[3], 0.5))
        a.tick_params(labelsize=7); a.grid(alpha=0.3, lw=0.4); a.set_aspect("equal")
    ax[0].set_title(f"{label} uav z_c={z_c:.2f}: red certified shadows ({cand['n_supports']}); green disc r=0.25 fits "
                    f"({cand['free_frac']:.0%}); blue tint start component\nstart clear {cand['start_clear']:.2f}, "
                    f"goal clear {cand['goal_clear']:.2f}, same comp {cand['same_component']}, "
                    f"line min dist {cand['line_min_dist']:.2f}", fontsize=10)
    ax[1].set_title(f"{cand['line_id']} [{cand['behaviour']}, tier {cand['tier']}]: top-down height; "
                    f"dashed = table frame; window {win}", fontsize=10)
    s, zb, zt, L = section(sec, st, gl, TOP_HALF)
    ax[2].vlines(s, np.maximum(zb, -0.1), np.minimum(zt, ceil_h + 0.2), lw=0.3, color="k", alpha=0.3)
    ax[2].axhspan(z_c - HALF_BAND, z_c + HALF_BAND, color="b", alpha=0.2, label="uav band")
    rule = cand["rule"]
    if rule["top"] is not None:
        ax[2].axhline(rule["top"], color="orange", lw=1, label=f"overhang top {rule['top']:.3f}")
        ax[2].axhline(rule["top"] + 0.25, color="orange", ls="--", lw=1, label="top + 0.25 (z_c lower limit)")
    if rule["ceiling_side"] is not None:
        ax[2].axhline(rule["ceiling_side"] - 0.30, color="r", ls="--", lw=1,
                      label=f"ceiling-side {rule['ceiling_side']:.2f} - 0.30")
    ax[2].axhline(rule["ceiling_strict"] - 0.30, color="m", ls=":", lw=1,
                  label=f"lowest underside above band {rule['ceiling_strict']:.2f} - 0.30")
    if rule["crossing"]:
        ax[2].axvspan(*rule["crossing"], color="orange", alpha=0.12, label="table crossing")
    ax[2].set_ylim(-0.1, ceil_h + 0.2); ax[2].set_xlim(0, L)
    ax[2].set_xlabel("distance along start->goal (m)"); ax[2].set_ylabel("height above floor (m)")
    ax[2].legend(fontsize=8, loc="upper right"); ax[2].grid(alpha=0.3)
    ax[2].set_title(f"section half-width {TOP_HALF:.2f} m (rho-extents of opaque splats)", fontsize=10)
    plt.tight_layout(); plt.savefig(path, dpi=70); plt.close()


def fig_overview(tag, fr, sec, lines, cands, path):
    c = fr[0]
    win = [c[0] - 4.5, c[1] - 4.5, c[0] + 4.5, c[1] + 4.5]
    fig, ax = plt.subplots(figsize=(11, 10))
    h = ax.imshow(height_map(sec, win).T, origin="lower", extent=[win[0], win[2], win[1], win[3]],
                  cmap="Greys", vmin=0, vmax=2.5, interpolation="nearest")
    plt.colorbar(h, ax=ax, shrink=0.7, label="highest rho-top below 2.5 m")
    best = {}
    for cd in cands:
        if cd["line_id"] not in best or rank_key(cd) < rank_key(best[cd["line_id"]]):
            best[cd["line_id"]] = cd
    for cd in best.values():
        colr = ("g" if cd["precheck_pass"] else "orange") if cd["behaviour"] == "over" else \
               ("c" if cd["precheck_pass"] else "0.6")
        ax.plot([cd["start"][0], cd["goal"][0]], [cd["start"][1], cd["goal"][1]], "-", color=colr, lw=0.8)
    ax.plot(*table_rect(fr).T, "r--", lw=1)
    ax.set_title(f"table {tag}: best candidate per line; green over+precheck pass, orange over+fail, "
                 f"cyan fallback pass, grey fallback fail", fontsize=9)
    ax.grid(alpha=0.3); ax.set_aspect("equal")
    plt.tight_layout(); plt.savefig(path, dpi=70); plt.close()


def synthetic_scene():
    """Tiny scene for a login-node smoke test (floor, one rotated table with legs and an item, wall, ceiling)."""
    means, sig = [], []

    def add(xyz, s):
        xyz = np.atleast_2d(np.asarray(xyz, float))
        means.append(xyz)
        sig.append(np.broadcast_to(np.asarray(s, float), xyz.shape))

    gx, gy = np.meshgrid(np.arange(-3, 3.01, 0.2), np.arange(-3, 3.01, 0.2))
    add(np.c_[gx.ravel(), gy.ravel(), np.zeros(gx.size)], (0.06, 0.06, 0.003))
    t = np.radians(20.0)
    v, u = np.array([np.cos(t), np.sin(t)]), np.array([-np.sin(t), np.cos(t)])
    aa, bb = np.meshgrid(np.linspace(-1, 1, 21), np.linspace(-0.35, 0.35, 8))
    xy = aa.ravel()[:, None] * v + bb.ravel()[:, None] * u
    add(np.c_[xy, np.full(len(xy), 0.78)], (0.05, 0.05, 0.01))
    for sa in (-0.95, 0.95):
        for sb in (-0.3, 0.3):
            p = sa * v + sb * u
            add(np.c_[np.full(7, p[0]), np.full(7, p[1]), np.arange(0.1, 0.75, 0.1)], (0.02, 0.02, 0.05))
    add([0.5 * v[0], 0.5 * v[1], 0.95], (0.05, 0.05, 0.05))
    wx, wz = np.meshgrid(np.arange(-3, 3.01, 0.1), np.arange(0.05, 3.0, 0.1))
    add(np.c_[wx.ravel(), np.full(wx.size, 2.9), wz.ravel()], (0.05, 0.01, 0.05))
    add(np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, 5.0)], (0.1, 0.1, 0.01))
    M, S = np.vstack(means), np.vstack(sig)
    covs = np.zeros((len(M), 3, 3))
    covs[:, [0, 1, 2], [0, 1, 2]] = S ** 2
    scene = GaussianScene3D(M, covs, np.full(len(M), 0.9), np.arange(len(M)), "synthetic")
    g0 = {"floor": {"z_floor": 0.0}, "ceiling_height_m": 5.0, "floor_rule": {"synthetic": True}}
    return scene, g0, {"S": ((0.0, 0.0), 20.0, 2.0, 0.7)}


def run_probe(scene, cand, z_f):
    """showcase_run.py's timing probe, verbatim logic, for this candidate (overhang s_interval = table crossing)."""
    from gmc.config import load_config
    from gmc.height.run import compile_and_query, with_overrides
    robot = robot_table(cand["z_c"])["uav"]
    cfg = load_config("configs/height_showcase.yaml")
    st, gl = np.array(cand["start"][:2]), np.array(cand["goal"][:2])
    s_int = cand["rule"]["crossing"]
    mid_s = np.mean(s_int) if s_int else 0.5 * np.linalg.norm(gl - st)
    c = st + (gl - st) / np.linalg.norm(gl - st) * mid_s
    pw = [c[0] - 1.0, c[1] - 1.0, c[0] + 1.0, c[1] + 1.0]
    sp, _ = project_scene(scene, robot, pw, z_floor=z_f)
    out = {"probe_window": [float(x) for x in pw], "n_supports_probe": len(sp.supports)}
    t0 = time.time()
    pcfg = with_overrides(cfg, initial_intervals=1, max_depth=0)
    res, _ = compile_and_query(sp, robot, pcfg, (pw[0] + 0.3, c[1], 0.0), (pw[2] - 0.3, c[1], 0.0))
    dt = time.time() - t0
    ratio = cand["n_supports"] / max(1, len(sp.supports))
    out.update(probe_compile_seconds=dt, probe_status=res["status"], ratio=ratio,
               projected_hours=dt * ratio * cfg.orientation.initial_intervals / 3600.0,
               initial_intervals=cfg.orientation.initial_intervals)
    return out


def empty_map_smoke():
    """Does the unchanged GMC compile+query accept a map with zero supports? (probe windows over a table may be empty)"""
    from shapely.geometry import box
    from gmc.config import load_config
    from gmc.height.run import compile_and_query, with_overrides
    from gmc.types import SceneModel2D
    robot = robot_table(1.20)["uav"]
    cfg = with_overrides(load_config("configs/height_showcase.yaml"), initial_intervals=1, max_depth=0)
    t0 = time.time()
    res, _ = compile_and_query(SceneModel2D((), box(0.0, 0.0, 2.0, 2.0), "empty"), robot, cfg,
                               (0.5, 1.0, 0.0), (1.5, 1.0, 0.0))
    return {"status": res["status"], "verify": res["verify"], "compile_seconds": res["compile_seconds"],
            "seconds": time.time() - t0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--synthetic", action="store_true")
    ap.add_argument("--out", default="results/height/percase/uav")
    ap.add_argument("--tables", default="A,C,B,G")
    ap.add_argument("--no-probe", action="store_true")
    a = ap.parse_args()
    T0 = time.time()
    out = Path(a.out)
    figs = out / "figs"
    figs.mkdir(parents=True, exist_ok=True)
    jpath = out / "search.json"
    if a.synthetic:
        scene, g0, tables = synthetic_scene()
    else:
        scene, g0 = load_processed()
        assert "floor_rule" in g0, "floor rule not applied"
        tables = {k: TABLES[k] for k in a.tables.split(",")}
    z_f = float(g0["floor"]["z_floor"])
    ceil_h = float(g0["ceiling_height_m"])
    log("scene", len(scene), "z_f", z_f, "ceiling", ceil_h, "load s", round(time.time() - T0, 1))
    S = {"meta": {"script": "experiments/percase_search_uav.py", "synthetic": a.synthetic,
                  "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                  "n_scene": len(scene), "z_floor": z_f, "ceiling_height_m": ceil_h,
                  "floor_rule": g0.get("floor_rule"), "robot": {"r": R_UAV, "band_half": HALF_BAND},
                  "zcs": ZCS, "tau": TAU, "rho": RHO, "raster": RASTER, "clear_min": CLEAR_MIN,
                  "glass_zone_excluded": GLASS_ZONE, "claims_boundary": CLAIMS,
                  "notes": __doc__},
         "tables": {}, "lines": [], "candidates": [], "errors": [], "timing": {}}
    LO, HI = scene.aabb(RHO)
    opq = scene.opacity > TAU
    state = {}
    for tag, spec in tables.items():
        if time.time() - T0 > WALL_BUDGET_S:
            S["errors"].append({"where": f"table {tag}", "error": "skipped: wall budget"})
            continue
        t_tab = time.time()
        try:
            fr = frame(spec)
            c = fr[0]
            m = (opq & (np.abs(scene.means[:, 0] - c[0]) <= SEC_REGION_HALF)
                 & (np.abs(scene.means[:, 1] - c[1]) <= SEC_REGION_HALF))
            sec = {"xy": scene.means[m, :2].copy(), "zb": LO[m, 2] - z_f, "zt": HI[m, 2] - z_f}
            pc = Prechecker(scene, LO, HI, z_f, c)
            state[tag] = (fr, sec, pc)
            lines = gen_lines(tag, fr)
            # one-off equality check with showcase_scene._section
            sec_scene = scene.subset(m)
            s_ref, zb_ref, zt_ref, _ = _section(sec_scene, z_f, lines[0]["start"], lines[0]["goal"], half=0.2)
            s_my, zb_my, zt_my, _ = section(sec, lines[0]["start"], lines[0]["goal"], 0.2)
            sec_equal = bool(len(s_ref) == len(s_my) and np.allclose(s_ref, s_my) and np.allclose(zb_ref, zb_my)
                             and np.allclose(zt_ref, zt_my))
            del sec_scene
            S["tables"][tag] = {"centre": c.tolist(), "long_axis": fr[1].tolist(), "short_axis": fr[2].tolist(),
                                "length": fr[3], "width": fr[4], "n_section_splats": int(m.sum()),
                                "n_projection_presubset": len(pc.sub), "section_equals_showcase_section": sec_equal}
            log("table", tag, S["tables"][tag])
            tab_cands = []
            for ln in lines:
                try:
                    rows, L = zc_rows(sec, ln["start"], ln["goal"], fr, ceil_h)
                    ln["length"] = L
                    ln["rows"] = rows
                    strict = [r for r in rows if r["tier"] == 0]
                    anyover = [r for r in rows if r["tier"] is not None]
                    ln["valid_zc_strict"] = [r["z_c"] for r in strict]
                    ln["valid_zc"] = [r["z_c"] for r in anyover]
                    todo = []
                    if ln["family"] != "parallel":
                        if strict:
                            todo.append((strict[0], "over"))
                        if anyover and (not strict or anyover[0]["z_c"] < strict[0]["z_c"]):
                            todo.append((anyover[0], "over"))
                    if not any(r["z_c"] == 1.20 for r, _ in todo):
                        todo.append((rows[0], "fallback"))
                    for row, beh in todo:
                        for wtag, win in gen_windows(ln["start"], ln["goal"]):
                            try:
                                cd = evaluate(pc, ln, row, wtag, win, beh)
                                tab_cands.append(cd)
                            except Exception as e:
                                S["errors"].append({"where": f"{ln['id']} {wtag} {win} z_c={row['z_c']}",
                                                    "error": repr(e), "tb": traceback.format_exc()[-1500:]})
                except Exception as e:
                    S["errors"].append({"where": ln["id"], "error": repr(e), "tb": traceback.format_exc()[-1500:]})
                S["lines"].append(ln)
            S["candidates"].extend(tab_cands)
            npass = [cd for cd in tab_cands if cd["precheck_pass"]]
            over_pass = [cd for cd in npass if cd["behaviour"] == "over"]
            S["tables"][tag].update(
                n_lines=len(lines), n_lines_over_valid=sum(bool(l.get("valid_zc")) for l in lines),
                n_lines_over_strict=sum(bool(l.get("valid_zc_strict")) for l in lines),
                n_candidates=len(tab_cands), n_precheck_pass=len(npass), n_over_precheck_pass=len(over_pass),
                n_windows_projected=len(pc.cache), seconds=time.time() - t_tab,
                best=(min(npass, key=rank_key) if npass else None))
            fig_overview(tag, fr, sec, lines, tab_cands, figs / f"overview_table{tag}.png")
            pc.cache.clear()
            log("table", tag, "done", {k: v for k, v in S["tables"][tag].items() if k != "best"})
            if npass:
                b = S["tables"][tag]["best"]
                log("  best", b["line_id"], b["behaviour"], "tier", b["tier"], "z_c", b["z_c"], "win", b["window"],
                    "supports", b["n_supports"], "clear", round(b["start_clear"], 2), round(b["goal_clear"], 2),
                    "line_min", round(b["line_min_dist"], 2))
        except Exception as e:
            S["errors"].append({"where": f"table {tag}", "error": repr(e), "tb": traceback.format_exc()[-3000:]})
            log("ERROR table", tag, repr(e))
        S["timing"][f"table_{tag}"] = time.time() - t_tab
        dump(jpath, S)

    # ---- ranking ----
    passing = sorted((cd for cd in S["candidates"] if cd["precheck_pass"]), key=rank_key)
    for i, cd in enumerate(S["candidates"]):
        cd["cand_index"] = i
    S["ranking"] = [cd["cand_index"] for cd in passing[:40]]
    S["best_over"] = next((cd for cd in passing if cd["behaviour"] == "over"), None)
    S["best_fallback"] = next((cd for cd in passing if cd["behaviour"] == "fallback"), None)
    top, seen = [], set()
    for cd in passing:
        if cd["line_id"] not in seen:
            seen.add(cd["line_id"])
            top.append(cd)
        if len(top) == 5:
            break
    S["top_distinct_lines"] = top
    log("precheck pass", len(passing), "of", len(S["candidates"]))
    for i, cd in enumerate(top):
        log(f"  top{i + 1}", cd["line_id"], cd["behaviour"], "tier", cd["tier"], "z_c", cd["z_c"], cd["window"],
            "supports", cd["n_supports"], "clear", round(cd["start_clear"], 2), round(cd["goal_clear"], 2),
            "line_min", round(cd["line_min_dist"], 2), "top", cd["rule"]["top"])
    dump(jpath, S)

    # ---- figures for top 3 (+ best fallback) ----
    figlist = [(f"top{i + 1}", cd) for i, cd in enumerate(top[:3])]
    if S["best_fallback"] is not None and all(cd is not S["best_fallback"] for _, cd in figlist):
        figlist.append(("fallback1", S["best_fallback"]))
    S["figures"] = {}
    for lab, cd in figlist:
        try:
            fr, sec, pc = state[cd["table"]]
            p = figs / f"precheck_{lab}_{cd['line_id']}_zc{cd['z_c']:.2f}.png"
            fig_candidate(cd, pc, sec, fr, ceil_h, p, lab)
            S["figures"][lab] = str(p)
        except Exception as e:
            S["errors"].append({"where": f"figure {lab}", "error": repr(e), "tb": traceback.format_exc()[-1500:]})
    dump(jpath, S)

    # ---- alternates: other valid z_c on the same window for the top distinct lines ----
    S["alternates"] = []
    lines_by_id = {ln["id"]: ln for ln in S["lines"]}
    for cd in top:
        if cd["behaviour"] != "over":
            continue
        ln = lines_by_id[cd["line_id"]]
        fr, sec, pc = state[cd["table"]]
        for row in [r for r in ln["rows"] if r["tier"] is not None and r["z_c"] != cd["z_c"]][:3]:
            try:
                alt = evaluate(pc, ln, row, cd["window_tag"], cd["window"], "over")
                alt["alternate_of"] = cd["cand_index"]
                S["alternates"].append(alt)
            except Exception as e:
                S["errors"].append({"where": f"alternate {ln['id']} z_c={row['z_c']}", "error": repr(e)})
        pc.cache.clear()
    dump(jpath, S)

    # ---- full-scene projection check (supports identical to the pre-subset) ----
    S["full_scene_check"] = []
    checks = top[:3] + ([S["best_fallback"]] if S["best_fallback"] is not None else [])
    for cd in checks:
        try:
            t = time.time()
            fr, sec, pc = state[cd["table"]]
            rob = robot_table(cd["z_c"])["uav"]
            s_full, st_full = project_scene(scene, rob, cd["window"], z_floor=z_f)
            s_sub, _ = pc.project(cd["window"], cd["z_c"])
            ids_full = sorted(s.primitive_id for s in s_full.supports)
            ids_sub = sorted(s.primitive_id for s in s_sub.supports)
            S["full_scene_check"].append({"cand_index": cd["cand_index"], "line_id": cd["line_id"],
                                          "window": cd["window"], "z_c": cd["z_c"],
                                          "kept_full": len(ids_full), "kept_subset": len(ids_sub),
                                          "ids_identical": ids_full == ids_sub, "stats_full": st_full,
                                          "seconds": time.time() - t})
            log("full-scene check", cd["line_id"], len(ids_full), len(ids_sub), ids_full == ids_sub)
        except Exception as e:
            S["errors"].append({"where": f"full check {cd['line_id']}", "error": repr(e),
                                "tb": traceback.format_exc()[-1500:]})
    dump(jpath, S)

    # ---- GMC smoke test on an empty map, and the timing probe for the top candidate ----
    try:
        S["empty_map_smoke"] = empty_map_smoke()
    except Exception as e:
        S["empty_map_smoke"] = {"error": repr(e), "tb": traceback.format_exc()[-2000:]}
    log("empty map smoke", json.dumps(S["empty_map_smoke"], default=_j)[:600])
    dump(jpath, S)
    if top and not a.no_probe:
        try:
            S["probe_top1"] = run_probe(scene, top[0], z_f)
        except Exception as e:
            S["probe_top1"] = {"error": repr(e), "tb": traceback.format_exc()[-2000:]}
        log("probe top1", json.dumps(S["probe_top1"], default=_j)[:600])
    S["timing"]["total"] = time.time() - T0
    S["meta"]["finished_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    dump(jpath, S)
    log("done", round(time.time() - T0, 1), "s; errors", len(S["errors"]))


if __name__ == "__main__":
    main()
