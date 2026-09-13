# gmc/experiments/showcase_scene.py
"""G0 for the showcase gallery GS (spec §5.3). Run one --step at a time."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage

from gmc.height.band_shadow import band_overlap_mask
from gmc.height.ply3d import (GaussianScene3D, crop_box, fit_floor, gravity_rotation,
                              load_3dgs_ply, rotate_scene)
from gmc.height.prism import robot_table

SRC = Path("/scratch/sy2366/Project/LccStudio-stage-archive/point_cloud.ply")
SRC_BYTES = 499_121_449
DATA = Path("/scratch/wg2381/splathjb/splatc_atlas/data/gs_scenes/showcase")
RES = Path("results/height/showcase")
FIGS = RES / "figs"
RHO, TAU, CELL = 2.0, 0.3, 0.05
BANDS = [(0.02, 0.10), (0.10, 0.55), (0.55, 1.00), (1.00, 1.75), (1.75, 2.50)]


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 24), b""):
            h.update(chunk)
    return h.hexdigest()


def step_copy():
    assert SRC.stat().st_size == SRC_BYTES, "source size changed; stop and report"
    dst = DATA / "raw" / "point_cloud.ply"
    dst.parent.mkdir(parents=True, exist_ok=True)
    if not dst.exists():
        shutil.copyfile(SRC, dst)
    a, b = sha256(SRC), sha256(dst)
    assert a == b, f"sha256 mismatch {a} != {b}"
    meta = {"raw_file": "raw/point_cloud.ply", "raw_sha256": b, "bytes": SRC_BYTES,
            "source": str(SRC), "lcc_capture": "19017822682139126", "lod": 0,
            "note": "art gallery; reception counter, plinths, benches, glass doors"}
    (DATA / "meta.json").write_text(json.dumps(meta, indent=2))
    print(json.dumps(meta, indent=2))


def _dense_peaks(z, bin_width=0.02, frac=0.2):
    counts, edges = np.histogram(z, bins=np.arange(z.min(), z.max() + bin_width, bin_width))
    thr = frac * counts.max()
    return [0.5 * (edges[i] + edges[i + 1]) for i in range(len(counts))
            if counts[i] >= thr and counts[i] >= counts[max(i - 1, 0)]
            and counts[i] >= counts[min(i + 1, len(counts) - 1)]]


def step_decode():
    scene, st = load_3dgs_ply(DATA / "raw" / "point_cloud.ply", name="showcase")
    g0 = {"decode": st, "n": len(scene)}
    f0 = fit_floor(scene)
    g0["floor_raw"] = f0
    if f0["tilt_deg"] > 0.5:
        scene = rotate_scene(scene, gravity_rotation(f0["normal"]), f0["centroid"])
        g0["gravity_rotation"] = gravity_rotation(f0["normal"]).tolist()
    else:
        g0["gravity_rotation"] = np.eye(3).tolist()
    f1 = fit_floor(scene)
    z_f = f1["z_floor"]
    g0["floor"] = f1
    opq = scene.means[scene.opacity > 0.5, 2]
    above = opq[opq > z_f + 1.5]
    ceiling = max(_dense_peaks(above)) if len(above) else float(np.percentile(opq, 99.5))
    g0["ceiling_z"] = float(ceiling)
    g0["ceiling_height_m"] = float(ceiling - z_f)
    lo = np.array([-np.inf, -np.inf, z_f - 0.2])
    hi = np.array([np.inf, np.inf, ceiling + 0.2])
    scene, dropped = crop_box(scene, lo, hi)
    g0["crop"] = {"z_range": [lo[2], hi[2]], "dropped": dropped, "kept": len(scene)}
    lo3, hi3 = scene.aabb(RHO)
    near_floor = (np.abs(scene.means[:, 2] - z_f) < 0.05) & (scene.opacity > TAU)
    tops = hi3[near_floor, 2] - z_f
    g0["floor_splat_top_above_floor_m"] = {q: float(np.percentile(tops, q)) for q in (50, 90, 99, 99.9)}
    g0["floor_splats_reaching_z_lo_0.02"] = int((tops > 0.02).sum())
    np.savez(DATA / "processed.npz", means=scene.means, covs=scene.covs,
             opacity=scene.opacity, ids=scene.ids)
    RES.mkdir(parents=True, exist_ok=True)
    (RES / "g0.json").write_text(json.dumps(g0, indent=2))
    print(json.dumps({k: v for k, v in g0.items() if k != "gravity_rotation"}, indent=2))


def load_processed():
    d = np.load(DATA / "processed.npz")
    g0 = json.loads((RES / "g0.json").read_text())
    return GaussianScene3D(d["means"], d["covs"], d["opacity"], d["ids"], "showcase"), g0


def _grid(scene, extent):
    nx = int(np.ceil((extent[2] - extent[0]) / CELL))
    ny = int(np.ceil((extent[3] - extent[1]) / CELL))
    return nx, ny


def _occupancy(scene, z_f, band, extent):
    """Cells touched by the xy-AABB of any opaque splat overlapping the band (selection aid only)."""
    sub = scene.subset(scene.opacity > TAU)
    m = band_overlap_mask(sub.means[:, 2], sub.covs[:, 2, 2], z_f + band[0], z_f + band[1], RHO)
    lo, hi = sub.subset(m).aabb(RHO)
    nx, ny = _grid(scene, extent)
    occ = np.zeros((nx, ny), dtype=bool)
    i0 = np.clip(((lo[:, 0] - extent[0]) / CELL).astype(int), 0, nx - 1)
    i1 = np.clip(((hi[:, 0] - extent[0]) / CELL).astype(int), 0, nx - 1)
    j0 = np.clip(((lo[:, 1] - extent[1]) / CELL).astype(int), 0, ny - 1)
    j1 = np.clip(((hi[:, 1] - extent[1]) / CELL).astype(int), 0, ny - 1)
    small = (i1 - i0 <= 4) & (j1 - j0 <= 4)
    for a, b, c, d in zip(i0[~small], i1[~small], j0[~small], j1[~small]):
        occ[a:b + 1, c:d + 1] = True
    for di in range(5):
        for dj in range(5):
            sel = small & (i0 + di <= i1) & (j0 + dj <= j1)
            occ[i0[sel] + di, j0[sel] + dj] = True
    return occ


def step_maps():
    scene, g0 = load_processed()
    z_f = g0["floor"]["z_floor"]
    opq = scene.means[scene.opacity > 0.5]
    ext = [*np.percentile(opq[:, 0:2], 0.5, axis=0) - 0.5, *np.percentile(opq[:, 0:2], 99.5, axis=0) + 0.5]
    ext = [float(ext[0]), float(ext[1]), float(ext[2]), float(ext[3])]
    FIGS.mkdir(parents=True, exist_ok=True)
    occ = {}
    for band in BANDS:
        occ[band] = _occupancy(scene, z_f, band, ext)
        plt.figure(figsize=(12, 10))
        plt.imshow(occ[band].T, origin="lower", cmap="Greys",
                   extent=[ext[0], ext[2], ext[1], ext[3]])
        plt.title(f"occupied cells, band {band} m above floor (selection aid)")
        plt.grid(alpha=0.3); plt.savefig(FIGS / f"band_{band[0]:.2f}_{band[1]:.2f}.png", dpi=120); plt.close()
    overhang = ~_occupancy(scene, z_f, (0.02, 0.15), ext) & occ[(0.55, 1.00)]
    plt.figure(figsize=(12, 10))
    plt.imshow(occ[(0.02, 0.10)].T, origin="lower", cmap="Greys", alpha=0.5,
               extent=[ext[0], ext[2], ext[1], ext[3]])
    plt.imshow(np.ma.masked_where(~overhang.T, overhang.T), origin="lower", cmap="autumn",
               extent=[ext[0], ext[2], ext[1], ext[3]])
    plt.title("overhang candidates: free in [0.02,0.15], occupied in [0.55,1.00]")
    plt.grid(alpha=0.3); plt.savefig(FIGS / "overhang_candidates.png", dpi=120); plt.close()
    np.savez(DATA / "maps.npz", extent=np.array(ext), overhang=overhang,
             **{f"occ_{a:.2f}_{b:.2f}": v for (a, b), v in occ.items()})
    print("extent", ext, "overhang cells", int(overhang.sum()))


def _section(scene, z_f, p0, p1, half=0.2):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    L = float(np.linalg.norm(d))
    u = d / L
    sub = scene.subset(scene.opacity > TAU)
    rel = sub.means[:, :2] - p0
    s = rel @ u
    off = np.abs(rel @ np.array([-u[1], u[0]]))
    sel = (s >= 0) & (s <= L) & (off <= half)
    lo, hi = sub.subset(sel).aabb(RHO)
    return s[sel], lo[:, 2] - z_f, hi[:, 2] - z_f, L


def step_section(seg, name):
    scene, g0 = load_processed()
    z_f = g0["floor"]["z_floor"]
    s, zb, zt, L = _section(scene, z_f, seg[:2], seg[2:])
    plt.figure(figsize=(14, 5))
    plt.vlines(s, zb, zt, lw=0.3, color="k", alpha=0.3)
    plt.xlabel("distance along section (m)"); plt.ylabel("height above floor (m)")
    plt.ylim(-0.1, g0["ceiling_height_m"] + 0.2); plt.grid(alpha=0.3)
    plt.title(f"section {name} {seg}")
    FIGS.mkdir(parents=True, exist_ok=True)
    plt.savefig(FIGS / f"section_{name}.png", dpi=120); plt.close()
    hist, edges = np.histogram(zt, bins=np.arange(0, g0["ceiling_height_m"] + 0.05, 0.02))
    peaks = [float(edges[i]) for i in np.argsort(hist)[-8:][::-1]]
    print(json.dumps({"name": name, "length": L, "n": int(len(s)), "top_height_modes": peaks}))


def _free_dist(scene, z_f, band, ext):
    occ = _occupancy(scene, z_f, band, ext)
    return ndimage.distance_transform_edt(~occ) * CELL


def step_case(window, start, goal, z_c, glass_clear):
    scene, g0 = load_processed()
    z_f = g0["floor"]["z_floor"]
    ext = list(map(float, window))
    robots = robot_table(z_c)
    crit, notes = {}, {}
    crit["window_at_most_8x8"] = (ext[2] - ext[0] <= 8.0) and (ext[3] - ext[1] <= 8.0)
    s, zb, zt, L = _section(scene, z_f, start[:2], goal[:2], half=0.2)
    free_low = np.ones(int(np.ceil(L / 0.02)) + 1, dtype=bool)
    over = np.zeros_like(free_low)
    uav_hit = np.zeros_like(free_low)
    k = np.clip((s / 0.02).astype(int), 0, len(free_low) - 1)
    lowhit = (zb <= 0.15) & (zt >= 0.02)
    free_low[k[lowhit]] = False
    top_mid = (zb > 0.15) & (zb < z_c - 0.10)
    over[k[top_mid]] = True
    uav_hit[k[(zb <= z_c + 0.10) & (zt >= z_c - 0.10)]] = True
    runs = np.flatnonzero(free_low & over & ~uav_hit)
    crit["straight_line_under_overhang"] = bool(len(runs) >= 5)
    if len(runs):
        seg = (zb > 0.15) & (zb < z_c - 0.10) & (s >= runs.min() * 0.02) & (s <= runs.max() * 0.02)
        top = float(zt[seg].max()) if seg.any() else None
        under = float(zb[seg].min()) if seg.any() else None
        notes["overhang"] = {"top": top, "underside": under,
                             "s_interval": [runs.min() * 0.02, runs.max() * 0.02]}
        crit["zc_above_overhang_top_by_0.25"] = top is not None and z_c >= top + 0.25
        higher = zb[(zb > (top or 0) + 0.5)]
        ceil_side = float(higher.min()) if len(higher) else g0["ceiling_height_m"]
        notes["ceiling_side_obstacle_above_floor"] = ceil_side
        crit["zc_band_below_ceiling_side_by_0.30"] = z_c + 0.10 <= ceil_side - 0.30
    for key in ("sweeper", "cylinder", "uav"):
        r = robots[key]
        dist = _free_dist(scene, z_f, (r.z_lo, r.z_hi), ext)
        for tag, p in (("start", start), ("goal", goal)):
            i = int((p[0] - ext[0]) / CELL); j = int((p[1] - ext[1]) / CELL)
            crit[f"{tag}_clear_0.5m_{key}"] = bool(dist[i, j] >= 0.5)
    dist_c = _free_dist(scene, z_f, (robots["cylinder"].z_lo, robots["cylinder"].z_hi), ext)
    lab, _ = ndimage.label(dist_c > robots["cylinder"].max_radius())
    li = lambda p: lab[int((p[0] - ext[0]) / CELL), int((p[1] - ext[1]) / CELL)]
    crit["cylinder_detour_plausible_raster"] = bool(li(start) != 0 and li(start) == li(goal))
    crit["not_through_glass_adjacent_opening"] = glass_clear == "yes"
    case = {"window": ext, "start": list(start), "goal": list(goal), "z_floor": z_f,
            "z_c": z_c, "overhang": notes.get("overhang"), "criteria": crit,
            "pass": all(crit.values()), "notes": notes,
            "gravity_rotation": g0["gravity_rotation"]}
    (RES / "case.json").write_text(json.dumps(case, indent=2))
    fig, ax = plt.subplots(1, 2, figsize=(16, 6))
    occ = _occupancy(scene, z_f, (0.02, 1.75), ext)
    ax[0].imshow(occ.T, origin="lower", cmap="Greys", extent=[ext[0], ext[2], ext[1], ext[3]])
    ax[0].plot([start[0], goal[0]], [start[1], goal[1]], "r-")
    ax[0].plot(*start[:2], "go"); ax[0].plot(*goal[:2], "bs"); ax[0].set_title("case window, band 0.02-1.75")
    ax[1].vlines(s, zb, zt, lw=0.3, color="k", alpha=0.3)
    for key, colr in (("sweeper", "g"), ("cylinder", "r"), ("uav", "b")):
        r = robots[key]
        ax[1].axhspan(r.z_lo, r.z_hi, color=colr, alpha=0.12, label=key)
    ax[1].legend(); ax[1].set_title("section along start->goal with robot bands")
    plt.savefig(FIGS / "case_overview.png", dpi=120); plt.close()
    print(json.dumps(case, indent=2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", required=True, choices=["copy", "decode", "maps", "section", "case"])
    ap.add_argument("--seg"); ap.add_argument("--name")
    ap.add_argument("--window"); ap.add_argument("--start"); ap.add_argument("--goal")
    ap.add_argument("--zc", type=float); ap.add_argument("--glass-clear", choices=["yes", "no"])
    a = ap.parse_args()
    f = lambda v: [float(x) for x in v.split(",")]
    if a.step == "copy":
        step_copy()
    elif a.step == "decode":
        step_decode()
    elif a.step == "maps":
        step_maps()
    elif a.step == "section":
        step_section(f(a.seg), a.name)
    else:
        step_case(f(a.window), f(a.start), f(a.goal), a.zc, a.glass_clear)


if __name__ == "__main__":
    main()
