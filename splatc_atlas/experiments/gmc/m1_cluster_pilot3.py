"""M1 pilot v3: tight sound clustering — tangent-polygon merged primitives.

v2 was sound (leak 0, false-open 0) but fat: fitting one ELLIPSE over a bucket
adds shape slack that shrank the door gap (gate open 0.64 -> 0.08). v3 drops
the ellipse: the merged primitive is the circumscribed 24-direction tangent
polygon of the UNION of member support ellipses — per direction u_k the support
is max_i [c_i.u_k + rho*sqrt(u_k^T Sigma_i u_k)] (exact, no fit slack), vertices
from intersecting consecutive tangent lines. Conservative by construction, and
the exact convex Minkowski sum with the robot polygon (v2 machinery) keeps the
soundness chain intact.

Outputs: results/gmc_h2/m1_cluster_pilot3.json + figs/m1_cluster_pilot3.png
"""
import json
import time

import numpy as np
import shapely
from shapely.geometry import box as shapely_box, LineString

import h2_k2_windows as base
import m1_cluster_pilot as v1
import m1_cluster_pilot2 as v2

OUT = base.OUT
NDIR = 24
MARGIN = 1.001
CHECK_THETAS = 12
WINDOW, ROBOT = "door_A", "L24"

_ALPHA = np.linspace(0, 2 * np.pi, NDIR, endpoint=False)
_U = np.stack([np.cos(_ALPHA), np.sin(_ALPHA)], axis=1)          # (NDIR, 2)


def bucket_indices(mu, S):
    ang = v1.major_angle(S)
    cell = np.floor(mu / v1.CELL).astype(np.int64)
    abin = np.floor(ang / (np.pi / v1.ANGLE_BINS)).astype(np.int64) % v1.ANGLE_BINS
    key = cell[:, 0] * 1_000_003 + cell[:, 1] * 1009 + abin
    order = np.argsort(key)
    ks = key[order]
    bounds = np.flatnonzero(np.r_[True, ks[1:] != ks[:-1], True])
    return [order[i:j] for i, j in zip(bounds[:-1], bounds[1:])]


def tangent_polys(mu, S):
    """(n_buckets, NDIR, 2) circumscribed tangent polygons of bucket unions."""
    buckets = bucket_indices(mu, S)
    # support of each splat in each direction: c.u + rho*sqrt(u^T S u)
    quad = np.einsum("ki,nij,kj->nk", _U, S, _U)                  # (n, NDIR)
    h_all = mu @ _U.T + base.RHO * np.sqrt(quad) * MARGIN         # (n, NDIR)
    polys = np.empty((len(buckets), NDIR, 2))
    # vertex k = intersection of tangent lines k and k+1
    u1, u2 = _U, np.roll(_U, -1, axis=0)
    det = u1[:, 0] * u2[:, 1] - u1[:, 1] * u2[:, 0]               # (NDIR,)
    for bi, idx in enumerate(buckets):
        h = h_all[idx].max(axis=0)                                # (NDIR,)
        h2 = np.roll(h, -1)
        polys[bi, :, 0] = (h * u2[:, 1] - h2 * u1[:, 1]) / det
        polys[bi, :, 1] = (h2 * u1[:, 0] - h * u2[:, 0]) / det
    return polys


def main():
    win = base.WINDOWS[WINDOW]
    rob = base.ROBOTS[ROBOT]
    mu, S = base.load_window(win["center"], win["half"])
    ws = shapely_box(win["center"][0] - win["half"], win["center"][1] - win["half"],
                     win["center"][0] + win["half"], win["center"][1] + win["half"])
    t0 = time.time()
    scene_polys = tangent_polys(mu, S)
    t_cluster = time.time() - t0
    n_merged = len(scene_polys)
    print(f"primitives: {len(mu)} raw -> {n_merged} tangent polys "
          f"({t_cluster:.2f}s)", flush=True)

    leak, overcov, t_raw, t_merged = [], [], [], []
    churn_raw = churn_merged = None
    for k, th in enumerate(np.linspace(0, np.pi, CHECK_THETAS, endpoint=False)):
        t0 = time.time()
        rpolys = base.forbidden_polys(mu, S, th, rob)
        runion = shapely.union_all(rpolys).intersection(ws)
        t_raw.append(time.time() - t0)
        t0 = time.time()
        mpolys = v2.merged_forbidden_polys(scene_polys, th, rob)
        munion = shapely.union_all(mpolys).intersection(ws)
        t_merged.append(time.time() - t0)
        leak.append(runion.difference(munion).area)
        overcov.append(munion.difference(runion).area)
        if k == 0:
            from shapely.strtree import STRtree
            tr = STRtree(rpolys)
            a, b = tr.query(rpolys, predicate="intersects")
            churn_raw = int((a < b).sum())
            tr = STRtree(list(mpolys))
            a, b = tr.query(list(mpolys), predicate="intersects")
            churn_merged = int((a < b).sum())

    ref = next(r for r in json.loads((OUT / "k2_windows.json").read_text())["runs"]
               if r["window"] == WINDOW and r["robot"] == ROBOT)
    probes = [LineString(p) for p in ref["probes"]]
    thetas = np.linspace(0, np.pi, ref["n_theta"], endpoint=False)
    flags_m = []
    t0 = time.time()
    for th in thetas:
        polys = v2.merged_forbidden_polys(scene_polys, th, rob)
        _, gate_open, _ = base.free_topology(polys, ws, probes)
        flags_m.append(bool(gate_open))
    t_gate = time.time() - t0
    flags_r = np.array(ref["open_flags"], bool)
    flags_m = np.array(flags_m)
    agree = float((flags_r == flags_m).mean())
    false_open = int((~flags_r & flags_m).sum())
    extra_closed = int((flags_r & ~flags_m).sum())

    report = {
        "window": WINDOW, "robot": ROBOT,
        "construction": "bucket-union tangent polygon (24 dirs, support-exact) "
                        "(+) robot ellipse via exact convex Minkowski sum",
        "n_raw": int(len(mu)), "n_merged": n_merged, "cluster_s": t_cluster,
        "conservative": {"max_leak_area": float(max(leak)),
                         "mean_overcoverage_area": float(np.mean(overcov))},
        "cost": {"slice_s_raw": float(np.mean(t_raw)),
                 "slice_s_merged": float(np.mean(t_merged)),
                 "speedup": float(np.mean(t_raw) / np.mean(t_merged)),
                 "churn_raw": churn_raw, "churn_merged": churn_merged,
                 "gate_sweep_720_s": t_gate},
        "gate_fidelity": {"agreement": agree, "false_open": false_open,
                          "extra_closed": extra_closed,
                          "open_frac_ref": float(flags_r.mean()),
                          "open_frac_merged": float(flags_m.mean())},
    }
    (OUT / "m1_cluster_pilot3.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=1), flush=True)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(15, 5.6))
    ax = axes[0]
    th_show = np.deg2rad(61)
    runion = shapely.union_all(base.forbidden_polys(mu, S, th_show, rob)).intersection(ws)
    munion = shapely.union_all(v2.merged_forbidden_polys(scene_polys, th_show, rob)).intersection(ws)
    for g in getattr(munion, "geoms", [munion]):
        if g.geom_type == "Polygon":
            ax.fill(*g.exterior.xy, color="tab:orange", alpha=0.5)
    for g in getattr(runion, "geoms", [runion]):
        if g.geom_type == "Polygon":
            ax.fill(*g.exterior.xy, color="0.3", alpha=0.8)
    ax.set_aspect("equal"); ax.set_xlim(ws.bounds[0], ws.bounds[2]); ax.set_ylim(ws.bounds[1], ws.bounds[3])
    ax.set_title(f"theta=61deg: raw (dark) inside tangent-poly cover (orange)\n"
                 f"leak={max(leak):.1e}, overcoverage={np.mean(overcov):.2f} u^2", fontsize=10)
    ax = axes[1]
    deg = np.rad2deg(thetas)
    ax.fill_between(deg, 1.05, 1.95, where=flags_r, color="tab:green", alpha=0.6)
    ax.fill_between(deg, 0.05, 0.95, where=flags_m, color="tab:blue", alpha=0.6)
    for d0 in deg[flags_r != flags_m]:
        ax.axvline(d0, color="tab:red", lw=0.5, alpha=0.5)
    ax.set_yticks([0.5, 1.5], ["merged v3", "raw ref"])
    ax.set_xlabel("theta (deg)"); ax.set_xlim(0, 180)
    ax.set_title(f"gate: agreement {agree:.3f}, false-open {false_open}, "
                 f"extra-closed {extra_closed}", fontsize=10)
    fig.suptitle(f"M1 v3 (tangent-polygon primitives) — {WINDOW}/{ROBOT}: "
                 f"{len(mu)}->{n_merged}, churn {churn_raw:.0e}->{churn_merged:.0e}")
    fig.tight_layout()
    fig.savefig(base.FIGS / "m1_cluster_pilot3.png", dpi=110)
    print("saved figure", flush=True)


if __name__ == "__main__":
    main()
