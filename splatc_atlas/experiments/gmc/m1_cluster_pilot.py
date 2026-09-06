"""M1 pilot: conservative splat clustering for the contact complex.

Motivation (worklog gmc_H2.md, v1 finding 3): raw K2 splats give ~1e7
intersecting primitive pairs per slice — the contact complex must be built on
merged primitives. This pilot merges stacked wall splats into wall-segment
ellipses and validates on the door_A window:

  1. CONSERVATIVE: merged forbidden union must cover the raw forbidden union
     at every theta (empirical leakage check at 12 thetas; the formal
     margin lemma is a G1 proof obligation).
  2. CHEAP: primitive count, churn pairs, per-slice wall time before/after.
  3. FAITHFUL: door_A/L24 gate profile (720 thetas) vs the v3 unclustered
     reference — agreement fraction + switch-angle shifts. Conservative
     merging may only close, never open, so disagreements must be open->closed.

Method (pilot): bucket by (0.5u grid cell, 22.5deg major-axis bin mod pi);
per bucket, sample member rho-support boundary points, fit merged ellipse by
moment fit inflated to contain all samples (x1.02 margin).

Outputs: results/gmc_h2/m1_cluster_pilot.json + figs/m1_cluster_pilot.png
"""
import json
import time
from pathlib import Path

import numpy as np
import shapely
from shapely.geometry import box as shapely_box, LineString

import h2_k2_windows as base

OUT = base.OUT
CELL = 0.5
ANGLE_BINS = 8          # bins over [0, pi)
SAMPLES = 16            # boundary samples per member ellipse
INFLATE = 1.02
CHECK_THETAS = 12
WINDOW, ROBOT = "door_A", "L24"


def major_angle(S):
    evals, evecs = np.linalg.eigh(S)
    v = evecs[:, :, 1]
    return np.arctan2(v[:, 1], v[:, 0]) % np.pi


def ellipse_boundary(mu, S, n=SAMPLES):
    """(m, n, 2) boundary points of rho-support ellipses."""
    a, b, d = S[:, 0, 0], S[:, 0, 1], S[:, 1, 1]
    sa = np.sqrt(a)
    l21, l22 = b / sa, np.sqrt(np.maximum(d - (b / sa) ** 2, 1e-12))
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    cx, sx = np.cos(ang), np.sin(ang)
    px = mu[:, 0, None] + base.RHO * sa[:, None] * cx[None]
    py = mu[:, 1, None] + base.RHO * (l21[:, None] * cx[None] + l22[:, None] * sx[None])
    return np.stack([px, py], axis=-1)


def merge_bucket(mu, S):
    pts = ellipse_boundary(mu, S).reshape(-1, 2)
    c = pts.mean(axis=0)
    rel = pts - c
    C = (rel.T @ rel) / len(rel)
    inv = np.linalg.inv(C)
    mah2 = np.einsum("ni,ij,nj->n", rel, inv, rel).max()
    Sm = C * (mah2 / base.RHO ** 2) * INFLATE ** 2
    return c, Sm


def cluster(mu, S):
    ang = major_angle(S)
    cell = np.floor(mu / CELL).astype(np.int64)
    abin = np.floor(ang / (np.pi / ANGLE_BINS)).astype(np.int64) % ANGLE_BINS
    key = cell[:, 0] * 1_000_003 + cell[:, 1] * 1009 + abin
    order = np.argsort(key)
    ks = key[order]
    bounds = np.flatnonzero(np.r_[True, ks[1:] != ks[:-1], True])
    out_mu, out_S = [], []
    for i, j in zip(bounds[:-1], bounds[1:]):
        idx = order[i:j]
        if len(idx) == 1:
            out_mu.append(mu[idx[0]]); out_S.append(S[idx[0]])
        else:
            c, Sm = merge_bucket(mu[idx], S[idx])
            out_mu.append(c); out_S.append(Sm)
    return np.array(out_mu), np.array(out_S)


def slice_stats(mu, S, rob, ws, th):
    t0 = time.time()
    polys = base.forbidden_polys(mu, S, th, rob)
    s3, _, free = base.free_topology(polys, ws, [])
    dt = time.time() - t0
    return polys, free, s3, dt


def main():
    win = base.WINDOWS[WINDOW]
    rob = base.ROBOTS[ROBOT]
    mu, S = base.load_window(win["center"], win["half"])
    ws = shapely_box(win["center"][0] - win["half"], win["center"][1] - win["half"],
                     win["center"][0] + win["half"], win["center"][1] + win["half"])
    t0 = time.time()
    cmu, cS = cluster(mu, S)
    t_cluster = time.time() - t0
    print(f"clustered {len(mu)} -> {len(cmu)} primitives in {t_cluster:.2f}s", flush=True)

    # 1. conservativeness + cost at CHECK_THETAS angles
    leak, overcov, t_raw, t_merged = [], [], [], []
    churn_raw = churn_merged = None
    for k, th in enumerate(np.linspace(0, np.pi, CHECK_THETAS, endpoint=False)):
        rpolys, rfree, _, dtr = slice_stats(mu, S, rob, ws, th)
        mpolys, mfree, _, dtm = slice_stats(cmu, cS, rob, ws, th)
        t_raw.append(dtr); t_merged.append(dtm)
        runion = shapely.union_all(rpolys).intersection(ws)
        munion = shapely.union_all(mpolys).intersection(ws)
        leak.append(runion.difference(munion).area)          # must be ~0
        overcov.append(munion.difference(runion).area)
        if k == 0:
            from shapely.strtree import STRtree
            for polys, slot in ((rpolys, "raw"), (mpolys, "merged")):
                tr = STRtree(polys)
                a, b = tr.query(polys, predicate="intersects")
                n = int((a < b).sum())
                if slot == "raw":
                    churn_raw = n
                else:
                    churn_merged = n

    # 2. gate profile vs v3 reference
    ref = next(r for r in json.loads((OUT / "k2_windows.json").read_text())["runs"]
               if r["window"] == WINDOW and r["robot"] == ROBOT)
    probes = [LineString(p) for p in ref["probes"]]
    thetas = np.linspace(0, np.pi, ref["n_theta"], endpoint=False)
    flags_m = []
    t0 = time.time()
    for th in thetas:
        polys = base.forbidden_polys(cmu, cS, th, rob)
        _, gate_open, _ = base.free_topology(polys, ws, probes)
        flags_m.append(bool(gate_open))
    t_gate = time.time() - t0
    flags_r = np.array(ref["open_flags"], bool)
    flags_m = np.array(flags_m)
    agree = float((flags_r == flags_m).mean())
    false_open = int((~flags_r & flags_m).sum())   # conservative => must be 0
    closed_extra = int((flags_r & ~flags_m).sum())

    report = {
        "window": WINDOW, "robot": ROBOT,
        "params": {"cell": CELL, "angle_bins": ANGLE_BINS, "samples": SAMPLES,
                   "inflate": INFLATE},
        "n_raw": int(len(mu)), "n_merged": int(len(cmu)),
        "reduction": float(len(mu) / len(cmu)),
        "cluster_s": t_cluster,
        "conservative": {"max_leak_area": float(max(leak)),
                         "mean_overcoverage_area": float(np.mean(overcov)),
                         "window_area": float(ws.area)},
        "cost": {"slice_s_raw": float(np.mean(t_raw)),
                 "slice_s_merged": float(np.mean(t_merged)),
                 "speedup": float(np.mean(t_raw) / np.mean(t_merged)),
                 "churn_raw": churn_raw, "churn_merged": churn_merged,
                 "gate_sweep_720_s": t_gate},
        "gate_fidelity": {"agreement": agree, "false_open": false_open,
                          "extra_closed": closed_extra,
                          "open_frac_ref": float(flags_r.mean()),
                          "open_frac_merged": float(flags_m.mean())},
    }
    (OUT / "m1_cluster_pilot.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=1), flush=True)

    # figure: primitives before/after + gate overlay
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse
    fig, axes = plt.subplots(1, 3, figsize=(19, 6))
    for ax, (m_, S_, ttl) in zip(axes[:2], [
            (mu, S, f"raw splats n={len(mu)} (subsampled render)"),
            (cmu, cS, f"merged primitives n={len(cmu)}")]):
        step = max(1, len(m_) // 4000)
        evals, evecs = np.linalg.eigh(S_[::step])
        for c, ev, evec in zip(m_[::step], evals, evecs):
            a = np.rad2deg(np.arctan2(evec[1, 1], evec[0, 1]))
            ax.add_patch(Ellipse(c, 2 * base.RHO * np.sqrt(ev[1]),
                                 2 * base.RHO * np.sqrt(ev[0]), angle=a,
                                 alpha=0.25, facecolor="tab:blue", edgecolor="none"))
        ax.set_xlim(ws.bounds[0], ws.bounds[2]); ax.set_ylim(ws.bounds[1], ws.bounds[3])
        ax.set_aspect("equal"); ax.set_title(ttl, fontsize=10)
    ax = axes[2]
    deg = np.rad2deg(thetas)
    ax.fill_between(deg, 1.05, 1.95, where=flags_r, color="tab:green", alpha=0.6)
    ax.fill_between(deg, 0.05, 0.95, where=flags_m, color="tab:blue", alpha=0.6)
    dis = flags_r != flags_m
    for d0 in deg[dis]:
        ax.axvline(d0, color="tab:red", lw=0.5, alpha=0.5)
    ax.set_yticks([0.5, 1.5], ["merged", "raw ref"])
    ax.set_xlabel("theta (deg)"); ax.set_xlim(0, 180)
    ax.set_title(f"gate profile: agreement {agree:.3f}, false-open {false_open}",
                 fontsize=10)
    fig.suptitle(f"M1 clustering pilot — {WINDOW}/{ROBOT}: "
                 f"{len(mu)}->{len(cmu)} primitives, churn {churn_raw:.0e}->{churn_merged:.0e}")
    fig.tight_layout()
    fig.savefig(base.FIGS / "m1_cluster_pilot.png", dpi=110)
    print("saved figure", flush=True)


if __name__ == "__main__":
    main()
