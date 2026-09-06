"""Whole-floor K2 orientation-event sweep (tiled, multiprocess) — G-H2 closure.

Tiles the building slab into overlapping 6x6 u windows (stride 5), sweeps
theta in [0, pi) per tile, and counts S3 free-space topology events — the
scale-out check that the window-level sparsity holds across the whole floor.
Robot: L24 (support 2.4 x 0.6 u), the gate-critical middle of the morphology
ladder.

Pilot-grade LOCAL run (8-way multiprocessing, ~30-45 min); the canonical
adjudication-chain rerun belongs on HPC later. Incremental per-tile jsonl ->
resumable after kill.

Outputs: results/gmc_h2/k2_fullfloor.jsonl (one line per tile),
         k2_fullfloor.json (summary), figs/k2_event_heatmap.png
"""
import json
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

import h2_k2_windows as base

OUT = base.OUT
JSONL = OUT / "k2_fullfloor.jsonl"
HALF = 3.0
STRIDE = 5.0
MIN_SPLATS = 500
ROBOT = "L24"
N_THETA = 720
WORKERS = 8


def tile_centers():
    xs = np.arange(5.0, 40.01, STRIDE)
    ys = np.arange(-24.0, 0.01, STRIDE)
    return [(float(x), float(y)) for x in xs for y in ys]


def sweep_tile(center):
    from shapely.geometry import box as shapely_box
    mu, S = base.load_window(center, HALF)
    if len(mu) < MIN_SPLATS:
        return {"center": center, "skipped": True, "n_splats": int(len(mu))}
    rob = base.ROBOTS[ROBOT]
    ws = shapely_box(center[0] - HALF, center[1] - HALF,
                     center[0] + HALF, center[1] + HALF)
    thetas = np.linspace(0, np.pi, N_THETA, endpoint=False)
    s3_prev, events, churn = None, [], None
    free_area = []
    t0 = time.time()
    for k, th in enumerate(thetas):
        polys = base.forbidden_polys(mu, S, th, rob)
        s3, _, free = base.free_topology(polys, ws, [])
        free_area.append(free.area)
        if s3_prev is not None and s3 != s3_prev:
            events.append(float(th))
        s3_prev = s3
        if k == 0:
            from shapely.strtree import STRtree
            tree = STRtree(polys)
            a, b = tree.query(polys, predicate="intersects")
            churn = int((a < b).sum())
    return {"center": center, "skipped": False, "n_splats": int(len(mu)),
            "n_s3_events": len(events), "s3_events": events,
            "churn_pairs_at_theta0": churn,
            "free_area_frac": float(np.mean(free_area) / ws.area),
            "wall_s": time.time() - t0}


def main():
    done = {}
    if JSONL.exists():
        for line in JSONL.read_text().splitlines():
            r = json.loads(line)
            done[tuple(r["center"])] = r
    centers = tile_centers()
    todo = [c for c in centers if c not in done]
    print(f"tiles: {len(centers)} total, {len(done)} cached, {len(todo)} to run",
          flush=True)
    results = list(done.values())
    with ProcessPoolExecutor(max_workers=WORKERS) as ex, open(JSONL, "a") as f:
        futs = {ex.submit(sweep_tile, c): c for c in todo}
        for fut in as_completed(futs):
            r = fut.result()
            results.append(r)
            f.write(json.dumps(r) + "\n")
            f.flush()
            if not r["skipped"]:
                print(f"tile {r['center']}: splats={r['n_splats']} "
                      f"S3={r['n_s3_events']} free={r['free_area_frac']:.2f} "
                      f"{r['wall_s']:.0f}s", flush=True)
    active = [r for r in results if not r["skipped"]]
    ev = [r["n_s3_events"] for r in active]
    summary = {
        "config": {"half": HALF, "stride": STRIDE, "robot": ROBOT,
                   "n_theta": N_THETA, "min_splats": MIN_SPLATS,
                   "w_obst": base.W_OBST, "rho": base.RHO,
                   "note": "pilot-grade local multiprocess run; canonical rerun on HPC"},
        "n_tiles_active": len(active), "n_tiles_skipped": len(results) - len(active),
        "s3_events_per_tile": {"min": int(min(ev)), "median": float(np.median(ev)),
                               "p90": float(np.percentile(ev, 90)), "max": int(max(ev))},
        "s3_step_fraction_median": float(np.median(ev) / (N_THETA - 1)),
        "total_events": int(sum(ev)),
    }
    (OUT / "k2_fullfloor.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=1), flush=True)

    # event-density heatmap over the floor
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    d = np.load(base.SLAB)
    mu, w = d["mean2"], d["weight"]
    m = w > base.W_OBST
    fig, ax = plt.subplots(figsize=(13, 10))
    ax.hist2d(mu[m, 0], mu[m, 1], bins=500,
              range=[[2, 43], [-28, 4]], norm=matplotlib.colors.LogNorm(),
              cmap="gray_r")
    xs = [r["center"][0] for r in active]
    ys = [r["center"][1] for r in active]
    sc = ax.scatter(xs, ys, c=[r["n_s3_events"] for r in active], s=900, alpha=0.65,
                    cmap="YlOrRd", edgecolors="k", linewidths=0.5, marker="s")
    for r in active:
        ax.annotate(str(r["n_s3_events"]), r["center"], ha="center", va="center",
                    fontsize=7)
    fig.colorbar(sc, ax=ax, label="S3 events per pi sweep (tile)")
    ax.set_aspect("equal")
    ax.set_title(f"K2 whole floor: orientation-event density per 6x6u tile "
                 f"(robot {ROBOT}, {N_THETA} thetas)")
    fig.tight_layout()
    fig.savefig(base.FIGS / "k2_event_heatmap.png", dpi=110)
    print("saved heatmap", flush=True)


if __name__ == "__main__":
    main()
