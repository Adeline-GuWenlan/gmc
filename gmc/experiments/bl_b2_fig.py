"""bl B2 figure: the shared C-space map and the two map-based baselines on pilot pairs (cylinder and sweeper).

One row per robot, one panel per region. Background = the frozen raster (dark = C-space occupied, i.e. where the body
centre cannot be; light = free), outlines = cust_fields' NF world (its squircle covers, frozen construction), black =
the pair's stored A* route, blue = PNO's judged route (``route_polyline``: grid path + completion segments), red =
cust_fields' judged route when it claimed one, x = endpoints the method could not keep (cust_fields FAIL).
Pairs: up to 2 per region (first pilot pairs, plus the first cust_fields SUCCESS if any).

Run (CPU sbatch, gmc-venv, from gmc/): python experiments/bl_b2_fig.py --out results/baselines/pilot/fig_b2_pilot.png
"""
from __future__ import annotations

import argparse
import json
import pickle
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import bl_custfields as C  # noqa: E402
import bl_harness as H  # noqa: E402
import bl_raster as R  # noqa: E402


def rows(method, region, robot):
    return {r["pair_id"]: r for r in H.iter_rows(Path(f"results/baselines/pilot/{method}/{region}/{robot}"))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    pno_cfg = json.loads(Path("configs/baselines/pno.json").read_text())
    cf_cfg = json.loads(Path("configs/baselines/cust_fields.json").read_text())
    pairs = {p["pair_id"]: p for p in H.load_pairs("f4")}
    fig, axs = plt.subplots(2, 3, figsize=(22, 10), gridspec_kw={"width_ratios": [8.7, 5.7, 9.8]})
    for i, robot in enumerate(("cylinder", "sweeper")):
        for j, reg in enumerate(("WWEST", "GAPW1", "S")):
            ax = axs[i, j]
            res = float(pno_cfg["robots"][robot]["raster_res_m"])
            arr, info = R.load(R.raster_path(reg, robot, res))
            ext = [info["u0"], info["u0"] + info["nx"] * info["res_m"], info["v0"], info["v0"] + info["ny"] * info["res_m"]]
            ax.imshow(arr["occ"], origin="lower", extent=ext, cmap="Greys", vmin=-.6, vmax=1.6, interpolation="nearest")
            # cust_fields world of the frozen config
            cfg = cf_cfg["robots"][robot]
            art = H.artifact_path("cust_fields", reg, robot, {k: v for k, v in cfg.items()})
            n_cov = 0
            if art.exists():
                st = pickle.loads(art.read_bytes())
                for ch in st["chains"]:
                    for c, aa, bb, th in ch:
                        P = C.squircle_boundary(np.asarray(c), aa, bb, th, float(cfg["s"]), 180)
                        ax.plot(np.r_[P[:, 0], P[0, 0]], np.r_[P[:, 1], P[0, 1]], "-", color="tab:orange", lw=.8)
                        n_cov += 1
            pn, cf = rows("pno", reg, robot), rows("cust_fields", reg, robot)
            ids = list(pn)[:2]
            ok = [k for k in cf if cf[k]["status"] == "SUCCESS"]
            if ok and ok[0] not in ids:
                ids = ids[:1] + ok[:1]
            for k in ids:
                p = pairs[k]
                rt = p["astar_route_uv"]
                rt = np.asarray(json.loads(rt) if isinstance(rt, str) else rt)
                ax.plot(rt[:, 0], rt[:, 1], "k-", lw=1.4)
                if pn.get(k, {}).get("route_polyline"):
                    q = np.asarray(pn[k]["route_polyline"])
                    ax.plot(q[:, 0], q[:, 1], "-", color="tab:blue", lw=1.2)
                if cf.get(k, {}).get("route_polyline"):
                    q = np.asarray(cf[k]["route_polyline"])
                    ax.plot(q[:, 0], q[:, 1], "-", color="tab:red", lw=1.2)
                s, g = np.asarray(p["start_uv"]), np.asarray(p["goal_uv"])
                ax.plot(*s, "o", color="green", ms=6)
                ax.plot(*g, "s", color="green", ms=6)
                ax.annotate(f"{k}\nPNO {pn.get(k, {}).get('status')}\nCF {cf.get(k, {}).get('status')}"
                            f" {cf.get(k, {}).get('claimed_reason') or ''}", s, fontsize=7, color="navy")
            ax.set_title(f"{reg} {robot}: raster {res * 1000:g} mm, {n_cov} cust_fields squircles", fontsize=10)
            ax.set_aspect("equal")
            ax.set_xlim(ext[0], ext[1])
            ax.set_ylim(ext[2], ext[3])
    fig.suptitle("B2 pilot: C-space map (dark = occupied for the body centre), cust_fields NF world (orange), "
                 "A* route (black), PNO (blue), cust_fields (red)", fontsize=12)
    fig.tight_layout()
    fig.savefig(a.out, dpi=90)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
