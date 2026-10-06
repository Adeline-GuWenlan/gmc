"""F3 Task 1.4: one quick-look top-down figure per failure case (sbatch: loads the archive crop of the region).

Run from ``gmc/`` with ``PYTHONPATH=src:experiments MPLBACKEND=Agg``.

  cases --cases cases.json --out-dir DIR
      cases.json: {"box_uv": [...], "cases": [{"case_id", "robot", "start_uv", "goal_uv", "astar_route_uv",
                    "gmc_status", "gmc_reason", "gmc_route_uv"?, "cut_gaussians"? [[id,u,v,z,top],...], "note"?}]}
      Background (route frame, 2 cm cells): the 2-sigma xy footprint (axis-aligned box of the route-frame ellipse) of
      every Gaussian whose 2-sigma z-extent overlaps the robot's body band [bottom, top] -- dark; Gaussians whose
      2-sigma top is below the chassis bottom + 5 mm (floor splats under the chassis) -- orange dots.  On top: the A*
      witness route (blue) with the body disk drawn at its tightest point, start (o) / goal (s), GMC's route if any
      (green), the GMC certificate's cut Gaussians if any (red x).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from aerial3dg_run import ROBOTS, _box, load_booth


def footprint(ctx, body, box, step=.02, pad=.6):
    g = ctx["scene"].gaussians
    lev = float(ctx["scene"].level)
    R = ctx["frame"].R
    mu = ctx["frame"].to_route(g.means)
    var = np.stack([np.einsum("j,njk,k->n", R[k], g.covs, R[k]) for k in range(3)], 1)
    sd = lev * np.sqrt(np.maximum(var, 0.))
    lo_z, hi_z = body.ground_clearance_m, body.ground_clearance_m + 2 * body.half_height_m
    band = (mu[:, 2] + sd[:, 2] > lo_z) & (mu[:, 2] - sd[:, 2] < hi_z)
    u0, v0, u1, v1 = box[0] - pad, box[1] - pad, box[2] + pad, box[3] + pad
    nu, nv = int((u1 - u0) / step), int((v1 - v0) / step)
    x, s = mu[band], sd[band]
    a0 = np.clip(((x[:, 0] - s[:, 0] - u0) / step).astype(int), 0, nu)
    a1 = np.clip(((x[:, 0] + s[:, 0] - u0) / step).astype(int) + 1, 0, nu)
    b0 = np.clip(((x[:, 1] - s[:, 1] - v0) / step).astype(int), 0, nv)
    b1 = np.clip(((x[:, 1] + s[:, 1] - v0) / step).astype(int) + 1, 0, nv)
    ok = (a1 > a0) & (b1 > b0)
    D = np.zeros((nu + 1, nv + 1))
    np.add.at(D, (a0[ok], b0[ok]), 1)
    np.add.at(D, (a1[ok], b0[ok]), -1)
    np.add.at(D, (a0[ok], b1[ok]), -1)
    np.add.at(D, (a1[ok], b1[ok]), 1)
    cover = D.cumsum(0).cumsum(1)[:nu, :nv]
    low = (mu[:, 2] + sd[:, 2] < lo_z + .005) & (mu[:, 2] + sd[:, 2] > lo_z - .01)
    inwin = (mu[:, 0] > u0) & (mu[:, 0] < u1) & (mu[:, 1] > v0) & (mu[:, 1] < v1)
    return cover, (u0, u1, v0, v1), mu[low & inwin, :2]


def cmd_cases(a):
    doc = json.loads(Path(a.cases).read_text())
    box = doc["box_uv"]
    ctx = load_booth(_box(box))
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    bg = {}
    for c in doc["cases"]:
        body = ROBOTS[c["robot"]]
        if c["robot"] not in bg:
            bg[c["robot"]] = footprint(ctx, body, box)
        cover, ext, low = bg[c["robot"]]
        fig, ax = plt.subplots(figsize=(10, 10 * (ext[3] - ext[2]) / (ext[1] - ext[0]) + .8))
        ax.imshow(np.log1p(cover).T, origin="lower", extent=ext, cmap="Greys", vmax=4, interpolation="nearest")
        ax.plot(low[:, 0], low[:, 1], ".", ms=1.5, color="orange", label="floor splats (2σ top < chassis + 5 mm)")
        ax.add_patch(plt.Rectangle(box[:2], box[2] - box[0], box[3] - box[1], fill=False, ec="k", ls="--", lw=1))
        R = np.asarray(c["astar_route_uv"])
        ax.plot(R[:, 0], R[:, 1], "-", color="tab:blue", lw=1.5, label="A* witness route (real body, 1 mm)")
        if c.get("tight_uv") is not None:
            ax.add_patch(Circle(c["tight_uv"], body.radius_m, fill=False, ec="tab:blue", lw=1))
        if c.get("gmc_route_uv"):
            G = np.asarray(c["gmc_route_uv"])
            ax.plot(G[:, 0], G[:, 1], "-", color="tab:green", lw=1.2, label="GMC route")
        if c.get("cut_gaussians"):
            X = np.asarray([g[1:3] for g in c["cut_gaussians"]])
            ax.plot(X[:, 0], X[:, 1], "x", color="red", ms=4, label="GMC cut-certificate Gaussians")
        for uv, m, lab in ((c["start_uv"], "o", "start"), (c["goal_uv"], "s", "goal")):
            ax.add_patch(Circle(uv, body.radius_m, fill=False, ec="m", lw=1))
            ax.plot(*uv, m, color="m", ms=7, label=lab)
        ax.set_xlim(ext[0], ext[1])
        ax.set_ylim(ext[2], ext[3])
        ax.set_xlabel("u (m)")
        ax.set_ylabel("v (m)")
        ax.set_title(f"{c['case_id']} {c['robot']}: GMC {c['gmc_status']} {c['gmc_reason']}\n{c.get('note', '')}",
                     fontsize=9)
        ax.legend(fontsize=7, loc="upper right")
        fig.tight_layout()
        fig.savefig(out / f"{c['case_id']}_{c['robot']}.png", dpi=90)
        plt.close(fig)
        print("wrote", c["case_id"], c["robot"], flush=True)


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("cases")
    c.add_argument("--cases", required=True)
    c.add_argument("--out-dir", required=True)
    a = p.parse_args(argv)
    {"cases": cmd_cases}[a.cmd](a)


if __name__ == "__main__":
    main()
