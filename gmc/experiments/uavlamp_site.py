"""Measure one candidate site (two real walls + a real table) in a wall-aligned frame.

Read-only on the archive.  Fits both wall faces from opaque Gaussians above table
height, defines the site frame (u along the first wall, v into the gallery, z up
from the plane floor), then measures table top/footprint, floor top, local
ceiling and wall face positions per u-bin.  Writes ``site.json`` + PNGs.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

TAU, LEVEL = .3, 2.


def fit_wall(means, covs, zc, a, b, other, band=.45):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = (b - a) / np.linalg.norm(b - a)
    n = np.array([-d[1], d[0]])
    if np.dot(np.asarray(other) - a, n) < 0:
        n = -n
    rel = means[:, :2] - a
    s, t = rel @ d, rel @ n
    sel = (np.abs(t) < band) & (s > -.5) & (s < np.linalg.norm(b - a) + .5) & (zc > 1.3) & (zc < 2.6)
    # PCA refit of direction on the selected centres.
    pts = means[sel, :2]
    c = pts.mean(0)
    w, v = np.linalg.eigh(np.cov((pts - c).T))
    d2 = v[:, -1] * np.sign(v[:, -1] @ d)
    n2 = np.array([-d2[1], d2[0]]) * np.sign(np.array([-d2[1], d2[0]]) @ n)
    n3 = np.array([n2[0], n2[1], 0.])
    ext = LEVEL * np.sqrt(np.einsum("i,nij,j->n", n3, covs[sel], n3))
    face = (means[sel, :2] - c) @ n2 + ext
    return {"point": c.tolist(), "dir": d2.tolist(), "normal_into_gallery": n2.tolist(),
            "n_supports": int(sel.sum()),
            "face_offset_quantiles": {q: float(np.quantile(face, float(q))) for q in ("0.5", "0.9", "0.99", "0.999")}}


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--archive", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--wall-a", type=float, nargs=4, required=True, help="x0 y0 x1 y1 of wall 1")
    p.add_argument("--wall-b", type=float, nargs=4, required=True, help="x0 y0 x1 y1 of wall 2")
    p.add_argument("--u-range", type=float, nargs=2, default=(-1., 10.))
    a = p.parse_args(argv)
    out = a.output
    out.mkdir(parents=True, exist_ok=True)
    with np.load(a.archive, allow_pickle=False) as d:
        meta = json.loads(str(d["meta"]))
        op = d["opacity"] > TAU
        means_all = d["means"]
        box = op & (means_all[:, 0] > min(a.wall_a[0::2] + a.wall_b[0::2]) - 3) \
            & (means_all[:, 0] < max(a.wall_a[0::2] + a.wall_b[0::2]) + 3) \
            & (means_all[:, 1] > min(a.wall_a[1::2] + a.wall_b[1::2]) - 3) \
            & (means_all[:, 1] < max(a.wall_a[1::2] + a.wall_b[1::2]) + 3)
        means = means_all[box]
        covs = d["covs"][box]
        ids = d["ids"][box]
    zf = float(meta["z_floor"])
    zc = means[:, 2] - zf
    wa = fit_wall(means, covs, zc, a.wall_a[:2], a.wall_a[2:], a.wall_b[:2])
    wb = fit_wall(means, covs, zc, a.wall_b[:2], a.wall_b[2:], a.wall_a[:2])
    # Site frame: u along wall A, v = wall A normal into the gallery, origin on wall A's face
    # (99.9% quantile of the gallery-facing 2-sigma extent) at the fitted centre.
    d2, n2 = np.asarray(wa["dir"]), np.asarray(wa["normal_into_gallery"])
    face = wa["face_offset_quantiles"]["0.999"]
    origin_xy = np.asarray(wa["point"]) + face * n2
    R = np.array([[d2[0], d2[1], 0.], [n2[0], n2[1], 0.], [0., 0., 1.]])
    origin = np.array([origin_xy[0], origin_xy[1], zf])
    loc = (means - origin) @ R.T
    hz = LEVEL * np.sqrt(covs[:, 2, 2])
    ex = LEVEL * np.sqrt(np.einsum("ij,njk,ik->ni", R, covs, R))  # route-axis 2-sigma half extents
    # Wall B face in site frame, per u bin: min over supports of (v - extent_v), above table height.
    u0, u1 = a.u_range
    ubins = np.arange(u0, u1 + 1e-9, .5)
    nb = np.asarray(wb["normal_into_gallery"])
    wall_rows = []
    for lo in ubins[:-1]:
        m = (loc[:, 0] >= lo) & (loc[:, 0] < lo + .5)
        hi_z = m & (loc[:, 2] > .05) & (loc[:, 2] < 2.6)
        a_side = hi_z & (loc[:, 1] < .6)
        b_side = hi_z & (loc[:, 1] > 1.2)
        row = {"u": [float(lo), float(lo + .5)]}
        if a_side.any():
            row["wallA_face_v_max"] = float(np.quantile(loc[a_side & (loc[:, 1] < .25), 1]
                                                        + ex[a_side & (loc[:, 1] < .25), 1], .999)) \
                if (a_side & (loc[:, 1] < .25)).any() else None
        if b_side.any():
            sel = b_side & (loc[:, 1] > 1.5)
            row["wallB_face_v_min_q001"] = float(np.quantile(loc[sel, 1] - ex[sel, 1], .001)) if sel.any() else None
        wall_rows.append(row)
    # Rasters in site frame, 0.05 m cells over u range and v in [-0.3, 4].
    c = .05
    V0, V1 = -.3, 4.
    nu, nv = int((u1 - u0) / c), int((V1 - V0) / c)
    iu = ((loc[:, 0] - u0) / c).astype(int)
    iv = ((loc[:, 1] - V0) / c).astype(int)
    ok = (iu >= 0) & (iu < nu) & (iv >= 0) & (iv < nv)
    top = loc[:, 2] + hz
    bot = loc[:, 2] - hz
    below = ok & (top < 1.6) & (top > .3)
    tmax = np.full((nv, nu), np.nan)
    np.fmax.at(tmax, (iv[below], iu[below]), top[below])
    floor = ok & (loc[:, 2] < .15)
    ftop = np.full((nv, nu), np.nan)
    np.fmax.at(ftop, (iv[floor], iu[floor]), top[floor])
    ceil = ok & (bot > 2.6)
    cbot = np.full((nv, nu), np.nan)
    np.fmin.at(cbot, (iv[ceil], iu[ceil]), bot[ceil])
    np.savez_compressed(out / "site_rasters.npz", tmax=tmax, ftop=ftop, cbot=cbot, u0=u0, v0=V0, cell=c)
    ext = [u0, u0 + nu * c, V0, V0 + nv * c]
    fig, axs = plt.subplots(3, 1, figsize=(14, 13))
    for ax, img, title, kw in ((axs[0], tmax, "max top (m) of supports with top in .3-1.6", dict(vmin=.3, vmax=1.6)),
                               (axs[1], ftop, "floor-support top (m)", dict(vmin=-.05, vmax=.2)),
                               (axs[2], cbot, "lowest support bottom above 2.6 m", dict(vmin=2.6, vmax=5.5))):
        im = ax.imshow(img, origin="lower", extent=ext, cmap="viridis", **kw)
        plt.colorbar(im, ax=ax); ax.set_title(title); ax.set_xlabel("u"); ax.set_ylabel("v")
        ax.set_xticks(np.arange(u0, u1 + .1, .5)); ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(out / "site_rasters.png", dpi=80); plt.close(fig)
    # Scatter views.
    sel = (loc[:, 0] > u0) & (loc[:, 0] < u1) & (loc[:, 1] > V0) & (loc[:, 1] < V1)
    fig, axs = plt.subplots(3, 1, figsize=(14, 15))
    for ax, zb in zip(axs[:2], ((.05, 1.3), (1.3, 3.0))):
        m = sel & (loc[:, 2] > zb[0]) & (loc[:, 2] < zb[1])
        ax.scatter(loc[m, 0], loc[m, 1], s=.3, c=loc[m, 2], cmap="viridis", vmin=0, vmax=3)
        ax.set_title(f"top view, centres z in {zb}"); ax.set_aspect("equal"); ax.grid(alpha=.3)
        ax.set_xticks(np.arange(u0, u1 + .1, .5))
    m = sel & (loc[:, 1] > .1) & (loc[:, 1] < 2.0)
    axs[2].scatter(loc[m, 0], loc[m, 2], s=.3, c=loc[m, 1], cmap="coolwarm", vmin=0, vmax=2)
    axs[2].set_title("side view u-z, v in (.1, 2.0)"); axs[2].set_aspect("equal"); axs[2].grid(alpha=.3)
    axs[2].set_ylim(-.2, 5.5); axs[2].set_xticks(np.arange(u0, u1 + .1, .5))
    fig.tight_layout(); fig.savefig(out / "site_scatter.png", dpi=80); plt.close(fig)
    # Table: supports with top in [.5, 1.3] and v in (0, 1.6).
    tab = sel & (top > .5) & (top < 1.3) & (loc[:, 1] > 0) & (loc[:, 1] < 1.6) & (loc[:, 2] > .3)
    doc = {"archive": str(a.archive), "z_floor": zf, "wall_a": wa, "wall_b": wb,
           "frame": {"origin_world_m": origin.tolist(), "world_to_route": R.tolist(),
                     "definition": "u along wall A, v = wall A normal into gallery, origin on wall A 99.9% face, z from plane floor"},
           "wall_rows": wall_rows,
           "table_candidates_top_quantiles": ({q: float(np.quantile(top[tab], q)) for q in (.5, .9, .99, 1.)}
                                              if tab.any() else None),
           "table_candidates_u_range": ([float(loc[tab, 0].min()), float(loc[tab, 0].max())] if tab.any() else None),
           "table_candidates_v_range": ([float(loc[tab, 1].min()), float(loc[tab, 1].max())] if tab.any() else None),
           "floor_top_quantiles": {q: float(np.nanquantile(ftop, q)) for q in (.5, .99, 1.)},
           "ceiling_bottom_quantiles": {q: float(np.nanquantile(cbot, q)) for q in (0., .01, .5)}}
    (out / "site.json").write_text(json.dumps(doc, indent=1))
    print(json.dumps(doc, indent=1)[:4000])


if __name__ == "__main__":
    main()
