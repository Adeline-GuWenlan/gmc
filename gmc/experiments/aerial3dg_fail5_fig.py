"""F5 Task 2: one review figure per failure case + class-overview maps (sbatch; reads only F5's trace records and the
compile's pair arrays saved by ``aerial3dg_fail5_trace.py``, no scene load).

Run from ``gmc/`` with ``PYTHONPATH=src:experiments MPLBACKEND=Agg``.

  cases     --sel results/aerial3dg/f5/fig_selection.json [--only CASE_ID ...]
            One PNG per selected (case, robot): results/aerial3dg/f5/fig/<case_id>_<robot>.png
              left   top-down map of the region (route frame u, v; 2 cm cells): every Gaussian of the compile whose
                     2-sigma z-extent overlaps the robot's body band, painted over its 2-sigma xy box and coloured by its
                     2-sigma top height; floor splats under the chassis (2-sigma top in [bottom - 10 mm, bottom]) as
                     orange dots; the compile box (dashed) and its domain inset; the A* route (blue) with the body
                     disk at its tightest lateral point; start (o) / goal (s) with the body footprint; GMC's partial
                     result: endpoint cells, the certified safe-graph path cells (green outlines), GMC's route
                     (green; for TIMEOUT rows the verdict-only certified route), the replay's failing edge and its
                     swept AABB (red), the endpoint blockers (red x); the failure locus (red circle)
              right  zoom at the failure locus (2-sigma xy ellipses, same colours, floor splats with their 2-sigma
                     top in mm, body footprint at the locus, 10 cm scale bar); side profile through the locus along
                     the local route direction (each Gaussian within body radius + 15 cm of the line: 2-sigma box in
                     (along, z); body rectangle; chassis bottom, - margin, - margin - buffer lines); the three
                     verification facts and the probe results as text
  overview  --sel ...   one class-overview map per class with > 30 rows: every row's failure locus over the region map
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Ellipse, Polygon, Rectangle
from mpl_toolkits.axes_grid1.anchored_artists import AnchoredSizeBar

F5 = Path("results/aerial3dg/f5")
BIG = Path("outputs/aerial3dg/f5")
STEP = .02
CMAP = plt.get_cmap("viridis")
FLOOR_C = "#e8730c"
ROUTE_C, GMC_C, FAIL_C, EP_C = "#1f5fbf", "#1a9a4b", "#d62728", "#b0249b"


class Scene:
    """The compile's pairs for one (region, robot) in the route frame + a 2 cm band raster (max 2-sigma top)."""

    def __init__(self, region, robot, box):
        d = np.load(BIG / f"{region}_{robot}_pairs.npz")
        self.box = box
        self.body = json.loads(str(d["body"]))
        self.r = self.body["radius_m"]
        self.bot = self.body["ground_clearance_m"]
        self.top_b = self.bot + 2 * self.body["half_height_m"]
        self.z_c = float(d["z_c"])
        self.ks_lo, self.ks_hi = d["ks_lower"], d["ks_upper"]
        lev = float(d["level"])
        mu, cov = d["means"].astype(float), d["covs"].astype(float)
        sd = lev * np.sqrt(np.maximum(np.diagonal(cov, axis1=1, axis2=2), 0.))
        self.mu, self.cxy, self.sd, self.lev = mu, cov[:, :2, :2], sd, lev
        self.ztop, self.zbot = mu[:, 2] + sd[:, 2], mu[:, 2] - sd[:, 2]
        self.ids = d["ids"]
        self.band = (self.ztop > self.bot) & (self.zbot < self.top_b)
        self.floor = (self.ztop <= self.bot) & (self.ztop >= self.bot - .01)
        pad = .3
        u0, v0, u1, v1 = box[0] - pad, box[1] - pad, box[2] + pad, box[3] + pad
        self.ext = (u0, u1, v0, v1)
        nu, nv = int(np.ceil((u1 - u0) / STEP)), int(np.ceil((v1 - v0) / STEP))
        img = np.full((nv, nu), np.nan)
        k = np.flatnonzero(self.band)
        k = k[np.argsort(self.ztop[k])]                    # higher tops painted last
        x, s, t = mu[k, :2], sd[k, :2], np.minimum(self.ztop[k], self.top_b)
        a0 = np.clip(((x[:, 0] - s[:, 0] - u0) / STEP).astype(int), 0, nu)
        a1 = np.clip(((x[:, 0] + s[:, 0] - u0) / STEP).astype(int) + 1, 0, nu)
        b0 = np.clip(((x[:, 1] - s[:, 1] - v0) / STEP).astype(int), 0, nv)
        b1 = np.clip(((x[:, 1] + s[:, 1] - v0) / STEP).astype(int) + 1, 0, nv)
        for i in range(len(k)):
            if a1[i] > a0[i] and b1[i] > b0[i]:
                img[b0[i]:b1[i], a0[i]:a1[i]] = t[i]
        self.img = img
        self.inwin = (mu[:, 0] > u0) & (mu[:, 0] < u1) & (mu[:, 1] > v0) & (mu[:, 1] < v1)

    def norm(self, z):
        return (np.asarray(z) - self.bot) / (self.top_b - self.bot)

    def draw_map(self, ax, floor_dots=True):
        ax.imshow(self.img, origin="lower", extent=self.ext, cmap=CMAP, vmin=self.bot, vmax=self.top_b,
                  interpolation="nearest", alpha=.85)
        if floor_dots:
            f = self.floor & self.inwin
            ax.plot(self.mu[f, 0], self.mu[f, 1], ".", ms=1.2, color=FLOOR_C, alpha=.7, zorder=2)
        b = self.box
        ax.add_patch(Rectangle(b[:2], b[2] - b[0], b[3] - b[1], fill=False, ec="k", ls="--", lw=.8))
        lo, hi = self.ks_lo, self.ks_hi
        ax.add_patch(Rectangle((lo[0], lo[1]), hi[0] - lo[0], hi[1] - lo[1], fill=False, ec="0.4", ls=":", lw=.6))
        ax.set_xlim(self.ext[0], self.ext[1])
        ax.set_ylim(self.ext[2], self.ext[3])
        ax.set_aspect("equal")

    def local(self, c, rad):
        m = np.all(np.abs(self.mu[:, :2] - c) < rad + 3 * self.sd[:, :2].max(1, keepdims=True), axis=1)
        return np.flatnonzero(m & (self.band | self.floor))


def ellipse(mu, C, lev, **kw):
    w, V = np.linalg.eigh(C)
    w = np.maximum(w, 1e-12)
    ang = np.degrees(np.arctan2(V[1, 1], V[0, 1]))
    return Ellipse(mu, 2 * lev * np.sqrt(w[1]), 2 * lev * np.sqrt(w[0]), angle=ang, **kw)


def _poly(ax, P, **kw):
    if P:
        ax.add_patch(Polygon(np.asarray(P), closed=True, **kw))


def _line(ax, P, **kw):
    if P:
        P = np.asarray(P)
        ax.plot(P[:, 0], P[:, 1], **kw)


def overlays(ax, sc, c, lw=1.):
    """GMC partial result + A* route on an axis (map or zoom)."""
    q = c["gmc_view"]
    for cell in q.get("path_cells") or []:
        _poly(ax, cell["poly_uv"], fill=False, ec=GMC_C, lw=.45 * lw, alpha=.55, zorder=3)
    for e, cell in (q.get("endpoint_cells") or {}).items():
        _poly(ax, cell["poly_uv"], fill=True, fc=GMC_C, alpha=.18, ec=GMC_C, lw=.8 * lw, zorder=3)
    _line(ax, c["astar_uv"], color=ROUTE_C, lw=1.4 * lw, zorder=4)
    _line(ax, q.get("route_uv"), color=GMC_C, lw=1.1 * lw, ls="-", zorder=5)
    for uv, mk in ((c["start_uv"], "o"), (c["goal_uv"], "s")):
        ax.add_patch(Circle(uv, sc.r, fill=False, ec=EP_C, lw=1. * lw, zorder=6))
        ax.plot(*uv, mk, color=EP_C, ms=5 * lw, zorder=6)
    fe = q.get("failed_edge")
    if fe:
        _line(ax, [fe["a_uv"], fe["b_uv"]], color=FAIL_C, lw=2.5 * lw, zorder=7)
        A = np.asarray(fe.get("aabb_corners_route_uv") or [])
        if len(A):
            from scipy.spatial import ConvexHull
            h = ConvexHull(A)
            ax.add_patch(Polygon(A[h.vertices], closed=True, fill=False, ec=FAIL_C, lw=.8 * lw, ls="--", zorder=7))
    for b in c.get("blockers") or []:
        ax.plot(b["mean_route"][0], b["mean_route"][1], "x", color=FAIL_C, ms=7 * lw, mew=1.5, zorder=8)
    lo, hi = sc.ks_lo, sc.ks_hi
    ax.add_patch(Rectangle((lo[0], lo[1]), hi[0] - lo[0], hi[1] - lo[1], fill=False, ec="k", ls="--", lw=.9, zorder=3))
    t = c.get("tight_uv")
    if t:
        ax.add_patch(Circle(t, sc.r, fill=False, ec=ROUTE_C, lw=.8 * lw, ls="--", zorder=6))


def render(sc, c, out):
    fig = plt.figure(figsize=(17, 9.2))
    gs = fig.add_gridspec(2, 3, width_ratios=[2.5, 1.05, .95], height_ratios=[1, 1], wspace=.22, hspace=.28,
                          left=.04, right=.995, top=.88, bottom=.06)
    ax = fig.add_subplot(gs[:, 0])
    sc.draw_map(ax)
    overlays(ax, sc, c, lw=.8)
    L = np.asarray(c["locus_uv"])
    ax.add_patch(Circle(L, .25, fill=False, ec=FAIL_C, lw=1.5, zorder=9))
    ax.set_xlabel("u (m, route frame)")
    ax.set_ylabel("v (m)")
    # zoom
    W = c.get("zoom_half", max(.3, sc.r + .2))
    az = fig.add_subplot(gs[0, 1])
    k = sc.local(L, W)
    order = k[np.argsort(sc.ztop[k])]
    for i in order:
        if sc.floor[i]:
            az.add_patch(ellipse(sc.mu[i, :2], sc.cxy[i], sc.lev, fill=False, ec=FLOOR_C, lw=.7, zorder=2))
        else:
            az.add_patch(ellipse(sc.mu[i, :2], sc.cxy[i], sc.lev, fc=CMAP(float(np.clip(sc.norm(sc.ztop[i]), 0, 1))),
                                 ec="none", alpha=.55, zorder=1))
    for b in c.get("blockers") or []:
        az.annotate(f"{b['scene_id']}\n2σ top {b['top_z_2sigma'] * 1e3:.2f} mm", b["mean_route"][:2], fontsize=6.5,
                    color=FAIL_C, xytext=(4, 4), textcoords="offset points", zorder=10)
    overlays(az, sc, c, lw=1.)
    az.add_patch(Circle(L, sc.r, fill=False, ec=FAIL_C, lw=1.6, zorder=9))
    az.plot(*L, "+", color=FAIL_C, ms=12, mew=1.6, zorder=11)
    az.annotate(c["locus_kind"].split(" (")[0], L, fontsize=6.5, color=FAIL_C, xytext=(6, -12),
                textcoords="offset points", zorder=11)
    az.set_xlim(L[0] - W, L[0] + W)
    az.set_ylim(L[1] - W, L[1] + W)
    az.set_aspect("equal")
    az.tick_params(labelsize=7)
    az.add_artist(AnchoredSizeBar(az.transData, .1, "10 cm", "lower right", pad=.3, frameon=True, size_vertical=.004,
                                  fontproperties={"size": 7}))
    az.set_title(f"zoom at failure locus: {c['locus_kind']} (red circle = body footprint)", fontsize=8)
    # side profile
    ap = fig.add_subplot(gs[1, 1])
    t = np.asarray(c.get("locus_dir") or [1., 0.], float)
    t = t / max(np.linalg.norm(t), 1e-12)
    nrm = np.array([-t[1], t[0]])
    k = sc.local(L, .7)
    rel = sc.mu[k, :2] - L
    s_al, w = rel @ t, rel @ nrm
    sel = (np.abs(w) < sc.r + .15) & (np.abs(s_al) < .6)
    zoomz = c["profile"] == "floor"
    for i, s_, w_ in zip(k[sel], s_al[sel], w[sel]):
        ss = sc.lev * np.sqrt(max(float(t @ sc.cxy[i] @ t), 0.))
        col = FLOOR_C if sc.floor[i] else CMAP(float(np.clip(sc.norm(sc.ztop[i]), 0, 1)))
        ap.add_patch(Rectangle((s_ - ss, sc.zbot[i]), 2 * ss, sc.ztop[i] - sc.zbot[i], fc=col,
                               alpha=.35 if abs(w_) > sc.r else .75, ec="k" if abs(w_) <= sc.r else "none", lw=.3))
    ap.add_patch(Rectangle((-sc.r, sc.bot), 2 * sc.r, sc.top_b - sc.bot, fill=False, ec=FAIL_C, lw=1.4))
    for z, ls, lab in ((sc.bot, "-", "chassis bottom"), (sc.bot - .001, "--", "- margin 1 mm"),
                       (sc.bot - .002, ":", "- margin - buffer 2 mm")):
        ap.axhline(z, color="k", ls=ls, lw=.7, label=lab)
    ap.set_xlim(-.6, .6)
    if zoomz:
        ap.set_ylim(sc.bot - .012, sc.bot + .008)
        ap.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v * 1e3:.0f}"))
        ap.set_ylabel("z above floor (mm)", fontsize=8)
    else:
        ap.set_ylim(-.02, sc.top_b + .1)
        ap.set_ylabel("z above floor (m)", fontsize=8)
    ap.set_xlabel(f"distance {c.get('locus_dir_kind', 'along the route')} through the locus (m); "
                  "dark-edged = within body radius of the cut", fontsize=7)
    ap.tick_params(labelsize=7)
    ap.legend(fontsize=6, loc="upper right", framealpha=.8)
    ap.set_title("side (z) profile at the locus" + (" (zoomed to the floor splats)" if zoomz else ""), fontsize=8)
    # text
    at = fig.add_subplot(gs[:, 2])
    at.axis("off")
    at.text(0, 1, c["text"], va="top", ha="left", fontsize=6.9, family="monospace")
    # legend (map)
    from matplotlib.lines import Line2D
    hs = [Line2D([], [], color=ROUTE_C, lw=1.5, label="A* route (real body, 1 mm)"),
          Line2D([], [], color=GMC_C, lw=1.2, label=c["gmc_view"].get("route_label", "GMC route")),
          Rectangle((0, 0), 1, 1, fill=False, ec=GMC_C, label="GMC certified cells (path / endpoint)"),
          Line2D([], [], color=FAIL_C, lw=2.5, label="replay failing edge / blocker (x)"),
          Line2D([], [], marker=".", ls="", color=FLOOR_C, label="floor splats (2σ top ≤ chassis bottom)"),
          Circle((0, 0), 1, fill=False, ec=EP_C, label="start o / goal s, body footprint")]
    ax.legend(handles=hs, fontsize=7, loc="upper left", framealpha=.85)
    sm = plt.cm.ScalarMappable(cmap=CMAP, norm=plt.Normalize(sc.bot, sc.top_b))
    cb = fig.colorbar(sm, ax=ax, fraction=.025, pad=.01)
    cb.set_label("2σ top of body-band Gaussians (m above floor)", fontsize=7)
    cb.ax.tick_params(labelsize=7)
    fig.suptitle(c["title"], fontsize=10.5, x=.02, ha="left")
    fig.savefig(out, dpi=95)
    plt.close(fig)


def cmd_cases(a):
    sel = json.loads(Path(a.sel).read_text())
    out = F5 / "fig"
    out.mkdir(parents=True, exist_ok=True)
    scenes = {}
    cases = [c for c in sel["cases"] if not a.only or c["case_id"] in a.only]
    if a.part is not None:
        cases = [c for i, c in enumerate(sorted(cases, key=lambda c: (c["region"], c["robot"], c["case_id"])))
                 if i % a.parts == a.part]
    for c in sorted(cases, key=lambda c: (c["region"], c["robot"], c["case_id"])):
        key = (c["region"], c["robot"])
        if key not in scenes:
            scenes.clear()
            scenes[key] = Scene(c["region"], c["robot"], sel["boxes"][c["region"]])
        f = out / f"{c['case_id']}_{c['robot']}.png"
        render(scenes[key], c, f)
        print("wrote", f, flush=True)


def cmd_overview(a):
    sel = json.loads(Path(a.sel).read_text())
    out = F5 / "fig" / "overview"
    out.mkdir(parents=True, exist_ok=True)
    for ov in sel["overviews"]:
        regs = sorted({p["region"] for p in ov["points"]})
        fig, axes = plt.subplots(len(regs), 1, figsize=(15, 4.6 * len(regs)), squeeze=False)
        for ax, reg in zip(axes[:, 0], regs):
            sc = Scene(reg, ov["robot_map"], sel["boxes"][reg])
            sc.draw_map(ax)
            P = [p for p in ov["points"] if p["region"] == reg]
            X = np.asarray([p["uv"] for p in P])
            v = np.asarray([p["value"] for p in P], float)
            s = ax.scatter(X[:, 0], X[:, 1], c=v, cmap=ov.get("cmap", "magma_r"), s=16, ec="k", lw=.3, zorder=5,
                           vmin=ov.get("vmin"), vmax=ov.get("vmax"))
            for b in ov.get("blockers", []):
                if b["region"] == reg:
                    ax.plot(*b["uv"], "x", color=FAIL_C, ms=8, mew=1.6, zorder=6)
                    ax.annotate(f"{b['scene_id']} ({b['n']} rows)", b["uv"], fontsize=7, color=FAIL_C,
                                xytext=(4, -10), textcoords="offset points")
            cb = fig.colorbar(s, ax=ax, fraction=.02, pad=.01)
            cb.set_label(ov["value_label"], fontsize=8)
            ax.set_title(f"{reg}: {len(P)} rows", fontsize=9)
        fig.suptitle(ov["title"], fontsize=11, x=.02, ha="left")
        fig.tight_layout()
        f = out / f"{ov['name']}.png"
        fig.savefig(f, dpi=90)
        plt.close(fig)
        print("wrote", f, flush=True)


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("cases")
    c.add_argument("--sel", default=str(F5 / "fig_selection.json"))
    c.add_argument("--only", nargs="*", default=None)
    c.add_argument("--part", type=int, default=None)
    c.add_argument("--parts", type=int, default=1)
    o = sub.add_parser("overview")
    o.add_argument("--sel", default=str(F5 / "fig_selection.json"))
    a = p.parse_args(argv)
    {"cases": cmd_cases, "overview": cmd_overview}[a.cmd](a)


if __name__ == "__main__":
    main()
