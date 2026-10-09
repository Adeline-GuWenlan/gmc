"""F5 follow-up: 3D renders of ONE failure case (default WWEST-F4X-01018, cylinder) from the exact planning archive.

Read-only.  Draws, in the real scene_v2 splats (per-Gaussian DC colours, ceiling cut away), the robot body (r 0.3 m,
0.02-1.75 m above the floor) and four routes on the floor:
  blue    A* route (real body, 1 mm margin; F5 trace ``astar_route_uv``)
  green   GMC route after the 60 s post-processing budget (probe ``budget``)
  grey    GMC route before shortening (probe ``verdict_only``, 29 vertices)
  red     the straight start -> goal line (what a human expects), dashed
Outputs (``--out``): overview_top.png, overview_oblique.png, locus_closeup.png, straight_line_blocker.png,
flythrough_gmc.mp4 (+ frames), render_manifest.json.

Run from ``gmc/`` with ``PYTHONPATH=src:experiments MPLBACKEND=Agg`` (sbatch: hpc/aerial3dg/f5_render3d.sbatch).
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from gmc.gs3d import scene_uavlamp as su
from gmc.height import ewa
from gmc.height.viz3d import load_dc_colors

from aerial3dg_run import ARCHIVE, MANIFEST
from uavlamp_render import cam_from

PLY = Path("/scratch/wg2381/splathjb/splatc_atlas/data/gs_scenes/showcase/raw/point_cloud.ply")
N_PLY = 7_340_008
TRACE = Path("results/aerial3dg/f5/trace")
PLAN = Path("results/aerial3dg/f5/plan")
BODY = dict(radius_m=0.3, z_lo=0.02, z_hi=1.75)             # CYLINDER (a3c manifest): clearance 0.02, half height 0.865
WIN = (-10.8, -2.0, -0.8, 4.6)                              # u0, v0, u1, v1 of the WWEST render window (route frame)
Z_CUT = 2.4                                                 # drop the ceiling / soffit
COL = {"astar": "#1f77ff", "gmc": "#14a05a", "raw": "#8a8a8a", "straight": "#e0262c"}
BODY_FACE, BODY_EDGE = (1.0, .45, .0, .55), (.75, .2, .0)


def load_case(case_id):
    region, robot = case_id.split("-")[0], "cylinder"
    pid = case_id.split("-", 1)[1]
    rec = plan = None
    for f in sorted(TRACE.glob(f"{region}_{robot}_timeout_*.jsonl")):
        for line in f.read_text().splitlines():
            r = json.loads(line)
            if r.get("pair_id") == pid:
                rec = r
    for f in sorted(PLAN.glob(f"{region}_{robot}_timeout_*.jsonl")):
        for line in f.read_text().splitlines():
            r = json.loads(line)
            if r.get("pair_id") == pid:
                plan = r
    if rec is None or plan is None:
        raise SystemExit(f"case {case_id} not found in {TRACE} / {PLAN}")
    ast = plan["astar_route_uv"]
    ast = json.loads(ast) if isinstance(ast, str) else ast
    P = rec["probes"]
    return {"start": rec["endpoints"]["start"]["uv"], "goal": rec["endpoints"]["goal"]["uv"],
            "astar": np.asarray(ast, float), "gmc": np.asarray(P["budget"]["polyline_uv"], float),
            "raw": np.asarray(P["verdict_only"]["polyline_uv"], float),
            "locus": rec["astar"]["lateral_tightest_edge"]["uv"], "rec": rec}


class Scene:
    def __init__(self):
        full, doc = su.load_uavlamp_derivative(ARCHIVE, MANIFEST)
        self.frame = su.Frame.from_dict(doc["frame"])
        loc = self.frame.to_route(full.means)
        keep = ((loc[:, 0] > WIN[0]) & (loc[:, 0] < WIN[2]) & (loc[:, 1] > WIN[1]) & (loc[:, 1] < WIN[3])
                & (loc[:, 2] > -.15) & (loc[:, 2] < Z_CUT) & (full.opacity > .05))
        ids = full.ids[keep]
        rgb = np.full((len(ids), 3), .62)
        real = ids < N_PLY
        rgb[real] = load_dc_colors(PLY, ids[real])
        for e in doc["edits"]:
            m = (ids >= e["id_range"][0]) & (ids <= e["id_range"][1])
            rgb[m] = e["color_rgb"]
        self.n = int(keep.sum())
        self.splats = ewa.pack(full.means[keep], full.covs[keep], full.opacity[keep], rgb)
        self.sha = doc["derivative"]["sha256"]

    def world(self, uv, z=0.01):
        uv = np.atleast_2d(uv)
        return self.frame.to_world(np.column_stack([uv[:, 0], uv[:, 1], np.full(len(uv), z)]))


def densify(P, step=0.02):
    out = [P[0]]
    for a, b in zip(P[:-1], P[1:]):
        n = max(int(np.ceil(np.linalg.norm(b - a) / step)), 1)
        out += [a + (b - a) * k / n for k in range(1, n + 1)]
    return np.asarray(out)


def overlay(ax, cam, surface, pts_w, colour, lw=2.2, ls="-", slack=.08):
    u, v, d = ewa.project_points(cam, pts_w)
    front = d > cam.near
    iu = np.clip(np.nan_to_num(u), 0, cam.width - 1).astype(int)
    iv = np.clip(np.nan_to_num(v), 0, cam.height - 1).astype(int)
    on = front & (u >= 0) & (u < cam.width) & (v >= 0) & (v < cam.height)
    vis = on & (d <= surface[iv, iu] + slack)
    ax.plot(np.where(front, u, np.nan), np.where(front, v, np.nan), color=colour, lw=lw * .5, ls=(0, (3, 2)),
            alpha=.55, zorder=3)                                     # hidden part: thin dashed
    ax.plot(np.where(vis, u, np.nan), np.where(vis, v, np.nan), color=colour, lw=lw, ls=ls, zorder=4)


def body_layer(S, cam, centres_uv):
    r = BODY["radius_m"]
    prisms, edges = [], []
    a = np.linspace(0, 2 * np.pi, 49)
    for c in centres_uv:
        w0 = S.world(c, 0.0)[0]
        # world z is the vertical used by every renderer in this chain (aerial3dg_viz.draw_frame_body)
        z0, z1 = w0[2] + BODY["z_lo"], w0[2] + BODY["z_hi"]
        prisms.append((w0[0], w0[1], r, z0, z1))
        for z in (z0, z1):
            ring = np.column_stack([w0[0] + r * np.cos(a), w0[1] + r * np.sin(a), np.full_like(a, z)])
            edges += [ring[k:k + 2] for k in range(48)]
    return ewa.prism_occluder(cam, prisms, face_rgba=BODY_FACE, edge_rgb=BODY_EDGE, edges=np.asarray(edges),
                              line_width=1.6)


def shot(S, C, eye_uv_z, tgt_uv_z, size, fov, bodies, routes, path, title, marks=()):
    cam = cam_from(S.frame, list(eye_uv_z), list(tgt_uv_z), *size, fov=fov)
    occ = body_layer(S, cam, bodies) if len(bodies) else None
    out = ewa.render(S.splats, cam, occ=occ, bg=(.97, .97, .97), xray=.25)
    surface = out["depth"] if occ is None else np.minimum(out["depth"], occ[2])
    fig = plt.figure(figsize=(size[0] / 100, size[1] / 100), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(np.clip(out["rgb"], 0, 1))
    for name, lw, ls in routes:
        P = np.vstack([C["start"], C["goal"]]) if name == "straight" else C[name]
        overlay(ax, cam, surface, S.world(densify(np.asarray(P, float)), .01), COL[name], lw, ls)
    for label, uv_ in marks:
        u, v, d = ewa.project_points(cam, S.world(uv_, .02))
        ax.text(float(u[0]) + 6, float(v[0]) - 6, label, fontsize=9, color="k", fontweight="bold",
                bbox=dict(fc="white", ec="none", alpha=.8))
    ax.text(12, 22, title, fontsize=9.5, bbox=dict(fc="white", ec="none", alpha=.85))
    hx = 12
    for name, lab in (("astar", "A* route"), ("gmc", "GMC route (60 s budget)"), ("raw", "GMC unshortened"),
                      ("straight", "straight start-goal line")):
        ax.plot([hx, hx + 26], [size[1] - 16] * 2, color=COL[name], lw=3, ls="--" if name == "straight" else "-")
        ax.text(hx + 32, size[1] - 11, lab, fontsize=8.5, bbox=dict(fc="white", ec="none", alpha=.7))
        hx += 40 + 7.2 * len(lab)
    ax.set_xlim(0, size[0]); ax.set_ylim(size[1], 0); ax.axis("off")
    fig.savefig(path, dpi=100)
    plt.close(fig)
    print("wrote", path, flush=True)


def flythrough(S, C, out, *, seconds=14., fps=20, size=(960, 540)):
    import imageio_ffmpeg
    route = densify(C["gmc"], 0.01)
    L = np.r_[0, np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
    n_move = int(seconds * fps)
    s = np.r_[np.zeros(fps), np.linspace(0, L[-1], n_move), np.full(fps, L[-1])]
    pos = np.column_stack([np.interp(s, L, route[:, 0]), np.interp(s, L, route[:, 1])])
    ker = np.ones(41) / 41
    sm = np.column_stack([np.convolve(np.pad(pos[:, j], 20, mode="edge"), ker, mode="valid") for j in range(2)])
    mid = np.array([-5.6, 1.0])
    mp4 = out / "flythrough_gmc.mp4"
    gen = imageio_ffmpeg.write_frames(str(mp4), size, fps=fps, codec="libx264", pix_fmt_in="rgb24",
                                      pix_fmt_out="yuv420p", macro_block_size=1, ffmpeg_log_level="error",
                                      output_params=["-crf", "23", "-preset", "medium"])
    gen.send(None)
    t0 = time.perf_counter()
    for f in range(len(pos)):
        prog = f / (len(pos) - 1)
        phi = math.radians(200 + 0 * prog)                           # camera on the -v side looking +v, fixed azimuth
        th = math.radians(52)
        tgt = np.array([.5 * sm[f, 0] + .5 * mid[0], .5 * sm[f, 1] + .5 * mid[1], .5])
        dist = 7.5
        eye = tgt + dist * np.array([math.sin(phi) * math.cos(th), math.cos(phi) * math.cos(th), math.sin(th)])
        cam = cam_from(S.frame, eye.tolist(), tgt.tolist(), *size, fov=55)
        occ = body_layer(S, cam, [pos[f]])
        o = ewa.render(S.splats, cam, occ=occ, bg=(.97, .97, .97), xray=.25)
        surface = np.minimum(o["depth"], occ[2])
        fig = plt.figure(figsize=(size[0] / 100, size[1] / 100), dpi=100)
        ax = fig.add_axes([0, 0, 1, 1])
        ax.imshow(np.clip(o["rgb"], 0, 1))
        for name, lw, ls in (("straight", 1.6, "--"), ("astar", 1.6, "-"), ("gmc", 2.6, "-")):
            P = np.vstack([C["start"], C["goal"]]) if name == "straight" else densify(C[name])
            overlay(ax, cam, surface, S.world(P, .01), COL[name], lw, ls)
        ax.text(12, 22, f"WWEST-F4X-01018 cylinder: GMC route ({L[-1]:.2f} m) with the 60 s shortening budget; "
                f"position u {pos[f, 0]:5.2f}, v {pos[f, 1]:4.2f}", fontsize=9.5,
                bbox=dict(fc="white", ec="none", alpha=.85))
        ax.text(12, size[1] - 12, "green = GMC route, blue = A* route, red dashed = straight start-goal line, "
                "orange cylinder = robot body (ceiling cut away)", fontsize=8.5,
                bbox=dict(fc="white", ec="none", alpha=.7))
        ax.set_xlim(0, size[0]); ax.set_ylim(size[1], 0); ax.axis("off")
        fig.canvas.draw()
        gen.send(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy().tobytes())
        plt.close(fig)
        if f % 40 == 0:
            print("frame", f, "/", len(pos), round(time.perf_counter() - t0, 1), "s", flush=True)
    gen.close()
    return mp4, L[-1]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--case", default="WWEST-F4X-01018")
    p.add_argument("--out", type=Path, default=Path("results/aerial3dg/f5/render3d"))
    p.add_argument("--skip-video", action="store_true")
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    C = load_case(a.case)
    t0 = time.perf_counter()
    S = Scene()
    print("scene", S.n, "splats in window", round(time.perf_counter() - t0, 1), "s", flush=True)
    L = lambda P: float(np.linalg.norm(np.diff(np.asarray(P), axis=0), axis=1).sum())
    lens = {k: round(L(C[k]), 3) for k in ("astar", "gmc", "raw")}
    lens["straight"] = round(float(np.linalg.norm(np.asarray(C["start"]) - np.asarray(C["goal"]))), 3)
    print("lengths", lens, flush=True)
    start, goal, loc = np.asarray(C["start"]), np.asarray(C["goal"]), np.asarray(C["locus"])
    ttl = f"{a.case} cylinder; lengths: straight {lens['straight']} m, A* {lens['astar']} m, GMC {lens['gmc']} m"
    marks = [("start", start), ("goal", goal)]
    allr = [("straight", 1.8, "--"), ("raw", 1.4, "-"), ("astar", 2.0, "-"), ("gmc", 2.4, "-")]
    shot(S, C, (-5.8, 1.1, 14.0), (-5.8, 1.1, 0.0), (1400, 1100), 38, [start, goal], allr,
         a.out / "overview_top.png", ttl, marks)
    shot(S, C, (-5.8, -6.5, 7.5), (-5.8, 1.0, 0.5), (1400, 800), 50, [start, goal], allr,
         a.out / "overview_oblique.png", ttl, marks)
    shot(S, C, (-5.8, 6.5, 8.0), (-5.8, 1.0, 0.5), (1400, 800), 50, [start, goal], allr,
         a.out / "overview_oblique_from_north.png", ttl, marks)
    shot(S, C, (loc[0] - 1.6, loc[1] - 2.2, 2.4), (loc[0], loc[1], 0.6), (1200, 800), 55, [loc], allr,
         a.out / "locus_closeup.png", ttl + "; body at the A* route's tightest lateral point", marks)
    # straight line: body at 4 evenly spaced points, looking along it from the goal side
    mids = [start + (goal - start) * t for t in (.25, .5, .75)]
    shot(S, C, (goal[0] - 1.5, goal[1] - 2.8, 3.2), (-5.0, 0.4, .6), (1400, 800), 55, mids, allr,
         a.out / "straight_line_blocker.png", ttl + "; bodies placed ON the straight line (25/50/75 %)", marks)
    man = {"case": a.case, "archive_sha256": S.sha, "n_splats_window": S.n, "lengths_m": lens,
           "window_uv": WIN, "z_cut_m": Z_CUT, "routes_uv": {k: np.asarray(C[k]).round(4).tolist()
                                                           for k in ("astar", "gmc", "raw")}}
    if not a.skip_video:
        mp4, glen = flythrough(S, C, a.out)
        man["video"] = {"path": str(mp4), "bytes": mp4.stat().st_size, "gmc_len_m": round(glen, 3)}
    (a.out / "render_manifest.json").write_text(json.dumps(man, indent=1, default=float) + "\n")
    print("done", round(time.perf_counter() - t0, 1), "s", flush=True)


if __name__ == "__main__":
    main()
