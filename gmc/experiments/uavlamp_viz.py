"""L2 visualisations of the single-query UAV routes on the uav-lamp booth archive.

Everything is drawn from the exact planning archive (manifest-checked loader) and the
exact trajectories in the run JSONs; nothing is re-planned here.

Outputs (``--out``):
  keyframes_{side,oblique,top}.png   EWA renders, UAV body (depth-composited cylinder) at 6
                                     poses + path polyline (solid where visible, dashed where
                                     hidden behind splats)
  flythrough.mp4 (+ frames)          chase/orbit over the rendered booth, UAV moving
  altitude_profile.png               z(s) of centre / body top / body bottom, lamp underside
                                     and table top, footprint intervals shaded
  necessity_high_start.png           (a) lamp only (b) designed (c) designed + plug, top + side
  necessity_low_start.png            same for the low start (panel a is L1's run)
  route_3d.html                      plotly: Gaussians near the route + trajectories
  viz_manifest.json                  hashes, cameras, keyframe poses, pixel positions
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

from gmc.gs3d import scene_uavlamp as su
from gmc.gs3d.trajectory import sample_linear_trajectory
from gmc.height import ewa
from gmc.height.viz3d import load_dc_colors

from uavlamp_render import N_PLY, PLY, cam_from
from uavlamp_run import ordered_evidence

R, H, MARGIN = .25, .10, .05
PATH_RGB, BODY_FACE, BODY_EDGE = "#1f77ff", (1.0, .45, .0, .75), (.75, .2, .0)
LAMP_C, TABLE_C, PLUG_C = "#c99a00", "#8b5a2b", "#d62728"


def sha256(path) -> str:
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(8 << 20), b""):
            h.update(block)
    return h.hexdigest()


def load_run(path, frame, spec):
    """Trajectory + status from an L2 run JSON (or an L1 ``{"result", "replay"}`` JSON)."""
    doc = json.loads(Path(path).read_text())
    res = doc["result"]
    out = {"path": str(path), "status": res["status"], "reason": res["reason"],
           "expansions": res["diagnostics"]["expansions"],
           "algorithm_wall_s": res["timings"]["algorithm_wall_s"],
           "replay_passed": (doc.get("replay") or {}).get("passed"), "traj": res.get("trajectory")}
    if out["traj"]:
        smp = sample_linear_trajectory(out["traj"], dt_s=.02)
        world = np.asarray(smp["poses"], float)[:, :3]
        out["world"], out["route"], out["time_s"] = world, frame.to_route(world), np.asarray(smp["time_s"])
        out["evidence"] = ordered_evidence(out["route"], lamp=spec["lamp"], table=spec["table"],
                                           radius=R, half_height=H, margin=MARGIN)
        out["s"] = np.r_[0., np.cumsum(np.linalg.norm(np.diff(out["route"], axis=0), axis=1))]
    return out


# ----------------------------------------------------------------------------- EWA scene

class Booth:
    def __init__(self, archive, manifest):
        full, self.doc = su.load_uavlamp_derivative(archive, manifest)
        self.frame = su.Frame.from_dict(self.doc["frame"])
        loc = self.frame.to_route(full.means)
        region = ((loc[:, 0] > -4.6) & (loc[:, 0] < 3.9) & (loc[:, 1] > -.7) & (loc[:, 1] < 3.0)
                  & (loc[:, 2] > -.15) & (loc[:, 2] < 3.2) & (full.opacity > .05))
        self.full = full
        self.ids, self.loc = full.ids[region], loc[region]
        self.means, self.covs, self.opac = full.means[region], full.covs[region], full.opacity[region]
        rgb = np.full((len(self.ids), 3), .62)
        real = self.ids < N_PLY
        rgb[real] = load_dc_colors(PLY, self.ids[real])
        self.role = np.full(len(self.ids), "", dtype=object)
        for e in self.doc["edits"]:
            m = (self.ids >= e["id_range"][0]) & (self.ids <= e["id_range"][1])
            rgb[m] = e["color_rgb"]
            self.role[m] = e["role"]
        self.rgb = rgb

    def splats(self, keep):
        return ewa.pack(self.means[keep], self.covs[keep], self.opac[keep], self.rgb[keep])


def body_edges(centre, n=48):
    a = np.linspace(0, 2 * np.pi, n + 1)
    segs = []
    for dz in (-H, H):
        ring = np.column_stack([centre[0] + R * np.cos(a), centre[1] + R * np.sin(a), np.full_like(a, centre[2] + dz)])
        segs += [ring[k:k + 2] for k in range(n)]
    return segs


def draw_frame(booth, splats, cam, bodies, path_world, done_upto=None, xray=.25, size=(1400, 800)):
    """Render splats with UAV bodies composited in depth order; return (rgb, surface depth)."""
    prisms = [(c[0], c[1], R, c[2] - H, c[2] + H) for c in bodies]
    edges = [s for c in bodies for s in body_edges(c)]
    occ = ewa.prism_occluder(cam, prisms, face_rgba=BODY_FACE, edge_rgb=BODY_EDGE,
                             edges=np.asarray(edges) if edges else (), line_width=1.6) if bodies else None
    out = ewa.render(splats, cam, occ=occ, bg=(.97, .97, .97), xray=xray)
    surface = out["depth"] if occ is None else np.minimum(out["depth"], occ[2])
    return out, surface


def overlay_path(ax, cam, surface, pts, colour=PATH_RGB, lw=2.4, label=None, slack=.06):
    u, v, d = ewa.project_points(cam, pts)
    front = d > cam.near
    iu = np.clip(np.nan_to_num(u), 0, cam.width - 1).astype(int)
    iv = np.clip(np.nan_to_num(v), 0, cam.height - 1).astype(int)
    on = front & (u >= 0) & (u < cam.width) & (v >= 0) & (v < cam.height)
    vis = on & (d <= surface[iv, iu] + slack)
    ax.plot(np.where(front, u, np.nan), np.where(front, v, np.nan), color=colour, lw=lw * .6, ls=(0, (3, 2)),
            alpha=.75, zorder=3)
    ax.plot(np.where(vis, u, np.nan), np.where(vis, v, np.nan), color=colour, lw=lw, zorder=4, label=label)
    return float(vis[on].mean()) if on.any() else 0.


def keyframe_indices(run):
    ev, s = run["evidence"], run["s"]
    under, above = ev["under_lamp_intervals"][0], ev["above_table_intervals"][0]
    targets = [0., .5 * under["s_from_m"], (under["s_from_m"] + under["s_to_m"]) / 2,
               (under["s_to_m"] + above["s_from_m"]) / 2, above["s_from_m"] + .4 * (s[-1] - above["s_from_m"]), s[-1]]
    labels = ["start", "approach", "under the lamp", "climbing", "over the table", "goal"]
    return [int(np.argmin(np.abs(s - t))) for t in targets], labels


VIEWS = {
    # camera beyond (cropped) wall B looking at -v: lamp underside, table top, path heights together
    "side": dict(eye=[-.35, 5.3, 1.2], target=[-.35, 1.0, 1.0], crop=lambda l: l[:, 1] < 2.22, fov=50),
    # high oblique from the far (table) end looking back at the lamp; wall B and soffit cut away
    "oblique": dict(eye=[3.1, 4.3, 3.1], target=[-.7, .9, .8], crop=lambda l: (l[:, 1] < 2.22) & (l[:, 2] < 2.34),
                    fov=55),
    # top-down, soffit and hall ceiling cut away; lamp drawn over the UAV with x-ray
    "top": dict(eye=[-.55, .95, 8.2], target=[-.55, 1.0, 0.], crop=lambda l: l[:, 2] < 2.34, fov=48),
}


def keyframes(booth, run, spec, out_dir, manifest):
    idx, labels = keyframe_indices(run)
    bodies = [run["world"][k] for k in idx]
    for name, v in VIEWS.items():
        keep = v["crop"](booth.loc)
        cam = cam_from(booth.frame, v["eye"], v["target"], 1400, 800, fov=v["fov"])
        out, surface = draw_frame(booth, booth.splats(keep), cam, bodies, run["world"])
        fig, ax = plt.subplots(figsize=(14, 8))
        ax.imshow(np.clip(out["rgb"], 0, 1))
        frac = overlay_path(ax, cam, surface, run["world"], label="planned route (UAV centre)")
        px = []
        for j, (k, lab, c) in enumerate(zip(idx, labels, bodies)):
            u, vv, d = ewa.project_points(cam, c[None])
            px.append({"label": lab, "sample": k, "route_xyz": run["route"][k].round(4).tolist(),
                       "pixel_uv": [float(u[0]), float(vv[0])], "depth_m": float(d[0])})
            ax.annotate(f"{lab}\nz={run['route'][k][2]:.2f} m", (u[0], vv[0]), xytext=(0, (-40 if j % 2 == 0 else 34) * (1 if name != "top" else -1)),
                        textcoords="offset points", ha="center", fontsize=8,
                        bbox=dict(boxstyle="round,pad=.2", fc="white", ec="none", alpha=.8), zorder=8)
        if name == "side":
            lamp, table = spec["lamp"], spec["table"]
            for (pt, txt) in (([-.85, 2.2, lamp["underside_z"]], f"lamp underside {lamp['underside_z']:.2f} m"),
                              ([2.6, .4, table["top_z"]], f"table top {table['top_z']:.2f} m")):
                u, vv, _ = ewa.project_points(cam, booth.frame.to_world(pt)[None])
                ax.annotate(txt, (u[0], vv[0]), xytext=(18, 22), textcoords="offset points", fontsize=9,
                            arrowprops=dict(arrowstyle="-", color="k", lw=.8),
                            bbox=dict(boxstyle="round,pad=.2", fc="#fff8dc", ec="k", lw=.5), zorder=8)
        ev = run["evidence"]
        ax.set_title(f"{spec['name']} — EWA render of the planning archive — {name} view\n"
                     f"one planner call (start + goal only); orange = UAV body (r .25, h .20) at 6 poses of the "
                     f"returned trajectory; z range {ev['z_range'][0]:.2f}–{ev['z_range'][1]:.2f} m",
                     fontsize=10)
        ax.set_xlim(0, 1400); ax.set_ylim(800, 0); ax.axis("off")
        ax.legend(loc="lower right", fontsize=8)
        fig.tight_layout()
        dst = out_dir / f"keyframes_{name}.png"
        fig.savefig(dst, dpi=100); plt.close(fig)
        manifest["keyframes"][name] = {"path": str(dst), "camera": cam.as_dict(), "eye_route": v["eye"],
                                       "target_route": v["target"], "n_splats": int(keep.sum()),
                                       "path_visible_fraction": frac, "bodies": px}
        print("keyframes", name, frac, flush=True)


# ----------------------------------------------------------------------------- video

def flythrough(booth, run, spec, out_dir, manifest, seconds=15., fps=20, size=(960, 540)):
    import imageio_ffmpeg
    keep = (booth.loc[:, 1] < 2.22) & (booth.loc[:, 2] < 2.34)
    splats = booth.splats(keep)
    t = run["time_s"]
    n_move = int(seconds * fps)
    hold = fps
    tt = np.r_[np.zeros(hold), np.linspace(0, t[-1], n_move), np.full(hold, t[-1])]
    ks = np.clip(np.searchsorted(t, tt), 0, len(t) - 1)
    booth_c = np.array([-.4, 1.0, 1.05])
    route = run["route"]
    # smooth the target so the chase camera does not jitter on lattice corners
    ker = np.ones(61) / 61
    sm = np.column_stack([np.convolve(np.pad(route[:, j], 30, mode="edge"), ker, mode="valid") for j in range(3)])
    idx, labels = keyframe_indices(run)
    key = dict(zip(labels, idx))
    under_mid, climb = key["under the lamp"], key["climbing"]
    wanted = {"start": 0, "under the lamp": int(np.argmin(np.abs(ks - under_mid))),
              "climbing": int(np.argmin(np.abs(ks - climb))), "at goal": len(ks) - 1}
    mp4 = out_dir / "flythrough.mp4"
    gen = imageio_ffmpeg.write_frames(str(mp4), size, fps=fps, codec="libx264", pix_fmt_in="rgb24",
                                      pix_fmt_out="yuv420p", macro_block_size=1, ffmpeg_log_level="error",
                                      output_params=["-crf", "23", "-preset", "medium"])
    gen.send(None)
    t0 = time.perf_counter()
    cams = []
    for f, k in enumerate(ks):
        prog = f / (len(ks) - 1)
        phi = math.radians(-50 + 80 * prog)          # from behind the start round to the booth side
        th = math.radians(16 + 10 * prog)
        tgt = .55 * sm[k] + .45 * booth_c
        d = 4.6
        eye = tgt + d * np.array([math.sin(phi) * math.cos(th), math.cos(phi) * math.cos(th), math.sin(th)])
        cam = cam_from(booth.frame, eye.tolist(), tgt.tolist(), *size, fov=55)
        out, surface = draw_frame(booth, splats, cam, [run["world"][k]], run["world"])
        fig = plt.figure(figsize=(size[0] / 100, size[1] / 100), dpi=100)
        ax = fig.add_axes([0, 0, 1, 1])
        ax.imshow(np.clip(out["rgb"], 0, 1))
        overlay_path(ax, cam, surface, run["world"], colour="#9ab8ff", lw=1.6)
        overlay_path(ax, cam, surface, run["world"][:k + 1], lw=2.6)
        z = route[k, 2]
        ax.text(12, 22, f"single planner query (start + goal only)   t = {t[k]:5.1f} s   UAV centre z = {z:.2f} m",
                fontsize=10, color="k", bbox=dict(fc="white", ec="none", alpha=.8))
        ax.text(12, size[1] - 14, f"lamp underside {spec['lamp']['underside_z']:.2f} m · table top "
                f"{spec['table']['top_z']:.2f} m · wall B and soffit cut away for the view", fontsize=8,
                color="k", bbox=dict(fc="white", ec="none", alpha=.7))
        ax.set_xlim(0, size[0]); ax.set_ylim(size[1], 0); ax.axis("off")
        fig.canvas.draw()
        img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
        plt.close(fig)
        gen.send(img.tobytes())
        cams.append({"frame": f, "sample": int(k), "eye_route": eye.round(3).tolist(), "target_route": tgt.round(3).tolist()})
    gen.close()
    render_s = time.perf_counter() - t0
    # Read the frames back *from the encoded MP4* so the report shows what the video contains.
    reader = imageio_ffmpeg.read_frames(str(mp4))
    meta = next(reader)
    w, h = meta["size"]
    inv = {v: k for k, v in wanted.items()}
    frames = {}
    for f, raw in enumerate(reader):
        if f in inv:
            img = np.frombuffer(raw, np.uint8).reshape(h, w, 3)
            dst = out_dir / f"flythrough_frame_{f:03d}_{inv[f].replace(' ', '_')}.png"
            plt.imsave(dst, img)
            frames[inv[f]] = {"path": str(dst), "frame": f, "route_xyz": route[ks[f]].round(4).tolist()}
    reader.close()
    manifest["video"] = {"path": str(mp4), "bytes": mp4.stat().st_size, "sha256": sha256(mp4), "fps": fps,
                         "n_frames": len(ks), "size": list(size), "render_s": render_s,
                         "frames_from_mp4": frames, "camera_path_every_20th": cams[::20],
                         "crop": "v < 2.22 (wall B removed) and z < 2.34 (soffit and hall ceiling removed)"}
    print("video", manifest["video"]["bytes"], render_s, flush=True)


# ----------------------------------------------------------------------------- 2-D figures

def altitude_profile(runs, spec, dst):
    fig, axes = plt.subplots(len(runs), 1, figsize=(11, 3.6 * len(runs)), squeeze=False)
    lamp, table = spec["lamp"], spec["table"]
    for ax, (title, run) in zip(axes[:, 0], runs):
        s, z, ev = run["s"], run["route"][:, 2], run["evidence"]
        for iv in ev["under_lamp_intervals"]:
            ax.axvspan(iv["s_from_m"], iv["s_to_m"], color=LAMP_C, alpha=.18, lw=0)
        lamp_ov = [i for i in ev["under_lamp_centre_in_footprint_intervals"]]
        for iv in lamp_ov:
            ax.axvspan(iv["s_from_m"], iv["s_to_m"], color=LAMP_C, alpha=.22, lw=0)
        for iv in ev["above_table_intervals"]:
            ax.axvspan(iv["s_from_m"], iv["s_to_m"], color=TABLE_C, alpha=.14, lw=0)
        ax.fill_between(s, z - H, z + H, color=PATH_RGB, alpha=.18, lw=0)
        ax.plot(s, z + H, color=PATH_RGB, lw=1, alpha=.7)
        ax.plot(s, z - H, color=PATH_RGB, lw=1, alpha=.7)
        ax.plot(s, z, color=PATH_RGB, lw=2)
        ax.axhline(lamp["underside_z"], color=LAMP_C, lw=2)
        ax.axhline(lamp["underside_z"] - MARGIN, color=LAMP_C, lw=1, ls=":")
        ax.axhline(table["top_z"], color=TABLE_C, lw=2)
        ax.axhline(table["top_z"] + MARGIN, color=TABLE_C, lw=1, ls=":")
        x1 = s[-1]
        ax.text(x1 * .995, lamp["underside_z"] + .03, f"lamp underside {lamp['underside_z']:.2f} m (dotted: −margin)",
                ha="right", fontsize=8, color="#5a4500")
        ax.text(x1 * .995, table["top_z"] - .09, f"table top {table['top_z']:.2f} m (dotted: +margin)",
                ha="right", fontsize=8, color=TABLE_C)
        u = ev["under_lamp_intervals"][0] if ev["under_lamp_intervals"] else None
        a = ev["above_table_intervals"][0] if ev["above_table_intervals"] else None
        if u:
            ax.text((u["s_from_m"] + u["s_to_m"]) / 2, .1, "body overlaps lamp\nfootprint, below it",
                    ha="center", fontsize=8, color="#5a4500")
        if a:
            ax.text((a["s_from_m"] + a["s_to_m"]) / 2, .1, "body overlaps table\nfootprint, above it",
                    ha="center", fontsize=8, color=TABLE_C)
        ax.set_xlim(0, x1); ax.set_ylim(0, 2.0)
        ax.set_xlabel("arc length s along the returned trajectory (m)")
        ax.set_ylabel("height above floor (m)")
        ax.grid(alpha=.25, lw=.5)
        ax.set_title(f"{title}: centre z (line) and body bottom/top (band); under-lamp "
                     f"{u['s_from_m']:.2f}–{u['s_to_m']:.2f} m precedes above-table "
                     f"{a['s_from_m']:.2f}–{a['s_to_m']:.2f} m" if (u and a) else title, fontsize=9)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    fig.tight_layout()
    fig.savefig(dst, dpi=120); plt.close(fig)


def _scene_points(booth, spec, frame):
    """Opaque centres for the 2-D panels, honouring the spec's scene variant."""
    op = booth.opac > .3
    loc, role = booth.loc[op], booth.role[op]
    keep = ~np.isin(role, spec.get("drop_roles", []))
    pts = {"loc": loc[keep], "role": role[keep]}
    plug = None
    for item in spec.get("extra_builders", []):
        g = su.build_test_only(item, frame)
        plug = frame.to_route(g.means)
    return pts, plug


def necessity(booth, panels, dst, suptitle):
    fig, axes = plt.subplots(2, 3, figsize=(19, 8.2))
    rng = np.random.default_rng(0)
    for col, (title, spec, run) in enumerate(panels):
        pts, plug = _scene_points(booth, spec, booth.frame)
        loc, role = pts["loc"], pts["role"]
        box = spec["box_route"]
        lo, hi = np.asarray(box["lower"]), np.asarray(box["upper"])
        inbox = np.all((loc >= lo - .4) & (loc <= hi + .4), axis=1)
        for row, (a, b, name) in enumerate(((0, 1, "top"), (0, 2, "side"))):
            ax = axes[row, col]
            if name == "top":
                sel = inbox & (loc[:, 2] > .05) & (loc[:, 2] < 2.34)       # floor and soffit omitted in plan
            else:
                sel = inbox
            src = sel & (role == "")
            pick = np.flatnonzero(src)
            if len(pick) > 120_000:
                pick = rng.choice(pick, 120_000, replace=False)
            ax.scatter(loc[pick, a], loc[pick, b], s=.4, c="#9a9a9a", alpha=.35, lw=0, rasterized=True,
                       label="captured Gaussians (centres)")
            for r, c in (("lamp", LAMP_C), ("bulkhead", "#2ca02c"), ("drop_ceiling", "#9467bd"),
                         ("back_panel", "#1f77b4")):
                m = sel & (role == r)
                if m.any():
                    ax.scatter(loc[m, a], loc[m, b], s=2, c=c, lw=0, rasterized=True, label=f"added: {r}")
            if plug is not None:
                ax.scatter(plug[:, a], plug[:, b], s=2, c=PLUG_C, lw=0, label="test-only plug under the lamp")
            (tu0, tv0), (tu1, tv1) = spec["table"]["top_route_uv"]
            if name == "top":
                ax.add_patch(Rectangle((tu0, tv0), tu1 - tu0, tv1 - tv0, fill=False, ec=TABLE_C, lw=1.5, ls="--",
                                       label="table top"))
            else:
                ax.plot([tu0, tu1], [spec["table"]["top_z"]] * 2, color=TABLE_C, lw=2, label="table top")
            ax.add_patch(Rectangle((lo[a], lo[b]), hi[a] - lo[a], hi[b] - lo[b], fill=False, ec="k", ls=":", lw=.8,
                                   label="planning box"))
            if run.get("traj"):
                rt = run["route"]
                ax.plot(rt[:, a], rt[:, b], color=PATH_RGB, lw=2.4, label="returned route (centre)")
                if name == "side":
                    ax.fill_between(rt[:, 0], rt[:, 2] - H, rt[:, 2] + H, color=PATH_RGB, alpha=.2, lw=0)
            else:
                ax.text(.5, .93 if name == "top" else .5,
                        f"NO PATH — {run['status']} / {run['reason']}\n"
                        f"queue emptied after {run['expansions']} expansions ({run['algorithm_wall_s']/3600:.1f} h)",
                        transform=ax.transAxes, ha="center", va="top", fontsize=9, color=PLUG_C,
                        bbox=dict(fc="white", ec=PLUG_C, lw=1))
            st, gl = np.asarray(spec["start_route"]), np.asarray(spec["goal_route"])
            ax.scatter([st[a]], [st[b]], s=70, c="#2ca02c", edgecolors="k", zorder=6, label="start")
            ax.scatter([gl[a]], [gl[b]], s=90, marker="*", c=PLUG_C, edgecolors="k", zorder=6, label="goal")
            ax.set_xlim(lo[0] - .3, hi[0] + .3)
            ax.set_ylim((lo[1] - .3, hi[1] + .3) if name == "top" else (-.1, 2.6))
            ax.set_aspect("equal")
            ax.set_xlabel("u (m, along the gallery)")
            ax.set_ylabel("v (m, across the gallery)" if name == "top" else "z above floor (m)")
            if row == 0:
                ev = run.get("evidence")
                verdict = ("goes UNDER the lamp" if ev and ev["passes_under_lamp"] else
                           "goes OVER the lamp" if ev and ev["over_lamp_intervals"] else
                           "no path" if not run.get("traj") else "other")
                ax.set_title(f"{title}\n{run['status']} — {verdict}", fontsize=10)
            if row == 1 and col == 2:
                h, l = [], []
                for axx in axes.ravel():
                    for hh, ll in zip(*axx.get_legend_handles_labels()):
                        if ll not in l:
                            h.append(hh); l.append(ll)
                leg = fig.legend(h, l, loc="lower center", ncol=7, fontsize=8)
                for hh in leg.legend_handles:
                    if hasattr(hh, "set_sizes"):
                        hh.set_sizes([30])
    fig.suptitle(suptitle, fontsize=11)
    fig.tight_layout(rect=(0, .05, 1, .95), w_pad=2.5)
    fig.savefig(dst, dpi=100); plt.close(fig)


def interactive(booth, runs, spec, dst):
    import plotly.graph_objects as go
    fig = go.Figure()
    keep = (booth.opac > .3) & (booth.loc[:, 1] < 2.3) & (booth.loc[:, 2] < 2.34) & (booth.loc[:, 2] > .02) \
        & (booth.loc[:, 0] > -3.2) & (booth.loc[:, 0] < 3.6) & (booth.loc[:, 1] > -.3)
    idx = np.flatnonzero(keep)
    idx = np.random.default_rng(0).choice(idx, min(40_000, len(idx)), replace=False)
    loc, rgb = booth.loc[idx], (np.clip(booth.rgb[idx], 0, 1) * 255).astype(int)
    fig.add_trace(go.Scatter3d(x=loc[:, 0], y=loc[:, 1], z=loc[:, 2], mode="markers", name="Gaussian centres (subsample)",
                               marker=dict(size=1.6, color=[f"rgb({r},{g},{b})" for r, g, b in rgb], opacity=.55),
                               hoverinfo="skip"))
    for (name, run), c in zip(runs, (PATH_RGB, "#e377c2")):
        rt = run["route"][::5]
        fig.add_trace(go.Scatter3d(x=rt[:, 0], y=rt[:, 1], z=rt[:, 2], mode="lines", name=name,
                                   line=dict(width=7, color=c)))
    fig.update_layout(title="uav-lamp booth: returned single-query routes (site frame u, v, z above floor; m)",
                      scene=dict(aspectmode="data", xaxis_title="u (m)", yaxis_title="v (m)", zaxis_title="z (m)"))
    fig.write_html(dst, include_plotlyjs="cdn")


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--configs", type=Path, default=Path("configs/uavlamp_l2"))
    p.add_argument("--runs", type=Path, default=Path("outputs/uavlamp/l2"))
    p.add_argument("--l1-c4", type=Path, default=Path("outputs/uavlamp/t3_v2/C4_counterfactual_lamp_only_low/result.json"))
    p.add_argument("--out", type=Path, default=Path("results/uavlamp/final"))
    p.add_argument("--video-seconds", type=float, default=15.)
    p.add_argument("--skip", nargs="*", default=[])
    a = p.parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)
    specs = {n: json.loads((a.configs / f"{n}.json").read_text()) for n in
             ("M1_main_low_start", "M5_high_start", "N2_plug_low_start", "N2h_plug_high_start", "C4h_lamp_only_high_start")}
    main_spec = specs["M1_main_low_start"]
    booth = Booth(main_spec["archive"], main_spec["manifest"])
    runs = {n: load_run(a.runs / n / "result.json", booth.frame, s) for n, s in specs.items()}
    runs["C4_L1"] = load_run(a.l1_c4, booth.frame, main_spec)
    manifest = {"archive_sha256": booth.doc["derivative"]["sha256"],
                "runs": {n: {"path": r["path"], "sha256": sha256(r["path"]), "status": r["status"]}
                         for n, r in runs.items()},
                "keyframes": {}}
    m1, m5 = runs["M1_main_low_start"], runs["M5_high_start"]
    if "keyframes" not in a.skip:
        keyframes(booth, m1, main_spec, a.out, manifest)
    altitude_profile([("M1 low start (main query)", m1), ("M5 high start", m5)], main_spec, a.out / "altitude_profile.png")
    same = lambda k: specs[k]
    necessity(booth, [("(a) lamp only (bulkhead, soffit, back panel removed)", same("C4h_lamp_only_high_start"),
                       runs["C4h_lamp_only_high_start"]),
                      ("(b) designed booth", same("M5_high_start"), m5),
                      ("(c) designed booth + test-only plug under the lamp", same("N2h_plug_high_start"),
                       runs["N2h_plug_high_start"])],
              a.out / "necessity_high_start.png",
              "Same start (−2.0, 1.2, 1.55) and goal (0.80, 0.55, 1.50) in all three panels; one planner call each")
    c4_spec = {**main_spec, "drop_roles": specs["C4h_lamp_only_high_start"]["drop_roles"]}
    necessity(booth, [("(a) lamp only — L1 run (job 18329386)", c4_spec, runs["C4_L1"]),
                      ("(b) designed booth", main_spec, m1),
                      ("(c) designed booth + test-only plug under the lamp", same("N2_plug_low_start"),
                       runs["N2_plug_low_start"])],
              a.out / "necessity_low_start.png",
              "Same start (−2.0, 1.2, 0.55) and goal (0.80, 0.55, 1.50) in all three panels; one planner call each")
    interactive(booth, [("M1 low start", m1), ("M5 high start", m5)], main_spec, a.out / "route_3d.html")
    if "video" not in a.skip:
        flythrough(booth, m1, main_spec, a.out, manifest, seconds=a.video_seconds)
    for f in sorted(a.out.glob("*")):
        if f.suffix in (".png", ".html", ".mp4"):
            manifest.setdefault("files", {})[f.name] = {"bytes": f.stat().st_size, "sha256": sha256(f)}
    (a.out / "viz_manifest.json").write_text(json.dumps(manifest, indent=1, default=float) + "\n")
    print("done", flush=True)


if __name__ == "__main__":
    main()
