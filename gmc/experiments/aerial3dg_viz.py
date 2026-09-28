"""G1 figures + videos for the ground-body demo pair (sweeper, cylinder) on the real uav-lamp archive.

Drawn from the exact planning archive (manifest-checked, via ``uavlamp_viz.Booth``) and the exact
demo JSONs written by ``aerial3dg_run.py demo``; nothing is re-planned.  Reuses the L2/C2 render
machinery (``uavlamp_viz``: Booth, overlay_path; ``gmc.height.ewa``; ``uavlamp_render.cam_from``);
only the body prism is parameterised by robot (the L2 ``draw_frame`` hard-codes the UAV).

Outputs (``--out``):
  routes_<pair>.png           plan views at each robot's own height (free space from the gs3d point map,
                              captured Gaussians in that body band, both routes) + straight-line evidence
  video/<pair>_<robot>_flythrough.mp4 (+ 4 frames read back from each MP4)
  viz_manifest.json
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np

from gmc.gs3d.trajectory import sample_linear_trajectory
from gmc.height import ewa

import uavlamp_viz as uv
from uavlamp_render import cam_from

ROBOT_C = {"sweeper": "#2a78d6", "cylinder": "#eb6834"}
FREE_C = {"sweeper": "#2a78d6", "cylinder": "#eb6834"}
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
LAMP_U = (-1.03, -.67)   # manifest lamp footprint u range (route m)
LABEL_C = {"SAFE": "#1baf7a", "BLOCKED": "#d62728", "UNKNOWN": "#eda100", "NONE": "#a3a29d"}
plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": ["DejaVu Sans"], "font.size": 9,
                     "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
                     "text.color": INK, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
                     "axes.edgecolor": GRID, "axes.spines.top": False, "axes.spines.right": False})


def load_demo(path, frame, pair):
    top = json.loads(Path(path).read_text())
    doc = top["pairs"][pair]
    r = doc["result"]
    # the planning frame is the route prism's own frame: check, since cut leaves are drawn from plan coords
    if not np.allclose(frame.to_route(np.asarray(r["start_world"])), r["start_plan"], atol=1e-9):
        raise ValueError("plan frame differs from the route frame")
    out = {"doc": doc, "top": top, "robot": top["robot"], "status": r["status"], "body": top["body"]}
    if r.get("gs3d_result"):
        smp = sample_linear_trajectory(r["gs3d_result"]["trajectory"], dt_s=.02)
        world = np.asarray(smp["poses"], float)[:, :3]
        out.update(world=world, route=frame.to_route(world), time_s=np.asarray(smp["time_s"]),
                   knots=frame.to_route(np.asarray(r["polyline_world"], float)),
                   length=r["metrics"]["path_length_m"])
    return out


# ----------------------------------------------------------------------------- plan figure
def fig_routes(booth, demos, cands, dst):
    order = ["sweeper", "cylinder"]
    pair = demos["sweeper"]["doc"]
    s_uv, g_uv = np.asarray(pair["start_uv"]), np.asarray(pair["goal_uv"])
    ulo, uhi = min(s_uv[0], g_uv[0]) - 1.2, max(s_uv[0], g_uv[0]) + 1.2
    us, vs = np.asarray(cands["us"]), np.asarray(cands["vs"])
    step = float(cands["step"])
    fig = plt.figure(figsize=(12, 12.5))
    gs = fig.add_gridspec(3, 1, height_ratios=[1, 1, .85], hspace=.32)
    op = booth.opac > .3
    loc = booth.loc[op]
    for row, name in enumerate(order):
        d = demos[name]
        body = d["body"]
        zc = body["ground_clearance_m"] + body["half_height_m"]
        z0, z1 = zc - body["half_height_m"], zc + body["half_height_m"]
        ax = fig.add_subplot(gs[row])
        free = np.asarray(cands["free"][name], bool)
        ax.imshow(np.where(free.T, 1., np.nan), origin="lower", cmap=matplotlib.colors.ListedColormap([FREE_C[name]]),
                  alpha=.13, extent=(us[0] - step / 2, us[-1] + step / 2, vs[0] - step / 2, vs[-1] + step / 2),
                  interpolation="nearest", zorder=0, aspect="auto")
        band = (loc[:, 2] > z0 - .02) & (loc[:, 2] < z1 + .02) & (loc[:, 0] > ulo - .5) & (loc[:, 0] < uhi + .5)
        ax.scatter(loc[band, 0], loc[band, 1], s=.5, c="#5f5e5a", alpha=.35, lw=0, rasterized=True, zorder=1)
        other = demos["cylinder" if name == "sweeper" else "sweeper"]
        if "route" in other:
            ax.plot(other["route"][:, 0], other["route"][:, 1], color=ROBOT_C[other["robot"]], lw=1.3, ls=(0, (4, 3)),
                    alpha=.9, zorder=3, label=f"{other['robot']} route ({other['length']:.2f} m), for comparison")
        if "route" in d:
            rt, kn = d["route"], d["knots"]
            for k in np.linspace(0, len(rt) - 1, 9).astype(int):
                ax.add_patch(Circle(rt[k, :2], body["radius_m"], fc=ROBOT_C[name] + "22", ec=ROBOT_C[name], lw=.8,
                                    zorder=4))
            ax.plot(rt[:, 0], rt[:, 1], color=ROBOT_C[name], lw=2.6, zorder=5, solid_capstyle="round",
                    label=f"{name} route: {d['length']:.2f} m, {len(kn)} vertices (footprint r {body['radius_m']:.3f} m)")
            ax.scatter(kn[:, 0], kn[:, 1], s=24, color=ROBOT_C[name], edgecolors="white", linewidths=1.1, zorder=6)
        else:
            cert = d["doc"]["result"].get("certificate") or {}
            cut = np.asarray(cert.get("cut_leaf_centres_plan") or [], float).reshape(-1, 3)
            if len(cut):
                ax.scatter(cut[:, 0], cut[:, 1], s=4, c="#4a3aa7", alpha=.6, lw=0, zorder=4,
                           label="BLOCKED leaves on the certified cut (each inside one pair's inner polytope)")
            roles = (d["doc"].get("certificate_attribution") or {}).get("cut_pairs_by_role", {})
            ax.text(.02, .06, f"{name}: {d['status']} ({d['doc']['reason']})" +
                    (f"\n{cert.get('blocked_leaves_on_cut')} cut leaves, {cert.get('cut_distinct_pairs')} distinct pairs: " +
                     ", ".join(f"{k} {v}" for k, v in sorted(roles.items())) if cert.get("blocked_leaves_on_cut") else
                     "\nno route and no cut certificate: not certified either way"),
                    transform=ax.transAxes, fontsize=8.5, color="#4a3aa7", zorder=8,
                    bbox=dict(fc="white", ec="#4a3aa7", lw=1))
        ax.plot([s_uv[0], g_uv[0]], [s_uv[1], g_uv[1]], color=INK2, lw=.8, ls=":", zorder=2,
                label=f"straight line, {np.linalg.norm(g_uv - s_uv):.2f} m")
        ax.scatter(*s_uv, s=80, c="#1baf7a", edgecolors="k", zorder=7, label="start")
        ax.scatter(*g_uv, s=130, marker="*", c="#e34948", edgecolors="k", zorder=7, label="goal")
        ax.set_xlim(ulo, uhi)
        ax.set_ylim(-.4, 2.8)
        ax.set_aspect("equal")
        ax.set_xlabel("u (m, along the gallery)")
        ax.set_ylabel("v (m, across)")
        ax.set_title(f"{name} at its own height (body band {z0:.2f}–{z1:.2f} m above the floor)\nshaded = centre positions "
                     f"the gs3d oracle certifies free (0.1 m grid); grey dots = captured Gaussians in that band",
                     fontsize=9.5, loc="left")
        ax.legend(loc="upper right", fontsize=7.5, framealpha=.92, ncol=2)
    ax = fig.add_subplot(gs[2])
    for name in order:
        sl = demos[name]["doc"]["straight_line"]
        rows = sl["rows"]
        s = np.array([r["s_m"] for r in rows])
        clr = np.array([r["oracle_clearance_m"] if r["oracle_occupancy"] == "free" and r["oracle_clearance_m"]
                        is not None else 0. for r in rows])
        ax.plot(s, clr * 100, color=ROBOT_C[name], lw=2, label=f"{name}: gs3d oracle clearance along the straight line")
        lab = [r["leaf_label"] for r in rows]
        y0 = -1.2 if name == "sweeper" else -2.4
        for k in range(len(s) - 1):
            ax.fill_between([s[k], s[k + 1]], y0, y0 + .9, color=LABEL_C[lab[k]], lw=0)
        ax.text(s[-1] + .03, y0 + .45, f"{name}: octree leaf label", fontsize=7.5, va="center", color=INK2)
        roles = sl["blocked_samples_by_role"]
        if roles:
            ax.text(.01, .93 if name == "sweeper" else .80,
                    f"{name}: {sl['label_counts']['BLOCKED']}/{sl['samples']} samples BLOCKED; certifying pairs by role: " +
                    ", ".join(f"{k} {v}" for k, v in sorted(roles.items())),
                    transform=ax.transAxes, fontsize=8, color=INK)
    for k, c in LABEL_C.items():
        ax.fill_between([], [], color=c, label=f"leaf {k}")
    ax.axhline(0, color=GRID, lw=.8)
    ax.set_ylabel("clearance (cm; 0 = occupied)")
    ax.set_xlabel("distance along the straight start→goal segment (m)")
    ax.set_ylim(-2.7, max(3., ax.get_ylim()[1]))
    ax.legend(loc="upper right", fontsize=7.5, ncol=3, framealpha=.92)
    ax.set_title("Why the routes differ: each robot's own certificates along the straight segment\n(octree leaf "
                 "labels; gs3d oracle point clearance at the robot's own z, capped by the oracle's padding)",
                 fontsize=9.5, loc="left")
    fig.suptitle(f"Pair {pair['name']}: start ({s_uv[0]:.2f}, {s_uv[1]:.2f}) → goal ({g_uv[0]:.2f}, {g_uv[1]:.2f}) (route u, v in m; {np.linalg.norm(g_uv - s_uv):.2f} m apart)\nsame archive; one query call per robot on that robot's own compile", fontsize=10.5, x=.01, ha="left")
    fig.savefig(dst, dpi=100, bbox_inches="tight")
    plt.close(fig)


# ----------------------------------------------------------------------------- video
def draw_frame_body(splats, cam, centres, radius, half_height, xray=.25):
    prisms = [(c[0], c[1], radius, c[2] - half_height, c[2] + half_height) for c in centres]
    a = np.linspace(0, 2 * np.pi, 49)
    edges = []
    for c in centres:
        for dz in (-half_height, half_height):
            ring = np.column_stack([c[0] + radius * np.cos(a), c[1] + radius * np.sin(a), np.full_like(a, c[2] + dz)])
            edges += [ring[k:k + 2] for k in range(48)]
    occ = ewa.prism_occluder(cam, prisms, face_rgba=uv.BODY_FACE, edge_rgb=uv.BODY_EDGE, edges=np.asarray(edges),
                             line_width=1.6)
    out = ewa.render(splats, cam, occ=occ, bg=(.97, .97, .97), xray=xray)
    return out, np.minimum(out["depth"], occ[2])


def flythrough(booth, d, other, out_dir, manifest, *, seconds=12., fps=20, size=(960, 540), dist=3.4, elev=32.):
    import imageio_ffmpeg
    name, body = d["robot"], d["body"]
    keep = (booth.loc[:, 1] < 2.22) & (booth.loc[:, 2] < 2.34)
    splats = booth.splats(keep)
    t, route = d["time_s"], d["route"]
    n_move = int(seconds * fps)
    tt = np.r_[np.zeros(fps), np.linspace(0, t[-1], n_move), np.full(fps, t[-1])]
    ks = np.clip(np.searchsorted(t, tt), 0, len(t) - 1)
    ker = np.ones(61) / 61
    sm = np.column_stack([np.convolve(np.pad(route[:, j], 30, mode="edge"), ker, mode="valid") for j in range(3)])
    mid = route.mean(axis=0)

    def on_floor(world):   # draw routes on the floor (1 cm up) so both robots' lines are comparable in plan
        r = booth.frame.to_route(world)
        r[:, 2] = .01
        return booth.frame.to_world(r)

    own_f = on_floor(d["world"])
    other_f = on_floor(other["world"]) if "world" in other else None
    wanted = {"start": 0, "one third": len(ks) // 3, "two thirds": 2 * len(ks) // 3, "at goal": len(ks) - 1}
    tag = d["doc"]["name"]
    mp4 = out_dir / f"{tag}_{name}_flythrough.mp4"
    gen = imageio_ffmpeg.write_frames(str(mp4), size, fps=fps, codec="libx264", pix_fmt_in="rgb24",
                                      pix_fmt_out="yuv420p", macro_block_size=1, ffmpeg_log_level="error",
                                      output_params=["-crf", "23", "-preset", "medium"])
    gen.send(None)
    t0 = time.perf_counter()
    cams = []
    for f, k in enumerate(ks):
        prog = f / (len(ks) - 1)
        phi = math.radians(-35 + 70 * prog)            # camera on the +v (cut-away wall B) side
        th = math.radians(elev)
        tgt = .6 * sm[k] + .4 * mid
        tgt[2] = .35
        eye = tgt + dist * np.array([math.sin(phi) * math.cos(th), math.cos(phi) * math.cos(th), math.sin(th)])
        cam = cam_from(booth.frame, eye.tolist(), tgt.tolist(), *size, fov=60)
        out, surface = draw_frame_body(splats, cam, [d["world"][k]], body["radius_m"], body["half_height_m"])
        fig = plt.figure(figsize=(size[0] / 100, size[1] / 100), dpi=100)
        ax = fig.add_axes([0, 0, 1, 1])
        ax.imshow(np.clip(out["rgb"], 0, 1))
        if other_f is not None:
            uv.overlay_path(ax, cam, surface, other_f, colour=ROBOT_C[other["robot"]], lw=1.2)
        uv.overlay_path(ax, cam, surface, own_f, colour="#9ab8ff", lw=1.6)
        uv.overlay_path(ax, cam, surface, own_f[:k + 1], colour=ROBOT_C[name], lw=2.8)
        ax.text(12, 22, f"{name} (r {body['radius_m']:.3f} m, body {body['ground_clearance_m']:.2f}–"
                f"{body['ground_clearance_m'] + 2 * body['half_height_m']:.2f} m above floor)   one query on the "
                f"cached compile   t = {t[k]:5.1f} s", fontsize=9.5, color="k", bbox=dict(fc="white", ec="none", alpha=.8))
        u_k = route[k, 0]
        under = LAMP_U[0] - body["radius_m"] <= u_k <= LAMP_U[1] + body["radius_m"]
        ax.text(12, 46, f"position u = {u_k:5.2f} m, v = {route[k, 1]:4.2f} m" +
                ("   UNDER THE LAMP (lamp underside 1.10 m, body top "
                 f"{body['ground_clearance_m'] + 2 * body['half_height_m']:.2f} m)" if under else ""),
                fontsize=9.5, color="#5a4500" if under else "k", fontweight="bold" if under else "normal",
                bbox=dict(fc="#fff3c4" if under else "white", ec="none", alpha=.85))
        ax.text(12, size[1] - 14, f"routes drawn on the floor · thick = this robot's route ({d['length']:.2f} m); thin "
                + (f"{other['robot']} = the other robot's route ({other['length']:.2f} m)" if "length" in other else
                   f"{other['robot']}: {other['status']}, no route") + " · wall B and soffit cut away for the view", fontsize=8, color="k", bbox=dict(fc="white", ec="none", alpha=.7))
        ax.set_xlim(0, size[0]); ax.set_ylim(size[1], 0); ax.axis("off")
        fig.canvas.draw()
        img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
        plt.close(fig)
        gen.send(img.tobytes())
        cams.append({"frame": f, "sample": int(k), "eye_route": eye.round(3).tolist(), "target_route": tgt.round(3).tolist()})
    gen.close()
    render_s = time.perf_counter() - t0
    reader = imageio_ffmpeg.read_frames(str(mp4))
    meta = next(reader)
    w, h = meta["size"]
    inv = {v: k for k, v in wanted.items()}
    frames = {}
    for f, raw in enumerate(reader):
        if f in inv:
            img = np.frombuffer(raw, np.uint8).reshape(h, w, 3)
            dst = out_dir / f"{tag}_{name}_frame_{f:03d}_{inv[f].replace(' ', '_')}.png"
            plt.imsave(dst, img)
            frames[inv[f]] = {"path": str(dst), "frame": f, "route_xyz": route[ks[f]].round(4).tolist()}
    reader.close()
    manifest.setdefault("videos", {})[f"{tag}:{name}"] = {
        "path": str(mp4), "bytes": mp4.stat().st_size, "sha256": uv.sha256(mp4), "fps": fps, "n_frames": len(ks),
        "size": list(size), "render_s": render_s, "frames_from_mp4": frames, "camera_path_every_20th": cams[::20],
        "crop": "v < 2.22 (wall B removed) and z < 2.34 (soffit and hall ceiling removed)"}
    print("video", name, mp4.stat().st_size, round(render_s, 1), flush=True)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--demo", type=Path, required=True, help="dir with demo_sweeper.json, demo_cylinder.json")
    p.add_argument("--candidates", type=Path, required=True, help="screen candidates.json (per-robot free maps)")
    p.add_argument("--uavlamp-root", type=Path, default=Path("/scratch/wg2381/splathjb-uavlamp/gmc"))
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--skip", nargs="*", default=[])
    p.add_argument("--video-seconds", type=float, default=12.)
    p.add_argument("--pairs", nargs="+", required=True, help="pair names in the demo JSONs")
    a = p.parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)
    root = a.uavlamp_root / "outputs/uavlamp/scene_v2"
    booth = uv.Booth(root / "uavlamp_scene.npz", root / "manifest.json")
    cands = json.loads(a.candidates.read_text())
    manifest = {"archive_sha256": booth.doc["derivative"]["sha256"],
                "demos": {n: str(a.demo / f"demo_{n}.json") for n in ("sweeper", "cylinder")}}
    for pair in a.pairs:
        demos = {n: load_demo(a.demo / f"demo_{n}.json", booth.frame, pair) for n in ("sweeper", "cylinder")}
        fig_routes(booth, demos, cands, a.out / f"routes_{pair}.png")
        print("routes done", pair, flush=True)
        if "video" not in a.skip:
            vdir = a.out / "video"
            vdir.mkdir(exist_ok=True)
            for n, o in (("sweeper", "cylinder"), ("cylinder", "sweeper")):
                if "world" in demos[n]:
                    flythrough(booth, demos[n], demos[o], vdir, manifest, seconds=a.video_seconds)
    for f in sorted(list(a.out.glob("*")) + list((a.out / "video").glob("*"))):
        if f.suffix in (".png", ".mp4"):
            manifest.setdefault("files", {})[str(f.relative_to(a.out))] = {"bytes": f.stat().st_size,
                                                                          "sha256": uv.sha256(f)}
    (a.out / "viz_manifest.json").write_text(json.dumps(manifest, indent=1, default=float) + "\n")
    print("done", flush=True)


if __name__ == "__main__":
    main()
