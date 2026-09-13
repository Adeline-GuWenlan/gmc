"""Videos and 3D exports that show whether and how a robot got through (spec §6)."""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
from matplotlib.patches import Ellipse, Rectangle

from .pathio import curve_from_dict, polyline_xy, sample_curve


def save_animation(fig, update, n_frames, out_stem, fps=20):
    out_stem = Path(out_stem)
    out_stem.parent.mkdir(parents=True, exist_ok=True)
    anim = animation.FuncAnimation(fig, update, frames=n_frames, blit=False)
    try:
        import imageio_ffmpeg
        plt.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
        path = out_stem.with_suffix(".mp4")
        anim.save(path, writer=animation.FFMpegWriter(fps=fps, bitrate=2400))
    except ImportError:
        path = out_stem.with_suffix(".gif")
        anim.save(path, writer=animation.PillowWriter(fps=fps))
    return path


def _support_patch(sup, **kw):
    shape = sup.level ** 2 * np.asarray(sup.covariance)
    w, v = np.linalg.eigh(shape)
    ang = np.degrees(np.arctan2(v[1, 1], v[0, 1]))
    return Ellipse(sup.mean, 2 * np.sqrt(w[1]), 2 * np.sqrt(w[0]), angle=ang, **kw)


def _footprint_patches(robot, pose, **kw):
    x, y, th = pose
    c, s = np.cos(th), np.sin(th)
    R = np.array([[c, -s], [s, c]])
    out = []
    for sup in robot.footprint.supports:
        shape = R @ (sup.level ** 2 * np.asarray(sup.covariance)) @ R.T
        w, v = np.linalg.eigh(shape)
        ang = np.degrees(np.arctan2(v[1, 1], v[0, 1]))
        out.append(Ellipse(np.array([x, y]) + R @ np.asarray(sup.mean),
                           2 * np.sqrt(w[1]), 2 * np.sqrt(w[0]), angle=ang, **kw))
    return out


def corridor_profile(scene3d, poly_xy, half_width, z_floor, tau=0.3):
    from shapely.geometry import LineString, Point
    sub = scene3d.subset(scene3d.opacity > tau)
    line = LineString(poly_xy)
    lo = np.min(poly_xy, axis=0) - half_width
    hi = np.max(poly_xy, axis=0) + half_width
    cand = np.flatnonzero(np.all((sub.means[:, :2] >= lo) & (sub.means[:, :2] <= hi), axis=1))
    s, z, w = [], [], []
    for i in cand:
        p = Point(sub.means[i, :2])
        if line.distance(p) <= half_width:
            s.append(line.project(p))
            z.append(sub.means[i, 2] - z_floor)
            w.append(sub.opacity[i])
    return np.asarray(s), np.asarray(z), np.asarray(w)


def _density(ax, scene3d, extent, tau):
    sub = scene3d.subset(scene3d.opacity > tau)
    H, _, _ = np.histogram2d(sub.means[:, 0], sub.means[:, 1], bins=400,
                             range=[[extent[0], extent[2]], [extent[1], extent[3]]],
                             weights=sub.opacity)
    ax.imshow(np.log1p(H.T), origin="lower", cmap="Greys",
              extent=[extent[0], extent[2], extent[1], extent[3]])


def _frame_poses(result, robot, max_frames):
    curve = curve_from_dict(result["curve"])
    poses = sample_curve(curve, 0.02, robot.max_radius())
    idx = np.unique(np.linspace(0, len(poses) - 1, min(max_frames, len(poses))).astype(int))
    return curve, poses[idx]


def robot_video(scene2d, robot, result, out_stem, *, scene3d=None, z_floor=0.0,
                title="", max_frames=300, tau=0.3, key_fracs=(0.0, 0.5, 1.0)):
    out_stem = Path(out_stem)
    out_stem.parent.mkdir(parents=True, exist_ok=True)
    ext = scene2d.workspace.bounds
    side = scene3d is not None and not robot.is_2d
    fig, axes = plt.subplots(1, 2 if side else 1, figsize=(13 if side else 7, 6),
                             squeeze=False)
    ax = axes[0, 0]
    if scene3d is not None:
        _density(ax, scene3d, ext, tau)
    for sup in scene2d.supports:
        ax.add_patch(_support_patch(sup, fc=(0.85, 0.2, 0.2, 0.25), ec="none"))
    ax.set_xlim(ext[0], ext[2]); ax.set_ylim(ext[1], ext[3]); ax.set_aspect("equal")
    head = f"{title}  {robot.name}: {result['status']}"
    if result.get("curve") is None:
        ax.set_title(head)
        png = out_stem.with_name(out_stem.name + "_status.png")
        fig.savefig(png, dpi=110); plt.close(fig)
        return {"video": None, "frames": [png]}
    curve, poses = _frame_poses(result, robot, max_frames)
    xy = polyline_xy(curve)
    ax.plot(xy[:, 0], xy[:, 1], "b--", lw=1)
    trail, = ax.plot([], [], "b-", lw=2)
    feet = []
    rep3 = result.get("replay3d") or {}
    ax.set_title(head + (f"  replay3d={rep3.get('passed')} lb={rep3.get('min_clearance_lb')}"
                         if rep3 else ""))
    if side:
        axs = axes[0, 1]
        s, z, w = corridor_profile(scene3d, xy, robot.max_radius(), z_floor, tau)
        if len(s):  # matplotlib rejects an empty per-point alpha array
            axs.scatter(s, z, s=2, c="k", alpha=np.clip(w, 0.05, 0.6))
        z_lo, z_hi = robot.z_lo, robot.z_hi
        band = Rectangle((0, z_lo), 2 * robot.max_radius(), z_hi - z_lo,
                         fc=(0.1, 0.4, 0.9, 0.5), ec="b")
        axs.add_patch(band)
        L = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
        axs.set_xlim(-0.5, L + 0.5); axs.set_ylim(0, max(2.2, z_hi + 0.3))
        axs.set_xlabel("distance along path (m)"); axs.set_ylabel("height above floor (m)")
        cum = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1))])

    def update(k):
        for p in feet:
            p.remove()
        feet.clear()
        for p in _footprint_patches(robot, poses[k], fc=(0.1, 0.4, 0.9, 0.6), ec="b"):
            ax.add_patch(p); feet.append(p)
        trail.set_data(poses[:k + 1, 0], poses[:k + 1, 1])
        if side:
            band.set_x(cum[k] - robot.max_radius())
        return feet

    frames = []
    for f in key_fracs:
        k = int(round(f * (len(poses) - 1)))
        update(k)
        png = out_stem.with_name(f"{out_stem.name}_f{int(100 * f):03d}.png")
        fig.savefig(png, dpi=110)
        frames.append(png)
    video = save_animation(fig, update, len(poses), out_stem)
    plt.close(fig)
    return {"video": video, "frames": frames}


def comparison_video(runs, out_stem, *, scene3d, z_floor, max_frames=300):
    fig, axes = plt.subplots(1, len(runs), figsize=(6 * len(runs), 6), squeeze=False)
    states = []
    for ax, run in zip(axes[0], runs):
        ext = run["scene2d"].workspace.bounds
        _density(ax, scene3d, ext, 0.3)
        for sup in run["scene2d"].supports:
            ax.add_patch(_support_patch(sup, fc=(0.85, 0.2, 0.2, 0.25), ec="none"))
        ax.set_xlim(ext[0], ext[2]); ax.set_ylim(ext[1], ext[3]); ax.set_aspect("equal")
        ax.set_title(f"{run['label']}: {run['result']['status']}")
        poses = None
        if run["result"].get("curve") is not None:
            _, poses = _frame_poses(run["result"], run["robot"], max_frames)
        states.append({"ax": ax, "robot": run["robot"], "poses": poses, "feet": []})

    def update(k):
        t = k / max(1, max_frames - 1)
        for st in states:
            for p in st["feet"]:
                p.remove()
            st["feet"].clear()
            if st["poses"] is None:
                continue
            j = int(round(t * (len(st["poses"]) - 1)))
            for p in _footprint_patches(st["robot"], st["poses"][j], fc=(0.1, 0.4, 0.9, 0.6), ec="b"):
                st["ax"].add_patch(p); st["feet"].append(p)
        return []

    update(max_frames // 2)
    png = Path(out_stem).with_name(Path(out_stem).name + "_f050.png")
    Path(png).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(png, dpi=100)
    video = save_animation(fig, update, max_frames, out_stem)
    plt.close(fig)
    return {"video": video, "frames": [png]}


def write_scene_ply(path, scene3d, window, z_floor, tracks, tau=0.3):
    xmin, ymin, xmax, ymax = window
    sub = scene3d.subset((scene3d.opacity > tau)
                         & (scene3d.means[:, 0] >= xmin) & (scene3d.means[:, 0] <= xmax)
                         & (scene3d.means[:, 1] >= ymin) & (scene3d.means[:, 1] <= ymax))
    hz = np.clip((sub.means[:, 2] - z_floor) / 2.5, 0, 1)
    rgb = (plt.get_cmap("viridis")(hz)[:, :3] * 255).astype(np.uint8)
    pts, cols = [sub.means], [rgb]
    for robot, poses, colour in tracks:
        ang = np.linspace(0, 2 * np.pi, 24, endpoint=False)
        z_lo, z_hi = robot.band_abs(z_floor)
        zs = np.linspace(z_lo, z_hi, 6)
        r = robot.max_radius()
        for x, y, _ in poses[:: max(1, len(poses) // 60)]:
            ring = np.column_stack([x + r * np.cos(ang), y + r * np.sin(ang)])
            for z in zs:
                pts.append(np.column_stack([ring, np.full(len(ring), z)]))
                cols.append(np.tile(np.asarray(colour, np.uint8), (len(ring), 1)))
    P, C = np.vstack(pts).astype("<f4"), np.vstack(cols)
    arr = np.zeros(len(P), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                  ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    arr["x"], arr["y"], arr["z"] = P[:, 0], P[:, 1], P[:, 2]
    arr["red"], arr["green"], arr["blue"] = C[:, 0], C[:, 1], C[:, 2]
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {len(P)}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\n"
              "end_header\n")
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        arr.tofile(f)
    return int(len(P))
