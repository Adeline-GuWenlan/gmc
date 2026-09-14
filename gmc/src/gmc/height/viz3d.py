"""3D point-cloud videos of a certified run (Amendment 2; spec §6 as amended).

Splat means are drawn with matplotlib mplot3d; frames are rendered in a process pool and encoded to H.264
MP4 (yuv420p) through imageio-ffmpeg. Real-scene colours are the spherical-harmonic DC term of the raw 3DGS
PLY, ``rgb = clip(0.5 + C0 * f_dc, 0, 1)``.

mplot3d orders whole artists, not single points, so every frame splits the cloud at the robot's depth along
the viewing direction: splats behind the robot are drawn first, then the prism, then the splats in front.
Front splats that cover the robot on screen (plus a margin) are drawn translucent, so a robot passing under a
table stays visible; the prism outline is drawn last.
"""
import math
import multiprocessing as mp
import os
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

SH_C0 = 0.28209479177387814

_PLY_TYPES = {"char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
              "short": "<i2", "int16": "<i2", "ushort": "<u2", "uint16": "<u2",
              "int": "<i4", "int32": "<i4", "uint": "<u4", "uint32": "<u4",
              "float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8"}

ROBOT_FACE = (1.0, 0.45, 0.0, 0.55)
ROBOT_EDGE = (0.80, 0.22, 0.0)
PATH_RGB = (0.05, 0.30, 0.85)
FLOOR_RGB = (0.93, 0.93, 0.92)
GRID_RGB = (0.78, 0.78, 0.78)


# ----------------------------------------------------------------------------- colours

def ply_vertex_layout(path):
    """(n_vertices, structured dtype, byte offset) of the vertex block of a binary little-endian PLY.

    ``ply3d._read_header`` is not reused: it requires the 3DGS geometry fields and float-only properties.
    """
    path = Path(path)
    with open(path, "rb") as f:
        head = f.read(65536)
    end = head.find(b"end_header\n")
    if not head.startswith(b"ply\n") or end < 0:
        raise ValueError(f"{path}: not a PLY file")
    lines = head[:end].decode("ascii").splitlines()
    if "format binary_little_endian 1.0" not in lines:
        raise ValueError(f"{path}: only binary_little_endian PLY is supported")
    n, fields, element = None, [], None
    for line in lines:
        tok = line.split()
        if not tok:
            continue
        if tok[0] == "element":
            if element == "vertex":
                break                       # later elements follow the vertex block
            if tok[1] != "vertex":
                raise ValueError(f"{path}: element '{tok[1]}' precedes 'vertex'; unsupported")
            element, n = "vertex", int(tok[2])
        elif tok[0] == "property" and element == "vertex":
            if tok[1] == "list" or tok[1] not in _PLY_TYPES:
                raise ValueError(f"{path}: unsupported vertex property type in '{line}'")
            fields.append((tok[2], _PLY_TYPES[tok[1]]))
    if n is None:
        raise ValueError(f"{path}: no vertex element")
    return n, np.dtype(fields), end + len(b"end_header\n")


def load_dc_colors(ply_path, ids):
    """(N,3) float64 RGB in [0,1] of raw PLY vertex rows ``ids`` (memory-mapped; only those pages are read).

    Non-finite DC coefficients decode to 0.5 (grey).
    """
    n, dtype, offset = ply_vertex_layout(ply_path)
    names = [f"f_dc_{k}" for k in range(3)]
    missing = [c for c in names if c not in dtype.names]
    if missing:
        raise ValueError(f"{ply_path}: missing PLY fields {missing}")
    if Path(ply_path).stat().st_size < offset + n * dtype.itemsize:
        raise ValueError(f"{ply_path}: truncated vertex block")
    ids = np.asarray(ids, dtype=np.int64).reshape(-1)
    if ids.size == 0:
        return np.zeros((0, 3))
    if ids.min() < 0 or ids.max() >= n:
        raise IndexError(f"ids out of range [0, {n})")
    mm = np.memmap(ply_path, dtype=dtype, mode="r", offset=offset, shape=(n,))
    order = np.argsort(ids, kind="stable")
    rows = mm[ids[order]]                   # ascending rows: sequential page access
    f_dc = np.column_stack([rows[c].astype(np.float64) for c in names])
    del mm
    rgb = np.empty_like(f_dc)
    rgb[order] = np.clip(np.nan_to_num(0.5 + SH_C0 * f_dc, nan=0.5), 0.0, 1.0)
    return rgb


def height_colors(z, z_floor, z_span=2.5, cmap="viridis"):
    """Colour by height above the floor (for scenes without a PLY)."""
    import matplotlib
    t = np.clip((np.asarray(z, dtype=float) - float(z_floor)) / float(z_span), 0.0, 1.0)
    return matplotlib.colormaps[cmap](t)[:, :3]


def weighted_subsample(weights, k, seed=0):
    """Sorted indices of ``k`` items drawn without replacement with P proportional to weight
    (Efraimidis-Spirakis keys); every index when ``k >= len(weights)``."""
    w = np.asarray(weights, dtype=float)
    n = len(w)
    if k >= n:
        return np.arange(n)
    rng = np.random.default_rng(seed)
    with np.errstate(divide="ignore"):
        keys = np.log(rng.random(n)) / np.maximum(w, 1e-300)
    keys[w <= 0] = -np.inf
    return np.sort(np.argpartition(keys, n - k)[n - k:])


# ----------------------------------------------------------------------------- geometry helpers

def _densify_xy(xy, step=0.02):
    xy = np.asarray(xy, dtype=float)[:, :2]
    if len(xy) < 2:
        return xy
    out = [xy[:1]]
    for a, b in zip(xy[:-1], xy[1:]):
        m = max(1, int(math.ceil(float(np.linalg.norm(b - a)) / step)))
        out.append(a + np.linspace(0.0, 1.0, m + 1)[1:, None] * (b - a))
    return np.vstack(out)


def _mask_box(xyz, x0, x1, y0, y1):
    xyz = np.array(xyz, dtype=float)
    out = (xyz[:, 0] < x0) | (xyz[:, 0] > x1) | (xyz[:, 1] < y0) | (xyz[:, 1] > y1)
    xyz[out] = np.nan
    return xyz


def _prism(x, y, th, r, z_lo, z_hi, azim_deg, n=40):
    t = np.linspace(0.0, 2 * np.pi, n + 1)
    px, py = x + r * np.cos(t), y + r * np.sin(t)
    lo, hi = np.full(n + 1, z_lo), np.full(n + 1, z_hi)
    side = [np.array([[px[i], py[i], z_lo], [px[i + 1], py[i + 1], z_lo],
                      [px[i + 1], py[i + 1], z_hi], [px[i], py[i], z_hi]]) for i in range(n)]
    caps = [np.column_stack([px[:-1], py[:-1], lo[:-1]]), np.column_stack([px[:-1], py[:-1], hi[:-1]])]
    edges = []                              # 2-point segments: Line3DCollection needs equal lengths
    for z in (lo, hi):
        ring = np.column_stack([px, py, z])
        edges.extend(np.stack([ring[:-1], ring[1:]], axis=1))
    edges.append(np.array([[x, y, z_hi], [x + r * math.cos(th), y + r * math.sin(th), z_hi]]))
    for sgn in (1.0, -1.0):                 # silhouette lines seen from the camera azimuth
        b = math.radians(azim_deg) + sgn * np.pi / 2
        ex, ey = x + r * math.cos(b), y + r * math.sin(b)
        edges.append(np.array([[ex, ey, z_lo], [ex, ey, z_hi]]))
    return side + caps, edges


def _default_azim(poses):
    d = np.asarray(poses)[-1, :2] - np.asarray(poses)[0, :2]
    if float(np.linalg.norm(d)) < 1e-6:
        return -60.0
    return float(np.degrees(np.arctan2(d[1], d[0]))) - 50.0   # 3/4 view: camera ahead-right of travel


def _cameras(poses, window, view_half, elev, orbit_deg, azim0, margin=0.8):
    """Per frame (cx, cy, hx, hy, azim, elev).

    Follow mode: the box centre sits between the path's bbox centre and the robot, pulled toward the robot
    only as far as needed to keep it ``margin`` inside the box (static camera when the path fits).
    """
    m = len(poses)
    xmin, ymin, xmax, ymax = map(float, window)
    azim0 = _default_azim(poses) if azim0 is None else float(azim0)
    t = np.linspace(0.0, 1.0, m) if m > 1 else np.zeros(1)
    azim = azim0 + float(orbit_deg) * (t - 0.5)
    if view_half is None or 2 * view_half >= max(xmax - xmin, ymax - ymin):
        c = np.tile([(xmin + xmax) / 2, (ymin + ymax) / 2], (m, 1))
        hx, hy = (xmax - xmin) / 2, (ymax - ymin) / 2
    else:
        from scipy.ndimage import gaussian_filter1d
        xy = np.asarray(poses, dtype=float)[:, :2]
        mid = 0.5 * (xy.min(axis=0) + xy.max(axis=0))
        reach = float(np.max(np.abs(xy - mid)))           # Chebyshev: the box is axis-aligned
        lam = 0.0 if reach < 1e-9 else float(np.clip(1.0 - (view_half - margin) / reach, 0.0, 1.0))
        c = mid + lam * (xy - mid)
        if m > 2:
            c = gaussian_filter1d(c, max(1.0, m / 25.0), axis=0, mode="nearest")
        hx = hy = float(view_half)
    return np.column_stack([c, np.full(m, hx), np.full(m, hy), azim, np.full(m, float(elev))])


# ----------------------------------------------------------------------------- drawing

def _state(points, colors, robot, poses, z_floor, *, title, window, dpi, z_span, point_size, figsize,
           path_xy, start, goal, note, zoom, focal_length):
    P = np.asarray(points, dtype=np.float32)
    C = np.asarray(colors, dtype=np.float32)
    if P.ndim != 2 or P.shape[1] != 3 or C.shape != P.shape:
        raise ValueError("points and colors must both be (N,3)")
    poses = np.asarray(poses, dtype=float)
    z_lo, z_hi = robot.band_abs(z_floor)
    return {"points": P, "colors": np.clip(C, 0.0, 1.0), "poses": poses, "z_floor": float(z_floor),
            "z_span": float(z_span), "band": (z_lo, z_hi), "r": robot.max_radius(), "title": title,
            "window": tuple(map(float, window)), "dpi": dpi, "figsize": tuple(figsize),
            "point_size": float(point_size),
            "path_xy": _densify_xy(poses[:, :2] if path_xy is None else path_xy),
            "start": None if start is None else tuple(map(float, start[:2])),
            "goal": None if goal is None else tuple(map(float, goal[:2])),
            "note": note, "zoom": float(zoom), "focal_length": float(focal_length), "grid": 0.5,
            "xray_alpha": 0.15, "xray_margin": 0.12, "title_size": 13}


def _new_fig(st):
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    fig = Figure(figsize=st["figsize"], dpi=st["dpi"])
    FigureCanvasAgg(fig)
    return fig


def _rgb(fig):
    fig.canvas.draw()
    buf = np.asarray(fig.canvas.buffer_rgba())
    h, w = buf.shape[0] // 2 * 2, buf.shape[1] // 2 * 2    # yuv420p needs even sides
    return np.ascontiguousarray(buf[:h, :w, :3])


def _caption(st, k):
    x, y, th = st["poses"][k]
    return (f"frame {k + 1}/{len(st['poses'])}   pose x = {x:+.2f} m, y = {y:+.2f} m, "
            f"θ = {math.degrees(th):+.0f}°   floor grid {st['grid']:g} m   {st['note']}")


def _draw(fig, st, cam, k=None, ghosts=(), caption=""):
    from matplotlib.lines import Line2D
    from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

    cx, cy, hx, hy, azim, elev = (float(v) for v in cam)
    x0, x1, y0, y1 = cx - hx, cx + hx, cy - hy, cy + hy
    zf = st["z_floor"]
    z0, z1 = zf - 0.02, zf + st["z_span"]
    z_lo, z_hi = st["band"]
    zmid, r = 0.5 * (z_lo + z_hi), st["r"]
    poses = st["poses"]
    fig.clf()
    ax = fig.add_axes((0.0, 0.045, 1.0, 0.865), projection="3d", computed_zorder=False)

    ax.add_collection3d(Poly3DCollection([[(x0, y0, zf), (x1, y0, zf), (x1, y1, zf), (x0, y1, zf)]],
                                         facecolors=[FLOOR_RGB], edgecolors="none", zorder=0))
    g = st["grid"]
    segs = ([[(x, y0, zf), (x, y1, zf)] for x in np.arange(math.ceil(x0 / g) * g, x1, g)]
            + [[(x0, y, zf), (x1, y, zf)] for y in np.arange(math.ceil(y0 / g) * g, y1, g)])
    if segs:
        ax.add_collection3d(Line3DCollection(segs, colors=[GRID_RGB], linewidths=0.5, zorder=0.5))

    P, C = st["points"], st["colors"]
    inb = (P[:, 0] >= x0) & (P[:, 0] <= x1) & (P[:, 1] >= y0) & (P[:, 1] <= y1)
    P, C = P[inb], C[inb]
    if k is not None and len(P):
        a, e = math.radians(azim), math.radians(elev)
        u = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
        x, y, _ = poses[k]
        rel = P.astype(np.float64) - np.array([x, y, zmid])
        depth = rel @ u                                   # > 0: nearer the camera than the robot centre
        o = rel - depth[:, None] * u                      # offset in the image plane
        vert = np.array([0.0, 0.0, 1.0]) - math.sin(e) * u
        hz = 0.5 * (z_hi - z_lo)
        t = np.clip(o @ vert / max(float(vert @ vert), 1e-12), -hz, hz)
        covers = np.linalg.norm(o - t[:, None] * vert, axis=1) < r + st["xray_margin"]
        front = depth > 0
        layers = [(~front, 1.0, 1.0), (front & ~covers, 1.0, 4.0), (front & covers, st["xray_alpha"], 4.1)]
    else:
        layers = [(np.ones(len(P), dtype=bool), 1.0, 1.0)]
    for sel, alpha, z in layers:
        if sel.any():
            ax.scatter(P[sel, 0], P[sel, 1], P[sel, 2], c=C[sel], s=st["point_size"], marker="o",
                       linewidths=0, depthshade=False, alpha=alpha, zorder=z)

    path = st["path_xy"]
    floor_path = _mask_box(np.column_stack([path, np.full(len(path), zf + 0.004)]), x0, x1, y0, y1)
    ax.plot(*floor_path.T, color=(0.12, 0.12, 0.12), lw=1.1, ls=(0, (4, 3)), zorder=2)
    mid_path = _mask_box(np.column_stack([path, np.full(len(path), zmid)]), x0, x1, y0, y1)
    ax.plot(*mid_path.T, color=PATH_RGB, lw=0.8, alpha=0.55, zorder=2)

    for j in ([k] if k is not None else list(ghosts)):
        x, y, th = poses[j]
        if not (x0 - r <= x <= x1 + r and y0 - r <= y <= y1 + r):
            continue
        polys, edges = _prism(x, y, th, r, z_lo, z_hi, azim)
        ax.add_collection3d(Poly3DCollection(polys, facecolors=[ROBOT_FACE], edgecolors="none", zorder=3))
        ax.add_collection3d(Line3DCollection(edges, colors=[ROBOT_EDGE], linewidths=1.3, zorder=5))
    if k is not None:
        trail = _mask_box(np.column_stack([poses[:k + 1, :2], np.full(k + 1, zmid)]), x0, x1, y0, y1)
        ax.plot(*trail.T, color=PATH_RGB, lw=2.6, zorder=3.5)
    else:
        ax.plot(*mid_path.T, color=PATH_RGB, lw=2.0, zorder=3.5)

    for p, mk, col, size in ((st["start"], "^", "#1a9850", 80), (st["goal"], "*", "#d73027", 160)):
        if p is not None and x0 <= p[0] <= x1 and y0 <= p[1] <= y1:
            ax.scatter([p[0]], [p[1]], [zf + 0.01], marker=mk, s=size, c=col, edgecolors="k",
                       linewidths=0.6, depthshade=False, zorder=6)

    for art in (*ax.collections, *ax.lines):     # Axes3D is a centred square; use the full frame width
        art.set_clip_on(False)
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_zlim(z0, z1)
    ax.set_box_aspect((x1 - x0, y1 - y0, z1 - z0), zoom=st["zoom"])
    ax.set_proj_type("persp", focal_length=st["focal_length"])
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()

    fig.suptitle(st["title"], fontsize=st["title_size"], y=0.985, va="top")
    handles = [Line2D([], [], color=ROBOT_FACE[:3], alpha=0.7, lw=7,
                      label=f"robot prism: disc r = {r:.3f} m × band "
                            f"[{z_lo - zf:.2f}, {z_hi - zf:.2f}] m above floor"),
               Line2D([], [], color=PATH_RGB, lw=2.6, label="trail (band mid-height)"),
               Line2D([], [], color=(0.12, 0.12, 0.12), lw=1.1, ls=(0, (4, 3)), label="certified path on floor"),
               Line2D([], [], color="#1a9850", marker="^", ls="none", mec="k", label="start"),
               Line2D([], [], color="#d73027", marker="*", ms=10, ls="none", mec="k", label="goal")]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.925), ncol=len(handles),
               fontsize=8.5, frameon=False, handlelength=2.6, columnspacing=2.2)
    if caption:
        fig.text(0.01, 0.012, caption, fontsize=9, color="0.25")


# ----------------------------------------------------------------------------- frame loop

_W = {}


def _worker_init(st):
    _W["st"] = st
    _W["fig"] = _new_fig(st)


def _worker_frame(k):
    st, fig = _W["st"], _W["fig"]
    t0 = time.perf_counter()
    _draw(fig, st, st["cams"][k], k=k, caption=_caption(st, k))
    img = _rgb(fig)
    return k, img, time.perf_counter() - t0


def _frame_iter(st, n, workers):
    if workers <= 1:
        _worker_init(st)
        for k in range(n):
            yield _worker_frame(k)
        return
    blas = ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")
    saved = {key: os.environ.get(key) for key in blas}
    os.environ.update({key: "1" for key in blas})     # one thread per spawned renderer
    # ProcessPoolExecutor, not multiprocessing.Pool: a dead worker raises BrokenProcessPool instead of
    # being respawned forever (a silent hang until the Slurm time limit).
    ex = ProcessPoolExecutor(workers, mp_context=mp.get_context("spawn"), initializer=_worker_init,
                             initargs=(st,))
    try:
        try:
            results = ex.map(_worker_frame, range(n))    # submits every frame now: workers spawn here
        finally:
            for key, val in saved.items():
                if val is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = val
        yield from results
    finally:
        ex.shutdown(wait=False, cancel_futures=True)


def pointcloud_video(points, colors, robot, poses, z_floor, out_stem, *, title, window, max_frames=300,
                     dpi=100, workers=4, fps=20, view_half=2.5, elev=35.0, orbit_deg=50.0, azim0=None,
                     z_span=2.5, point_size=2.5, figsize=(12.8, 7.2), path_xy=None, start=None, goal=None,
                     note="", key_fracs=(0.0, 0.5, 1.0), zoom=1.35, focal_length=1.5):
    """Render the robot prism moving along ``poses`` through a coloured point cloud to ``<out_stem>.mp4``.

    ``view_half`` (m): half width of a camera box that follows the robot; ``None`` shows the whole window.
    The azimuth drifts by ``orbit_deg`` over the clip. Returns paths and timing.
    """
    import imageio_ffmpeg
    from matplotlib.image import imsave

    out_stem = Path(out_stem)
    out_stem.parent.mkdir(parents=True, exist_ok=True)
    poses = np.asarray(poses, dtype=float)
    if len(poses) > max_frames:
        poses = poses[np.unique(np.linspace(0, len(poses) - 1, max_frames).astype(int))]
    st = _state(points, colors, robot, poses, z_floor, title=title, window=window, dpi=dpi, z_span=z_span,
                point_size=point_size, figsize=figsize, path_xy=path_xy, start=start, goal=goal, note=note,
                zoom=zoom, focal_length=focal_length)
    st["cams"] = _cameras(poses, window, view_half, elev, orbit_deg, azim0, margin=0.6 + st["r"])
    n = len(poses)
    keys = [(int(round(f * (n - 1))), f) for f in key_fracs]
    mp4 = out_stem.with_name(out_stem.name + ".mp4")
    frames, dts, size, gen, written = [], [], None, None, 0
    t0 = time.perf_counter()
    try:
        for k, img, dt in _frame_iter(st, n, workers):
            if gen is None:
                size = [int(img.shape[1]), int(img.shape[0])]
                gen = imageio_ffmpeg.write_frames(str(mp4), tuple(size), fps=fps, codec="libx264",
                                                  pix_fmt_in="rgb24", pix_fmt_out="yuv420p", quality=None,
                                                  macro_block_size=1, ffmpeg_log_level="error",
                                                  output_params=["-crf", "20", "-preset", "medium"])
                gen.send(None)
            gen.send(img.tobytes())
            written += 1
            dts.append(dt)
            for kk, f in keys:
                if kk == k:
                    png = out_stem.with_name(f"{out_stem.name}_f{int(round(100 * f)):03d}.png")
                    imsave(png, img)
                    frames.append(png)
    finally:
        if gen is not None:
            gen.close()
    return {"video": mp4, "frames": frames, "n_frames": written, "n_points": int(len(st["points"])),
            "seconds": time.perf_counter() - t0,
            "render_seconds_per_frame": float(np.mean(dts)) if dts else None,
            "workers": int(workers), "size": size, "view_half": view_half, "elev": elev,
            "orbit_deg": orbit_deg}


def overview_png(points, colors, robot, poses, z_floor, out_png, *, title, window, path_xy=None, start=None,
                 goal=None, elev=55.0, azim=None, n_ghosts=6, pad=0.3, z_span=2.5, point_size=1.5, dpi=110,
                 figsize=(12.8, 8.0), note="", zoom=1.35, focal_length=1.5):
    """Static above-oblique view of the whole window, whole path and the prism at ``n_ghosts`` poses."""
    poses = np.asarray(poses, dtype=float)
    st = _state(points, colors, robot, poses, z_floor, title=title, window=window, dpi=dpi, z_span=z_span,
                point_size=point_size, figsize=figsize, path_xy=path_xy, start=start, goal=goal, note=note,
                zoom=zoom, focal_length=focal_length)
    xmin, ymin, xmax, ymax = map(float, window)
    azim = _default_azim(poses) if azim is None else float(azim)
    cam = ((xmin + xmax) / 2, (ymin + ymax) / 2, (xmax - xmin) / 2 + pad, (ymax - ymin) / 2 + pad, azim, elev)
    ghosts = np.unique(np.linspace(0, len(poses) - 1, n_ghosts).astype(int))
    fig = _new_fig(st)
    _draw(fig, st, cam, ghosts=ghosts,
          caption=f"overview: whole certified path, prism at {len(ghosts)} poses (drawn over the splats)   "
                  f"floor grid {st['grid']:g} m   {note}")
    out_png = Path(out_png)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=dpi)
    return out_png
