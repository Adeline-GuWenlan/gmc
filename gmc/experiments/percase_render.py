# gmc/experiments/percase_render.py
"""Amendment 2 videos for one robot: 3D MP4, two-panel MP4, scene PLY, manifest.

    --robot R      loads the 7.3 M-splat showcase scene: run only through gmc/hpc/percase_render.sbatch
    --toy-check    renders the toy T2-1 sweeper result on the synthetic table (login-node safe)
    --renderer     "splat" (default) rasterises the Gaussians themselves through gmc.height.ewa;
                   "scatter" is the old mplot3d dots of the splat means
    --stills K     render only K preview frames into <robot>/video/preview/ and stop (no 2-panel, no PLY)
"""
import argparse
import json
import os
import time
from pathlib import Path

import numpy as np

from gmc.height.pathio import sample_curve
from gmc.height.prism import robot_table
from gmc.height.project import project_scene
from gmc.height.viz import _frame_poses, robot_video, write_scene_ply
from gmc.height import ewa
from gmc.height.viz3d import (height_colors, load_dc_colors, overview_png, pointcloud_video,
                              ply_vertex_layout, weighted_subsample)

PERCASE = Path("results/height/percase")
PLANE_SCENE = Path("/scratch/wg2381/splathjb/gmc/outputs/height/plane/processed_planefloor.npz")
TAU, DILATE, Z_SPAN = 0.3, 1.0, 2.5
CLAIM = "per-robot showcase case, not a morphology comparison · floor rule on"
CLAIM_PLANE = ("Amendment 3 shared case: one window, one start, one goal, three robots · plane floor = "
               "USER-APPROVED MANUAL SCENE EDIT, sound w.r.t. the edited scene only · criterion 3 "
               "relaxed by user decision D1 to r + 0.05 m")
VIEW_HALF = {"sweeper": 1.75, "uav": 2.5, "cylinder": 2.5}
VIEW_PAD = 0.5       # camera framing: case window + 0.5 m (splats are selected over window + 1 m)


def pick_run(rdir, robot):
    tried = {}
    for name in (f"{robot}.json", f"{robot}_b2.json"):
        p = rdir / name
        if not p.exists():
            tried[name] = "missing"
            continue
        run = json.loads(p.read_text())
        res = run.get("result") or {}
        tried[name] = res.get("status")
        if res.get("status") == "REACHABLE" and res.get("curve") is not None:
            return p, run
    raise SystemExit(f"percase_render: no REACHABLE run with a curve for {robot} in {rdir}: {tried}")


def headline(robot, res):
    rep = res.get("replay3d") or {}
    lb = rep.get("min_clearance_lb")
    ok = bool((res.get("verify") or {}).get("certified")) and bool(rep.get("passed"))
    head = f"{robot} · {res['status']} · replay3d lb = " + (f"{lb:.3f} m" if lb is not None else "n/a")
    return head + ("" if ok else " · WARNING: verify.certified / replay3d.passed not both true"), ok


def rgb_stats(rgb):
    q = lambda p: np.round(np.percentile(rgb, p, axis=0), 4).tolist()
    return {"mean": np.round(rgb.mean(axis=0), 4).tolist(), "p5": q(5), "p50": q(50), "p95": q(95),
            "frac_at_0": float((rgb <= 0).mean()), "frac_at_1": float((rgb >= 1).mean())}


def frame_count(path):
    import imageio_ffmpeg
    if path is None or Path(path).suffix != ".mp4":
        return None
    return int(imageio_ffmpeg.count_frames_and_secs(str(path))[0])


def render_outputs(name, vdir, *, pts, cols, robot, res, s2, scene_near, z_f, window, start, goal, frames,
                   workers, view_half, title, title_2panel, note, view_window=None,
                   renderer="scatter", splats=None, splat_opts=None):
    vdir.mkdir(parents=True, exist_ok=True)
    timings = {}
    vw = window if view_window is None else view_window
    curve, poses = _frame_poses(res, robot, frames)
    path_xy = sample_curve(curve, 0.02, robot.max_radius())[:, :2]

    t = time.time()
    v3 = pointcloud_video(pts, cols, robot, poses, z_f, vdir / f"{name}_3d", title=title, window=vw,
                          max_frames=frames, workers=workers, view_half=view_half, path_xy=path_xy,
                          start=start, goal=goal, note=note, renderer=renderer, splats=splats,
                          splat_opts=splat_opts)
    timings["video_3d"] = time.time() - t
    print(f"3d video {v3['video']} frames={v3['n_frames']} {timings['video_3d']:.0f}s "
          f"({v3['render_seconds_per_frame']:.2f}s/frame/worker)", flush=True)

    t = time.time()
    ov = overview_png(pts, cols, robot, poses, z_f, vdir / f"{name}_overview.png", title=title,
                      window=vw, path_xy=path_xy, start=start, goal=goal, note=note,
                      renderer=renderer, splats=splats, splat_opts=splat_opts)
    timings["overview"] = time.time() - t

    t = time.time()
    v2 = robot_video(s2, robot, res, vdir / f"{name}_2panel", scene3d=scene_near, z_floor=z_f,
                     title=title_2panel, max_frames=frames, tau=TAU)
    timings["video_2panel"] = time.time() - t
    print(f"2panel video {v2['video']} {timings['video_2panel']:.0f}s", flush=True)

    t = time.time()
    ply = vdir / f"{name}_scene.ply"
    n_ply = write_scene_ply(ply, scene_near, window, z_f, [(robot, poses, (255, 90, 0))], tau=TAU)
    timings["scene_ply"] = time.time() - t

    outputs = {"video_3d": str(v3["video"]), "n_frames_3d": frame_count(v3["video"]),
               "key_frames_3d": [str(p) for p in v3["frames"]], "overview_png": str(ov),
               "render_3d": {k: v3[k] for k in ("n_points", "render_seconds_per_frame", "workers", "size",
                                                 "view_half", "elev", "orbit_deg", "renderer",
                                                 "splat_opts")},
               "video_2panel": str(v2["video"]) if v2["video"] else None, "n_frames_2panel": frame_count(v2["video"]),
               "key_frames_2panel": [str(p) for p in v2["frames"]],
               "scene_ply": str(ply), "scene_ply_vertices": n_ply, "n_poses": int(len(poses))}
    return outputs, timings


def parse_splat_opts(pairs):
    """--splat-opt KEY=VALUE into a dict, checked against the known camera/culling knobs."""
    import ast
    from gmc.height.viz3d import SPLAT_OPTS
    out = {}
    for item in pairs:
        if "=" not in item:
            raise SystemExit(f"--splat-opt expects KEY=VALUE, got {item!r}")
        key, _, val = item.partition("=")
        key = key.strip()
        if key not in SPLAT_OPTS:
            raise SystemExit(f"--splat-opt: unknown key {key!r}; known: {sorted(SPLAT_OPTS)}")
        try:
            out[key] = ast.literal_eval(val.strip())
        except (ValueError, SyntaxError):
            raise SystemExit(f"--splat-opt {key}: cannot parse value {val!r}")
    return out


def dc_colors(ply_path, ids, z=None, z_floor=None):
    """DC colours for splat ``ids``, tolerating ids the PLY does not have.

    The Amendment 3 plane floor inserts analytic tiles whose ids continue past the end of the source
    PLY; they are real geometry in the edited scene and must still be drawn. They get the height colour
    map instead of a DC colour, and the count is reported so a manifest never hides them.
    """
    ids = np.asarray(ids, dtype=np.int64).reshape(-1)
    n = ply_vertex_layout(ply_path)[0]
    known = ids < n
    rgb = np.empty((len(ids), 3), dtype=float)
    if known.any():
        rgb[known] = load_dc_colors(ply_path, ids[known])
    if (~known).any():
        if z is not None and z_floor is not None:
            rgb[~known] = height_colors(np.asarray(z)[~known], z_floor, z_span=Z_SPAN)[:, :3]
        else:
            rgb[~known] = 0.5
    return rgb, int((~known).sum())


def splat_scene(scene, ply_path, win, radius, z_floor=None):
    """ewa.pack of every Gaussian within ``radius`` m of the case window (``radius <= 0``: whole scene).

    No opacity threshold and no subsample, unlike the scatter path: a real rasteriser alpha-blends the
    faint splats instead of dropping them, and they are most of what makes a surface read as a surface.
    """
    if radius and radius > 0:
        m = ((scene.means[:, 0] >= win[0] - radius) & (scene.means[:, 0] <= win[2] + radius)
             & (scene.means[:, 1] >= win[1] - radius) & (scene.means[:, 1] <= win[3] + radius))
        sub = scene.subset(m)
    else:
        sub = scene
    cols, n_inserted = dc_colors(ply_path, sub.ids, sub.means[:, 2], z_floor)
    return ewa.pack(sub.means, sub.covs, sub.opacity, cols), cols, int(len(sub)), n_inserted


def render_real(robot_key, a):
    from showcase_scene import DATA, load_processed
    rdir = Path(a.case_dir) if a.case_dir else PERCASE / robot_key
    vdir = rdir / "video"
    case = json.loads((rdir / "case.json").read_text())
    run_path, run = pick_run(rdir, robot_key)
    rc = run.get("case") or {}
    diff = [k for k in ("window", "start", "goal", "z_floor", "z_c")
            if k in case and k in rc and not np.allclose(case[k], rc[k])]
    if diff:
        raise SystemExit(f"percase_render: {run_path} was run on a case that differs from case.json in {diff}")
    res = run["result"]
    head, ok = headline(robot_key, res)
    if not ok:
        print(f"WARNING: {run_path}: {head}", flush=True)
    robot = robot_table(case.get("z_c", 1.20))[robot_key]
    z_f, win = float(case["z_floor"]), [float(v) for v in case["window"]]
    timings = {}

    t = time.time()
    if a.scene == "planefloor":
        from gmc.height.planefloor import load_plane_scene
        scene, meta = load_plane_scene(PLANE_SCENE)
        g0 = {"floor": meta["floor"], "ceiling_height_m": meta["ceiling_height_m"],
              "gravity_rotation": meta["gravity_rotation"], "planefloor": meta}
    else:
        scene, g0 = load_processed()
    timings["load_processed"] = time.time() - t
    print(f"loaded {len(scene)} splats in {timings['load_processed']:.0f}s", flush=True)

    t = time.time()
    m = ((scene.means[:, 0] >= win[0] - DILATE) & (scene.means[:, 0] <= win[2] + DILATE)
         & (scene.means[:, 1] >= win[1] - DILATE) & (scene.means[:, 1] <= win[3] + DILATE))
    near = scene.subset(m)
    cand = np.flatnonzero((near.opacity > TAU) & (near.means[:, 2] <= z_f + Z_SPAN))
    pick = cand[weighted_subsample(near.opacity[cand], a.max_points, seed=0)]
    pts = near.means[pick]
    timings["select"] = time.time() - t

    t = time.time()
    ply_path = DATA / "raw" / "point_cloud.ply"
    cols, n_inserted_pts = dc_colors(ply_path, near.ids[pick], pts[:, 2], z_f)
    timings["colors"] = time.time() - t
    print(f"selected {len(pick)} of {len(cand)} splats; colours in {timings['colors']:.0f}s", flush=True)

    t = time.time()
    s2, pstats = project_scene(scene, robot, win, z_floor=z_f, tau=TAU)
    timings["project"] = time.time() - t
    run_kept = (run.get("projection") or {}).get("kept")
    print(f"projected {pstats['kept']} supports (run: {run_kept})", flush=True)
    splats, n_splat = None, None
    if a.renderer == "splat":
        t = time.time()
        splats, cols, n_splat, n_inserted_pts = splat_scene(scene, ply_path, win, a.scene_radius, z_f)
        timings["splat_pack"] = time.time() - t
        print(f"packed {n_splat:,} Gaussians for the EWA rasteriser in "
              f"{timings['splat_pack']:.0f}s", flush=True)
    n_scene = len(scene)
    del scene, m

    view_half = VIEW_HALF[robot_key] if a.view_half is None else (a.view_half if a.view_half > 0 else None)
    vw = [win[0] - VIEW_PAD, win[1] - VIEW_PAD, win[2] + VIEW_PAD, win[3] + VIEW_PAD]
    claim = CLAIM_PLANE if a.scene == "planefloor" else CLAIM
    title = f"{head}\n{claim}"
    note = (f"{n_splat:,} Gaussians, EWA splatting, no opacity threshold, no subsample, DC colour"
            if a.renderer == "splat" else
            f"{len(pts):,} opaque splats (τ = {TAU}), opacity-weighted subsample, DC colour")

    if a.stills > 0:                 # cheap realism preview: a few frames, no 2-panel and no scene PLY
        pdir = vdir / "preview"
        curve, poses = _frame_poses(res, robot, a.stills)
        path_xy = sample_curve(curve, 0.02, robot.max_radius())[:, :2]
        t = time.time()
        v = pointcloud_video(pts, cols, robot, poses, z_f, pdir / f"{robot_key}_preview", title=title,
                             window=vw, max_frames=a.stills, workers=a.workers, view_half=view_half,
                             path_xy=path_xy, start=case["start"], goal=case["goal"], note=note,
                             key_fracs=tuple(np.linspace(0.0, 1.0, a.stills)),
                             renderer=a.renderer, splats=splats, splat_opts=a.splat_opts)
        info = {"robot": robot_key, "preview": True, "renderer": v["renderer"],
                "frames": [str(x) for x in v["frames"]], "video": str(v["video"]),
                "n_rendered": v["n_points"], "n_scene_after_floor_rule": n_scene,
                "scene_radius_m": a.scene_radius, "size": v["size"],
                "seconds_per_frame": v["render_seconds_per_frame"], "splat_opts": v["splat_opts"],
                "colors": {"rule": "clip(0.5 + 0.28209479 * f_dc, 0, 1)", **rgb_stats(cols)},
                "seconds": round(time.time() - t, 1)}
        (pdir / "manifest.json").write_text(json.dumps(info, indent=2, default=str))
        print(json.dumps(info, indent=2, default=str), flush=True)
        return

    outputs, t_out = render_outputs(
        robot_key, vdir, pts=pts, cols=cols, robot=robot, res=res, s2=s2, scene_near=near, z_f=z_f,
        window=win, view_window=vw,
        start=case["start"], goal=case["goal"], frames=a.frames, workers=a.workers,
        view_half=view_half, title=title, title_2panel=f"{claim}\n", note=note,
        renderer=a.renderer, splats=splats, splat_opts=a.splat_opts)
    timings.update(t_out)
    rep = res.get("replay3d") or {}
    manifest = {
        "robot": robot_key, "claims": claim, "title": f"{head}\n{claim}",
        "case_json": str(rdir / "case.json"), "run_json": str(run_path), "status": res["status"],
        "verify_certified": bool((res.get("verify") or {}).get("certified")),
        "replay3d_passed": bool(rep.get("passed")), "replay3d_min_clearance_lb": rep.get("min_clearance_lb"),
        "scene": {"variant": a.scene, "n_splats": n_scene, "floor_rule": g0.get("floor_rule"),
                  "planefloor": (g0.get("planefloor") or {}).get("claims_boundary"),
                  "gravity_rotation_is_identity": bool(np.allclose(g0["gravity_rotation"], np.eye(3)))},
        "render": {"renderer": a.renderer, "n_rendered": n_splat if n_splat is not None else int(len(pts)),
                   "scene_radius_m": a.scene_radius if a.renderer == "splat" else None,
                   "opacity_threshold": None if a.renderer == "splat" else TAU,
                   "subsampled": a.renderer != "splat"},
        "selection": {"window": win, "view_window_pad_m": VIEW_PAD, "dilate_m": DILATE, "z_max_above_floor_m": Z_SPAN, "tau": TAU,
                      "n_window_xy_dilated": int(len(near)), "n_opaque_below_zmax": int(len(cand)),
                      "n_rendered": int(len(pick)), "max_points": a.max_points,
                      "subsample": "opacity-weighted without replacement (Efraimidis-Spirakis), seed 0"},
        "colors": {"source": str(ply_path), "rule": "clip(0.5 + 0.28209479 * f_dc, 0, 1)",
                   "n_without_a_ply_row_height_coloured": int(n_inserted_pts), **rgb_stats(cols)},
        "projection": {"kept": pstats["kept"], "run_kept": run_kept, "matches_run": pstats["kept"] == run_kept},
        "outputs": outputs, "timings_s": {k: round(v, 1) for k, v in timings.items()}}
    (vdir / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str))
    print(json.dumps({k: manifest[k] for k in ("status", "verify_certified", "replay3d_passed",
                                               "selection", "projection", "timings_s")}, default=str), flush=True)


def splat_samples(means, covs, per, level=2.0, seed=0):
    """``per`` points inside each splat's level-sigma ellipsoid (to make a sparse synthetic scene readable)."""
    rng = np.random.default_rng(seed)
    L = np.linalg.cholesky(covs + 1e-12 * np.eye(3))
    z = rng.standard_normal((len(means), per, 3))
    z *= np.minimum(1.0, level / np.maximum(np.linalg.norm(z, axis=2, keepdims=True), 1e-12))
    return (means[:, None, :] + np.einsum("nij,nkj->nki", L, z)).reshape(-1, 3)


def render_toy(a):
    from gmc.height.synth3d import GOAL, START, WORKSPACE, Z_FLOOR, table_scene
    vdir = PERCASE / "_toy_check"
    res = json.loads(Path("results/height/toy/T2-1.json").read_text())["result"]
    s3, _ = table_scene("closed")
    robot = robot_table(1.20)["sweeper"]
    s2, pstats = project_scene(s3, robot, WORKSPACE, z_floor=Z_FLOOR)
    pts = splat_samples(s3.means, s3.covs, 40)
    cols = height_colors(pts[:, 2], Z_FLOOR, Z_SPAN)
    splats = (ewa.pack(s3.means, s3.covs, s3.opacity,
                       height_colors(s3.means[:, 2], Z_FLOOR, Z_SPAN)) if a.renderer == "splat" else None)
    head, ok = headline("sweeper", res)
    t = time.time()
    view_half = VIEW_HALF["sweeper"] if a.view_half is None else (a.view_half if a.view_half > 0 else None)
    outputs, timings = render_outputs(
        "T2-1_sweeper", vdir, pts=pts, cols=cols, robot=robot, res=res, s2=s2, scene_near=s3, z_f=Z_FLOOR,
        window=WORKSPACE, start=START, goal=GOAL, frames=a.frames, workers=a.workers, view_half=view_half,
        title=f"{head}\nTOY CHECK: synthetic T2-1 table, coloured by height (not a showcase result)",
        title_2panel="TOY CHECK T2-1\n", renderer=a.renderer, splats=splats,
        splat_opts=a.splat_opts,
        note=(f"{len(s3)} synthetic Gaussians, EWA splatting" if a.renderer == "splat"
              else f"{len(pts):,} samples of {len(s3)} synthetic splats"))
    manifest = {"toy": "T2-1", "status": res["status"], "certified_and_replayed": ok,
                "renderer": a.renderer,
                "projection_kept": pstats["kept"], "n_points": int(len(pts)), "outputs": outputs,
                "timings_s": {k: round(v, 2) for k, v in timings.items()}, "total_s": round(time.time() - t, 1)}
    (vdir / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str))
    print(json.dumps(manifest, indent=2, default=str), flush=True)


def main():
    ap = argparse.ArgumentParser(description="Amendment 2 per-robot videos")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--robot", choices=["sweeper", "uav", "cylinder"])
    g.add_argument("--toy-check", action="store_true")
    ap.add_argument("--case-dir", default="",
                    help="directory holding case.json and <robot>.json; default results/height/percase/<robot>")
    ap.add_argument("--scene", default="processed", choices=["processed", "planefloor"],
                    help="'planefloor' renders the Amendment 3 P1b plane-floor scene (a USER-APPROVED "
                         "MANUAL SCENE EDIT)")
    ap.add_argument("--renderer", choices=["splat", "scatter"], default="splat",
                    help="splat: real EWA Gaussian rasteriser (gmc.height.ewa); scatter: old mplot3d dots")
    ap.add_argument("--scene-radius", type=float, default=14.0,
                    help="metres beyond the case window kept for the splat renderer; <= 0 keeps the "
                         "whole scene")
    ap.add_argument("--stills", type=int, default=0,
                    help="render only this many preview frames into <robot>/video/preview/ and stop")
    ap.add_argument("--splat-opt", action="append", default=[], metavar="KEY=VALUE",
                    help="override one gmc.height.viz3d.SPLAT_OPTS entry (camera and culling knobs), "
                         "e.g. --splat-opt fit_scale=0.8 --splat-opt near_frac=0.45; repeatable")
    ap.add_argument("--max-points", type=int, default=250_000)
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--workers", type=int, default=min(4, len(os.sched_getaffinity(0))))
    ap.add_argument("--view-half", type=float, default=None,
                    help="half width (m) of the follow camera box; <= 0 shows the whole window")
    a = ap.parse_args()
    a.splat_opts = parse_splat_opts(a.splat_opt)
    if a.toy_check:
        render_toy(a)
    else:
        render_real(a.robot, a)


if __name__ == "__main__":
    main()
