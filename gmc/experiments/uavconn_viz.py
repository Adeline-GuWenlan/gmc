"""C2 figures that need the archive: route overlays, aerial3d necessity panels, EWA keyframes + MP4.

Everything is drawn from the exact planning archive (manifest-checked loader, via
``uavlamp_viz.Booth``) and the exact trajectories in the run JSONs; nothing is re-planned.
Reuses the L2 machinery unchanged (``uavlamp_viz``: Booth, draw_frame, overlay_path,
keyframe_indices, VIEWS, flythrough).

Outputs (``--out``):
  routes_M1_M5.png             M1 and M5, top + side, aerial3d (blue) and lattice (orange) routes over the
                               captured Gaussians; lamp, header, soffit, back panel and table drawn
  necessity_aerial3d.png       aerial3d, high start: (a) lamp only -> over; (b) booth -> under;
                               (c) booth + plug -> certified UNREACHABLE, cut leaves drawn
  aerial3d_M1_keyframes_{side,oblique,top}.png   EWA renders, UAV body at 6 poses of the aerial3d M1 route
                               (lattice M1 route overlaid thin orange in the side view)
  video/flythrough.mp4 (+ 4 frames read back from the MP4)   aerial3d M1 flythrough
  viz_manifest.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

from gmc.aerial3d.metrics import polyline_metrics
from gmc.gs3d import scene_uavlamp as su
from gmc.gs3d.trajectory import sample_linear_trajectory
from gmc.height import ewa

import uavlamp_viz as uv
from uavlamp_render import cam_from
from uavlamp_run import ordered_evidence

BLUE, ORANGE = "#2a78d6", "#eb6834"
STRUCT, LAMP_C, TABLE_C, PLUG_C = "#5f5e5a", "#c99a00", "#8b5a2b", "#d62728"
L2 = Path("/scratch/wg2381/splathjb-uavlamp/gmc/outputs/uavlamp/l2")


def load_a3(path, frame, spec):
    doc = json.loads(Path(path).read_text())
    r = doc["result"]
    out = {"path": str(path), "status": r["status"], "reason": r["reason"], "expansions": None,
           "algorithm_wall_s": doc["timing"]["cold_algorithm_wall_s"][0],
           "replay_passed": (doc.get("replay") or {}).get("passed"), "doc": doc,
           "traj": (r.get("gs3d_result") or {}).get("trajectory")}
    if out["traj"]:
        smp = sample_linear_trajectory(out["traj"], dt_s=.02)
        world = np.asarray(smp["poses"], float)[:, :3]
        out["world"], out["route"], out["time_s"] = world, frame.to_route(world), np.asarray(smp["time_s"])
        out["evidence"] = ordered_evidence(out["route"], lamp=spec["lamp"], table=spec["table"],
                                           radius=uv.R, half_height=uv.H, margin=uv.MARGIN)
        out["s"] = np.r_[0., np.cumsum(np.linalg.norm(np.diff(out["route"], axis=0), axis=1))]
        out["knots"] = frame.to_route(np.asarray(r["polyline_world"], float))
    return out


def structures(ax, doc, spec, view, *, drop=(), label=True):
    """Added booth structures from the manifest edits + L1's lamp/table rectangles."""
    a, b = (0, 1) if view == "top" else (0, 2)
    done = set()
    for e in doc["edits"]:
        role = e["role"]
        if role in drop or role == "lamp":
            continue
        c, (s0, s1), pl = np.asarray(e["center_route"]), e["size_m"], e["plane"]
        name = {"drop_ceiling": "soffit", "bulkhead": "header", "back_panel": "back panel"}.get(role, role)
        kw = dict(color=STRUCT, lw=2.2, solid_capstyle="butt", zorder=4)
        if view == "top":
            if pl == "vz":
                ax.plot([c[0], c[0]], [c[1] - s0 / 2, c[1] + s0 / 2], **kw)
                if label and name not in done:
                    ax.text(c[0] + .05, c[1] + s0 / 2 - .12, name, fontsize=7, color=STRUCT, zorder=6)
            continue          # the soffit (uv) would cover the plan view; drawn only in the side view
        if pl == "uv":
            ax.plot([c[0] - s0 / 2, c[0] + s0 / 2], [c[2], c[2]], **kw)
            if label and name not in done:
                ax.text(c[0] + s0 / 2 - .1, c[2] + .05, name, fontsize=7, color=STRUCT, ha="right", zorder=6)
        elif pl == "vz":
            ax.plot([c[0], c[0]], [c[2] - s1 / 2, c[2] + s1 / 2], **kw)
            if label and name not in done:
                ax.text(c[0] + .05, c[2] + s1 / 2 - .15, name, fontsize=7, color=STRUCT, zorder=6)
        done.add(name)
    (u0, v0), (u1, v1) = spec["lamp"]["footprint_route_uv"]
    (t0, w0), (t1, w1) = spec["table"]["top_route_uv"]
    if view == "top":
        ax.add_patch(Rectangle((u0, v0), u1 - u0, v1 - v0, fc=LAMP_C + "40", ec=LAMP_C, lw=1.5, zorder=3))
        ax.add_patch(Rectangle((t0, w0), t1 - t0, w1 - w0, fill=False, ec=TABLE_C, lw=1.5, ls="--", zorder=3))
        if label:
            ax.text(u1 + .03, v0 + .05, "lamp", fontsize=7, color="#5a4500", zorder=6)
            ax.text(t1 - .05, w1 + .05, "table top", fontsize=7, color=TABLE_C, ha="right", zorder=6)
    else:
        lz, tz = spec["lamp"]["underside_z"], spec["lamp"]["top_z"]
        ax.add_patch(Rectangle((u0, lz), u1 - u0, tz - lz, fc=LAMP_C + "60", ec=LAMP_C, lw=1.5, zorder=3))
        ax.plot([t0, t1], [spec["table"]["top_z"]] * 2, color=TABLE_C, lw=2.5, zorder=3)
        if label:
            ax.text(u0 - .05, lz - .12, f"lamp underside {lz:.2f} m", fontsize=7, color="#5a4500", ha="right", zorder=6)
            ax.text(t1 - .05, spec["table"]["top_z"] - .12, f"table top {spec['table']['top_z']:.2f} m", fontsize=7,
                    color=TABLE_C, ha="right", zorder=6)


def backdrop(ax, booth, spec, view, rng, drop=()):
    op = booth.opac > .3
    loc, role = booth.loc[op], booth.role[op]
    lo, hi = np.asarray(spec["box_route"]["lower"]), np.asarray(spec["box_route"]["upper"])
    inbox = np.all((loc >= lo - .4) & (loc <= hi + .4), axis=1) & (role == "")
    if view == "top":
        inbox &= (loc[:, 2] > .05) & (loc[:, 2] < 2.34)
    pick = np.flatnonzero(inbox)
    if len(pick) > 120_000:
        pick = rng.choice(pick, 120_000, replace=False)
    a, b = (0, 1) if view == "top" else (0, 2)
    ax.scatter(loc[pick, a], loc[pick, b], s=.4, c="#a3a29d", alpha=.35, lw=0, rasterized=True, zorder=1)
    ax.add_patch(Rectangle((lo[a], lo[b]), hi[a] - lo[a], hi[b] - lo[b], fill=False, ec="k", ls=":", lw=.8,
                           zorder=2))
    ax.set_xlim(lo[0] - .3, hi[0] + .3)
    ax.set_ylim((lo[1] - .3, hi[1] + .3) if view == "top" else (-.1, 2.6))
    ax.set_aspect("equal")
    ax.set_xlabel("u (m, along the gallery)")
    ax.set_ylabel("v (m, across)" if view == "top" else "z above floor (m)")
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


def route(ax, run, view, colour, label, knots=False, lw=2.4):
    a, b = (0, 1) if view == "top" else (0, 2)
    rt = run["route"]
    ax.plot(rt[:, a], rt[:, b], color=colour, lw=lw, label=label, zorder=5, solid_capstyle="round")
    if view == "side":
        ax.fill_between(rt[:, 0], rt[:, 2] - uv.H, rt[:, 2] + uv.H, color=colour, alpha=.12, lw=0, zorder=4)
    if knots and run.get("knots") is not None:
        k = run["knots"]
        ax.scatter(k[:, a], k[:, b], s=26, color=colour, edgecolors="white", linewidths=1.2, zorder=6)


def fig_routes(booth, cases, dst):
    fig, axes = plt.subplots(2, 2, figsize=(17, 8.4))
    rng = np.random.default_rng(0)
    for col, (title, spec, a3, lat) in enumerate(cases):
        for row, view in enumerate(("top", "side")):
            ax = axes[row, col]
            backdrop(ax, booth, spec, view, rng)
            structures(ax, booth.doc, spec, view)
            la = polyline_metrics(np.asarray(lat["traj"]["poses"], float)[:, :3])["path_length_m"]
            aa = polyline_metrics(np.asarray(a3["doc"]["result"]["polyline_world"], float))["path_length_m"]
            route(ax, lat, view, ORANGE, f"lattice A* (baseline): {la:.3f} m, {len(lat['traj']['poses'])} knots", lw=2)
            route(ax, a3, view, BLUE, f"aerial3d (new): {aa:.3f} m, {len(a3['knots'])} vertices", knots=True)
            st, gl = np.asarray(spec["start_route"]), np.asarray(spec["goal_route"])
            i, j = (0, 1) if view == "top" else (0, 2)
            ax.scatter([st[i]], [st[j]], s=70, c="#1baf7a", edgecolors="k", zorder=7, label="start")
            ax.scatter([gl[i]], [gl[j]], s=110, marker="*", c="#e34948", edgecolors="k", zorder=7, label="goal")
            if row == 0:
                ax.set_title(title, fontsize=10, loc="left")
            ax.legend(loc="upper right" if view == "top" else "upper left", fontsize=7.5, frameon=True,
                      framealpha=.9)
    fig.suptitle("Same start + goal, same archive: returned routes of both methods (both pass the same fresh gs3d "
                 "replay). Top: plan view (soffit omitted). Bottom: side view with body band (±0.10 m).",
                 fontsize=10, x=.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, .96))
    fig.savefig(dst, dpi=100)
    plt.close(fig)


def fig_necessity(booth, panels, dst):
    fig, axes = plt.subplots(2, 3, figsize=(19, 8.2))
    rng = np.random.default_rng(0)
    for col, (title, spec, run) in enumerate(panels):
        drop = tuple(spec.get("drop_roles", ()))
        plug = None
        for item in spec.get("extra_builders", []):
            plug = booth.frame.to_route(su.build_test_only(item, booth.frame).means)
        for row, view in enumerate(("top", "side")):
            ax = axes[row, col]
            backdrop(ax, booth, spec, view, rng, drop)
            structures(ax, booth.doc, spec, view, drop=drop)
            i, j = (0, 1) if view == "top" else (0, 2)
            if plug is not None:
                ax.scatter(plug[:, i], plug[:, j], s=2, c=PLUG_C, lw=0, zorder=3, label="test-only plug")
            if run.get("traj"):
                route(ax, run, view, BLUE, f"aerial3d: {run['evidence']['path_length_m']:.3f} m", knots=True)
            else:
                cert = run["doc"]["result"]["certificate"]
                cut = np.asarray(cert["cut_leaf_centres_plan"], float)
                if len(cut):
                    pf = json.loads(Path(run["doc"]["compile"]["file"]).read_text())["compile"]["frame"]
                    world = cut @ np.asarray(pf["world_to_plan"], float) + np.asarray(pf["origin_world_m"], float)
                    cut = booth.frame.to_route(world)
                    ax.scatter(cut[:, i], cut[:, j], s=3, c="#4a3aa7", alpha=.5, lw=0, zorder=4,
                               label="BLOCKED leaves on the cut (each inside one pair's inner polytope)")
                roles = run["doc"]["certificate_attribution"]["cut_pairs_by_role"]
                if view == "top":
                    ax.text(.73, .97,
                        f"certified UNREACHABLE (possible-space cut) in {run['algorithm_wall_s']:.2f} s\n"
                        f"{cert['blocked_leaves_on_cut']} cut leaves, {cert['cut_distinct_pairs']} distinct pairs:\n" +
                        ", ".join(f"{k} {v}" for k, v in sorted(roles.items())),
                        transform=ax.transAxes, ha="center", va="top", fontsize=7.5, color="#4a3aa7",
                        bbox=dict(fc="white", ec="#4a3aa7", lw=1), zorder=8)
            st, gl = np.asarray(spec["start_route"]), np.asarray(spec["goal_route"])
            ax.scatter([st[i]], [st[j]], s=70, c="#1baf7a", edgecolors="k", zorder=7, label="start")
            ax.scatter([gl[i]], [gl[j]], s=110, marker="*", c="#e34948", edgecolors="k", zorder=7, label="goal")
            if row == 0:
                ev = run.get("evidence")
                verdict = ("goes UNDER the lamp" if ev and ev["passes_under_lamp"] else
                           "goes OVER the lamp" if ev and ev["over_lamp_intervals"] else
                           "no path (certified)" if run["status"] == "UNREACHABLE" else run["status"])
                ax.set_title(f"{title}\naerial3d {run['status']} — {verdict}", fontsize=10, loc="left")
            ax.legend(loc="lower right", fontsize=7, framealpha=.9)
    fig.suptitle("aerial3d, same high start (−2.0, 1.2, 1.55) and goal (0.80, 0.55, 1.50) in all three panels; one "
                 "query call each on the scene variant's compile", fontsize=10, x=.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, .95))
    fig.savefig(dst, dpi=100)
    plt.close(fig)


def keyframes(booth, run, lat, spec, out_dir, manifest):
    idx, labels = uv.keyframe_indices(run)
    bodies = [run["world"][k] for k in idx]
    for name, v in uv.VIEWS.items():
        keep = v["crop"](booth.loc)
        cam = cam_from(booth.frame, v["eye"], v["target"], 1400, 800, fov=v["fov"])
        out, surface = uv.draw_frame(booth, booth.splats(keep), cam, bodies, run["world"])
        fig, ax = plt.subplots(figsize=(14, 8))
        ax.imshow(np.clip(out["rgb"], 0, 1))
        if name == "side":
            uv.overlay_path(ax, cam, surface, lat["world"], colour=ORANGE, lw=1.6,
                            label=f"lattice A* baseline route ({lat['evidence']['path_length_m']:.2f} m)")
        frac = uv.overlay_path(ax, cam, surface, run["world"], colour=BLUE,
                               label=f"aerial3d route, UAV centre ({run['evidence']['path_length_m']:.2f} m)")
        px = []
        for jj, (k, lab, c) in enumerate(zip(idx, labels, bodies)):
            u, vv, d = ewa.project_points(cam, c[None])
            px.append({"label": lab, "sample": int(k), "route_xyz": run["route"][k].round(4).tolist(),
                       "pixel_uv": [float(u[0]), float(vv[0])], "depth_m": float(d[0])})
            ax.annotate(f"{lab}\nz={run['route'][k][2]:.2f} m", (u[0], vv[0]),
                        xytext=(0, (-40 if jj % 2 == 0 else 34) * (1 if name != "top" else -1)),
                        textcoords="offset points", ha="center", fontsize=8,
                        bbox=dict(boxstyle="round,pad=.2", fc="white", ec="none", alpha=.8), zorder=8)
        ev = run["evidence"]
        ax.set_title(f"aerial3d (new connectivity backend) — M1 — EWA render of the planning archive — {name} view\n"
                     f"one query call (start + goal only) on the booth compile; orange bodies = UAV (r .25, h .20) at "
                     f"6 poses of the returned path; z {ev['z_range'][0]:.2f}–{ev['z_range'][1]:.2f} m", fontsize=10)
        ax.set_xlim(0, 1400); ax.set_ylim(800, 0); ax.axis("off")
        ax.legend(loc="lower right", fontsize=8)
        fig.tight_layout()
        dst = out_dir / f"aerial3d_M1_keyframes_{name}.png"
        fig.savefig(dst, dpi=100); plt.close(fig)
        manifest["keyframes"][name] = {"path": str(dst), "camera": cam.as_dict(), "eye_route": v["eye"],
                                       "target_route": v["target"], "n_splats": int(keep.sum()),
                                       "path_visible_fraction": frac, "bodies": px}
        print("keyframes", name, frac, flush=True)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--configs", type=Path, default=Path("configs/uavlamp_l2"))
    p.add_argument("--a3", type=Path, default=Path("outputs/uavconn/c2/aerial3d"))
    p.add_argument("--uavlamp-root", type=Path, default=Path("/scratch/wg2381/splathjb-uavlamp/gmc"))
    p.add_argument("--out", type=Path, default=Path("results/uavconn/final"))
    p.add_argument("--skip", nargs="*", default=[])
    p.add_argument("--video-seconds", type=float, default=12.)
    a = p.parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)
    specs = {n: json.loads((a.configs / f"{n}.json").read_text()) for n in
             ("M1_main_low_start", "M5_high_start", "N2h_plug_high_start", "C4h_lamp_only_high_start")}
    m1s = specs["M1_main_low_start"]
    booth = uv.Booth(a.uavlamp_root / m1s["archive"], a.uavlamp_root / m1s["manifest"])
    grp = {"M1_main_low_start": "booth", "M5_high_start": "booth", "N2h_plug_high_start": "plug",
           "C4h_lamp_only_high_start": "lamp"}
    a3 = {n: load_a3(a.a3 / grp[n] / f"l2_{n}" / "result.json", booth.frame, s) for n, s in specs.items()}
    lat = {n: uv.load_run(L2 / n / "result.json", booth.frame, specs[n]) for n in ("M1_main_low_start", "M5_high_start")}
    manifest = {"archive_sha256": booth.doc["derivative"]["sha256"], "keyframes": {},
                "runs": {**{f"aerial3d:{n}": r["path"] for n, r in a3.items()},
                         **{f"lattice:{n}": r["path"] for n, r in lat.items()}}}
    fig_routes(booth, [("M1 low start (−2.0, 1.2, 0.55) → (0.80, 0.55, 1.50)", m1s, a3["M1_main_low_start"],
                        lat["M1_main_low_start"]),
                       ("M5 high start (−2.0, 1.2, 1.55) → same goal", specs["M5_high_start"], a3["M5_high_start"],
                        lat["M5_high_start"])], a.out / "routes_M1_M5.png")
    print("routes done", flush=True)
    fig_necessity(booth, [("(a) lamp only (header, soffit, back panel removed)", specs["C4h_lamp_only_high_start"],
                           a3["C4h_lamp_only_high_start"]),
                          ("(b) designed booth", specs["M5_high_start"], a3["M5_high_start"]),
                          ("(c) designed booth + test-only plug under the lamp", specs["N2h_plug_high_start"],
                           a3["N2h_plug_high_start"])], a.out / "necessity_aerial3d.png")
    print("necessity done", flush=True)
    if "keyframes" not in a.skip:
        keyframes(booth, a3["M1_main_low_start"], lat["M1_main_low_start"], m1s, a.out, manifest)
    if "video" not in a.skip:
        vdir = a.out / "video"
        vdir.mkdir(exist_ok=True)
        uv.flythrough(booth, a3["M1_main_low_start"], m1s, vdir, manifest, seconds=a.video_seconds)
    for f in sorted(list(a.out.glob("*")) + list((a.out / "video").glob("*"))):
        if f.suffix in (".png", ".mp4"):
            manifest.setdefault("files", {})[str(f.relative_to(a.out))] = {"bytes": f.stat().st_size,
                                                                          "sha256": uv.sha256(f)}
    (a.out / "viz_manifest.json").write_text(json.dumps(manifest, indent=1, default=float) + "\n")
    print("done", flush=True)


if __name__ == "__main__":
    main()
