"""EWA renders of the uav-lamp booth from the exact planning archive.

Source splats keep their captured DC colour (raw PLY row = Gaussian id); the
plane-floor edit rows are neutral grey; added edits use their manifest colour.
Views are cropped in the site frame so that walls do not occlude the booth.
Optionally overlays planned routes (``--route name=result.json``).
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from gmc.gs3d import scene_uavlamp as su
from gmc.gs3d.trajectory import sample_linear_trajectory
from gmc.height import ewa
from gmc.height.viz3d import load_dc_colors

PLY = Path("/scratch/wg2381/splathjb/splatc_atlas/data/gs_scenes/showcase/raw/point_cloud.ply")
N_PLY = 7_340_008


def cam_from(frame, eye_route, target_route, w, h, fov=55., near=.1):
    eye, tgt = frame.to_world(eye_route), frame.to_world(target_route)
    back = eye - tgt
    dist = float(np.linalg.norm(back))
    back /= dist
    az, el = math.degrees(math.atan2(back[1], back[0])), math.degrees(math.asin(back[2]))
    return ewa.camera(tgt, az, el, dist, w, h, fov_y_deg=fov, near=near)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--archive", type=Path, required=True)
    p.add_argument("--manifest", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--route", action="append", default=[], help="label=path/to/result.json")
    p.add_argument("--drop-roles", nargs="*", default=[])
    p.add_argument("--tag", default="scene")
    a = p.parse_args(argv)
    a.output.mkdir(parents=True, exist_ok=True)
    full, doc = su.load_uavlamp_derivative(a.archive, a.manifest)
    frame = su.Frame.from_dict(doc["frame"])
    loc = frame.to_route(full.means)
    region = (loc[:, 0] > -4.6) & (loc[:, 0] < 3.9) & (loc[:, 1] > -.7) & (loc[:, 1] < 3.0) \
        & (loc[:, 2] > -.15) & (loc[:, 2] < 3.2) & (full.opacity > .05)
    drop = np.zeros(len(full.ids), bool)
    for e in doc["edits"]:
        if e["role"] in a.drop_roles:
            drop |= (full.ids >= e["id_range"][0]) & (full.ids <= e["id_range"][1])
    region &= ~drop
    ids, loc_r = full.ids[region], loc[region]
    rgb = np.full((len(ids), 3), .62)
    real = ids < N_PLY
    rgb[real] = load_dc_colors(PLY, ids[real])
    for e in doc["edits"]:
        m = (ids >= e["id_range"][0]) & (ids <= e["id_range"][1])
        rgb[m] = e["color_rgb"]
    means, covs, opac = full.means[region], full.covs[region], full.opacity[region]
    routes = []
    for item in a.route:
        label, path = item.split("=", 1)
        r = json.loads(Path(path).read_text())["result"]
        if r.get("trajectory"):
            s = sample_linear_trajectory(r["trajectory"], dt_s=.05)
            routes.append((label, np.asarray(s["poses"], float)[:, :3]))
    q = doc["query"]
    views = {
        # From the approach, looking along +u at the booth entrance.
        "entrance": dict(eye=[-3.9, 1.35, 1.35], target=[.8, .9, 1.05], keep=np.ones(len(ids), bool)),
        # Wall B and everything above 3.0 m cut away; camera beyond wall B looking at -v.
        "cutaway_side": dict(eye=[-.3, 6.4, 1.5], target=[-.3, 1.0, 1.15], keep=(loc_r[:, 1] < 2.22)),
        # Wall B and the soffit cut away, high oblique.
        "cutaway_oblique": dict(eye=[-3.3, 5.2, 4.2], target=[.2, 1.0, 1.0],
                                keep=(loc_r[:, 1] < 2.22) & (loc_r[:, 2] < 2.34)),
    }
    manifest = {"archive_sha256": doc["derivative"]["sha256"], "tag": a.tag, "dropped_roles": a.drop_roles,
                "n_region": int(region.sum()), "views": {}}
    for name, v in views.items():
        cam = cam_from(frame, v["eye"], v["target"], 1200, 800)
        keep = v["keep"]
        splats = ewa.pack(means[keep], covs[keep], opac[keep], rgb[keep])
        out = ewa.render(splats, cam, bg=(.97, .97, .97), xray=0.)
        fig, ax = plt.subplots(figsize=(12, 8))
        ax.imshow(out["rgb"])
        for (label, pts), col in zip(routes, ["deepskyblue", "magenta", "orange", "lime"]):
            uu, vv, dd = ewa.project_points(cam, pts)
            ok = dd > cam.near
            ax.plot(uu[ok], vv[ok], "-", color=col, lw=2.5, label=label)
            ax.scatter(uu[ok][[0, -1]], vv[ok][[0, -1]], c=["lime", "red"], s=40, edgecolors="k", zorder=5)
        ax.set_xlim(0, 1200); ax.set_ylim(800, 0); ax.axis("off")
        ax.set_title(f"{a.tag} — EWA render of the planning archive — view '{name}'", fontsize=10)
        if routes:
            ax.legend(loc="lower right", fontsize=8)
        fig.tight_layout()
        dst = a.output / f"{a.tag}_{name}.png"
        fig.savefig(dst, dpi=100); plt.close(fig)
        manifest["views"][name] = {"path": str(dst), "camera": cam.as_dict(), "eye_route": v["eye"],
                                   "target_route": v["target"], "n_splats": int(keep.sum()),
                                   "n_visible": out.get("n_visible"), "n_drawn": out.get("n_drawn")}
        print(name, manifest["views"][name]["n_splats"], flush=True)
    (a.output / f"{a.tag}_render_manifest.json").write_text(json.dumps(manifest, indent=1))


if __name__ == "__main__":
    main()
