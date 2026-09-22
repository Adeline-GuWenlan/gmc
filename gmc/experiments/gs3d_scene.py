"""Build, inspect and render the deterministic GS3D airborne showcase edit.

Run ``fixture`` first (tiny synthetic scale companion), then run ``build`` and
``render`` only through the GS3D compute helper for the 7M-Gaussian scene.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from gmc.gs3d.scene import (DEFAULT_SOURCE, LEVEL, SCENE_ID, build_showcase_derivative,
                            load_showcase_derivative, route_frame)
from gmc.height import ewa


def _paths(out: Path) -> tuple[Path, Path]:
    return out / "showcase_airborne_v1.npz", out / "showcase_airborne_v1.manifest.json"


def fixture(out: Path) -> None:
    """Tiny companion: precisely inspect manual scale/orientation without source data."""
    out.mkdir(parents=True, exist_ok=True)
    z_floor = -1.2271749593107995
    from gmc.gs3d.scene import manual_edits, candidate_knots_world
    edits = manual_edits(z_floor)
    pts = candidate_knots_world(z_floor)
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection="3d")
    for x in edits:
        c = np.asarray(x.mean_world_m); a = np.asarray(x.semiaxes_route_m)
        th, ph = np.linspace(0, 2 * np.pi, 40), np.linspace(0, np.pi, 20)
        local = np.stack([a[0] * np.outer(np.cos(th), np.sin(ph)),
                          a[1] * np.outer(np.sin(th), np.sin(ph)),
                          a[2] * np.outer(np.ones_like(th), np.cos(ph))], axis=-1)
        # Route horizontal axes are rotated in the world frame; draw that exact orientation.
        _, R = route_frame(z_floor)
        world = local @ R + c
        ax.plot_surface(world[..., 0], world[..., 1], world[..., 2], color=x.color_rgb, alpha=.55, linewidth=0)
    ax.plot(*pts.T, color="deepskyblue", lw=3, label="candidate centre path")
    ax.set(xlabel="world x (m)", ylabel="world y (m)", zlabel="world z (m)", title="A2 synthetic scale companion — manual edit dimensions")
    ax.legend(); fig.tight_layout(); fig.savefig(out / "tiny_companion_scale.png", dpi=150); plt.close(fig)
    (out / "tiny_companion_scale.json").write_text(json.dumps({"scene_id": SCENE_ID, "z_floor": z_floor,
        "manual_geometry": [x.__dict__ for x in edits], "candidate_path_world_m": pts.tolist(),
        "note": "Synthetic scale inspection only; no source Gaussian or planning claim."}, indent=2) + "\n")


def build(out: Path, source: Path) -> None:
    archive, manifest = _paths(out)
    doc = build_showcase_derivative(source, archive, manifest)
    print(json.dumps({"archive": str(archive), "manifest": str(manifest),
                      "hash": doc["derivative"]["sha256"], "all_legs_pass": doc["feasibility_witness"]["all_legs_pass"],
                      "table_support_count": doc["existing_table_arrangement"]["count"]}, indent=2))


def render(out: Path) -> None:
    """Actual EWA GS renders from the archive consumed by downstream planners."""
    archive, manifest = _paths(out)
    scene, doc = load_showcase_derivative(archive, manifest)
    z_floor = float(doc["configuration"]["candidate_path_world_m"][0][2] - .65)
    origin, R = route_frame(z_floor)
    # Crop uses complete 2-sigma AABB overlap, not centres, and includes all manual obstacles.
    local = (scene.means - origin) @ R.T
    cov_local = R @ scene.covs @ R.T
    half = LEVEL * np.sqrt(np.maximum(np.einsum("nii->ni", cov_local), 0))
    lo, hi = local - half, local + half
    crop_lo, crop_hi = np.array([-1.0, -1.4, -.1]), np.array([3.1, 1.4, 5.45])
    keep = np.all(hi >= crop_lo, axis=1) & np.all(lo <= crop_hi, axis=1)
    sub = scene.subset(keep)
    # Height colours make original geometry readable without pretending source DC values exist for manual rows.
    h = np.clip((sub.means[:, 2] - z_floor) / 5.4, 0, 1)
    colors = plt.get_cmap("viridis")(h)[:, :3]
    for row in doc["manual_geometry"]:
        k = np.flatnonzero(sub.ids == row["gaussian_id"])
        if len(k): colors[k[0]] = row["color_rgb"]
    splats = ewa.pack(sub.means, sub.covs, sub.opacity, colors)
    path = np.asarray(doc["configuration"]["candidate_path_world_m"])
    views = {"oblique": (np.array([9.6, 6.4, z_floor + 1.55]), -130., 16., 6.0),
             "side": (np.array([9.6, 6.4, z_floor + 1.55]), -52., 4., 6.0),
             "high": (np.array([9.6, 6.4, z_floor + 1.55]), -130., 38., 6.0)}
    rendered = {}
    for name, (target, azim, elev, dist) in views.items():
        cam = ewa.camera(target, azim, elev, dist, 900, 620, fov_y_deg=43.)
        frame = ewa.render(splats, cam, bg=(.96, .96, .96), xray=.2)
        rgb = frame["rgb"]
        fig, ax = plt.subplots(figsize=(10, 7))
        ax.imshow(rgb)
        u, v, depth = ewa.project_points(cam, path)
        shown = depth > cam.near
        ax.plot(u[shown], v[shown], "-", color="deepskyblue", lw=2.5, label="candidate centre path")
        ax.scatter(u[shown][[0, -1]], v[shown][[0, -1]], c=["lime", "red"], s=42, edgecolors="black", zorder=3)
        ax.set_title(f"A2 {name}: actual EWA Gaussian render + candidate witness\nmanual light is gold; original table remains source geometry")
        ax.legend(loc="lower right"); ax.axis("off"); fig.tight_layout()
        p = out / f"showcase_airborne_{name}.png"; fig.savefig(p, dpi=140); plt.close(fig)
        rendered[name] = {"path": str(p), "camera": cam.as_dict(),
                          "n_visible": frame["n_visible"], "n_drawn": frame["n_drawn"],
                          "tile_gaussian_pairs": frame["n_pairs"]}
    review = {"scene_id": SCENE_ID, "archive_sha256": doc["derivative"]["sha256"], "n_rendered_gaussians": int(len(sub)),
              "selection": "2-sigma world Gaussian AABB overlaps route-frame render crop", "views": rendered,
              "note": "Actual EWA render of the same derivative archive validated by manifest; blue path is a feasibility witness, not planner output."}
    (out / "render_manifest.json").write_text(json.dumps(review, indent=2) + "\n")
    print(json.dumps(review, indent=2))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("step", choices=("fixture", "build", "render"))
    ap.add_argument("--out", type=Path, default=Path("results/gs3d_scene"))
    ap.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    a = ap.parse_args()
    {"fixture": lambda: fixture(a.out), "build": lambda: build(a.out, a.source), "render": lambda: render(a.out)}[a.step]()


if __name__ == "__main__":
    main()
