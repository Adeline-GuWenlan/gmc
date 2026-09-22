"""Render A5 trajectories against the exact derivative archive they planned on."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from gmc.gs3d.robots import crop_by_support_aabb
from gmc.gs3d.scene import LEVEL, TAU, load_showcase_derivative
from gmc.height import ewa


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _load(path: Path) -> dict:
    return json.loads(path.read_text())


def _colours(scene, z_floor, manual_rows):
    height = np.clip((scene.means[:, 2] - z_floor) / 5.4, 0., 1.)
    colors = plt.get_cmap("viridis")(height)[:, :3]
    for row in manual_rows:
        where = np.flatnonzero(scene.ids == row["gaussian_id"])
        if len(where):
            colors[where[0]] = row["color_rgb"]
    return colors


def _cylinder_wire(centre, radius, half_height, samples=64):
    angle = np.linspace(0., 2 * np.pi, samples)
    circles = []
    for z in (-half_height, half_height):
        circles.append(np.column_stack([centre[0] + radius * np.cos(angle),
                                        centre[1] + radius * np.sin(angle),
                                        np.full_like(angle, centre[2] + z)]))
    return circles


def _overlay_path(ax, camera, path, keyframes=()):
    u, v, depth = ewa.project_points(camera, path)
    shown = depth > camera.near
    ax.plot(u[shown], v[shown], "-", color="deepskyblue", lw=3,
            label="exported raw xyz trajectory")
    ax.scatter(u[shown][[0, -1]], v[shown][[0, -1]], c=["lime", "red"],
               s=42, edgecolors="black", zorder=5)
    projected = []
    for label, centre, color in keyframes:
        wires = _cylinder_wire(centre, .25, .10)
        for wire in wires:
            wu, wv, wd = ewa.project_points(camera, wire)
            visible = wd > camera.near
            ax.plot(wu[visible], wv[visible], color=color, lw=2.2, zorder=6)
        cu, cv, cd = ewa.project_points(camera, np.asarray([centre]))
        ax.scatter(cu, cv, c=color, s=50, edgecolors="black", zorder=7, label=label)
        projected.append({"label": label, "world_xyz": list(map(float, centre)),
                          "pixel_uv": [float(cu[0]), float(cv[0])],
                          "camera_depth_m": float(cd[0])})
    return projected


def _uav_renders(full, document, uav, output: Path):
    origin = np.asarray(document["route_frame"]["origin_world_m"], float)
    rotation = np.asarray(document["route_frame"]["world_to_route"], float)
    local = (full.means - origin) @ rotation.T
    cov_local = rotation @ full.covs @ rotation.T
    half = LEVEL * np.sqrt(np.maximum(np.einsum("nii->ni", cov_local), 0.))
    lo, hi = local - half, local + half
    crop_lo, crop_hi = np.array([-1., -1.4, -.1]), np.array([3.1, 1.4, 5.45])
    keep = (full.opacity > TAU) & np.all(hi >= crop_lo, axis=1) & np.all(lo <= crop_hi, axis=1)
    scene = full.subset(keep)
    z_floor = float(origin[2])
    splats = ewa.pack(scene.means, scene.covs, scene.opacity,
                      _colours(scene, z_floor, document["manual_geometry"]))
    path = np.asarray(uav["trajectory"]["poses"], float)[:, :3]
    low = np.array([0., 0., .65]) @ rotation + origin
    high = np.array([1.40, 0., 1.40]) @ rotation + origin
    keyframes = (("low body under light", low, "orange"),
                 ("high body over table", high, "magenta"))
    views = {"side": (np.array([9.6, 6.4, z_floor + 1.55]), -52., 4., 6.),
             "oblique": (np.array([9.6, 6.4, z_floor + 1.55]), -130., 16., 6.),
             "high": (np.array([9.6, 6.4, z_floor + 1.55]), -130., 38., 6.)}
    rendered = {}
    for name, (target, azimuth, elevation, distance) in views.items():
        camera = ewa.camera(target, azimuth, elevation, distance, 900, 620, fov_y_deg=43.)
        frame = ewa.render(splats, camera, bg=(.96, .96, .96), xray=.2)
        fig, ax = plt.subplots(figsize=(10, 7))
        ax.imshow(frame["rgb"])
        projected = _overlay_path(ax, camera, path, keyframes)
        ax.set_title("A5 actual derivative GS + exported UAV path\n"
                     f"{name}: low orange body precedes high magenta body")
        ax.legend(loc="lower right", fontsize=8); ax.axis("off"); fig.tight_layout()
        destination = output / f"uav_{name}.png"
        fig.savefig(destination, dpi=140); plt.close(fig)
        rendered[name] = {"path": str(destination), "sha256": _sha256(destination),
                          "camera": camera.as_dict(), "n_selected_gaussians": int(len(scene)),
                          "n_visible": frame["n_visible"], "n_drawn": frame["n_drawn"],
                          "projected_keyframes": projected}
    return rendered, scene


def _ground_render(full, document, sweeper, cylinder, output: Path):
    exact = cylinder["goal_sensitivity"]["cases"][0]["result"]
    crop = exact["provenance"]["crop"]
    scene, evidence = crop_by_support_aabb(full, crop["bounds_min"], crop["bounds_max"],
                                           level=LEVEL, tau=TAU)
    z_floor = float(exact["provenance"]["support_evidence"]["z_floor_m"])
    splats = ewa.pack(scene.means, scene.covs, scene.opacity,
                      _colours(scene, z_floor, document["manual_geometry"]))
    cylinder_path = np.asarray(exact["trajectory"]["poses"], float)[:, :3]
    sweeper_path = np.asarray(sweeper["result"]["trajectory"]["poses"], float)[:, :3]
    bounds_lo, bounds_hi = np.asarray(crop["bounds_min"]), np.asarray(crop["bounds_max"])
    target = (bounds_lo + bounds_hi) / 2
    target[2] = z_floor + .6
    camera = ewa.camera(target, -90., 67., 16., 1000, 720, fov_y_deg=48.)
    frame = ewa.render(splats, camera, bg=(.96, .96, .96), xray=.25)
    fig, ax = plt.subplots(figsize=(11, 8))
    ax.imshow(frame["rgb"])
    for path, color, label in ((cylinder_path, "red", "cylinder exact 8.58 m goal"),
                               (sweeper_path, "deepskyblue", "sweeper supported route")):
        u, v, depth = ewa.project_points(camera, path)
        shown = depth > camera.near
        ax.plot(u[shown], v[shown], color=color, lw=2.5, label=label)
        ax.scatter(u[shown][[0, -1]], v[shown][[0, -1]], c=["lime", color], s=36,
                   edgecolors="black", zorder=5)
    ax.set_title("A5 actual derivative GS: supported ground-robot trajectories\n"
                 "raw piecewise-linear paths; rendering does not imply smoothness")
    ax.legend(loc="lower right"); ax.axis("off"); fig.tight_layout()
    destination = output / "ground_overview.png"
    fig.savefig(destination, dpi=140); plt.close(fig)
    return {"path": str(destination), "sha256": _sha256(destination),
            "camera": camera.as_dict(), "selection": evidence,
            "n_visible": frame["n_visible"], "n_drawn": frame["n_drawn"]}, scene


def _interactive(document, uav, sweeper, cylinder, uav_scene, ground_scene, output: Path):
    import plotly.graph_objects as go

    figure = go.Figure()
    for label, scene, color in (("UAV crop GS means", uav_scene, "#777777"),
                                ("ground crop GS means", ground_scene, "#999999")):
        active = np.flatnonzero(scene.opacity > TAU)
        stride = max(1, int(np.ceil(len(active) / 20_000)))
        selected = active[::stride]
        means = scene.means[selected]
        figure.add_trace(go.Scatter3d(x=means[:, 0], y=means[:, 1], z=means[:, 2],
                                      mode="markers", name=label,
                                      marker={"size": 1.2, "color": color, "opacity": .18}))
    paths = (("UAV raw xyz", np.asarray(uav["trajectory"]["poses"], float), "#00aaff"),
             ("Sweeper raw xyz", np.asarray(sweeper["result"]["trajectory"]["poses"], float), "#22aa44"),
             ("Cylinder exact raw xyz", np.asarray(cylinder["goal_sensitivity"]["cases"][0]["result"]["trajectory"]["poses"], float), "#dd2222"))
    for name, rows, color in paths:
        figure.add_trace(go.Scatter3d(x=rows[:, 0], y=rows[:, 1], z=rows[:, 2], mode="lines+markers",
                                      name=name, line={"width": 6, "color": color},
                                      marker={"size": 3, "color": color}))
    for row in document["manual_geometry"]:
        mean = np.asarray(row["mean_world_m"], float)
        figure.add_trace(go.Scatter3d(x=[mean[0]], y=[mean[1]], z=[mean[2]], mode="markers",
                                      name=row["role"] + " centre",
                                      marker={"size": 7, "color": "orange"}))
    figure.update_layout(title="A5 exact derivative archive and exported raw 3-D trajectories",
                         scene={"aspectmode": "data", "xaxis_title": "world x (m)",
                                "yaxis_title": "world y (m)", "zaxis_title": "world z (m)"},
                         legend={"itemsizing": "constant"})
    destination = output / "trajectories_3d.html"
    figure.write_html(destination, include_plotlyjs="cdn")
    return {"path": str(destination), "sha256": _sha256(destination),
            "note": "Interactive context uses deterministic opacity-active mean subsampling; numeric safety comes only from the complete 3-D oracle replay."}


def render(scene_path: Path, scene_manifest_path: Path, integration_dir: Path,
           output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    full, document = load_showcase_derivative(scene_path, scene_manifest_path)
    uav = _load(integration_dir / "uav.json")
    sweeper = _load(integration_dir / "sweeper.json")
    cylinder = _load(integration_dir / "cylinder.json")
    uav_views, uav_scene = _uav_renders(full, document, uav, output)
    ground_view, ground_scene = _ground_render(full, document, sweeper, cylinder, output)
    interactive = _interactive(document, uav, sweeper, cylinder, uav_scene, ground_scene, output)
    manifest = {"schema_version": "gs3d.a5-render.v1",
                "scene_archive": str(scene_path), "scene_archive_sha256": _sha256(scene_path),
                "scene_manifest_sha256": _sha256(scene_manifest_path),
                "trajectory_hashes": {name: _sha256(integration_dir / f"{name}.json")
                                      for name in ("uav", "sweeper", "cylinder")},
                "uav_views": uav_views, "ground_view": ground_view,
                "interactive_3d": interactive,
                "numeric_visual_reconciliation": {
                    "uav_world_z_range_m": uav["ordered_gate_evidence"]["world_centre_z_range_m"],
                    "uav_floor_relative_z_range_m": uav["ordered_gate_evidence"]["floor_relative_centre_z_range_m"],
                    "uav_altitude_range_m": uav["ordered_gate_evidence"]["altitude_range_m"],
                    "plotted_uav_rows": len(uav["trajectory"]["poses"]),
                    "plotted_sweeper_rows": len(sweeper["result"]["trajectory"]["poses"]),
                    "plotted_cylinder_rows": len(cylinder["goal_sensitivity"]["cases"][0]["result"]["trajectory"]["poses"]),
                    "same_archive_for_planner_and_renderer": True,
                    "same_trajectory_json_for_replay_and_renderer": True},
                "limitations": ["Rendered interpolation is not a smoothness claim.",
                                "Interactive mean subsampling is context only; full supports remain in numeric validation."]}
    destination = output / "render_manifest.json"
    destination.write_text(json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, default=Path("results/gs3d_scene/showcase_airborne_v1.npz"))
    parser.add_argument("--scene-manifest", type=Path,
                        default=Path("results/gs3d_scene/showcase_airborne_v1.manifest.json"))
    parser.add_argument("--integration", type=Path, default=Path("results/gs3d/integration"))
    parser.add_argument("--output", type=Path, default=Path("results/gs3d/integration/render"))
    args = parser.parse_args(argv)
    manifest = render(args.scene, args.scene_manifest, args.integration, args.output)
    print(json.dumps({"render_manifest": str(args.output / "render_manifest.json"),
                      "uav_views": sorted(manifest["uav_views"]),
                      "interactive": manifest["interactive_3d"]["path"]}, sort_keys=True))


if __name__ == "__main__":
    main()
