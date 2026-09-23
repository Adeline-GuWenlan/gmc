"""Render A6 raw/smoothed paths in the exact A5 derivative Gaussian scene."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from gmc.gs3d.scene import LEVEL, TAU, load_showcase_derivative


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _load(path: Path) -> dict:
    return json.loads(path.read_text())


def _thin(points: np.ndarray, maximum: int = 20_000) -> np.ndarray:
    stride = max(1, int(np.ceil(len(points) / maximum)))
    return points[::stride]


def _selected(payload: dict) -> np.ndarray:
    return np.asarray(payload["smoothing"]["trajectory"]["poses"], float)[:, :3]


def _raw(payload: dict) -> np.ndarray:
    return np.asarray(payload["raw_trajectory"]["poses"], float)[:, :3]


def render(scene_path: Path, scene_manifest_path: Path, smoothing_dir: Path,
           output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    full, document = load_showcase_derivative(scene_path, scene_manifest_path)
    robots = {name: _load(smoothing_dir / f"{name}.json")
              for name in ("uav", "sweeper", "cylinder")}
    origin = np.asarray(document["route_frame"]["origin_world_m"], float)
    rotation = np.asarray(document["route_frame"]["world_to_route"], float)
    active = full.opacity > TAU
    local = (full.means - origin) @ rotation.T
    uav_mask = (active & np.all(local >= [-.8, -1.25, -.05], axis=1)
                & np.all(local <= [2.9, 1.25, 2.25], axis=1))
    uav_cloud = _thin(local[uav_mask])
    raw_uav = (_raw(robots["uav"]) - origin) @ rotation.T
    smooth_uav = (_selected(robots["uav"]) - origin) @ rotation.T

    ground_paths = np.vstack([_raw(robots["sweeper"]), _raw(robots["cylinder"]),
                              _selected(robots["sweeper"]), _selected(robots["cylinder"])])
    lower = ground_paths.min(axis=0) - [1., 1., .25]
    upper = ground_paths.max(axis=0) + [1., 1., 1.2]
    ground_mask = active & np.all(full.means >= lower, axis=1) & np.all(full.means <= upper, axis=1)
    ground_cloud = _thin(full.means[ground_mask])

    figure = plt.figure(figsize=(15, 7.5))
    uax = figure.add_subplot(121, projection="3d")
    if len(uav_cloud):
        uax.scatter(uav_cloud[:, 0], uav_cloud[:, 1], uav_cloud[:, 2], s=.45,
                    c=uav_cloud[:, 2], cmap="viridis", alpha=.13, rasterized=True)
    uax.plot(*raw_uav.T, "--", color="black", lw=1.5, label="A5 verified raw")
    uax.plot(*smooth_uav.T, color="deepskyblue", lw=3., label="A6 selected")
    for edit in document["manual_geometry"]:
        centre = (np.asarray(edit["mean_world_m"]) - origin) @ rotation.T
        uax.scatter(*centre, color="gold", edgecolor="black", s=55,
                    label=edit["role"] if edit["role"] == "hanging_light_shade" else None)
    uax.set(xlabel="route s (m)", ylabel="route lateral (m)", zlabel="height above floor (m)",
            title="UAV: under light, rise, then over table")
    uax.view_init(elev=23, azim=-61); uax.legend(loc="upper left", fontsize=8)

    gax = figure.add_subplot(122, projection="3d")
    if len(ground_cloud):
        gax.scatter(*ground_cloud.T, s=.45, c=ground_cloud[:, 2], cmap="cividis",
                    alpha=.13, rasterized=True)
    for name, color in (("sweeper", "royalblue"), ("cylinder", "crimson")):
        raw, selected = _raw(robots[name]), _selected(robots[name])
        gax.plot(*raw.T, "--", color=color, lw=1.3, alpha=.75,
                 label=f"{name} raw")
        gax.plot(*selected.T, color=color, lw=3., label=f"{name} A6 selected")
    gax.set(xlabel="world x (m)", ylabel="world y (m)", zlabel="world z (m)",
            title="Ground bodies: supported same-scene routes")
    gax.view_init(elev=58, azim=-84); gax.legend(loc="upper left", fontsize=8)
    figure.suptitle("A6 constrained smoothing — exact A5 derivative scene (visual context only)",
                    y=.985)
    figure.tight_layout(rect=(0., 0., 1., .91))
    overview = output / "raw_vs_smoothed_3d.png"
    figure.savefig(overview, dpi=160); plt.close(figure)

    figure, (ax0, ax1) = plt.subplots(1, 2, figsize=(14, 5.5))
    ax0.scatter(uav_cloud[:, 0], uav_cloud[:, 2], s=.35, c="0.45", alpha=.12,
                rasterized=True)
    ax0.plot(raw_uav[:, 0], raw_uav[:, 2], "k--", lw=1.5, label="raw")
    ax0.plot(smooth_uav[:, 0], smooth_uav[:, 2], color="deepskyblue", lw=3,
             label="selected")
    ax0.axvspan(-.2, .2, color="gold", alpha=.18, label="under-light gate")
    ax0.axvspan(.92, 1.88, color="magenta", alpha=.10, label="over-table gate")
    ax0.set(xlabel="route s (m)", ylabel="centre height above floor (m)",
            title="Altitude/order profile"); ax0.legend(fontsize=8); ax0.grid(alpha=.2)
    for name, color in (("sweeper", "royalblue"), ("cylinder", "crimson")):
        raw, selected = _raw(robots[name]), _selected(robots[name])
        ax1.plot(raw[:, 0], raw[:, 1], "--", color=color, lw=1.2,
                 label=f"{name} raw")
        ax1.plot(selected[:, 0], selected[:, 1], color=color, lw=2.8,
                 label=f"{name} selected")
    ax1.set_aspect("equal", adjustable="box"); ax1.set(xlabel="world x (m)", ylabel="world y (m)",
            title="Ground plan view"); ax1.legend(fontsize=8); ax1.grid(alpha=.2)
    figure.tight_layout()
    profiles = output / "trajectory_profiles.png"
    figure.savefig(profiles, dpi=160); plt.close(figure)

    manifest = {"schema_version": "gs3d.a6-render.v1",
                "scene_archive": str(scene_path), "scene_archive_sha256": _sha256(scene_path),
                "same_derivative_as_collision": True,
                "artifacts": {"raw_vs_smoothed_3d": {"path": str(overview), "sha256": _sha256(overview)},
                              "trajectory_profiles": {"path": str(profiles), "sha256": _sha256(profiles)}},
                "rows": {name: {"raw": len(_raw(payload)), "selected": len(_selected(payload)),
                                 "selection": payload["smoothing"]["selected"]}
                         for name, payload in robots.items()},
                "visualization_semantics": "Gaussian means are deterministic context only; safety comes from complete covariance/body tube verification.",
                "limitations": ["Static keyframes do not certify tracking or flight dynamics.",
                                "Context point clouds are deterministically thinned; numeric validation uses all indexed supports."]}
    (output / "render_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--scene-manifest", type=Path, required=True)
    parser.add_argument("--smoothing", type=Path, default=Path("results/gs3d/smoothing"))
    parser.add_argument("--output", type=Path, default=Path("results/gs3d/smoothing/render"))
    args = parser.parse_args(argv)
    manifest = render(args.scene, args.scene_manifest, args.smoothing, args.output)
    print(json.dumps({"artifacts": manifest["artifacts"], "output": str(args.output)}, sort_keys=True))


if __name__ == "__main__":
    main()
