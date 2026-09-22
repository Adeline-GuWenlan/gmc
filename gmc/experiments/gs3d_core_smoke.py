"""A1 reproducible synthetic artifact and core scaling evidence.

Run --visual first on submitted compute; open PNGs and record visual_review.json.
Only then run --checks (and pytest) on submitted compute. No source assets needed.
The PNGs are analytic 3D ray renders of the same ellipsoids and finite cylinders
used by the oracle, not a projected occupancy raster. HTML retains interactive 3D.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from gmc.gs3d.contracts import GoalRegion, PlannerConfig, Pose3, SearchBudget
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.planner import LatticePlanner
from gs3d_core_fixtures import UAV, make_scene, under_over


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def _surface(mean, covariance, level):
    u = np.linspace(0, 2 * np.pi, 49)
    v = np.linspace(0, np.pi, 25)
    sphere = np.stack(np.broadcast_arrays(np.cos(u)[None, :] * np.sin(v)[:, None],
        np.sin(u)[None, :] * np.sin(v)[:, None], np.cos(v)[:, None]), axis=-1)
    return mean + level * sphere @ np.linalg.cholesky(covariance).T


def _ray_render(scene, rows, camera, target, output, title):
    """CPU vectorized analytic ellipsoid + capped-cylinder intersection viewer."""
    from PIL import Image, ImageDraw

    width, height = 960, 640
    eye = np.asarray(camera, float)
    forward = np.asarray(target) - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0, 0, 1.])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    x, y = np.meshgrid(np.linspace(-1, 1, width), np.linspace(1, -1, height))
    rays = forward + .40 * (x[..., None] * right * width / height + y[..., None] * up)
    rays /= np.linalg.norm(rays, axis=-1)[..., None]
    rays = rays.reshape(-1, 3)
    depth = np.full(len(rays), np.inf)
    pixels = np.tile([.94, .96, .98], (len(rays), 1))
    light = np.array([-.3, -.6, 1.])
    light /= np.linalg.norm(light)

    def paint(t, normals, colour):
        keep = (t > 1e-6) & (t < depth) & np.isfinite(t)
        if not np.any(keep):
            return
        normal = normals[keep]
        normal /= np.maximum(np.linalg.norm(normal, axis=1)[:, None], 1e-30)
        illumination = .35 + .65 * np.maximum(normal @ light, 0)
        pixels[keep] = np.asarray(colour) * illumination[:, None]
        depth[keep] = t[keep]

    def ellipsoid(mean, covariance, level, colour):
        inv = np.linalg.inv(covariance) / level ** 2
        offset = eye - mean
        A = np.einsum("ni,ij,nj->n", rays, inv, rays)
        B = 2 * rays @ inv @ offset
        C = offset @ inv @ offset - 1
        disc = B * B - 4 * A * C
        t = np.where(disc >= 0, (-B - np.sqrt(np.maximum(disc, 0))) / (2 * A), np.inf)
        safe_t = np.where(np.isfinite(t), t, 0.)
        normals = (offset + safe_t[:, None] * rays) @ inv
        paint(t, normals, colour)

    # Ground/context grid in world metres; outside the known workspace is pale.
    with np.errstate(divide="ignore", invalid="ignore"):
        floor_t = -eye[2] / rays[:, 2]
        ground = eye + floor_t[:, None] * rays
    keep = floor_t > 0
    depth[keep] = floor_t[keep]
    inside = np.all((ground[:, :2] >= scene.bounds_min[:2]) & (ground[:, :2] <= scene.bounds_max[:2]), axis=1)
    grid = np.any(np.mod(ground[:, :2] + .015, .5) < .03, axis=1)
    pixels[keep] = [.85, .87, .89]
    pixels[keep & inside] = [.77, .85, .82]
    pixels[keep & grid] *= .85
    for i, (mean, covariance) in enumerate(zip(scene.gaussians.means, scene.gaussians.covs)):
        ellipsoid(mean, covariance, scene.level, [.9, .56, .16] if i == 0 else [.52, .37, .23])
    # Executed linear path is blue. Display spheres at <= .035 m spacing so the
    # full polyline remains continuous visually; they are not collision bodies.
    samples = []
    for a, b in zip(rows, rows[1:]):
        count = max(2, int(np.ceil(np.linalg.norm(b[:3] - a[:3]) / .035)))
        samples.extend(np.linspace(a[:3], b[:3], count))
    for point in samples:
        ellipsoid(np.asarray(point), np.eye(3) * .014 ** 2, 1., [.03, .25, .88])

    indices = np.unique(np.linspace(0, len(rows) - 1, 7).astype(int))
    for row in rows[indices]:
        offset = eye - row[:3]
        A = np.sum(rays[:, :2] ** 2, axis=1)
        B = 2 * rays[:, :2] @ offset[:2]
        C = offset[:2] @ offset[:2] - UAV.radius_m ** 2
        disc = B ** 2 - 4 * A * C
        root = np.sqrt(np.maximum(disc, 0))
        with np.errstate(divide="ignore", invalid="ignore"):
            for t in ((-B - root) / (2 * A), (-B + root) / (2 * A)):
                points = offset + t[:, None] * rays
                valid = (disc >= 0) & (np.abs(points[:, 2]) <= UAV.half_height_m)
                normals = points.copy()
                normals[:, 2] = 0
                paint(np.where(valid, t, np.inf), normals, [.85, .08, .14])
            for sign in (-1, 1):
                t = (sign * UAV.half_height_m - offset[2]) / rays[:, 2]
                points = offset + t[:, None] * rays
                valid = np.sum(points[:, :2] ** 2, axis=1) <= UAV.radius_m ** 2
                paint(np.where(valid, t, np.inf), np.tile([0., 0., sign], (len(rays), 1)), [.95, .18, .21])
    image = Image.fromarray((np.clip(pixels.reshape(height, width, 3), 0, 1) * 255).astype("uint8"))
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, width, 46), fill="white")
    draw.text((15, 8), title, fill="black")
    draw.text((15, 25), "Orange: hanging Gaussian | Brown: low Gaussian | Red: finite UAV bodies | Blue: planned xyz", fill="black")
    image.save(output)


def visual(output):
    scene, start, goal = under_over()
    result = LatticePlanner().plan(scene, UAV, start, GoalRegion(goal), PlannerConfig(resolution_m=.2))
    write_json(output / "visual_plan.json", result)
    # A failure leaves explicit diagnostic evidence and no invented route.
    if result["trajectory"] is None:
        raise RuntimeError("visual fixture planning failed: " + result["reason"])
    rows = np.asarray(result["trajectory"]["poses"])
    write_json(output / "synthetic_scene.json", {
        "scene_id": scene.scene_id, "means": scene.gaussians.means.tolist(),
        "covariances": scene.gaussians.covs.tolist(), "ids": scene.gaussians.ids.tolist(),
        "level": scene.level, "tau": scene.tau, "bounds_min": scene.bounds_min,
        "bounds_max": scene.bounds_max, "body": asdict(UAV), "coverage_policy": scene.provenance["coverage_policy"]})
    _ray_render(scene, rows, [3.9, -7., 3.2], [0, 0, 1.05], output / "under_over_perspective.png", "A1 actual 3D Gaussian supports and planned swept-body path: perspective")
    _ray_render(scene, rows, [0, -8., 1.5], [0, 0, 1.2], output / "under_over_side.png", "A1 side camera: low passage, ascent and high passage")
    _ray_render(scene, rows, [3.5, -3., 7.], [0, 0, 1.], output / "under_over_overhead.png", "A1 high camera: constrained lateral corridor and body extent")
    import plotly.graph_objects as go
    figure = go.Figure()
    meshes = []
    for i, (mean, cov) in enumerate(zip(scene.gaussians.means, scene.gaussians.covs)):
        points = _surface(mean, cov, scene.level)
        figure.add_trace(go.Surface(x=points[:, :, 0], y=points[:, :, 1], z=points[:, :, 2],
            opacity=.55, showscale=False, colorscale=[[0, "orange" if i == 0 else "saddlebrown"], [1, "orange" if i == 0 else "saddlebrown"]]))
        meshes.append(points)
    figure.add_trace(go.Scatter3d(x=rows[:, 0], y=rows[:, 1], z=rows[:, 2], mode="lines+markers", line={"color": "blue", "width": 6}, name="actual xyz path"))
    for row in rows[np.unique(np.linspace(0, len(rows) - 1, 7).astype(int))]:
        theta = np.linspace(0, 2 * np.pi, 49)
        z = row[2] + np.array([-UAV.half_height_m, UAV.half_height_m])
        xx, zz = np.meshgrid(row[0] + UAV.radius_m * np.cos(theta), z)
        yy, _ = np.meshgrid(row[1] + UAV.radius_m * np.sin(theta), z)
        figure.add_trace(go.Surface(x=xx, y=yy, z=zz, showscale=False, opacity=.7,
                                   colorscale=[[0, "red"], [1, "red"]]))
        meshes.append(np.stack((xx, yy, zz), axis=-1))
    figure.update_layout(title="A1 direct 3D supports and finite body path (rotate/zoom)", scene={"aspectmode": "data"})
    figure.write_html(output / "under_over_3d.html", include_plotlyjs=True)
    # Native triangle mesh artifact for external 3D viewers; not just an image.
    vertices, faces = [], []
    for mesh in meshes:
        base = len(vertices)
        nrow, ncol = mesh.shape[:2]
        vertices.extend(mesh.reshape(-1, 3))
        for r in range(nrow - 1):
            for c in range(ncol - 1):
                a = base + r * ncol + c
                faces.extend(((a, a + 1, a + ncol), (a + 1, a + ncol + 1, a + ncol)))
    with (output / "under_over_mesh.ply").open("w") as stream:
        stream.write(f"ply\nformat ascii 1.0\nelement vertex {len(vertices)}\nproperty float x\nproperty float y\nproperty float z\nelement face {len(faces)}\nproperty list uchar int vertex_indices\nend_header\n")
        for v in vertices:
            stream.write(" ".join(map(str, v)) + "\n")
        for face in faces:
            stream.write("3 " + " ".join(map(str, face)) + "\n")
    write_json(output / "visual_artifacts.json", {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
               for p in output.iterdir() if p.suffix in (".png", ".ply", ".html")})


def checks(output):
    review = json.loads((output / "visual_review.json").read_text())
    if not review.get("opened"):
        raise RuntimeError("Open actual 3D render artifacts before scalar acceptance")
    plan = json.loads((output / "visual_plan.json").read_text())
    assert plan["status"] == "success" and plan["clearance_lower_m"] > .05
    assert plan["diagnostics"]["altitude_range_m"] >= .5
    scaling = []
    # Increasing spatial extent at fixed density isolates index scaling from
    # fundamentally impossible dense obstacles. All supports participate in BVH.
    for n in (1000, 10000, 100000):
        rng = np.random.default_rng(0)
        extent = (n / .1) ** (1 / 3)
        means = rng.uniform(-extent, extent, (n, 3))
        means[:, 2] += extent + 4.
        # A nearby nonblocking anisotropic support exercises candidate and narrowphase work.
        means[0] = [1.49, .30, 1.]
        axes = np.tile([.08, .08, .12], (n, 1))
        axes[0] = [.2, .03, .03]
        scene = make_scene(means, axes, lower=(-extent-1, -extent-1, 0),
                           upper=(extent+1, extent+1, 2*extent+5), name=f"scaling_{n}")
        t = perf_counter()
        prepared = PreparedScene(scene)
        preparation = perf_counter() - t
        oracle = GaussianBodyOracle(prepared)
        t = perf_counter()
        reports = [oracle.edge(Pose3((-1, 0, 1)), Pose3((1, 0, 1)), UAV, margin_m=.05) for _ in range(20)]
        query_s = perf_counter() - t
        assert all(r.occupancy == "free" for r in reports)
        result = LatticePlanner(prepared).plan(scene, UAV, Pose3((-1, 0, 1)), GoalRegion(Pose3((1, 0, 1))), PlannerConfig())
        assert result["status"] == "success"
        scaling.append({"supports": n, "preparation_wall_s": preparation, "20_edge_queries_wall_s": query_s,
                        "oracle_stats": oracle.stats, "planner_algorithm_wall_s": result["timings"]["algorithm_wall_s"],
                        "planner_diagnostics": result["diagnostics"], "prepared": prepared.stats})
    write_json(output / "scaling.json", scaling)
    scene, start, goal = under_over()
    sensitivities = []
    for resolution in (.2, .1, .05):
        result = LatticePlanner().plan(scene, UAV, start, GoalRegion(goal), PlannerConfig(resolution_m=resolution))
        write_json(output / f"under_over_{resolution:.2f}.json", result)
        assert result["status"] == "success", result["reason"]
        sensitivities.append({"resolution_m": resolution, "clearance_lower_m": result["clearance_lower_m"],
                              "altitude_range_m": result["diagnostics"]["altitude_range_m"],
                              "expansions": result["diagnostics"]["expansions"],
                              "wall_s": result["timings"]["algorithm_wall_s"]})
    write_json(output / "numerical_acceptance.json", {"under_over": sensitivities,
        "scaling_sizes": [r["supports"] for r in scaling], "visual_review": review,
        "fixed_z_impossible": {"low_centre_upper_bound_m": 2 - np.sqrt(.99) - .1 - .05,
                                "high_centre_lower_bound_m": .3 + .95 * np.sqrt(.99) + .1 + .05},
        "safety_scope": "numerical continuous separating bounds; declared synthetic map/body; no dynamics or physical certification"})


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--visual", action="store_true")
    group.add_argument("--checks", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    (visual if args.visual else checks)(args.output)
