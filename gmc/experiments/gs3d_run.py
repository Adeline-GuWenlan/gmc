"""A3 robot/goal entrypoints for synthetic checks and the real long cylinder case.

This runner intentionally has no rendering or timing implementation; A4/A5 add
those through the planner's existing hooks.  It uses only the direct GS3D stack.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import numpy as np

from gmc.gs3d.contracts import (GoalRegion, PlannerConfig, Pose3, SceneSpec,
                                SearchBudget)
from gmc.gs3d.goals import cylinder_goal_sensitivity, plan_with_refinement
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.planner import LatticePlanner
from gmc.gs3d.robots import (CYLINDER, SWEEPER, UAV, BoxKnownSpace,
                             EvidenceBoundedPlaneSupport, crop_by_support_aabb,
                             supported_pose)
from gmc.gs3d.trajectory import replay_plan
from gmc.height.planefloor import load_plane_scene
from gmc.height.ply3d import GaussianScene3D


DEFAULT_PLANE_SCENE = Path(
    "/scratch/wg2381/splathjb/gmc/outputs/height/plane/processed_planefloor.npz")
DEFAULT_LONG_CASE = Path("results/height/plane/long/cyl_ladder/rung4/cylinder.json")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_json(path, payload):
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return output


def _empty_gaussians(name="synthetic"):
    return GaussianScene3D(np.empty((0, 3)), np.empty((0, 3, 3)), np.empty(0),
                           np.empty(0, dtype=np.int64), name)


def _synthetic_scene():
    bounds_min, bounds_max = (-6., -2., -.1), (6., 2., 3.)
    known = BoxKnownSpace(bounds_min, bounds_max)
    support = EvidenceBoundedPlaneSupport((0., 0., 1.), (0., 0., 0.), known)
    # Tall central obstacle forces both ground bodies to take a supported detour.
    means = np.array([[0., 0., .9]])
    covs = np.array([np.diag((np.array([.15, .45, .8]) / 2.) ** 2)])
    gaussians = GaussianScene3D(means, covs, np.array([.9]), np.array([0]), "a3_synth")
    return SceneSpec("a3-synthetic-all-robots-v1", gaussians, bounds_min, bounds_max,
                     .3, 2., known, support,
                     {"coverage_policy": "synthetic_known_box", "seed": 0})


def run_synthetic() -> dict:
    scene = _synthetic_scene()
    prepared = PreparedScene(scene)
    planner = LatticePlanner(prepared)
    results = {}
    for body in (SWEEPER, CYLINDER):
        start = supported_pose(-1.2, 0., .6, body, scene.support)
        goal = supported_pose(1.2, 0., -.4, body, scene.support)
        policy = plan_with_refinement(planner, scene, body, start, GoalRegion(goal),
            PlannerConfig(resolution_m=.2, margin_m=.001), refinement_resolution_m=.1)
        result = policy["result"]
        policy["replay"] = replay_plan(result, GaussianBodyOracle(prepared))
        results[body.name] = policy

    # A ceiling-height ellipsoid blocks a straight low route and forces real z motion.
    means = np.array([[0., 0., .7]])
    covs = np.array([np.diag((np.array([.15, 1.5, .35]) / 2.) ** 2)])
    uav_scene = SceneSpec(
        "a3-synthetic-variable-z-v1",
        GaussianScene3D(means, covs, np.array([.9]), np.array([1]), "a3_uav"),
        (-2., -.5, .3), (2., .5, 2.2), .3, 2.,
        BoxKnownSpace((-2., -.5, .3), (2., .5, 2.2)), None,
        {"coverage_policy": "synthetic_known_box", "seed": 0})
    uav_prepared = PreparedScene(uav_scene)
    uav = plan_with_refinement(LatticePlanner(uav_prepared), uav_scene, UAV,
        Pose3((-1.5, 0., .7)), GoalRegion(Pose3((1.5, 0., .7))),
        PlannerConfig(resolution_m=.2, margin_m=.05), refinement_resolution_m=.1)
    uav["replay"] = replay_plan(uav["result"], GaussianBodyOracle(uav_prepared))
    results["uav"] = uav
    return {"schema_version": "gs3d.a3-synthetic.v1", "results": results,
            "all_success": all(item["result"]["status"] == "success" for item in results.values()),
            "uav_variable_z": uav["replay"]["variable_z"]}


def _legacy_trace_summary(long_result_path: Path) -> dict:
    ladder = long_result_path.parent.parent
    rows = []
    for rung in range(5):
        result_path = ladder / f"rung{rung}" / "cylinder.json"
        if not result_path.exists():
            continue
        payload = json.loads(result_path.read_text())
        result, case = payload["result"], payload["case"]
        precheck = case.get("precheck", {}).get("cylinder", {})
        rows.append({"rung": rung, "distance_m": case["dist_m"],
                     "projected_supports": result["n_supports"],
                     "legacy_status": result["status"], "legacy_reason": result.get("reason"),
                     "projected_endpoint_clear": bool(precheck.get("start_clear_ok")
                                                       and precheck.get("goal_clear_ok")),
                     "projected_raster_connected": precheck.get("connected"),
                     "query_seconds": result.get("query_seconds")})
    return {
        "rows": rows,
        "diagnosis": {
            "endpoint_map_status": "legacy projected prechecks say clear; they are not 3D coverage evidence",
            "budget_exhaustion": [row["rung"] for row in rows
                                  if row["legacy_reason"] == "query_support_budget_exhausted"],
            "connectivity": "legacy connectivity is projected-raster evidence only",
            "scaling": [{"rung": row["rung"], "supports": row["projected_supports"],
                         "query_seconds": row["query_seconds"]} for row in rows],
        }}


def _constant_floor_support(meta, window, z_min, z_max):
    floor = meta["floor"]
    z_floor = float(meta["z_floor"])
    fitted_normal = np.asarray(floor["normal"], float)
    fitted_point = np.asarray(floor["centroid"], float)
    corners = np.array([[x, y] for x in (window[0], window[2])
                        for y in (window[1], window[3])])
    fitted_z = fitted_point[2] - (
        fitted_normal[0] * (corners[:, 0] - fitted_point[0])
        + fitted_normal[1] * (corners[:, 1] - fitted_point[1])) / fitted_normal[2]
    maximum_deviation = float(np.max(np.abs(fitted_z - z_floor)))
    evidence = BoxKnownSpace((window[0], window[1], z_min),
                             (window[2], window[3], z_max))
    support = EvidenceBoundedPlaneSupport(
        (0., 0., 1.), (0., 0., z_floor), evidence,
        height_error_m=maximum_deviation, max_height_error_m=.05,
        max_travel_m=.05, max_slope_deg=5.)
    return support, {"contact_manifold": "constant_legacy_floor_reference",
                     "z_floor_m": z_floor,
                     "fitted_plane_max_corner_deviation_m": maximum_deviation,
                     "allowed_height_error_m": .05,
                     "tile_top_clamp_m": z_floor + .015,
                     "chassis_bottom_m": z_floor + .02}


def build_long_cylinder_scene(source: Path, trace_path: Path):
    trace = json.loads(trace_path.read_text())
    case = trace["case"]
    full, meta = load_plane_scene(source, name="showcase_planefloor_a3_long")
    window = tuple(map(float, case["window"]))
    z_floor = float(meta["z_floor"])
    z_min, z_max = z_floor - .10, z_floor + float(meta["ceiling_height_m"])
    bounds_min = (window[0], window[1], z_min)
    bounds_max = (window[2], window[3], z_max)
    cropped, crop = crop_by_support_aabb(full, bounds_min, bounds_max, level=2., tau=.3)
    support, support_evidence = _constant_floor_support(meta, window, z_min, z_max)
    provenance = {
        "coverage_policy": "assumed_map_domain",
        "source": str(source), "source_sha256": _sha256(source),
        "manual_scene_edit": meta.get("claims_boundary"),
        "crop": crop, "support_evidence": support_evidence,
        "legacy_case": str(trace_path), "legacy_distance_m": case["dist_m"],
        "original_goal_unchanged": True, "tau": .3, "level": 2., "seed": 0,
    }
    scene_id = "showcase-planefloor-a3-rung4-" + provenance["source_sha256"][:16]
    scene = SceneSpec(scene_id, cropped, bounds_min, bounds_max, .3, 2.,
                      BoxKnownSpace(bounds_min, bounds_max), support, provenance)
    start = supported_pose(case["start"][0], case["start"][1], case["start"][2],
                           CYLINDER, support)
    goal = supported_pose(case["goal"][0], case["goal"][1], case["goal"][2],
                          CYLINDER, support)
    return scene, start, goal, trace, meta


def run_long_cylinder(source: Path, trace_path: Path, *, resolution_m=.20,
                      refinement_resolution_m=.10, wall_s=600.) -> dict:
    scene, start, goal, trace, meta = build_long_cylinder_scene(source, trace_path)
    prepared = PreparedScene(scene)
    config = PlannerConfig(
        resolution_m=resolution_m, margin_m=.001, seed=0,
        budget=SearchBudget(max_wall_s=wall_s, max_expansions=500_000,
                            max_oracle_calls=5_000_000,
                            max_narrowphase_pairs=20_000_000))
    matrix = cylinder_goal_sensitivity(
        LatticePlanner(prepared), scene, start, goal, config, body=CYLINDER,
        refinement_resolution_m=refinement_resolution_m)
    oracle = GaussianBodyOracle(prepared)
    for case in matrix["cases"]:
        case["replay"] = replay_plan(case["result"], oracle)
    distance = float(np.linalg.norm(np.asarray(goal.xyz) - start.xyz))
    return {"schema_version": "gs3d.a3-cylinder-long.v1",
            "scene_id": scene.scene_id, "source": str(source),
            "source_sha256": scene.provenance["source_sha256"],
            "start": [*start.xyz, start.yaw], "original_goal": [*goal.xyz, goal.yaw],
            "start_to_original_goal_m": distance,
            "legacy_declared_xy_distance_m": trace["case"]["dist_m"],
            "body": asdict(CYLINDER), "margin_m": .001,
            "scene_provenance": scene.provenance,
            "prepared": prepared.stats,
            "legacy_trace_diagnosis": _legacy_trace_summary(trace_path),
            "goal_sensitivity": matrix,
            "exact_success": matrix["cases"][0]["result"]["status"] == "success",
            "any_relaxed_success": any(case["result"]["status"] == "success"
                                       for case in matrix["cases"][1:]),
            "all_successful_replays_pass": all(
                case["replay"]["passed"] for case in matrix["cases"]
                if case["result"]["status"] == "success"),
            "limitations": [
                "coverage is a declared assumed map domain, not sensor-observed free space",
                "the floor is the existing user-approved manual edit, not a reconstruction guarantee",
                "piecewise-linear knots do not establish acceleration bounds",
            ]}


def main(argv=None):
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    synthetic = sub.add_parser("synthetic")
    synthetic.add_argument("--output", required=True)
    long = sub.add_parser("cylinder-long")
    long.add_argument("--source", type=Path, default=DEFAULT_PLANE_SCENE)
    long.add_argument("--trace", type=Path, default=DEFAULT_LONG_CASE)
    long.add_argument("--output", required=True)
    long.add_argument("--resolution", type=float, default=.20)
    long.add_argument("--refinement-resolution", type=float, default=.10)
    long.add_argument("--wall-seconds", type=float, default=600.)
    args = parser.parse_args(argv)
    if args.command == "synthetic":
        payload = run_synthetic()
    else:
        payload = run_long_cylinder(args.source, args.trace,
                                    resolution_m=args.resolution,
                                    refinement_resolution_m=args.refinement_resolution,
                                    wall_s=args.wall_seconds)
    _write_json(args.output, payload)
    print(json.dumps({"output": args.output, "schema_version": payload["schema_version"],
                      "success": payload.get("all_success", payload.get("exact_success"))},
                     sort_keys=True))


if __name__ == "__main__":
    main()
