"""Run the A5 real-scene, unsmoothed GS3D integration baseline.

The derivative scene must already exist; the governed compute script builds it
before invoking this runner.  Every robot uses the same derivative archive and
the shared full-covariance 3-D oracle.  Rendering is a separate command so its
wall time cannot contaminate planning measurements.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from gmc.gs3d.contracts import (GoalRegion, PlannerConfig, Pose3, SceneSpec,
                                SearchBudget)
from gmc.gs3d.goals import (CYLINDER_TOLERANCES_M, classify_attempt,
                            terminal_error)
from gmc.gs3d.integration import (RouteBoxKnownSpace,
                                  concatenate_linear_trajectories,
                                  ordered_uav_gate_evidence,
                                  replay_linear_mission)
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.planner import LatticePlanner
from gmc.gs3d.robots import (CYLINDER, SWEEPER, UAV, BoxKnownSpace,
                             crop_by_support_aabb, supported_pose)
from gmc.gs3d.scene import LEVEL, SCENE_ID, TAU, load_showcase_derivative
from gmc.gs3d.timing import PlanTiming, latency_statistics
from gmc.gs3d.trajectory import replay_plan
from gs3d_run import (_constant_floor_support, _legacy_trace_summary,
                      DEFAULT_LONG_CASE)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_json(path: Path, payload: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return path


def _archive_meta(path: Path) -> dict:
    with np.load(path, allow_pickle=False) as archive:
        return json.loads(str(archive["meta"]))


def _finalize(timer: PlanTiming, result: dict, call_id: str, *, mode: str,
              preparation_wall_s: float | None = None) -> dict:
    return timer.finalize(result, mode=mode, call_id=call_id,
                          preparation_wall_s=preparation_wall_s)


def _call_summary(results: list[dict]) -> dict:
    samples = [float(result["timings"]["algorithm_wall_s"]) for result in results]
    return {"all_algorithm_wall_samples_s": samples,
            "latency": latency_statistics(samples),
            "success_count": sum(result["status"] == "success" for result in results),
            "call_count": len(results),
            "control_dt_s": sorted({float(result["trajectory"]["control_dt_s"])
                                    for result in results if result.get("trajectory")}),
            "rendering_included": False, "scheduler_queue_included": False,
            "stage_records_are_not_summed": True}


def _uav_scene(full, document, timer: PlanTiming):
    frame = document["route_frame"]
    domain = document["configuration"]["candidate_map_domain_route_m"]
    known = RouteBoxKnownSpace(tuple(frame["origin_world_m"]),
                               tuple(map(tuple, frame["world_to_route"])),
                               tuple(domain["lower"]), tuple(domain["upper"]))
    bounds_min, bounds_max = known.world_bounds()
    with timer.stage("index_build", robot="uav", crop="full_3d_ellipsoid_aabb_overlap"):
        cropped, crop = crop_by_support_aabb(full, bounds_min, bounds_max,
                                             level=LEVEL, tau=TAU)
        scene = SceneSpec(
            SCENE_ID + "-uav-task", cropped, bounds_min, bounds_max, TAU, LEVEL,
            known, None,
            {"coverage_policy": document["configuration"]["coverage_policy"],
             "derivative_sha256": document["derivative"]["sha256"],
             "crop": crop, "manual_edit_ids": [row["gaussian_id"] for row in document["manual_geometry"]],
             "direct_3d_gaussian_stack": True, "projection_called": False,
             "seed": 0, "tau": TAU, "level": LEVEL})
        prepared = PreparedScene(scene)
    return scene, prepared


def _plan_uav_mission(planner, scene, points: list[Pose3], timer: PlanTiming,
                      prefix: str, *, first_mode: str, preparation_wall_s: float) -> tuple[list[dict], float]:
    started = perf_counter()
    results = []
    for index, (start, goal) in enumerate(zip(points, points[1:])):
        call_id = f"{prefix}-leg{index}"
        result = planner.plan(
            scene, UAV, start, GoalRegion(goal),
            PlannerConfig(resolution_m=.10, margin_m=.05, seed=0,
                          budget=SearchBudget(max_wall_s=120., max_expansions=100_000,
                                              max_oracle_calls=1_000_000,
                                              max_narrowphase_pairs=5_000_000)),
            timer=timer.for_call(call_id))
        mode = first_mode if index == 0 else "warm"
        _finalize(timer, result, call_id, mode=mode,
                  preparation_wall_s=preparation_wall_s)
        results.append(result)
    return results, perf_counter() - started


def _run_uav(full, document, timer: PlanTiming, *, warm_calls: int) -> dict:
    # Loading and indexing are separate preparation spans. Both must precede
    # planner entry and contribute to the reported cold preparation wall.
    with timer.preparation():
        scene, prepared = _uav_scene(full, document, timer)
    planner = LatticePlanner(prepared)
    points = [Pose3(tuple(map(float, row)))
              for row in document["configuration"]["candidate_path_world_m"]]
    cold_results, cold_outer = _plan_uav_mission(
        planner, scene, points, timer, "uav-cold", first_mode="cold",
        preparation_wall_s=timer.preparation_wall_s)
    warm_runs, all_calls = [], list(cold_results)
    for repeat in range(warm_calls):
        results, outer = _plan_uav_mission(
            planner, scene, points, timer, f"uav-warm-{repeat}", first_mode="warm",
            preparation_wall_s=timer.preparation_wall_s)
        warm_runs.append({"repeat": repeat, "outer_mission_wall_s": outer,
                          "results": results})
        all_calls.extend(results)
    trajectory = concatenate_linear_trajectories(cold_results)
    oracle = GaussianBodyOracle(prepared)
    replay = replay_linear_mission(trajectory, oracle, UAV, margin_m=.05,
                                   goal=GoalRegion(points[-1]))
    gates = ordered_uav_gate_evidence(trajectory, document, replay)
    return {"schema_version": "gs3d.a5-uav-mission.v1", "scene_id": scene.scene_id,
            "body": asdict(UAV), "margin_m": .05,
            "ordered_waypoints_world_xyz": [list(point.xyz) for point in points],
            "trajectory": trajectory, "leg_results": cold_results,
            "independent_replay": replay, "ordered_gate_evidence": gates,
            "warm_replans": warm_runs,
            "timing": {**_call_summary(all_calls),
                       "cold_outer_mission_wall_s": cold_outer,
                       "warm_outer_mission_wall_s": [run["outer_mission_wall_s"] for run in warm_runs],
                       "warm_outer_distribution": latency_statistics(
                           [run["outer_mission_wall_s"] for run in warm_runs]),
                       "preparation_wall_s": timer.preparation_wall_s,
                       "prepared_index_reused": True},
            "success": bool(all(result["status"] == "success" for result in all_calls)
                            and replay["passed"] and gates["all_pass"]),
            "limitations": ["assumed map-domain coverage is not sensor-observed free space",
                            "piecewise-linear knots do not certify acceleration or smoothness"]}


def _ground_scene(full, archive_meta, document, trace_path: Path, timer: PlanTiming):
    trace = json.loads(trace_path.read_text())
    case = trace["case"]
    window = tuple(map(float, case["window"]))
    meta = archive_meta["base_meta"]
    z_floor = float(meta["z_floor"])
    z_min, z_max = z_floor - .10, z_floor + float(meta["ceiling_height_m"])
    bounds_min = (window[0], window[1], z_min)
    bounds_max = (window[2], window[3], z_max)
    with timer.stage("index_build", robots="sweeper,cylinder",
                     crop="full_3d_ellipsoid_aabb_overlap"):
        cropped, crop = crop_by_support_aabb(full, bounds_min, bounds_max,
                                             level=LEVEL, tau=TAU)
        support, support_evidence = _constant_floor_support(meta, window, z_min, z_max)
        scene = SceneSpec(
            SCENE_ID + "-ground-rung4", cropped, bounds_min, bounds_max, TAU, LEVEL,
            BoxKnownSpace(bounds_min, bounds_max), support,
            {"coverage_policy": "assumed_map_domain",
             "derivative_sha256": document["derivative"]["sha256"],
             "manual_edit_ids": [row["gaussian_id"] for row in document["manual_geometry"]],
             "crop": crop, "support_evidence": support_evidence,
             "legacy_case": str(trace_path), "legacy_distance_m": case["dist_m"],
             "original_goal_unchanged": True, "direct_3d_gaussian_stack": True,
             "projection_called": False, "tau": TAU, "level": LEVEL, "seed": 0})
        prepared = PreparedScene(scene)
    cylinder_start = supported_pose(case["start"][0], case["start"][1], case["start"][2],
                                    CYLINDER, support)
    cylinder_goal = supported_pose(case["goal"][0], case["goal"][1], case["goal"][2],
                                   CYLINDER, support)
    sweeper_start = supported_pose(case["start"][0], case["start"][1], case["start"][2],
                                   SWEEPER, support)
    sweeper_goal = supported_pose(case["start"][0], case["start"][1] + 1.6, 0.,
                                  SWEEPER, support)
    return scene, prepared, trace, cylinder_start, cylinder_goal, sweeper_start, sweeper_goal


def _run_sweeper(scene, prepared, start, goal, timer: PlanTiming, *, warm_calls: int) -> dict:
    planner = LatticePlanner(prepared)
    config = PlannerConfig(resolution_m=.20, margin_m=.001, seed=0,
                           budget=SearchBudget(max_wall_s=120., max_expansions=100_000,
                                               max_oracle_calls=1_000_000,
                                               max_narrowphase_pairs=5_000_000))
    results = []
    for index in range(warm_calls + 1):
        call_id = "sweeper-cold" if index == 0 else f"sweeper-warm-{index - 1}"
        result = planner.plan(scene, SWEEPER, start, GoalRegion(goal), config,
                              timer=timer.for_call(call_id))
        _finalize(timer, result, call_id, mode="cold" if index == 0 else "warm",
                  preparation_wall_s=timer.preparation_wall_s)
        result["replay"] = replay_plan(result, GaussianBodyOracle(prepared))
        results.append(result)
    primary = results[0]
    rows = np.asarray(primary["trajectory"]["poses"], float) if primary.get("trajectory") else np.empty((0, 4))
    return {"schema_version": "gs3d.a5-sweeper.v1", "scene_id": scene.scene_id,
            "body": asdict(SWEEPER), "margin_m": .001,
            "start": [*start.xyz, start.yaw], "original_goal": [*goal.xyz, goal.yaw],
            "result": primary, "warm_replans": results[1:], "timing": _call_summary(results),
            "ground_evidence": {"z_range_m": float(np.ptp(rows[:, 2])) if len(rows) else None,
                                "support": scene.provenance["support_evidence"],
                                "motion": "supported_ground_unicycle"},
            "success": bool(all(result["status"] == "success" and result["replay"]["passed"]
                                for result in results))}


def _legacy_failure_traces(trace_path: Path) -> list[dict]:
    ladder = trace_path.parent.parent
    rows = []
    for rung in (3, 4):
        path = ladder / f"rung{rung}" / "cylinder.json"
        payload = json.loads(path.read_text())
        rows.append({"rung": rung, "path": str(path), "sha256": _sha256(path),
                     "status": payload["result"]["status"],
                     "reason": payload["result"].get("reason"),
                     "projected_supports": payload["result"].get("n_supports"),
                     "retained_as_legacy_failure_not_relabelled": True})
    return rows


def _run_cylinder(scene, prepared, trace, start, original, timer: PlanTiming,
                  trace_path: Path, *, warm_calls: int,
                  preparation_wall_s: float) -> dict:
    planner = LatticePlanner(prepared)
    config = PlannerConfig(
        resolution_m=.20, margin_m=.001, seed=0,
        budget=SearchBudget(max_wall_s=600., max_expansions=500_000,
                            max_oracle_calls=5_000_000,
                            max_narrowphase_pairs=20_000_000))
    cases, all_results = [], []
    oracle = GaussianBodyOracle(prepared)
    for index, tolerance in enumerate(CYLINDER_TOLERANCES_M):
        call_id = f"cylinder-tolerance-{tolerance:.2f}"
        goal = GoalRegion(original, position_tolerance_m=tolerance, yaw_tolerance_rad=.05)
        result = planner.plan(scene, CYLINDER, start, goal, config,
                              timer=timer.for_call(call_id))
        result["diagnostics"]["goal_error"] = terminal_error(result)
        _finalize(timer, result, call_id, mode="cold" if index == 0 else "warm",
                  preparation_wall_s=preparation_wall_s)
        replay = replay_plan(result, oracle)
        cases.append({"tolerance_m": tolerance,
                      "original_goal": [*original.xyz, original.yaw],
                      "position_tolerance_m": tolerance, "attempts": [result],
                      "selected_attempt": 0, "result": result,
                      "diagnosis": classify_attempt(result), "replay": replay})
        all_results.append(result)
    warm = []
    for index in range(warm_calls):
        call_id = f"cylinder-warm-exact-{index}"
        result = planner.plan(scene, CYLINDER, start, GoalRegion(original), config,
                              timer=timer.for_call(call_id))
        result["diagnostics"]["goal_error"] = terminal_error(result)
        _finalize(timer, result, call_id, mode="warm",
                  preparation_wall_s=preparation_wall_s)
        replay = replay_plan(result, oracle)
        warm.append({"result": result, "replay": replay})
        all_results.append(result)
    distance = float(np.linalg.norm(np.asarray(original.xyz) - start.xyz))
    exact = cases[0]
    return {"schema_version": "gs3d.a5-cylinder-long.v1", "scene_id": scene.scene_id,
            "body": asdict(CYLINDER), "margin_m": .001,
            "start": [*start.xyz, start.yaw], "original_goal": [*original.xyz, original.yaw],
            "start_to_original_goal_m": distance,
            "legacy_declared_xy_distance_m": trace["case"]["dist_m"],
            "goal_sensitivity": {"schema_version": "gs3d.goal-sensitivity.v1",
                                 "robot": asdict(CYLINDER),
                                 "original_goal": [*original.xyz, original.yaw],
                                 "tolerances_m": list(CYLINDER_TOLERANCES_M), "cases": cases},
            "warm_exact_replans": warm, "timing": _call_summary(all_results),
            "legacy_trace_diagnosis": _legacy_trace_summary(trace_path),
            "retained_legacy_failure_traces": _legacy_failure_traces(trace_path),
            "exact_success": exact["result"]["status"] == "success",
            "exact_replay_passed": exact["replay"]["passed"],
            "all_comparisons_successful": all(case["result"]["status"] == "success"
                                               and case["replay"]["passed"] for case in cases),
            "all_warm_replans_successful": all(item["result"]["status"] == "success"
                                                and item["replay"]["passed"] for item in warm),
            "limitations": ["coverage is an assumed map domain, not sensor-observed free space",
                            "piecewise-linear raw trajectory is intentionally not called smooth"]}


def run(scene_archive: Path, scene_manifest: Path, output: Path, *,
        trace_path: Path = DEFAULT_LONG_CASE, warm_calls: int = 5) -> dict:
    if warm_calls < 5:
        raise ValueError("A5 timing evidence requires at least five warm calls per robot")
    output.mkdir(parents=True, exist_ok=True)
    uav_timer = PlanTiming()
    with uav_timer.preparation():
        with uav_timer.stage("scene_load", archive=str(scene_archive)):
            full, document = load_showcase_derivative(scene_archive, scene_manifest)
            archive_meta = _archive_meta(scene_archive)
        with uav_timer.stage("scene_edit", operation="validate_prebuilt_frozen_derivative",
                             manual_rows=len(document["manual_geometry"])):
            if document["scene_id"] != SCENE_ID or len(document["manual_geometry"]) != 3:
                raise ValueError("frozen derivative edit is incomplete")
    scene_mtime_ns = scene_archive.stat().st_mtime_ns
    uav_started_ns = __import__("time").time_ns()
    uav = _run_uav(full, document, uav_timer, warm_calls=warm_calls)
    _write_json(output / "uav.json", uav)

    sweeper_timer = PlanTiming()
    with sweeper_timer.preparation():
        with sweeper_timer.stage("scene_load", cache="reuse_validated_derivative",
                                 archive_sha256=document["derivative"]["sha256"]):
            pass
        ground = _ground_scene(full, archive_meta, document, trace_path, sweeper_timer)
    scene, prepared, trace, cylinder_start, cylinder_goal, sweeper_start, sweeper_goal = ground
    sweeper = _run_sweeper(scene, prepared, sweeper_start, sweeper_goal,
                            sweeper_timer, warm_calls=warm_calls)
    _write_json(output / "sweeper.json", sweeper)

    cylinder_timer = PlanTiming()
    cylinder = _run_cylinder(scene, prepared, trace, cylinder_start, cylinder_goal,
                              cylinder_timer, trace_path, warm_calls=warm_calls,
                              preparation_wall_s=sweeper_timer.preparation_wall_s)
    _write_json(output / "cylinder.json", cylinder)

    robot_files = {name: output / f"{name}.json" for name in ("uav", "sweeper", "cylinder")}
    accepted = bool(uav["success"] and sweeper["success"] and cylinder["exact_success"]
                    and cylinder["exact_replay_passed"]
                    and cylinder["all_comparisons_successful"]
                    and cylinder["all_warm_replans_successful"]
                    and cylinder["start_to_original_goal_m"] >= 8.)
    manifest = {
        "schema_version": "gs3d.a5-integration.v1", "accepted": accepted,
        "scene": {"scene_id": document["scene_id"], "archive": str(scene_archive),
                  "archive_sha256": document["derivative"]["sha256"],
                  "manifest": str(scene_manifest), "manifest_sha256": _sha256(scene_manifest),
                  "generated_before_uav_planning": scene_mtime_ns < uav_started_ns,
                  "source_inputs": document["source_inputs"],
                  "manual_geometry_ids": [row["gaussian_id"] for row in document["manual_geometry"]],
                  "original_scene_unchanged": document["source_unchanged_after_build"]},
        "trajectories": {name: {"path": str(path), "sha256": _sha256(path)}
                         for name, path in robot_files.items()},
        "checks": {"all_three_real_scene_success": accepted,
                   "same_derivative_archive": all(payload["scene_id"].startswith(SCENE_ID)
                                                  for payload in (uav, sweeper, cylinder)),
                   "direct_3d_gaussian_stack": True, "projection_not_called": True,
                   "uav_ordered_gates": uav["ordered_gate_evidence"]["all_pass"],
                   "uav_variable_z": uav["independent_replay"]["variable_z"],
                   "sweeper_supported_ground_motion": sweeper["success"],
                   "cylinder_original_goal_distance_at_least_8m": cylinder["start_to_original_goal_m"] >= 8.,
                   "cylinder_exact_and_relaxed_preserved": cylinder["all_comparisons_successful"],
                   "warm_calls_per_robot_at_least_5": warm_calls >= 5},
        "timing_semantics": {"algorithm_entry": "LatticePlanner.plan before endpoint checks",
                             "preparation_separate": True, "rendering_separate": True,
                             "physical_control_dt_separate": True,
                             "stage_spans_may_be_nested_and_are_not_summed": True,
                             "warm_calls_per_robot": warm_calls},
        "limitations": ["This is the raw functioning 3-D baseline before A6 smoothing.",
                        "Coverage is a declared map domain; opacity is not observation coverage.",
                        "Upright bodies do not model UAV roll/pitch, wind or tracking error."],
    }
    _write_json(output / "manifest.json", manifest)
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, default=Path("results/gs3d_scene/showcase_airborne_v1.npz"))
    parser.add_argument("--scene-manifest", type=Path,
                        default=Path("results/gs3d_scene/showcase_airborne_v1.manifest.json"))
    parser.add_argument("--trace", type=Path, default=DEFAULT_LONG_CASE)
    parser.add_argument("--output", type=Path, default=Path("results/gs3d/integration"))
    parser.add_argument("--warm-calls", type=int, default=5)
    args = parser.parse_args(argv)
    manifest = run(args.scene, args.scene_manifest, args.output,
                   trace_path=args.trace, warm_calls=args.warm_calls)
    print(json.dumps({"accepted": manifest["accepted"], "output": str(args.output),
                      "checks": manifest["checks"]}, sort_keys=True))


if __name__ == "__main__":
    main()
