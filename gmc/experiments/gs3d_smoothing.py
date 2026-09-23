"""A6 constrained smoothing of the accepted A5 real-scene trajectories."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from gmc.gs3d.contracts import GoalRegion, Pose3
from gmc.gs3d.integration import ordered_uav_gate_evidence
from gmc.gs3d.oracle import GaussianBodyOracle
from gmc.gs3d.robots import CYLINDER, SWEEPER, UAV
from gmc.gs3d.scene import load_showcase_derivative
from gmc.gs3d.smoothing import SmoothingConfig, optimize_trajectory
from gmc.gs3d.timing import PlanTiming
from gs3d_integration import DEFAULT_LONG_CASE, _archive_meta, _ground_scene, _uav_scene


LIMITS = {
    "uav": {"max_speed_mps": .5, "max_vertical_speed_mps": .3,
            "max_yaw_rate_radps": 1., "max_acceleration_mps2": .5,
            "max_yaw_acceleration_radps2": 1.},
    "ground": {"max_speed_mps": .3, "max_vertical_speed_mps": 0.,
               "max_yaw_rate_radps": 1., "max_acceleration_mps2": .3,
               "max_yaw_acceleration_radps2": 1.},
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _load(path: Path) -> dict:
    return json.loads(path.read_text())


def _write(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n")


def _goal(row, tolerance=0.) -> GoalRegion:
    return GoalRegion(Pose3(tuple(map(float, row[:3])), float(row[3])),
                      position_tolerance_m=float(tolerance), yaw_tolerance_rad=.05)


def run(scene_path: Path, scene_manifest_path: Path, raw_dir: Path,
        acceptance_path: Path, output: Path, trace_path: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    accepted = _load(acceptance_path)
    expected = accepted["verification"]["raw_artifact_hashes"]
    raw_paths = {name: raw_dir / f"{name}.json" for name in ("uav", "sweeper", "cylinder")}
    actual = {name: _sha256(path) for name, path in raw_paths.items()}
    if any(actual[name] != expected[name] for name in actual):
        raise ValueError("A5 raw input hash mismatch; smoothing must use the frozen accepted cases")
    if _sha256(scene_path) != accepted["scene"]["archive_sha256"]:
        raise ValueError("scene archive differs from A5 collision/render authority")

    preparation_started = perf_counter()
    full, document = load_showcase_derivative(scene_path, scene_manifest_path)
    archive_meta = _archive_meta(scene_path)
    uav_timer = PlanTiming()
    uav_scene, uav_prepared = _uav_scene(full, document, uav_timer)
    ground_timer = PlanTiming()
    ground = _ground_scene(full, archive_meta, document, trace_path, ground_timer)
    ground_scene, ground_prepared = ground[:2]
    preparation_wall_s = perf_counter() - preparation_started

    raw_uav, raw_sweeper, raw_cylinder = (_load(raw_paths[name])
                                          for name in ("uav", "sweeper", "cylinder"))
    inputs = {
        "uav": (raw_uav["trajectory"], UAV, LIMITS["uav"], .05,
                _goal([*raw_uav["ordered_waypoints_world_xyz"][-1], 0.]), (1, 2)),
        "sweeper": (raw_sweeper["result"]["trajectory"], SWEEPER, LIMITS["ground"], .001,
                    _goal(raw_sweeper["original_goal"]), ()),
        "cylinder": (raw_cylinder["goal_sensitivity"]["cases"][0]["result"]["trajectory"],
                     CYLINDER, LIMITS["ground"], .001,
                     _goal(raw_cylinder["original_goal"]), ()),
    }
    outputs = {}
    for name, (trajectory, body, limits, margin, goal, protected) in inputs.items():
        oracle = GaussianBodyOracle(uav_prepared if name == "uav" else ground_prepared)
        result = optimize_trajectory(
            trajectory, oracle, body, limits, margin_m=margin, goal=goal,
            protected_position_indices=protected,
            config=SmoothingConfig(max_shortcut_span=64, max_certificate_depth=14,
                                   certificate_deviation_m=.005,
                                   min_turn_improvement_fraction=.10))
        payload = {"schema_version": "gs3d.a6-robot-smoothing.v1", "robot": name,
                   "scene_id": uav_scene.scene_id if name == "uav" else ground_scene.scene_id,
                   "seed": 0, "margin_m": margin, "body": vars(body), "limits": limits,
                   "original_goal": [*goal.original.xyz, goal.original.yaw],
                   "position_tolerance_m": goal.position_tolerance_m,
                   "raw_input_sha256": actual[name], "raw_trajectory": trajectory,
                   "smoothing": result}
        if name == "uav" and result.get("candidate_verification", {}).get("passed"):
            replay = {"passed": True, "geometry": {
                "clearance_lower_m": result["candidate_verification"]["clearance_lower_m"]}}
            gates = ordered_uav_gate_evidence(result["candidate_trajectory"], document, replay)
            payload["ordered_gate_evidence"] = gates
            if result["selected"] == "smoothed" and not gates["all_pass"]:
                result["selected"] = "raw_verified_fallback"
                result["reason"] = "ordered_uav_gate_revalidation_failed"
                result["trajectory"] = trajectory
        outputs[name] = payload
        _write(output / f"{name}.json", payload)

    selections = {name: payload["smoothing"]["selected"] for name, payload in outputs.items()}
    improvement = {name: payload["smoothing"].get("metrics", {}).get("turn_improvement_fraction")
                   for name, payload in outputs.items()}
    continuous = {name: bool(payload["smoothing"].get("candidate_verification", {}).get("passed"))
                  for name, payload in outputs.items()}
    all_success = all(payload["smoothing"]["success"] for payload in outputs.values())
    uav_order = bool(outputs["uav"].get("ordered_gate_evidence", {}).get("all_pass"))
    required_improvement = all(
        (not payload["smoothing"].get("metrics", {}).get("nontrivial_turn_case", False))
        or (payload["smoothing"].get("metrics", {}).get("turn_improvement_fraction", -np.inf) >= .10)
        for payload in outputs.values() if payload["smoothing"]["selected"] == "smoothed")
    manifest = {
        "schema_version": "gs3d.a6-smoothing.v1", "accepted": bool(
            all_success and selections["uav"] == "smoothed" and uav_order
            and any(value == "smoothed" for value in selections.values()) and required_improvement),
        "scene": {"archive": str(scene_path), "archive_sha256": _sha256(scene_path),
                  "same_a5_scene_authority": True, "projection_called": False,
                  "coverage_policy": "assumed_map_domain; not sensor-observed free space"},
        "inputs": {"raw_directory": str(raw_dir), "hashes": actual,
                   "same_cases_and_seed_as_a5": True, "seed": 0},
        "robots": {name: {"artifact": str(output / f"{name}.json"),
                           "selected": selections[name], "turn_improvement_fraction": improvement[name],
                           "candidate_continuous_verification": continuous[name],
                           "raw_fallback_verified": payload["smoothing"]["raw_fallback"]["verified"],
                           "optimizer_wall_s": payload["smoothing"]["optimizer_wall_s"]}
                   for name, payload in outputs.items()},
        "checks": {"frozen_a5_hashes_match": True, "same_3d_scene_oracle": True,
                   "uav_ordered_under_then_over_retained": uav_order,
                   "at_least_one_nontrivial_smoothed_route": any(v == "smoothed" for v in selections.values()),
                   "selected_nontrivial_turn_improvement_at_least_10pct": required_improvement,
                   "every_raw_fallback_verified": all(p["smoothing"]["raw_fallback"]["verified"] for p in outputs.values()),
                   "no_ground_altitude_search": True},
        "timing": {"preparation_wall_s": preparation_wall_s,
                   "smoothing_algorithm_wall_s": {name: outputs[name]["smoothing"]["optimizer_wall_s"] for name in outputs},
                   "physical_control_dt_s": {name: inputs[name][0]["control_dt_s"] for name in inputs},
                   "rendering_and_scheduler_queue_excluded": True},
        "limitations": [
            "Continuous collision claims are conservative floating-point tube bounds relative to the A5 map/oracle; they are not exact-arithmetic certificates.",
            "Curvature/turn diagnostics use finite sampling and do not authorize safety.",
            "The upright UAV model omits roll, pitch, wind, tracking error, actuator dynamics and closed-loop flight certification.",
            "Ground timing is acceleration-bounded stop-and-go spline execution, not a wheel-torque or traction proof.",
            "Coverage remains an assumed map domain rather than sensor-observed free space.",
        ],
    }
    _write(output / "manifest.json", manifest)
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--scene-manifest", type=Path, required=True)
    parser.add_argument("--raw-dir", type=Path, required=True)
    parser.add_argument("--acceptance", type=Path, default=Path("results/gs3d/integration_acceptance.json"))
    parser.add_argument("--trace", type=Path, default=DEFAULT_LONG_CASE)
    parser.add_argument("--output", type=Path, default=Path("results/gs3d/smoothing"))
    args = parser.parse_args(argv)
    manifest = run(args.scene, args.scene_manifest, args.raw_dir, args.acceptance,
                   args.output, args.trace)
    print(json.dumps({"accepted": manifest["accepted"], "checks": manifest["checks"],
                      "output": str(args.output)}, sort_keys=True))


if __name__ == "__main__":
    main()
