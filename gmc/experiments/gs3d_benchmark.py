#!/usr/bin/env python3
"""Reproducible small GS3D cold/warm benchmark (three physical robot bodies)."""
from __future__ import annotations

import argparse
import json
import platform
import sys
from pathlib import Path

from gmc.gs3d.contracts import GoalRegion, PlannerConfig, Pose3
from gmc.gs3d.oracle import PreparedScene
from gmc.gs3d.planner import LatticePlanner
from gmc.gs3d.timing import PlanTiming, latency_statistics

from gs3d_core_fixtures import CYLINDER, SWEEPER, UAV, FlatSupport, make_scene, under_over


def _case(robot: str):
    if robot == "uav":
        scene, start, goal = under_over()
        return scene, UAV, start, GoalRegion(goal), PlannerConfig(resolution_m=.2)
    body = SWEEPER if robot == "sweeper" else CYLINDER
    z = body.half_height_m + body.ground_clearance_m
    scene = make_scene([(0., .55, .8)], [(.12, .20, .70)], support=FlatSupport(),
                       name="timing_ground_shared_3d")
    return scene, body, Pose3((-.8, 0., z)), GoalRegion(Pose3((.8, 0., z))), \
        PlannerConfig(resolution_m=.2, margin_m=.001)


def _sizes(scene, result):
    return {"gaussian_count": int(len(scene.gaussians.means)),
            "expansions": int(result["diagnostics"].get("expansions", 0)),
            "oracle_calls": int(result["diagnostics"].get("oracle_calls", 0)),
            "narrowphase_pairs": int(result["diagnostics"].get("narrowphase_pairs", 0))}


def benchmark_robot(robot: str, *, warm_calls: int = 5) -> dict:
    """One prepared cold call followed by at least five cache-reusing replans."""
    if warm_calls < 5:
        raise ValueError("warm_calls must be at least 5 for the GS3D benchmark")
    cold_timing = PlanTiming()
    with cold_timing.preparation():
        with cold_timing.stage("scene_load", call_id=f"{robot}-cold"):
            scene, body, start, goal, config = _case(robot)
        with cold_timing.stage("index_build", call_id=f"{robot}-cold", gaussians=len(scene.gaussians.means)):
            prepared = PreparedScene(scene)
    planner = LatticePlanner(prepared)
    cold = planner.plan(scene, body, start, goal, config, timer=cold_timing.for_call(f"{robot}-cold"))
    cold_timing.finalize(cold, mode="cold", call_id=f"{robot}-cold")
    warm_timing = PlanTiming()
    warm = []
    for index in range(warm_calls):
        call_id = f"{robot}-warm-{index}"
        result = planner.plan(scene, body, start, goal, config, timer=warm_timing.for_call(call_id))
        warm_timing.finalize(result, mode="warm", call_id=call_id, preparation_wall_s=0.0)
        warm.append(result)
    for result in [cold, *warm]:
        if result["status"] != "success":
            raise RuntimeError(f"{robot} benchmark plan failed: {result['reason']}")
    trajectory = cold["trajectory"]
    return {"robot": robot, "body": cold["robot"], "seed": config.seed,
            "cache": {"cold": "new scene + PreparedScene index", "warm": "same SceneSpec and PreparedScene index"},
            "cold": {"status": cold["status"], "timing": cold["timings"], "sizes": _sizes(scene, cold)},
            "warm": {"calls": warm_calls, "status_counts": {"success": len(warm)},
                     "algorithm_wall_samples_s": [r["timings"]["algorithm_wall_s"] for r in warm],
                     "latency": latency_statistics([r["timings"]["algorithm_wall_s"] for r in warm]),
                     "sizes_last": _sizes(scene, warm[-1])},
            # Control/replay timing is physical semantics, never benchmark wall time.
            "physical_control": {"control_dt_s": trajectory["control_dt_s"],
                                 "trajectory_duration_s": trajectory["time_s"][-1],
                                 "render_fps": None}}


def run(*, warm_calls: int = 5) -> dict:
    return {"schema_version": "gs3d.benchmark.v1", "seed": 0,
            "environment": {"python": sys.version.split()[0], "platform": platform.platform(),
                            "clock": "time.perf_counter monotonic wall clock", "gpu_synchronized": False},
            "definitions": {"algorithm_wall_s": "Planner.plan entry through result assembly; excludes preparation/export/render/queue",
                            "preparation_wall_s": "scene construction/load and PreparedScene index build",
                            "physical_control_dt_s": "trajectory control period, not algorithm timing"},
            "robots": [benchmark_robot(name, warm_calls=warm_calls) for name in ("uav", "sweeper", "cylinder")]}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--warm-calls", type=int, default=5)
    args = parser.parse_args()
    payload = run(warm_calls=args.warm_calls)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
