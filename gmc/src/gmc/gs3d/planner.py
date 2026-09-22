"""Sparse bounded 26-neighbour xyz / 8-neighbour supported xy A*.

No SE(2) compilation, global occupancy raster, or point-only collision authority.
Every graph edge and exact connector uses the shared swept-body oracle; exported
paths are independently revalidated. A3 owns richer robot/goal/trajectory adapters.
"""
from __future__ import annotations

from contextlib import nullcontext
from dataclasses import asdict
import heapq
from itertools import count, product
import math
from time import perf_counter

import numpy as np

from .contracts import BodySpec, GoalRegion, PlannerConfig, Pose3, SceneSpec
from .oracle import (BudgetExceeded, GaussianBodyOracle, PreparedScene, QueryBudget,
                     validate_body, validate_pose)
from .validation import (GOAL_ROUNDOFF_M, angle_delta, validate_goal,
                         verify_linear_trajectory, verify_path)


def _row(q):
    return (*map(float, q.xyz), float(q.yaw))


def _limits(body):
    uav = body.motion == "uav_translation"
    return {"max_speed_mps": .5 if uav else .3,
            "max_vertical_speed_mps": .3 if uav else 0., "max_yaw_rate_radps": 1.,
            "max_acceleration_mps2": .5 if uav else .3, "max_yaw_acceleration_radps2": 1.}


def _linear_trajectory(path, body, start_yaw, end_yaw):
    """Minimal contract-complete export, with explicit stopped ground turns."""
    limits = _limits(body)
    poses = [Pose3(path[0].xyz, start_yaw)]
    times = [0.]

    def append(q, seconds):
        if seconds > 1e-12:
            poses.append(q)
            times.append(times[-1] + seconds)

    for q in path[1:]:
        previous = poses[-1]
        delta = np.asarray(q.xyz) - previous.xyz
        length = float(np.linalg.norm(delta))
        if length <= 1e-12:
            continue
        yaw = previous.yaw
        if body.motion == "ground_unicycle":
            heading = math.atan2(delta[1], delta[0])
            turn = angle_delta(heading, yaw)
            yaw += turn
            append(Pose3(previous.xyz, yaw), abs(turn) / limits["max_yaw_rate_radps"])
        duration = length / limits["max_speed_mps"]
        if body.motion == "uav_translation":
            duration = max(duration, abs(float(delta[2])) / limits["max_vertical_speed_mps"])
        append(Pose3(q.xyz, yaw), duration)
    turn = angle_delta(end_yaw, poses[-1].yaw)
    append(Pose3(poses[-1].xyz, poses[-1].yaw + turn), abs(turn) / limits["max_yaw_rate_radps"])
    return poses, {"poses": [_row(q) for q in poses], "time_s": times,
                   "interpolation": "linear_xyz_yaw", "segments": [], "control_dt_s": .05}


class LatticePlanner:
    """Reusable preparation; each plan resets all query counters and sparse search.

    Pass ``PreparedScene(scene)`` to exclude preparation from algorithm timing.
    Unprepared calls also work; their index build is inside algorithm wall time,
    flagged in diagnostics, and is not double-counted as outer preparation time.
    Prepared reuse requires the identical SceneSpec object (including coverage).
    """

    def __init__(self, prepared: PreparedScene | None = None):
        self.prepared = prepared

    def plan(self, scene: SceneSpec, body: BodySpec, start: Pose3, goal: GoalRegion,
             config: PlannerConfig, *, timer=None):
        entered = perf_counter()
        diagnostics = {"seed": config.seed, "expansions": 0, "oracle_calls": 0,
                       "narrowphase_pairs": 0, "termination": "initializing",
                       "resolution_m": config.resolution_m, "margin_m": config.margin_m,
                       "coverage_policy": scene.provenance.get("coverage_policy", "explicit_provider_unspecified"),
                       "config": asdict(config), "frontier_peak": 0, "visited_nodes": 0,
                       "map_unknown_rejections": 0, "unproven_rejections": 0,
                       "occupied_rejections": 0, "goal_candidates_checked": 0,
                       "connectivity": 26 if body.motion == "uav_translation" else 8,
                       "goal_roundoff_m": GOAL_ROUNDOFF_M,
                       "preparation_in_algorithm": self.prepared is None}
        result = {"schema_version": "gs3d.v1", "scene_id": scene.scene_id,
                  "robot": {**asdict(body), "limits": _limits(body)},
                  "status": "invalid_input", "reason": "initializing", "safety": "invalid",
                  "original_goal": _row(goal.original), "attained_goal": None,
                  "position_tolerance_m": goal.position_tolerance_m,
                  "yaw_tolerance_rad": goal.yaw_tolerance_rad,
                  "trajectory": None, "clearance_lower_m": None, "endpoint_reports": {},
                  "diagnostics": diagnostics, "provenance": dict(scene.provenance),
                  "timings": {"algorithm_wall_s": 0., "preparation_wall_s": 0.,
                              "mode": "warm" if self.prepared is not None else "cold",
                              "records": [], "replans_s": []}}
        oracle = None

        def stage(name, **sizes):
            return timer.stage(name, **sizes) if timer is not None else nullcontext()

        def finish(status, reason, safety="unresolved"):
            result.update(status=status, reason=reason, safety=safety)
            diagnostics["termination"] = reason
            if oracle is not None:
                diagnostics.update(oracle.stats)
                diagnostics["preparation"] = oracle.prepared.stats
            # Result assembly is included. External export/render are not.
            assembled = _json_finite(result)
            assembled["timings"]["algorithm_wall_s"] = perf_counter() - entered
            return assembled

        def observe(report):
            if report.occupancy == "free" and report.safety == "continuous_bound":
                return True
            key = ("map_unknown_rejections" if report.reason == "map_unknown" else
                   "occupied_rejections" if report.occupancy == "occupied" else "unproven_rejections")
            diagnostics[key] += 1
            return False

        try:
            validate_body(body)
            validate_pose(start)
            validate_goal(goal)
            if (not np.isfinite([config.resolution_m, config.margin_m]).all()
                    or config.resolution_m <= 0 or config.margin_m < 0
                    or isinstance(config.seed, bool) or not isinstance(config.seed, (int, np.integer))):
                raise ValueError("invalid resolution, margin or seed")
            budget = QueryBudget(config.budget, entered_at=entered)
            if self.prepared is not None:
                if self.prepared.scene is not scene:
                    raise ValueError("prepared scene identity mismatch; prepare the requested scene")
                prepared = self.prepared
            else:
                with stage("index_build"):
                    prepared = PreparedScene(scene, check=budget.check)
            oracle = GaussianBodyOracle(prepared, budget=budget)
            with stage("endpoint_check"):
                start_report = oracle.pose(start, body, margin_m=config.margin_m)
                result["endpoint_reports"]["start"] = asdict(start_report)
                original_report = oracle.pose(goal.original, body, margin_m=config.margin_m)
                result["endpoint_reports"]["original"] = asdict(original_report)
            if not observe(start_report):
                status = "map_unknown" if start_report.reason == "map_unknown" else "start_invalid"
                return finish(status, "start:" + start_report.reason, start_report.safety)
            if goal.position_tolerance_m == 0 and not observe(original_report):
                status = "map_unknown" if original_report.reason == "map_unknown" else "goal_invalid"
                return finish(status, "original_goal:" + original_report.reason, original_report.safety)

            def edge(a, b):
                with stage("edge_validation"):
                    return observe(oracle.edge(a, b, body, margin_m=config.margin_m))

            path = None
            if original_report.occupancy == "free" and edge(start, goal.original):
                path = [start, goal.original]
            if path is None:
                with stage("search"):
                    path = self._search(scene, body, start, goal, config, budget, diagnostics,
                                        edge, stage)
            if path is None:
                if diagnostics["map_unknown_rejections"]:
                    return finish("map_unknown", "reachable_frontier_meets_unknown_coverage")
                if diagnostics["unproven_rejections"]:
                    return finish("verification_failed", "reachable_frontier_has_unproven_edges")
                return finish("no_path_on_lattice", "reachable_lattice_exhausted")
            with stage("trajectory_build"):
                poses, trajectory = _linear_trajectory(path, body, start.yaw, goal.original.yaw)
            with stage("verify"):
                verification = verify_path(oracle, poses, body, margin_m=config.margin_m, goal=goal)
                kinematics = verify_linear_trajectory(trajectory, body, _limits(body))
                diagnostics["verification"] = verification
                diagnostics["kinematics"] = kinematics
                attained = oracle.pose(poses[-1], body, margin_m=config.margin_m)
                result["endpoint_reports"]["attained"] = asdict(attained)
            if not verification["passed"] or not kinematics["passed"] or attained.occupancy != "free":
                return finish("verification_failed", "post_build_verification_failed")
            result.update(trajectory=trajectory, attained_goal=_row(poses[-1]),
                          clearance_lower_m=verification["clearance_lower_m"])
            diagnostics["path_length_m"] = sum(float(np.linalg.norm(np.asarray(b.xyz) - a.xyz))
                                               for a, b in zip(poses, poses[1:]))
            diagnostics["altitude_range_m"] = float(np.ptp([q.xyz[2] for q in poses]))
            return finish("success", "verified_goal_reached", "continuous_bound")
        except BudgetExceeded as exc:
            return finish("budget_exhausted", str(exc))
        except (ValueError, TypeError, AttributeError, OverflowError) as exc:
            return finish("invalid_input", str(exc), "invalid")

    @staticmethod
    def _search(scene, body, start, goal, config, budget, diagnostics, edge, stage):
        uav = body.motion == "uav_translation"
        axes = 3 if uav else 2
        offsets = [q for q in product((-1, 0, 1), repeat=axes) if any(q)]
        origin = np.asarray(start.xyz)
        half = np.array([body.radius_m, body.radius_m, body.half_height_m])
        lo = np.asarray(scene.bounds_min) + half + config.margin_m
        hi = np.asarray(scene.bounds_max) - half - config.margin_m
        start_key = (0,) * axes
        costs, parents, poses = {start_key: 0.}, {}, {start_key: start}
        closed = set()
        serial = count()

        def heuristic(q):
            return max(0., float(np.linalg.norm(np.asarray(q.xyz) - goal.original.xyz)) - goal.position_tolerance_m)

        queue = [(heuristic(start), next(serial), 0., start_key)]
        while queue:
            budget.check()
            if diagnostics["expansions"] >= config.budget.max_expansions:
                raise BudgetExceeded("max_expansions")
            _, _, cost, key = heapq.heappop(queue)
            if key in closed or cost > costs[key]:
                continue
            closed.add(key)
            diagnostics["expansions"] += 1
            q = poses[key]
            dist = float(np.linalg.norm(np.asarray(q.xyz) - goal.original.xyz))
            terminal = None
            with stage("goal_candidates"):
                if dist <= goal.position_tolerance_m + GOAL_ROUNDOFF_M:
                    diagnostics["goal_candidates_checked"] += 1
                    terminal = q
                elif dist <= max(2 * config.resolution_m, goal.position_tolerance_m + config.resolution_m):
                    diagnostics["goal_candidates_checked"] += 1
                    if edge(q, goal.original):
                        terminal = goal.original
            if terminal is not None:
                path = [q]
                cursor = key
                while cursor in parents:
                    cursor = parents[cursor]
                    path.append(poses[cursor])
                path.reverse()
                if terminal is not q:
                    path.append(terminal)
                return path
            for step in offsets:
                budget.check()
                next_key = tuple(a + b for a, b in zip(key, step))
                if next_key in closed:
                    continue
                xyz = origin.copy()
                xyz[:axes] += config.resolution_m * np.asarray(next_key)
                if not uav:
                    if scene.support is None:
                        continue
                    height = scene.support.height(float(xyz[0]), float(xyz[1]))
                    if height is None or not np.isfinite(height):
                        diagnostics["unproven_rejections"] += 1
                        continue
                    xyz[2] = height + body.half_height_m + body.ground_clearance_m
                if np.any(xyz < lo) or np.any(xyz > hi):
                    continue
                proposal = Pose3(tuple(map(float, xyz)), 0.)
                tentative = cost + float(np.linalg.norm(xyz - q.xyz))
                if tentative >= costs.get(next_key, float("inf")) - 1e-12:
                    continue
                if not edge(q, proposal):
                    continue
                costs[next_key], parents[next_key], poses[next_key] = tentative, key, proposal
                heapq.heappush(queue, (tentative + heuristic(proposal), next(serial), tentative, next_key))
            diagnostics["frontier_peak"] = max(diagnostics["frontier_peak"], len(queue))
            diagnostics["visited_nodes"] = len(costs)
        return None


def _json_finite(value):
    """Invalid input diagnostics must still serialize with allow_nan=False."""
    if isinstance(value, dict):
        return {key: _json_finite(v) for key, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_finite(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value
