"""Bounded exact/relaxed goal policy and independently classified diagnostics."""
from __future__ import annotations

from dataclasses import asdict, replace
import math

import numpy as np

from .contracts import GoalRegion, PlannerConfig, Pose3
from .validation import angle_delta, validate_goal


CYLINDER_TOLERANCES_M = (0., .10, .25, .50)
REFINABLE_STATUSES = frozenset(("no_path_on_lattice", "verification_failed"))


def terminal_error(result: dict) -> dict | None:
    """Return recorded attained-to-original error without changing the target."""
    attained = result.get("attained_goal")
    if attained is None:
        return None
    original = np.asarray(result["original_goal"], float)
    attained = np.asarray(attained, float)
    return {"position_m": float(np.linalg.norm(attained[:3] - original[:3])),
            "yaw_rad": abs(float(angle_delta(attained[3], original[3])))}


def classify_attempt(result: dict) -> dict:
    """Keep endpoint, budget, connectivity and scale evidence separate."""
    diagnostics = result.get("diagnostics", {})
    endpoint = result.get("endpoint_reports", {})
    return {
        "status": result.get("status"),
        "termination": diagnostics.get("termination", result.get("reason")),
        "endpoint_map_status": {
            key: {field: report.get(field) for field in ("occupancy", "safety", "reason")}
            for key, report in endpoint.items()
        },
        "budget_exhaustion": (result.get("reason") if result.get("status") == "budget_exhausted"
                              else None),
        "connectivity": {
            "neighbourhood": diagnostics.get("connectivity"),
            "visited_nodes": diagnostics.get("visited_nodes", 0),
            "frontier_peak": diagnostics.get("frontier_peak", 0),
            "reachable_lattice_exhausted": result.get("reason") == "reachable_lattice_exhausted",
            "map_unknown_rejections": diagnostics.get("map_unknown_rejections", 0),
        },
        "scaling": {
            key: diagnostics.get(key, 0) for key in
            ("expansions", "oracle_calls", "candidate_count", "narrowphase_pairs")
        },
        "terminal_error": terminal_error(result),
    }


def plan_with_refinement(planner, scene, body, start: Pose3, goal: GoalRegion,
                         config: PlannerConfig, *, refinement_resolution_m: float | None = None,
                         timer=None) -> dict:
    """Run the declared resolution and at most one explicitly bounded refinement.

    Attempts are retained verbatim.  Endpoint/map/budget failures are not relabelled
    as connectivity failures, and budget exhaustion does not trigger an even more
    expensive automatic retry.
    """
    validate_goal(goal)
    resolutions = [float(config.resolution_m)]
    if refinement_resolution_m is not None:
        refined = float(refinement_resolution_m)
        if (not math.isfinite(refined) or refined <= 0 or refined >= config.resolution_m):
            raise ValueError("refinement resolution must be finite, positive and finer")
        resolutions.append(refined)
    attempts = []
    for index, resolution in enumerate(resolutions):
        result = planner.plan(scene, body, start, goal,
                              replace(config, resolution_m=resolution), timer=timer)
        result["diagnostics"]["goal_error"] = terminal_error(result)
        attempts.append(result)
        if result["status"] == "success" or result["status"] not in REFINABLE_STATUSES:
            break
        if index == 1:
            break
    selected = attempts[-1]
    return {"schema_version": "gs3d.goal-policy.v1",
            "original_goal": list((*goal.original.xyz, goal.original.yaw)),
            "position_tolerance_m": goal.position_tolerance_m,
            "attempts": attempts,
            "selected_attempt": len(attempts) - 1,
            "result": selected,
            "diagnosis": classify_attempt(selected)}


def cylinder_goal_sensitivity(planner, scene, start: Pose3, original: Pose3,
                              config: PlannerConfig, *, body, tolerances=CYLINDER_TOLERANCES_M,
                              refinement_resolution_m: float | None = None, timer=None) -> dict:
    """Compare exact and finite relaxed goals while retaining one original pose."""
    values = tuple(float(value) for value in tolerances)
    if not values or values[0] != 0 or any(not math.isfinite(v) or not 0 <= v <= .5 for v in values):
        raise ValueError("cylinder tolerances must start at exact and remain in [0,.5] m")
    cases = []
    for tolerance in values:
        policy = plan_with_refinement(
            planner, scene, body, start,
            GoalRegion(original, position_tolerance_m=tolerance, yaw_tolerance_rad=.05),
            config, refinement_resolution_m=refinement_resolution_m, timer=timer)
        cases.append({"tolerance_m": tolerance, **policy})
    return {"schema_version": "gs3d.goal-sensitivity.v1",
            "robot": asdict(body),
            "original_goal": list((*original.xyz, original.yaw)),
            "tolerances_m": list(values),
            "cases": cases}
