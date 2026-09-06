"""Reproducible P5 BVH and fixed-slice incremental scaling benchmark."""
from __future__ import annotations

import cProfile
from dataclasses import replace
from datetime import datetime, timezone
import gc
from pathlib import Path
import platform
import tracemalloc
import time

import numpy as np
import shapely
from shapely.geometry import Point

from ..config import load_config
from ..geometry.support import build_oracles
from ..io.robot_io import ellipse_robot
from ..spatial.bvh import (SceneSupportBVH, pair_bounding_disc)
from ..spatial.incremental import IncrementalSliceCompiler
from ..types import GaussianSupport2D, SceneModel2D


def _ordered_counts(raw, label: str) -> tuple[int, ...]:
    if isinstance(raw, (str, bytes)):
        raise ValueError(f"{label} must be a strictly increasing integer list")
    counts = tuple(int(value) for value in raw)
    if not counts or any(value < 1 for value in counts):
        raise ValueError(f"{label} must contain positive integers")
    if any(right <= left for left, right in zip(counts, counts[1:])):
        raise ValueError(f"{label} must be strictly increasing")
    return counts


def _workspace(bounds) -> object:
    if len(bounds) != 4:
        raise ValueError("workspace must be [xmin, ymin, xmax, ymax]")
    xmin, ymin, xmax, ymax = map(float, bounds)
    if not (xmin < xmax and ymin < ymax):
        raise ValueError("workspace bounds must have positive area")
    return shapely.box(xmin, ymin, xmax, ymax)


def _support(primitive_id: int, mean, axes, angle: float = 0.0):
    c, s = np.cos(angle), np.sin(angle)
    rotation = np.array([[c, -s], [s, c]], dtype=float)
    covariance = rotation @ np.diag(np.square(axes)) @ rotation.T
    return GaussianSupport2D(
        mean=np.asarray(mean, dtype=float), covariance=covariance,
        level=1.0, primitive_id=int(primitive_id),
    )


def _bvh_support_pool(count: int, cfg: dict) -> tuple:
    """Generate a nested deterministic mix of workspace-near and far supports."""
    rng = np.random.default_rng(int(cfg.get("seed", 20260901)))
    workspace_bounds = tuple(map(float, cfg["workspace"]))
    xmin, ymin, xmax, ymax = workspace_bounds
    near_fraction = float(cfg.get("near_fraction", 0.125))
    if not 0.0 < near_fraction <= 1.0:
        raise ValueError("near_fraction must lie in (0, 1]")
    near_stride = max(1, int(round(1.0 / near_fraction)))
    axes_min = np.asarray(cfg.get("scene_axes_min", [0.03, 0.03]), float)
    axes_max = np.asarray(cfg.get("scene_axes_max", [0.12, 0.09]), float)
    far_center = np.asarray(cfg.get("far_center", [30.0, -24.0]), float)
    far_jitter = np.asarray(cfg.get("far_jitter", [6.0, 5.0]), float)
    if axes_min.shape != (2,) or axes_max.shape != (2,):
        raise ValueError("scene axes bounds must be length-two vectors")
    if np.any(axes_min <= 0.0) or np.any(axes_max < axes_min):
        raise ValueError("scene axes bounds are invalid")

    supports = []
    near_low = np.array([xmin, ymin]) * 0.85
    near_high = np.array([xmax, ymax]) * 0.85
    for primitive_id in range(count):
        if primitive_id % near_stride == 0:
            mean = rng.uniform(near_low, near_high)
        else:
            # Alternate two separated clusters so hierarchy pruning remains a
            # genuine tree-level operation rather than one accidental outlier.
            sign = 1.0 if primitive_id % 2 else -1.0
            mean = sign * far_center + rng.uniform(-far_jitter, far_jitter)
        axes = rng.uniform(axes_min, axes_max)
        supports.append(_support(
            primitive_id, mean, axes, rng.uniform(-np.pi, np.pi)
        ))
    return tuple(supports)


def _flat_candidate_ids(scene, robot) -> tuple[tuple[int, int], ...]:
    """Independent legacy flat predicate used only for semantic validation."""
    expanded = scene.workspace.buffer(robot.max_rotational_radius())
    selected = []
    for oracle in build_oracles(scene, robot):
        center, radius = pair_bounding_disc(oracle)
        if expanded.distance(Point(center)) <= radius:
            selected.append((oracle.pair_id.scene_id, oracle.pair_id.body_id))
    return tuple(selected)


def _bvh_memory_replay(scene, robot, leaf_size: int) -> int:
    """Peak traced Python allocation for BVH build+query, excluding scene input."""
    gc.collect()
    already_tracing = tracemalloc.is_tracing()
    if not already_tracing:
        tracemalloc.start()
    base_current, _ = tracemalloc.get_traced_memory()
    tracemalloc.reset_peak()
    try:
        index = SceneSupportBVH.from_scene(scene, leaf_size=leaf_size)
        index.query(robot, scene.workspace)
        _, peak = tracemalloc.get_traced_memory()
        return int(max(0, peak - base_current))
    finally:
        if not already_tracing:
            tracemalloc.stop()


def _run_bvh_case(scene, robot, leaf_size: int) -> dict:
    t0 = time.perf_counter()
    index = SceneSupportBVH.from_scene(scene, leaf_size=leaf_size)
    build_wall = time.perf_counter() - t0
    t0 = time.perf_counter()
    result = index.query(robot, scene.workspace)
    query_wall = time.perf_counter() - t0
    stats = result.stats
    actual_ids = tuple((oracle.pair_id.scene_id, oracle.pair_id.body_id)
                       for oracle in result.oracles)
    flat_ids = _flat_candidate_ids(scene, robot)
    return {
        "scene_supports": int(stats.scene_supports),
        "body_supports": int(stats.body_supports),
        "total_pairs": int(stats.total_pairs),
        "candidate_pairs": int(stats.candidate_pairs),
        "pair_tests": int(stats.pair_tests),
        "pruned_pairs": int(stats.pruned_pairs),
        "nodes_visited": int(stats.nodes_visited),
        "nodes_pruned": int(stats.nodes_pruned),
        "tree_nodes": int(stats.tree_nodes),
        "tree_depth": int(stats.tree_depth),
        "build_wall_seconds": float(build_wall),
        "query_wall_seconds": float(query_wall),
        "peak_memory_bytes": _bvh_memory_replay(scene, robot, leaf_size),
        "validation": {
            "candidate_set_matches_flat": actual_ids == flat_ids,
            "flat_candidate_pairs": len(flat_ids),
            "pair_accounting_conserved": (
                stats.pair_tests + stats.pruned_pairs == stats.total_pairs
            ),
        },
    }


def _incremental_support_pool(count: int, cfg: dict) -> tuple:
    """Nested low-discrepancy support layout entirely inside the workspace."""
    radius = float(cfg.get("scene_radius", 0.055))
    spread = float(cfg.get("layout_radius", 1.8))
    if radius <= 0.0 or spread <= 0.0:
        raise ValueError("incremental scene radius and layout radius must be positive")
    golden_angle = np.pi * (3.0 - np.sqrt(5.0))
    supports = []
    for primitive_id in range(count):
        radial = spread * np.sqrt((primitive_id + 0.5) / count)
        angle = primitive_id * golden_angle
        mean = radial * np.array([np.cos(angle), np.sin(angle)])
        supports.append(_support(primitive_id, mean, (radius, radius)))
    return tuple(supports)


def _semantic_diff(incremental, full, tolerance: float) -> dict:
    plus_area = float(incremental.C_plus.symmetric_difference(full.C_plus).area)
    minus_area = float(incremental.C_minus.symmetric_difference(full.C_minus).area)
    component_counts_equal = (
        len(incremental.D_safe) == len(full.D_safe)
        and len(incremental.D_possible) == len(full.D_possible)
    )
    pair_ids_equal = (
        tuple(item.pair_id for item in incremental.sandwiches)
        == tuple(item.pair_id for item in full.sandwiches)
    )
    status_equal = incremental.status is full.status
    return {
        "C_plus_symmetric_difference_area": plus_area,
        "C_minus_symmetric_difference_area": minus_area,
        "component_counts_equal": component_counts_equal,
        "pair_ids_equal": pair_ids_equal,
        "status_equal": status_equal,
        "area_tolerance": float(tolerance),
        "semantic_equal": bool(
            plus_area <= tolerance and minus_area <= tolerance
            and component_counts_equal and pair_ids_equal and status_equal
        ),
    }


def _phase_payload(result, compiler, wall: float) -> dict:
    return {
        "envelope_support_calls": int(result.support_calls),
        "union_operations": int(compiler.stats.last_union_operations),
        "union_leaf_updates": int(compiler.stats.last_union_leaf_updates),
        "envelopes_computed": int(compiler.stats.last_computed),
        "envelopes_reused": int(compiler.stats.last_reused),
        "orientation_seeds_used": int(
            compiler.stats.last_orientation_seeded),
        "orientation_seeds_bypassed": int(
            compiler.stats.last_orientation_seeds_bypassed),
        "wall_seconds": float(wall),
    }


def _edited_scene(scene, edit_index: int, delta) -> SceneModel2D:
    supports = list(scene.supports)
    old = supports[edit_index]
    supports[edit_index] = GaussianSupport2D(
        mean=old.mean + np.asarray(delta, dtype=float),
        covariance=old.covariance,
        level=old.level,
        primitive_id=old.primitive_id,
    )
    return SceneModel2D(tuple(supports), scene.workspace, scene.name + "_edited")


def _run_incremental_case(scene, edited, robot, gmc_cfg, theta: float,
                          tolerance: float) -> dict:
    t0 = time.perf_counter()
    compiler = IncrementalSliceCompiler(scene, robot, gmc_cfg)
    cold = compiler.compile_slice(theta)
    cold_wall = time.perf_counter() - t0
    cold_payload = _phase_payload(cold, compiler, cold_wall)

    t0 = time.perf_counter()
    changed = compiler.update_models(scene=edited)
    incremental = compiler.compile_slice(theta)
    incremental_wall = time.perf_counter() - t0
    incremental_payload = _phase_payload(
        incremental, compiler, incremental_wall
    )

    t0 = time.perf_counter()
    full_compiler = IncrementalSliceCompiler(edited, robot, gmc_cfg)
    full = full_compiler.compile_slice(theta)
    full_wall = time.perf_counter() - t0
    full_payload = _phase_payload(full, full_compiler, full_wall)

    return {
        "scene_supports": len(scene.supports),
        "theta": float(theta),
        "edited_support_id": int(edited.supports[len(scene.supports) // 2].primitive_id),
        "changed_pair_count": len(changed),
        "cold_initial": cold_payload,
        "full_edit": full_payload,
        "incremental_edit": incremental_payload,
        "work_relation": {
            "incremental_support_calls_le_full": (
                incremental.support_calls <= full.support_calls
            ),
            "incremental_union_operations_le_full": (
                compiler.stats.last_union_operations
                <= full_compiler.stats.last_union_operations
            ),
        },
        "geometry_semantic_diff": _semantic_diff(
            incremental, full, tolerance
        ),
    }


def _profile_rows(call, top_n: int) -> list[dict]:
    """Return structured top cumulative cProfile rows without text parsing."""
    profiler = cProfile.Profile()
    profiler.enable()
    try:
        call()
    finally:
        profiler.disable()
    rows = []
    for entry in profiler.getstats():
        code = entry.code
        if isinstance(code, str):
            filename, line, function = "~", 0, code
        else:
            filename = Path(code.co_filename).name
            line = int(code.co_firstlineno)
            function = code.co_name
        rows.append({
            "function": function,
            "file": filename,
            "line": line,
            "primitive_calls": int(entry.callcount - entry.reccallcount),
            "total_calls": int(entry.callcount),
            "self_seconds": float(entry.inlinetime),
            "cumulative_seconds": float(entry.totaltime),
        })
    rows.sort(key=lambda row: (
        -row["cumulative_seconds"], row["file"], row["line"], row["function"]
    ))
    return rows[:top_n]


def run_p5_system_spec(spec: dict, *, spec_dir: str | Path = ".") -> dict:
    if spec.get("kind") != "p5_system_scaling":
        raise ValueError("system benchmark kind must be p5_system_scaling")
    base = Path(spec_dir).resolve()
    profile_top_n = int(spec.get("profile_top_n", 15))
    if profile_top_n < 1:
        raise ValueError("profile_top_n must be positive")

    bvh_cfg = dict(spec.get("bvh", {}))
    bvh_counts = _ordered_counts(bvh_cfg["support_counts"], "BVH counts")
    leaf_size = int(bvh_cfg.get("leaf_size", 8))
    bvh_workspace = _workspace(bvh_cfg["workspace"])
    bvh_robot_axes = tuple(map(float, bvh_cfg.get("robot_axes", [0.22, 0.12])))
    bvh_robot = ellipse_robot(*bvh_robot_axes, name="p5_bvh_robot")
    bvh_pool = _bvh_support_pool(bvh_counts[-1], bvh_cfg)
    bvh_curve = []
    for count in bvh_counts:
        scene = SceneModel2D(
            bvh_pool[:count], bvh_workspace, f"p5_bvh_{count}"
        )
        bvh_curve.append(_run_bvh_case(scene, bvh_robot, leaf_size))

    incremental_cfg = dict(spec.get("incremental", {}))
    incremental_counts = _ordered_counts(
        incremental_cfg["support_counts"], "incremental counts"
    )
    theta = float(incremental_cfg.get("theta", 0.31))
    edit_delta = tuple(map(float, incremental_cfg.get(
        "edit_delta", [0.025, -0.015]
    )))
    if len(edit_delta) != 2:
        raise ValueError("edit_delta must be a length-two vector")
    tolerance = float(incremental_cfg.get("semantic_area_tolerance", 1e-12))
    incremental_workspace = _workspace(incremental_cfg["workspace"])
    incremental_robot_axes = tuple(map(float, incremental_cfg.get(
        "robot_axes", [0.18, 0.09]
    )))
    incremental_robot = ellipse_robot(
        *incremental_robot_axes, name="p5_incremental_robot"
    )
    gmc_cfg_path = _resolve(base, incremental_cfg["gmc_config"])
    gmc_cfg = load_config(gmc_cfg_path)
    pair_overrides = dict(incremental_cfg.get("pair_approx", {}))
    if pair_overrides:
        gmc_cfg = replace(
            gmc_cfg,
            pair_approx=replace(
                gmc_cfg.pair_approx,
                initial_directions=int(pair_overrides.get(
                    "initial_directions", gmc_cfg.pair_approx.initial_directions
                )),
                max_directions=int(pair_overrides.get(
                    "max_directions", gmc_cfg.pair_approx.max_directions
                )),
                eps_pair=float(pair_overrides.get(
                    "eps_pair", gmc_cfg.pair_approx.eps_pair
                )),
                certificate_mode=str(pair_overrides.get(
                    "certificate_mode", gmc_cfg.pair_approx.certificate_mode
                )),
            ),
        )
    incremental_pool = _incremental_support_pool(
        incremental_counts[-1], incremental_cfg
    )
    incremental_curve = []
    incremental_cases = []
    for count in incremental_counts:
        scene = SceneModel2D(
            incremental_pool[:count], incremental_workspace,
            f"p5_incremental_{count}",
        )
        edit_index = count // 2
        edited = _edited_scene(scene, edit_index, edit_delta)
        incremental_cases.append((scene, edited))
        incremental_curve.append(_run_incremental_case(
            scene, edited, incremental_robot, gmc_cfg, theta, tolerance
        ))

    max_bvh_scene = SceneModel2D(
        bvh_pool, bvh_workspace, f"p5_bvh_{bvh_counts[-1]}_profile"
    )
    max_incremental_scene, max_incremental_edited = incremental_cases[-1]
    profiles = {
        "bvh_max_case": {
            "scene_supports": bvh_counts[-1],
            "scope": "SceneSupportBVH build plus one query",
            "sort": "cumulative_seconds_descending",
            "top_functions": _profile_rows(
                lambda: SceneSupportBVH.from_scene(
                    max_bvh_scene, leaf_size=leaf_size
                ).query(bvh_robot, max_bvh_scene.workspace),
                profile_top_n,
            ),
        },
        "incremental_max_case": {
            "scene_supports": incremental_counts[-1],
            "scope": "cold initial, one-support incremental edit, full-edit rebuild",
            "sort": "cumulative_seconds_descending",
            "top_functions": _profile_rows(
                lambda: _run_incremental_case(
                    max_incremental_scene, max_incremental_edited,
                    incremental_robot, gmc_cfg, theta, tolerance,
                ),
                profile_top_n,
            ),
        },
    }

    bvh_max = bvh_curve[-1]
    incremental_max = incremental_curve[-1]
    aggregate = {
        "bvh": {
            "n_cases": len(bvh_curve),
            "max_scene_supports": bvh_max["scene_supports"],
            "max_total_pairs": bvh_max["total_pairs"],
            "max_candidate_pairs": bvh_max["candidate_pairs"],
            "max_pair_tests": bvh_max["pair_tests"],
            "max_pruned_pairs": bvh_max["pruned_pairs"],
            "all_candidate_sets_match_flat": all(
                row["validation"]["candidate_set_matches_flat"]
                for row in bvh_curve
            ),
        },
        "incremental": {
            "n_cases": len(incremental_curve),
            "max_scene_supports": incremental_max["scene_supports"],
            "max_full_edit_envelope_support_calls": (
                incremental_max["full_edit"]["envelope_support_calls"]
            ),
            "max_incremental_edit_envelope_support_calls": (
                incremental_max["incremental_edit"]["envelope_support_calls"]
            ),
            "max_full_edit_union_operations": (
                incremental_max["full_edit"]["union_operations"]
            ),
            "max_incremental_edit_union_operations": (
                incremental_max["incremental_edit"]["union_operations"]
            ),
            "all_geometry_semantically_equal": all(
                row["geometry_semantic_diff"]["semantic_equal"]
                for row in incremental_curve
            ),
        },
    }

    return {
        "schema_version": 1,
        "name": str(spec.get("name", "p5-system-scaling")),
        "kind": "p5_system_scaling",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "reproducibility": {
            "seed": int(bvh_cfg.get("seed", 20260901)),
            "python": platform.python_version(),
            "numpy": np.__version__,
            "shapely": shapely.__version__,
            "atlas_accessed": False,
            "input_order": "each curve is a prefix of one max-size pool",
        },
        "bvh_contract": {
            "support_counts": list(bvh_counts),
            "leaf_size": leaf_size,
            "peak_memory_measurement": (
                "Python tracemalloc peak for a separate BVH build+query replay; "
                "prebuilt scene input excluded"
            ),
            "flat_validation_charged_to_build_or_query_wall": False,
        },
        "bvh_curve": bvh_curve,
        "incremental_contract": {
            "support_counts": list(incremental_counts),
            "theta": theta,
            "edit_delta": list(edit_delta),
            "edit": "replace exactly support floor(N/2), fixed theta",
            "gmc_config": str(gmc_cfg_path),
            "pair_approx": {
                "initial_directions": gmc_cfg.pair_approx.initial_directions,
                "max_directions": gmc_cfg.pair_approx.max_directions,
                "eps_pair": gmc_cfg.pair_approx.eps_pair,
                "certificate_mode": gmc_cfg.pair_approx.certificate_mode,
            },
        },
        "incremental_curve": incremental_curve,
        "profiles": profiles,
        # Kept so the existing CLI can print a compact result for every
        # benchmark kind without benchmark-specific presentation logic.
        "aggregate": aggregate,
        "limitations": [
            "wall and cProfile time are observational and not pass/fail gates",
            "tracemalloc reports Python-tracked allocations, not full process RSS",
            "the incremental curve is fixed-theta and edits one scene support",
            "synthetic curves exercise P5 systems behavior, not Atlas accuracy",
        ],
    }


def _resolve(base: Path, value: str | Path) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (base / path).resolve()
