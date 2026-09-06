"""Replayable artifact manifests and run directories (Guide §3.3, §13.2).

The run directory is a self-contained evidence bundle.  In particular, no
query or verification step needs the configuration file that happened to be
used when the bundle was created.
"""
from __future__ import annotations

from dataclasses import asdict, is_dataclass, replace
from enum import Enum
import gzip
import hashlib
import json
import math
import platform
import subprocess
import sys
from pathlib import Path

import networkx as nx
import numpy as np
import shapely
from shapely.geometry import mapping
import yaml

from .proof_records import (pair_pruning_payload,
                            validate_pair_pruning_payload)
from .replay import (INTERVAL_INDEX_RELATIVE, INTERVAL_TREE_SCHEMA_VERSION,
                     file_sha256, stable_component_id, stable_interval_id,
                     stable_pair_certificate_id,
                     validate_interval_artifact_tree)


_RUN_SUBDIRS = (
    "input",
    "pairs",
    "slices",
    "slabs",
    "orientation/intervals",
    "mobility/witnesses",
    "queries",
    "figures",
)


def _versions():
    import networkx
    import scipy
    import shapely
    try:
        proc = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True,
                              text=True, cwd=Path(__file__).parent)
        git = proc.stdout.strip() if proc.returncode == 0 else "n/a"
    except Exception:
        git = "n/a"
    return {"python": sys.version.split()[0], "numpy": np.__version__,
            "scipy": scipy.__version__, "shapely": shapely.__version__,
            "geos": shapely.geos_version_string,
            "networkx": networkx.__version__,
            "platform": platform.platform(), "git": git}


def _jsonable(value):
    """Convert nested evidence to strict, structure-preserving JSON values."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, (float, np.floating)):
        number = float(value)
        if math.isfinite(number):
            return number
        if math.isnan(number):
            return "NaN"
        return "Infinity" if number > 0 else "-Infinity"
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.ndarray):
        return [_jsonable(v) for v in value.tolist()]
    if isinstance(value, Enum):
        return value.name
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list, set)):
        return [_jsonable(v) for v in value]
    if hasattr(value, "scene_id") and hasattr(value, "body_id"):
        return {"scene_id": int(value.scene_id),
                "body_id": int(value.body_id)}
    # Unknown values remain explicit without destroying the surrounding
    # report/ambiguity structure.
    return {"python_type": type(value).__name__, "repr": repr(value)}


def _write_json(path: Path, payload) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_jsonable(payload), indent=2,
                               sort_keys=False, allow_nan=False) + "\n")
    return path


def _write_gzip_json(path: Path, payload) -> Path:
    """Write deterministic compressed JSON for large proof tables."""
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (json.dumps(
        _jsonable(payload), indent=2, sort_keys=False, allow_nan=False,
    ) + "\n").encode("utf-8")
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("wb") as raw:
        with gzip.GzipFile(
                filename="", mode="wb", fileobj=raw, mtime=0) as stream:
            stream.write(encoded)
    tmp.replace(path)
    return path


def _read_gzip_json(path: Path):
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        return json.load(stream)


def scene_hash(scene) -> str:
    h = hashlib.sha256()
    h.update(b"GMC_SCENE_ORDERED_V2\0")
    h.update(np.asarray(len(scene.supports), dtype="<i8").tobytes())
    # Candidate-pair enumeration is scene-major in tuple order.  Bind that
    # order into the fingerprint: sorting by primitive_id here would let two
    # YAML inputs share a hash while producing differently ordered artifacts.
    for position, s in enumerate(scene.supports):
        h.update(np.asarray(position, dtype="<i8").tobytes())
        h.update(np.asarray(s.primitive_id, dtype="<i8").tobytes())
        h.update(np.asarray(s.mean, dtype="<f8").tobytes())
        h.update(np.asarray(s.covariance, dtype="<f8").tobytes())
        h.update(np.asarray(s.level, dtype="<f8").tobytes())
    # WKB records complete polygon topology, including every interior ring.
    h.update(shapely.normalize(scene.workspace).wkb)
    return h.hexdigest()[:16]


def robot_hash(robot) -> str:
    h = hashlib.sha256()
    h.update(b"GMC_ROBOT_ORDERED_V2\0")
    h.update(np.asarray(len(robot.supports), dtype="<i8").tobytes())
    # Pair enumeration is body-minor in this tuple order; preserve it for I7.
    for position, s in enumerate(robot.supports):
        h.update(np.asarray(position, dtype="<i8").tobytes())
        h.update(np.asarray(s.primitive_id, dtype="<i8").tobytes())
        h.update(np.asarray(s.mean, dtype="<f8").tobytes())
        h.update(np.asarray(s.covariance, dtype="<f8").tobytes())
        h.update(np.asarray(s.level, dtype="<f8").tobytes())
    return h.hexdigest()[:16]


def numerical_validation_context(scene, robot, cfg) -> dict:
    """Replay the binary64 resolution gate in ``validate_models``.

    This is evidence, not a replacement for validation: compile/load call the
    actual M0 validator before recording or accepting the bundle.
    """
    workspace_coords = [np.asarray(scene.workspace.exterior.coords, float)]
    workspace_coords.extend(
        np.asarray(ring.coords, float) for ring in scene.workspace.interiors)
    workspace_abs = max(
        (float(np.max(np.abs(coords))) for coords in workspace_coords
         if coords.size),
        default=0.0,
    )
    robot_extent = float(robot.max_rotational_radius())
    scene_expanded_abs = max(
        (float(np.max(np.abs(s.mean))) + s.bounding_radius() + robot_extent
         for s in scene.supports),
        default=0.0,
    )
    max_world_coordinate = max(workspace_abs, scene_expanded_abs)
    coordinate_ulp = float(abs(np.spacing(max_world_coordinate)))
    tolerances = {
        "workspace_precision": float(cfg.geometry.workspace_precision),
        "eps_pair": float(cfg.pair_approx.eps_pair),
        "eps_clear": float(cfg.query.eps_clear),
    }
    comparisons = {
        name: {
            "threshold": value,
            "checked": value > 0.0,
            "ulp_not_coarser": (coordinate_ulp <= value
                                if value > 0.0 else None),
            "threshold_minus_ulp": (value - coordinate_ulp
                                    if value > 0.0 else None),
        }
        for name, value in tolerances.items()
    }
    positive = [value for value in tolerances.values() if value > 0.0]
    accepted = (np.isfinite(max_world_coordinate)
                and np.isfinite(coordinate_ulp)
                and all(row["ulp_not_coarser"] is not False
                        for row in comparisons.values()))
    return {
        "policy_source": "gmc.io.gs_io.validate_models",
        "coordinate_backend": "IEEE-754 binary64 world coordinates + GEOS",
        "world_coordinate_max_magnitude": max_world_coordinate,
        "world_coordinate_ulp": coordinate_ulp,
        "workspace_max_magnitude": workspace_abs,
        "scene_expanded_max_magnitude": scene_expanded_abs,
        "robot_max_rotational_radius": robot_extent,
        "ulp_definition": "abs(numpy.spacing(world_coordinate_max_magnitude))",
        "acceptance_rule": (
            "reject when world_coordinate_ulp exceeds any positive "
            "workspace_precision, eps_pair, or eps_clear threshold"
        ),
        "minimum_positive_threshold": min(positive) if positive else None,
        "thresholds": comparisons,
        "accepted": bool(accepted),
    }


def make_run_dir(root: str | Path, run_id: str) -> Path:
    """Create a fresh run layout, refusing to mix with prior evidence."""
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    run = root / run_id
    if run.exists():
        if not run.is_dir():
            raise FileExistsError(f"run path already exists and is not a directory: {run}")
        if any(run.iterdir()):
            raise FileExistsError(
                f"run directory already contains evidence; choose a new run id: {run}"
            )
    else:
        run.mkdir()
    for sub in _RUN_SUBDIRS:
        (run / sub).mkdir(parents=True, exist_ok=True)
    return run


def config_payload(cfg) -> dict:
    """Canonical YAML representation of every run-affecting config field."""
    return {
        "units": {"length": str(cfg.length_unit)},
        "geometry": {
            "workspace_precision": float(cfg.geometry.workspace_precision),
            "min_cov_eigenvalue": float(cfg.geometry.min_cov_eigenvalue),
            "support_level_scene": float(cfg.geometry.support_level_scene),
            "support_level_robot": float(cfg.geometry.support_level_robot),
        },
        "pair_approx": {
            "initial_directions": int(cfg.pair_approx.initial_directions),
            "max_directions": int(cfg.pair_approx.max_directions),
            "eps_pair": float(cfg.pair_approx.eps_pair),
            "certificate_mode": str(cfg.pair_approx.certificate_mode),
        },
        "orientation": {
            "initial_intervals": int(cfg.orientation.initial_intervals),
            "theta_min": float(cfg.orientation.theta_min),
            "max_depth": int(cfg.orientation.max_depth),
        },
        "query": {
            "max_support_calls": int(cfg.query.max_support_calls),
            "max_wall_seconds": float(cfg.query.max_wall_seconds),
            "eps_clear": float(cfg.query.eps_clear),
            "max_refinement_rounds": int(cfg.query.max_refinement_rounds),
        },
        "logging": {
            "save_intermediate_geometry": bool(
                cfg.logging.save_intermediate_geometry),
            "save_failed_cases": bool(cfg.logging.save_failed_cases),
        },
    }


def save_frozen_config(run_dir: Path, cfg) -> Path:
    """Write the canonical, run-local configuration used for all replay."""
    path = Path(run_dir) / "input" / "config.yaml"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(config_payload(cfg), sort_keys=False))
    return path


def _resolve_run_relative(run_dir: Path, relative: str) -> Path:
    if not isinstance(relative, str) or not relative:
        raise ValueError("manifest input path must be a non-empty relative path")
    candidate = (run_dir / relative).resolve()
    base = run_dir.resolve()
    try:
        candidate.relative_to(base)
    except ValueError as exc:
        raise ValueError(f"manifest input escapes run directory: {relative}") from exc
    return candidate


def _first_difference(expected, actual, path="config"):
    """Return the first field-level mismatch between JSON-like values."""
    if isinstance(expected, dict) and isinstance(actual, dict):
        expected_keys, actual_keys = set(expected), set(actual)
        if expected_keys != actual_keys:
            missing = sorted(expected_keys - actual_keys)
            extra = sorted(actual_keys - expected_keys)
            return f"{path} keys differ (missing={missing}, extra={extra})"
        for key in sorted(expected):
            difference = _first_difference(
                expected[key], actual[key], f"{path}.{key}")
            if difference is not None:
                return difference
        return None
    if type(expected) is not type(actual) or expected != actual:
        return f"{path}: manifest={expected!r}, frozen={actual!r}"
    return None


def load_run_inputs(run_dir: str | Path):
    """Load frozen inputs and reject tampered or accidentally mixed bundles."""
    from ..config import load_config
    from ..io.gs_io import load_scene, validate_models
    from ..io.robot_io import load_robot

    run_dir = Path(run_dir)
    manifest_path = run_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    inputs = manifest.get("inputs", {})
    scene_path = _resolve_run_relative(
        run_dir, inputs.get("scene", "input/scene.yaml"))
    robot_path = _resolve_run_relative(
        run_dir, inputs.get("robot", "input/robot.yaml"))
    config_source = manifest.get("config", {}).get(
        "source", inputs.get("config", "input/config.yaml"))
    config_path = _resolve_run_relative(run_dir, config_source)
    scene = load_scene(scene_path)
    robot = load_robot(robot_path)
    cfg = load_config(config_path)
    validate_models(scene, robot, cfg)

    declared_config = manifest.get("config", {}).get("values")
    if not isinstance(declared_config, dict):
        raise ValueError(
            "manifest is missing the complete frozen config declaration")
    actual_config = config_payload(cfg)
    config_difference = _first_difference(declared_config, actual_config)
    if config_difference is not None:
        raise ValueError(
            "run input config does not match manifest declaration: "
            + config_difference
        )

    actual_scene_hash = scene_hash(scene)
    actual_robot_hash = robot_hash(robot)
    if actual_scene_hash != manifest.get("scene_hash"):
        raise ValueError(
            "run input scene hash mismatch: "
            f"expected {manifest.get('scene_hash')}, got {actual_scene_hash}"
        )
    if actual_robot_hash != manifest.get("robot_hash"):
        raise ValueError(
            "run input robot hash mismatch: "
            f"expected {manifest.get('robot_hash')}, got {actual_robot_hash}"
        )

    declared_numerics = manifest.get("numerical_validation")
    if not isinstance(declared_numerics, dict):
        raise ValueError(
            "manifest is missing the numerical validation context")
    actual_numerics = numerical_validation_context(scene, robot, cfg)
    numerical_difference = _first_difference(
        declared_numerics, actual_numerics, path="numerical_validation")
    if numerical_difference is not None:
        raise ValueError(
            "run numerical context does not match frozen inputs: "
            + numerical_difference
        )
    return manifest, cfg, scene, robot


_MANIFEST_RESERVED_KEYS = frozenset({
    "schema_version", "scene_hash", "robot_hash", "scene_name",
    "robot_name", "n_scene_supports", "n_robot_supports", "length_unit",
    "workspace", "scene_support_levels", "robot_support_levels",
    "scene_support_order", "robot_support_order", "inputs", "config",
    "numerical_validation", "i7_complete", "i7_incomplete_reason",
    "interval_cover_artifacts", "pair_pruning_artifact",
    "fixed_slice_direction_sets_artifact", "determinism",
    "software",
})


def _interval_manifest_payload(run_dir: Path) -> dict:
    """Bind the independently validated interval tree into the manifest."""
    index_path = Path(run_dir) / INTERVAL_INDEX_RELATIVE
    if not index_path.is_file():
        return {
            "schema_version": INTERVAL_TREE_SCHEMA_VERSION,
            "profile": "interval_pair_cover_v1",
            "index": INTERVAL_INDEX_RELATIVE,
            "index_sha256": None,
            "status": "not_recorded",
            "slab_count": 0,
            "certified_interval_cover_count": 0,
            "global_cover_status": "UNKNOWN",
            "global_cover_provenance": {
                "reason": "interval_cover_index_missing",
                "evidence_scope": "none",
            },
            "interval_event_free_certified": False,
            "artifact_tree_validated": False,
        }
    index = json.loads(index_path.read_text())
    report = validate_interval_artifact_tree(run_dir)
    if not report["valid"]:
        raise RuntimeError(
            "refusing to publish an invalid interval artifact tree: "
            + ", ".join(report["errors"])
        )
    return {
        "schema_version": index.get("schema_version"),
        "profile": index.get("profile"),
        "index": INTERVAL_INDEX_RELATIVE,
        "index_sha256": file_sha256(index_path),
        "status": "serialized_tranche_validated",
        "slab_count": index.get("slab_count"),
        "certified_interval_cover_count": report[
            "certified_interval_cover_count"],
        "global_cover_status": index.get("global_cover_status"),
        "global_cover_provenance": index.get("global_cover_provenance"),
        # The current event predicates are finite samples.  This field must
        # remain false even when the independent interval cover is certified.
        "interval_event_free_certified": False,
        "artifact_tree_validated": True,
    }


def _augment_artifact_scope(extra: dict, interval_payload: dict) -> dict:
    """Advertise the tranche without changing the honest full-I7 verdict."""
    result = dict(extra)
    scope = result.get("artifact_scope")
    if not isinstance(scope, dict):
        return result
    scope = dict(scope)
    produced = list(scope.get("produced", ()))
    advertised = []
    if interval_payload["artifact_tree_validated"]:
        advertised.append(INTERVAL_INDEX_RELATIVE)
    if (interval_payload["artifact_tree_validated"]
            and interval_payload["certified_interval_cover_count"]):
        advertised.extend((
            "orientation/intervals/*/certificate.json",
            "orientation/intervals/*/geometry.geojson",
        ))
    for relative in advertised:
        if relative not in produced:
            produced.append(relative)
    scope["produced"] = produced
    scope["interval_cover_tranche"] = {
        "status": interval_payload["status"],
        "artifact_tree_validated": interval_payload[
            "artifact_tree_validated"],
        "index": interval_payload["index"],
        "interval_event_free_certified": False,
        "full_i7_implication": False,
    }
    result["artifact_scope"] = scope
    return result


def _pair_pruning_manifest_payload(run_dir: Path, scene, robot) -> dict:
    """Bind a replayed all-pairs BVH decision table into the manifest."""
    relative = "pairs/pruning.json"
    path = Path(run_dir) / relative
    if not path.is_file():
        return {
            "schema_version": 1,
            "profile": "exact_bvh_pair_pruning_v1",
            "path": relative,
            "sha256": None,
            "status": "not_recorded",
            "independent_replay_validated": False,
        }
    payload = json.loads(path.read_text())
    report = validate_pair_pruning_payload(
        payload, scene, robot, scene.workspace,
    )
    if not report["valid"]:
        raise RuntimeError(
            "refusing to publish invalid pair-pruning evidence: "
            + ", ".join(report["errors"])
        )
    return {
        "schema_version": payload.get("schema_version"),
        "profile": payload.get("profile"),
        "path": relative,
        "sha256": file_sha256(path),
        "status": "complete",
        "decision_count": report["decision_count"],
        "retained_count": report["retained_count"],
        "independent_replay_validated": True,
    }


def _fixed_slice_directions_manifest_payload(run_dir: Path) -> dict:
    """Bind the proof-critical fixed-slice schedules into the manifest."""
    relative = "slices/direction_sets.json"
    path = Path(run_dir) / relative
    if not path.is_file():
        return {
            "schema_version": 1,
            "profile": "fixed_slice_direction_index_v1",
            "path": relative,
            "sha256": None,
            "status": "not_recorded",
            "structurally_validated": False,
        }
    payload = json.loads(path.read_text())
    report = validate_fixed_slice_direction_artifact_tree(
        run_dir, index=payload,
    )
    if not report["valid"]:
        raise RuntimeError(
            "refusing to publish invalid fixed-slice directions: "
            + ", ".join(report["errors"])
        )
    return {
        "schema_version": payload.get("schema_version"),
        "profile": payload.get("profile"),
        "path": relative,
        "sha256": file_sha256(path),
        "status": "complete",
        "unique_slice_count": report["unique_slice_count"],
        "cache_key_count": report["cache_key_count"],
        "pair_schedule_count": report["pair_schedule_count"],
        "unique_schedule_count": report["unique_schedule_count"],
        "structurally_validated": True,
    }


def write_manifest(run_dir: Path, scene, robot, cfg, extra: dict) -> Path:
    if not isinstance(extra, dict):
        raise TypeError("manifest extra must be a dictionary")
    conflicts = sorted(_MANIFEST_RESERVED_KEYS.intersection(extra))
    if conflicts:
        raise ValueError(
            "manifest extra cannot override reserved keys: "
            + ", ".join(conflicts)
        )
    interval_payload = _interval_manifest_payload(Path(run_dir))
    pruning_payload = _pair_pruning_manifest_payload(
        Path(run_dir), scene, robot,
    )
    direction_payload = _fixed_slice_directions_manifest_payload(
        Path(run_dir),
    )
    extra = _augment_artifact_scope(extra, interval_payload)
    workspace = {
        "exterior": [list(map(float, xy))
                     for xy in scene.workspace.exterior.coords],
        "holes": [[list(map(float, xy)) for xy in ring.coords]
                  for ring in scene.workspace.interiors],
    }
    manifest = {
        "schema_version": 4,
        "scene_hash": scene_hash(scene), "robot_hash": robot_hash(robot),
        "scene_name": scene.name, "robot_name": robot.name,
        "n_scene_supports": len(scene.supports),
        "n_robot_supports": len(robot.supports),
        "length_unit": cfg.length_unit,
        "workspace": workspace,
        "scene_support_levels": [
            {"primitive_id": s.primitive_id, "level": float(s.level)}
            for s in scene.supports
        ],
        "robot_support_levels": [
            {"primitive_id": s.primitive_id, "level": float(s.level)}
            for s in robot.supports
        ],
        "scene_support_order": [s.primitive_id for s in scene.supports],
        "robot_support_order": [s.primitive_id for s in robot.supports],
        "inputs": {
            "scene": "input/scene.yaml",
            "robot": "input/robot.yaml",
            "config": "input/config.yaml",
        },
        "config": {
            "source": "input/config.yaml",
            "frozen": True,
            "values": config_payload(cfg),
            "support_level_scene": cfg.geometry.support_level_scene,
            "support_level_robot": cfg.geometry.support_level_robot,
            "min_cov_eigenvalue": cfg.geometry.min_cov_eigenvalue,
            "eps_pair": cfg.pair_approx.eps_pair,
            "certificate_mode": cfg.pair_approx.certificate_mode,
            "initial_directions": cfg.pair_approx.initial_directions,
            "max_directions": cfg.pair_approx.max_directions,
            "initial_intervals": cfg.orientation.initial_intervals,
            "theta_min": cfg.orientation.theta_min,
            "max_depth": cfg.orientation.max_depth,
            "max_support_calls": cfg.query.max_support_calls,
            "max_wall_seconds": cfg.query.max_wall_seconds,
            "eps_clear": cfg.query.eps_clear,
            "max_refinement_rounds": cfg.query.max_refinement_rounds,
            "workspace_precision": cfg.geometry.workspace_precision,
        },
        "numerical_validation": numerical_validation_context(
            scene, robot, cfg),
        # Guide invariant I7 requires a single bundle containing all
        # tolerances, exact direction sets, pair-pruning bounds, event
        # brackets, and deterministic ordering.  v0 still omits some of those
        # formal objects; field presence must not be mistaken for completion.
        "i7_complete": False,
        "i7_incomplete_reason": (
            "the interval-pair cover tranche is replayable, but formal event "
            "brackets are not generic compile artifacts and per-query proofs "
            "have not yet been assembled into a full-run replay contract"
        ),
        "interval_cover_artifacts": interval_payload,
        "pair_pruning_artifact": pruning_payload,
        "fixed_slice_direction_sets_artifact": direction_payload,
        "determinism": {
            "random_seed": None,
            "randomness_used": False,
            "policy": (
                "Compilation uses canonical support-direction schedules, "
                "exact binary64 theta cache keys, and contains no stochastic "
                "operation. Scene/body tuple order is preserved by pair "
                "enumeration, frozen YAML, manifests, and ordered input hashes."
            ),
            "input_order_policy": (
                "support tuple order is semantic and hash-bound; YAML support "
                "reordering therefore changes scene_hash or robot_hash"
            ),
        },
        "software": _versions(),
        **extra,
    }
    # The manifest is intentionally the final compile artifact.  Atomic
    # replacement prevents a partial JSON document from looking complete.
    path = Path(run_dir) / "manifest.json"
    tmp = path.with_suffix(".json.tmp")
    _write_json(tmp, manifest)
    tmp.replace(path)
    return path


def _witness_key(raw) -> str:
    return json.dumps(_jsonable(raw), sort_keys=True, separators=(",", ":"),
                      allow_nan=False)


def _edge_witness_payload(mc, a, b, data, certificate_id: str) -> dict:
    from ..verification.continuous import rotation_interval_safe

    raw = data.get("witness")
    if raw is None:
        raise ValueError(f"SAFE mobility edge {a!r}-{b!r} has no witness")
    anchors, theta_start, theta_end = raw
    theta_start, theta_end = float(theta_start), float(theta_end)
    theta_min = float(mc.cfg.orientation.theta_min)
    eps_clear = float(mc.cfg.query.eps_clear)
    checks = []
    for anchor_index, anchor in enumerate(anchors):
        anchor = np.asarray(anchor, dtype=float)
        collision_free, min_margin = rotation_interval_safe(
            mc.oracles, anchor, theta_start, theta_end, theta_min, floor=0.0)
        eps_certified, eps_min_margin = rotation_interval_safe(
            mc.oracles, anchor, theta_start, theta_end, theta_min,
            floor=eps_clear)
        checks.append({
            "anchor_index": anchor_index,
            "anchor": anchor,
            "collision_free": bool(collision_free),
            "min_margin": float(min_margin),
            "eps_clear_certified": bool(eps_certified),
            "eps_min_margin": float(eps_min_margin),
        })
    if not checks or not all(row["collision_free"] for row in checks):
        raise RuntimeError(
            f"independent rotation witness recheck failed for SAFE edge {a}-{b}")
    return {
        "schema_version": 2,
        "certificate_id": certificate_id,
        "edge": [a, b],
        "status": str(data.get("status", "")),
        "weight": float(data.get("weight", 1.0)),
        "input_binding": {
            "scene_hash": scene_hash(mc.scene),
            "robot_hash": robot_hash(mc.robot),
            "scene": "../../input/scene.yaml",
            "robot": "../../input/robot.yaml",
            "config": "../../input/config.yaml",
        },
        "witness": {
            "kind": "ROTATION",
            "anchors": [np.asarray(anchor, dtype=float) for anchor in anchors],
            "theta_start": theta_start,
            "theta_end": theta_end,
            "theta_min": theta_min,
            "eps_clear": eps_clear,
            "certificate_ids": [certificate_id],
            "provenance": (
                "Recomputed from frozen supports with "
                "verification.continuous.rotation_interval_safe"
            ),
            "checks": checks,
            "all_collision_free": True,
            "all_eps_clear_certified": all(
                row["eps_clear_certified"] for row in checks),
            "min_margin": min(row["min_margin"] for row in checks),
            "eps_min_margin": min(
                row["eps_min_margin"] for row in checks),
        },
    }


def _graphml_value(value):
    if isinstance(value, Enum):
        return value.name
    if isinstance(value, (str, bool, int, float)) and not isinstance(
            value, np.generic):
        return value
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating) and np.isfinite(value):
        return float(value)
    return json.dumps(_jsonable(value), sort_keys=True, allow_nan=False)


def _component_artifact_id(mc, side: str, slab_id: int,
                           component_index: int) -> str:
    decomposition = getattr(mc, "decomposition", None)
    slabs = getattr(decomposition, "slabs", ())
    if not 0 <= int(slab_id) < len(slabs):
        return (
            f"unbound_component[{side},slab={int(slab_id)},"
            f"index={int(component_index)}]"
        )
    slab = slabs[int(slab_id)]
    interval_id = stable_interval_id(
        slab.interval.lo, slab.interval.hi,
    )
    return stable_component_id(
        interval_id, side, int(component_index),
    )


def _deterministic_safe_edge_links(mc) -> dict:
    links = {}
    safe_edges = sorted(
        mc.M_safe.edges(data=True), key=lambda row: tuple(sorted(row[:2])))
    for index, (a, b, _data) in enumerate(safe_edges):
        links[frozenset((a, b))] = (
            f"witnesses/safe_edge_{index:05d}.json",
            f"rotation_safe_edge_{index:05d}",
        )
    return links


def mobility_lineage_payload(mc) -> dict:
    """Canonical complete node/edge/candidate lineage for replay."""
    witness_links = _deterministic_safe_edge_links(mc)
    nodes = []
    for side, graph in (("safe", mc.M_safe),
                        ("possible", mc.M_possible)):
        for node, data in sorted(graph.nodes(data=True)):
            slab_id = int(data["slab"])
            component_index = int(data["comp"])
            decomposition = getattr(mc, "decomposition", None)
            slabs = getattr(decomposition, "slabs", ())
            interval_id = None
            if 0 <= slab_id < len(slabs):
                slab = slabs[slab_id]
                interval_id = stable_interval_id(
                    slab.interval.lo, slab.interval.hi,
                )
            nodes.append({
                "node_id": node,
                "side": side,
                "slab_id": slab_id,
                "interval_id": interval_id,
                "component_index": component_index,
                "component_id": _component_artifact_id(
                    mc, side, slab_id, component_index),
                "interval_cover": bool(data.get("interval_cover", False)),
            })

    edges = []
    for side, graph in (("safe", mc.M_safe),
                        ("possible", mc.M_possible)):
        ordered = sorted(
            graph.edges(data=True),
            key=lambda row: tuple(sorted(row[:2])),
        )
        for a, b, data in ordered:
            node_a, node_b = sorted((a, b))
            a_data, b_data = graph.nodes[node_a], graph.nodes[node_b]
            witness = witness_links.get(frozenset((a, b)))
            if side == "possible" and witness is None \
                    and data.get("witness") is not None:
                key = _witness_key(data.get("witness"))
                for sa, sb, safe_data in mc.M_safe.edges(data=True):
                    if _witness_key(safe_data.get("witness")) == key:
                        witness = witness_links.get(frozenset((sa, sb)))
                        if witness is not None:
                            break
            edges.append({
                "edge_id": f"{side}::{min(a, b)}--{max(a, b)}",
                "side": side,
                "node_a": node_a,
                "node_b": node_b,
                "component_a": _component_artifact_id(
                    mc, side, int(a_data["slab"]), int(a_data["comp"])),
                "component_b": _component_artifact_id(
                    mc, side, int(b_data["slab"]), int(b_data["comp"])),
                "status": str(data.get("status", "")),
                "weight": _float_record(float(data.get("weight", 1.0))),
                "reasons": _jsonable(data.get("reasons", ())),
                "witness_file": witness[0] if witness else None,
                "certificate_id": witness[1] if witness else None,
            })

    mappings = []
    for (slab_id, safe_index), possible_node in sorted(
            getattr(mc, "safe_to_possible", {}).items()):
        possible_data = mc.M_possible.nodes[possible_node]
        mappings.append({
            "slab_id": int(slab_id),
            "safe_component_id": _component_artifact_id(
                mc, "safe", slab_id, safe_index),
            "possible_component_id": _component_artifact_id(
                mc, "possible", int(possible_data["slab"]),
                int(possible_data["comp"])),
            "possible_node_id": possible_node,
        })

    relations = []
    for index, source in enumerate(getattr(mc, "lineage_records", ())):
        row = _jsonable(source)
        side = str(source["side"])
        row.update({
            "record_id": f"candidate_lineage[{index}]",
            "component_a_id": _component_artifact_id(
                mc, side, source["slab_a"], source["component_a"]),
            "component_b_id": _component_artifact_id(
                mc, side, source["slab_b"], source["component_b"]),
        })
        witness = witness_links.get(frozenset((
            source["node_a"], source["node_b"],
        ))) if side == "safe" else None
        row["witness_file"] = witness[0] if witness else None
        row["certificate_id"] = witness[1] if witness else None
        relations.append(row)

    return {
        "schema_version": 1,
        "profile": (
            "complete_component_lineage_v1"
            if getattr(mc, "structural_contract", False)
            else "unbound_graph_lineage_v1"
        ),
        "graph_revision": int(getattr(mc, "graph_revision", 0)),
        "node_count": len(nodes),
        "edge_count": len(edges),
        "safe_to_possible_count": len(mappings),
        "candidate_relation_count": len(relations),
        "nodes": nodes,
        "edges": edges,
        "safe_to_possible": mappings,
        "candidate_relations": relations,
    }


def validate_mobility_lineage_payload(payload, mc) -> dict:
    """Compare a serialized lineage relation with the compiled source."""
    expected = mobility_lineage_payload(mc)
    errors = []
    if payload != expected:
        errors.append("compiled_lineage_replay_mismatch")
    if not isinstance(payload, dict):
        return {"valid": False, "errors": ["payload_type"]}
    relations = payload.get("candidate_relations")
    if not isinstance(relations, list):
        errors.append("candidate_relations_type")
        relations = []
    if payload.get("candidate_relation_count") != len(relations):
        errors.append("candidate_relation_count")
    return {
        "valid": not errors,
        "errors": list(dict.fromkeys(errors)),
        "candidate_relation_count": len(relations),
    }


def save_mobility(run_dir: Path, mc) -> dict:
    run_dir = Path(run_dir)
    witness_dir = run_dir / "mobility" / "witnesses"
    witness_dir.mkdir(parents=True, exist_ok=True)
    safe_edges = sorted(
        mc.M_safe.edges(data=True), key=lambda row: tuple(sorted(row[:2])))
    safe_edge_links = {}
    witness_links = {}
    eps_certified_edges = 0
    for index, (a, b, data) in enumerate(safe_edges):
        certificate_id = f"rotation_safe_edge_{index:05d}"
        filename = f"safe_edge_{index:05d}.json"
        payload = _edge_witness_payload(
            mc, a, b, data, certificate_id=certificate_id)
        _write_json(witness_dir / filename, payload)
        link = f"witnesses/{filename}"
        safe_edge_links[frozenset((a, b))] = (link, certificate_id)
        witness_links.setdefault(_witness_key(data.get("witness")),
                                 (link, certificate_id))
        eps_certified_edges += int(
            payload["witness"]["all_eps_clear_certified"])

    for name, g in (("safe", mc.M_safe), ("possible", mc.M_possible)):
        gg = nx.Graph()
        for node, attributes in g.nodes(data=True):
            node_attributes = {
                str(key): _graphml_value(value)
                for key, value in attributes.items()
            }
            node_attributes["graph_side"] = name
            gg.add_node(node, **node_attributes)
        for a, b, d in g.edges(data=True):
            witness_link = (safe_edge_links.get(frozenset((a, b)))
                            if name == "safe" else None)
            if witness_link is None and d.get("witness") is not None:
                witness_link = witness_links.get(_witness_key(d["witness"]))
            edge_attributes = {
                str(key): _graphml_value(value)
                for key, value in d.items()
                if key not in {"witness", "reasons"}
            }
            edge_attributes.update({
                "weight": float(d.get("weight", 1.0)),
                "status": str(d.get("status", "")),
                "reasons": json.dumps(
                    _jsonable(d.get("reasons", ())), sort_keys=True,
                    allow_nan=False),
                "witness": json.dumps(
                    _jsonable(d.get("witness")), sort_keys=True,
                    allow_nan=False),
                "witness_file": witness_link[0] if witness_link else "",
                "certificate_id": witness_link[1] if witness_link else "",
            })
            gg.add_edge(a, b, **edge_attributes)
        nx.write_graphml(gg, run_dir / "mobility" / f"{name}.graphml")
    lineage_payload = mobility_lineage_payload(mc)
    lineage_report = validate_mobility_lineage_payload(lineage_payload, mc)
    if not lineage_report["valid"]:
        raise RuntimeError(
            "refusing to write invalid mobility lineage: "
            + ", ".join(lineage_report["errors"])
        )
    lineage_path = _write_json(
        run_dir / "mobility" / "lineage.json", lineage_payload,
    )
    return {
        "safe_edge_witness_files": len(safe_edges),
        "safe_edge_eps_clear_certified": eps_certified_edges,
        "lineage_file": "mobility/lineage.json",
        "lineage_sha256": file_sha256(lineage_path),
        "lineage_candidate_relations": lineage_report[
            "candidate_relation_count"],
    }


def _float_record(value: float) -> dict:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("artifact identity values must be finite")
    return {"value": value, "hex": value.hex()}


def _event_regularity_record(slab) -> dict:
    """Serialize samples without promoting them to an interval theorem."""
    return {
        "evidence_scope": "finite_left_mid_right_samples_only",
        "sampled_regular_candidate": bool(slab.predicates.regular_candidate),
        "sampled_predicate_report": _jsonable(slab.predicates),
        # No current event detector supplies an interval-complete proof.  This
        # stays independent from (and false under) a certified obstacle cover.
        "interval_event_free_certified": False,
        "reason": "no_interval_complete_event_exclusion_certificate",
    }


def _geometry_feature(feature_id: str, role: str, geometry, **properties):
    return {
        "type": "Feature",
        "geometry": mapping(geometry),
        "properties": {
            "feature_id": feature_id,
            "role": role,
            **properties,
        },
    }


def _save_certified_interval_cover(root: Path, slab,
                                    interval_id: str,
                                    event_regularity: dict) -> dict:
    """Write one certified slab cover and return its index record."""
    cover = slab.cover_slice
    digest = hashlib.sha256(interval_id.encode("utf-8")).hexdigest()[:20]
    relative_dir = Path("orientation") / "intervals" / f"interval_{digest}"
    artifact_dir = root / relative_dir
    artifact_dir.mkdir(parents=True, exist_ok=False)

    features = []
    pair_records = []
    midpoint_sandwiches = {
        sandwich.pair_id: sandwich
        for sandwich in slab.mid_slice.sandwiches
    }
    for certificate in sorted(
            cover.sandwiches,
            key=lambda item: (item.pair_id.scene_id, item.pair_id.body_id)):
        certificate_interval_id = stable_interval_id(
            certificate.interval.lo, certificate.interval.hi)
        if certificate_interval_id != interval_id:
            raise RuntimeError(
                "interval pair certificate is bound to the wrong slab")
        if certificate.status.name != "CERTIFIED":
            raise RuntimeError(
                "certified interval cover contains an uncertified pair")
        midpoint_sandwich = midpoint_sandwiches.get(certificate.pair_id)
        if midpoint_sandwich is None:
            raise RuntimeError(
                "interval pair certificate has no midpoint sandwich input")
        if (midpoint_sandwich.status.name != "CERTIFIED"
                or float(midpoint_sandwich.theta) != float(certificate.theta)):
            raise RuntimeError(
                "interval pair certificate midpoint input is not certified")
        scene_id = int(certificate.pair_id.scene_id)
        body_id = int(certificate.pair_id.body_id)
        certificate_id = stable_pair_certificate_id(
            interval_id, certificate.theta, scene_id, body_id)
        outer_id = certificate_id + "::geometry[outer_cover]"
        inner_id = certificate_id + "::geometry[inner_common]"
        midpoint_outer_id = certificate_id + "::geometry[midpoint_outer]"
        midpoint_inner_id = certificate_id + "::geometry[midpoint_inner]"
        common_properties = {
            "certificate_id": certificate_id,
            "scene_id": scene_id,
            "body_id": body_id,
            "status": certificate.status.name,
        }
        features.extend((
            _geometry_feature(
                outer_id, "pair_outer_cover", certificate.outer_cover,
                **common_properties),
            _geometry_feature(
                inner_id, "pair_inner_common", certificate.inner_common,
                **common_properties),
            _geometry_feature(
                midpoint_outer_id, "midpoint_pair_outer",
                midpoint_sandwich.outer, **common_properties),
            _geometry_feature(
                midpoint_inner_id, "midpoint_pair_inner",
                midpoint_sandwich.inner, **common_properties),
        ))
        pair_records.append({
            "certificate_id": certificate_id,
            "interval_id": interval_id,
            "pair_id": {
                "scene_id": scene_id,
                "body_id": body_id,
                "stable_id": f"pair[{scene_id},{body_id}]",
            },
            "midpoint_theta": _float_record(certificate.theta),
            "certificate_interval": {
                "lo": _float_record(certificate.interval.lo),
                "hi": _float_record(certificate.interval.hi),
            },
            "theta_lipschitz": _float_record(certificate.theta_lipschitz),
            "motion_bound": _float_record(certificate.motion_bound),
            "status": certificate.status.name,
            "provenance": _jsonable(certificate.provenance),
            "midpoint_sandwich": {
                "status": midpoint_sandwich.status.name,
                "support_directions_rad": [
                    _float_record(angle) for angle in midpoint_sandwich.angles
                ],
                "hausdorff_upper": _float_record(
                    midpoint_sandwich.hausdorff_upper),
                "gap_estimate": _float_record(
                    midpoint_sandwich.gap_estimate),
                "support_calls": int(midpoint_sandwich.support_calls),
                "outer_translation_slack": _float_record(
                    midpoint_sandwich.outer_translation_slack),
                "inner_translation_slack": _float_record(
                    midpoint_sandwich.inner_translation_slack),
                "inner_shrink_factor": _float_record(
                    midpoint_sandwich.inner_shrink_factor),
            },
            "geometry_features": {
                "outer_cover": outer_id,
                "inner_common": inner_id,
                "midpoint_outer": midpoint_outer_id,
                "midpoint_inner": midpoint_inner_id,
            },
        })

    aggregate = {}
    for role, geometry in (("C_plus", cover.C_plus),
                           ("C_minus", cover.C_minus)):
        feature_id = f"{interval_id}::geometry[{role}]"
        aggregate[role] = feature_id
        features.append(_geometry_feature(
            feature_id, f"aggregate_{role}", geometry,
            interval_id=interval_id, status=cover.status.name,
        ))

    component_records = []
    for side, components in (("safe", cover.D_safe),
                             ("possible", cover.D_possible)):
        for index, component in enumerate(components):
            component_id = stable_component_id(interval_id, side, index)
            feature_id = component_id + "::geometry"
            features.append(_geometry_feature(
                feature_id, f"cover_component_{side}", component.geometry,
                component_id=component_id, side=side, index=index,
                status=component.status.name,
            ))
            component_records.append({
                "component_id": component_id,
                "source_component_id": str(component.component_id),
                "interval_id": interval_id,
                "side": side,
                "index": index,
                "status": component.status.name,
                "area": _float_record(component.area),
                "representative": [
                    _float_record(value) for value in component.representative
                ],
                "boundary_signature": _jsonable(
                    component.boundary_signature),
                "causing_pairs": [
                    {"scene_id": int(pair.scene_id),
                     "body_id": int(pair.body_id)}
                    for pair in component.causing_pairs
                ],
                "geometry_feature": feature_id,
            })

    geometry_path = artifact_dir / "geometry.geojson"
    _write_json(geometry_path, {
        "type": "FeatureCollection",
        "schema_version": INTERVAL_TREE_SCHEMA_VERSION,
        "interval_id": interval_id,
        "features": features,
    })
    geometry_digest = file_sha256(geometry_path)
    interval = {
        "lo": _float_record(slab.interval.lo),
        "hi": _float_record(slab.interval.hi),
        "midpoint": _float_record(slab.interval.midpoint),
    }
    certificate_path = artifact_dir / "certificate.json"
    _write_json(certificate_path, {
        "schema_version": INTERVAL_TREE_SCHEMA_VERSION,
        "profile": "interval_pair_cover_v1",
        "slab_id": int(slab.slab_id),
        "interval_id": interval_id,
        "interval": interval,
        "cover_status": cover.status.name,
        "cover_provenance": _jsonable(slab.cover_provenance),
        "event_regularity": event_regularity,
        "pair_certificates": pair_records,
        "aggregate_geometry_features": aggregate,
        "cover_components": component_records,
        "geometry_file": geometry_path.name,
        "geometry_sha256": geometry_digest,
    })
    return {
        "certificate": str(relative_dir / certificate_path.name),
        "certificate_sha256": file_sha256(certificate_path),
        "geometry": str(relative_dir / geometry_path.name),
        "geometry_sha256": geometry_digest,
        "pair_certificate_count": len(pair_records),
        "safe_component_count": len(cover.D_safe),
        "possible_component_count": len(cover.D_possible),
    }


def save_slabs(run_dir: Path, decomposition) -> None:
    """Persist slabs plus the independently replayable interval-cover tranche."""
    run_dir = Path(run_dir)
    interval_root = run_dir / "orientation" / "intervals"
    interval_root.mkdir(parents=True, exist_ok=True)
    if any(interval_root.iterdir()):
        raise FileExistsError(
            "interval artifact directory already contains evidence")

    rows = []
    entries = []
    for slab in sorted(decomposition.slabs,
                       key=lambda item: int(item.slab_id)):
        interval_id = stable_interval_id(slab.interval.lo, slab.interval.hi)
        interval = {
            "lo": _float_record(slab.interval.lo),
            "hi": _float_record(slab.interval.hi),
            "midpoint": _float_record(slab.interval.midpoint),
        }
        event_regularity = _event_regularity_record(slab)
        cover = getattr(slab, "cover_slice", None)
        cover_status = (cover.status.name if cover is not None else "UNKNOWN")
        cover_provenance = _jsonable(
            getattr(slab, "cover_provenance", {}))
        artifact = None
        if (cover is not None and cover_status == "CERTIFIED"
                and cover_provenance.get("status") == "CERTIFIED"):
            artifact = _save_certified_interval_cover(
                run_dir, slab, interval_id, event_regularity)
        entry = {
            "slab_id": int(slab.slab_id),
            "interval_id": interval_id,
            "interval": interval,
            "slab_kind": str(slab.kind),
            "slab_status": slab.status.name,
            "cover_status": cover_status,
            "cover_provenance": cover_provenance,
            "event_regularity": event_regularity,
            "artifact": artifact,
        }
        entries.append(entry)
        rows.append({
            "slab_id": int(slab.slab_id),
            "interval_id": interval_id,
            "lo": float(slab.interval.lo),
            "lo_hex": float(slab.interval.lo).hex(),
            "hi": float(slab.interval.hi),
            "hi_hex": float(slab.interval.hi).hex(),
            "kind": slab.kind,
            "status": slab.status.name,
            "left_theta": float(slab.left_slice.theta),
            "left_theta_hex": float(slab.left_slice.theta).hex(),
            "mid_theta": float(slab.mid_slice.theta),
            "mid_theta_hex": float(slab.mid_slice.theta).hex(),
            "right_theta": float(slab.right_slice.theta),
            "right_theta_hex": float(slab.right_slice.theta).hex(),
            "n_safe": len(slab.mid_slice.D_safe),
            "n_possible": len(slab.mid_slice.D_possible),
            "predicates": slab.predicates,
            "event_regularity": event_regularity,
            "cover_status": cover_status,
            "cover_provenance": cover_provenance,
            "interval_cover_artifact": artifact,
        })
    _write_json(run_dir / "slabs" / "slabs.json", rows)

    global_status = getattr(
        getattr(decomposition, "global_possible_cover_status", None),
        "name", "UNKNOWN")
    global_provenance = _jsonable(getattr(
        decomposition, "global_possible_cover_provenance", {}))
    _write_json(run_dir / INTERVAL_INDEX_RELATIVE, {
        "schema_version": INTERVAL_TREE_SCHEMA_VERSION,
        "profile": "interval_pair_cover_v1",
        "slab_count": len(entries),
        "certified_interval_cover_count": sum(
            int(entry["cover_status"] == "CERTIFIED") for entry in entries),
        "global_cover_status": global_status,
        "global_cover_provenance": global_provenance,
        "event_proof_contract": {
            "evidence_scope": "finite_left_mid_right_samples_only",
            "interval_event_free_certified": False,
            "reason": "no_interval_complete_event_exclusion_certificate",
            "independent_from_interval_obstacle_cover": True,
        },
        "slabs": entries,
    })


def _valid_float_record(record) -> bool:
    if not isinstance(record, dict) or set(record) != {"value", "hex"}:
        return False
    try:
        value = float(record["value"])
    except (TypeError, ValueError):
        return False
    return math.isfinite(value) and record["hex"] == value.hex()


def _fixed_slice_inventory(decomposition):
    cache_items = sorted(
        decomposition.slice_cache.items(), key=lambda item: float(item[0]),
    )
    unique = {}
    for _cache_key, fixed_slice in cache_items:
        unique.setdefault(id(fixed_slice), fixed_slice)
    ordered = sorted(unique.values(), key=lambda item: float(item.theta))
    theta_hexes = [float(item.theta).hex() for item in ordered]
    if len(theta_hexes) != len(set(theta_hexes)):
        raise ValueError("multiple fixed-slice objects share one exact theta")
    slice_ids = {
        id(fixed_slice): f"fixed_slice::{float(fixed_slice.theta).hex()}"
        for fixed_slice in ordered
    }
    return cache_items, ordered, slice_ids


def _direction_schedule_id(angles: np.ndarray,
                           directions: np.ndarray) -> str:
    digest = hashlib.sha256()
    digest.update(b"GMC_FIXED_DIRECTIONS_V1\0")
    digest.update(np.asarray(angles, dtype="<f8").tobytes())
    digest.update(np.asarray(directions, dtype="<f8").tobytes())
    return "direction_schedule::" + digest.hexdigest()[:24]


def fixed_slice_direction_set_payload(fixed_slice, slice_id: str) -> dict:
    """One slice's exact schedules, deduplicated across Gaussian pairs."""
    schedules_by_id = {}
    pair_bindings = []
    for sandwich in sorted(
            fixed_slice.sandwiches,
            key=lambda item: (
                int(item.pair_id.scene_id), int(item.pair_id.body_id),
            )):
        angles = np.asarray(sandwich.angles, dtype=float)
        directions = np.asarray(sandwich.directions, dtype=float)
        if (angles.ndim != 1 or directions.shape != (len(angles), 2)
                or not np.all(np.isfinite(angles))
                or not np.all(np.isfinite(directions))):
            raise ValueError("fixed-slice direction schedule is invalid")
        schedule_id = _direction_schedule_id(angles, directions)
        schedule = {
            "schedule_id": schedule_id,
            "direction_count": len(angles),
            "angles_rad": [_float_record(value) for value in angles],
            "unit_directions": [
                {
                    "x": _float_record(direction[0]),
                    "y": _float_record(direction[1]),
                }
                for direction in directions
            ],
        }
        previous = schedules_by_id.setdefault(schedule_id, schedule)
        if previous != schedule:
            raise RuntimeError("direction schedule digest collision")
        pair_bindings.append({
            "pair_id": {
                "scene_id": int(sandwich.pair_id.scene_id),
                "body_id": int(sandwich.pair_id.body_id),
            },
            "status": sandwich.status.name,
            "schedule_id": schedule_id,
        })
    schedules = [schedules_by_id[key] for key in sorted(schedules_by_id)]
    return {
        "schema_version": 1,
        "profile": "fixed_slice_direction_set_v1",
        "slice_id": slice_id,
        "theta": _float_record(fixed_slice.theta),
        "status": fixed_slice.status.name,
        "pair_schedule_count": len(pair_bindings),
        "unique_schedule_count": len(schedules),
        "schedules": schedules,
        "pair_bindings": pair_bindings,
    }


def validate_fixed_slice_direction_set_payload(
        payload, fixed_slice=None, slice_id: str | None = None) -> dict:
    """Validate one bounded slice file and optionally replay it exactly."""
    errors = []
    if not isinstance(payload, dict):
        return {"valid": False, "errors": ["payload_type"],
                "pair_schedule_count": 0, "unique_schedule_count": 0}
    if (payload.get("schema_version") != 1
            or payload.get("profile") != "fixed_slice_direction_set_v1"):
        errors.append("schema")
    if not isinstance(payload.get("slice_id"), str):
        errors.append("slice_id")
    if slice_id is not None and payload.get("slice_id") != slice_id:
        errors.append("slice_id_binding")
    if not _valid_float_record(payload.get("theta")):
        errors.append("theta")
    schedules = payload.get("schedules")
    bindings = payload.get("pair_bindings")
    if not isinstance(schedules, list):
        errors.append("schedules_type")
        schedules = []
    if not isinstance(bindings, list):
        errors.append("pair_bindings_type")
        bindings = []
    schedule_ids = set()
    for index, schedule in enumerate(schedules):
        prefix = f"schedule_{index}"
        if not isinstance(schedule, dict):
            errors.append(prefix + "_type")
            continue
        schedule_id = schedule.get("schedule_id")
        angles = schedule.get("angles_rad")
        directions = schedule.get("unit_directions")
        if (not isinstance(schedule_id, str)
                or schedule_id in schedule_ids):
            errors.append(prefix + "_id")
        else:
            schedule_ids.add(schedule_id)
        if (not isinstance(angles, list)
                or not isinstance(directions, list)
                or len(angles) < 3 or len(angles) != len(directions)
                or schedule.get("direction_count") != len(angles)):
            errors.append(prefix + "_shape")
            continue
        if not all(_valid_float_record(item) for item in angles):
            errors.append(prefix + "_angles")
        if not all(
                isinstance(direction, dict)
                and set(direction) == {"x", "y"}
                and _valid_float_record(direction["x"])
                and _valid_float_record(direction["y"])
                for direction in directions):
            errors.append(prefix + "_directions")
            continue
        angle_values = np.asarray(
            [item["value"] for item in angles], dtype=float)
        direction_values = np.asarray([
            [item["x"]["value"], item["y"]["value"]]
            for item in directions
        ], dtype=float)
        if schedule_id != _direction_schedule_id(
                angle_values, direction_values):
            errors.append(prefix + "_digest")
    for index, binding in enumerate(bindings):
        if (not isinstance(binding, dict)
                or binding.get("schedule_id") not in schedule_ids
                or not isinstance(binding.get("pair_id"), dict)):
            errors.append(f"pair_binding_{index}")
    if payload.get("pair_schedule_count") != len(bindings):
        errors.append("pair_schedule_count")
    if payload.get("unique_schedule_count") != len(schedules):
        errors.append("unique_schedule_count")
    if fixed_slice is not None:
        try:
            expected = fixed_slice_direction_set_payload(
                fixed_slice, slice_id or payload.get("slice_id"),
            )
        except (AttributeError, TypeError, ValueError) as exc:
            errors.append("compiler_replay_error:" + type(exc).__name__)
        else:
            if payload != expected:
                errors.append("compiler_replay_mismatch")
    return {
        "valid": not errors,
        "errors": list(dict.fromkeys(errors)),
        "pair_schedule_count": len(bindings),
        "unique_schedule_count": len(schedules),
    }


def validate_fixed_slice_direction_artifact_tree(
        run_dir: str | Path, *, decomposition=None, index=None) -> dict:
    """Stream-validate the direction index and every compressed slice file."""
    run_dir = Path(run_dir).resolve()
    errors = []
    index_path = run_dir / "slices" / "direction_sets.json"
    try:
        if index is None:
            index = json.loads(index_path.read_text())
    except (OSError, ValueError, json.JSONDecodeError):
        return {"valid": False, "errors": ["index_invalid"],
                "unique_slice_count": 0, "cache_key_count": 0,
                "pair_schedule_count": 0, "unique_schedule_count": 0}
    if (not isinstance(index, dict)
            or index.get("schema_version") != 1
            or index.get("profile") != "fixed_slice_direction_index_v1"):
        errors.append("index_schema")
    files = index.get("slice_files") if isinstance(index, dict) else None
    cache_keys = index.get("cache_keys") if isinstance(index, dict) else None
    if not isinstance(files, list):
        errors.append("slice_files_type")
        files = []
    if not isinstance(cache_keys, list):
        errors.append("cache_keys_type")
        cache_keys = []
    expected_by_id = {}
    if decomposition is not None:
        try:
            _items, expected_slices, expected_ids = _fixed_slice_inventory(
                decomposition)
            expected_by_id = {
                expected_ids[id(item)]: item for item in expected_slices
            }
        except (AttributeError, TypeError, ValueError) as exc:
            errors.append("compiler_inventory_error:" + type(exc).__name__)
    seen_ids = set()
    pair_count = 0
    unique_schedule_count = 0
    for index_number, record in enumerate(files):
        prefix = f"slice_file_{index_number}"
        if not isinstance(record, dict):
            errors.append(prefix + "_type")
            continue
        slice_id = record.get("slice_id")
        relative = record.get("path")
        if (not isinstance(slice_id, str) or slice_id in seen_ids
                or not isinstance(relative, str)):
            errors.append(prefix + "_binding")
            continue
        seen_ids.add(slice_id)
        path = (run_dir / relative).resolve()
        try:
            path.relative_to(run_dir)
        except ValueError:
            errors.append(prefix + "_path_escape")
            continue
        if not path.is_file():
            errors.append(prefix + "_missing")
            continue
        if record.get("sha256") != file_sha256(path):
            errors.append(prefix + "_digest")
        try:
            payload = _read_gzip_json(path)
        except (OSError, ValueError, json.JSONDecodeError):
            errors.append(prefix + "_invalid")
            continue
        report = validate_fixed_slice_direction_set_payload(
            payload, fixed_slice=expected_by_id.get(slice_id),
            slice_id=slice_id,
        )
        if not report["valid"]:
            errors.extend(prefix + "_" + item for item in report["errors"])
        pair_count += report["pair_schedule_count"]
        unique_schedule_count += report["unique_schedule_count"]
        if record.get("pair_schedule_count") != report[
                "pair_schedule_count"]:
            errors.append(prefix + "_pair_count")
        if record.get("unique_schedule_count") != report[
                "unique_schedule_count"]:
            errors.append(prefix + "_unique_count")
    if decomposition is not None and seen_ids != set(expected_by_id):
        errors.append("compiler_slice_set_mismatch")
    for index_number, cache_record in enumerate(cache_keys):
        if (not isinstance(cache_record, dict)
                or not _valid_float_record(cache_record.get("theta_key"))
                or cache_record.get("slice_id") not in seen_ids):
            errors.append(f"cache_key_{index_number}")
    if index.get("unique_slice_count") != len(files):
        errors.append("unique_slice_count")
    if index.get("cache_key_count") != len(cache_keys):
        errors.append("cache_key_count")
    if index.get("pair_schedule_count") != pair_count:
        errors.append("pair_schedule_count")
    if index.get("unique_schedule_count") != unique_schedule_count:
        errors.append("unique_schedule_count")
    return {
        "valid": not errors,
        "errors": list(dict.fromkeys(errors)),
        "unique_slice_count": len(files),
        "cache_key_count": len(cache_keys),
        "pair_schedule_count": pair_count,
        "unique_schedule_count": unique_schedule_count,
    }


def save_fixed_slice_direction_sets(run_dir: Path, decomposition) -> Path:
    """Persist exact directions per slice without retaining the full tree."""
    run_dir = Path(run_dir)
    index_path = run_dir / "slices" / "direction_sets.json"
    directory = run_dir / "slices" / "direction_sets"
    if index_path.exists() or directory.exists():
        raise FileExistsError(
            "fixed-slice direction proof already exists; refusing to mix evidence"
        )
    directory.mkdir(parents=True)
    cache_items, slices, slice_ids = _fixed_slice_inventory(decomposition)
    records = []
    pair_count = 0
    unique_schedule_count = 0
    for fixed_slice in slices:
        slice_id = slice_ids[id(fixed_slice)]
        digest = hashlib.sha256(slice_id.encode("utf-8")).hexdigest()[:20]
        relative = f"slices/direction_sets/slice_{digest}.json.gz"
        payload = fixed_slice_direction_set_payload(fixed_slice, slice_id)
        report = validate_fixed_slice_direction_set_payload(
            payload, fixed_slice=fixed_slice, slice_id=slice_id,
        )
        if not report["valid"]:
            raise RuntimeError(
                "refusing to write invalid fixed-slice directions: "
                + ", ".join(report["errors"])
            )
        path = _write_gzip_json(run_dir / relative, payload)
        records.append({
            "slice_id": slice_id,
            "theta": _float_record(fixed_slice.theta),
            "path": relative,
            "sha256": file_sha256(path),
            "pair_schedule_count": report["pair_schedule_count"],
            "unique_schedule_count": report["unique_schedule_count"],
        })
        pair_count += report["pair_schedule_count"]
        unique_schedule_count += report["unique_schedule_count"]
    index = {
        "schema_version": 1,
        "profile": "fixed_slice_direction_index_v1",
        "construction": (
            "exact binary64 rows retained by the final pair sandwiches; "
            "identical schedules are content-deduplicated within each slice"
        ),
        "unique_slice_count": len(slices),
        "cache_key_count": len(cache_items),
        "pair_schedule_count": pair_count,
        "unique_schedule_count": unique_schedule_count,
        "cache_keys": [
            {"theta_key": _float_record(cache_key),
             "slice_id": slice_ids[id(fixed_slice)]}
            for cache_key, fixed_slice in cache_items
        ],
        "slice_files": records,
    }
    _write_json(index_path, index)
    tree_report = validate_fixed_slice_direction_artifact_tree(
        run_dir, decomposition=decomposition, index=index,
    )
    if not tree_report["valid"]:
        raise RuntimeError(
            "fixed-slice direction artifact tree failed replay: "
            + ", ".join(tree_report["errors"])
        )
    return index_path


def save_pair_pruning(run_dir: Path, pair_query, *, leaf_size: int = 8) -> Path:
    """Persist every retained/rejected Cartesian pair and its exact bound."""
    payload = pair_pruning_payload(pair_query, leaf_size=leaf_size)
    path = Path(run_dir) / "pairs" / "pruning.json"
    if path.exists():
        raise FileExistsError(
            "pair-pruning proof already exists; refusing to mix evidence"
        )
    return _write_json(path, payload)


def save_intermediate_geometry(run_dir: Path, scene, pair_query,
                               decomposition) -> dict:
    """Persist inspectable, non-pickle geometry/provenance artifacts."""
    run_dir = Path(run_dir)
    workspace_feature = {
        "type": "Feature",
        "geometry": mapping(scene.workspace),
        "properties": {"kind": "workspace", "scene": scene.name},
    }
    _write_json(run_dir / "input" / "workspace.geojson", workspace_feature)

    pair_stats = asdict(pair_query.stats) if is_dataclass(pair_query.stats) \
        else dict(pair_query.stats)
    pair_stats["retained_pair_ids"] = [
        {"scene_id": o.pair_id.scene_id, "body_id": o.pair_id.body_id}
        for o in pair_query.oracles
    ]
    _write_json(run_dir / "pairs" / "stats.json", pair_stats)

    # Cache values are the actual unique fixed-orientation slices.  Sort by
    # theta so file names and manifest counts are independent of traversal.
    seen = set()
    slices = []
    for sl in decomposition.slice_cache.values():
        marker = id(sl)
        if marker not in seen:
            seen.add(marker)
            slices.append(sl)
    slices.sort(key=lambda sl: float(sl.theta))

    for index, sl in enumerate(slices):
        sdir = run_dir / "slices" / f"slice_{index:05d}"
        sdir.mkdir(parents=True, exist_ok=True)
        meta = {
            "slice_index": index,
            "theta": float(sl.theta),
            "status": sl.status.name,
            "support_calls": int(sl.support_calls),
            "safe_component_count": len(sl.D_safe),
            "possible_component_count": len(sl.D_possible),
            "sandwich_count": len(sl.sandwiches),
        }
        _write_json(sdir / "meta.json", meta)

        component_features = []
        for side, components in (("safe", sl.D_safe),
                                 ("possible", sl.D_possible)):
            for component in components:
                component_features.append({
                    "type": "Feature",
                    "geometry": mapping(component.geometry),
                    "properties": {
                        "side": side,
                        "component_id": component.component_id,
                        "area": float(component.area),
                        "representative": component.representative,
                        "boundary_signature": component.boundary_signature,
                        "status": component.status.name,
                    },
                })
        _write_json(sdir / "components.geojson", {
            "type": "FeatureCollection", "features": component_features})

        sandwich_features = []
        directions = []
        for sandwich in sorted(
                sl.sandwiches,
                key=lambda s: (s.pair_id.scene_id, s.pair_id.body_id)):
            pair = {"scene_id": sandwich.pair_id.scene_id,
                    "body_id": sandwich.pair_id.body_id}
            common = {
                "pair_id": pair,
                "status": sandwich.status.name,
                "hausdorff_upper": sandwich.hausdorff_upper,
                "gap_estimate": float(sandwich.gap_estimate),
                "support_calls": int(sandwich.support_calls),
                "outer_translation_slack": float(
                    sandwich.outer_translation_slack),
                "inner_translation_slack": float(
                    sandwich.inner_translation_slack),
                "inner_shrink_factor": float(
                    sandwich.inner_shrink_factor),
            }
            for bound, geom in (("inner", sandwich.inner),
                                ("outer", sandwich.outer)):
                sandwich_features.append({
                    "type": "Feature", "geometry": mapping(geom),
                    "properties": {**common, "bound": bound},
                })
            angles = np.asarray(sandwich.angles, dtype=float)
            directions.append({
                **common,
                "angles_rad": angles,
                "unit_directions": np.asarray(
                    sandwich.directions, dtype=float),
            })
        _write_json(sdir / "sandwiches.geojson", {
            "type": "FeatureCollection", "features": sandwich_features})
        _write_json(sdir / "directions.json", directions)
        _write_json(sdir / "provenance.json", {
            "inputs": {
                "scene": "../../input/scene.yaml",
                "robot": "../../input/robot.yaml",
                "config": "../../input/config.yaml",
            },
            "theta": float(sl.theta),
            "construction": (
                "C_plus is the union of pair outer envelopes; C_minus is "
                "the union of pair inner envelopes; D_safe=workspace\\C_plus; "
                "D_possible=workspace\\C_minus."
            ),
            "certificate_scope": sl.status.name,
        })
        _write_json(sdir / "sandwich_unions.geojson", {
            "type": "FeatureCollection",
            "features": [
                {"type": "Feature", "geometry": mapping(sl.C_plus),
                 "properties": {"bound": "C_plus"}},
                {"type": "Feature", "geometry": mapping(sl.C_minus),
                 "properties": {"bound": "C_minus"}},
            ],
        })
    return {
        "unique_slice_artifact_count": len(slices),
        "workspace_geojson": "input/workspace.geojson",
        "pair_stats": "pairs/stats.json",
    }


def _query_proof_bindings(run_dir: Path, query_id: str, result, mc,
                          request: dict) -> dict:
    """Create proof objects and bind every returned path segment to one."""
    from ..mobility.witness import PoseCurve
    from ..types import PlanStatus, Pose2
    from ..verification.path import verify_curve

    qdir = Path(run_dir) / "queries" / query_id
    base = {
        "schema_version": 1,
        "profile": "query_proof_bindings_v1",
        "query_id": query_id,
        "status": result.status.name,
        "input_binding": {
            "scene_hash": scene_hash(mc.scene),
            "robot_hash": robot_hash(mc.robot),
            "config": "../../input/config.yaml",
        },
        "graph_revision": int(getattr(mc, "graph_revision", 0)),
        "graph_path": list(result.graph_path),
        "request": _jsonable(request),
        "segment_provenance": _jsonable(
            getattr(result, "segment_provenance", ())),
        "segments": [],
        "verdict_evidence": _jsonable(result.report),
    }
    if result.status is PlanStatus.REACHABLE:
        if result.curve is None or not result.curve.segments:
            raise RuntimeError("REACHABLE result has no curve to bind")
        bound_segments = []
        segment_records = []
        for index, segment in enumerate(result.curve.segments):
            material = {
                "query_id": query_id,
                "index": index,
                "kind": segment.kind.name,
                "q0": [*map(float, segment.q0.xy), float(segment.q0.theta)],
                "q1": [*map(float, segment.q1.xy), float(segment.q1.theta)],
                "control_points": [
                    [*map(float, pose.xy), float(pose.theta)]
                    for pose in segment.control_points
                ],
                "scene_hash": base["input_binding"]["scene_hash"],
                "robot_hash": base["input_binding"]["robot_hash"],
                "graph_revision": base["graph_revision"],
            }
            digest = hashlib.sha256(json.dumps(
                material, sort_keys=True, separators=(",", ":"),
                allow_nan=False,
            ).encode("utf-8")).hexdigest()[:24]
            certificate_id = f"query_segment::{digest}"
            bound = replace(
                segment,
                certificate_ids=tuple(dict.fromkeys((
                    *segment.certificate_ids, certificate_id,
                ))),
            )
            report = verify_curve(
                mc.oracles, mc.scene.workspace, PoseCurve((bound,)),
                mc.cfg.query.eps_clear, mc.cfg.orientation.theta_min,
                expected_start=bound.q0, expected_goal=bound.q1,
            )
            if not report.certified:
                raise RuntimeError(
                    "query segment proof replay failed at segment "
                    f"{index}: {report.reason}"
                )
            provenance = (
                result.segment_provenance[index]
                if index < len(result.segment_provenance) else None
            )
            segment_records.append({
                "certificate_id": certificate_id,
                "segment_index": index,
                "material": material,
                "certificate_ids": list(bound.certificate_ids),
                "ownership": _jsonable(provenance),
                "independent_verification": _jsonable(report),
            })
            bound_segments.append(bound)
        result.curve = PoseCurve(tuple(bound_segments))

        try:
            raw_start = request["start"]
            raw_goal = request["goal"]
            expected_start = Pose2(
                np.asarray(raw_start[:2], dtype=float), float(raw_start[2]))
            expected_goal = Pose2(
                np.asarray(raw_goal[:2], dtype=float), float(raw_goal[2]))
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            raise ValueError(
                "query proof binding requires exact start/goal request"
            ) from exc
        whole = verify_curve(
            mc.oracles, mc.scene.workspace, result.curve,
            mc.cfg.query.eps_clear, mc.cfg.orientation.theta_min,
            expected_start=expected_start, expected_goal=expected_goal,
        )
        if not whole.certified:
            raise RuntimeError(
                "whole query proof replay failed: " + whole.reason
            )
        base["proof_kind"] = "verified_pose_curve"
        base["segments"] = segment_records
        base["whole_curve_verification"] = _jsonable(whole)
    elif result.status is PlanStatus.UNREACHABLE:
        cut = result.report.get("cut_certificate")
        cover = result.report.get("global_possible_cover")
        if not isinstance(cut, dict) or not isinstance(cover, dict):
            raise RuntimeError(
                "UNREACHABLE result lacks cut/global-cover proof"
            )
        base["proof_kind"] = "certified_possible_graph_cut"
        base["cut_certificate"] = _jsonable(cut)
        base["global_possible_cover"] = _jsonable(cover)
    else:
        base["proof_kind"] = "explicit_nonterminal_or_invalid_result"

    proof_path = _write_json(qdir / "proofs.json", base)
    return {
        "profile": base["profile"],
        "path": "proofs.json",
        "sha256": file_sha256(proof_path),
        "status": "complete",
        "proof_kind": base["proof_kind"],
        "segment_certificate_count": len(base["segments"]),
    }


def _save_failed_query_case(run_dir: Path, query_id: str, result, mc,
                            request: dict, proof_binding: dict) -> dict | None:
    """Persist a deterministic reproducer for non-terminal query outcomes."""
    from ..types import PlanStatus

    if result.status in {PlanStatus.REACHABLE, PlanStatus.UNREACHABLE}:
        return None
    if not bool(mc.cfg.logging.save_failed_cases):
        return None
    relative = "failure_case.json"
    qdir = Path(run_dir) / "queries" / query_id
    path = _write_json(qdir / relative, {
        "schema_version": 1,
        "profile": "gmc_query_failure_case_v1",
        "query_id": query_id,
        "status": result.status.name,
        "input_binding": {
            "scene_hash": scene_hash(mc.scene),
            "robot_hash": robot_hash(mc.robot),
            "scene": "../../input/scene.yaml",
            "robot": "../../input/robot.yaml",
            "config": "../../input/config.yaml",
        },
        "request": _jsonable(request),
        "reason": _jsonable(result.report.get("reason")),
        "ambiguity": _jsonable(result.ambiguity),
        "refinement_trace": _jsonable(
            result.report.get("refinement_trace", ())),
        "graph_revision": int(getattr(mc, "graph_revision", 0)),
        "query_proof_binding": _jsonable(proof_binding),
        "replay_contract": {
            "compiler": "rebuild from the frozen run inputs",
            "query": "reissue the exact start and goal arrays above",
            "serialized_graph_is_authoritative": False,
        },
    })
    return {
        "profile": "gmc_query_failure_case_v1",
        "path": relative,
        "sha256": file_sha256(path),
        "status": "complete",
    }


def save_plan_result(run_dir: Path, query_id: str, result, versions=True,
                     request: dict | None = None,
                     artifact_scope: dict | None = None,
                     proof_context=None) -> Path:
    qdir = Path(run_dir) / "queries" / query_id
    qdir.mkdir(parents=True, exist_ok=True)
    request = request or {}
    proof_binding = None
    failed_case_binding = None
    if proof_context is not None:
        proof_binding = _query_proof_bindings(
            run_dir, query_id, result, proof_context, request,
        )
        failed_case_binding = _save_failed_query_case(
            run_dir, query_id, result, proof_context, request, proof_binding,
        )
    scope = dict(artifact_scope or {
        "execution_graph": "not_recorded",
        "missing": [
            "query execution graph provenance was not supplied",
            ("end-to-end PoseCurve proof binding; certificate_ids are "
             "preserved per segment but may be empty"),
        ],
    })
    if proof_binding is not None:
        scope["query_proof_binding"] = proof_binding
        produced = list(scope.get("produced", ()))
        if "proofs.json" not in produced:
            produced.append("proofs.json")
        if failed_case_binding is not None \
                and failed_case_binding["path"] not in produced:
            produced.append(failed_case_binding["path"])
        scope["produced"] = produced
        scope["missing"] = [
            item for item in scope.get("missing", ())
            if "PoseCurve proof binding" not in str(item)
        ]
    if failed_case_binding is not None:
        scope["failed_case_binding"] = failed_case_binding
    payload = {
        "schema_version": 2,
        "status": result.status.name,
        "query_id": query_id,
        "graph_path": list(result.graph_path),
        "clearance_lower_bound": result.clearance_lower_bound,
        "ambiguity": result.ambiguity,
        "report": result.report,
        "request": request,
        "artifact_scope": scope,
        "query_proof_binding": proof_binding,
        "failed_case_binding": failed_case_binding,
        "software": _versions() if versions else {},
    }
    if result.curve is not None:
        payload["trajectory_file"] = "path.json"
        _write_json(qdir / "path.json", {
            "schema_version": 2,
            "query_id": query_id,
            "segments": [{
                "kind": seg.kind.name,
                "q0": [*map(float, seg.q0.xy), float(seg.q0.theta)],
                "q1": [*map(float, seg.q1.xy), float(seg.q1.theta)],
                "control_points": [[*map(float, p.xy), float(p.theta)]
                                   for p in seg.control_points],
                "certificate_ids": seg.certificate_ids,
            } for seg in result.curve.segments],
        })
    return _write_json(qdir / "result.json", payload)


def validate_saved_query_proof(query_dir: str | Path, scene, robot) -> dict:
    """Validate query proof hashes and per-segment content bindings."""
    query_dir = Path(query_dir).resolve()
    errors = []
    result_path = query_dir / "result.json"
    path_path = query_dir / "path.json"
    if not result_path.is_file():
        return {"present": False, "valid": False,
                "errors": ["result_missing"]}
    try:
        result = json.loads(result_path.read_text())
    except (OSError, ValueError, json.JSONDecodeError):
        return {"present": True, "valid": False,
                "errors": ["result_invalid"]}
    binding = result.get("query_proof_binding")
    if not isinstance(binding, dict):
        return {"present": False, "valid": False,
                "errors": ["query_proof_binding_missing"]}
    relative = binding.get("path")
    if not isinstance(relative, str) or not relative:
        return {"present": True, "valid": False,
                "errors": ["query_proof_path"]}
    proof_path = (query_dir / relative).resolve()
    try:
        proof_path.relative_to(query_dir)
    except ValueError:
        return {"present": True, "valid": False,
                "errors": ["query_proof_path_escape"]}
    if not proof_path.is_file():
        return {"present": True, "valid": False,
                "errors": ["query_proof_missing"]}
    if binding.get("sha256") != file_sha256(proof_path):
        errors.append("query_proof_digest")
    try:
        proof = json.loads(proof_path.read_text())
    except (OSError, ValueError, json.JSONDecodeError):
        return {"present": True, "valid": False,
                "errors": [*errors, "query_proof_invalid"]}
    if (proof.get("schema_version") != 1
            or proof.get("profile") != "query_proof_bindings_v1"):
        errors.append("query_proof_schema")
    if proof.get("query_id") != result.get("query_id"):
        errors.append("query_id_binding")
    if proof.get("status") != result.get("status"):
        errors.append("query_status_binding")
    if proof.get("request") != result.get("request"):
        errors.append("query_request_binding")
    if proof.get("graph_path") != result.get("graph_path"):
        errors.append("query_graph_path_binding")
    if (binding.get("profile") != proof.get("profile")
            or binding.get("proof_kind") != proof.get("proof_kind")
            or binding.get("status") != "complete"):
        errors.append("query_proof_metadata_binding")
    inputs = proof.get("input_binding")
    if (not isinstance(inputs, dict)
            or inputs.get("scene_hash") != scene_hash(scene)
            or inputs.get("robot_hash") != robot_hash(robot)):
        errors.append("query_input_binding")

    if proof.get("proof_kind") == "verified_pose_curve":
        if not path_path.is_file():
            errors.append("query_path_missing")
            path = {"segments": []}
        else:
            try:
                path = json.loads(path_path.read_text())
            except (OSError, ValueError, json.JSONDecodeError):
                errors.append("query_path_invalid")
                path = {"segments": []}
        path_segments = path.get("segments", ())
        proof_segments = proof.get("segments", ())
        if (not isinstance(path_segments, list)
                or not isinstance(proof_segments, list)
                or len(path_segments) != len(proof_segments)):
            errors.append("query_segment_count_binding")
        else:
            for index, (segment, record) in enumerate(zip(
                    path_segments, proof_segments)):
                if not isinstance(segment, dict) or not isinstance(record, dict):
                    errors.append(f"query_segment_{index}_schema")
                    continue
                material = {
                    "query_id": result.get("query_id"),
                    "index": index,
                    "kind": segment.get("kind"),
                    "q0": segment.get("q0"),
                    "q1": segment.get("q1"),
                    "control_points": segment.get("control_points", []),
                    "scene_hash": scene_hash(scene),
                    "robot_hash": robot_hash(robot),
                    "graph_revision": proof.get("graph_revision"),
                }
                try:
                    digest = hashlib.sha256(json.dumps(
                        material, sort_keys=True, separators=(",", ":"),
                        allow_nan=False,
                    ).encode("utf-8")).hexdigest()[:24]
                except (TypeError, ValueError):
                    errors.append(f"query_segment_{index}_material")
                    continue
                certificate_id = f"query_segment::{digest}"
                if (record.get("material") != material
                        or record.get("certificate_id") != certificate_id
                        or certificate_id not in segment.get(
                            "certificate_ids", ())
                        or record.get("certificate_ids")
                        != segment.get("certificate_ids")):
                    errors.append(f"query_segment_{index}_binding")
                verification = record.get("independent_verification")
                if (not isinstance(verification, dict)
                        or verification.get("certified") is not True):
                    errors.append(f"query_segment_{index}_verification")
        whole = proof.get("whole_curve_verification")
        if (not isinstance(whole, dict)
                or whole.get("certified") is not True):
            errors.append("whole_curve_verification")
        if binding.get("segment_certificate_count") != len(proof_segments):
            errors.append("query_segment_count_metadata")
    return {
        "present": True,
        "valid": not errors,
        "errors": list(dict.fromkeys(errors)),
        "proof_kind": proof.get("proof_kind"),
    }


def save_verify_result(query_dir: Path, query_id: str, report=None,
                       *, error: Exception | None = None,
                       input_hashes: dict | None = None,
                       proof_validation: dict | None = None) -> Path:
    if report is not None:
        payload = {
            "schema_version": 1,
            "query_id": query_id,
            "certified": bool(report.certified),
            "min_clearance": report.min_clearance,
            "failed_segment": report.failed_segment,
            "reason": report.reason,
            "input_hashes": input_hashes or {},
            "query_proof_validation": _jsonable(proof_validation),
            "software": _versions(),
        }
    else:
        payload = {
            "schema_version": 1,
            "query_id": query_id,
            "certified": False,
            "min_clearance": None,
            "failed_segment": None,
            "reason": "verification_error",
            "error": {
                "type": type(error).__name__ if error is not None else "Error",
                "message": str(error) if error is not None else "unknown error",
            },
            "input_hashes": input_hashes or {},
            "query_proof_validation": _jsonable(proof_validation),
            "software": _versions(),
        }
    return _write_json(Path(query_dir) / "verify.json", payload)
