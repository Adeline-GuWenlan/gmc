"""Independent validation for serialized interval-cover evidence.

This module deliberately consumes only JSON/GeoJSON files.  It does not trust
the in-memory compiler objects that produced them, so it can detect truncated,
missing, mixed, or edited evidence before a manifest is treated as replayable.
The interval-cover tranche is narrower than the full Guide I7 contract: a
valid tree proves that this tranche was serialized consistently, not that all
of I7 (event brackets, pruning proofs, lineage, and query proofs) is complete.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import shapely
from shapely.geometry import shape


INTERVAL_INDEX_RELATIVE = "orientation/interval_cover_index.json"
INTERVAL_TREE_SCHEMA_VERSION = 1


def file_sha256(path: str | Path) -> str:
    """Return the complete-file SHA-256 used to bind an artifact tree."""
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def stable_interval_id(lo: float, hi: float) -> str:
    """Binary64-exact interval identity, independent of decimal formatting."""
    lo = float(lo)
    hi = float(hi)
    if not math.isfinite(lo) or not math.isfinite(hi) or not lo < hi:
        raise ValueError("interval identity requires finite lo < hi")
    return f"theta[{lo.hex()},{hi.hex()}]"


def stable_pair_certificate_id(interval_id: str, theta: float,
                               scene_id: int, body_id: int) -> str:
    """Stable identity for one interval/pair theorem certificate."""
    theta = float(theta)
    if not isinstance(interval_id, str) or not interval_id:
        raise ValueError("pair certificate requires an interval identity")
    if not math.isfinite(theta):
        raise ValueError("pair certificate theta must be finite")
    return (f"{interval_id}::mid[{theta.hex()}]::"
            f"pair[{int(scene_id)},{int(body_id)}]")


def stable_component_id(interval_id: str, side: str, index: int) -> str:
    """Stable identity for an ordered interval-cover component."""
    if side not in {"safe", "possible"}:
        raise ValueError("component side must be safe or possible")
    if not isinstance(index, int) or isinstance(index, bool) or index < 0:
        raise ValueError("component index must be a non-negative integer")
    return f"{interval_id}::component[{side},{index}]"


def _resolve_relative(run_dir: Path, relative: object) -> Path:
    if not isinstance(relative, str) or not relative:
        raise ValueError("artifact path must be a non-empty relative string")
    candidate = (run_dir / relative).resolve()
    try:
        candidate.relative_to(run_dir.resolve())
    except ValueError as exc:
        raise ValueError(f"artifact path escapes run directory: {relative}") from exc
    return candidate


def _read_json(path: Path):
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def _check_float_record(record: object, name: str, errors: list[str]) -> float | None:
    if not isinstance(record, dict):
        errors.append(f"{name}_record")
        return None
    value, encoded = record.get("value"), record.get("hex")
    if (not isinstance(value, (int, float)) or isinstance(value, bool)
            or not isinstance(encoded, str)):
        errors.append(f"{name}_record")
        return None
    try:
        decoded = float.fromhex(encoded)
    except ValueError:
        errors.append(f"{name}_hex")
        return None
    value = float(value)
    if not math.isfinite(value) or decoded != value or value.hex() != encoded:
        errors.append(f"{name}_hex_binding")
        return None
    return value


def _empty_or_valid_polygonal(geometry) -> bool:
    if geometry.is_empty:
        return True
    return bool(
        isinstance(geometry, (shapely.Polygon, shapely.MultiPolygon))
        and geometry.is_valid
        and math.isfinite(float(geometry.area))
    )


def _validate_interval_payload(run_dir: Path, entry: dict,
                               expected_paths: set[str],
                               errors: list[str]) -> None:
    slab_id = entry.get("slab_id")
    label = f"slab_{slab_id}"
    interval = entry.get("interval")
    if not isinstance(interval, dict):
        errors.append(f"{label}_interval")
        return
    lo = _check_float_record(interval.get("lo"), f"{label}_lo", errors)
    hi = _check_float_record(interval.get("hi"), f"{label}_hi", errors)
    mid = _check_float_record(
        interval.get("midpoint"), f"{label}_midpoint", errors)
    if lo is None or hi is None or mid is None:
        return
    try:
        interval_id = stable_interval_id(lo, hi)
    except ValueError:
        errors.append(f"{label}_interval_order")
        return
    if entry.get("interval_id") != interval_id:
        errors.append(f"{label}_interval_id")
    if mid != float(0.5 * (lo + hi)):
        errors.append(f"{label}_midpoint_value")

    event = entry.get("event_regularity")
    if (not isinstance(event, dict)
            or event.get("evidence_scope")
            != "finite_left_mid_right_samples_only"
            or event.get("interval_event_free_certified") is not False):
        errors.append(f"{label}_event_scope")

    status = entry.get("cover_status")
    artifact = entry.get("artifact")
    if status != "CERTIFIED":
        if artifact is not None:
            errors.append(f"{label}_uncertified_artifact")
        return
    if not isinstance(artifact, dict):
        errors.append(f"{label}_missing_artifact_record")
        return
    cover_provenance = entry.get("cover_provenance")
    if (not isinstance(cover_provenance, dict)
            or cover_provenance.get("status") != "CERTIFIED"
            or cover_provenance.get("evidence_scope")
            != "entire_orientation_interval"):
        errors.append(f"{label}_cover_provenance")

    try:
        certificate_path = _resolve_relative(
            run_dir, artifact.get("certificate"))
        geometry_path = _resolve_relative(run_dir, artifact.get("geometry"))
    except ValueError:
        errors.append(f"{label}_artifact_path")
        return
    certificate_relative = str(certificate_path.relative_to(run_dir.resolve()))
    geometry_relative = str(geometry_path.relative_to(run_dir.resolve()))
    expected_paths.update((certificate_relative, geometry_relative))
    for role, path, declared in (
            ("certificate", certificate_path,
             artifact.get("certificate_sha256")),
            ("geometry", geometry_path, artifact.get("geometry_sha256"))):
        if not path.is_file():
            errors.append(f"{label}_{role}_missing")
            return
        if not isinstance(declared, str) or file_sha256(path) != declared:
            errors.append(f"{label}_{role}_digest")
            return

    try:
        payload = _read_json(certificate_path)
        geojson = _read_json(geometry_path)
    except (OSError, ValueError, json.JSONDecodeError):
        errors.append(f"{label}_artifact_json")
        return
    if payload.get("schema_version") != INTERVAL_TREE_SCHEMA_VERSION:
        errors.append(f"{label}_certificate_schema")
    if payload.get("interval_id") != interval_id:
        errors.append(f"{label}_certificate_interval_id")
    if payload.get("interval") != interval:
        errors.append(f"{label}_certificate_interval_binding")
    if payload.get("cover_status") != "CERTIFIED":
        errors.append(f"{label}_certificate_cover_status")
    if payload.get("event_regularity") != event:
        errors.append(f"{label}_event_binding")
    if payload.get("cover_provenance") != entry.get("cover_provenance"):
        errors.append(f"{label}_cover_provenance_binding")
    if payload.get("geometry_file") != geometry_path.name:
        errors.append(f"{label}_geometry_link")
    if payload.get("geometry_sha256") != file_sha256(geometry_path):
        errors.append(f"{label}_geometry_payload_digest")

    features = geojson.get("features")
    if geojson.get("type") != "FeatureCollection" or not isinstance(features, list):
        errors.append(f"{label}_geometry_collection")
        return
    geometries = {}
    properties = {}
    try:
        for feature in features:
            if not isinstance(feature, dict) or feature.get("type") != "Feature":
                raise ValueError
            props = feature.get("properties")
            if not isinstance(props, dict):
                raise ValueError
            feature_id = props.get("feature_id")
            if not isinstance(feature_id, str) or feature_id in geometries:
                raise ValueError
            geometry = shape(feature.get("geometry"))
            if not _empty_or_valid_polygonal(geometry):
                raise ValueError
            geometries[feature_id] = geometry
            properties[feature_id] = props
    except (TypeError, ValueError, shapely.GEOSException):
        errors.append(f"{label}_invalid_geometry_feature")
        return

    pair_certificates = payload.get("pair_certificates")
    if not isinstance(pair_certificates, list):
        errors.append(f"{label}_pair_certificates")
        return
    seen_pairs = set()
    pair_outer, pair_inner = [], []
    for pair in pair_certificates:
        if not isinstance(pair, dict):
            errors.append(f"{label}_pair_record")
            continue
        pair_id = pair.get("pair_id")
        if not isinstance(pair_id, dict):
            errors.append(f"{label}_pair_id")
            continue
        scene_id, body_id = pair_id.get("scene_id"), pair_id.get("body_id")
        if (not isinstance(scene_id, int) or isinstance(scene_id, bool)
                or not isinstance(body_id, int) or isinstance(body_id, bool)):
            errors.append(f"{label}_pair_id")
            continue
        key = (scene_id, body_id)
        if key in seen_pairs:
            errors.append(f"{label}_duplicate_pair")
        seen_pairs.add(key)
        expected_id = stable_pair_certificate_id(
            interval_id, mid, scene_id, body_id)
        if pair.get("certificate_id") != expected_id:
            errors.append(f"{label}_pair_certificate_id")
        if pair.get("status") != "CERTIFIED":
            errors.append(f"{label}_pair_not_certified")
        provenance = pair.get("provenance")
        if (not isinstance(provenance, dict)
                or provenance.get("construction")
                != "midpoint_hausdorff_interval_pair_cover"
                or provenance.get("reason") != "ok"
                or not isinstance(provenance.get("checks"), dict)
                or not provenance["checks"]
                or any(value is not True
                       for value in provenance["checks"].values())):
            errors.append(f"{label}_pair_provenance")
        pair_theta = _check_float_record(
            pair.get("midpoint_theta"), f"{label}_pair_theta", errors)
        if pair_theta is not None and pair_theta != mid:
            errors.append(f"{label}_pair_theta_binding")
        certificate_interval = pair.get("certificate_interval")
        if not isinstance(certificate_interval, dict):
            errors.append(f"{label}_pair_interval")
        else:
            pair_lo = _check_float_record(
                certificate_interval.get("lo"),
                f"{label}_pair_interval_lo", errors)
            pair_hi = _check_float_record(
                certificate_interval.get("hi"),
                f"{label}_pair_interval_hi", errors)
            if pair_lo is not None and pair_hi is not None \
                    and (pair_lo != lo or pair_hi != hi):
                errors.append(f"{label}_pair_interval_binding")
        lipschitz = _check_float_record(
            pair.get("theta_lipschitz"), f"{label}_pair_lipschitz", errors)
        motion_bound = _check_float_record(
            pair.get("motion_bound"), f"{label}_pair_motion_bound", errors)
        if (lipschitz is not None and motion_bound is not None
                and motion_bound < lipschitz * max(mid - lo, hi - mid)):
            errors.append(f"{label}_pair_motion_bound_binding")
        midpoint_sandwich = pair.get("midpoint_sandwich")
        if (not isinstance(midpoint_sandwich, dict)
                or midpoint_sandwich.get("status") != "CERTIFIED"):
            errors.append(f"{label}_midpoint_sandwich")
        else:
            directions = midpoint_sandwich.get("support_directions_rad")
            decoded_directions = []
            if not isinstance(directions, list) or len(directions) < 3:
                errors.append(f"{label}_midpoint_directions")
            else:
                for direction_index, direction in enumerate(directions):
                    value = _check_float_record(
                        direction,
                        f"{label}_midpoint_direction_{direction_index}",
                        errors)
                    if value is not None:
                        decoded_directions.append(value % math.tau)
                ordered = sorted(decoded_directions)
                if len(ordered) == len(directions):
                    gaps = [right - left
                            for left, right in zip(ordered, ordered[1:])]
                    gaps.append(ordered[0] + math.tau - ordered[-1])
                    if any(gap <= 0.0 or gap >= math.pi for gap in gaps):
                        errors.append(f"{label}_midpoint_direction_schedule")
            for field in (
                    "hausdorff_upper", "gap_estimate",
                    "outer_translation_slack", "inner_translation_slack",
                    "inner_shrink_factor"):
                _check_float_record(
                    midpoint_sandwich.get(field),
                    f"{label}_midpoint_{field}", errors)
            support_calls = midpoint_sandwich.get("support_calls")
            if (not isinstance(support_calls, int)
                    or isinstance(support_calls, bool) or support_calls < 0):
                errors.append(f"{label}_midpoint_support_calls")
        refs = pair.get("geometry_features")
        if not isinstance(refs, dict):
            errors.append(f"{label}_pair_geometry_refs")
            continue
        outer_id, inner_id = refs.get("outer_cover"), refs.get("inner_common")
        midpoint_outer_id = refs.get("midpoint_outer")
        midpoint_inner_id = refs.get("midpoint_inner")
        if (outer_id not in geometries or inner_id not in geometries
                or midpoint_outer_id not in geometries
                or midpoint_inner_id not in geometries):
            errors.append(f"{label}_pair_geometry_missing")
            continue
        outer, inner = geometries[outer_id], geometries[inner_id]
        midpoint_outer = geometries[midpoint_outer_id]
        midpoint_inner = geometries[midpoint_inner_id]
        outer_props, inner_props = properties[outer_id], properties[inner_id]
        midpoint_outer_props = properties[midpoint_outer_id]
        midpoint_inner_props = properties[midpoint_inner_id]
        if (outer_props.get("role") != "pair_outer_cover"
                or inner_props.get("role") != "pair_inner_common"
                or midpoint_outer_props.get("role")
                != "midpoint_pair_outer"
                or midpoint_inner_props.get("role")
                != "midpoint_pair_inner"
                or outer_props.get("certificate_id") != expected_id
                or inner_props.get("certificate_id") != expected_id
                or midpoint_outer_props.get("certificate_id") != expected_id
                or midpoint_inner_props.get("certificate_id") != expected_id):
            errors.append(f"{label}_pair_geometry_provenance")
        if (outer.is_empty or midpoint_outer.is_empty
                or not inner.difference(outer).is_empty
                or not midpoint_inner.difference(midpoint_outer).is_empty
                or not midpoint_outer.difference(outer).is_empty
                or not inner.difference(midpoint_inner).is_empty):
            errors.append(f"{label}_pair_geometry_nesting")
        pair_outer.append(outer)
        pair_inner.append(inner)
    if artifact.get("pair_certificate_count") != len(pair_certificates):
        errors.append(f"{label}_pair_count_binding")

    aggregate = payload.get("aggregate_geometry_features")
    if not isinstance(aggregate, dict):
        errors.append(f"{label}_aggregate_refs")
        return
    plus_id, minus_id = aggregate.get("C_plus"), aggregate.get("C_minus")
    if plus_id not in geometries or minus_id not in geometries:
        errors.append(f"{label}_aggregate_geometry_missing")
        return
    c_plus, c_minus = geometries[plus_id], geometries[minus_id]
    if (properties[plus_id].get("role") != "aggregate_C_plus"
            or properties[minus_id].get("role") != "aggregate_C_minus"
            or properties[plus_id].get("interval_id") != interval_id
            or properties[minus_id].get("interval_id") != interval_id):
        errors.append(f"{label}_aggregate_geometry_provenance")
    if not c_minus.difference(c_plus).is_empty:
        errors.append(f"{label}_aggregate_nesting")
    try:
        expected_plus = shapely.union_all(pair_outer) if pair_outer \
            else shapely.Polygon()
        expected_minus = shapely.union_all(pair_inner) if pair_inner \
            else shapely.Polygon()
        # ``assemble_slice`` applies one additional direction-preserving
        # precision operation to every leaf.  Its outer leaves may therefore
        # expand and its inner leaves may shrink.  These are the exact sound
        # replay obligations; topological equality with the pre-snap pair
        # geometries would be an invalid requirement.
        if not expected_plus.difference(c_plus).is_empty:
            errors.append(f"{label}_C_plus_pair_cover")
        if not c_minus.difference(expected_minus).is_empty:
            errors.append(f"{label}_C_minus_pair_common")
    except shapely.GEOSException:
        errors.append(f"{label}_aggregate_union")

    component_records = payload.get("cover_components")
    if not isinstance(component_records, list):
        errors.append(f"{label}_components")
        return
    by_side = {"safe": [], "possible": []}
    expected_indices = {"safe": 0, "possible": 0}
    for component in component_records:
        if not isinstance(component, dict):
            errors.append(f"{label}_component_record")
            continue
        side, index = component.get("side"), component.get("index")
        if side not in by_side or index != expected_indices.get(side):
            errors.append(f"{label}_component_order")
            continue
        expected_indices[side] += 1
        expected_id = stable_component_id(interval_id, side, index)
        if component.get("component_id") != expected_id:
            errors.append(f"{label}_component_id")
        feature_id = component.get("geometry_feature")
        if feature_id not in geometries:
            errors.append(f"{label}_component_geometry_missing")
            continue
        geometry = geometries[feature_id]
        props = properties[feature_id]
        if (props.get("role") != f"cover_component_{side}"
                or props.get("component_id") != expected_id
                or props.get("side") != side
                or props.get("index") != index):
            errors.append(f"{label}_component_geometry_provenance")
        if geometry.is_empty:
            errors.append(f"{label}_component_empty")
        area = _check_float_record(
            component.get("area"), f"{label}_component_area", errors)
        if area is not None and float(geometry.area).hex() != area.hex():
            errors.append(f"{label}_component_area_binding")
        by_side[side].append(geometry)
    if artifact.get("safe_component_count") != len(by_side["safe"]):
        errors.append(f"{label}_safe_component_count_binding")
    if artifact.get("possible_component_count") != len(by_side["possible"]):
        errors.append(f"{label}_possible_component_count_binding")
    try:
        safe_union = shapely.union_all(by_side["safe"]) \
            if by_side["safe"] else shapely.Polygon()
        possible_union = shapely.union_all(by_side["possible"]) \
            if by_side["possible"] else shapely.Polygon()
        if not safe_union.difference(possible_union).is_empty:
            errors.append(f"{label}_free_component_nesting")
    except shapely.GEOSException:
        errors.append(f"{label}_component_union")


def validate_interval_artifact_tree(
        run_dir: str | Path, manifest: dict | None = None) -> dict:
    """Validate the interval-cover tranche and return a structured report.

    The function never upgrades the bundle to full I7.  It validates file
    hashes, exact binary64 identities, cross-file provenance, geometry
    nesting, full-circle claims, and the explicit absence of an interval
    event-free theorem.
    """
    run_dir = Path(run_dir).resolve()
    errors: list[str] = []
    checked_files = 0
    index_path = run_dir / INTERVAL_INDEX_RELATIVE
    if not index_path.is_file():
        return {"valid": False, "errors": ["interval_index_missing"],
                "checked_files": 0}
    try:
        index = _read_json(index_path)
    except (OSError, ValueError, json.JSONDecodeError):
        return {"valid": False, "errors": ["interval_index_invalid"],
                "checked_files": 0}
    checked_files += 1
    if index.get("schema_version") != INTERVAL_TREE_SCHEMA_VERSION:
        errors.append("interval_index_schema")
    if index.get("profile") != "interval_pair_cover_v1":
        errors.append("interval_index_profile")
    event = index.get("event_proof_contract")
    if (not isinstance(event, dict)
            or event.get("interval_event_free_certified") is not False
            or event.get("evidence_scope")
            != "finite_left_mid_right_samples_only"):
        errors.append("global_event_scope")
    entries = index.get("slabs")
    if not isinstance(entries, list) or not entries:
        errors.append("interval_index_slabs")
        entries = []
    if index.get("slab_count") != len(entries):
        errors.append("interval_index_slab_count")

    expected_paths: set[str] = set()
    intervals = []
    certified = 0
    for expected_slab_id, entry in enumerate(entries):
        if not isinstance(entry, dict):
            errors.append(f"slab_{expected_slab_id}_entry")
            continue
        if entry.get("slab_id") != expected_slab_id:
            errors.append(f"slab_{expected_slab_id}_id_order")
        interval = entry.get("interval")
        if isinstance(interval, dict):
            lo_record, hi_record = interval.get("lo"), interval.get("hi")
            if isinstance(lo_record, dict) and isinstance(hi_record, dict):
                try:
                    intervals.append((float.fromhex(lo_record["hex"]),
                                      float.fromhex(hi_record["hex"])))
                except (KeyError, TypeError, ValueError):
                    pass
        certified += int(entry.get("cover_status") == "CERTIFIED")
        _validate_interval_payload(
            run_dir, entry, expected_paths, errors)

    if index.get("certified_interval_cover_count") != certified:
        errors.append("interval_index_certified_count")

    actual_paths = {
        str(path.resolve().relative_to(run_dir))
        for path in (run_dir / "orientation" / "intervals").rglob("*")
        if path.is_file()
    } if (run_dir / "orientation" / "intervals").is_dir() else set()
    if actual_paths != expected_paths:
        if expected_paths - actual_paths:
            errors.append("interval_tree_declared_file_missing")
        if actual_paths - expected_paths:
            errors.append("interval_tree_unexpected_file")
    checked_files += len(expected_paths & actual_paths)

    full_circle = bool(intervals and intervals[0][0] == 0.0)
    if full_circle:
        for left, right in zip(intervals, intervals[1:]):
            if left[1] != right[0]:
                full_circle = False
                break
        full_circle = full_circle and intervals[-1][1] == float(2.0 * math.pi)
    global_status = index.get("global_cover_status")
    global_provenance = index.get("global_cover_provenance")
    if global_status not in {"CERTIFIED", "UNKNOWN"}:
        errors.append("global_cover_status")
    if not isinstance(global_provenance, dict):
        errors.append("global_cover_provenance")
    else:
        if (global_provenance.get("certified_interval_cover_count") != certified
                or global_provenance.get("slab_count") != len(entries)
                or global_provenance.get("full_circle_tiling") != full_circle):
            errors.append("global_cover_provenance_binding")
        if global_status == "CERTIFIED":
            if certified != len(entries) or not full_circle:
                errors.append("global_cover_claim")
            if (global_provenance.get("evidence_scope")
                    != "certified_full_circle_interval_cover"):
                errors.append("global_cover_provenance_binding")

    if manifest is not None:
        declared = manifest.get("interval_cover_artifacts") \
            if isinstance(manifest, dict) else None
        if not isinstance(declared, dict):
            errors.append("manifest_interval_artifacts")
        else:
            if declared.get("index") != INTERVAL_INDEX_RELATIVE:
                errors.append("manifest_interval_index_path")
            if declared.get("index_sha256") != file_sha256(index_path):
                errors.append("manifest_interval_index_digest")
            bindings = {
                "schema_version": index.get("schema_version"),
                "profile": index.get("profile"),
                "slab_count": index.get("slab_count"),
                "certified_interval_cover_count": certified,
                "global_cover_status": global_status,
                "global_cover_provenance": global_provenance,
                "interval_event_free_certified": False,
            }
            for key, expected in bindings.items():
                if declared.get(key) != expected:
                    errors.append(f"manifest_interval_{key}_binding")

    return {
        "valid": not errors,
        "errors": list(dict.fromkeys(errors)),
        "checked_files": checked_files,
        "slab_count": len(entries),
        "certified_interval_cover_count": certified,
        "global_cover_status": global_status,
        "interval_event_free_certified": False,
    }
