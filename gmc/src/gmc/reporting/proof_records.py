"""Canonical replay records for proof-critical compiler decisions.

This module intentionally contains no artifact-directory policy.  It turns an
in-memory BVH query into deterministic JSON data and can independently rebuild
that query from frozen inputs to detect deletion, reordering, or edited bounds.
"""

from dataclasses import asdict
import math

from ..spatial.bvh import query_candidate_pairs


_FLOAT_FIELDS = (
    "bound_radius",
    "descendant_center_distance",
    "descendant_radius",
    "descendant_containment_margin",
    "workspace_expansion",
    "body_extent",
    "workspace_distance",
    "threshold",
    "coordinate_scale",
    "coordinate_ulp",
    "rounding_slack",
    "comparison_rhs",
)


def _number(value: float) -> dict:
    value = float(value)
    if math.isfinite(value):
        return {"value": value, "hex": value.hex(), "finite": True}
    return {
        "value": (
            "NaN" if math.isnan(value)
            else "Infinity" if value > 0.0 else "-Infinity"
        ),
        "hex": None,
        "finite": False,
    }


def pair_pruning_payload(pair_query, *, leaf_size: int = 8) -> dict:
    """Encode all Cartesian pair decisions with exact binary64 identities."""
    records = []
    for decision in pair_query.decisions:
        raw = asdict(decision)
        raw["bound_center"] = [
            _number(value) for value in decision.bound_center
        ]
        for field in _FLOAT_FIELDS:
            raw[field] = _number(getattr(decision, field))
        records.append(raw)
    stats = asdict(pair_query.stats)
    retained = [
        {
            "scene_id": int(oracle.pair_id.scene_id),
            "body_id": int(oracle.pair_id.body_id),
        }
        for oracle in pair_query.oracles
    ]
    return {
        "schema_version": 1,
        "profile": "exact_bvh_pair_pruning_v1",
        "leaf_size": int(leaf_size),
        "ordering": "scene_tuple_index_then_body_tuple_index",
        "decision_count": len(records),
        "stats": stats,
        "retained_pair_ids": retained,
        "decisions": records,
    }


def validate_pair_pruning_payload(payload, scene, robot, workspace) -> dict:
    """Rebuild and compare a pruning proof against the frozen inputs."""
    errors = []
    if not isinstance(payload, dict):
        return {"valid": False, "errors": ["payload_type"]}
    leaf_size = payload.get("leaf_size")
    if (not isinstance(leaf_size, int) or isinstance(leaf_size, bool)
            or leaf_size < 1):
        return {"valid": False, "errors": ["leaf_size"]}
    if payload.get("schema_version") != 1:
        errors.append("schema_version")
    if payload.get("profile") != "exact_bvh_pair_pruning_v1":
        errors.append("profile")
    if payload.get("ordering") != \
            "scene_tuple_index_then_body_tuple_index":
        errors.append("ordering")
    try:
        rebuilt = query_candidate_pairs(
            scene, robot, workspace, leaf_size=leaf_size,
        )
        expected = pair_pruning_payload(rebuilt, leaf_size=leaf_size)
    except Exception as exc:
        return {
            "valid": False,
            "errors": ["rebuild_failed"],
            "error_type": type(exc).__name__,
            "error": str(exc),
        }
    if payload != expected:
        errors.append("frozen_input_replay_mismatch")

    decisions = payload.get("decisions")
    total = len(scene.supports) * len(robot.supports)
    if not isinstance(decisions, list):
        errors.append("decisions_type")
        decisions = []
    if payload.get("decision_count") != len(decisions):
        errors.append("decision_count")
    if len(decisions) != total:
        errors.append("cartesian_pair_completeness")
    ids = [row.get("record_id") for row in decisions
           if isinstance(row, dict)]
    if len(ids) != len(set(ids)):
        errors.append("record_id_uniqueness")
    return {
        "valid": not errors,
        "errors": list(dict.fromkeys(errors)),
        "decision_count": len(decisions),
        "retained_count": len(payload.get("retained_pair_ids", ())),
    }


__all__ = [
    "pair_pruning_payload",
    "validate_pair_pruning_payload",
]
