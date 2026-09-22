"""Deterministic airborne-edit showcase scene for the GS3D handoff.

This module deliberately only prepares scene data.  It does not plan a route or
turn the enclosure witness into a collision certificate.  The archive produced
by :func:`build_showcase_derivative` is the one later planners and renderers
must load; its manifest includes byte hashes and checks exact added rows.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np

from gmc.height.planefloor import load_plane_scene
from gmc.height.ply3d import GaussianScene3D

TAU = 0.3
LEVEL = 2.0
UAV_RADIUS_M = 0.25
UAV_HALF_HEIGHT_M = 0.10
UAV_MARGIN_M = 0.05
SCENE_ID = "showcase_planefloor_airborne_v1"

# The source is deliberately external/read-only.  The derived archive is always
# written by the caller to a worktree-local output location.
DEFAULT_SOURCE = Path("/scratch/wg2381/splathjb/gmc/outputs/height/plane/processed_planefloor.npz")
DEFAULT_ORIGINAL = Path("/scratch/wg2381/splathjb/splatc_atlas/data/gs_scenes/showcase/processed.npz")
DEFAULT_PLY = Path("/scratch/wg2381/splathjb/splatc_atlas/data/gs_scenes/showcase/raw/point_cloud.ply")


@dataclass(frozen=True)
class ManualGaussian:
    """One explicitly manual scene-edit primitive, in the world frame."""

    edit_id: str
    role: str
    mean_world_m: tuple[float, float, float]
    semiaxes_route_m: tuple[float, float, float]
    opacity: float
    color_rgb: tuple[float, float, float]
    note: str


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(8 << 20), b""):
            h.update(block)
    return h.hexdigest()


def route_frame(z_floor: float) -> tuple[np.ndarray, np.ndarray]:
    """Return world origin and world-to-route orthonormal rotation.

    A route row is ``local @ R + origin``.  Route axes are forward ``u``,
    lateral ``v`` and world-up.  This matches A0's accepted fixture exactly.
    """
    origin = np.array([9.0, 5.75, float(z_floor)], dtype=float)
    world_to_route = np.array([[0.6, 0.8, 0.0], [-0.8, 0.6, 0.0], [0.0, 0.0, 1.0]])
    return origin, world_to_route


def _world_from_route(local: np.ndarray, origin: np.ndarray, world_to_route: np.ndarray) -> np.ndarray:
    return np.asarray(local, dtype=float) @ world_to_route + origin


def _cov_from_route_axes(axes: tuple[float, float, float], world_to_route: np.ndarray) -> np.ndarray:
    # An ellipsoid semiaxis a at LEVEL has covariance eigenvalue (a/LEVEL)^2.
    route_cov = np.diag((np.asarray(axes, dtype=float) / LEVEL) ** 2)
    return world_to_route.T @ route_cov @ world_to_route


def manual_edits(z_floor: float) -> tuple[ManualGaussian, ...]:
    origin, R = route_frame(z_floor)
    rows = (
        ("a2_hanging_shade", "hanging_light_shade", (0.0, 0.0, 1.20), (.18, .65, .15),
         (1.0, .68, .05), "manual opaque hanging-light shade; underside is 1.05 m above floor"),
        ("a2_light_suspension", "light_suspension", (0.0, 0.0, 3.25), (.025, .025, 1.95),
         (.48, .48, .50), "manual opaque suspension from shade to ceiling attachment"),
        ("a2_ceiling_attachment", "ceiling_obstruction", (0.0, 0.0, 5.20), (.45, .85, .10),
         (.72, .75, .80), "manual opaque ceiling attachment; contextual obstruction"),
    )
    return tuple(ManualGaussian(name, role,
                                tuple(_world_from_route(np.array(local), origin, R)),
                                axes, .95, color, note)
                 for name, role, local, axes, color, note in rows)


def _source_paths(source: Path) -> list[dict[str, Any]]:
    paths = [("edited_floor_source", Path(source)), ("decoded_original", DEFAULT_ORIGINAL),
             ("original_ply", DEFAULT_PLY)]
    return [{"role": role, "path": str(path), "bytes": path.stat().st_size,
             "sha256": sha256(path)} for role, path in paths]


def _aabb(scene: GaussianScene3D, level: float = LEVEL) -> tuple[np.ndarray, np.ndarray]:
    return scene.aabb(level)


def _table_supports(scene: GaussianScene3D, z_floor: float) -> dict[str, Any]:
    """Label original table-A support candidates from their complete 3-D AABBs.

    The closed route-frame envelope is intentionally wider than the body gate:
    it records the full tabletop arrangement around the high crossing rather
    than just a convenient centreline sample.  It does not delete or replace
    any original Gaussian.
    """
    origin, R = route_frame(z_floor)
    local = (scene.means - origin) @ R.T
    cov_local = R @ scene.covs @ R.T
    half = LEVEL * np.sqrt(np.maximum(np.einsum("nii->ni", cov_local), 0.0))
    lo, hi = local - half, local + half
    opaque = scene.opacity > TAU
    # Table-A measured arrangement envelope: forward section surrounding the
    # old centreline s=[.92,1.88], full lateral top and legs; heights above floor.
    envelope_lo = np.array([.65, -1.20, .15])
    envelope_hi = np.array([2.15, 1.20, 1.25])
    hit = opaque & np.all(hi >= envelope_lo, axis=1) & np.all(lo <= envelope_hi, axis=1)
    if not np.any(hit):
        raise ValueError("table-A support envelope selected no opaque source Gaussians")
    sel_lo, sel_hi = lo[hit], hi[hit]
    return {
        "selection": "opaque source Gaussian 2-sigma AABB intersects closed table-A route envelope",
        "manual_edit": False,
        "route_envelope_m": {"lower": envelope_lo.tolist(), "upper": envelope_hi.tolist()},
        "source_ids": [int(x) for x in scene.ids[hit]],
        "count": int(hit.sum()),
        "route_aabb_union_m": {"lower": sel_lo.min(axis=0).tolist(), "upper": sel_hi.max(axis=0).tolist()},
        "top_height_above_floor_m": float(sel_hi[:, 2].max()),
        "note": "Original table geometry retained; this is a label/measurement, not a manual edit.",
    }


def _edit_rows(edits: tuple[ManualGaussian, ...], z_floor: float, first_id: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, list[dict[str, Any]]]:
    _, R = route_frame(z_floor)
    means = np.asarray([x.mean_world_m for x in edits], dtype=float)
    covs = np.asarray([_cov_from_route_axes(x.semiaxes_route_m, R) for x in edits], dtype=float)
    opacity = np.asarray([x.opacity for x in edits], dtype=float)
    ids = np.arange(first_id, first_id + len(edits), dtype=np.int64)
    entries = []
    for edit, ident, cov in zip(edits, ids, covs):
        local_mean = (np.asarray(edit.mean_world_m) - route_frame(z_floor)[0]) @ R.T
        entries.append({**asdict(edit), "manual_edit": True, "gaussian_id": int(ident),
                        "mean_route_m": local_mean.tolist(), "covariance_world_m2": cov.tolist(),
                        "covariance_eigenvalues_m2": np.linalg.eigvalsh(cov).tolist(),
                        "level": LEVEL, "units": {"position": "m", "covariance": "m^2"},
                        "frame": "world xyz, z up; route frame documented in manifest"})
    return means, covs, opacity, ids, entries


def candidate_knots_world(z_floor: float) -> np.ndarray:
    origin, R = route_frame(z_floor)
    local = np.array([[-.25, 0.0, .65], [.50, 0.0, .65], [.50, 0.0, 1.40], [2.50, 0.0, 1.40]])
    return _world_from_route(local, origin, R)


def _candidate_enclosure(scene: GaussianScene3D, z_floor: float) -> dict[str, Any]:
    """A0-style continuous enclosing-box audit of the exact output scene.

    It is intentionally marked as an enclosure feasibility witness, not A1's
    production ellipsoid/body oracle certificate.
    """
    origin, R = route_frame(z_floor)
    local = (scene.means - origin) @ R.T
    cov_local = R @ scene.covs @ R.T
    half = LEVEL * np.sqrt(np.maximum(np.einsum("nii->ni", cov_local), 0.0))
    lo, hi = local - half, local + half
    crop_lo, crop_hi = np.array([-1.5, -1.5, -.1]), np.array([3.5, 1.5, 2.5])
    mask = (scene.opacity > TAU) & np.all(hi >= crop_lo, axis=1) & np.all(lo <= crop_hi, axis=1)
    lo, hi, ids = lo[mask], hi[mask], scene.ids[mask]
    knots = np.array([[-.25, 0.0, .65], [.50, 0.0, .65], [.50, 0.0, 1.40], [2.50, 0.0, 1.40]])
    body = np.array([UAV_RADIUS_M, UAV_RADIUS_M, UAV_HALF_HEIGHT_M])
    legs = []
    for a, b in zip(knots[:-1], knots[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / .02)))
        worst, witness = float("inf"), None
        for j in range(n):
            aa, bb = a + (b - a) * j / n, a + (b - a) * (j + 1) / n
            blo, bhi = np.minimum(aa, bb) - body, np.maximum(aa, bb) + body
            gap = np.maximum(np.maximum(lo - bhi, blo - hi), 0.0)
            bounds = np.linalg.norm(gap, axis=1) - 1e-8
            k = int(np.argmin(bounds))
            if bounds[k] < worst:
                worst, witness = float(bounds[k]), int(ids[k])
        legs.append({"a_route_m": a.tolist(), "b_route_m": b.tolist(), "subintervals": n,
                     "clearance_lower_m": worst, "limiting_gaussian_id": witness,
                     "passes_strict_0p05_m": bool(worst > UAV_MARGIN_M)})
    return {"method": "closed 0.02 m swept upright-body boxes against complete 2-sigma Gaussian AABB boxes; lower bound with 1e-8 m slack",
            "strength": "conservative enclosure feasibility witness only; not a planner result or production oracle certificate",
            "candidate_count": int(mask.sum()), "crop_route_m": {"lower": crop_lo.tolist(), "upper": crop_hi.tolist()},
            "body": {"shape": "upright_cylinder_enclosed_by_box", "radius_m": UAV_RADIUS_M,
                     "half_height_m": UAV_HALF_HEIGHT_M, "margin_m": UAV_MARGIN_M},
            "legs": legs, "all_legs_pass": bool(all(x["passes_strict_0p05_m"] for x in legs)),
            "altitude_swing_m": .75,
            "ordered_constraints": {"low_under_light_before_high_over_table": True,
                                    "low_crossing_route_s_m": [-.20, .20],
                                    "high_table_crossing_route_s_m": [.92, 1.88]},
    }


def build_showcase_derivative(source: Path, output: Path, manifest: Path) -> dict[str, Any]:
    """Append the frozen manual airborne edit and atomically record its identity."""
    source, output, manifest = Path(source), Path(output), Path(manifest)
    before = _source_paths(source)
    base, meta = load_plane_scene(source)
    z_floor = float(meta["z_floor"])
    edits = manual_edits(z_floor)
    first_id = int(base.ids.max()) + 1
    means, covs, opacity, ids, entries = _edit_rows(edits, z_floor, first_id)
    edited = GaussianScene3D(np.vstack([base.means, means]), np.concatenate([base.covs, covs]),
                              np.concatenate([base.opacity, opacity]), np.concatenate([base.ids, ids]), SCENE_ID)
    table = _table_supports(base, z_floor)
    output.parent.mkdir(parents=True, exist_ok=True)
    archive_meta = {"scene_id": SCENE_ID, "source": str(source), "source_sha256": before[0]["sha256"],
                    "base_meta": meta, "manual_edit_ids": [int(v) for v in ids], "manifest": str(manifest)}
    np.savez(output, means=edited.means, covs=edited.covs, opacity=edited.opacity, ids=edited.ids,
             meta=np.array(json.dumps(archive_meta, sort_keys=True)))
    after = _source_paths(source)
    if before != after:
        raise RuntimeError("read-only source changed during derivative build; refusing manifest")
    origin, R = route_frame(z_floor)
    feasibility = _candidate_enclosure(edited, z_floor)
    doc = {"schema_version": "gs3d.scene.v1", "scene_id": SCENE_ID, "units": "metres, metres^2 covariance; world xyz frame with +z up",
           "source_inputs": before, "source_unchanged_after_build": True,
           "derivative": {"path": str(output), "bytes": output.stat().st_size, "sha256": sha256(output),
                          "n_base_gaussians": int(len(base)), "n_output_gaussians": int(len(edited)),
                          "edit_code_sha256": sha256(Path(__file__))},
           "route_frame": {"origin_world_m": origin.tolist(), "world_to_route": R.tolist(),
                           "route_to_world_row_formula": "world = route @ world_to_route + origin"},
           "configuration": {"tau": TAU, "level": LEVEL, "body_clearance_margin_m": UAV_MARGIN_M,
                             "candidate_map_domain_route_m": {"lower": [-.75, -1., .20], "upper": [3., 1., 2.]},
                             "coverage_policy": "assumed_map_domain; not sensor-observed free space",
                             "candidate_path_world_m": candidate_knots_world(z_floor).tolist()},
           "manual_geometry": entries, "existing_table_arrangement": table, "feasibility_witness": feasibility,
           "limitations": ["Original table geometry is retained and labelled, not manually modified.",
                           "Candidate corridor is a geometry witness, not a planner success or flight execution claim.",
                           "A1/A5 must validate with the shared 3-D ellipsoid/body oracle and coverage policy."],}
    manifest.parent.mkdir(parents=True, exist_ok=True)
    manifest.write_text(json.dumps(doc, indent=2) + "\n")
    return doc


def load_showcase_derivative(path: Path, manifest: Path | None = None) -> tuple[GaussianScene3D, dict[str, Any]]:
    """Load the exact planning/render archive and reject a mismatched manifest."""
    path = Path(path)
    with np.load(path, allow_pickle=False) as data:
        scene = GaussianScene3D(data["means"], data["covs"], data["opacity"], data["ids"], SCENE_ID)
        archive_meta = json.loads(str(data["meta"]))
    if archive_meta.get("scene_id") != SCENE_ID:
        raise ValueError("not the frozen GS3D airborne showcase derivative")
    if manifest is not None:
        doc = json.loads(Path(manifest).read_text())
        if doc["derivative"]["sha256"] != sha256(path):
            raise ValueError("derivative hash differs from its manifest")
        edits = doc["manual_geometry"]
        for row in edits:
            k = int(np.flatnonzero(scene.ids == row["gaussian_id"])[0])
            if not np.array_equal(scene.means[k], np.asarray(row["mean_world_m"], dtype=float)) or not np.array_equal(scene.covs[k], np.asarray(row["covariance_world_m2"], dtype=float)):
                raise ValueError("manual Gaussian rows differ from the frozen manifest")
        return scene, doc
    return scene, archive_meta
