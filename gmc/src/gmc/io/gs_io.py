"""Scene I/O (Guide §4, M0): YAML serialization with exact round-trip.

v0 does not run on raw 3D splats; scenes are 2D Gaussian iso-ellipsoid
supports with frozen levels. Opacity never decides solidity (§4.1).
"""
from pathlib import Path

import numpy as np
import yaml
from shapely.geometry import Polygon

from ..types import GaussianSupport2D, SceneModel2D


def save_scene(scene: SceneModel2D, path: str | Path) -> None:
    data = {
        "name": scene.name,
        "workspace": {
            "exterior": [list(map(float, c)) for c in scene.workspace.exterior.coords],
            "holes": [[list(map(float, c)) for c in ring.coords]
                      for ring in scene.workspace.interiors],
        },
        "supports": [
            {"primitive_id": s.primitive_id,
             "mean": [float(s.mean[0]), float(s.mean[1])],
             "covariance": [[float(s.covariance[0, 0]), float(s.covariance[0, 1])],
                            [float(s.covariance[1, 0]), float(s.covariance[1, 1])]],
             "level": float(s.level)}
            for s in scene.supports
        ],
    }
    Path(path).write_text(yaml.safe_dump(data, sort_keys=False))


def load_scene(path: str | Path) -> SceneModel2D:
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict):
        raise ValueError("scene root must be a mapping")
    ws = Polygon(data["workspace"]["exterior"],
                 holes=data["workspace"].get("holes", []))
    supports = tuple(
        GaussianSupport2D(mean=np.array(s["mean"]),
                          covariance=np.array(s["covariance"]),
                          level=s["level"],
                          primitive_id=s["primitive_id"])
        for s in data["supports"])
    return SceneModel2D(supports=supports, workspace=ws, name=data["name"])


def spd_check(supports, min_eig: float):
    """M0 acceptance: every covariance strictly SPD above min_eig (§4.2)."""
    min_eig = float(min_eig)
    if not np.isfinite(min_eig) or min_eig < 0.0:
        raise ValueError("min_eig must be finite and non-negative")
    bad = []
    for s in supports:
        covariance = np.asarray(s.covariance, dtype=float)
        if (covariance.shape != (2, 2)
                or not np.all(np.isfinite(covariance))
                or not np.allclose(covariance, covariance.T,
                                   rtol=1e-10, atol=1e-12)):
            bad.append(s.primitive_id)
            continue
        eig = np.linalg.eigvalsh(0.5 * (covariance + covariance.T))
        if eig[0] <= min_eig:
            bad.append(s.primitive_id)
    return bad


def validate_models(scene, robot, cfg) -> None:
    """Enforce the configured M0 covariance floor for both input models.

    Shape, finiteness, strict SPD, positive frozen levels, workspace validity,
    and robot non-emptiness are schema invariants.  This entrance check adds
    the run-specific ``lambda_min`` threshold from the configuration.
    """
    min_eig = cfg.geometry.min_cov_eigenvalue
    scene_bad = spd_check(scene.supports, min_eig)
    robot_bad = spd_check(robot.supports, min_eig)
    if scene_bad or robot_bad:
        details = []
        if scene_bad:
            details.append(f"scene primitive IDs {scene_bad}")
        if robot_bad:
            details.append(f"robot primitive IDs {robot_bad}")
        raise ValueError(
            "covariance eigenvalue must exceed configured "
            f"min_cov_eigenvalue={min_eig}: " + "; ".join(details)
        )

    # GEOS and the serialized model use binary64 world coordinates.  If one
    # world-coordinate ULP already exceeds a configured geometric tolerance,
    # neither precision snapping nor a clearance/approximation certificate at
    # that tolerance is representable.  Continuing would allow an obstacle to
    # collapse during buffer/set operations and turn numerical uncertainty
    # into a false-safe result, so reject the model at the M0 entrance.
    workspace_coords = [np.asarray(scene.workspace.exterior.coords, float)]
    workspace_coords.extend(
        np.asarray(ring.coords, float) for ring in scene.workspace.interiors
    )
    workspace_abs = max(
        (float(np.max(np.abs(coords))) for coords in workspace_coords
         if coords.size),
        default=0.0,
    )
    robot_extent = float(robot.max_rotational_radius())
    scene_abs = max(
        (float(np.max(np.abs(s.mean))) + s.bounding_radius() + robot_extent
         for s in scene.supports),
        default=0.0,
    )
    max_world_coordinate = max(workspace_abs, scene_abs)
    coordinate_ulp = float(abs(np.spacing(max_world_coordinate)))
    if not np.isfinite(max_world_coordinate) or not np.isfinite(coordinate_ulp):
        raise ValueError("world-coordinate extent/resolution must be finite")
    tolerances = {
        "workspace_precision": float(cfg.geometry.workspace_precision),
        "eps_pair": float(cfg.pair_approx.eps_pair),
        "eps_clear": float(cfg.query.eps_clear),
    }
    unresolved = {
        name: value for name, value in tolerances.items()
        # A zero clearance floor means strict collision-freedom, not a demand
        # for zero-ULP world coordinates.  It is handled by the pair-local
        # separation kernel; positive configured resolution targets must be
        # representable by the world-coordinate backend.
        if value > 0.0 and coordinate_ulp > value
    }
    if unresolved:
        details = ", ".join(f"{name}={value}" for name, value in unresolved.items())
        raise ValueError(
            "binary64 world-coordinate resolution is coarser than configured "
            f"geometry tolerance(s): ulp={coordinate_ulp} at "
            f"|coordinate|={max_world_coordinate}; {details}"
        )
