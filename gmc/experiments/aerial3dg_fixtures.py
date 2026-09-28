"""Synthetic ground scenes for the aerial3d ground-body tests (G1).

Same hole-free panel slabs as ``aerial3d_fixtures`` (level-2 flat Gaussians whose
in-plane semiaxis equals the grid step), plus a flat ``FlatSupport`` floor so the
gs3d ground contract (on-manifold body centre, swept support evidence) applies.
"""
from __future__ import annotations

import numpy as np

from gmc.gs3d.contracts import SceneSpec
from gmc.gs3d.robots import CYLINDER, SWEEPER
from gmc.height.ply3d import GaussianScene3D
from aerial3d_fixtures import box_panel
from gs3d_core_fixtures import FlatSupport, KnownBox

__all__ = ["SWEEPER", "CYLINDER", "table_corridor", "ground_z"]


def ground_z(body, floor_z: float = 0.) -> float:
    """Body-centre height on a flat floor (gs3d ``supported_pose`` convention)."""
    return floor_z + body.ground_clearance_m + body.half_height_m


def table_corridor():
    """A 4 m corridor with a 'table' slab at z 0.47-0.53 over its upper part.

    Floor slab top at z = 0.01 (below both chassis bottoms at 0.02).  The table
    (support y >= -0.7 incl. the one-spacing panel overhang) is a roof for the
    sweeper (top 0.10) and a wall for the cylinder (top 1.75), which must pass
    through the free strip y in [-2, -0.7] (centre y <= -1.0).
    Queries ``under``: start/goal at y = 0.5 on either side of the table.
    """
    lower, upper = (-2.5, -2.0, -.10), (2.5, 1.5, 2.2)
    parts = [box_panel((-2.5, -2.0, -.02), (2.5, 1.5, -.02), "xy"),        # floor, top -0.02+0.03
             box_panel((-.5, -.6, .50), (.5, 1.5, .50), "xy")]             # table top
    means = np.vstack([m for m, _ in parts])
    covs = np.concatenate([c for _, c in parts])
    g = GaussianScene3D(means, covs, np.full(len(means), .95), np.arange(len(means)), "table_corridor")
    known = KnownBox(lower, upper)
    scene = SceneSpec("table_corridor", g, lower, upper, .3, 2., known, FlatSupport(0., known),
                      {"coverage_policy": "synthetic_declared_known_box", "seed": 0})
    queries = {}
    for body in (SWEEPER, CYLINDER):
        z = ground_z(body)
        queries[body.name] = {"under": ((-1.8, .5, z), (1.8, .5, z))}
    return scene, queries
