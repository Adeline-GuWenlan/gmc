"""Synthetic Gaussian scenes for the aerial3d acceptance tests (uav.md §17/§18, U0 rows).

Walls, slabs and lamp boxes are hole-free panel slabs built with the same
generator as the uav-lamp scene (``gs3d.scene_uavlamp.edit_gaussians``): flat
Gaussians on a grid whose in-plane semiaxis equals the grid step.  All scenes are
full 3-D Gaussians (mean + full covariance + id) with an explicit known box.
Each builder returns ``(scene, queries)`` with queries = {name: (start, goal)}.
"""
from __future__ import annotations

import numpy as np

from gmc.gs3d import scene_uavlamp as su
from gmc.gs3d.contracts import BodySpec, SceneSpec
from gmc.height.ply3d import GaussianScene3D
from gs3d_core_fixtures import KnownBox, under_over

UAV = BodySpec("uav", .25, .10, "uav_translation")
SMALL_UAV = BodySpec("uav_small", .15, .06, "uav_translation")
BIG_UAV = BodySpec("uav_big", .30, .12, "uav_translation")
_IDENTITY = su.Frame(np.zeros(3), np.eye(3))
SPACING, HALF_T = .10, .03


def panel(center, plane, size, *, spacing=SPACING, half_thickness=HALF_T):
    """Rectangle of flat Gaussians; ``plane`` in {'xy','xz','yz'} (route axes u,v,z = x,y,z)."""
    edit = {"kind": "panel", "center_route": list(map(float, center)),
            "plane": plane.replace("x", "u").replace("y", "v"), "size_m": list(map(float, size)),
            "spacing_m": spacing, "half_thickness_m": half_thickness}
    return su.edit_gaussians(edit, _IDENTITY)


def box_panel(lower, upper, plane, **kw):
    """Panel covering the axis-aligned rectangle lower..upper within ``plane``."""
    lo, hi = np.asarray(lower, float), np.asarray(upper, float)
    return panel((lo + hi) / 2, plane, _plane_size(lo, hi, plane), **kw)


def _plane_size(lo, hi, plane):
    axes = {"x": 0, "y": 1, "z": 2}
    return [hi[axes[plane[0]]] - lo[axes[plane[0]]], hi[axes[plane[1]]] - lo[axes[plane[1]]]]


def lamp(center_xy, size_xy, underside_z, height=.15, **kw):
    out = []
    for e in su.lamp_box_edits("lamp", center_uv=center_xy, size_uv=size_xy, underside_z=underside_z,
                               height=height, spacing=kw.get("spacing", SPACING),
                               half_thickness=kw.get("half_thickness", HALF_T)):
        out.append(su.edit_gaussians(e, _IDENTITY))
    return out


def make(parts, lower, upper, name) -> SceneSpec:
    parts = [p for p in parts if len(p[0])]
    means = np.vstack([m for m, _ in parts]) if parts else np.empty((0, 3))
    covs = np.concatenate([c for _, c in parts]) if parts else np.empty((0, 3, 3))
    g = GaussianScene3D(means, covs, np.full(len(means), .95), np.arange(len(means)), name)
    return SceneSpec(name, g, tuple(map(float, lower)), tuple(map(float, upper)), .3, 2.,
                     KnownBox(tuple(map(float, lower)), tuple(map(float, upper))), None,
                     {"coverage_policy": "synthetic_declared_known_box", "seed": 0})


# ----------------------------------------------------------------------------- cases

def open_ascent():
    lo, hi = (-1., -1., 0.), (1., 1., 3.)
    return make([], lo, hi, "open_ascent"), {"ascent": ((0., 0., .3), (0., 0., 2.6))}


def low_wall():
    lo, hi = (-2., -1., 0.), (2., 1., 2.5)
    wall = box_panel((0, -1.1, 0.), (0, 1.1, 1.0), "yz")
    return make([wall], lo, hi, "low_wall"), {"over": ((-1.5, 0., .5), (1.5, 0., .5))}


def full_wall(gap: bool = False):
    lo, hi = (-2., -1., 0.), (2., 1., 2.5)
    y_hi = .1 if gap else 1.1  # with gap: open corridor y in (~.2, 1) at one end
    wall = box_panel((0, -1.1, -.05), (0, y_hi, 2.6), "yz")
    name = "full_wall_gap" if gap else "full_wall_closed"
    return make([wall], lo, hi, name), {"across": ((-1.5, -.4, 1.), (1.5, -.4, 1.))}


def overhang():
    lo, hi = (-2., -1., 0.), (2., 1., 3.)
    slab = box_panel((-2.1, -1.1, 1.5), (.5, 1.1, 1.5), "xy")
    return make([slab], lo, hi, "overhang"), {"up_past_ceiling": ((-1.5, 0., .6), (-1.5, 0., 2.5))}


def bridge():
    """Deck through a full wall: two openings (under and over the deck) at the same xy."""
    lo, hi = (-2., -1., 0.), (2., 1., 3.)
    deck = box_panel((-.6, -1.1, 1.45), (.6, 1.1, 1.45), "xy")
    # wall at x=0 everywhere except directly under and over the deck (z in [.05,1.3] and [1.6,2.9])
    # opening |y| < .5 (panels reach one spacing past their nominal edge): 0.4 m wide in C-space
    w_lo = box_panel((0, -1.1, -.05), (0, -.6, 3.1), "yz")
    w_hi = box_panel((0, .6, -.05), (0, 1.1, 3.1), "yz")
    queries = {"under": ((-1.5, 0., .6), (1.5, 0., .6)), "over": ((-1.5, 0., 2.3), (1.5, 0., 2.3))}
    return make([deck, w_lo, w_hi], lo, hi, "bridge"), queries


def shaft():
    lo, hi = (-1.5, -1.5, 0.), (1.5, 1.5, 3.8)
    parts = [box_panel((-1.6, -1.6, 2.), (1.6, .45, 2.), "xy"),   # floor-2 with a corner hole
             box_panel((-1.6, .45, 2.), (.45, 1.6, 2.), "xy"),
             box_panel((.45, .45, 2.), (.45, 1.6, 3.), "yz"),      # shaft walls above the hole
             box_panel((.45, .45, 2.), (1.6, .45, 3.), "xz")]
    return make(parts, lo, hi, "shaft"), {"up_the_shaft": ((-1., -1., .5), (-1., -1., 3.3))}


def window_wall():
    """Narrow window (small UAV only) on the direct line; large opening off to the side."""
    lo, hi = (-2., -1.6, 0.), (2., 1.6, 2.5)
    x = 0.
    # Panels reach one spacing (.1) past their nominal edges.  Window A (actual y +-.3,
    # z .8..1.3) passes r=.15 (needs .4 x .22) but not r=.30 (needs .7).  Window B (actual
    # y .75..box face, z .6..1.5) passes both.
    parts = [box_panel((x, -1.7, -.05), (x, -.4, 2.6), "yz"),     # left of window A
             box_panel((x, -.4, -.05), (x, .4, .7), "yz"),        # below window A
             box_panel((x, -.4, 1.4), (x, .4, 2.6), "yz"),        # above window A
             box_panel((x, .4, -.05), (x, .65, 2.6), "yz"),       # between A and B
             box_panel((x, .65, -.05), (x, 1.7, .5), "yz"),       # below window B
             box_panel((x, .65, 1.6), (x, 1.7, 2.6), "yz")]       # above window B
    return make(parts, lo, hi, "window_wall"), {"through": ((-1.5, 0., 1.05), (1.5, 0., 1.05))}


def under_over_f1():
    scene, start, goal = under_over()
    return scene, {"f1": (tuple(start.xyz), tuple(goal.xyz))}


def mini_booth(plug: bool = False):
    """uav-lamp in miniature: corridor walls, lamp + header + soffit + back panel; the only
    passage from the start region into the booth is under the lamp.  ``plug`` fills it."""
    lo, hi = (-2.5, -1.1, 0.), (2.6, 1.1, 2.0)
    parts = [box_panel((-2.6, -1.0, -.05), (2.7, -1.0, 2.1), "xz"),     # wall A
             box_panel((-2.6, 1.0, -.05), (2.7, 1.0, 2.1), "xz"),       # wall B
             box_panel((0., -1.05, 1.05), (0., 1.05, 1.95), "yz"),      # header over the lamp
             box_panel((-.1, -1.05, 1.9), (2.5, 1.05, 1.9), "xy"),      # soffit (booth ceiling)
             box_panel((2.4, -1.05, -.05), (2.4, 1.05, 1.95), "yz")]    # back panel
    parts += lamp((0., 0.), (.3, 2.1), .9)
    if plug:
        parts.append(box_panel((0., -1.05, -.05), (0., 1.05, .9), "yz"))
    name = "mini_booth_plug" if plug else "mini_booth"
    queries = {"low_start": ((-1.8, 0., .45), (1.3, .3, 1.35)),
               "high_start": ((-1.8, 0., 1.5), (1.3, .3, 1.35))}
    return make(parts, lo, hi, name), queries
