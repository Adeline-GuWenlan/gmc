"""G1 single-orientation-gate scene family (problem_spec.md §4-§6).

Two rooms separated by a corridor-style door of width w in a wall of
thickness T = 1.2, composed of disc primitives:
  - corridor edge rows   r=0.10 spacing 0.02  (envelope ripple <= 5e-5 m)
  - wall face columns    r=0.10 spacing 0.04  (ripple <= 2e-3 m)
  - interior fill        r=0.15 grid 0.20 < r*sqrt(2)  (void-free)
Jamb corners are rounded with radius 0.10 (recorded; mid-corridor gate truth
is unaffected).
"""
from __future__ import annotations

import numpy as np

from ..gaussian_geometry.primitives import SceneGeometry, Robot

WORKSPACE = (-3.5, 3.5, -2.3, 2.3)
WALL_T = 1.2
R_EDGE = 0.10
R_FILL = 0.15

Q_START = (-2.0, 0.0, np.pi / 2)
GOAL = (2.0, 0.0)
GOAL_RADIUS = 0.30
ELL_R = 0.5


def robot_library():
    return {
        "R_small_circle": Robot("R_small_circle", 0.25, 0.25),
        "R_long_ellipse": Robot("R_long_ellipse", 0.60, 0.25),
        "R_big_circle": Robot("R_big_circle", 0.40, 0.40),
    }


def blind_robot_library():
    """Morphology holdout: referenced ONLY by the sealed blind manifest."""
    return {
        "R_blind_ellipse": Robot("R_blind_ellipse", 0.50, 0.30),
    }


def make_g1_scene(w, wall_t=WALL_T, workspace=WORKSPACE,
                  door_offset=0.0, door_tilt=0.0, door_plug=False):
    """Build the G1 scene for door width w.

    De-alignment parameters (03 spec, Sprint A finding): the whole wall+door
    is rigidly rotated by door_tilt about the door center and shifted so the
    door center sits at (0, door_offset).  Analytic gate labels carry over by
    rotational equivariance: passable set is |theta - door_tilt| (mod pi)
    < gate_half_angle(a, b, w); grid-phase alignment with y=0 / theta=0 is
    broken, which is required for thin-gate claims to be measurable.
    """
    xmin, xmax, ymin, ymax = workspace
    half_t = wall_t / 2.0
    # seal walls past the workspace edge; extend further under de-alignment
    y_top_ext = ymax + 0.05 + abs(door_offset) \
        + (xmax - xmin) * 0.55 * abs(np.sin(door_tilt))

    # exact-symmetric construction: all x coordinates are k*step with integer k,
    # so x -> -x and y -> -y mirrors are floating-point-exact (validation
    # relies on this; arange accumulation would break it at the ulp level)
    edge, fill = [], []
    edge_tag, fill_tag = [], []  # +1 upper wall segment, -1 lower (door frame)
    n_edge = int(round((half_t - R_EDGE) / 0.02))
    n_fill = int(round((half_t - R_EDGE - 0.1) / 0.2))
    x_face = half_t - R_EDGE
    for sign in (+1.0, -1.0):
        y_edge = sign * (w / 2.0 + R_EDGE)
        # corridor edge row (flat door jamb, fine spacing)
        for k in range(-n_edge, n_edge + 1):
            edge.append((k * 0.02, y_edge))
            edge_tag.append(int(sign))
        # wall face columns
        for xf in (-x_face, x_face):
            for y in np.arange(w / 2.0 + R_EDGE + 0.04, y_top_ext + 1e-9, 0.04):
                edge.append((xf, sign * y))
                edge_tag.append(int(sign))
        # interior fill
        for k in range(-n_fill, n_fill + 1):
            for y in np.arange(w / 2.0 + 0.25, y_top_ext + 0.2, 0.2):
                fill.append((k * 0.2, sign * y))
                fill_tag.append(int(sign))

    if door_plug:
        # G5 remote closure: plug the corridor near its FAR (goal-side) exit,
        # in door frame at x=+0.45 — outside the broad-phase shell of every
        # start-room pose, so start-local contact values are bitwise identical
        # between the open/closed pair while global reachability flips.
        n_plug = int(round((w / 2.0 + 0.05) / 0.02))
        for k in range(-n_plug, n_plug + 1):
            edge.append((0.45, k * 0.02))
            edge_tag.append(0)

    edge = np.array(edge)
    fill = np.array(fill)
    if door_tilt != 0.0 or door_offset != 0.0:
        ct, st = np.cos(door_tilt), np.sin(door_tilt)
        rot_m = np.array([[ct, -st], [st, ct]])
        edge = edge @ rot_m.T
        fill = fill @ rot_m.T
        edge[:, 1] += door_offset
        fill[:, 1] += door_offset

    groups = [(R_EDGE, edge), (R_FILL, fill)]
    meta = {"family": "G1", "door_width": w, "wall_thickness": wall_t,
            "workspace": list(workspace), "door_style": "corridor",
            "jamb_corner_radius": R_EDGE,
            "door_offset": door_offset, "door_tilt": door_tilt,
            "door_plug": door_plug,
            "n_primitives": len(edge) + len(fill),
            # active-pair identity: wall-side tag per disc, parallel to groups
            "group_tags": [np.array(edge_tag), np.array(fill_tag)]}
    sid = f"G1_w{w:.3f}"
    if door_offset or door_tilt:
        sid += f"_dy{door_offset:+.3f}_tilt{np.degrees(door_tilt):+.1f}"
    if door_plug:
        sid += "_plugged"
    return SceneGeometry(sid, workspace, groups, meta)


# -- analytic gate truth (corridor door, problem_spec §5) --------------------


def projection_radius(a, b, theta):
    """r(theta): projection half-width onto the door-width (y) direction;
    theta measured between the robot major axis and the door normal (+x)."""
    return np.sqrt((a * np.sin(theta)) ** 2 + (b * np.cos(theta)) ** 2)


def gate_half_angle(a, b, w):
    """Corridor-door passable set is |theta mod pi| < gate_half_angle.
    Returns 0.0 if the gate is physically closed (w <= 2b), pi/2 if fully
    open (w >= 2a)."""
    if w <= 2.0 * b:
        return 0.0
    if w >= 2.0 * a:
        return np.pi / 2.0
    s2 = (w * w / 4.0 - b * b) / (a * a - b * b)
    return float(np.arcsin(np.sqrt(s2)))


def gate_half_angle_thin_wall(a, b, w):
    """Thin-wall variant (chord condition) -- NOT the benchmark default.
    2ab/sqrt(a^2 cos^2 t + b^2 sin^2 t) < w."""
    if w <= 2.0 * b:
        return 0.0
    if w >= 2.0 * a:
        return np.pi / 2.0
    # a^2 cos^2 + b^2 sin^2 > 4a^2b^2/w^2  =>  cos^2 > (4a^2b^2/w^2 - b^2)/(a^2-b^2)
    c2 = (4.0 * a * a * b * b / (w * w) - b * b) / (a * a - b * b)
    if c2 <= 0.0:
        return np.pi / 2.0
    return float(np.arccos(np.sqrt(np.clip(c2, 0.0, 1.0))))
