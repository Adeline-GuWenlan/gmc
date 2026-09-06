"""Analytic synthetic scene generators (Guide §16.2 commit 3, §15.1 families).

All walls are built from small isotropic Gaussian supports so that solid
envelopes are exact discs; door edges are placed so the effective opening
equals the nominal width. Every generator is deterministic.

Analytic ground truth for the corridor single door (thick wall):
the robot ellipse (semi-axes a >= b) passes iff its projection half-width on
the door line r(t) = sqrt(a^2 sin^2 t + b^2 cos^2 t) < w/2, where t is the
angle between the robot major axis and the door normal. Legal set:
|theta mod pi| < half_angle(a, b, w).
"""
import numpy as np
from shapely.geometry import Polygon, box as shapely_box

from .types import GaussianSupport2D, SceneModel2D

DISC_R = 0.08
SPACING = 0.10
SCENE_LEVEL = 2.0


def _disc(mean, r, pid, level=SCENE_LEVEL):
    return GaussianSupport2D(mean=np.asarray(mean, float),
                             covariance=np.eye(2) * (r / level) ** 2,
                             level=level, primitive_id=pid)


def _fill_rect(x0, x1, y0, y1, start_id, r=DISC_R, spacing=SPACING):
    nx = max(1, int(round((x1 - x0) / spacing)) + 1)
    ny = max(1, int(round((y1 - y0) / spacing)) + 1)
    xs = np.linspace(x0, x1, nx)
    ys = np.linspace(y0, y1, ny)
    pts = [(x, y) for x in xs for y in ys]
    return [_disc(p, r, start_id + k) for k, p in enumerate(pts)]


def projection_radius(a: float, b: float, t: float) -> float:
    return float(np.sqrt((a * np.sin(t)) ** 2 + (b * np.cos(t)) ** 2))


def gate_half_angle(a: float, b: float, w: float) -> float:
    """Corridor-door legal set is |theta mod pi| < half_angle (0 closed,
    pi/2 fully open)."""
    if w <= 2.0 * b:
        return 0.0
    if w >= 2.0 * a:
        return np.pi / 2.0
    s2 = (w * w / 4.0 - b * b) / (a * a - b * b)
    return float(np.arcsin(np.sqrt(s2)))


def single_door(width: float, wall_t: float = 1.2,
                workspace=(-3.0, 3.0, -2.0, 2.0)) -> SceneModel2D:
    """Vertical corridor wall at x=0 with a door of the given width at y=0.

    Disc centers sit at |y| >= width/2 + DISC_R so the solid opening is
    exactly `width`; the wall seals past the workspace edge."""
    xmin, xmax, ymin, ymax = workspace
    half_t = wall_t / 2.0
    supports = []
    y_edge = width / 2.0 + DISC_R
    for ysign in (+1, -1):
        y0, y1 = y_edge, (ymax if ysign > 0 else -ymin) + 0.5
        band = _fill_rect(-half_t + DISC_R, half_t - DISC_R,
                          y0, y1, start_id=len(supports))
        supports += [_disc((s.mean[0], ysign * s.mean[1]), DISC_R,
                           len(supports) + k)
                     for k, s in enumerate(band)]
    supports = [GaussianSupport2D(s.mean, s.covariance, s.level, i)
                for i, s in enumerate(supports)]
    ws = shapely_box(xmin, ymin, xmax, ymax)
    return SceneModel2D(supports=tuple(supports), workspace=ws,
                        name=f"single_door_w{width:g}")


def keyhole(slot_width: float = 0.7, half: float = 1.0,
            workspace=(-3.0, 3.0, -2.0, 2.0)) -> SceneModel2D:
    """Closed chamber with one slot in the right wall."""
    xmin, xmax, ymin, ymax = workspace
    supports = []

    def wall_line(p0, p1):
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        n = max(2, int(np.ceil(np.linalg.norm(p1 - p0) / SPACING)) + 1)
        for t in np.linspace(0, 1, n):
            supports.append(_disc(p0 + t * (p1 - p0), DISC_R, len(supports)))

    e = slot_width / 2.0 + DISC_R
    wall_line((-half, -half), (-half, half))
    wall_line((-half, half), (half, half))
    wall_line((-half, -half), (half, -half))
    wall_line((half, -half), (half, -e))
    wall_line((half, e), (half, half))
    ws = shapely_box(xmin, ymin, xmax, ymax)
    return SceneModel2D(supports=tuple(supports), workspace=ws,
                        name=f"keyhole_w{slot_width:g}")


def dual_route(w1: float = 0.9, w2: float = 0.6,
               workspace=(-3.0, 3.0, -2.0, 2.0)) -> SceneModel2D:
    """Vertical wall with two doors at y = +-1.0 (two route classes)."""
    xmin, xmax, ymin, ymax = workspace
    supports = []

    def seg(y0, y1):
        for y in np.arange(y0, y1 + 1e-9, SPACING):
            supports.append(_disc((0.0, y), DISC_R, len(supports)))

    seg(ymin - 0.5, -1.0 - w2 / 2.0 - DISC_R)
    seg(-1.0 + w2 / 2.0 + DISC_R, 1.0 - w1 / 2.0 - DISC_R)
    seg(1.0 + w1 / 2.0 + DISC_R, ymax + 0.5)
    ws = shapely_box(xmin, ymin, xmax, ymax)
    return SceneModel2D(supports=tuple(supports), workspace=ws,
                        name=f"dual_route_{w1:g}_{w2:g}")


def boundary_pinch(gap: float = 0.5, wall_top: float = 1.2,
                   x_span=(-3.0, 3.0), ymin: float = -2.0) -> SceneModel2D:
    """T-Slice-04 construction: the SCENE is identical for every gap — a
    single wall stub from below the workspace up to wall_top — and only the
    workspace top boundary moves (ymax = wall_top + gap). The pair graph is
    literally unchanged; free-space connectivity changes only through the
    workspace boundary (§14.4)."""
    supports = []
    for y in np.arange(ymin - 0.5, wall_top + 1e-9, SPACING):
        supports.append(_disc((0.0, y), DISC_R, len(supports)))
    ws = shapely_box(x_span[0], ymin, x_span[1], wall_top + gap)
    return SceneModel2D(supports=tuple(supports), workspace=ws,
                        name=f"boundary_pinch_{gap:g}")


def triple_contact(sep: float = 1.05, r: float = 0.5,
                   workspace=(-3.0, 3.0, -2.5, 2.5)) -> SceneModel2D:
    """Three large discs in a triangle; sep controls proximity to a triple
    common intersection of the C-obstacles (near-degenerate events)."""
    centers = [(0.0, sep), (-sep * np.sqrt(3) / 2, -sep / 2),
               (sep * np.sqrt(3) / 2, -sep / 2)]
    supports = tuple(_disc(c, r, i) for i, c in enumerate(centers))
    xmin, xmax, ymin, ymax = workspace
    return SceneModel2D(supports=supports,
                        workspace=shapely_box(xmin, ymin, xmax, ymax),
                        name=f"triple_{sep:g}")


def quad_overlap(r: float = 0.8, workspace=(-3.0, 3.0, -2.0, 2.0)) -> SceneModel2D:
    """Four discs with a common intersection (T-Slice-03 / T-Nerve-02)."""
    centers = [(-0.3, 0.0), (0.3, 0.0), (0.0, -0.3), (0.0, 0.3)]
    supports = tuple(_disc(c, r, i) for i, c in enumerate(centers))
    xmin, xmax, ymin, ymax = workspace
    return SceneModel2D(supports=supports,
                        workspace=shapely_box(xmin, ymin, xmax, ymax),
                        name="quad_overlap")


def empty_scene(workspace=(-3.0, 3.0, -2.0, 2.0)) -> SceneModel2D:
    xmin, xmax, ymin, ymax = workspace
    return SceneModel2D(supports=(),
                        workspace=shapely_box(xmin, ymin, xmax, ymax),
                        name="empty")


def single_obstacle(r: float = 0.6,
                    workspace=(-3.0, 3.0, -2.0, 2.0)) -> SceneModel2D:
    xmin, xmax, ymin, ymax = workspace
    return SceneModel2D(supports=(_disc((0.0, 0.0), r, 0),),
                        workspace=shapely_box(xmin, ymin, xmax, ymax),
                        name=f"single_obstacle_{r:g}")


FAMILIES = {
    "single-door": single_door,
    "keyhole": keyhole,
    "dual-route": dual_route,
    "boundary-pinch": boundary_pinch,
    "triple-contact": triple_contact,
    "quad-overlap": quad_overlap,
}
