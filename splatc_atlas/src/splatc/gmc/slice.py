"""Fixed-theta slice in HARD-SUPPORT (Minkowski) semantics on the frozen
G1 bench primitives (disc scenes + ellipse robots).

Forbidden primitive per scene disc = disc (+) robot support ellipse at theta,
computed as an exact convex polygon Minkowski sum. Discs in a SceneGeometry
group share one radius, so each (group, theta, side) needs ONE prototype
Minkowski polygon, translated to every center.

Two-sided certification (master plan v3 §5.1):
  side="outer": circumscribed polygonizations -> forbidden OVER-approximated
     -> free under-approximated -> connectivity/OPEN verdicts are certified.
  side="inner": inscribed -> forbidden UNDER-approximated -> free
     over-approximated -> DISCONNECTED/CLOSED verdicts are certified.
  open(outer) subset true-open subset open(inner); the sandwich width is the
  polygonization gap, shrinking as ndir grows.

Witnesses: every free component carries a representative pose, certified by
the frozen independent checker (SceneGeometry.eval_points), never by this
polygon pipeline.
"""
from dataclasses import dataclass, field

import numpy as np
import shapely
from shapely.geometry import box as shapely_box

from ..gaussian_geometry.primitives import support_half_widths
from .geometry import circum_factor, ellipse_polys, minkowski_sum, rot2

_AREA_TOL = 1e-9
_EDGE_EPS = 1e-9


def robot_support_poly(robot, theta, ndir=48, factor=1.0):
    """CCW polygon of the robot support ellipse (semi-axes a, b) at theta,
    centered at the origin.

    Built as a RIGID ROTATION of the theta=0 polygon (not a re-polygonization
    of the rotated ellipse). Rotation commutes with circum/inscription, so the
    G1 sandwich is unaffected, and the polygon system becomes exactly
    Hausdorff-Lipschitz in theta with rate L = max vertex radius — the basis
    of the G2 perturbation certificates (events.py)."""
    base = ellipse_polys(np.zeros((1, 2)),
                         np.diag([robot.a ** 2, robot.b ** 2])[None],
                         1.0, ndir, factor)[0]
    return base @ rot2(theta).T


def disc_poly(radius, ndir=48, factor=1.0):
    ang = np.linspace(0, 2 * np.pi, ndir, endpoint=False)
    return radius * factor * np.stack([np.cos(ang), np.sin(ang)], axis=1)


@dataclass
class Slice:
    theta: float
    side: str                     # "outer" | "inner"
    workspace: tuple
    free_geoms: list = field(repr=False)   # free components, largest first
    witnesses: np.ndarray = None           # (k, 2) representative points
    forbidden: object = field(default=None, repr=False)

    @property
    def n_components(self):
        return len(self.free_geoms)

    def component_of(self, xy):
        for i, g in enumerate(self.free_geoms):
            if g.covers(shapely.Point(xy)):
                return i
        return -1

    def label_grid(self, X, Y):
        """Component label per grid point (-1 = not in any free component)."""
        lab = np.full(X.shape, -1, dtype=np.int32)
        for i, g in enumerate(self.free_geoms):
            m = shapely.contains_xy(g, X, Y)
            lab[m & (lab == -1)] = i
        return lab


def build_slice(scene, robot, theta, ndir=48, side="outer", area_min=1e-6,
                perturb=0.0):
    """Hard-support forbidden union and free components at fixed theta.

    perturb > 0 SHRINKS free space (discs dilated by perturb, workspace eroded
    by an extra perturb); perturb < 0 grows it (discs eroded, workspace box
    enlarged). Because F_i = disc_i (+) E_robot, disc dilation IS Minkowski
    dilation of F_i (exact), and per-primitive disc erosion is a sound subset
    of F_i erosion. Used by events.py: verdict stable at +eps => stable for
    |dtheta| <= eps / L (Hausdorff rate of the rigidly-rotating system).
    Erosion is capped at the smallest disc radius (beyond that the proxy is
    unsound); callers must keep -perturb < min radius."""
    factor = circum_factor(ndir) if side == "outer" else 1.0
    rp = robot_support_poly(robot, theta, ndir, factor)
    polys = []
    for radius, centers in scene.disc_groups:
        r_eff = radius + perturb
        if r_eff <= 0:
            raise ValueError(
                f"erosion {perturb} exceeds disc radius {radius}; "
                "the per-primitive erosion proxy is unsound past that")
        proto = minkowski_sum(disc_poly(r_eff, ndir, factor)[None], rp)[0]
        polys.append(proto[None, :, :] + np.asarray(centers)[:, None, :])
    verts = np.concatenate(polys, axis=0)
    forbidden = shapely.union_all(shapely.polygons(verts))
    # bench semantics keeps the whole BODY inside the workspace (eval_points
    # erodes by support_half_widths) — a center-only box would open zero-width
    # wrap-around passages along the workspace edge (caught by keyhole
    # theta=90 in g1_slice_validation). Erode accordingly; the outer side adds
    # an epsilon so outer free stays a strict subset of true free.
    px, py = support_half_widths(robot.a, robot.b, theta)
    px, py = px + perturb, py + perturb
    if side == "outer":
        px, py = px + _EDGE_EPS, py + _EDGE_EPS
    xmin, xmax, ymin, ymax = scene.workspace
    ws = shapely_box(xmin + px, ymin + py, xmax - px, ymax - py)
    free = ws.difference(forbidden)
    geoms = [g for g in getattr(free, "geoms", [free])
             if g.geom_type == "Polygon" and g.area > max(area_min, _AREA_TOL)]
    geoms.sort(key=lambda g: -g.area)
    wit = np.array([[g.representative_point().x, g.representative_point().y]
                    for g in geoms]) if geoms else np.empty((0, 2))
    return Slice(theta=float(theta), side=side, workspace=scene.workspace,
                 free_geoms=geoms, witnesses=wit, forbidden=forbidden)


def certify_witnesses(sl: Slice, scene, robot):
    """Independent-checker certificates for every component witness.

    Returns (all_free: bool, margins: np.ndarray). For side="outer" every
    witness MUST verify free — a failure is a pipeline bug, not a boundary
    effect, because outer free is a subset of true free."""
    if len(sl.witnesses) == 0:
        return True, np.empty(0)
    X = sl.witnesses[:, 0][None, :]
    Y = sl.witnesses[:, 1][None, :]
    free, rho = scene.eval_points(robot, sl.theta, X, Y)
    return bool(np.all(free)), np.asarray(rho).ravel()


def probes_connected(sl: Slice, probe_geoms):
    """True iff one free component intersects every probe geometry."""
    return any(all(g.intersects(p) for p in probe_geoms) for g in sl.free_geoms)
