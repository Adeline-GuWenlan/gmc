"""Certified pair envelopes over an orientation interval.

For a fixed pair let ``O(theta)`` be its exact convex C-obstacle and let a
certified midpoint sandwich satisfy::

    inner_mid  subset O(theta_mid) subset outer_mid.

``PairOracle.theta_lipschitz`` supplies an outward-rounded Hausdorff rate
``L``.  Hence, for ``r = max(theta_mid-lo, hi-theta_mid)`` and every theta in
the (unwrapped) interval,

    inner_mid erosion (L r) subset O(theta)
    O(theta) subset outer_mid dilation (L r).

This module materialises those two interval-wide sets.  The outer dilation is
*not* delegated to a round GEOS buffer: it is built as an intersection of
outward-shifted supporting halfplanes.  Binary64 export and precision-grid
rounding are then padded outward and independently checked.  The inner set is
allowed to collapse to empty at any point; that loses information but cannot
create a false collision certificate.
"""

from dataclasses import dataclass
from fractions import Fraction

import numpy as np
import shapely
from shapely.geometry import Polygon
from shapely.geometry.polygon import orient

from ..geometry.envelopes import (ConvexSandwich,
                                  _convex_contains_vertices,
                                  _filtered_orientation_rows)
from ..geometry.predicates import set_precision_inner, set_precision_outer
from ..geometry.support import PairOracle, unit_dirs
from ..types import CertStatus, PairID
from .intervals import TWO_PI, Interval


@dataclass(frozen=True)
class IntervalPairCertificate:
    """One pair's common obstacle bounds over an orientation interval.

    ``outer_cover`` contains ``O(theta)`` for every theta in ``interval``;
    ``inner_common`` is contained in every ``O(theta)``.  Consumers may use
    the fixed-slice aliases ``outer`` and ``inner`` only when ``status`` is
    :class:`~gmc.types.CertStatus.CERTIFIED`.
    """

    pair_id: PairID
    theta: float
    interval: Interval
    outer_cover: Polygon
    inner_common: Polygon
    theta_lipschitz: float
    motion_bound: float
    status: CertStatus
    provenance: dict

    @property
    def outer(self) -> Polygon:
        return self.outer_cover

    @property
    def inner(self) -> Polygon:
        return self.inner_common


def _empty_certificate(sandwich, interval, *, lipschitz=np.nan,
                       motion_bound=np.nan, reason: str, checks=None,
                       grid_size=np.nan) -> IntervalPairCertificate:
    """Return an unmistakably unusable, fail-closed result."""
    pair_id = getattr(sandwich, "pair_id", PairID(-1, -1))
    theta = float(getattr(sandwich, "theta", np.nan))
    return IntervalPairCertificate(
        pair_id=pair_id,
        theta=theta,
        interval=interval,
        outer_cover=Polygon(),
        inner_common=Polygon(),
        theta_lipschitz=float(lipschitz),
        motion_bound=float(motion_bound),
        status=CertStatus.UNKNOWN,
        provenance={
            "construction": "midpoint_hausdorff_interval_pair_cover",
            "reason": reason,
            "grid_size": float(grid_size),
            "checks": {} if checks is None else dict(checks),
        },
    )


def _finite_polygon(poly) -> bool:
    if not isinstance(poly, Polygon) or poly.is_empty or not poly.is_valid:
        return False
    bounds = np.asarray(poly.bounds, dtype=float)
    return bool(
        bounds.shape == (4,)
        and np.all(np.isfinite(bounds))
        and np.isfinite(poly.area)
        and poly.area > 0.0
        and len(poly.interiors) == 0
    )


def _local_vertices(poly: Polygon, center: np.ndarray) -> np.ndarray:
    vertices = np.asarray(poly.exterior.coords[:-1], dtype=np.longdouble)
    if vertices.ndim != 2 or vertices.shape[0] < 3 or vertices.shape[1] != 2:
        raise FloatingPointError("polygon has fewer than three vertices")
    local = vertices - np.asarray(center, dtype=np.longdouble)[None, :]
    if not np.all(np.isfinite(local)):
        raise FloatingPointError("polygon has non-finite local vertices")
    return local


def _direction_schedule(angles: np.ndarray,
                        directions: np.ndarray) -> np.ndarray:
    """Validate, order, and return the proof-critical binary64 rows.

    The rows retained by :class:`ConvexSandwich` are the rows that were used
    for the midpoint support evaluations.  Recomputing cosine/sine from the
    angles here would silently create a second, platform-dependent proof
    schedule, so the angles are only used to validate ordering and binding.
    """
    angles = np.asarray(angles, dtype=float)
    if angles.ndim != 1 or len(angles) < 3 or not np.all(np.isfinite(angles)):
        raise FloatingPointError("invalid midpoint support direction schedule")
    directions = np.asarray(directions, dtype=float)
    if (directions.shape != (len(angles), 2)
            or not np.all(np.isfinite(directions))):
        raise ValueError("stored midpoint support directions are invalid")
    expected_query = unit_dirs(angles)
    # PairOracle normalizes public query rows once more.  The fixed sandwich
    # stores that second-normalized array, so reproduce exactly the same
    # binary64 operation for the binding check and then consume the stored
    # rows below.
    expected_norms = np.linalg.norm(expected_query, axis=1)
    expected = expected_query / expected_norms[:, None]
    if not np.array_equal(directions, expected):
        raise ValueError(
            "stored midpoint support directions do not match their angles")
    norm2 = np.sum(
        np.asarray(directions, dtype=np.longdouble) ** 2, axis=1,
    )
    unit_tol = (np.longdouble(32.0) * np.longdouble(np.finfo(float).eps))
    if np.any(np.abs(norm2 - np.longdouble(1.0)) > unit_tol):
        raise ValueError("stored midpoint support directions are not unit rows")
    wrapped = np.mod(angles, TWO_PI)
    order = np.argsort(wrapped)
    wrapped = wrapped[order]
    gaps = np.diff(np.append(wrapped, wrapped[0] + TWO_PI))
    # Positive gaps make every adjacent line intersection well-defined; a gap
    # below pi makes their intersection bounded.
    if np.any(gaps <= 0.0) or np.any(gaps >= np.pi):
        raise FloatingPointError("support directions do not bound a polygon")
    return directions[order].astype(np.longdouble)


def _support_intersections(normals: np.ndarray,
                           support: np.ndarray) -> np.ndarray:
    """Intersect consecutive supporting lines in extended precision."""
    u1 = np.asarray(normals, dtype=np.longdouble)
    u2 = np.roll(u1, -1, axis=0)
    h1 = np.asarray(support, dtype=np.longdouble)
    h2 = np.roll(h1, -1)
    det = u1[:, 0] * u2[:, 1] - u1[:, 1] * u2[:, 0]
    # The schedule and the exported polygon are binary64 even on platforms
    # where ``longdouble`` has an 80-bit mantissa.  Conditioning must therefore
    # be judged at the precision of the proof inputs, not the accumulator.
    eps = np.longdouble(np.finfo(float).eps)
    if (not np.all(np.isfinite(det))
            or np.any(det <= np.longdouble(64.0) * eps)):
        raise FloatingPointError("ill-conditioned supporting halfplanes")
    vx = (h1 * u2[:, 1] - h2 * u1[:, 1]) / det
    vy = (h2 * u1[:, 0] - h1 * u2[:, 0]) / det
    vertices = np.stack([vx, vy], axis=1)
    if not np.all(np.isfinite(vertices)):
        raise FloatingPointError("non-finite supporting-line intersection")
    return vertices


def _longdouble_error_scale(*arrays) -> np.longdouble:
    """Conservative error allowance for binary64 proof inputs/outputs.

    ``np.longdouble`` is binary64 on Apple Silicon but extended precision on
    x86 Linux.  The certificate is ultimately consumed by binary64 GEOS
    geometry on both platforms, so using the smaller x86 long-double epsilon
    made the same proof status platform dependent.  Accumulation may be more
    accurate, but its error floor may never be below binary64 resolution.
    """
    scale = np.longdouble(1.0)
    for array in arrays:
        values = np.asarray(array, dtype=np.longdouble)
        if values.size:
            scale = max(scale, np.max(np.abs(values)))
    arithmetic_eps = max(
        np.longdouble(np.finfo(np.longdouble).eps),
        np.longdouble(np.finfo(float).eps),
    )
    return np.longdouble(1024.0) * arithmetic_eps * scale


def _exact_orientation(a: np.ndarray, b: np.ndarray,
                       c: np.ndarray) -> int:
    """Exact orient2d sign for three finite binary64 points."""
    ax, ay = (Fraction.from_float(float(value)) for value in a)
    bx, by = (Fraction.from_float(float(value)) for value in b)
    cx, cy = (Fraction.from_float(float(value)) for value in c)
    determinant = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    return int(determinant > 0) - int(determinant < 0)


def _filtered_orientation_signs(a: np.ndarray, b: np.ndarray,
                                points: np.ndarray) -> np.ndarray:
    """Robust orient2d signs with exact dyadic fallback near zero.

    There is no geometric tolerance in this predicate.  The filter merely
    decides when the long-double sign is separated far enough from its
    binary64 input error bound; every ambiguous sign is recomputed exactly.
    """
    a64 = np.asarray(a, dtype=float)
    b64 = np.asarray(b, dtype=float)
    points64 = np.asarray(points, dtype=float)
    ld = np.longdouble
    a_ld = np.asarray(a64, dtype=ld)
    b_ld = np.asarray(b64, dtype=ld)
    p_ld = np.asarray(points64, dtype=ld)
    edge = b_ld - a_ld
    rel = p_ld - a_ld[None, :]
    first = edge[0] * rel[:, 1]
    second = edge[1] * rel[:, 0]
    determinant = first - second

    eps = ld(np.finfo(float).eps)
    edge_error = ld(2.0) * eps * (np.abs(b_ld) + np.abs(a_ld))
    rel_error = ld(2.0) * eps * (
        np.abs(p_ld) + np.abs(a_ld)[None, :]
    )
    propagated = (
        np.abs(edge[0]) * rel_error[:, 1]
        + np.abs(rel[:, 1]) * edge_error[0]
        + edge_error[0] * rel_error[:, 1]
        + np.abs(edge[1]) * rel_error[:, 0]
        + np.abs(rel[:, 0]) * edge_error[1]
        + edge_error[1] * rel_error[:, 0]
        + eps * (np.abs(first) + np.abs(second))
    )
    error = ld(32.0) * propagated
    signs = np.zeros(len(points64), dtype=np.int8)
    signs[determinant > error] = 1
    signs[determinant < -error] = -1
    for index in np.flatnonzero(signs == 0):
        signs[index] = _exact_orientation(a64, b64, points64[index])
    return signs


def _convex_polygon_covers_vertices(outer: Polygon,
                                    inner: Polygon) -> bool:
    """Exact containment of one stored polygon in a convex stored polygon.

    This verifies the actual binary64 vertices rather than asking GEOS to
    classify points lying one ULP from a shared edge.  A true one-ULP
    protrusion remains a failure; only an exactly collinear boundary passes.
    """
    if inner.is_empty:
        return True
    if not _finite_polygon(outer) or not _finite_polygon(inner):
        return False
    outer_vertices = np.asarray(
        orient(outer, sign=1.0).exterior.coords[:-1], dtype=float,
    )
    inner_vertices = np.asarray(inner.exterior.coords[:-1], dtype=float)
    if len(outer_vertices) < 3 or len(inner_vertices) < 3:
        return False

    # First prove that the half-plane/fan test is applicable to the stored
    # outer.  A negative turn is a malformed non-convex proof object and must
    # fail closed.  The shared exact convex helper uses its O(n + m log n) fan
    # path for strict convexity and retains its all-edges fallback for
    # non-strict/collinear convex inputs.
    previous = np.roll(outer_vertices, 1, axis=0)
    following = np.roll(outer_vertices, -1, axis=0)
    turns = _filtered_orientation_rows(
        previous, outer_vertices, following,
    )
    if np.any(turns < 0):
        return False
    return _convex_contains_vertices(outer_vertices, inner_vertices)


def _build_outer_halfplane_cover(outer_mid: Polygon, angles: np.ndarray,
                                 directions: np.ndarray,
                                 oracle: PairOracle, theta: float,
                                 delta: float, grid_size: float) -> Polygon:
    """Build and audit a polygon containing ``outer_mid (+) delta B``."""
    outer_mid = orient(outer_mid.convex_hull, sign=1.0)
    center = oracle.center_longdouble(theta)
    source = _local_vertices(outer_mid, center)
    normals = _direction_schedule(angles, directions)

    # Each finite supporting halfplane contains outer_mid.  Shifting every
    # unit-normal support by delta therefore contains its Euclidean dilation.
    support = np.max(normals @ source.T, axis=1)
    arithmetic_pad = _longdouble_error_scale(normals, source, support, delta)
    base = support + np.longdouble(delta) + arithmetic_pad

    # Export can perturb every miter vertex.  Inflate all support halfplanes
    # until the expanded polygon absorbs that measured perturbation, just as
    # the fixed-theta outer-envelope construction does.
    export_slack = np.longdouble(0.0)
    world = None
    for _ in range(24):
        ideal = _support_intersections(normals, base + export_slack)
        local = np.asarray(ideal, dtype=float)
        cast_delta = np.asarray(local, dtype=np.longdouble) - ideal
        cast_error = np.max(np.sqrt(np.sum(cast_delta * cast_delta, axis=1)))
        world, translation_error = oracle.translate_local_points(theta, local)
        needed = (np.longdouble(translation_error) + cast_error
                  + _longdouble_error_scale(ideal, local))
        needed = np.nextafter(needed, np.longdouble(np.inf))
        if needed <= export_slack:
            break
        # Do not chase a platform-dependent one-ULP fixed point.  On x86 the
        # extended-precision intersection changes ``needed`` by one final
        # binary64 ULP whenever it is rebuilt with exactly the previous
        # value; setting equality can therefore miss convergence for every
        # finite iteration count.  Add a binary64-scale headroom term and
        # rebuild once more.  This changes the constructed outer set in the
        # conservative direction rather than accepting an unmet bound.
        headroom = _longdouble_error_scale(
            ideal, local, needed, export_slack,
        )
        export_slack = np.nextafter(
            needed + headroom, np.longdouble(np.inf),
        )
    else:
        raise FloatingPointError("outer interval export bound did not converge")

    raw = Polygon(world)
    if not _finite_polygon(raw):
        raise FloatingPointError("outer interval halfplanes produced invalid geometry")
    # Convex hull is an outward-only repair.  The precision helper adds an
    # outward pad before snapping and explicitly verifies containment.
    snapped = set_precision_outer(raw.convex_hull, grid_size)
    if not _finite_polygon(snapped):
        raise FloatingPointError("outer interval precision geometry is invalid")
    snapped = orient(snapped.convex_hull, sign=1.0)

    # Independent necessary-and-sufficient convex containment audit.  For
    # every output edge normal n, require
    #   h_output(n) >= h_outer_mid(n) + delta ||n||.
    # Raw (non-unit) edge normals avoid certifying on a rounded-down norm.
    cover = _local_vertices(snapped, center)
    nxt = np.roll(cover, -1, axis=0)
    edges = nxt - cover
    raw_normals = np.stack([edges[:, 1], -edges[:, 0]], axis=1)
    lengths = np.sqrt(np.sum(raw_normals * raw_normals, axis=1))
    if np.any(lengths <= 0.0) or not np.all(np.isfinite(lengths)):
        raise FloatingPointError("outer interval cover has a degenerate edge")
    h_cover = np.minimum(
        np.sum(raw_normals * cover, axis=1),
        np.sum(raw_normals * nxt, axis=1),
    )
    h_source = np.max(raw_normals @ source.T, axis=1)
    required = np.longdouble(delta) * lengths
    projection_scales = np.concatenate([
        (np.abs(raw_normals) @ np.abs(cover).T).ravel(),
        (np.abs(raw_normals) @ np.abs(source).T).ravel(),
    ])
    audit_pad = _longdouble_error_scale(
        raw_normals, cover, source, h_cover, h_source, required,
        projection_scales,
    )
    if np.any(h_cover - h_source < required + audit_pad):
        raise FloatingPointError(
            "outer interval polygon failed support-containment audit")
    if not _convex_polygon_covers_vertices(snapped, outer_mid):
        raise FloatingPointError(
            "outer interval polygon failed midpoint containment audit")
    return snapped


def _inner_edge_audit(inner_mid: Polygon, candidate: Polygon,
                      center: np.ndarray, delta: float) -> bool:
    """Prove candidate is inside ``inner_mid erosion delta B``."""
    if candidate.is_empty:
        return True
    if (not _finite_polygon(candidate)
            or not _convex_polygon_covers_vertices(inner_mid, candidate)):
        return False
    source_poly = orient(inner_mid, sign=1.0)
    source = _local_vertices(source_poly, center)
    points = _local_vertices(candidate, center)
    nxt = np.roll(source, -1, axis=0)
    edges = nxt - source
    normals = np.stack([edges[:, 1], -edges[:, 0]], axis=1)
    lengths = np.sqrt(np.sum(normals * normals, axis=1))
    if np.any(lengths <= 0.0) or not np.all(np.isfinite(lengths)):
        return False
    # For a CCW convex polygon, each edge gives n.x <= h.  Every candidate
    # vertex must retain at least delta Euclidean distance from every edge.
    h_source = np.minimum(
        np.sum(normals * source, axis=1),
        np.sum(normals * nxt, axis=1),
    )
    h_candidate = np.max(normals @ points.T, axis=1)
    required = np.longdouble(delta) * lengths
    projection_scales = np.concatenate([
        (np.abs(normals) @ np.abs(source).T).ravel(),
        (np.abs(normals) @ np.abs(points).T).ravel(),
    ])
    audit_pad = _longdouble_error_scale(
        normals, source, points, h_source, h_candidate, required,
        projection_scales,
    )
    return bool(np.all(h_source - h_candidate >= required + audit_pad))


def _build_inner_common(inner_mid: Polygon, oracle: PairOracle, theta: float,
                        delta: float, grid_size: float) -> Polygon:
    if inner_mid.is_empty:
        return Polygon()
    if not _finite_polygon(inner_mid):
        return Polygon()
    convex = inner_mid.convex_hull
    # Expanding a non-convex inner approximation would be unsound.  Empty is
    # always a valid common inner obstacle and is the conservative fallback.
    if not inner_mid.covers(convex):
        return Polygon()
    try:
        eroded = convex.buffer(-delta, join_style="mitre")
        if eroded.is_empty or not isinstance(eroded, Polygon):
            return Polygon()
        candidate = set_precision_inner(eroded, grid_size)
    except (FloatingPointError, ValueError, shapely.GEOSException):
        return Polygon()
    if candidate.is_empty:
        return Polygon()
    center = oracle.center_longdouble(theta)
    if not _inner_edge_audit(convex, candidate, center, delta):
        return Polygon()
    return orient(candidate, sign=1.0)


def _lift_theta_near(theta: float, target: float) -> np.longdouble:
    """Lift a periodic theta to the representative nearest target."""
    turns = np.rint((np.longdouble(target) - np.longdouble(theta))
                    / np.longdouble(TWO_PI))
    return np.longdouble(theta) + turns * np.longdouble(TWO_PI)


def build_interval_pair_certificate(
    sandwich: ConvexSandwich,
    oracle: PairOracle,
    interval: Interval,
    grid_size: float,
) -> IntervalPairCertificate:
    """Construct a two-sided pair certificate valid for all theta in I.

    The function reports uncertainty as data.  No result is marked CERTIFIED
    unless the supplied fixed-orientation sandwich was itself CERTIFIED and
    every provenance, numeric, precision, and support-containment check passes.
    """
    checks = {}
    try:
        grid_size = float(grid_size)
        lo, hi = float(interval.lo), float(interval.hi)
        theta = float(sandwich.theta)
        if (not np.isfinite(grid_size) or grid_size <= 0.0
                or not np.isfinite(lo) or not np.isfinite(hi)
                or hi < lo or not np.isfinite(theta)):
            return _empty_certificate(
                sandwich, interval, reason="invalid_interval_or_precision",
                checks=checks, grid_size=grid_size,
            )

        checks["pair_id_matches"] = sandwich.pair_id == oracle.pair_id
        midpoint = np.longdouble(0.5) * (
            np.longdouble(lo) + np.longdouble(hi))
        lifted_theta = _lift_theta_near(theta, float(midpoint))
        midpoint_error = abs(lifted_theta - midpoint)
        midpoint_tol = (np.longdouble(16.0) * np.finfo(float).eps
                        * max(np.longdouble(1.0), abs(midpoint)))
        checks["midpoint_matches"] = bool(midpoint_error <= midpoint_tol)
        checks["fixed_sandwich_certified"] = (
            sandwich.status is CertStatus.CERTIFIED)
        checks["hausdorff_bound_finite"] = bool(
            sandwich.hausdorff_upper is not None
            and np.isfinite(sandwich.hausdorff_upper)
            and sandwich.hausdorff_upper >= 0.0)
        checks["outer_mid_valid"] = _finite_polygon(sandwich.outer)
        checks["inner_mid_valid_or_empty"] = bool(
            sandwich.inner.is_empty or _finite_polygon(sandwich.inner))
        checks["midpoint_sandwich_nested"] = (
            _convex_polygon_covers_vertices(
                sandwich.outer, sandwich.inner,
            )
        )

        world_bounds = np.asarray(sandwich.outer.bounds, dtype=float)
        world_scale = float(np.max(np.abs(world_bounds), initial=0.0))
        coordinate_ulp = float(abs(np.spacing(world_scale)))
        checks["world_precision_representable"] = bool(
            np.isfinite(coordinate_ulp) and coordinate_ulp <= grid_size)
        if not checks["world_precision_representable"]:
            raise FloatingPointError(
                "interval precision grid is finer than binary64 world ULP")

        lipschitz = float(oracle.theta_lipschitz())
        checks["theta_lipschitz_finite"] = bool(
            np.isfinite(lipschitz) and lipschitz >= 0.0)
        radius = max(abs(np.longdouble(lo) - lifted_theta),
                     abs(np.longdouble(hi) - lifted_theta))
        product = np.longdouble(lipschitz) * radius
        if product == 0.0:
            motion_bound = 0.0
        else:
            product += _longdouble_error_scale(lipschitz, radius, product)
            motion_bound = float(np.nextafter(product, np.longdouble(np.inf)))
            motion_bound = float(np.nextafter(motion_bound, np.inf))
        checks["motion_bound_finite"] = bool(
            np.isfinite(motion_bound) and motion_bound >= 0.0)

        prerequisites = all(checks.values())
        if not prerequisites:
            return _empty_certificate(
                sandwich, interval, lipschitz=lipschitz,
                motion_bound=motion_bound,
                reason="interval_pair_prerequisite_failed", checks=checks,
                grid_size=grid_size,
            )

        outer_cover = _build_outer_halfplane_cover(
            sandwich.outer, sandwich.angles, sandwich.directions,
            oracle, theta,
            motion_bound, grid_size,
        )
        inner_common = _build_inner_common(
            sandwich.inner, oracle, theta, motion_bound, grid_size,
        )
        checks["outer_support_audit"] = True
        checks["inner_erosion_audit"] = True
    except (FloatingPointError, OverflowError,
            np.linalg.LinAlgError, shapely.GEOSException) as exc:
        # Expected numerical failures are admissible UNKNOWN evidence.  A
        # ValueError here instead means that a supposedly certified internal
        # object has the wrong shape/type (for example a corrupted direction
        # schedule); do not disguise that programming/provenance fault as an
        # ordinary inability to certify the geometry.
        return _empty_certificate(
            sandwich, interval,
            lipschitz=locals().get("lipschitz", np.nan),
            motion_bound=locals().get("motion_bound", np.nan),
            reason="interval_pair_numeric_failure",
            checks={
                **checks,
                "exception": type(exc).__name__,
                "exception_message": str(exc),
            },
            grid_size=locals().get("grid_size", np.nan),
        )

    return IntervalPairCertificate(
        pair_id=sandwich.pair_id,
        theta=theta,
        interval=interval,
        outer_cover=outer_cover,
        inner_common=inner_common,
        theta_lipschitz=lipschitz,
        motion_bound=motion_bound,
        status=CertStatus.CERTIFIED,
        provenance={
            "construction": "midpoint_hausdorff_interval_pair_cover",
            "outer_method": "shifted_support_halfplanes_then_outward_precision",
            "inner_method": "midpoint_inner_erosion_then_inward_precision",
            "reason": "ok",
            "grid_size": grid_size,
            "interval_lo": lo,
            "interval_hi": hi,
            "midpoint_theta": theta,
            "angular_radius": float(radius),
            "theta_lipschitz": lipschitz,
            "motion_bound": motion_bound,
            "inner_common_empty": bool(inner_common.is_empty),
            "checks": checks,
        },
    )


__all__ = [
    "IntervalPairCertificate",
    "build_interval_pair_certificate",
]
