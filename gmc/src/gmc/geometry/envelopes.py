"""M3 convex inner/outer envelopes (Guide §6).

inner = convex hull of true support points  (subset of O)
outer = intersection of supporting halfplanes (superset of O)

Two certificate levels (§6.3):
  prototype: midpoint support residual heuristic -> APPROX_UNCERTIFIED
  theorem:   sector-Lipschitz Hausdorff upper bound -> CERTIFIED when
             bound <= eps_pair.

Theorem-mode bound (triangle-height certificate). Between adjacent contact
points p_k, p_{k+1} (touching the tangent lines of directions u_k, u_{k+1})
the true convex boundary arc lies inside the triangle (p_k, v_k, p_{k+1}),
where v_k is the outer polygon vertex (tangent-line intersection): the arc is
above the chord by convexity and below both tangent lines by definition of
support. Hence BOTH the outer excess dist(v_k, O) and the inner deficit in
that sector are bounded by dist(v_k, chord(p_k, p_{k+1})) — directly
computable, sound, and second-order in the sector width.
"""
from dataclasses import dataclass
from fractions import Fraction

import numpy as np
import shapely
from shapely.geometry import Polygon

from ..types import CertStatus, PairID
from .support import PairOracle, unit_dirs


@dataclass(frozen=True)
class ConvexSandwich:
    pair_id: PairID
    theta: float
    inner: Polygon
    outer: Polygon
    angles: np.ndarray
    # Exact binary64 unit rows used for the final support evaluations.  They
    # are retained separately from ``angles`` because recomputing cos/sin (or
    # omitting the row normalization in ``unit_dirs``) is not an exact replay
    # of the proof-critical direction set.
    directions: np.ndarray
    hausdorff_upper: float | None      # certified bound (theorem mode)
    gap_estimate: float                # heuristic residual (both modes)
    status: CertStatus
    support_calls: int
    # Scale-aware binary64 construction/export terms included in
    # ``hausdorff_upper``.  The historical field names are retained in the
    # artifact schema, but the values now cover local support arithmetic too.
    outer_translation_slack: float = 0.0
    inner_translation_slack: float = 0.0
    inner_shrink_factor: float = 0.0


def _binary64_error_scale(*arrays, factor=4096.0) -> np.longdouble:
    """Conservative allowance for data produced by binary64 arithmetic.

    Several proof-critical inputs are evaluated in :class:`PairOracle` before
    this module can promote them to extended precision.  Using
    ``longdouble.eps`` for those already-rounded values is platform dependent:
    it is binary64 on arm64 macOS but 80-bit extended precision on x86 Linux.
    The allowance must therefore be floored at binary64 epsilon.
    """
    scale = np.longdouble(1.0)
    for array in arrays:
        values = np.asarray(array, dtype=np.longdouble)
        if values.size:
            scale = max(scale, np.max(np.abs(values)))
    value = (np.longdouble(factor) * np.longdouble(np.finfo(float).eps)
             * scale)
    return np.nextafter(value, np.longdouble(np.inf))


def _direction_schedule(angles: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return query rows and the exact rows consumed by PairOracle.

    ``unit_dirs`` performs one binary64 normalization and PairOracle performs
    another at its public boundary.  Reproduce that second normalization once
    and retain those exact rows for tangent construction and artifact replay;
    never reconstruct proof directions from angles later in this module.
    """
    seed = unit_dirs(angles)
    norms = np.linalg.norm(seed, axis=1)
    if (not np.all(np.isfinite(norms)) or np.any(norms <= 0.0)):
        raise FloatingPointError("invalid support direction schedule")
    return seed, seed / norms[:, None]


def _outer_vertices(directions: np.ndarray, h: np.ndarray) -> np.ndarray:
    """Intersect adjacent support lines using the supplied exact rows."""
    u1 = np.asarray(directions, dtype=np.longdouble)
    u2 = np.roll(u1, -1, axis=0)
    h1 = np.asarray(h, dtype=np.longdouble)
    h2 = np.roll(h1, -1)
    det = u1[:, 0] * u2[:, 1] - u1[:, 1] * u2[:, 0]
    floor = (np.longdouble(64.0) * np.finfo(np.longdouble).eps)
    if (not np.all(np.isfinite(det)) or np.any(det <= floor)):
        raise FloatingPointError("ill-conditioned support direction schedule")
    vx = (h1 * u2[:, 1] - h2 * u1[:, 1]) / det
    vy = (h2 * u1[:, 0] - h1 * u2[:, 0]) / det
    vertices = np.stack([vx, vy], axis=1)
    if not np.all(np.isfinite(vertices)):
        raise FloatingPointError("non-finite supporting-line intersection")
    return np.asarray(vertices, dtype=float)


def _exact_orientation(a, b, c) -> int:
    """Exact orient2d sign for three binary64 coordinates."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    c = np.asarray(c, dtype=float)
    ax, ay = (Fraction.from_float(float(value)) for value in a)
    bx, by = (Fraction.from_float(float(value)) for value in b)
    cx, cy = (Fraction.from_float(float(value)) for value in c)
    exact = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    return int(exact > 0) - int(exact < 0)


def _filtered_orientation_matrix(starts: np.ndarray, ends: np.ndarray,
                                 points: np.ndarray) -> np.ndarray:
    """Robust signs for every directed edge x point pair in one batch.

    The bound is solely a floating-point filter: it never turns a small
    negative determinant into an accepted boundary.  Every unresolved sign is
    recomputed from the exact dyadic values of the binary64 coordinates.
    """
    starts64 = np.asarray(starts, dtype=float)
    ends64 = np.asarray(ends, dtype=float)
    points64 = np.asarray(points, dtype=float)
    ld = np.longdouble
    starts_ld = np.asarray(starts64, dtype=ld)
    ends_ld = np.asarray(ends64, dtype=ld)
    points_ld = np.asarray(points64, dtype=ld)
    edges = ends_ld - starts_ld
    relative = points_ld[None, :, :] - starts_ld[:, None, :]
    first = edges[:, None, 0] * relative[:, :, 1]
    second = edges[:, None, 1] * relative[:, :, 0]
    determinant = first - second

    # The stored coordinates are binary64, while the filter may accumulate in
    # either binary64 (arm64 macOS) or extended precision (x86 Linux).  The
    # binary64 floor keeps the same conservative branch decision on both.
    eps = max(ld(np.finfo(ld).eps), ld(np.finfo(float).eps))
    edge_error = ld(2.0) * eps * (
        np.abs(ends_ld) + np.abs(starts_ld)
    )
    relative_error = ld(2.0) * eps * (
        np.abs(points_ld)[None, :, :] + np.abs(starts_ld)[:, None, :]
    )
    propagated = (
        np.abs(edges[:, None, 0]) * relative_error[:, :, 1]
        + np.abs(relative[:, :, 1]) * edge_error[:, None, 0]
        + edge_error[:, None, 0] * relative_error[:, :, 1]
        + np.abs(edges[:, None, 1]) * relative_error[:, :, 0]
        + np.abs(relative[:, :, 0]) * edge_error[:, None, 1]
        + edge_error[:, None, 1] * relative_error[:, :, 0]
        + eps * (np.abs(first) + np.abs(second))
    )
    error = ld(32.0) * propagated
    signs = np.zeros(determinant.shape, dtype=np.int8)
    signs[determinant > error] = 1
    signs[determinant < -error] = -1
    for edge_index, point_index in np.argwhere(signs == 0):
        signs[edge_index, point_index] = _exact_orientation(
            starts64[edge_index], ends64[edge_index],
            points64[point_index],
        )
    return signs


def _filtered_orientation_rows(starts: np.ndarray, ends: np.ndarray,
                               points: np.ndarray) -> np.ndarray:
    """Robust orient2d signs for aligned ``(start, end, point)`` rows.

    This is the linear-size counterpart of
    :func:`_filtered_orientation_matrix`.  Convex point containment only
    needs a logarithmic number of aligned edge tests per point; materialising
    the full edge-by-point matrix made theorem envelope construction
    quadratic in the final direction count.
    """
    starts64 = np.asarray(starts, dtype=float)
    ends64 = np.asarray(ends, dtype=float)
    points64 = np.asarray(points, dtype=float)
    if (starts64.ndim != 2 or starts64.shape[1:] != (2,)
            or ends64.shape != starts64.shape
            or points64.shape != starts64.shape):
        raise ValueError("aligned orientation inputs must have shape (K, 2)")
    ld = np.longdouble
    starts_ld = np.asarray(starts64, dtype=ld)
    ends_ld = np.asarray(ends64, dtype=ld)
    points_ld = np.asarray(points64, dtype=ld)
    edges = ends_ld - starts_ld
    relative = points_ld - starts_ld
    first = edges[:, 0] * relative[:, 1]
    second = edges[:, 1] * relative[:, 0]
    determinant = first - second

    eps = max(ld(np.finfo(ld).eps), ld(np.finfo(float).eps))
    edge_error = ld(2.0) * eps * (
        np.abs(ends_ld) + np.abs(starts_ld)
    )
    relative_error = ld(2.0) * eps * (
        np.abs(points_ld) + np.abs(starts_ld)
    )
    propagated = (
        np.abs(edges[:, 0]) * relative_error[:, 1]
        + np.abs(relative[:, 1]) * edge_error[:, 0]
        + edge_error[:, 0] * relative_error[:, 1]
        + np.abs(edges[:, 1]) * relative_error[:, 0]
        + np.abs(relative[:, 0]) * edge_error[:, 1]
        + edge_error[:, 1] * relative_error[:, 0]
        + eps * (np.abs(first) + np.abs(second))
    )
    error = ld(32.0) * propagated
    signs = np.zeros(determinant.shape, dtype=np.int8)
    signs[determinant > error] = 1
    signs[determinant < -error] = -1
    for index in np.flatnonzero(signs == 0):
        signs[index] = _exact_orientation(
            starts64[index], ends64[index], points64[index],
        )
    return signs


def _polygon_orientation(vertices: np.ndarray) -> int:
    """Exact sign of a binary64 polygon's signed area."""
    promoted = np.asarray(vertices, dtype=np.longdouble)
    # Shoelace on world coordinates is catastrophically translation
    # sensitive: for a unit polygon near 1e10, each O(1e20) cross product can
    # lose the complete O(1) area before the final sum.  Signed area is
    # translation invariant, so form the fan about one stored vertex first.
    # The subtraction error is explicitly propagated below; on common nearby
    # coordinates Sterbenz' lemma makes the translation exact in practice.
    shifted = promoted - promoted[0]
    following = np.roll(shifted, -1, axis=0)
    first = shifted[:, 0] * following[:, 1]
    second = shifted[:, 1] * following[:, 0]
    terms = first - second
    twice_area_approx = np.sum(terms)

    # Certified floating-point filter.  Bounding only ``sum(abs(terms))`` is
    # insufficient because each term is itself the difference of two rounded
    # products.  Track errors from the anchor subtraction, both products,
    # each cross-product subtraction, and the final reduction.  A generous
    # factor also covers rounding while evaluating this bound.  If any bound
    # arithmetic is non-finite (or the sign is not separated), fall through
    # to the exact dyadic computation.
    ld = np.longdouble
    finfo = np.finfo(ld)
    eps = ld(finfo.eps)
    tiny = ld(finfo.smallest_subnormal)
    anchor_scale = np.abs(promoted) + np.abs(promoted[0])[None, :]
    shifted_error = eps * anchor_scale + tiny
    following_error = np.roll(shifted_error, -1, axis=0)

    first_input_error = (
        np.abs(shifted[:, 0]) * following_error[:, 1]
        + np.abs(following[:, 1]) * shifted_error[:, 0]
        + shifted_error[:, 0] * following_error[:, 1]
    )
    second_input_error = (
        np.abs(shifted[:, 1]) * following_error[:, 0]
        + np.abs(following[:, 0]) * shifted_error[:, 1]
        + shifted_error[:, 1] * following_error[:, 0]
    )
    first_scale = ((np.abs(shifted[:, 0]) + shifted_error[:, 0])
                   * (np.abs(following[:, 1])
                      + following_error[:, 1]))
    second_scale = ((np.abs(shifted[:, 1]) + shifted_error[:, 1])
                    * (np.abs(following[:, 0])
                       + following_error[:, 0]))
    first_error = first_input_error + eps * first_scale + tiny
    second_error = second_input_error + eps * second_scale + tiny
    term_error = (
        first_error + second_error
        + eps * (np.abs(first) + np.abs(second)) + tiny
    )

    count = ld(max(1, len(terms)))
    reduction_factor = count * eps
    if reduction_factor < ld(0.5):
        reduction_error = (
            reduction_factor / (ld(1.0) - reduction_factor)
            * np.sum(np.abs(terms))
            + count * tiny
        )
        filter_error = np.nextafter(
            ld(16.0) * (np.sum(term_error) + reduction_error),
            ld(np.inf),
        )
        if np.isfinite(twice_area_approx) and np.isfinite(filter_error):
            if twice_area_approx > filter_error:
                return 1
            if twice_area_approx < -filter_error:
                return -1

    xs = [Fraction.from_float(float(value)) for value in vertices[:, 0]]
    ys = [Fraction.from_float(float(value)) for value in vertices[:, 1]]
    twice_area = sum(
        xs[index] * ys[(index + 1) % len(vertices)]
        - ys[index] * xs[(index + 1) % len(vertices)]
        for index in range(len(vertices))
    )
    return int(twice_area > 0) - int(twice_area < 0)


def _convex_contains_vertices(outer_vertices: np.ndarray,
                              inner_vertices: np.ndarray) -> bool:
    """Exact binary64 convex containment, with boundary included."""
    outer = np.asarray(outer_vertices, dtype=float)
    inner = np.asarray(inner_vertices, dtype=float)
    if inner.size == 0:
        return True
    if (outer.ndim != 2 or outer.shape[1] != 2 or len(outer) < 3
            or inner.ndim != 2 or inner.shape[1] != 2
            or not np.all(np.isfinite(outer))
            or not np.all(np.isfinite(inner))):
        return False
    orientation = _polygon_orientation(outer)
    if orientation == 0:
        return False
    if orientation < 0:
        outer = outer[::-1]

    # The tangent polygon produced by this module is strictly convex.  Verify
    # that prerequisite exactly, then locate each inner vertex in the fan
    # rooted at outer[0].  This reduces the audit from O(n*m) robust
    # predicates to O(n + m log n).  Retain the historical quadratic path as
    # a fail-safe for a valid convex polygon containing collinear vertices.
    following = np.roll(outer, -1, axis=0)
    turns = _filtered_orientation_rows(
        outer, following, np.roll(outer, -2, axis=0),
    )
    if np.any(turns <= 0):
        return bool(np.all(
            _filtered_orientation_matrix(outer, following, inner) >= 0,
        ))

    count = len(inner)
    anchor = np.repeat(outer[0][None, :], count, axis=0)
    first = np.repeat(outer[1][None, :], count, axis=0)
    last = np.repeat(outer[-1][None, :], count, axis=0)
    first_sign = _filtered_orientation_rows(anchor, first, inner)
    last_sign = _filtered_orientation_rows(anchor, last, inner)
    if np.any(first_sign < 0) or np.any(last_sign > 0):
        return False

    def on_segment(points, a, b):
        lower = np.minimum(a, b)
        upper = np.maximum(a, b)
        return np.all((points >= lower) & (points <= upper), axis=1)

    first_boundary = first_sign == 0
    if np.any(first_boundary) and not np.all(on_segment(
            inner[first_boundary], outer[0], outer[1])):
        return False
    last_boundary = last_sign == 0
    if np.any(last_boundary) and not np.all(on_segment(
            inner[last_boundary], outer[0], outer[-1])):
        return False

    active_indices = np.flatnonzero(~(first_boundary | last_boundary))
    if len(active_indices) == 0:
        return True
    active = inner[active_indices]
    lo = np.ones(len(active), dtype=np.int64)
    hi = np.full(len(active), len(outer) - 1, dtype=np.int64)
    while np.any(hi - lo > 1):
        mid = (lo + hi) // 2
        signs = _filtered_orientation_rows(
            np.repeat(outer[0][None, :], len(active), axis=0),
            outer[mid], active,
        )
        advance = signs >= 0
        lo = np.where(advance, mid, lo)
        hi = np.where(advance, hi, mid)
    final_signs = _filtered_orientation_rows(
        outer[lo], outer[hi], active,
    )
    return bool(np.all(final_signs >= 0))


def _sector_widths(angles: np.ndarray) -> np.ndarray:
    return np.diff(np.append(angles, angles[0] + 2 * np.pi))


def _triangle_heights(vertices: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Sector triangle heights used by the theorem certificate."""
    p1, p2 = points, np.roll(points, -1, axis=0)
    seg = p2 - p1
    seglen2 = np.einsum("ki,ki->k", seg, seg)
    tpar = np.clip(np.einsum("ki,ki->k", vertices - p1, seg)
                   / np.maximum(seglen2, 1e-300), 0.0, 1.0)
    foot = p1 + tpar[:, None] * seg
    return np.linalg.norm(vertices - foot, axis=1)


def _local_numeric_allowances(oracle: PairOracle, theta: float,
                              directions: np.ndarray, support: np.ndarray,
                              points: np.ndarray) -> tuple[float, float]:
    """Bound binary64 support-value and support-point evaluation error.

    The long-double recomputation measures the observed cast/evaluation
    discrepancy.  A binary64-epsilon condition allowance remains even on
    x86, where long double is more precise, so the proof is not accidentally
    weaker than the arithmetic that produced ``support`` and ``points``.
    """
    _, _, body_covariance, _ = oracle._frame(theta)
    ld = np.longdouble
    rows = np.asarray(directions, dtype=ld)
    scene_covariance = np.asarray(oracle.scene.covariance, dtype=ld)
    body_covariance = np.asarray(body_covariance, dtype=ld)
    scene_product = rows @ scene_covariance.T
    body_product = rows @ body_covariance.T
    scene_quadratic = np.sum(rows * scene_product, axis=1)
    body_quadratic = np.sum(rows * body_product, axis=1)
    if (np.any(scene_quadratic <= 0.0) or np.any(body_quadratic <= 0.0)
            or not np.all(np.isfinite(scene_quadratic))
            or not np.all(np.isfinite(body_quadratic))):
        raise FloatingPointError("support arithmetic lost positive definiteness")
    scene_root = np.sqrt(scene_quadratic)
    body_root = np.sqrt(body_quadratic)
    reference_support = (
        ld(oracle.scene.level) * scene_root
        + ld(oracle.body.level) * body_root
    )
    reference_points = (
        ld(oracle.scene.level) * scene_product / scene_root[:, None]
        + ld(oracle.body.level) * body_product / body_root[:, None]
    )
    support_discrepancy = np.max(np.abs(
        np.asarray(support, dtype=ld) - reference_support
    ))
    point_delta = np.asarray(points, dtype=ld) - reference_points
    point_discrepancy = np.max(np.sqrt(np.sum(
        point_delta * point_delta, axis=1,
    )))

    # Support points divide by sqrt(u^T Sigma u); include the actual schedule
    # conditioning rather than assuming a well-rounded ellipse.
    condition_scale = max(
        ld(1.0),
        np.max(np.abs(reference_support)),
        np.max(np.abs(reference_points)),
        ld(oracle.scene.level)
        * np.max(np.abs(scene_covariance)) / np.min(scene_root),
        ld(oracle.body.level)
        * np.max(np.abs(body_covariance)) / np.min(body_root),
    )
    support_pad = support_discrepancy + _binary64_error_scale(
        directions, support, reference_support, condition_scale,
    )
    point_pad = point_discrepancy + _binary64_error_scale(
        directions, points, reference_points, condition_scale,
        factor=8192.0,
    )
    return (
        float(np.nextafter(support_pad, ld(np.inf))),
        float(np.nextafter(point_pad, ld(np.inf))),
    )


def _radius_upper_bound(oracle: PairOracle) -> float:
    radius = float(oracle.radius_bound())
    pad = _binary64_error_scale(radius)
    return float(np.nextafter(np.longdouble(radius) + pad,
                              np.longdouble(np.inf)))


def _inradius_lower_bound(oracle: PairOracle) -> float:
    inradius = float(oracle.inradius_bound())
    covariance_scale = max(
        np.max(np.abs(oracle.scene.covariance)),
        np.max(np.abs(oracle.body.covariance)),
    )
    pad = _binary64_error_scale(inradius, covariance_scale)
    lower = np.longdouble(inradius) - pad
    return float(np.nextafter(lower, np.longdouble(-np.inf)))


def _outer_support_margin(oracle: PairOracle, theta: float,
                          world_vertices: np.ndarray) -> float:
    """Directed lower bound on the distance from O to every outer edge."""
    vertices = np.asarray(world_vertices, dtype=float)
    orientation = _polygon_orientation(vertices)
    if orientation == 0:
        return -np.inf
    if orientation < 0:
        vertices = vertices[::-1]
    # PairOracle's public world support uses its binary64 centre.  Audit the
    # materialised polygon against that same formal M1 object; the separate
    # translation bound already forced the construction far enough outward.
    center = np.asarray(oracle.center(theta), dtype=np.longdouble)
    local = np.asarray(vertices, dtype=np.longdouble) - center[None, :]
    nxt = np.roll(local, -1, axis=0)
    edges = nxt - local
    normals = np.stack([edges[:, 1], -edges[:, 0]], axis=1)
    lengths = np.sqrt(np.sum(normals * normals, axis=1))
    if np.any(lengths <= 0.0) or not np.all(np.isfinite(lengths)):
        return -np.inf
    query_rows = np.asarray(normals / lengths[:, None], dtype=float)
    # PairOracle normalizes once at its public boundary.  Reproduce that exact
    # row set for the candidate-edge projection too.
    query_norms = np.linalg.norm(query_rows, axis=1)
    evaluated_rows = query_rows / query_norms[:, None]
    true_support = oracle.local_support_values(theta, query_rows)
    support_pad = _binary64_error_scale(
        evaluated_rows, true_support, oracle.scene.covariance,
        oracle.body.covariance, _radius_upper_bound(oracle),
    )
    evaluated = np.asarray(evaluated_rows, dtype=np.longdouble)
    candidate_support = np.minimum(
        np.sum(evaluated * local, axis=1),
        np.sum(evaluated * nxt, axis=1),
    )
    audit_pad = _binary64_error_scale(
        evaluated, local, candidate_support, true_support,
    )
    margin = (candidate_support - np.asarray(true_support, dtype=np.longdouble)
              - np.longdouble(support_pad) - audit_pad)
    return float(np.min(margin))


def _outward_world_vertices(oracle: PairOracle, theta: float,
                            directions: np.ndarray, h: np.ndarray,
                            support_error: float):
    """Translate an outer tangent polygon without losing containment.

    If each ideal vertex moves by at most ``s`` during binary64 export, the
    support function of the perturbed vertex hull can decrease by at most
    ``s``.  Building the ideal tangent polygon from supports ``h + s`` first
    contains ``O (+) sB``; the exported polygon therefore still contains O.
    The fixed point is monotone because the measured/derived export bound may
    change by an ulp after the support inflation moves a vertex.
    """
    slack = float(np.nextafter(support_error, np.inf))
    for _ in range(24):
        ideal = np.asarray(_outer_vertices(
            directions, np.asarray(h, dtype=np.longdouble) + slack,
        ), dtype=float)
        # `_outer_vertices` exports extended-precision intersections once.
        # Cover that cast and all preceding binary64 support arithmetic before
        # the second export into world coordinates.
        local_pad = float(_binary64_error_scale(
            directions, h, ideal, slack,
        ))
        world, translation_error = oracle.translate_local_points(theta, ideal)
        candidate = Polygon(world)
        finite_candidate = bool(
            not candidate.is_empty and candidate.is_valid
            and np.isfinite(candidate.area) and candidate.area > 0.0
        )
        margin = (_outer_support_margin(oracle, theta, world)
                  if finite_candidate else -np.inf)
        if margin >= 0.0:
            total_error = float(np.nextafter(
                support_error + local_pad + translation_error, np.inf,
            ))
            return ideal, world, slack, total_error

        # Inflate by the measured proof deficit plus a fresh arithmetic pad.
        # This changes the constructed set; it never accepts an existing
        # negative margin within a tolerance.
        deficit = 0.0 if np.isfinite(margin) else slack
        if np.isfinite(margin) and margin < 0.0:
            deficit = -margin
        step = max(
            float(np.nextafter(deficit, np.inf)),
            local_pad + translation_error,
            float(np.nextafter(support_error, np.inf)),
        )
        slack = float(np.nextafter(slack + step, np.inf))
    raise FloatingPointError(
        "outer containment certificate did not converge")


def _prototype_outward_world_vertices(oracle: PairOracle, theta: float,
                                       directions: np.ndarray,
                                       h: np.ndarray):
    """Cheap export-only compensation for an uncertified prototype pair.

    Prototype geometry can guide candidate search, but it is never evidence
    for a global possible-space cut.  Keep the historical world-translation
    fixed point without paying for theorem-only true-support edge audits.
    """
    slack = 0.0
    for _ in range(16):
        local = _outer_vertices(
            directions, np.asarray(h, dtype=np.longdouble) + slack,
        )
        world, needed = oracle.translate_local_points(theta, local)
        if needed <= slack:
            return local, world, slack, needed
        slack = max(slack, needed)
    raise FloatingPointError(
        "prototype outer world-coordinate bound did not converge")


def _inward_world_points(oracle: PairOracle, theta: float,
                         points: np.ndarray, local_point_error: float):
    """Translate inner vertices after a provably sufficient homothety.

    The centred C-obstacle contains ``rB`` where ``r`` is
    :meth:`PairOracle.inradius_bound`.  If export moves a vertex by at most
    ``e``, shrinking every certified inner point by ``alpha >= e/r`` keeps the
    exported vertex inside O by convexity:
    ``(1-alpha) O (+) eB subset O``.
    """
    inradius = _inradius_lower_bound(oracle)
    if not np.isfinite(inradius) or inradius <= 0.0:
        raise FloatingPointError("pair inradius bound is not positive")
    alpha = 0.0
    for _ in range(16):
        local = (1.0 - alpha) * points
        world, export_error = oracle.translate_local_points(theta, local)
        multiply_error = float(_binary64_error_scale(
            points, local, alpha,
        ))
        error = ((1.0 - alpha) * local_point_error
                 + multiply_error + export_error)
        needed = float(np.nextafter(error / inradius, np.inf))
        if needed <= alpha:
            return local, world, alpha, error
        if not np.isfinite(needed) or needed >= 1.0:
            # There is no representable nonempty inner certificate at this
            # world scale.  Returning an empty inner set is conservative; the
            # compiler's M0 resolution gate rejects such runs before use.
            return np.empty((0, 2), dtype=float), np.empty((0, 2), dtype=float), \
                1.0, error
        alpha = max(alpha, needed)
    raise FloatingPointError(
        "inner world-coordinate rounding bound did not converge")


def approximate_pair(oracle: PairOracle, theta: float, cfg,
                     seed_angles=None) -> ConvexSandwich:
    """Approximate one pair from a canonical direction schedule.

    ``seed_angles`` is retained as a compatibility-only argument.  A previous
    implementation initialized adaptive refinement from a neighbouring
    orientation, making geometry and status depend on cache history.  Until
    continuation has a canonical refinement rule, seeds must not influence the
    geometric result.
    """
    del seed_angles
    angles = np.linspace(0.0, 2 * np.pi, cfg.initial_directions,
                         endpoint=False)
    calls0 = oracle.calls
    mode = cfg.certificate_mode
    # Adaptive densification retains every old direction and promotes the
    # selected sector midpoints into the next schedule.  Cache those exact
    # binary64 angle evaluations within this construction instead of paying
    # for the same support value/point again at every 32 -> 64 -> 128 round.
    # The key is the unquantized float angle, so this cannot merge nearby
    # directions or make the result history dependent.
    value_cache: dict[float, float] = {}
    point_cache: dict[float, np.ndarray] = {}

    def cached_values(schedule: np.ndarray,
                      query_rows: np.ndarray) -> np.ndarray:
        keys = tuple(float(value) % (2 * np.pi) for value in schedule)
        missing = [index for index, key in enumerate(keys)
                   if key not in value_cache]
        if missing:
            computed = oracle.local_support_values(
                theta, query_rows[np.asarray(missing, dtype=int)],
            )
            for index, value in zip(missing, computed):
                value_cache[keys[index]] = float(value)
        hits = len(keys) - len(missing)
        if hits and oracle.ledger is not None:
            oracle.ledger.charge("cache_hits", hits)
        return np.asarray([value_cache[key] for key in keys], dtype=float)

    def cached_points(schedule: np.ndarray,
                      query_rows: np.ndarray) -> np.ndarray:
        keys = tuple(float(value) % (2 * np.pi) for value in schedule)
        missing = [index for index, key in enumerate(keys)
                   if key not in point_cache]
        if missing:
            computed = oracle.local_support_points(
                theta, query_rows[np.asarray(missing, dtype=int)],
            )
            for index, value in zip(missing, computed):
                point_cache[keys[index]] = np.asarray(
                    value, dtype=float,
                ).copy()
        hits = len(keys) - len(missing)
        if hits and oracle.ledger is not None:
            oracle.ledger.charge("cache_hits", hits)
        return np.asarray([point_cache[key] for key in keys], dtype=float)

    while True:
        query_U, U = _direction_schedule(angles)
        # Everything geometric is constructed about the pair centre.  Only
        # the final polygon vertices cross into large world coordinates.
        h = cached_values(angles, query_U)
        pts = cached_points(angles, query_U)
        base_verts = _outer_vertices(U, h)
        if mode == "theorem":
            support_error, point_error = _local_numeric_allowances(
                oracle, theta, U, h, pts,
            )
            inner_pts, world_inner_pts, inner_alpha, inner_error = \
                _inward_world_points(oracle, theta, pts, point_error)
            verts, world_verts, outer_slack, outer_error = \
                _outward_world_vertices(
                    oracle, theta, U, h, support_error,
                )
        else:
            # This branch is intentionally incapable of producing a theorem
            # status.  It preserves only the export compensation needed for
            # stable candidate geometry and delegates every final REACHABLE
            # claim to the independent continuous verifier.
            point_error = 0.0
            inner_pts, world_inner_pts, inner_alpha, inner_error = \
                _inward_world_points(oracle, theta, pts, point_error)
            verts, world_verts, outer_slack, outer_error = \
                _prototype_outward_world_vertices(
                    oracle, theta, U, h,
                )
        widths = _sector_widths(angles)

        mid = angles + widths / 2.0
        query_mid, u_mid = _direction_schedule(mid)
        h_mid = cached_values(mid, query_mid)
        # The inner dimension is exactly two.  Spell out the dot products
        # instead of dispatching thousands of tiny GEMMs to platform BLAS;
        # Accelerate has been observed to segfault here late in a long test
        # process even though the same isolated calculation is valid.
        midpoint_projections = (
            u_mid[:, 0, None] * pts[None, :, 0]
            + u_mid[:, 1, None] * pts[None, :, 1]
        )
        h_inner_mid = np.max(midpoint_projections, axis=1)
        residual = h_mid - h_inner_mid          # per-sector sandwich residual

        if mode == "theorem":
            base_sector = _triangle_heights(base_verts, pts)
            outer_sector = _triangle_heights(verts, pts)
            # Export can move the inflated outer hull inward by outer_error.
            # For the inner hull, homothetic shrink loses at most alpha times
            # the pair circumradius, followed by inner_error on export.
            inner_loss = (
                inner_alpha * _radius_upper_bound(oracle) + inner_error
            )
            sector_bound = np.maximum(
                outer_sector + point_error + outer_error,
                base_sector + point_error + inner_loss,
            )
        else:
            sector_bound = residual

        gap = float(sector_bound.max())
        if gap <= cfg.eps_pair or len(angles) >= cfg.max_directions:
            break
        # adaptive densification, batched: split every sector whose bound
        # exceeds eps_pair at once (converges in a few rebuilds instead of
        # one direction per rebuild)
        room = cfg.max_directions - len(angles)
        bad = np.flatnonzero(sector_bound > cfg.eps_pair)
        if len(bad) > room:
            bad = bad[np.argsort(sector_bound[bad])[::-1][:room]]
        angles = np.sort(np.append(angles, mid[bad]))

    inner = (shapely.MultiPoint(world_inner_pts).convex_hull
             if len(world_inner_pts) else Polygon())
    outer = Polygon(world_verts)
    if mode == "theorem":
        outer_vertices = np.asarray(
            outer.exterior.coords[:-1], dtype=float,
        )
        inner_vertices = (
            np.asarray(inner.exterior.coords[:-1], dtype=float)
            if isinstance(inner, Polygon) and not inner.is_empty
            else np.empty((0, 2), dtype=float)
        )
        geometry_certified = bool(
            isinstance(outer, Polygon)
            and not outer.is_empty and outer.is_valid and outer.area > 0.0
            and (inner.is_empty or (
                isinstance(inner, Polygon)
                and inner.is_valid and inner.area > 0.0
            ))
            and _convex_contains_vertices(outer_vertices, inner_vertices)
        )
        status = (CertStatus.CERTIFIED
                  if gap <= cfg.eps_pair and geometry_certified
                  else CertStatus.APPROX_UNCERTIFIED)
        hausdorff = gap
    else:
        status = CertStatus.APPROX_UNCERTIFIED
        hausdorff = None
    return ConvexSandwich(pair_id=oracle.pair_id, theta=float(theta),
                          inner=inner, outer=outer, angles=angles,
                          directions=np.array(U, dtype=float, copy=True),
                          hausdorff_upper=hausdorff,
                          gap_estimate=float(residual.max()),
                          status=status,
                          support_calls=oracle.calls - calls0,
                          outer_translation_slack=float(outer_slack),
                          inner_translation_slack=float(inner_error),
                          inner_shrink_factor=float(inner_alpha))


def dilated_outer(sandwich: ConvexSandwich, delta: float) -> Polygon:
    """Outer polygon of the obstacle dilated by delta (support + delta):
    used by orientation interval certificates (§8)."""
    return sandwich.outer.buffer(delta, join_style="mitre", mitre_limit=8.0)
