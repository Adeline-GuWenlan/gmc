"""M8 continuous safety kernel (Guide §11.2).

Everything here recomputes from scene/robot supports and poses — never from
stored safety booleans (§11.3 independence).

Primitives:
  pair margin lower bound: for direction u, m = u.t - h(u) > 0 implies
    dist(t, O) >= m (O lies in the halfplane u.x <= h(u)).
  translation: adaptive stepping — clearance is 1-Lipschitz in t, so a step
    of 0.9*c from a point with clearance lb c keeps the whole sub-segment
    certified.
  rotation-in-place: recursive theta-interval bounds — pair margin at the
    midpoint minus L_ij * halfwidth > 0 certifies the sub-interval
    (L_ij = pair theta-Lipschitz rate).
"""
import numpy as np


def _vec2(value, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (2,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite vector of shape (2,)")
    return result


def _finite_scalar(value, name: str) -> float:
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _nonnegative_int(value, name: str, minimum: int = 0) -> int:
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} must be an integer")
    result = int(value)
    if result != value or result < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")
    return result


def pair_margin(oracle, t, theta: float, coarse: int = 17,
                refine_rounds: int = 2) -> float:
    """Sound lower bound on dist(t, O_ij^theta); positive means free.

    Every direction u yields the valid halfplane bound u.t - h(u) (O lies in
    u.x <= h(u)); the maximum over directions equals the true distance for t
    outside convex O. The naive GJK-style fixed point cycles when the current
    bound is negative (direction reversal), so instead run a batched angular
    scan: `coarse` directions across +-pi/2 around the center direction, then
    `refine_rounds` local refinements around the best angle. The result is a
    max over finitely many valid bounds — sound by construction, tight to the
    angular resolution."""
    t = _vec2(t, "t")
    theta = _finite_scalar(theta, "theta")
    coarse = _nonnegative_int(coarse, "coarse", minimum=3)
    refine_rounds = _nonnegative_int(
        refine_rounds, "refine_rounds", minimum=0)
    c = np.asarray(oracle.center(theta), dtype=float)
    if c.shape != (2,) or not np.all(np.isfinite(c)):
        return -np.inf
    d = t - c
    base = np.arctan2(d[1], d[0]) if np.linalg.norm(d) > 1e-12 else 0.0

    def bounds_at(angles):
        U = np.stack([np.cos(angles), np.sin(angles)], axis=1)
        if hasattr(oracle, "separation_values"):
            # PairOracle cancels the large world translation before taking
            # directional differences.  The fallback below preserves support
            # for test doubles/third-party oracles, but certified GMC oracles
            # never form ``u@t - h_world`` at large coordinates.
            bounds = np.asarray(
                oracle.separation_values(theta, t, U), dtype=float)
        else:
            h = np.asarray(oracle.support_values(theta, U), dtype=float)
            if h.shape != (len(U),) or not np.all(np.isfinite(h)):
                return None
            bounds = U @ t - h
        if bounds.shape != (len(U),) or not np.all(np.isfinite(bounds)):
            return None
        return bounds

    angles = base + np.linspace(-np.pi / 2, np.pi / 2, coarse)
    vals = bounds_at(angles)
    if vals is None:
        return -np.inf
    best_i = int(np.argmax(vals))
    best, best_a = float(vals[best_i]), float(angles[best_i])
    span = np.pi / (coarse - 1)
    for _ in range(refine_rounds):
        angles = best_a + np.linspace(-span, span, 7)
        vals = bounds_at(angles)
        if vals is None:
            return -np.inf
        i = int(np.argmax(vals))
        if float(vals[i]) > best:
            best, best_a = float(vals[i]), float(angles[i])
        span /= 3.0
    return best


def clearance_lb(oracles, t, theta: float) -> float:
    t = _vec2(t, "t")
    theta = _finite_scalar(theta, "theta")
    oracles = tuple(oracles)
    if not oracles:
        return np.inf
    # Safe branch-and-bound over pairs.  The C-obstacle lies in the disc with
    # ``radius_bound`` about ``center``, hence ``disc_lb`` lower-bounds the
    # pair clearance.  pair_margin's default odd angular grid contains the
    # center direction and is therefore at least this radial disc bound.  Once
    # a smaller pair margin is known, pairs whose disc lower bound cannot beat
    # it are irrelevant to the minimum and require no support evaluations.
    ranked = []
    for order, oracle in enumerate(oracles):
        try:
            if hasattr(oracle, "disc_clearance_lower_bound"):
                disc_lb = float(
                    oracle.disc_clearance_lower_bound(theta, t))
            else:
                center = np.asarray(oracle.center(theta), dtype=float)
                radius = float(oracle.radius_bound())
                if (center.shape != (2,) or not np.all(np.isfinite(center))
                        or not np.isfinite(radius) or radius < 0.0):
                    disc_lb = -np.inf
                else:
                    disc_lb = float(np.linalg.norm(t - center) - radius)
            if not np.isfinite(disc_lb):
                disc_lb = -np.inf
        except (AttributeError, TypeError, ValueError, FloatingPointError):
            # Unknown oracle implementations remain evaluable and sort first;
            # lack of a valid pruning bound must never drop a safety check.
            disc_lb = -np.inf
        ranked.append((disc_lb, order, oracle))
    ranked.sort(key=lambda item: (item[0], item[1]))

    best = np.inf
    for disc_lb, _, oracle in ranked:
        if disc_lb >= best:
            continue
        best = min(best, pair_margin(oracle, t, theta))
    return float(best)


def translation_safe(oracles, p0, p1, theta: float,
                     min_step: float = 1e-6, floor: float = 0.0):
    """Certified sweep of the segment p0 -> p1 at fixed theta.
    Returns (ok, min_clearance_lb). `floor` demands clearance >= floor
    everywhere (fail-fast for eps_clear, §11.2.6): near-critical candidates
    abort in one probe instead of crawling with vanishing steps."""
    p0, p1 = _vec2(p0, "p0"), _vec2(p1, "p1")
    theta = _finite_scalar(theta, "theta")
    min_step = _finite_scalar(min_step, "min_step")
    floor = _finite_scalar(floor, "floor")
    if min_step <= 0.0:
        raise ValueError("min_step must be positive")
    if floor < 0.0:
        raise ValueError("floor must be non-negative")
    oracles = tuple(oracles)
    # Work over a dyadic parameter interval instead of repeatedly forming
    # ``p0 + s * direction`` in binary64 world coordinates.  At a large world
    # origin that expression can jump across several local-clearance scales.
    # Each sampled point is still exported for the oracle API, but its proven
    # interpolation/cast error is subtracted from the clearance bound.
    ld = np.longdouble
    ld_eps = np.finfo(ld).eps
    p0_long = np.asarray(p0, dtype=ld)
    p1_long = np.asarray(p1, dtype=ld)
    delta = p1_long - p0_long
    gamma4 = (ld(4.0) * ld_eps) / (ld(1.0) - ld(4.0) * ld_eps)
    delta_error = gamma4 * (np.abs(p0_long) + np.abs(p1_long))
    length_computed = np.sqrt(np.sum(delta * delta))
    length_error = (np.sqrt(np.sum(delta_error * delta_error))
                    + gamma4 * max(ld(1.0), abs(length_computed)))
    length_upper = np.nextafter(length_computed + length_error, ld(np.inf))
    if not np.isfinite(length_upper):
        return False, -np.inf
    if length_computed == 0.0:
        c = clearance_lb(oracles, p0, theta)
        return c > floor, c
    if not oracles:
        return True, np.inf

    def affine_probe(alpha):
        if alpha == ld(0.0):
            return np.asarray(p0, dtype=float), 0.0
        if alpha == ld(1.0):
            return np.asarray(p1, dtype=float), 0.0
        product = alpha * delta
        point_long = p0_long + product
        point = np.asarray(point_long, dtype=float)
        if not np.all(np.isfinite(point)):
            return None, np.inf
        # Bound long-double subtraction/multiply/add plus the measured final
        # cast to binary64.  The source endpoints are binary64 and therefore
        # exactly representable in every NumPy long-double implementation.
        product_error = (abs(alpha) * delta_error
                         + gamma4 * np.abs(product))
        arithmetic_error = (product_error
                            + gamma4 * (np.abs(p0_long)
                                        + np.abs(product)))
        cast_error = np.abs(np.asarray(point, dtype=ld) - point_long)
        component_error = arithmetic_error + cast_error
        error = np.sqrt(np.sum(component_error * component_error))
        return point, float(np.nextafter(error, ld(np.inf)))

    swept_lb = np.inf
    alpha = ld(0.0)
    # Candidates this close to the requested floor are deliberately left
    # UNKNOWN instead of spending the entire query budget chasing a vanishing
    # step.  This is a conservative completeness tradeoff, not safety slack.
    progress_margin = 1e-3
    while alpha < ld(1.0):
        point, interpolation_error = affine_probe(alpha)
        if point is None or not np.isfinite(interpolation_error):
            return False, -np.inf
        sampled = clearance_lb(oracles, point, theta)
        c = float(np.nextafter(
            ld(sampled) - ld(interpolation_error), -ld(np.inf)))
        if not np.isfinite(c) or c <= floor + progress_margin:
            return False, float(min(swept_lb, c))

        # Advance along the exact affine parameter.  The available clearance
        # is rounded downward and only 3/4 is spent, leaving ample room for
        # the two arithmetic roundings used to form the parameter step.
        available = np.nextafter(ld(c) - ld(floor), -ld(np.inf))
        certified_distance = np.nextafter(
            ld(0.75) * available, -ld(np.inf))
        remaining_fraction = np.nextafter(
            ld(1.0) - alpha, ld(np.inf))
        remaining_distance = np.nextafter(
            length_upper * remaining_fraction, ld(np.inf))
        if remaining_distance <= certified_distance:
            worst = float(np.nextafter(
                ld(c) - remaining_distance, -ld(np.inf)))
            endpoint, endpoint_error = affine_probe(ld(1.0))
            endpoint_clearance = float(np.nextafter(
                ld(clearance_lb(oracles, endpoint, theta))
                - ld(endpoint_error), -ld(np.inf)))
            swept_lb = min(swept_lb, worst, endpoint_clearance)
            return (endpoint_clearance > floor and swept_lb > floor,
                    float(swept_lb))

        step_fraction = np.nextafter(
            certified_distance / length_upper, ld(0.0))
        step_distance = np.nextafter(
            length_upper * step_fraction, ld(np.inf))
        worst = float(np.nextafter(
            ld(c) - step_distance, -ld(np.inf)))
        swept_lb = min(swept_lb, worst)
        if (not np.isfinite(step_fraction) or step_fraction <= 0.0
                or step_distance < min_step):
            return False, float(swept_lb)
        # Round progress toward the previous point so successive certified
        # intervals overlap rather than leaving an unproved floating gap.
        new_alpha = np.nextafter(alpha + step_fraction, -ld(np.inf))
        if not np.isfinite(new_alpha) or new_alpha <= alpha:
            return False, float(swept_lb)
        alpha = min(new_alpha, ld(1.0))
    return swept_lb > floor, float(swept_lb)


def rotation_interval_safe(oracles, t, th0: float, th1: float,
                           theta_min: float = 1e-4, floor: float = 0.0):
    """Certified rotation in place at t over [th0, th1] (th1 may exceed 2*pi
    for wrap crossings). Returns (ok, min_margin_lb). `floor` demands
    margin >= floor throughout (fail-fast, §11.2.6)."""
    t = _vec2(t, "t")
    th0 = _finite_scalar(th0, "th0")
    th1 = _finite_scalar(th1, "th1")
    theta_min = _finite_scalar(theta_min, "theta_min")
    floor = _finite_scalar(floor, "floor")
    if theta_min <= 0.0:
        raise ValueError("theta_min must be positive")
    if floor < 0.0:
        raise ValueError("floor must be non-negative")
    oracles = tuple(oracles)
    lo, hi = (th0, th1) if th1 >= th0 else (th1, th0)
    if hi == lo:
        c = clearance_lb(oracles, t, lo)
        return c > floor, c
    if not oracles:
        return True, np.inf

    interval_lb = np.inf
    pending = [(lo, hi)]
    while pending:
        a, b = pending.pop()
        # The represented endpoints, not an absolute angular tolerance,
        # define a degenerate interval.  With a large body-frame offset even
        # a sub-femtoredian rotation can move a support macroscopically.
        # Compute the centre without ``a+b`` overflow and use the actual
        # maximum endpoint distance when rounding puts the midpoint slightly
        # off-centre (or equal to an endpoint).
        mid = a + 0.5 * (b - a)
        ld = np.longdouble
        half = max(abs(ld(mid) - ld(a)), abs(ld(b) - ld(mid)))
        worst = np.inf
        for o in oracles:
            margin = pair_margin(o, t, mid)
            rate = float(o.theta_lipschitz())
            if (not np.isfinite(margin) or not np.isfinite(rate)
                    or rate < 0.0):
                worst = -np.inf
                break
            product = np.nextafter(
                ld(rate) * half, ld(np.inf))
            pair_worst = float(np.nextafter(
                ld(margin) - product, -ld(np.inf)))
            worst = min(worst, pair_worst)
        if worst > floor:
            interval_lb = min(interval_lb, worst)
            continue
        if b - a <= theta_min or not a < mid < b:
            return False, float(min(interval_lb, worst))
        pending.append((mid, b))
        pending.append((a, mid))
    return True, float(interval_lb)
