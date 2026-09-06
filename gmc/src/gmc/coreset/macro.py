"""M2 certified macro-primitive construction (design revision 5, gate C1).

For a hierarchy node ``v`` with member ellipses ``{E_i}`` we build a single
outer ellipse with

    union_i E_i  subset  E_plus.

The construction is a minimum-volume enclosing ellipse (Khachiyan) followed by
a *certified* radial inflation.  Sampling directions alone would only bound the
containment defect at the sampled directions; the inflation therefore adds the
Lipschitz half-step term, which makes the guarantee hold at every direction on
the circle:

    h_K is Lipschitz in the direction angle with constant R_K (circumradius),
    so for uniform directions of angular step D,
        sup_u f(u) <= max_sampled f(u) + (R_union + R_E) * D/2.

Radial inflation by ``gamma`` raises the ellipse support function by at least
``gamma * r_min``, so ``gamma = eps / r_min`` closes any residual defect.  The
result is re-certified before it is returned; a construction that still fails
falls back to the bounding disc, which contains the union by definition.
"""
from dataclasses import dataclass

import numpy as np
from shapely.geometry import MultiPoint, Polygon

from ..types import GaussianSupport2D, Vec2

# Directions used to certify containment.  4096 uniform directions give an
# angular half-step of pi/4096 ~ 7.7e-4 rad; at the ~1 m scale of the toy
# scenes the Lipschitz term is then sub-millimetre, far below any geometry the
# compiler resolves.
CERT_DIRECTIONS = 4096
# Directions used to seed the enclosing-ellipse point cloud per member.
SEED_DIRECTIONS = 64


@dataclass(frozen=True)
class MacroEllipse:
    """An outer macro primitive plus its containment certificate."""
    support: GaussianSupport2D
    slack: float          # certified margin; >= 0 means containment is proved
    excess_area: float    # area(E_plus) - area(union members), diagnostics only
    n_members: int
    fallback: bool        # True if the MVEE failed and the disc was used

    @property
    def certified(self) -> bool:
        return self.slack >= 0.0


def _unit_directions(n: int) -> np.ndarray:
    angles = 2.0 * np.pi * np.arange(n, dtype=np.float64) / float(n)
    return np.column_stack([np.cos(angles), np.sin(angles)])


def shape_matrix(s: GaussianSupport2D) -> np.ndarray:
    """``S`` with ``h(u) = u.mu + sqrt(u^T S u)``."""
    return (s.level ** 2) * np.asarray(s.covariance, dtype=np.float64)


def support_values(s: GaussianSupport2D, U: np.ndarray,
                   origin: Vec2) -> np.ndarray:
    """Support values of ``s`` about ``origin`` for unit rows ``U``."""
    S = shape_matrix(s)
    offset = np.asarray(s.mean, dtype=np.float64) - np.asarray(origin,
                                                               dtype=np.float64)
    quad = np.einsum("ki,ij,kj->k", U, S, U)
    return U @ offset + np.sqrt(np.maximum(quad, 0.0))


def union_support_values(members, U: np.ndarray, origin: Vec2) -> np.ndarray:
    """``h_{union}(u) = max_i h_i(u)`` -- exact for a union of convex bodies."""
    return np.max(np.stack([support_values(m, U, origin) for m in members]),
                  axis=0)


def ellipse_polygon(s: GaussianSupport2D, n: int = 256,
                    outer: bool = True) -> Polygon:
    """Polygonal approximation of an ellipse.

    ``outer=True`` returns a strict superset (the inscribed polygon scaled by
    ``1/cos(pi/n)``), which keeps every downstream free-space test conservative.
    """
    if n < 8:
        raise ValueError("ellipse polygon needs at least 8 vertices")
    S = shape_matrix(s)
    eigval, eigvec = np.linalg.eigh(S)
    semi = np.sqrt(np.maximum(eigval, 0.0))
    t = 2.0 * np.pi * np.arange(n, dtype=np.float64) / float(n)
    pts = np.column_stack([semi[0] * np.cos(t), semi[1] * np.sin(t)])
    if outer:
        pts = pts / np.cos(np.pi / n)
    return Polygon((eigvec @ pts.T).T + np.asarray(s.mean, dtype=np.float64))


def _mvee(points: np.ndarray, tol: float = 1e-9,
          max_iter: int = 400) -> tuple[np.ndarray, np.ndarray]:
    """Khachiyan minimum-volume enclosing ellipse of a 2D point cloud.

    ``max_iter`` is deliberately modest: stopping early yields a slightly
    loose ellipse, never an invalid one, because :func:`outer_ellipse` always
    certifies containment afterwards and inflates if the bound is not met.
    """
    P = np.asarray(points, dtype=np.float64)
    n, d = P.shape
    if n <= d:
        raise np.linalg.LinAlgError("not enough points for an enclosing ellipse")
    Q = np.vstack([P.T, np.ones(n)])
    u = np.full(n, 1.0 / n)
    for _ in range(max_iter):
        X = Q @ (u[:, None] * Q.T)
        M = np.einsum("ij,jk,ki->i", Q.T, np.linalg.inv(X), Q)
        j = int(np.argmax(M))
        if M[j] <= d + 1.0:
            break
        step = (M[j] - d - 1.0) / ((d + 1.0) * (M[j] - 1.0))
        u_next = (1.0 - step) * u
        u_next[j] += step
        if np.linalg.norm(u_next - u) <= tol:
            u = u_next
            break
        u = u_next
    center = P.T @ u
    S = d * (P.T @ (u[:, None] * P) - np.outer(center, center))
    return center, 0.5 * (S + S.T)


def _as_support(center: Vec2, S: np.ndarray, level: float,
                primitive_id: int) -> GaussianSupport2D:
    """Rebuild a ``GaussianSupport2D`` from a shape matrix, flooring the
    spectrum so a degenerate (collinear) group stays positive definite."""
    eigval, eigvec = np.linalg.eigh(0.5 * (S + S.T))
    floor = max(float(np.max(eigval)), 1.0) * 1e-12
    eigval = np.maximum(eigval, floor)
    S_pd = eigvec @ np.diag(eigval) @ eigvec.T
    cov = 0.5 * (S_pd + S_pd.T) / (level ** 2)
    return GaussianSupport2D(mean=np.asarray(center, dtype=np.float64),
                             covariance=cov, level=level,
                             primitive_id=primitive_id)


def certify_outer_containment(members, candidate: GaussianSupport2D,
                              n_dirs: int = CERT_DIRECTIONS) -> float:
    """Certified margin of ``union(members) subset candidate``.

    Returns ``min_u [h_cand(u) - h_union(u)]`` reduced by the Lipschitz
    half-step term.  A non-negative return value proves containment at every
    direction, not only the sampled ones.
    """
    U = _unit_directions(n_dirs)
    origin = np.asarray(candidate.mean, dtype=np.float64)
    h_union = union_support_values(members, U, origin)
    h_cand = support_values(candidate, U, origin)
    sampled = float(np.min(h_cand - h_union))
    # Circumradii about the candidate centre bound both Lipschitz constants.
    r_union = max(float(np.linalg.norm(np.asarray(m.mean) - origin)
                        + m.bounding_radius()) for m in members)
    r_cand = float(candidate.bounding_radius())
    half_step = np.pi / float(n_dirs)
    return sampled - (r_union + r_cand) * half_step


def _bounding_disc_support(members, primitive_id: int, level: float,
                           n_dirs: int = CERT_DIRECTIONS) -> GaussianSupport2D:
    """Always-valid fallback: a disc that contains every member ellipse.

    The exact covering disc *touches* the union, so its sampled slack is zero
    and the Lipschitz half-step would push the certificate negative.  Solve for
    the radius that certifies instead: with ``hs = pi / n_dirs`` we need
    ``Rd - R0 >= (R0 + Rd) * hs``, i.e. ``Rd >= R0 (1 + hs) / (1 - hs)``.
    """
    means = np.array([m.mean for m in members], dtype=np.float64)
    center = means.mean(axis=0)
    r0 = max(float(np.linalg.norm(m.mean - center) + m.bounding_radius())
             for m in members)
    hs = np.pi / float(n_dirs)
    radius = r0 * (1.0 + hs) / (1.0 - hs) if hs < 1.0 else 2.0 * r0
    radius = float(np.nextafter(radius * (1.0 + 1e-12), np.inf))
    cov = np.eye(2) * (radius / level) ** 2
    return GaussianSupport2D(mean=center, covariance=cov, level=level,
                             primitive_id=primitive_id)


def outer_ellipse(members, primitive_id: int = 0,
                  n_dirs: int = CERT_DIRECTIONS,
                  seed_dirs: int = SEED_DIRECTIONS) -> MacroEllipse:
    """Build a certified outer ellipse for ``members``.

    ``members`` is a non-empty sequence of :class:`GaussianSupport2D`.
    """
    members = list(members)
    if not members:
        raise ValueError("outer_ellipse needs at least one member")
    levels = {m.level for m in members}
    level = levels.pop() if len(levels) == 1 else 1.0
    if len(members) == 1:
        only = members[0]
        return MacroEllipse(
            GaussianSupport2D(only.mean, only.covariance, only.level,
                              primitive_id),
            slack=float("inf"), excess_area=0.0, n_members=1, fallback=False)

    # Seed cloud: boundary points of every member, which are exactly the
    # extreme points of the union in those directions.
    U_seed = _unit_directions(seed_dirs)
    cloud = []
    for m in members:
        S = shape_matrix(m)
        Su = U_seed @ S.T
        norms = np.sqrt(np.maximum(np.einsum("ki,ki->k", U_seed, Su), 0.0))
        cloud.append(np.asarray(m.mean)[None, :] + Su / norms[:, None])
    cloud = np.vstack(cloud)
    # The minimum-volume enclosing ellipse depends only on the convex hull of
    # the cloud, and a 484-member group seeds ~31k points.  Hulling first cuts
    # the Khachiyan solve to a few dozen points with an identical optimum.
    if len(cloud) > 16:
        try:
            hull = MultiPoint(cloud).convex_hull
            if hull.geom_type == "Polygon":
                cloud = np.asarray(hull.exterior.coords[:-1], dtype=np.float64)
        except Exception:
            pass
    # Work about the cloud centroid: the compiler is sensitive to large world
    # coordinates, and the MVEE solve is much better conditioned locally.
    shift = cloud.mean(axis=0)

    fallback = False
    try:
        center_local, S = _mvee(cloud - shift[None, :])
        candidate = _as_support(center_local + shift, S, level, primitive_id)
    except (np.linalg.LinAlgError, ValueError):
        candidate = _bounding_disc_support(members, primitive_id, level, n_dirs)
        fallback = True

    slack = certify_outer_containment(members, candidate, n_dirs)
    if slack < 0.0:
        # Radial inflation: scaling the shape matrix by (1+gamma)^2 raises the
        # support function by at least gamma * r_min everywhere.
        S_cand = shape_matrix(candidate)
        r_min = float(np.sqrt(max(np.linalg.eigvalsh(S_cand)[0], 0.0)))
        if r_min > 0.0:
            gamma = (-slack) / r_min
            # One extra ulp-scale nudge so the re-certification is not decided
            # by the rounding of the inflation itself.
            gamma = np.nextafter(gamma * (1.0 + 1e-12) + 1e-15, np.inf)
            candidate = _as_support(candidate.mean, (1.0 + gamma) ** 2 * S_cand,
                                    level, primitive_id)
            slack = certify_outer_containment(members, candidate, n_dirs)
    if slack < 0.0:
        candidate = _bounding_disc_support(members, primitive_id, level, n_dirs)
        slack = certify_outer_containment(members, candidate, n_dirs)
        fallback = True
    if slack < 0.0:
        raise RuntimeError(
            "outer macro construction failed to certify containment; "
            "refusing to emit an uncertified macro primitive")

    from shapely.ops import unary_union
    union = unary_union([ellipse_polygon(m, 128, outer=False) for m in members])
    excess = float(ellipse_polygon(candidate, 256, outer=False).area
                   - union.area)
    return MacroEllipse(candidate, float(slack), excess, len(members), fallback)
