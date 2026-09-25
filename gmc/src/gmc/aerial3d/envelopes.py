"""Certified pair envelopes (uav.md §6) on a fixed icosphere direction set.

Outer polytope  P+_i = {x : u.x <= u.mu_i + rho_i(u) + eps  for u in +-U}   (contains O_i)
Inner polytope  P-_i = conv{x_i(u) : u in +-U}, H-rep shrunk by eps_in      (inside O_i)

Box tests used by the octree and the cell builder:

* ``best_gap``: for box B (centre c, half extents d) and direction u the slab
  ``|u.(mu_i - c)| - rho_i(u) - |u|.d`` is the width of empty space between B and
  O_i along u.  A positive value is a support plane of P+_i separating B.
* ``depth``: min over U of ``rho_i(u) + eps - |u.(mu_i - c)| - |u|.d`` >= 0 iff
  B is inside P+_i -- a necessary condition for B inside P-_i.
* ``box_in_inner``: all 8 corners strictly inside the shrunk H-rep of P-_i.
* ``refine_gap``: adaptive spherical-direction refinement (pattern search) for a
  single pair; the refined halfspace is a valid support halfspace of O_i.
"""
from __future__ import annotations

from collections import OrderedDict
from itertools import product

import numpy as np
from scipy.spatial import ConvexHull

from .pairs import PairSet

_CORNERS = np.array(list(product((-1., 1.), repeat=3)))


def _icosphere(level: int, *, with_faces: bool = False):
    phi = (1 + 5 ** .5) / 2
    verts = [(-1, phi, 0), (1, phi, 0), (-1, -phi, 0), (1, -phi, 0), (0, -1, phi), (0, 1, phi),
             (0, -1, -phi), (0, 1, -phi), (phi, 0, -1), (phi, 0, 1), (-phi, 0, -1), (-phi, 0, 1)]
    faces = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4),
             (11, 10, 2), (10, 7, 6), (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8),
             (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    verts = [np.asarray(v, float) / np.linalg.norm(v) for v in verts]
    for _ in range(level):
        cache, new_faces = {}, []

        def mid(a, b):
            key = (min(a, b), max(a, b))
            if key not in cache:
                m = verts[a] + verts[b]
                verts.append(m / np.linalg.norm(m))
                cache[key] = len(verts) - 1
            return cache[key]

        for a, b, c in faces:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            new_faces += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        faces = new_faces
    if with_faces:
        return np.asarray(verts), np.asarray(faces)
    return np.asarray(verts)


def cover_angle(level: int = 2) -> float:
    """Covering radius (radians) of the icosphere vertex set on S^2.

    The icosphere is a spherical Delaunay triangulation of its vertices, so every
    point of the sphere is within the largest spherical-triangle circumradius of a
    vertex.  Rounded up by 1e-9 rad.
    """
    V, faces = _icosphere(level, with_faces=True)
    a, b, c = V[faces[:, 0]], V[faces[:, 1]], V[faces[:, 2]]
    n = np.cross(b - a, c - a)
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    n *= np.sign(np.sum(n * a, axis=1))[:, None]
    return float(np.max(np.arccos(np.clip(np.sum(n * a, axis=1), -1., 1.)))) + 1e-9


def direction_set(level: int = 2) -> np.ndarray:
    """One direction of each antipodal pair of the icosphere; exact axes; binary64 rows.

    Level 2 has 162 vertices (81 antipodal pairs, the +-axes among them). These
    stored rows are exactly the rows every consumer evaluates.
    """
    V = _icosphere(level)
    for e in np.eye(3):  # snap the (already present) axes to exact unit vectors
        hit = np.argmax(V @ e)
        if V[hit] @ e > 1 - 1e-12:
            V[hit] = e
    key = np.where(np.abs(V[:, 2]) > 1e-12, V[:, 2], np.where(np.abs(V[:, 1]) > 1e-12, V[:, 1], V[:, 0]))
    U = V[key > 0]
    U = U / np.linalg.norm(U, axis=1, keepdims=True)
    for e in np.eye(3):
        U[np.all(np.abs(U - e) < 1e-12, axis=1)] = e
    U.flags.writeable = False
    return U


class EnvelopeTable:
    """Per-pair outer support values on U (stored) and lazily built inner hulls."""

    def __init__(self, pairs: PairSet, U: np.ndarray | None = None, *, inner_cache_size: int = 20000,
                 cover: float | None = None):
        self.pairs = pairs
        self.U = direction_set() if U is None else np.asarray(U, float)
        # A custom U without a stated covering radius disables the refinement skip.
        self.cover_angle = cover_angle() if U is None else (np.pi if cover is None else float(cover))
        lam = np.linalg.eigvalsh(pairs.covs)[:, -1] if len(pairs) else np.empty(0)
        self.circumradius = ((pairs.level * np.sqrt(np.maximum(lam, 0.))
                              + np.hypot(pairs.radius, pairs.half_height) + pairs.margin)
                             * (1 + 1e-12) + pairs.slack)
        self.full = np.vstack([self.U, -self.U])
        self.absU = np.abs(self.U)
        self.rho = pairs.rho(self.U)
        self.rho.flags.writeable = False
        self.eps = pairs.slack
        self.eps_inner = 4 * pairs.slack
        self._inner: OrderedDict[int, np.ndarray] = OrderedDict()
        self._inner_cap = int(inner_cache_size)
        self.stats = {"inner_hulls_built": 0, "inner_hull_cache_hits": 0, "refinements": 0,
                      "refined_successes": 0}

    # ------------------------------------------------------------------ outer side
    def outer_rows(self, i: int) -> tuple[np.ndarray, np.ndarray]:
        r = self.rho[i]
        return self.full, self.full @ self.pairs.means[i] + np.r_[r, r] + self.eps

    def _proj(self, index, c):
        return (self.pairs.means[index] - np.asarray(c, float)) @ self.U.T

    def gaps(self, index, c, d) -> np.ndarray:
        return np.abs(self._proj(index, c)) - self.rho[index] - (self.absU @ np.asarray(d, float))[None]

    def best_gap(self, index, c, d):
        """(gap, k, sign): max over +-U of the empty slab between box and O_i."""
        index = np.asarray(index)
        P = self._proj(index, c)
        g = np.abs(P) - self.rho[index] - (self.absU @ np.asarray(d, float))[None]
        k = np.argmax(g, axis=1)
        rows = np.arange(len(index))
        sign = np.where(P[rows, k] >= 0, 1., -1.)
        return g[rows, k], k, sign

    def depth(self, index, c, d) -> np.ndarray:
        return np.min(self.rho[np.asarray(index)] + self.eps - np.abs(self._proj(index, c))
                      - (self.absU @ np.asarray(d, float))[None], axis=1)

    def refine_upper_bound(self, index, c, d, fixed_gap) -> np.ndarray:
        """Upper bound on the gap over *all* unit directions, from the best fixed one.

        g(u) = |u.(mu-c)| - rho(u) - |u|.d is Lipschitz on the sphere with constant
        |mu-c| + R_i + |d| (R_i bounds |x - mu| on O_i), and every unit vector is within
        ``cover_angle`` of +-U.  Refinement cannot succeed where this bound fails.
        """
        index = np.asarray(index)
        lip = (np.linalg.norm(self.pairs.means[index] - np.asarray(c, float), axis=1)
               + self.circumradius[index] + np.linalg.norm(np.asarray(d, float)))
        return np.asarray(fixed_gap, float) + self.cover_angle * lip

    def refine_gap(self, i: int, c, d, u0=None) -> tuple[float, np.ndarray]:
        """Pattern search on the sphere for a better separating direction (one pair).

        Returns (gap, u) with u oriented towards the obstacle (u.(mu - c) >= 0).
        Every candidate is evaluated with the outward-rounded exact support, so the
        resulting halfspace is a valid support halfspace of O_i.
        """
        self.stats["refinements"] += 1
        c, d = np.asarray(c, float), np.asarray(d, float)
        mu = self.pairs.means[i]

        def evaluate(W):
            W = W / np.linalg.norm(W, axis=1, keepdims=True)
            return np.abs(W @ (mu - c)) - self.pairs.rho(W, [i])[0] - np.abs(W) @ d, W

        seeds = [self.U[np.argmax(self.gaps([i], c, d)[0])]]
        toward = mu - c
        if np.linalg.norm(toward) > 0:
            seeds.append(toward)
        if u0 is not None:
            seeds.append(np.asarray(u0, float))
        g, W = evaluate(np.asarray(seeds))
        best = int(np.argmax(g))
        gap, u = float(g[best]), W[best]
        phis = np.linspace(0, 2 * np.pi, 12, endpoint=False)
        for theta in np.radians([8., 4., 2., 1., .5, .25, .12, .06, .03]):
            for _ in range(6):
                if abs(u[0]) < .9:   # a = u x e_x, b = u x a (manual: np.cross is slow here)
                    a = np.array([0., u[2], -u[1]])
                else:
                    a = np.array([-u[2], 0., u[0]])
                a /= np.linalg.norm(a)
                b = np.array([u[1] * a[2] - u[2] * a[1], u[2] * a[0] - u[0] * a[2],
                              u[0] * a[1] - u[1] * a[0]])
                cand = (np.cos(theta) * u[None] + np.sin(theta)
                        * (np.cos(phis)[:, None] * a + np.sin(phis)[:, None] * b))
                g, W = evaluate(cand)
                j = int(np.argmax(g))
                if g[j] > gap:
                    gap, u = float(g[j]), W[j]
                else:
                    break
        if u @ (mu - c) < 0:
            u = -u
        if gap > 0:
            self.stats["refined_successes"] += 1
        return gap, u

    # ------------------------------------------------------------------ inner side
    def inner_equations(self, i: int) -> np.ndarray:
        i = int(i)
        eq = self._inner.get(i)
        if eq is not None:
            self._inner.move_to_end(i)
            self.stats["inner_hull_cache_hits"] += 1
            return eq
        X = self.pairs.support_points(self.full, [i])[0]
        eq = ConvexHull(X).equations  # unit normals: n.x + off <= 0 inside
        eq.flags.writeable = False
        self._inner[i] = eq
        self.stats["inner_hulls_built"] += 1
        if len(self._inner) > self._inner_cap:
            self._inner.popitem(last=False)
        return eq

    def points_in_inner(self, i: int, P) -> np.ndarray:
        eq = self.inner_equations(i)
        P = np.atleast_2d(np.asarray(P, float))
        return np.all(P @ eq[:, :3].T + eq[:, 3][None] <= -self.eps_inner, axis=1)

    def box_in_inner(self, i: int, c, d) -> bool:
        corners = np.asarray(c, float) + _CORNERS * np.asarray(d, float)
        return bool(np.all(self.points_in_inner(i, corners)))


def sandwich_audit(table: EnvelopeTable, indices, *, n_dense: int = 5000, seed: int = 0) -> dict:
    """Replayable nesting audit P- ⊆ O ⊆ P+ (uav.md §6.4) on the given pairs.

    Checks, minus the declared floating-point tolerance (a positive value is a
    violation): inner-hull vertices satisfy every dense exact support inequality;
    exact support points in dense directions satisfy every outer row; inner-hull
    vertices satisfy every outer row.
    """
    rng = np.random.default_rng(seed)
    V = rng.normal(size=(n_dense, 3))
    V /= np.linalg.norm(V, axis=1, keepdims=True)
    pairs = table.pairs
    worst = {"inner_vertices_outside_exact_max": -np.inf, "exact_points_outside_outer_max": -np.inf,
             "inner_vertices_outside_outer_max": -np.inf}
    count = 0
    for i in indices:
        i = int(i)
        count += 1
        X_in = pairs.support_points(table.full, [i])[0]
        hv = pairs.support(V, [i])[0]
        worst["inner_vertices_outside_exact_max"] = max(
            worst["inner_vertices_outside_exact_max"], float(np.max(X_in @ V.T - hv[None])) - table.eps)
        A, b = table.outer_rows(i)
        X_ex = pairs.support_points(V, [i])[0]
        worst["exact_points_outside_outer_max"] = max(
            worst["exact_points_outside_outer_max"], float(np.max(X_ex @ A.T - b[None])))
        worst["inner_vertices_outside_outer_max"] = max(
            worst["inner_vertices_outside_outer_max"], float(np.max(X_in @ A.T - b[None])))
    out = {k: float(v) for k, v in worst.items()}
    out.update(pairs_audited=count, dense_directions=n_dense, seed=seed,
               tolerance_m=float(table.eps),
               passed=bool(count and all(v <= 0 for v in worst.values())),
               note="sampled nesting audit with declared binary64 slack; not an exact-arithmetic proof")
    return out
