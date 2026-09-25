"""Query-time routing on the cell complex: locate, portal A*, lifting, shortening, shortcut.

Lifting (uav.md §10.1): the graph path  start -> portal_1 -> ... -> portal_k -> goal
alternates cells X_0..X_k; consecutive points lie in one convex cell, so every
segment lies in that cell (certified free by its support planes).  Shortening moves
each portal point inside its portal box (a box contained in both adjacent cells),
which keeps that property.  Shortcuts are accepted only through the direct verifier.
"""
from __future__ import annotations

import heapq

import numpy as np
from scipy.optimize import linprog, minimize

from .cells import CellComplex, ConvexCell


def cells_containing(cells: list[ConvexCell], point, tol: float = 0.) -> list[int]:
    p = np.asarray(point, float)
    out = []
    for i, c in enumerate(cells):
        if np.all(c.bbox_lower - 1e-9 <= p) and np.all(p <= c.bbox_upper + 1e-9) \
                and np.all(c.A @ p <= c.b + tol):
            out.append(i)
    return out


def chebyshev_portal(a: ConvexCell, b: ConvexCell):
    """Largest ball inside both cells (LP); returns (centre, radius) or None."""
    A = np.vstack([a.A, b.A])
    rhs = np.r_[a.b, b.b]
    norms = np.linalg.norm(A, axis=1)
    res = linprog(np.r_[0., 0., 0., -1.], A_ub=np.c_[A, norms], b_ub=rhs,
                  bounds=[(None, None)] * 3 + [(0., 1.)], method="highs")
    if res.status != 0 or res.x[3] <= 1e-9:
        return None
    return res.x[:3], float(res.x[3])


class PortalGraph:
    """Portals of a complex plus per-query extra cells/portals (never mutates the complex)."""

    def __init__(self, cx: CellComplex):
        self.cx = cx
        self.cells = list(cx.cells)
        self.cell_portals = [list(p) for p in cx.cell_portals]
        self.p_cells = [tuple(map(int, x)) for x in cx.portal_cells]
        self.p_lower = [x for x in cx.portal_lower]
        self.p_upper = [x for x in cx.portal_upper]
        self.p_kind = list(cx.portal_kind)
        self.lp_portals = 0

    def add_cell(self, cell: ConvexCell, *, max_lp: int = 400) -> int:
        cid = len(self.cells)
        self.cells.append(cell)
        self.cell_portals.append([])
        cand = [i for i, c in enumerate(self.cells[:-1])
                if np.all(c.bbox_lower <= cell.bbox_upper) and np.all(cell.bbox_lower <= c.bbox_upper)]
        cand.sort(key=lambda i: np.linalg.norm((self.cells[i].bbox_lower + self.cells[i].bbox_upper) / 2
                                              - (cell.bbox_lower + cell.bbox_upper) / 2))
        for i in cand[:max_lp]:
            hit = chebyshev_portal(cell, self.cells[i])
            if hit is None:
                continue
            centre, r = hit
            half = np.full(3, .999 * r / np.sqrt(3.))  # cube inscribed in the common ball
            self._add_portal(cid, i, centre - half, centre + half, "lp")
            self.lp_portals += 1
        return cid

    def _add_portal(self, a, b, lower, upper, kind):
        p = len(self.p_cells)
        self.p_cells.append((a, b))
        self.p_lower.append(np.asarray(lower, float))
        self.p_upper.append(np.asarray(upper, float))
        self.p_kind.append(kind)
        self.cell_portals[a].append(p)
        self.cell_portals[b].append(p)

    def search(self, start, goal, start_cells, goal_cells):
        """A* over portal points; returns (portal ids, cell ids) or None."""
        n = len(self.p_cells)
        if not n and not (set(start_cells) & set(goal_cells)):
            return None
        centres = (np.asarray(self.p_lower).reshape(-1, 3) + np.asarray(self.p_upper).reshape(-1, 3)) / 2
        start, goal = np.asarray(start, float), np.asarray(goal, float)
        common = set(start_cells) & set(goal_cells)
        if common:
            return [], [min(common)]
        START, GOAL = n, n + 1
        pos = np.vstack([centres, start, goal]) if n else np.vstack([start, goal])
        goal_set = set(goal_cells)
        dist = np.full(n + 2, np.inf)
        parent = np.full(n + 2, -1)
        via = np.full(n + 2, -1)
        dist[START] = 0.
        heur = np.linalg.norm(pos - goal, axis=1)
        heap = [(heur[START], 0., START)]
        closed = np.zeros(n + 2, dtype=bool)
        cell_arr = [np.asarray(p, dtype=np.int64) for p in self.cell_portals]
        while heap:
            _, d, node = heapq.heappop(heap)
            if closed[node]:
                continue
            closed[node] = True
            if node == GOAL:
                break
            here = pos[node]
            cells_here = list(start_cells) if node == START else list(self.p_cells[node])
            for cell in cells_here:
                nb = cell_arr[cell]
                if len(nb):
                    nd = d + np.linalg.norm(centres[nb] - here, axis=1)
                    better = nd < dist[nb]
                    for j, v in zip(nb[better], nd[better]):
                        if not closed[j]:
                            dist[j], parent[j], via[j] = v, node, cell
                            heapq.heappush(heap, (v + heur[j], v, int(j)))
                if cell in goal_set:
                    v = d + float(np.linalg.norm(goal - here))
                    if v < dist[GOAL]:
                        dist[GOAL], parent[GOAL], via[GOAL] = v, node, cell
                        heapq.heappush(heap, (v, v, GOAL))
        if not np.isfinite(dist[GOAL]):
            return None
        portals, cells = [], []
        node = GOAL
        while node != START:
            cells.append(int(via[node]))
            node = int(parent[node])
            if node != START:
                portals.append(node)
        return portals[::-1], cells[::-1]


def shorten(start, goal, lowers, uppers, *, max_iter: int = 500) -> np.ndarray:
    """Minimise polyline length with each interior point inside its (closed) portal box."""
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    k = len(lowers)
    if k == 0:
        return np.vstack([start, goal])
    lo, hi = np.asarray(lowers, float), np.asarray(uppers, float)
    x0 = ((lo + hi) / 2).ravel()

    def f(x):
        P = np.vstack([start, x.reshape(k, 3), goal])
        d = np.diff(P, axis=0)
        L = np.sqrt(np.sum(d * d, axis=1) + 1e-18)
        g = d / L[:, None]
        grad = g[:-1] - g[1:]
        return float(L.sum()), grad.ravel()

    res = minimize(f, x0, jac=True, method="L-BFGS-B",
                   bounds=list(zip(lo.ravel(), hi.ravel())), options={"maxiter": max_iter})
    X = np.clip(res.x.reshape(k, 3), lo, hi)
    return np.vstack([start, X, goal])


def shorten_in_cells(start, goal, cells: list[ConvexCell], init: np.ndarray, *,
                     max_iter: int = 200) -> np.ndarray | None:
    """Shortest polyline through a fixed cell sequence X_0..X_k (GCS-style convex program).

    Interior point p_j must lie in X_{j-1} ∩ X_j, so segment p_j -> p_{j+1} lies in X_j.
    Solved with SLSQP from the portal-box solution ``init`` (feasible); returns None if
    the solver result is not verified inside every required cell.
    """
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    init = np.asarray(init, float)
    k = len(init) - 2
    if k <= 0:
        return init
    rows_A, rows_b, cols = [], [], []
    for j in range(k):
        for cell in (cells[j], cells[j + 1]):
            rows_A.append(cell.A); rows_b.append(cell.b); cols.append(np.full(len(cell.b), j))
    A, b, col = np.vstack(rows_A), np.concatenate(rows_b), np.concatenate(cols)
    G = np.zeros((len(b), 3 * k))
    for d in range(3):
        G[np.arange(len(b)), 3 * col + d] = A[:, d]

    def f(x):
        P = np.vstack([start, x.reshape(k, 3), goal])
        dd = np.diff(P, axis=0)
        L = np.sqrt(np.sum(dd * dd, axis=1) + 1e-18)
        g = dd / L[:, None]
        return float(L.sum()), (g[:-1] - g[1:]).ravel()

    x0 = init[1:-1].ravel()
    try:
        res = minimize(f, x0, jac=True, method="SLSQP",
                       constraints=[{"type": "ineq", "fun": lambda x: b - G @ x, "jac": lambda x: -G}],
                       options={"maxiter": max_iter, "ftol": 1e-10})
    except (ValueError, np.linalg.LinAlgError):
        return None
    X = res.x.reshape(k, 3)
    P = np.vstack([start, X, goal])
    if not np.all(G @ res.x <= b + 1e-12):
        return None
    if f(res.x)[0] > f(x0)[0]:
        return None
    return P


def simplify(points, tol: float = 1e-9) -> np.ndarray:
    """Drop repeated points and points on the straight segment between their neighbours."""
    P = [np.asarray(points[0], float)]
    for p in np.asarray(points, float)[1:]:
        if np.linalg.norm(p - P[-1]) > tol:
            P.append(p)
    changed = True
    while changed and len(P) > 2:
        changed = False
        for i in range(1, len(P) - 1):
            a, b, c = P[i - 1], P[i], P[i + 1]
            ab, ac = b - a, c - a
            t = float(ab @ ac) / float(ac @ ac)
            if 0 <= t <= 1 and np.linalg.norm(ab - t * ac) <= tol:
                del P[i]
                changed = True
                break
    return np.asarray(P)


def shortcut(points, accept, *, max_attempts: int = 400) -> tuple[np.ndarray, int]:
    """Greedy farthest-first shortcut; ``accept(a, b)`` must certify the new segment."""
    P = np.asarray(points, float)
    out, i, attempts = [P[0]], 0, 0
    while i < len(P) - 1:
        j_ok = i + 1
        for j in range(len(P) - 1, i + 1, -1):
            if attempts >= max_attempts:
                break
            attempts += 1
            if accept(P[i], P[j]):
                j_ok = j
                break
        out.append(P[j_ok])
        i = j_ok
    return np.asarray(out), attempts


def tighten(points, accept, *, rounds: int = 3, sweeps: int = 12, min_step_m: float = 2e-3,
            max_checks: int = 3000) -> tuple[np.ndarray, int]:
    """Certified taut-string tightening (uav.md §10.3: every modification re-verified).

    Each interior vertex moves toward the closest point of its neighbours' chord (full
    step, then halving); a move is kept only if ``accept`` certifies both new segments.
    Between rounds every segment is split at its midpoint so the string can bend around
    corners; collinear points are dropped at the end.  Moving a vertex toward the chord
    never lengthens the polyline.
    """
    P = [np.asarray(p, float) for p in points]
    checks = 0
    for rnd in range(rounds + 1):
        for _ in range(sweeps):
            moved = False
            for j in range(1, len(P) - 1):
                a, b, c = P[j - 1], P[j], P[j + 1]
                ac = c - a
                t = float(np.clip((b - a) @ ac / max(float(ac @ ac), 1e-300), 0., 1.))
                target = a + t * ac
                step = target - b
                alpha = 1.
                while np.linalg.norm(alpha * step) >= min_step_m and checks < max_checks:
                    cand = b + alpha * step
                    checks += 2
                    if accept(a, cand) and accept(cand, c):
                        P[j] = cand
                        moved = True
                        break
                    alpha *= .5
            if not moved or checks >= max_checks:
                break
        if rnd < rounds:
            Q = [P[0]]
            for a, b in zip(P[:-1], P[1:]):
                if np.linalg.norm(b - a) > 4 * min_step_m:
                    Q.append((a + b) / 2)
                Q.append(b)
            P = Q
    return simplify(np.asarray(P), tol=1e-7), checks


def merge_corners(points, accept, *, max_increase_m: float, max_checks: int = 2000,
                  line_gap_m: float = 1e-4) -> tuple[np.ndarray, int]:
    """Replace a run of vertices wrapping a corner by the intersection of its end tangents.

    For interior run P[i..j], the lines P[i-1]->P[i] (continued forward) and P[j+1]->P[j]
    (continued backward) meet at x (closest points within ``line_gap_m`` for skew lines);
    P[i..j] becomes the single vertex x if ``accept`` certifies both new segments and the
    total length grows by no more than the remaining ``max_increase_m``.  Fewer, longer
    segments suit stop-at-anchor smoothers; every change is re-certified (§10.3).
    """
    P = [np.asarray(p, float) for p in points]
    budget, checks = float(max_increase_m), 0
    length = lambda X: float(sum(np.linalg.norm(b - a) for a, b in zip(X[:-1], X[1:])))
    changed = True
    while changed and checks < max_checks:
        changed = False
        for i in range(1, len(P) - 1):
            for j in range(len(P) - 2, i, -1):
                d1, d2 = P[i] - P[i - 1], P[j + 1] - P[j]
                M = np.column_stack([d1, d2])
                if np.linalg.matrix_rank(M, tol=1e-12) < 2:
                    continue
                (s, t), *_ = np.linalg.lstsq(M, P[j] - P[i], rcond=None)
                if s < 0 or t < 0:
                    continue
                x1, x2 = P[i] + s * d1, P[j] - t * d2
                if np.linalg.norm(x1 - x2) > line_gap_m:
                    continue
                x = (x1 + x2) / 2
                Q = P[:i] + [x] + P[j + 1:]
                delta = length(Q) - length(P)
                if delta > budget:
                    continue
                checks += 2
                if accept(P[i - 1], x) and accept(x, P[j + 1]):
                    P, budget, changed = Q, budget - max(delta, 0.), True
                    break
                if checks >= max_checks:
                    break
            if changed or checks >= max_checks:
                break
    return np.asarray(P), checks
