"""Support-plane convex free cells (IRIS-style, one greedy pass) and certified portals.

A cell grown from a certified-free seed box S is ``D ∩ {n_r . x <= beta_r}`` where
each pair row is a support plane of the outer polytope of one pair, pushed back by
the buffer:  ``beta = n.mu_i - rho_i(n) - buffer = min_{O_i} n.x - buffer`` with n
the best separating direction between S and O_i.  Pairs are processed nearest
first; a pair is dropped once one earlier plane already keeps all of O_i at least
``buffer`` outside.  So every point of the cell is at distance > buffer from every
O_i (hence free), and every facet is a pair support plane (pair index, id,
direction index or adaptive, outer side) or a domain face.

SAFE leaves, largest first, seed cells until each SAFE leaf lies inside some cell
(or the cell budget ends: the rest become *box cells* = the leaf itself, certified
by its SAFE label, with box-face facets).  Portals join two cells through a SAFE
leaf contained in both (3-D portal) or through the shared face of two
face-adjacent SAFE leaves, one in each cell (2-D portal); both lie inside both
cells.  Hence the cell graph is at least as connected as the SAFE-leaf face graph.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from time import perf_counter

import numpy as np
from scipy.optimize import linprog
from scipy.spatial import cKDTree

from .envelopes import EnvelopeTable
from .graph import face_pairs, shared_face, touch_pairs
from .octree import SAFE, Octree
from .pairs import Domain


@dataclass(frozen=True)
class CellConfig:
    max_cells: int = 5000
    max_planes_per_cell: int = 20000
    max_wall_s: float | None = None   # stop growing support-plane cells after this; rest -> box cells
    gap_chunk: int = 32768
    portal_samples: int = 8           # portal regions kept per cell pair (spread over bins)
    portal_bin_m: float = .5          # spatial bin for spreading them

    def __post_init__(self):
        if (self.max_cells < 0 or self.max_planes_per_cell < 1 or self.gap_chunk < 1
                or self.portal_samples < 1 or not self.portal_bin_m > 0):
            raise ValueError("invalid cell configuration")


@dataclass
class ConvexCell:
    A: np.ndarray
    b: np.ndarray
    row_kind: np.ndarray          # 'domain' | 'pair' | 'box'
    row_pair: np.ndarray          # pair index or -1
    row_pair_id: np.ndarray       # scene Gaussian id or -1
    row_dir: np.ndarray           # index into U (sign in row_sign), or -1 for adaptive / non-pair
    row_sign: np.ndarray
    kind: str                     # 'support_plane' | 'box' | 'query'
    seed_leaf: int
    bbox_lower: np.ndarray = field(default=None)
    bbox_upper: np.ndarray = field(default=None)

    def contains_points(self, P, tol: float = 0.) -> np.ndarray:
        P = np.atleast_2d(np.asarray(P, float))
        return np.all(P @ self.A.T <= self.b[None] + tol, axis=1)

    def contains_boxes(self, C, D, tol: float = 0.) -> np.ndarray:
        C, D = np.atleast_2d(np.asarray(C, float)), np.atleast_2d(np.asarray(D, float))
        return np.all(C @ self.A.T + D @ np.abs(self.A).T <= self.b[None] + tol, axis=1)

    def summary(self) -> dict:
        kinds = {k: int(np.count_nonzero(self.row_kind == k)) for k in ("domain", "pair", "box")}
        return {"kind": self.kind, "seed_leaf": int(self.seed_leaf), "facets": int(len(self.b)),
                "facets_by_kind": kinds,
                "pair_ids": sorted({int(i) for i in self.row_pair_id if i >= 0})[:64],
                "bbox_lower": self.bbox_lower.tolist(), "bbox_upper": self.bbox_upper.tolist()}


def _bbox(A, b):
    lower, upper = np.empty(3), np.empty(3)
    for k in range(3):
        for sign, out in ((1., lower), (-1., upper)):
            c = np.zeros(3)
            c[k] = sign
            res = linprog(c, A_ub=A, b_ub=b, bounds=[(None, None)] * 3, method="highs")
            if res.status != 0:
                raise ValueError(f"cell bbox LP failed: {res.message}")
            out[k] = res.x[k]
    return lower, upper


def box_cell(centre, half, seed_leaf: int, kind: str = "box") -> ConvexCell:
    c, d = np.asarray(centre, float), np.asarray(half, float)
    A = np.vstack([np.eye(3), -np.eye(3)])
    b = np.r_[c + d, -(c - d)]
    m = len(b)
    return ConvexCell(A, b, np.array(["box"] * m), np.full(m, -1), np.full(m, -1), np.full(m, -1),
                      np.zeros(m), kind, int(seed_leaf), c - d, c + d)


def grow_cell(table: EnvelopeTable, domain: Domain, centre, half, *, buffer_m: float,
              seed_leaf: int = -1, kind: str = "support_plane", max_planes: int = 20000,
              chunk: int = 32768, stats: dict | None = None) -> ConvexCell | None:
    """Greedy support-plane cell around the seed box; None if the seed is not certifiable."""
    pairs = table.pairs
    c, d = np.asarray(centre, float), np.asarray(half, float)
    n_pairs = len(pairs)
    threshold = buffer_m + pairs.slack
    rows_A, rows_b = [a for a in domain.A], [float(v) for v in domain.b]
    kinds = ["domain"] * len(rows_b)
    rpair, rid, rdir, rsign = [-1] * len(rows_b), [-1] * len(rows_b), [-1] * len(rows_b), [0.] * len(rows_b)
    if domain.box_status(c, d) != "inside":
        return None
    if n_pairs:
        gaps = np.empty(n_pairs)
        ks = np.empty(n_pairs, dtype=np.int64)
        signs = np.empty(n_pairs)
        for s in range(0, n_pairs, chunk):
            g, k, sg = table.best_gap(np.arange(s, min(s + chunk, n_pairs)), c, d)
            gaps[s:s + len(g)], ks[s:s + len(g)], signs[s:s + len(g)] = g, k, sg
        adaptive = {}
        for i in np.flatnonzero(gaps <= threshold):
            rg, u = table.refine_gap(int(i), c, d)
            if rg <= threshold:
                return None
            adaptive[int(i)] = u
            gaps[i] = rg
        order = np.argsort(gaps, kind="stable")
        alive = np.ones(n_pairs, dtype=bool)
        alive_idx = np.arange(n_pairs)
        planes = 0
        for i in order:
            if not alive[i]:
                continue
            planes += 1
            if planes > max_planes:
                return None
            if int(i) in adaptive:
                n = adaptive[int(i)]
                rho_alive = pairs.rho(n[None], alive_idx)[:, 0]
                rho_i = pairs.rho(n[None], [i])[0, 0]
                kdir = -1
            else:
                kdir = int(ks[i])
                n = signs[i] * table.U[kdir]
                rho_alive = table.rho[alive_idx, kdir]
                rho_i = table.rho[i, kdir]
            beta = float(n @ pairs.means[i] - rho_i - buffer_m)
            rows_A.append(n); rows_b.append(beta); kinds.append("pair")
            rpair.append(int(i)); rid.append(int(pairs.ids[i])); rdir.append(kdir)
            rsign.append(float(signs[i]) if kdir >= 0 else 0.)
            low = pairs.means[alive_idx] @ n - rho_alive
            dead = low > beta + buffer_m + pairs.slack
            alive[alive_idx[dead]] = False
            alive[i] = False
            alive_idx = alive_idx[alive[alive_idx]]
        if stats is not None:
            stats["planes"] = stats.get("planes", 0) + planes
            stats["adaptive_seed_directions"] = stats.get("adaptive_seed_directions", 0) + len(adaptive)
    A, b = np.asarray(rows_A, float), np.asarray(rows_b, float)
    lower, upper = _bbox(A, b)
    return ConvexCell(A, b, np.asarray(kinds), np.asarray(rpair, dtype=np.int64),
                      np.asarray(rid, dtype=np.int64), np.asarray(rdir, dtype=np.int64),
                      np.asarray(rsign), kind, int(seed_leaf), lower, upper)


class CellComplex:
    """Cells, leaf membership and portals of the pair-certified support-plane cell complex."""

    def __init__(self):
        self.cells: list[ConvexCell] = []
        self.leaf_cells: dict[int, list[int]] = {}
        self.portal_cells = np.empty((0, 2), dtype=np.int64)
        self.portal_lower = np.empty((0, 3))
        self.portal_upper = np.empty((0, 3))
        self.portal_kind = np.empty(0, dtype="<U4")
        self.portal_leaves = np.empty((0, 2), dtype=np.int64)
        self.stats: dict = {}

    @classmethod
    def build(cls, tree: Octree, table: EnvelopeTable, domain: Domain,
              config: CellConfig = CellConfig(), *, check=None) -> "CellComplex":
        started = perf_counter()
        self = cls()
        buffer_m = tree.config.buffer_m
        safe = np.flatnonzero(tree.status == SAFE)
        centres, halves = tree.boxes(safe)
        order = np.lexsort((safe, -tree.size[safe]))
        kd = cKDTree(centres) if len(safe) else None
        max_half = float(halves.max()) if len(safe) else 0.
        covered = np.zeros(len(safe), dtype=bool)
        grow_stats = {"planes": 0, "adaptive_seed_directions": 0, "seed_failures": 0}
        members: list[list[int]] = [[] for _ in range(len(safe))]
        grown = 0
        for j in order:
            if covered[j]:
                continue
            if grown >= config.max_cells:
                break
            if config.max_wall_s is not None and perf_counter() - started > config.max_wall_s:
                break
            if check is not None:
                check()
            cell = grow_cell(table, domain, centres[j], halves[j], buffer_m=buffer_m,
                             seed_leaf=int(safe[j]), max_planes=config.max_planes_per_cell,
                             chunk=config.gap_chunk, stats=grow_stats)
            if cell is None:
                grow_stats["seed_failures"] += 1
                cell = box_cell(centres[j], halves[j], int(safe[j]))
                self.cells.append(cell)
                members[j].append(len(self.cells) - 1)
                covered[j] = True
                continue
            cid = len(self.cells)
            self.cells.append(cell)
            grown += 1
            mid = (cell.bbox_lower + cell.bbox_upper) / 2
            radius = float(np.max(cell.bbox_upper - cell.bbox_lower)) / 2 + max_half + 1e-9
            near = np.asarray(kd.query_ball_point(mid, radius, p=np.inf), dtype=np.int64)
            if len(near):
                inside = near[cell.contains_boxes(centres[near], halves[near])]
                for jj in inside:
                    members[jj].append(cid)
                covered[inside] = True
            if not covered[j]:
                raise RuntimeError("seed leaf not inside its own cell (numerical failure)")
        support_cells = len(self.cells)
        for j in np.flatnonzero(~covered):
            self.cells.append(box_cell(centres[j], halves[j], int(safe[j])))
            members[j].append(len(self.cells) - 1)
        self.leaf_cells = {int(safe[j]): members[j] for j in range(len(safe))}
        t_cells = perf_counter() - started
        self._portals(tree, safe, config)
        self.stats = {"cells": len(self.cells),
                      "support_plane_cells": sum(c.kind == "support_plane" for c in self.cells),
                      "box_cells": sum(c.kind == "box" for c in self.cells),
                      "seeded_attempts": support_cells, **grow_stats,
                      "portals": int(len(self.portal_cells)),
                      "cell_pairs_joined": int(self.cell_pairs_joined),
                      "portals_3d": int(np.count_nonzero(self.portal_kind == "leaf")),
                      "portals_2d": int(np.count_nonzero(self.portal_kind == "face")),
                      "cell_growth_wall_s": t_cells, "build_wall_s": perf_counter() - started,
                      "config": asdict(config)}
        return self

    def _portals(self, tree: Octree, safe: np.ndarray, config: CellConfig):
        """Per cell pair, the best region (3-D shared leaf over 2-D shared face, then larger)
        in each ``portal_bin_m`` bin; the ``portal_samples`` best bins are kept as portals."""
        best: dict[tuple, tuple] = {}
        bin_m = config.portal_bin_m

        def offer(a, b, dim, measure, lower, upper, leaves):
            centre = (lower + upper) / 2
            key = (min(a, b), max(a, b), *np.floor(centre / bin_m).astype(np.int64).tolist())
            cur = best.get(key)
            if cur is None or (dim, measure) > (cur[0], cur[1]):
                best[key] = (dim, measure, lower, upper, leaves)

        for leaf, cids in self.leaf_cells.items():
            if len(cids) > 1:
                c, d = tree.box(leaf)
                vol = float(np.prod(2 * d))
                for x in range(len(cids)):
                    for y in range(x + 1, len(cids)):
                        offer(cids[x], cids[y], 3, vol, c - d, c + d, (leaf, leaf))
        fp, _ = face_pairs(tree, touch_pairs(tree, safe))
        for a_leaf, b_leaf in fp:
            ca, cb = self.leaf_cells[int(a_leaf)], self.leaf_cells[int(b_leaf)]
            lower = upper = None
            for x in ca:
                for y in cb:
                    if x == y:
                        continue
                    if lower is None:
                        lower, upper = shared_face(tree, int(a_leaf), int(b_leaf), 0)
                        area = float(np.prod(np.sort(upper - lower)[1:]))
                    offer(x, y, 2, area, lower, upper, (int(a_leaf), int(b_leaf)))
        per_pair: dict[tuple, list] = {}
        for key, val in best.items():
            per_pair.setdefault(key[:2], []).append(val)
        rows = []
        for pair in sorted(per_pair):
            vals = sorted(per_pair[pair], key=lambda v: (v[0], v[1]), reverse=True)
            rows += [(pair, v) for v in vals[:config.portal_samples]]
        self.portal_cells = np.asarray([p for p, _ in rows], dtype=np.int64).reshape(-1, 2)
        self.portal_lower = np.asarray([v[2] for _, v in rows], float).reshape(-1, 3)
        self.portal_upper = np.asarray([v[3] for _, v in rows], float).reshape(-1, 3)
        self.portal_kind = np.asarray(["leaf" if v[0] == 3 else "face" for _, v in rows], dtype="<U4")
        self.portal_leaves = np.asarray([v[4] for _, v in rows], dtype=np.int64).reshape(-1, 2)
        self.cell_pairs_joined = len(per_pair)
        self.cell_portals: list[list[int]] = [[] for _ in self.cells]
        for p, (a, b) in enumerate(self.portal_cells):
            self.cell_portals[a].append(p)
            self.cell_portals[b].append(p)

    def traceability(self) -> dict:
        kinds = np.concatenate([c.row_kind for c in self.cells if c.kind == "support_plane"]) \
            if any(c.kind == "support_plane" for c in self.cells) else np.empty(0, dtype="<U6")
        n = len(kinds)
        box_rows = sum(len(c.b) for c in self.cells if c.kind == "box")
        known = {"domain", "pair", "box"}
        missing = sum(int(np.count_nonzero(~np.isin(c.row_kind, list(known)))) for c in self.cells)
        return {"support_plane_cell_facets": int(n),
                "pair_facet_fraction_of_support_plane_cells": float(np.mean(kinds == "pair")) if n else 0.,
                "domain_facet_fraction_of_support_plane_cells": float(np.mean(kinds == "domain")) if n else 0.,
                "box_cell_facets": int(box_rows), "facets_without_provenance": int(missing),
                "note": "pair facets are support planes of outer pair polytopes; domain facets are the "
                        "workspace/known-space rows; box facets bound fallback box cells, whose freedom "
                        "is certified by the leaf's pair separation"}
