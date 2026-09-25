"""Pair-driven adaptive octree over the C-space domain (uav.md §13.3 conditions).

Leaves are closed boxes of body-centre space labelled only by pair certificates:

* SAFE    box inside the domain and, for every candidate pair, strictly beyond a
          support plane of the outer polytope P+_i by more than ``buffer + slack``
          (fixed icosphere directions, then adaptive refinement for a few pairs);
* BLOCKED box inside the inner polytope P-_i of one named pair (so inside O_i);
* OUTSIDE one domain row is violated on the whole box (no configuration of D);
* UNKNOWN minimum-size box that is none of the above; stores the unseparated pair
          indices (and whether the domain boundary crosses it).

Candidates are inherited from the parent and filtered by the exact C-obstacle
AABB inflated by the buffer, so a pair that is not a candidate is farther than
the buffer from the box along an axis.  Nothing is sampled: every label holds
for the whole closed box, and is replayable from the pair set.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from time import perf_counter
from typing import Callable

import numpy as np
from scipy.spatial import cKDTree

from .envelopes import EnvelopeTable
from .pairs import Domain

OUTSIDE, SAFE, BLOCKED, UNKNOWN = 0, 1, 2, 3
STATUS_NAMES = {OUTSIDE: "OUTSIDE", SAFE: "SAFE", BLOCKED: "BLOCKED", UNKNOWN: "UNKNOWN"}
_CHILD = np.array([[i, j, k] for i in (0, 1) for j in (0, 1) for k in (0, 1)], dtype=np.int64)


@dataclass(frozen=True)
class OctreeConfig:
    min_cell_m: float = .05
    root_levels: int = 3          # root box edge = min_cell_m * 2**root_levels
    buffer_m: float = 1e-3        # SAFE needs separation > buffer (robust vs. the replay oracle)
    max_refine_pairs: int = 8     # adaptive direction refinement per node if at most this many fail
    blocked_candidates: int = 4   # inner-hull containment tests per node
    max_nodes: int = 20_000_000

    def __post_init__(self):
        if (not np.isfinite([self.min_cell_m, self.buffer_m]).all() or self.min_cell_m <= 0
                or self.buffer_m < 0 or self.root_levels < 0 or self.root_levels > 12
                or self.max_refine_pairs < 0 or self.blocked_candidates < 0 or self.max_nodes < 1):
            raise ValueError("invalid octree configuration")


class Octree:
    def __init__(self, table: EnvelopeTable, domain: Domain, config: OctreeConfig = OctreeConfig(),
                 *, check: Callable[[], None] | None = None):
        started = perf_counter()
        self.table, self.domain, self.config = table, domain, config
        self.root_units = 2 ** config.root_levels
        target_root = float(config.min_cell_m) * self.root_units
        # The grid tiles the domain bbox inset by 2 tol, so boxes on an axis-aligned domain
        # face are strictly inside it.  The uncovered sliver (width 2 tol) is thinner than the
        # inner-hull shrink eps_inner: a sliver point next to a BLOCKED box is still inside that
        # pair's inner polytope, so the possible-side cut argument extends over the sliver.
        inset = 2 * domain.tol
        if inset >= table.eps_inner:
            raise ValueError("domain tolerance too large for the sliver argument")
        self.origin = np.asarray(domain.bbox_lower, float) + inset
        extent = np.asarray(domain.bbox_upper, float) - inset - self.origin
        if np.any(extent <= 0):
            raise ValueError("empty domain")
        shape = np.maximum(1, np.ceil(extent / target_root - 1e-12)).astype(np.int64)
        self.root_shape = tuple(int(v) for v in shape)
        self.unit = extent / (shape * self.root_units)   # per-axis leaf edge, <= min_cell_m
        self.root_m = self.unit * self.root_units
        self.inset_m = inset
        self.stats = {"nodes": 0, "refined_nodes": 0, "refine_pair_attempts": 0,
                      "refine_pair_successes": 0, "refine_skipped_by_bound": 0,
                      "inner_tests": 0, "max_candidates": 0,
                      "root_assignments": 0}
        self._build(check)
        self._kd = None
        self.stats.update(self.summary())
        self.stats["build_wall_s"] = perf_counter() - started

    # ------------------------------------------------------------------ build
    def _root_candidates(self):
        pairs = self.table.pairs
        pad = self.config.buffer_m + pairs.slack
        shape = np.asarray(self.root_shape)
        lo = np.floor((pairs.aabb_lower - pad - self.origin) / self.root_m).astype(np.int64)
        hi = np.floor((pairs.aabb_upper + pad - self.origin) / self.root_m).astype(np.int64)
        ok = np.all(hi >= 0, axis=1) & np.all(lo < shape, axis=1)
        idx = np.flatnonzero(ok)
        lo, hi = np.maximum(lo[idx], 0), np.minimum(hi[idx], shape - 1)
        spans = hi - lo + 1
        counts = np.prod(spans, axis=1)
        total = int(counts.sum())
        self.stats["root_assignments"] = total
        pair_rep = np.repeat(idx, counts)
        local = np.arange(total) - np.repeat(np.cumsum(counts) - counts, counts)
        sx, sy = np.repeat(spans[:, 0], counts), np.repeat(spans[:, 1], counts)
        off = np.stack([local % sx, (local // sx) % sy, local // (sx * sy)], axis=1)
        cell = np.repeat(lo, counts, axis=0) + off
        lin = np.ravel_multi_index(tuple(cell.T), self.root_shape)
        order = np.argsort(lin, kind="stable")
        lin, pair_rep = lin[order], pair_rep[order]
        bounds = np.searchsorted(lin, np.arange(int(np.prod(self.root_shape)) + 1))
        return [pair_rep[bounds[r]:bounds[r + 1]] for r in range(int(np.prod(self.root_shape)))]

    def _build(self, check):
        cfg, table, dom = self.config, self.table, self.domain
        pairs = table.pairs
        pad = cfg.buffer_m + pairs.slack
        threshold = cfg.buffer_m + pairs.slack
        roots = self._root_candidates()
        L_lo, L_size, L_status, L_block, L_ncand, L_gap, L_gpair, L_partial = ([] for _ in range(8))
        self.unknown_pairs: dict[int, np.ndarray] = {}
        self.unknown_pair_counts: dict[int, int] = {}
        self.adaptive_witnesses: dict[int, list] = {}
        stack = []
        for r in reversed(range(len(roots))):
            ijk = np.array(np.unravel_index(r, self.root_shape), dtype=np.int64)
            stack.append((ijk * self.root_units, self.root_units, roots[r]))

        def leaf(lo, size, status, block=-1, ncand=0, gap=np.inf, gpair=-1, partial=False):
            L_lo.append(lo); L_size.append(size); L_status.append(status); L_block.append(block)
            L_ncand.append(ncand); L_gap.append(gap); L_gpair.append(gpair); L_partial.append(partial)
            return len(L_lo) - 1

        U, absU, rho, means = table.U, table.absU, table.rho, pairs.means
        while stack:
            lo, size, cand = stack.pop()
            self.stats["nodes"] += 1
            if self.stats["nodes"] > cfg.max_nodes:
                raise RuntimeError("octree node budget exhausted")
            if check is not None and self.stats["nodes"] % 256 == 0:
                check()
            d = .5 * size * self.unit
            c = self.origin + (lo + .5 * size) * self.unit
            where = dom.box_status(c, d)
            if where == "outside":
                leaf(lo, size, OUTSIDE)
                continue
            if len(cand):
                keep = (np.all(pairs.aabb_upper[cand] >= c - d - pad, axis=1)
                        & np.all(pairs.aabb_lower[cand] <= c + d + pad, axis=1))
                cand = cand[keep]
            n = len(cand)
            self.stats["max_candidates"] = max(self.stats["max_candidates"], n)
            g = None
            if n:
                P = (means[cand] - c) @ U.T
                w = absU @ d
                g = np.abs(P) - rho[cand] - w[None]
                k = np.argmax(g, axis=1)
                gap = g[np.arange(n), k]
            if where == "inside":
                if n == 0:
                    leaf(lo, size, SAFE)
                    continue
                bad = np.flatnonzero(gap <= threshold)
                witnesses = []
                hopeless = (len(bad) > cfg.max_refine_pairs or (len(bad) and np.any(
                    table.refine_upper_bound(cand[bad], c, d, gap[bad]) <= threshold)))
                if len(bad) and hopeless:
                    self.stats["refine_skipped_by_bound"] += 1
                if 0 < len(bad) and not hopeless:
                    self.stats["refined_nodes"] += 1
                    for j in bad:
                        self.stats["refine_pair_attempts"] += 1
                        rg, u = table.refine_gap(int(cand[j]), c, d)
                        if rg > threshold:
                            self.stats["refine_pair_successes"] += 1
                            gap[j] = rg
                            witnesses.append((int(cand[j]), u.tolist(), float(rg)))
                        else:
                            break
                    bad = np.flatnonzero(gap <= threshold)
                if len(bad) == 0:
                    j = int(np.argmin(gap))
                    ident = leaf(lo, size, SAFE, ncand=n, gap=float(gap[j]), gpair=int(cand[j]))
                    if witnesses:
                        self.adaptive_witnesses[ident] = witnesses
                    continue
            blocked = -1
            if n:
                depth = np.min(-g - 2 * w[None], axis=1) + table.eps
                for j in np.argsort(-depth, kind="stable")[:cfg.blocked_candidates]:
                    if depth[j] < 0:
                        break
                    self.stats["inner_tests"] += 1
                    if table.box_in_inner(int(cand[j]), c, d):
                        blocked = int(cand[j])
                        break
            if blocked >= 0:
                leaf(lo, size, BLOCKED, block=blocked, ncand=n)
                continue
            if size > 1:
                half = size // 2
                for off in _CHILD[::-1]:
                    stack.append((lo + off * half, half, cand))
                continue
            ident = leaf(lo, size, UNKNOWN, ncand=n, partial=where != "inside")
            if n:
                unresolved = cand[gap <= threshold]
                self.unknown_pairs[ident] = unresolved[:32].copy()
                self.unknown_pair_counts[ident] = int(len(unresolved))
        self.lo = np.asarray(L_lo, dtype=np.int64).reshape(-1, 3)
        self.size = np.asarray(L_size, dtype=np.int64)
        self.status = np.asarray(L_status, dtype=np.int8)
        self.blocked_pair = np.asarray(L_block, dtype=np.int64)
        self.n_candidates = np.asarray(L_ncand, dtype=np.int64)
        self.min_gap = np.asarray(L_gap, dtype=float)
        self.min_gap_pair = np.asarray(L_gpair, dtype=np.int64)
        self.domain_partial = np.asarray(L_partial, dtype=bool)

    # ------------------------------------------------------------------ geometry
    def __len__(self):
        return len(self.status)

    def box(self, leaf: int):
        size = self.size[leaf]
        return self.origin + (self.lo[leaf] + .5 * size) * self.unit, .5 * size * self.unit

    def boxes(self, leaves=None):
        leaves = np.arange(len(self)) if leaves is None else np.asarray(leaves)
        half = .5 * self.size[leaves][:, None] * self.unit[None]
        centres = self.origin + (self.lo[leaves] + .5 * self.size[leaves][:, None]) * self.unit
        return centres, half

    def locate(self, point, tol: float = 1e-9) -> np.ndarray:
        """Leaves whose closed box contains ``point`` (several on shared faces)."""
        if self._kd is None:
            c, _ = self.boxes()
            self._kd = cKDTree(c)
            self._max_half = .5 * float(self.size.max()) * float(self.unit.max())
        q = np.asarray(point, float)
        near = np.asarray(self._kd.query_ball_point(q, self._max_half + tol, p=np.inf), dtype=np.int64)
        if not len(near):
            return near
        c, d = self.boxes(near)
        inside = np.all(np.abs(q[None] - c) <= d + tol, axis=1)
        return near[inside]

    def summary(self) -> dict:
        counts = {STATUS_NAMES[s]: int(np.count_nonzero(self.status == s)) for s in STATUS_NAMES}
        vol = self.size.astype(float) ** 3 * float(np.prod(self.unit))
        volume = {STATUS_NAMES[s]: float(vol[self.status == s].sum()) for s in STATUS_NAMES}
        return {"leaves": int(len(self)), "leaves_by_status": counts, "volume_m3_by_status": volume,
                "root_shape": list(self.root_shape), "leaf_unit_m": self.unit.tolist(),
                "root_m": self.root_m.tolist(), "grid_inset_m": self.inset_m,
                "config": asdict(self.config)}
