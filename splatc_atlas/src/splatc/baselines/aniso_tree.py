"""Round-2 machine: anisotropic certified cell tree.

Fixes the round-1 capacity disease measured by the oracle-guided floor
(floor ∝ margin^-2 under isotropic 2x2x2 splits): each split now bisects ONE
dimension — the one contributing most to the Lipschitz radius
r_cell = |(sx/2, sy/2)| + a_max*(s_theta/2) — so refinement pays 2x per
generation instead of 8x, and cells shrink only where certification needs it.

Certification, billing, seeding, and connectivity semantics are identical to
probe_methods.ProbeTree (tri-state via eval_points_bounds; FREE-only
union-find; box-disc goal test).  Scoring is pluggable for round-2 arms.
"""
from __future__ import annotations

import heapq
from collections import defaultdict

import numpy as np

from ..common.se2 import TWO_PI


class _UF:
    def __init__(self):
        self.p = {}

    def find(self, a):
        self.p.setdefault(a, a)
        while self.p[a] != a:
            self.p[a] = self.p[self.p[a]]
            a = self.p[a]
        return a

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[ra] = rb


class AnisoTree:
    ROOT = (7, 5, 8)

    def __init__(self, scene, robot, score_fn, max_level=14, batch_splits=48,
                 roi=None, theta_span=None, pair_info=False):
        # roi / theta_span: P3.7 domain matching (see ProbeTree.__init__;
        # theta_span=pi is sound only for centrally symmetric robots).
        # pair_info=True computes tag-free bilateral (ha, hb) per evaluated
        # cell (same call and billing convention as ProbeTree's "pair"
        # policy) and stores them in rec for the score_fn.  Defaults
        # reproduce the original behavior.
        self.scene, self.robot = scene, robot
        self.score_fn = score_fn      # (key, rec, tree) -> float | None(skip)
        self.max_level = max_level    # per-dimension cap
        self.batch_splits = batch_splits
        self.pair_info = pair_info
        if roi is None:
            xmin, xmax, ymin, ymax = scene.workspace
        else:
            xmin, xmax, ymin, ymax = roi
        self.theta_span = TWO_PI if theta_span is None else float(theta_span)
        self.origin = np.array([xmin, ymin, 0.0])
        self.span = np.array([xmax - xmin, ymax - ymin, self.theta_span])
        self.leaves = {}
        self.split_dim = {}           # key -> dimension it was split along
        self.queries = 0
        self._heap = []
        self._tick = 0

    # key = (lx, ly, lk, ix, iy, ik)
    def cell_size(self, key):
        lv = np.array(key[:3])
        return self.span / (np.array(self.ROOT) * (2.0 ** lv))

    def cell_center(self, key):
        size = self.cell_size(key)
        return self.origin + (np.array(key[3:]) + 0.5) * size

    def cell_radius(self, key):
        sx, sy, sk = self.cell_size(key)
        a = max(self.robot.a, self.robot.b)
        return float(np.hypot(sx / 2, sy / 2) + a * (sk / 2))

    def pick_dim(self, key):
        sx, sy, sk = self.cell_size(key)
        a = max(self.robot.a, self.robot.b)
        contrib = [sx / 2, sy / 2, a * sk / 2]
        for d in np.argsort(contrib)[::-1]:
            if key[d] < self.max_level:
                return int(d)
        return None

    def locate(self, x, y, th):
        f = (np.array([x, y, th % self.theta_span]) - self.origin) / self.span
        if not (0 <= f[0] < 1 and 0 <= f[1] < 1):
            return None
        idx = np.minimum((f * self.ROOT).astype(int), np.array(self.ROOT) - 1)
        key = (0, 0, 0, int(idx[0]), int(idx[1]), int(idx[2]))
        while key not in self.leaves:
            if key not in self.split_dim:
                return None
            d = self.split_dim[key]
            lv = list(key[:3]); ix = list(key[3:])
            lv[d] += 1
            n_d = self.ROOT[d] * (2 ** lv[d])
            ix[d] = min(int(f[d] * n_d), n_d - 1)
            key = (*lv, *ix)
        return key

    def _evaluate(self, keys):
        by_theta = defaultdict(list)
        for k in keys:
            c = self.cell_center(k)
            by_theta[round(float(c[2]), 12)].append((k, c[0], c[1]))
        for th, items in by_theta.items():
            X = np.array([i[1] for i in items])
            Y = np.array([i[2] for i in items])
            free, rho, m_free, m_pen = self.scene.eval_points_bounds(
                self.robot, th, X, Y)
            for n, (k, cx, cy) in enumerate(items):
                r_cell = self.cell_radius(k)
                if m_free[n] > r_cell:
                    status = "FREE"
                elif m_pen[n] > r_cell:
                    status = "COLL"
                else:
                    status = "AMBIG"
                rec = {"status": status, "rho": float(rho[n]),
                       "m_free": float(m_free[n]), "m_pen": float(m_pen[n])}
                if self.pair_info:
                    ha, hb, _ = self.scene.bilateral_rho_tagfree(
                        self.robot, (cx, cy, th))
                    rec["pair_ha"], rec["pair_hb"] = float(ha), float(hb)
                self.leaves[k] = rec
                if status == "AMBIG" and self.pick_dim(k) is not None:
                    s = self.score_fn(k, rec, self)
                    if s is not None:
                        self._tick += 1
                        heapq.heappush(self._heap, (s, self._tick, k))
        self.queries += len(keys)

    def _split(self, key):
        d = self.pick_dim(key)
        if d is None:
            return []
        del self.leaves[key]
        self.split_dim[key] = d
        return self.children_of(key)

    def children_of(self, key):
        d = self.split_dim[key]
        lv = list(key[:3]); ix = list(key[3:])
        lv[d] += 1
        kids = []
        for c in (0, 1):
            ix2 = ix.copy()
            ix2[d] = 2 * ix[d] + c
            kids.append((*lv, *ix2))
        return kids

    def _bounds(self, key):
        size = self.cell_size(key)
        c = self.cell_center(key)
        return c - size / 2.0, c + size / 2.0

    def face_adjacent_leaves(self, key):
        """EXACT neighbor enumeration (round-7 fairness fix; the old
        single face-center probe misses hanging-face neighbors).  For
        each face: descend from the root cell just across the face,
        keeping every leaf whose region contains the face plane along
        `dim` and overlaps the cell's transverse intervals with positive
        measure.  Theta wraps; x/y domain edges end the domain."""
        out = set()
        lo, hi = self._bounds(key)
        tol = 1e-9
        n0 = np.array(self.ROOT)
        for dim in range(3):
            for sgn in (-1, 1):
                p = np.array([(lo[d] + hi[d]) / 2.0 for d in range(3)])
                p[dim] = (hi[dim] if sgn > 0 else lo[dim]) + sgn * tol
                if dim == 2:
                    p[2] = self.origin[2] + (
                        (p[2] - self.origin[2]) % self.span[2])
                frac = (p - self.origin) / self.span
                if not (0 <= frac[0] < 1 and 0 <= frac[1] < 1):
                    continue
                ridx = np.minimum((frac * n0).astype(int), n0 - 1)
                stack = [(0, 0, 0, int(ridx[0]), int(ridx[1]),
                          int(ridx[2]))]
                while stack:
                    cur = stack.pop()
                    clo, chi = self._bounds(cur)
                    if not (clo[dim] - tol <= p[dim] <= chi[dim] + tol):
                        continue
                    ok = True
                    for d in range(3):
                        if d == dim:
                            continue
                        if min(hi[d], chi[d]) - max(lo[d], clo[d]) <= tol:
                            ok = False
                            break
                    if not ok:
                        continue
                    if cur in self.leaves:
                        if cur != key:
                            out.add(cur)
                    elif cur in self.split_dim:
                        stack.extend(self.children_of(cur))
        return out

    def run(self, checkpoints, q_start, goal_xy, goal_radius):
        checkpoints = sorted(checkpoints)
        results = {}
        n0 = self.ROOT
        self._evaluate([(0, 0, 0, i, j, k) for i in range(n0[0])
                        for j in range(n0[1]) for k in range(n0[2])])
        for seed in (tuple(q_start), (goal_xy[0], goal_xy[1], q_start[2])):
            for _ in range(3 * self.max_level):
                key = self.locate(*seed)
                if key is None or self.leaves[key]["status"] != "AMBIG":
                    break
                kids = self._split(key)
                if not kids:
                    break
                self._evaluate(kids)
        while checkpoints:
            if self.queries >= checkpoints[0]:
                results[checkpoints.pop(0)] = self._answer(
                    q_start, goal_xy, goal_radius)
                continue
            batch = []
            while self._heap and len(batch) < self.batch_splits:
                _, _, key = heapq.heappop(self._heap)
                if key in self.leaves and self.leaves[key]["status"] == "AMBIG":
                    batch.append(key)
            if not batch:
                while checkpoints:
                    results[checkpoints.pop(0)] = self._answer(
                        q_start, goal_xy, goal_radius)
                break
            kids = []
            for key in batch:
                kids += self._split(key)
            self._evaluate(kids)
        return results

    def _answer(self, q_start, goal_xy, goal_radius):
        # Diagnostic parity with ProbeTree._answer (P3.5 fields; P3.7):
        # goal_resolved, optimistic FREE+AMBIG connectivity, failure_reason
        # taxonomy, scene counter snapshots.  MEASUREMENT ONLY — the
        # conservative `reachable` semantics (FREE-only connectivity,
        # budgeted false negative on miss) are unchanged.
        gx, gy = goal_xy

        def goal_hit(k):
            size = self.cell_size(k)
            c = self.cell_center(k)
            dx = max(abs(gx - c[0]) - size[0] / 2, 0.0)
            dy = max(abs(gy - c[1]) - size[1] / 2, 0.0)
            return dx * dx + dy * dy <= goal_radius ** 2

        def connectivity(statuses):
            # round-7: exact face-overlap adjacency (was: one face-center
            # probe per face, which misses hanging-face neighbors)
            uf = _UF()
            keys = [k for k, r in self.leaves.items()
                    if r["status"] in statuses]
            keyset = set(keys)
            for k in keys:
                for nb in self.face_adjacent_leaves(k):
                    if nb in keyset:
                        uf.union(k, nb)
            return keys, uf

        free_keys, uf = connectivity(("FREE",))
        stats = {"queries": self.queries, "leaves": len(self.leaves),
                 "free_cells": len(free_keys),
                 "ambig_cells": sum(1 for r in self.leaves.values()
                                    if r["status"] == "AMBIG")}
        if hasattr(self.scene, "pair_ops_total"):
            stats["pair_ops"] = int(self.scene.pair_ops_total())
        if hasattr(self.scene, "bp_hits_total"):
            stats["bp_hits"] = int(self.scene.bp_hits_total())
        start = self.locate(*q_start)
        start_free = (start is not None and
                      self.leaves.get(start, {}).get("status") == "FREE")
        goal_free = [k for k in free_keys if goal_hit(k)]
        stats["start_resolved"] = bool(start_free)
        stats["goal_resolved"] = bool(goal_free)
        ok_keys, uf_o = connectivity(("FREE", "AMBIG"))
        okset = set(ok_keys)
        goal_o = [k for k in ok_keys if goal_hit(k)]
        reach_opt = bool(start in okset and any(
            uf_o.find(k) == uf_o.find(start) for k in goal_o))
        stats["reachable_optimistic"] = reach_opt
        if not start_free:
            stats.update({"reachable": False,
                          "failure_reason": "start_unresolved"})
            return stats
        reach = any(uf.find(k) == uf.find(start) for k in goal_free)
        if reach:
            reason = None
        elif not goal_free:
            reason = "goal_unresolved"
        elif reach_opt:
            reason = "ambiguous_cut"
        else:
            reason = "optimistically_disconnected"
        stats.update({"reachable": bool(reach), "failure_reason": reason})
        return stats
