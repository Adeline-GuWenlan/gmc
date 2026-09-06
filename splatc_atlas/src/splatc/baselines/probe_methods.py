"""Budget-matched probing policies over a certified adaptive cell tree
(Sprint B decisive experiment; implements plan Stage 3 step 2:
FREE / COLLISION / AMBIGUOUS certification).

Certification: a cell with Lipschitz half-radius
  r_cell = |(sx/2, sy/2)| + a_max * (s_theta/2)
is certified FREE if the sound metric clearance lower bound at its center
exceeds r_cell, certified COLLIDING if the sound penetration lower bound
exceeds r_cell, else AMBIGUOUS.  Certified cells are terminal; only ambiguous
cells may be subdivided.  Connectivity runs over certified-FREE cells only,
so a REACHABLE answer can never be a false-reachable (one-sided error by
construction; the measured failure mode is false-unreachable under budget).

The three policies differ ONLY in how they order the ambiguous queue:
  uniform  : breadth-first by level (uniform refinement of the unknown set)
  generic  : boundary proximity |rho| (scalar clearance only — B2 contract)
  contact  : bilateral per-side clearance (active-pair identity from the
             same pose evaluation) ranks gate-suspect cells first

Billing: 1 collision query per evaluated cell center, identical for all
policies (pair identities/side minima are by-products of the same pairwise
h_ij evaluation).
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


class ProbeTree:
    ROOT = (7, 5, 8)

    def __init__(self, scene, robot, policy, max_level=10, batch_splits=48,
                 roi=None, theta_span=None):
        # roi=(xmin, xmax, ymin, ymax) restricts the search domain (P3.7
        # domain matching); theta_span=pi runs the [0, pi) quotient — SOUND
        # ONLY for centrally symmetric robots (collision status must be
        # pi-periodic in theta; the caller asserts symmetry).  Defaults
        # reproduce the frozen v4 behavior bit-for-bit.
        assert policy in ("uniform", "generic", "contact", "pair")
        self.scene, self.robot, self.policy = scene, robot, policy
        self.max_level = max_level
        self.batch_splits = batch_splits
        if roi is None:
            xmin, xmax, ymin, ymax = scene.workspace
        else:
            xmin, xmax, ymin, ymax = roi
        self.theta_span = TWO_PI if theta_span is None else float(theta_span)
        self.origin = np.array([xmin, ymin, 0.0])
        self.span = np.array([xmax - xmin, ymax - ymin, self.theta_span])
        self.leaves = {}   # key -> {status: FREE|COLL|AMBIG, rho, bilateral}
        self.split_set = set()   # keys that were split (exact adjacency)
        self.queries = 0
        self._heap = []
        self._tick = 0

    def split_cell(self, key):
        """Split a leaf: remove it, record the split topology (exact
        adjacency needs it), return the 8 children keys to evaluate."""
        l, ix, iy, ik = key
        del self.leaves[key]
        self.split_set.add(key)
        return [(l + 1, 2 * ix + dx, 2 * iy + dy, 2 * ik + dk)
                for dx in (0, 1) for dy in (0, 1) for dk in (0, 1)]

    def face_adjacent_leaves(self, key):
        """EXACT neighbor enumeration (round-7 fairness fix).  The old
        face-center single probe found only ONE of the finer leaves
        sharing a hanging face, so legal FREE adjacencies could be
        missed — a false negative in exactly the direction the
        budgeted-false-negative claims depend on.  For each face, the
        same-size virtual neighbor is either covered by a same-or-coarser
        leaf (unique; ancestor walk) or was split, in which case ALL its
        face-touching descendant leaves are collected.  Theta wraps
        periodically; x/y domain edges have no neighbor."""
        out = set()
        l, ix, iy, ik = key
        counts = [self.ROOT[0] * 2 ** l, self.ROOT[1] * 2 ** l,
                  self.ROOT[2] * 2 ** l]
        for dim in range(3):
            for sgn in (-1, 1):
                idx = [ix, iy, ik]
                idx[dim] += sgn
                if dim == 2:
                    idx[2] %= counts[2]
                elif not (0 <= idx[dim] < counts[dim]):
                    continue
                nb = (l, idx[0], idx[1], idx[2])
                if nb in self.leaves:
                    out.add(nb)
                    continue
                if nb in self.split_set:
                    side = 0 if sgn > 0 else 1
                    stack = [nb]
                    while stack:
                        cur = stack.pop()
                        if cur in self.leaves:
                            out.add(cur)
                            continue
                        if cur not in self.split_set:
                            continue
                        cl, cx, cy, ck = cur
                        for dx in (0, 1):
                            for dy in (0, 1):
                                for dk in (0, 1):
                                    if (dx, dy, dk)[dim] != side:
                                        continue
                                    stack.append((cl + 1, 2 * cx + dx,
                                                  2 * cy + dy, 2 * ck + dk))
                    continue
                # covered by a coarser leaf: unique ancestor
                k2 = nb
                while k2[0] > 0:
                    k2 = (k2[0] - 1, k2[1] // 2, k2[2] // 2, k2[3] // 2)
                    if k2 in self.leaves:
                        out.add(k2)
                        break
        return out

    # -- cell geometry --------------------------------------------------------

    def cell_center(self, key):
        l, ix, iy, ik = key
        n = np.array(self.ROOT) * (2 ** l)
        return self.origin + (np.array([ix, iy, ik]) + 0.5) / n * self.span

    def cell_radius(self, level):
        n = np.array(self.ROOT) * (2 ** level)
        sx, sy, sk = self.span / n
        a_max = max(self.robot.a, self.robot.b)
        return float(np.hypot(sx / 2, sy / 2) + a_max * (sk / 2))

    def locate(self, x, y, th):
        th = th % self.theta_span
        for l in range(self.max_level, -1, -1):
            n = np.array(self.ROOT) * (2 ** l)
            f = (np.array([x, y, th]) - self.origin) / self.span
            if not (0 <= f[0] < 1 and 0 <= f[1] < 1):
                return None
            idx = np.minimum((f * n).astype(int), n - 1)
            key = (l, int(idx[0]), int(idx[1]), int(idx[2]))
            if key in self.leaves:
                return key
        return None

    # -- evaluation ------------------------------------------------------------

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
            if self.policy == "contact":
                sides = self.scene.side_rho(self.robot, th, X, Y)
                s_bi = np.maximum(sides[1], sides[-1])
                s_bal = np.abs(sides[1] - sides[-1])
            if self.policy == "pair":
                # pair-aware volumetric baseline (round-5 review §5): the
                # METHOD's tag-free pair information, volumetric
                # representation, NO station-to-station continuation.
                # Billing: pair info from the same pose evaluation, 1 query
                # per cell center (same convention as the contact policy).
                pair_vals = []
                for (_, cx, cy) in items:
                    ha, hb, _ = self.scene.bilateral_rho_tagfree(
                        self.robot, (cx, cy, th))
                    pair_vals.append((ha, hb))
            for n, (k, _, _) in enumerate(items):
                r_cell = self.cell_radius(k[0])
                if m_free[n] > r_cell:
                    status = "FREE"
                elif m_pen[n] > r_cell:
                    status = "COLL"
                else:
                    status = "AMBIG"
                # m_free is the sound free lower bound the FREE verdict
                # compared against r_cell — kept so a chart can serialize
                # its per-cell certificate slack (P4a.1); pure record
                # field, no effect on refinement order or billing
                rec = {"status": status, "rho": float(rho[n]),
                       "m_free": float(m_free[n])}
                if self.policy == "contact":
                    rec["bilateral"] = float(s_bi[n])
                    rec["balance"] = float(s_bal[n])
                if self.policy == "pair":
                    rec["pair_ha"] = float(pair_vals[n][0])
                    rec["pair_hb"] = float(pair_vals[n][1])
                self.leaves[k] = rec
                if status == "AMBIG":
                    self._push(k, rec)
        self.queries += len(keys)

    def _push(self, key, rec):
        if key[0] >= self.max_level:
            return
        self._tick += 1
        l = key[0]
        if self.policy == "uniform":
            score = float(l)
        elif self.policy == "generic":
            score = abs(rec["rho"]) + 0.25 * l
        elif self.policy == "pair":
            # design frozen at creation (round-5 §5): contact-v4's cue
            # family with its round-1 pathologies fixed — bilateral rank
            # requires BOTH opposing groups in shell AND a free pose
            # (no priority reward for double-sided collision); balance
            # cue |B| only then; colliding-bilateral next; generic tail.
            ha = rec.get("pair_ha", np.inf)
            hb = rec.get("pair_hb", np.inf)
            m = min(ha, hb)
            if max(ha, hb) < 0.2 and m > 0:
                score = abs(ha - hb) + 0.25 * l
            elif max(ha, hb) < 0.2:
                score = 0.5 + abs(m) + 0.25 * l
            else:
                score = 1.0 + abs(rec["rho"]) + 0.25 * l
        else:
            b = rec.get("bilateral", np.inf)
            if b < 0.2:   # both wall sides genuinely inside the shell
                # v4 FROZEN (round-1 artifact; see probe_round1_full_report):
                # scalar equal-clearance heuristic — NOT the Morse condition
                # (no gradient terms, hence no angular selectivity).  KNOWN
                # FLAW kept for the ablation record: b=max(h+,h-) can be
                # negative for double-sided collisions, giving them top
                # priority (review §9).  Fixes go into new round-2 arms only.
                score = rec.get("balance", 1.0) + 0.5 * b + 0.15 * l
            else:
                score = 3.0 + abs(rec["rho"]) + 0.25 * l
        if rec["rho"] > 0.15 and l <= 2:
            # open-space coarse certification branch (identical for every
            # policy, COARSE LEVELS ONLY): builds the coarse FREE skeleton
            # (Atlas charts) with a bounded budget (~15k queries), leaving all
            # deep refinement to the policy-specific signals
            score = min(score, 0.6 + 0.6 * l)
        heapq.heappush(self._heap, (score, self._tick, key))

    # -- main loop --------------------------------------------------------------

    def run(self, budget_checkpoints, q_start, goal_xy, goal_radius):
        checkpoints = sorted(budget_checkpoints)
        results = {}
        n0 = self.ROOT
        self._evaluate([(0, i, j, k) for i in range(n0[0])
                        for j in range(n0[1]) for k in range(n0[2])])
        # seed refinement: the cells containing the (known-free) start pose
        # and the goal center are force-refined until certified — identical
        # for every policy, so budgets stay matched
        for seed in (tuple(q_start), (goal_xy[0], goal_xy[1], q_start[2])):
            for _ in range(self.max_level):
                key = self.locate(*seed)
                if key is None or self.leaves[key]["status"] != "AMBIG":
                    break
                self._evaluate(self.split_cell(key))
        while checkpoints:
            if self.queries >= checkpoints[0]:
                b = checkpoints.pop(0)
                results[b] = self._answer(q_start, goal_xy, goal_radius)
                continue
            batch = []
            while self._heap and len(batch) < self.batch_splits:
                _, _, key = heapq.heappop(self._heap)
                if key in self.leaves and self.leaves[key]["status"] == "AMBIG" \
                        and key[0] < self.max_level:
                    batch.append(key)
            if not batch:
                while checkpoints:
                    b = checkpoints.pop(0)
                    results[b] = self._answer(q_start, goal_xy, goal_radius)
                break
            children = []
            for key in batch:
                children += self.split_cell(key)
            self._evaluate(children)
        return results

    # -- answer (no queries) -----------------------------------------------------

    def _answer(self, q_start, goal_xy, goal_radius):
        # P3.5 (review round-4): diagnostic fields — goal_resolved,
        # optimistic (FREE+AMBIG) connectivity, failure_reason taxonomy.
        # MEASUREMENT ONLY: refinement policy, billing and the conservative
        # `reachable` semantics (FREE-only connectivity) are UNCHANGED
        # (v4 freeze).  `reachable=False` means "not established under
        # budget" (a budgeted false negative when truth is reachable),
        # never a certified UNREACHABLE.
        gx, gy = goal_xy

        def goal_hit(k):  # goal disc intersects the cell's xy footprint
            l = k[0]
            n = np.array(self.ROOT) * (2 ** l)
            size = self.span / n
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
        # P3.7 three-currency accounting: scene-level counter snapshots at
        # checkpoint time (measurement only; refinement order unchanged)
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
        # optimistic upper bound: treat every AMBIG cell as free
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
