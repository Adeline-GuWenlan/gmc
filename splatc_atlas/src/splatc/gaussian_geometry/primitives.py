"""Scene / robot primitive containers and the vectorized pose evaluator.

Scene primitives in the G1 pilot are discs (isotropic ellipses), grouped by
radius, with one cKDTree per group cached on the scene object.  The evaluator
implements the broad-phase contract from problem_spec.md §2:

  quick-free    : nearest center farther than a+R+PAD  -> pair irrelevant,
                  rho capped at RHO_CAP (sign exact, clearance capped)
  quick-collide : nearest center closer than b+R       -> collision certain
  ambiguous     : all centers within a+R+PAD get an exact PW evaluation
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree

from .contact import pw_ellipse_disc, checker2_signed, support_half_widths
from ..common.se2 import world_to_body

PAD = 0.2      # broad-phase shell padding (m); rho is exact within the shell
RHO_CAP = 0.25  # dimensionless cap for rho outside the shell
# tag-free opposing-side clustering (P1a, review round-3): a cold contact
# set is bilateral iff the second-largest circular gap between sorted
# contact directions exceeds this.  Frozen at design time; not tuned.
GAP_MIN_RAD = np.radians(45.0)
# conservative tie-break for measure-zero exact tangencies (problem_spec §2):
# FREE requires margin > FREE_EPS, so +-1ulp float noise at mathematically
# tangent poses can never flip a label (found by the V4 symmetry audit)
FREE_EPS = 1e-9


@dataclass
class Robot:
    robot_id: str
    a: float  # semi-major (body x)
    b: float  # semi-minor (body y)

    @property
    def is_circle(self):
        return abs(self.a - self.b) < 1e-12

    @property
    def symmetric(self):
        return True  # all pilot bodies are centro-symmetric

    def spec(self):
        return {"robot_id": self.robot_id, "family": "ellipse",
                "semi_axes": [self.a, self.b],
                "circumradius": max(self.a, self.b),
                "inradius": min(self.a, self.b),
                "aspect_ratio": max(self.a, self.b) / min(self.a, self.b)}


@dataclass
class SceneGeometry:
    scene_id: str
    workspace: tuple  # (xmin, xmax, ymin, ymax)
    disc_groups: list  # list of (radius, centers[N,2])
    meta: dict = field(default_factory=dict)
    _trees: list = field(default_factory=list, repr=False)

    def __post_init__(self):
        self._trees = [cKDTree(centers) for _, centers in self.disc_groups]
        # P3.5 three-layer accounting (review round-4 §2.2): MEASUREMENT
        # ONLY — pair_ops counts exact narrow-phase pair evaluations
        # (pw_ellipse_disc elements / circle closed forms / checker#2
        # point-to-ellipse), bp_hits counts broad-phase KDTree lookups.
        self.pair_ops = 0
        self.bp_hits = 0

    def pair_ops_total(self):
        """pair_ops including tag sub-scenes (side_rho oracle path)."""
        t = self.pair_ops
        for sub in self.meta.get("_tag_cache", {}).values():
            t += sub.pair_ops
        return t

    def bp_hits_total(self):
        t = self.bp_hits
        for sub in self.meta.get("_tag_cache", {}).values():
            t += sub.bp_hits
        return t

    @property
    def n_primitives(self):
        return sum(len(c) for _, c in self.disc_groups)

    # -- exact evaluation ---------------------------------------------------

    def eval_points(self, robot, theta, X, Y):
        """Free mask + rho for world points (X,Y) at heading theta.

        Returns (free, rho); rho is the PW value min over pairs inside the
        broad-phase shell, capped at RHO_CAP outside, and only meaningful
        where containment holds.
        """
        X = np.asarray(X, dtype=np.float64).ravel()
        Y = np.asarray(Y, dtype=np.float64).ravel()
        n = X.size
        xmin, xmax, ymin, ymax = self.workspace
        px, py = support_half_widths(robot.a, robot.b, theta)
        free = ((X - px > xmin + FREE_EPS) & (X + px < xmax - FREE_EPS)
                & (Y - py > ymin + FREE_EPS) & (Y + py < ymax - FREE_EPS))
        rho = np.full(n, RHO_CAP, dtype=np.float64)

        pts = np.column_stack([X, Y])
        for (Rg, centers), tree in zip(self.disc_groups, self._trees):
            reach = robot.a + Rg + PAD
            alive = np.flatnonzero(free)
            if alive.size == 0:
                break
            d1, _ = tree.query(pts[alive], k=1)
            self.bp_hits += int(alive.size)
            if robot.is_circle:
                h1 = d1 / (robot.a + Rg) - 1.0
                self.pair_ops += int(alive.size)
                rho[alive] = np.minimum(rho[alive], np.where(d1 > reach, RHO_CAP, h1))
                free[alive] &= h1 > FREE_EPS
                continue
            near = d1 <= reach
            idx_near = alive[near]
            if idx_near.size == 0:
                continue
            # quick-collide
            crit = d1[near] < (robot.b + Rg)
            free[idx_near[crit]] = False
            rho[idx_near[crit]] = np.minimum(rho[idx_near[crit]], -0.5)
            # ambiguous -> exact PW over every center within reach
            amb = idx_near[~crit]
            if amb.size == 0:
                continue
            neigh = tree.query_ball_point(pts[amb], r=reach)
            counts = np.fromiter((len(nb) for nb in neigh), dtype=np.int64,
                                 count=len(neigh))
            if counts.sum() == 0:
                continue
            pt_rep = np.repeat(amb, counts)
            ctr_idx = np.concatenate([np.asarray(nb, dtype=np.int64)
                                      for nb in neigh if len(nb)])
            vx = centers[ctr_idx, 0] - X[pt_rep]
            vy = centers[ctr_idx, 1] - Y[pt_rep]
            u1, u2 = world_to_body(theta, vx, vy)
            h = pw_ellipse_disc(u1, u2, robot.a, robot.b, Rg)
            self.pair_ops += int(h.size)
            np.minimum.at(rho, pt_rep, h)
        free &= rho > FREE_EPS
        return free, rho

    def eval_points_bounds(self, robot, theta, X, Y):
        """Like eval_points, but additionally returns SOUND metric bounds
        (meters) for cell certification (plan Stage 3 step 2):

          m_free : lower bound on true clearance when free
                   (triangle bound d1-(a+R) far away; PW scaling bound
                   h*(b+R) inside the shell; workspace box margins)
          m_pen  : lower bound on penetration depth when colliding
                   ((b+R)-d1 when the nearest center is inside the robot's
                   inradius ring; (-h)*(b+R) from PW pairs; box violation)

        Both bounds are conservative: m_free > r_cell certifies an entire
        cell FREE, m_pen > r_cell certifies it COLLIDING, where r_cell is the
        cell's Lipschitz half-diagonal |(sx/2, sy/2)| + a*(s_theta/2).
        """
        X = np.asarray(X, dtype=np.float64).ravel()
        Y = np.asarray(Y, dtype=np.float64).ravel()
        n = X.size
        xmin, xmax, ymin, ymax = self.workspace
        px, py = support_half_widths(robot.a, robot.b, theta)
        box_margin = np.minimum.reduce([X - px - xmin, xmax - (X + px),
                                        Y - py - ymin, ymax - (Y + py)])
        m_free = box_margin.copy()                    # signed; clamp at caller
        m_pen = -box_margin                           # box violation depth
        rho = np.full(n, RHO_CAP, dtype=np.float64)
        pts = np.column_stack([X, Y])
        for (Rg, centers), tree in zip(self.disc_groups, self._trees):
            d1, _ = tree.query(pts, k=1)
            self.bp_hits += int(n)
            g_free = d1 - (robot.a + Rg)              # sound everywhere
            g_pen = (robot.b + Rg) - d1               # sound where positive
            if robot.is_circle:
                rho = np.minimum(rho, d1 / (robot.a + Rg) - 1.0)
                # for circles both bounds are exact
            else:
                near = np.flatnonzero(d1 <= robot.a + Rg + PAD)
                if near.size:
                    neigh = tree.query_ball_point(pts[near],
                                                  r=robot.a + Rg + PAD)
                    counts = np.fromiter((len(nb) for nb in neigh),
                                         dtype=np.int64, count=len(neigh))
                    if counts.sum():
                        pt_rep = near[np.repeat(np.arange(len(near)), counts)]
                        ctr_idx = np.concatenate(
                            [np.asarray(nb, dtype=np.int64)
                             for nb in neigh if len(nb)])
                        vx = centers[ctr_idx, 0] - X[pt_rep]
                        vy = centers[ctr_idx, 1] - Y[pt_rep]
                        u1, u2 = world_to_body(theta, vx, vy)
                        h = pw_ellipse_disc(u1, u2, robot.a, robot.b, Rg)
                        self.pair_ops += int(h.size)
                        hmin = np.full(n, np.inf)
                        np.minimum.at(hmin, pt_rep, h)
                        np.minimum.at(rho, pt_rep, h)
                        seen = np.isfinite(hmin)
                        scale = robot.b + Rg          # sound PW->metric factor
                        # within a group, either bound is valid: take the better
                        g_free = g_free.copy()
                        g_free[seen] = np.maximum(g_free[seen],
                                                  hmin[seen] * scale)
                        g_pen = g_pen.copy()
                        g_pen[seen] = np.maximum(g_pen[seen],
                                                 -hmin[seen] * scale)
            m_free = np.minimum(m_free, g_free)
            m_pen = np.maximum(m_pen, g_pen)
        free = (box_margin > FREE_EPS) & (rho > FREE_EPS)
        return free, rho, m_free, m_pen

    def check_pose(self, robot, q):
        """Exact single-pose check.  Returns (free, rho)."""
        x, y, theta = q
        free, rho = self.eval_points(robot, theta, np.array([x]), np.array([y]))
        return bool(free[0]), float(rho[0])

    # -- independent checker #2 --------------------------------------------

    # -- active-pair identity (module 2: pair identity beyond scalar rho) ----

    def by_tag(self, tag):
        """Sub-scene containing only primitives with the given tag (from
        meta['group_tags']).  Cached.  This is the minimal active-pair
        identity API: per-side rho decomposes the scalar clearance min."""
        cache = self.meta.setdefault("_tag_cache", {})
        if tag not in cache:
            tags = self.meta["group_tags"]
            groups = [(R, centers[t == tag])
                      for (R, centers), t in zip(self.disc_groups, tags)
                      if np.any(t == tag)]
            cache[tag] = SceneGeometry(f"{self.scene_id}|tag{tag}",
                                       self.workspace, groups,
                                       {"parent": self.scene_id})
        return cache[tag]

    def side_rho(self, robot, theta, X, Y):
        """Per-side clearance (rho restricted to each wall-side tag).
        Conceptually free with the pose evaluation: real per-pair h values
        already carry pair identity; the sub-scene re-query here is an
        implementation convenience, billed as the SAME collision query."""
        out = {}
        for tag in sorted({int(t) for arr in self.meta["group_tags"]
                           for t in np.unique(arr)}):
            _, rho = self.by_tag(tag).eval_points(robot, theta, X, Y)
            out[tag] = rho
        return out

    # -- tag-free bilateral grouping (P1a, review round-3) -------------------

    def contact_pairs(self, robot, q):
        """Active-pair identity at a pose WITHOUT wall-side tags: every
        primitive inside the broad-phase shell contributes (h, contact
        direction), direction = unit vector robot-center -> disc center in
        the world frame (normal proxy sufficient for opposing-side
        clustering).  Billed as ONE collision query — same information base
        as eval_points' internal ambiguous-pair loop; review round-3:
        active pair identity is method-legal, pre-given side tags are not.
        Returns (h[N], dirs[N,2])."""
        x, y, theta = q
        hs, dirs = [], []
        for (Rg, centers), tree in zip(self.disc_groups, self._trees):
            reach = robot.a + Rg + PAD
            idx = tree.query_ball_point([x, y], r=reach)
            self.bp_hits += 1
            if not idx:
                continue
            c = centers[np.asarray(idx, dtype=np.int64)]
            vx = c[:, 0] - x
            vy = c[:, 1] - y
            u1, u2 = world_to_body(theta, vx, vy)
            h = pw_ellipse_disc(u1, u2, robot.a, robot.b, Rg)
            self.pair_ops += int(h.size)
            n = np.hypot(vx, vy)
            n[n < 1e-12] = 1.0
            hs.append(h)
            dirs.append(np.column_stack([vx / n, vy / n]))
        if not hs:
            return np.empty(0), np.empty((0, 2))
        return np.concatenate(hs), np.vstack(dirs)

    def bilateral_rho_tagfree(self, robot, q, ref=None):
        """Tag-free per-side clearance via opposing-direction clustering.

        With ref = (dir_a[2], dir_b[2]) from the previous evaluation
        (temporal signature continuity), each pair joins the nearer
        reference direction.  Cold (ref=None): sort contact angles, split
        at the two largest circular gaps; unilateral if the second-largest
        gap < GAP_MIN_RAD.  A missing side reads RHO_CAP and inherits its
        reference direction (antipode when cold).

        Returns (h_a, h_b, (dir_a, dir_b)).  Deterministic; label identity
        is only meaningful through the ref chain (B = h_a - h_b sign
        continuity), matching how side_rho's fixed tags were used."""
        h, d = self.contact_pairs(robot, q)
        if h.size == 0:
            if ref is not None:
                return RHO_CAP, RHO_CAP, ref
            e = np.array([0.0, 1.0])
            return RHO_CAP, RHO_CAP, (e, -e)
        if ref is not None:
            sa = d @ np.asarray(ref[0])
            sb = d @ np.asarray(ref[1])
            in_a = sa >= sb
        else:
            ang = np.arctan2(d[:, 1], d[:, 0])
            order = np.argsort(ang)
            if h.size == 1:
                in_a = np.array([True])
            else:
                sa_sorted = ang[order]
                gaps = np.diff(np.append(sa_sorted,
                                         sa_sorted[0] + 2 * np.pi))
                g1, g2 = np.argsort(gaps)[-2:]  # two largest gaps
                if gaps[min(g1, g2)] < GAP_MIN_RAD \
                        or gaps[max(g1, g2)] < GAP_MIN_RAD:
                    in_a = np.ones(h.size, dtype=bool)   # unilateral
                else:
                    lo, hi = sorted((g1, g2))
                    arc = np.zeros(h.size, dtype=bool)
                    arc[order[lo + 1:hi + 1]] = True     # one arc
                    in_a = arc
        def side(mask, other_dir, own_ref):
            # top capped at RHO_CAP like side_rho; penetration reports the
            # EXACT PW value (side_rho's quick-collide path fills a -0.5
            # marker instead — the tag-free value is strictly more
            # informative; divergence is confined to colliding poses)
            if not mask.any():
                if own_ref is not None:
                    return RHO_CAP, np.asarray(own_ref)
                return RHO_CAP, -other_dir
            m = float(min(RHO_CAP, h[mask].min()))
            v = d[mask].sum(axis=0)
            nv = np.linalg.norm(v)
            v = v / nv if nv > 1e-12 else d[mask][0]
            return m, v
        ra = ref[0] if ref is not None else None
        rb = ref[1] if ref is not None else None
        h_a, dir_a = side(in_a, np.array([0.0, -1.0]), ra)
        h_b, dir_b = side(~in_a, dir_a, rb)
        if ref is None and in_a.all():
            dir_b = -dir_a                                # unilateral cold
        # deterministic cold labeling: A = larger-y mean direction
        if ref is None and not in_a.all() and dir_a[1] < dir_b[1]:
            h_a, h_b, dir_a, dir_b = h_b, h_a, dir_b, dir_a
        return float(h_a), float(h_b), (dir_a, dir_b)

    def check_pose_independent(self, robot, q):
        """Checker #2: metric margin via point-to-ellipse distance (discs) or
        circle-circle closed form; plus the same containment test.  Only the
        SIGN is contract-comparable with checker #1."""
        x, y, theta = q
        xmin, xmax, ymin, ymax = self.workspace
        px, py = support_half_widths(robot.a, robot.b, theta)
        contained = (x - px > xmin + FREE_EPS) and (x + px < xmax - FREE_EPS) \
            and (y - py > ymin + FREE_EPS) and (y + py < ymax - FREE_EPS)
        margin = np.inf
        for Rg, centers in self.disc_groups:
            self.pair_ops += int(len(centers))
            vx = centers[:, 0] - x
            vy = centers[:, 1] - y
            if robot.is_circle:
                m = np.hypot(vx, vy) - (robot.a + Rg)
            else:
                u1, u2 = world_to_body(theta, vx, vy)
                m = checker2_signed(u1, u2, robot.a, robot.b, Rg)
            margin = min(margin, float(np.min(m)))
        return bool(contained and margin > FREE_EPS), margin
