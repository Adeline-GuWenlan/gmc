"""P3.8 chart-factored connector baselines (round-7/8/10 mandate).

Task contract, identical for every arm: given TWO serialized certified
chart regions (F_L, F_R) plus query access to the scene, construct a
CERTIFIED CONNECTOR — a waypoint polyline whose first pose lies inside
region_L (chart_contains), whose last pose lies inside region_R, and
which passes certify_path_conservative in full.  No door axis, no
admissible-theta window, no oracle centerline: the only geometric inputs
are the serialized regions themselves (their bbox is the sampling
domain, their max-slack cell centers are the roots — both derivable by
any method from the given frontiers).  scene.meta is never read.

Billing (P3.5 three-currency): every pose evaluation goes through
BilledChecker.nq; pair_ops / bp_hits accumulate on the scene's own
counters; wall time is the caller's.  Certification of a candidate
connector is billed to the arm (the continuation arm pays its own
certification the same way).

Theta handling: all pilot bodies are centro-symmetric, so collision
status is pi-periodic (asserted by direct replay in the harness).  The
arms sample theta in [0, pi) but keep node angles UNWRAPPED — a steering
target picks the pi-representative nearest the parent node, so every
polyline is continuous in real theta and the conservative certificate
(which uses real rotation) applies unchanged.
"""
from __future__ import annotations

import heapq

import numpy as np

from ..reference.oracle import certify_path_conservative, metric_margin
from ..compiler.chart_builder import cell_bounds, chart_contains
from ..common.se2 import wrap_diff


# ---- shared geometry helpers (query-free, derived from given regions) ----

def region_bbox(region):
    """xy bbox of the region's member cells."""
    g = region["grid"]
    lo_all = np.array([np.inf, np.inf])
    hi_all = np.array([-np.inf, -np.inf])
    for key in region["cells"]:
        lo, hi = cell_bounds(g["root"], g["origin"], g["span"], tuple(key))
        lo_all = np.minimum(lo_all, lo[:2])
        hi_all = np.maximum(hi_all, hi[:2])
    return float(lo_all[0]), float(hi_all[0]), float(lo_all[1]), float(hi_all[1])


def joint_bbox(region_a, region_b):
    xa0, xa1, ya0, ya1 = region_bbox(region_a)
    xb0, xb1, yb0, yb1 = region_bbox(region_b)
    return (min(xa0, xb0), max(xa1, xb1), min(ya0, yb0), max(ya1, yb1))


def region_top_cells(region, k=1):
    """Centers of the k largest-slack member cells (certified free poses,
    zero queries)."""
    g = region["grid"]
    order = np.argsort(-np.asarray(region["cell_cert_slack_m"]))[:k]
    out = []
    for i in order:
        lo, hi = cell_bounds(g["root"], g["origin"], g["span"],
                             tuple(region["cells"][int(i)]))
        out.append(tuple((lo + hi) / 2.0))
    return out


def in_region(region, q):
    return chart_contains(region, q) is not None


class BilledChecker:
    """Counts every pose evaluation an arm makes; hard budget."""

    def __init__(self, scene, robot, budget):
        self.scene, self.robot, self.budget = scene, robot, int(budget)
        self.nq = 0

    def exhausted(self):
        return self.nq >= self.budget

    def free(self, q):
        self.nq += 1
        ok, _ = self.scene.check_pose(self.robot, q)
        return ok

    def margin(self, q):
        self.nq += 1
        return metric_margin(self.scene, self.robot, q)

    def free_batch(self, theta, X, Y):
        """Vectorized fixed-theta evaluation (HRM sweep lines); billed one
        query per pose."""
        self.nq += int(np.asarray(X).size)
        f, _ = self.scene.eval_points(self.robot, theta, X, Y)
        return f

    def edge_free(self, q1, q2, step=0.02):
        """Sampled binary freeness of the straight segment (NOT a
        certificate); every sample billed."""
        dth = q2[2] - q1[2]
        L = np.hypot(q2[0] - q1[0], q2[1] - q1[1]) \
            + max(self.robot.a, self.robot.b) * abs(dth)
        n = max(2, int(np.ceil(L / step)) + 1)
        for t in np.linspace(0.0, 1.0, n):
            if not self.free((q1[0] + t * (q2[0] - q1[0]),
                              q1[1] + t * (q2[1] - q1[1]),
                              q1[2] + t * dth)):
                return False
        return True

    def certify(self, wps):
        ok, mmin, _, checks = certify_path_conservative(
            self.scene, self.robot, np.asarray(wps, dtype=float))
        self.nq += checks
        return ok, mmin

    def polish(self, wps, rounds=2):
        """Retraction polish available to every arm: coordinate pattern
        search maximizing metric margin at each interior waypoint (the
        standard post-smoothing a sampling planner would run before
        attempting a conservative certificate).  Billed."""
        wps = [list(p) for p in wps]
        steps = [(0.02, 0.02, 0.035), (0.005, 0.005, 0.01)][:rounds]
        for dx, dy, dt in steps:
            for i in range(1, len(wps) - 1):
                q = wps[i]
                m0 = self.margin(tuple(q))
                for axis, d in ((0, dx), (1, dy), (2, dt)):
                    for s in (+d, -d):
                        cand = list(q)
                        cand[axis] += s
                        m = self.margin(tuple(cand))
                        if m > m0:
                            q, m0 = cand, m
                wps[i] = q
        return [tuple(p) for p in wps]


def _theta_rep(th, ref):
    """pi-representative of th nearest ref (centro-symmetric body)."""
    k = round((ref - th) / np.pi)
    return th + k * np.pi


def _dist(a_max, q1, q2):
    dth = _theta_rep(q2[2], q1[2]) - q1[2]
    return np.hypot(q2[0] - q1[0], q2[1] - q1[1]) + a_max * abs(dth)


def _embed(a_max, q):
    """Embedding for approximate nearest-neighbor search under the
    pi-quotient metric: theta doubles onto a circle of radius a_max/2,
    whose chord approximates a_max*|dtheta|_pi.  Selection quality only —
    exact distances/certificates never use it."""
    return (q[0], q[1], 0.5 * a_max * np.cos(2 * q[2]),
            0.5 * a_max * np.sin(2 * q[2]))


class _NN:
    """Incremental approximate NN: cKDTree over the embedding, rebuilt
    every `rebuild` insertions; the unindexed tail is scanned vectorized."""

    def __init__(self, a_max, rebuild=256):
        from scipy.spatial import cKDTree
        self._ck = cKDTree
        self.a_max, self.rebuild = a_max, rebuild
        self.emb = []
        self.tree = None
        self.n_indexed = 0

    def add(self, q):
        self.emb.append(_embed(self.a_max, q))
        if len(self.emb) - self.n_indexed >= self.rebuild:
            self.tree = self._ck(np.asarray(self.emb))
            self.n_indexed = len(self.emb)

    def nearest(self, q, k=1):
        e = np.asarray(_embed(self.a_max, q))
        best = []
        if self.tree is not None:
            d, i = self.tree.query(e, k=min(k, self.n_indexed))
            d, i = np.atleast_1d(d), np.atleast_1d(i)
            best += list(zip(d.tolist(), i.tolist()))
        if len(self.emb) > self.n_indexed:
            tail = np.asarray(self.emb[self.n_indexed:])
            dt = np.linalg.norm(tail - e, axis=1)
            best += [(float(dv), self.n_indexed + j)
                     for j, dv in enumerate(dt)]
        best.sort()
        return [i for _, i in best[:k]]


# ---- arm 1: RRT-Connect ---------------------------------------------------

def rrt_connect(scene, robot, region_L, region_R, seed, budget,
                step=0.10, cert_period=1):
    """Bidirectional RRT-Connect rooted at the regions' max-slack cell
    centers.  Extensions are sampled-checked (2cm); on tree connection
    the extracted polyline is polished and conservatively certified —
    only a certified connector counts as success.  A certification
    failure blacklists that connect edge and the search continues."""
    rng = np.random.RandomState(seed)
    ck = BilledChecker(scene, robot, budget)
    a_max = max(robot.a, robot.b)
    x0, x1, y0, y1 = joint_bbox(region_L, region_R)
    roots = (region_top_cells(region_L, 1)[0], region_top_cells(region_R, 1)[0])
    trees = [{"q": [roots[0]], "parent": [-1], "nn": _NN(a_max)},
             {"q": [roots[1]], "parent": [-1], "nn": _NN(a_max)}]
    for t in trees:
        t["nn"].add(t["q"][0])
    first_sampled = None
    cert_fails = 0

    def nearest(tree, q):
        return tree["nn"].nearest(q, 1)[0]

    def steer(q_from, q_to):
        th_to = _theta_rep(q_to[2], q_from[2])
        d = _dist(a_max, q_from, (q_to[0], q_to[1], th_to))
        if d <= step:
            return (q_to[0], q_to[1], th_to), True
        t = step / d
        return (q_from[0] + t * (q_to[0] - q_from[0]),
                q_from[1] + t * (q_to[1] - q_from[1]),
                q_from[2] + t * (th_to - q_from[2])), False

    def extend(tree, q_target):
        i = nearest(tree, q_target)
        q_new, reached = steer(tree["q"][i], q_target)
        if not ck.free(q_new):
            return None, False
        if not ck.edge_free(tree["q"][i], q_new):
            return None, False
        tree["q"].append(q_new)
        tree["parent"].append(i)
        tree["nn"].add(q_new)
        return len(tree["q"]) - 1, reached

    def path_to_root(tree, i):
        out = []
        while i >= 0:
            out.append(tree["q"][i])
            i = tree["parent"][i]
        return out

    banned = set()
    a, b = 0, 1
    while not ck.exhausted():
        q_rand = (rng.uniform(x0, x1), rng.uniform(y0, y1),
                  rng.uniform(0.0, np.pi))
        ia, _ = extend(trees[a], q_rand)
        if ia is not None:
            # greedy CONNECT of the other tree toward the new node
            q_new = trees[a]["q"][ia]
            ib, reached = extend(trees[b], q_new)
            while ib is not None and not reached and not ck.exhausted():
                ib, reached = extend(trees[b], q_new)
            if ib is not None and reached:
                key = (a, ia, ib)
                if key not in banned:
                    if first_sampled is None:
                        first_sampled = ck.nq
                    pa = path_to_root(trees[a], ia)[::-1]
                    pb = path_to_root(trees[b], ib)
                    raw = (pa + pb) if a == 0 else (pb[::-1] + pa[::-1])
                    # re-chain theta representatives: the two trees carry
                    # independent unwrapped angles, so the junction can sit
                    # a whole pi apart (same body pose, centro-symmetric);
                    # chaining keeps in-tree deltas and zeroes the junction
                    wps = [raw[0]]
                    for q in raw[1:]:
                        wps.append((q[0], q[1],
                                    _theta_rep(q[2], wps[-1][2])))
                    wps = ck.polish(wps)
                    ok, mmin = ck.certify(wps)
                    if ok and in_region(region_L, wps[0]) \
                            and in_region(region_R, wps[-1]):
                        return {"success": True, "nq": ck.nq,
                                "nq_first_sampled": first_sampled,
                                "n_states": len(trees[0]["q"])
                                + len(trees[1]["q"]),
                                "cert_fail_events": cert_fails,
                                "cert_min_margin_mm": round(mmin * 1e3, 2),
                                "path": [list(p) for p in wps]}
                    cert_fails += 1
                    banned.add(key)
        a, b = b, a
    return {"success": False, "nq": ck.nq,
            "nq_first_sampled": first_sampled,
            "n_states": len(trees[0]["q"]) + len(trees[1]["q"]),
            "cert_fail_events": cert_fails,
            "cert_min_margin_mm": None, "path": None}


# ---- arm 2: lazy PRM with bridge-test sampling ----------------------------

def lazy_prm_bridge(scene, robot, region_L, region_R, seed, budget,
                    batch=400, k_nn=10, bridge_frac=0.5,
                    bridge_sigma=(0.25, 0.25, 0.4)):
    """Roadmap over the joint bbox with 50% bridge-test samples (the
    classic narrow-passage sampler), lazy edge validation on the current
    best candidate path, and conservative certification edge-by-edge.
    Roots are the regions' max-slack cell centers."""
    rng = np.random.RandomState(seed)
    ck = BilledChecker(scene, robot, budget)
    a_max = max(robot.a, robot.b)
    x0, x1, y0, y1 = joint_bbox(region_L, region_R)
    nodes = [region_top_cells(region_L, 1)[0], region_top_cells(region_R, 1)[0]]
    edges = {}            # (i,j) i<j -> "unknown" | "free" | "dead"
    adj = {0: set(), 1: set()}    # ALIVE edges only, kept incremental —
    dead = set()                  # a per-search rebuild is O(E) and was
    first_sampled = None          # the wall-clock bottleneck
    cert_fails = 0
    nn = _NN(a_max)
    for q in nodes:
        nn.add(q)

    def kill_edge(e):
        edges[e] = "dead"
        dead.add(e)
        adj[e[0]].discard(e[1])
        adj[e[1]].discard(e[0])

    def add_node(q):
        i = len(nodes)
        nodes.append(q)
        adj[i] = set()
        for j in nn.nearest(q, k_nn):
            e = (min(i, j), max(i, j))
            if e not in dead and e not in edges:
                edges[e] = "unknown"
                adj[i].add(j)
                adj[j].add(i)
        nn.add(q)

    def sample_batch():
        n_new = 0
        while n_new < batch and not ck.exhausted():
            if rng.uniform() < bridge_frac:
                q1 = (rng.uniform(x0, x1), rng.uniform(y0, y1),
                      rng.uniform(0.0, np.pi))
                if ck.free(q1):
                    continue
                q2 = (q1[0] + rng.normal(0, bridge_sigma[0]),
                      q1[1] + rng.normal(0, bridge_sigma[1]),
                      q1[2] + rng.normal(0, bridge_sigma[2]))
                if ck.exhausted() or ck.free(q2):
                    continue
                qm = ((q1[0] + q2[0]) / 2, (q1[1] + q2[1]) / 2,
                      (q1[2] + q2[2]) / 2)
                if not ck.exhausted() and ck.free(qm):
                    add_node(qm)
                    n_new += 1
            else:
                q = (rng.uniform(x0, x1), rng.uniform(y0, y1),
                     rng.uniform(0.0, np.pi))
                if ck.free(q):
                    add_node(q)
                    n_new += 1

    def shortest():
        # Dijkstra over the ALIVE adjacency (0 -> 1)
        dist = {0: 0.0}
        prev = {}
        pq = [(0.0, 0)]
        while pq:
            d, i = heapq.heappop(pq)
            if d > dist.get(i, np.inf):
                continue
            if i == 1:
                break
            for j in adj[i]:
                nd = d + _dist(a_max, nodes[i], nodes[j])
                if nd < dist.get(j, np.inf):
                    dist[j] = nd
                    prev[j] = i
                    heapq.heappush(pq, (nd, j))
        if 1 not in prev:
            return None
        path = [1]
        while path[-1] != 0:
            path.append(prev[path[-1]])
        return path[::-1]

    while not ck.exhausted():
        sample_batch()
        n_search = 0
        while not ck.exhausted():
            n_search += 1
            if n_search > 40:              # bound Dijkstra churn per batch
                break
            path = shortest()
            if path is None:
                break                      # need more samples
            # lazy validation: sample-check unknown edges along the path
            bad = None
            for i, j in zip(path[:-1], path[1:]):
                e = (min(i, j), max(i, j))
                if edges[e] == "unknown":
                    qa = nodes[i]
                    qb = (nodes[j][0], nodes[j][1],
                          _theta_rep(nodes[j][2], qa[2]))
                    if ck.edge_free(qa, qb):
                        edges[e] = "free"
                    else:
                        kill_edge(e)
                        bad = e
                        break
                if ck.exhausted():
                    break
            if bad is not None:
                continue
            if any(edges[(min(i, j), max(i, j))] != "free"
                   for i, j in zip(path[:-1], path[1:])):
                break                      # budget died mid-validation
            if first_sampled is None:
                first_sampled = ck.nq
            wps = [nodes[path[0]]]
            for i, j in zip(path[:-1], path[1:]):
                wps.append((nodes[j][0], nodes[j][1],
                            _theta_rep(nodes[j][2], wps[-1][2])))
            wps = ck.polish(wps)
            ok, mmin = ck.certify(wps)
            if ok and in_region(region_L, wps[0]) \
                    and in_region(region_R, wps[-1]):
                return {"success": True, "nq": ck.nq,
                        "nq_first_sampled": first_sampled,
                        "n_states": len(nodes),
                        "cert_fail_events": cert_fails,
                        "cert_min_margin_mm": round(mmin * 1e3, 2),
                        "path": [list(p) for p in wps]}
            cert_fails += 1
            # kill the longest edge of the failed path and retry
            worst = max(zip(path[:-1], path[1:]),
                        key=lambda ij: _dist(a_max, nodes[ij[0]],
                                             nodes[ij[1]]))
            kill_edge((min(worst), max(worst)))
    return {"success": False, "nq": ck.nq,
            "nq_first_sampled": first_sampled, "n_states": len(nodes),
            "cert_fail_events": cert_fails,
            "cert_min_margin_mm": None, "path": None}


# ---- arm 3: HRM-style SE(2) sweep roadmap ---------------------------------

def hrm_se2(scene, robot, region_L, region_R, budget,
            n_theta0=8, n_col0=30, dy0=0.05, max_rounds=4):
    """Highway-RoadMap-style decomposition: fixed theta layers over
    [0, pi); per layer, vertical sweep columns across the joint bbox with
    free intervals detected along y (vectorized, every sample billed);
    intervals connect within a layer (adjacent columns, y-overlap) and
    across adjacent theta layers (same column, y-overlap; pi-periodic).
    Resolution doubles each round until success or budget.  Deterministic."""
    ck = BilledChecker(scene, robot, budget)
    x0, x1, y0, y1 = joint_bbox(region_L, region_R)
    first_sampled = None
    cert_fails = 0
    n_states_last = 0
    for rnd in range(max_rounds):
        n_theta = n_theta0 * (2 ** rnd)
        n_col = n_col0 * (2 ** rnd)
        dy = dy0 / (2 ** rnd)
        thetas = np.arange(n_theta) * (np.pi / n_theta)
        cols = np.linspace(x0, x1, n_col)
        ys = np.arange(y0, y1 + 1e-9, dy)
        cost = n_theta * n_col * len(ys)
        if ck.nq + cost > ck.budget:
            break
        # interval detection
        intervals = {}          # (it, ic) -> list of (ylo, yhi)
        for it, th in enumerate(thetas):
            for ic, cx in enumerate(cols):
                f = ck.free_batch(th, np.full(len(ys), cx), ys)
                runs, i0 = [], None
                for i, ok in enumerate(f):
                    if ok and i0 is None:
                        i0 = i
                    elif not ok and i0 is not None:
                        runs.append((float(ys[i0]), float(ys[i - 1])))
                        i0 = None
                if i0 is not None:
                    runs.append((float(ys[i0]), float(ys[-1])))
                if runs:
                    intervals[(it, ic)] = runs
        # graph over intervals
        node_id = {}
        for k, runs in intervals.items():
            for r, iv in enumerate(runs):
                node_id[k + (r,)] = len(node_id)
        n_states_last = len(node_id)
        adj = [[] for _ in range(len(node_id))]

        def overlap(a, b):
            return min(a[1], b[1]) - max(a[0], b[0]) > -1e-12

        for (it, ic), runs in intervals.items():
            for r, iv in enumerate(runs):
                u = node_id[(it, ic, r)]
                for it2, ic2 in (((it, ic + 1)), ((it + 1) % n_theta, ic)):
                    for r2, iv2 in enumerate(intervals.get((it2, ic2), ())):
                        if overlap(iv, iv2):
                            v = node_id[(it2, ic2, r2)]
                            adj[u].append(v)
                            adj[v].append(u)
        # terminal sets by region membership of interval midpoints
        starts, goals = [], []
        mid = {}
        for (it, ic, r), u in node_id.items():
            th = float(thetas[it])
            iv = intervals[(it, ic)][r]
            q = (float(cols[ic]), 0.5 * (iv[0] + iv[1]), th)
            mid[u] = q
            if in_region(region_L, q):
                starts.append(u)
            if in_region(region_R, q):
                goals.append(u)
        if starts and goals:
            # BFS
            prev = {u: None for u in starts}
            queue = list(starts)
            hit = None
            gset = set(goals)
            while queue:
                u = queue.pop(0)
                if u in gset:
                    hit = u
                    break
                for v in adj[u]:
                    if v not in prev:
                        prev[v] = u
                        queue.append(v)
            if hit is not None:
                if first_sampled is None:
                    first_sampled = ck.nq
                chain = []
                u = hit
                while u is not None:
                    chain.append(mid[u])
                    u = prev[u]
                chain = chain[::-1]
                wps = [chain[0]]
                for q in chain[1:]:
                    wps.append((q[0], q[1], _theta_rep(q[2], wps[-1][2])))
                wps = ck.polish(wps)
                ok, mmin = ck.certify(wps)
                if ok and in_region(region_L, wps[0]) \
                        and in_region(region_R, wps[-1]):
                    return {"success": True, "nq": ck.nq,
                            "nq_first_sampled": first_sampled,
                            "n_states": len(node_id),
                            "cert_fail_events": cert_fails,
                            "cert_min_margin_mm": round(mmin * 1e3, 2),
                            "path": [list(p) for p in wps],
                            "rounds": rnd + 1}
                cert_fails += 1
    return {"success": False, "nq": ck.nq,
            "nq_first_sampled": first_sampled, "n_states": n_states_last,
            "cert_fail_events": cert_fails,
            "cert_min_margin_mm": None, "path": None}
