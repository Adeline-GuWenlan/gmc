"""Dense SE(2) oracle (problem_spec.md §8).

Reference-side only: dense (theta, y, x) free/rho maps, connected components
with periodic theta, reachability labels, Dijkstra shortest path under the
frozen common cost, and continuous edge certification via the exact checker.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra as cs_dijkstra

from ..common.se2 import TWO_PI


@dataclass
class OracleGrid:
    xs: np.ndarray
    ys: np.ndarray
    thetas: np.ndarray

    @property
    def dx(self):
        return float(self.xs[1] - self.xs[0])

    @property
    def dy(self):
        return float(self.ys[1] - self.ys[0])

    @property
    def dtheta(self):
        return float(self.thetas[1] - self.thetas[0])

    @property
    def shape(self):
        return (len(self.thetas), len(self.ys), len(self.xs))


def make_grid(workspace, dx, n_theta):
    xmin, xmax, ymin, ymax = workspace
    nx = int(round((xmax - xmin) / dx)) + 1
    ny = int(round((ymax - ymin) / dx)) + 1
    return OracleGrid(np.linspace(xmin, xmax, nx),
                      np.linspace(ymin, ymax, ny),
                      np.arange(n_theta) * (TWO_PI / n_theta))


RESOLUTIONS = {  # name -> (dx, n_theta)
    "coarse": (0.10, 72),
    "medium": (0.05, 144),
    "fine": (0.025, 288),
}


def dense_oracle(scene, robot, grid, verbose=False):
    """Dense free/rho maps of shape (n_theta, ny, nx)."""
    nt, ny, nx = grid.shape
    X, Y = np.meshgrid(grid.xs, grid.ys)  # (ny, nx)
    free = np.empty((nt, ny, nx), dtype=bool)
    rho = np.empty((nt, ny, nx), dtype=np.float32)
    n_eval = nt // 2 if robot.symmetric and nt % 2 == 0 else nt
    for k in range(n_eval):
        f, r = scene.eval_points(robot, grid.thetas[k], X, Y)
        free[k] = f.reshape(ny, nx)
        rho[k] = r.reshape(ny, nx).astype(np.float32)
        if verbose and k % 16 == 0:
            print(f"  slice {k}/{n_eval}", flush=True)
    if n_eval < nt:  # centro-symmetric body: h(theta+pi) = h(theta)
        free[n_eval:] = free[:nt - n_eval]
        rho[n_eval:] = rho[:nt - n_eval]
    return free, rho


class _UnionFind:
    def __init__(self, n):
        self.p = list(range(n))

    def find(self, i):
        while self.p[i] != i:
            self.p[i] = self.p[self.p[i]]
            i = self.p[i]
        return i

    def union(self, i, j):
        ri, rj = self.find(i), self.find(j)
        if ri != rj:
            self.p[ri] = rj


def components(free):
    """6-connected labels with periodic theta (axis 0). Returns (labels, n)."""
    structure = ndimage.generate_binary_structure(3, 1)
    labels, n = ndimage.label(free, structure=structure)
    if n == 0:
        return labels, 0
    uf = _UnionFind(n + 1)
    l0, l1 = labels[0], labels[-1]
    both = (l0 > 0) & (l1 > 0)
    for a, b in set(zip(l0[both].tolist(), l1[both].tolist())):
        uf.union(a, b)
    remap = np.zeros(n + 1, dtype=np.int32)
    roots = {}
    for lab in range(1, n + 1):
        r = uf.find(lab)
        if r not in roots:
            roots[r] = len(roots) + 1
        remap[lab] = roots[r]
    return remap[labels], len(roots)


def pose_to_index(grid, q):
    x, y, theta = q
    i = int(round((x - grid.xs[0]) / grid.dx))
    j = int(round((y - grid.ys[0]) / grid.dy))
    k = int(round((theta % TWO_PI) / grid.dtheta)) % len(grid.thetas)
    return k, j, i


def goal_mask(grid, goal_xy, radius):
    X, Y = np.meshgrid(grid.xs, grid.ys)
    disk = (X - goal_xy[0]) ** 2 + (Y - goal_xy[1]) ** 2 <= radius ** 2
    return np.broadcast_to(disk, (len(grid.thetas),) + disk.shape)


def reachability(free, grid, q_start, goal_xy, radius):
    """Returns (reachable, labels, n_components, start_label)."""
    labels, n = components(free)
    k, j, i = pose_to_index(grid, q_start)
    if not free[k, j, i]:
        return False, labels, n, 0
    start_label = labels[k, j, i]
    gm = goal_mask(grid, goal_xy, radius) & free
    reach = bool(np.any(labels[gm] == start_label))
    return reach, labels, n, start_label


_XY_OFFSETS = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]


def _free_graph(free, grid, ell_r):
    nt, ny, nx = free.shape
    idx = -np.ones(free.shape, dtype=np.int64)
    nfree = int(free.sum())
    idx[free] = np.arange(nfree)
    rows, cols, data = [], [], []
    # xy moves (theta fixed); add each undirected edge once
    for dj, di in _XY_OFFSETS[:4] + [(1, 1), (1, -1)]:
        w = float(np.hypot(di * grid.dx, dj * grid.dy))
        src = idx[:, max(0, dj):ny - max(0, -dj), max(0, di):nx - max(0, -di)]
        dst = idx[:, max(0, -dj):ny - max(0, dj), max(0, -di):nx - max(0, di)]
        ok = (src >= 0) & (dst >= 0)
        rows.append(src[ok])
        cols.append(dst[ok])
        data.append(np.full(int(ok.sum()), w))
    # theta moves (periodic)
    w = float(ell_r * grid.dtheta)
    src = idx
    dst = np.roll(idx, -1, axis=0)
    ok = (src >= 0) & (dst >= 0)
    rows.append(src[ok])
    cols.append(dst[ok])
    data.append(np.full(int(ok.sum()), w))
    rows = np.concatenate(rows)
    cols = np.concatenate(cols)
    data = np.concatenate(data)
    g = coo_matrix((data, (rows, cols)), shape=(nfree, nfree)).tocsr()
    return g, idx


def shortest_path(free, grid, q_start, goal_xy, radius, ell_r):
    """Dijkstra under the frozen cost. Returns (poses, cost) or (None, inf).

    The search graph is restricted to the start's 6-connected component so
    that path existence can never contradict the (conservative) reachability
    label: diagonal xy moves are allowed only WITHIN a component, never as a
    bridge between components (problem_spec §8 adjacency contract)."""
    k, j, i = pose_to_index(grid, q_start)
    if not free[k, j, i]:
        return None, np.inf
    labels, _ = components(free)
    free = free & (labels == labels[k, j, i])
    g, idx = _free_graph(free, grid, ell_r)
    if idx[k, j, i] < 0:
        return None, np.inf
    start_node = idx[k, j, i]
    dist, pred = cs_dijkstra(g, directed=False, indices=start_node,
                             return_predecessors=True)
    gm = goal_mask(grid, goal_xy, radius) & (idx >= 0)
    goal_nodes = idx[gm]
    if goal_nodes.size == 0 or not np.any(np.isfinite(dist[goal_nodes])):
        return None, np.inf
    best = goal_nodes[np.argmin(dist[goal_nodes])]
    # backtrack
    chain = [best]
    while chain[-1] != start_node:
        chain.append(pred[chain[-1]])
    chain = chain[::-1]
    # node -> pose
    kji = np.argwhere(idx >= 0)
    order = idx[idx >= 0]
    lookup = np.empty((order.size, 3), dtype=np.int64)
    lookup[order] = kji
    poses = np.array([[grid.xs[i2], grid.ys[j2], grid.thetas[k2]]
                      for k2, j2, i2 in lookup[chain]])
    return poses, float(dist[best])


def certify_path(scene, robot, poses, n_sub=4):
    """Dense sampled check (NOT a conservative certificate — see
    certify_path_conservative). Returns (all_free, min_rho)."""
    from ..common.se2 import wrap_diff
    all_free, min_rho = True, np.inf
    for p, q in zip(poses[:-1], poses[1:]):
        dth = wrap_diff(q[2] - p[2])
        for t in np.linspace(0.0, 1.0, n_sub + 2):
            pose = (p[0] + t * (q[0] - p[0]),
                    p[1] + t * (q[1] - p[1]),
                    p[2] + t * dth)
            ok, r = scene.check_pose(robot, pose)
            all_free &= ok
            min_rho = min(min_rho, r)
    return all_free, min_rho


def metric_margin(scene, robot, q):
    """Metric clearance in meters: min over (disc margins via the independent
    checker #2, workspace box margins).  Lipschitz-1 w.r.t. body-point motion."""
    from ..gaussian_geometry.contact import support_half_widths
    x, y, theta = q
    _, m_disc = scene.check_pose_independent(robot, q)
    xmin, xmax, ymin, ymax = scene.workspace
    px, py = support_half_widths(robot.a, robot.b, theta)
    m_box = min(x - px - xmin, xmax - (x + px), y - py - ymin, ymax - (y + py))
    return min(m_disc, m_box)


def certify_path_conservative(scene, robot, poses, max_depth=14):
    """TRUE swept-edge certificate (plan 12.5): every body point moves at most
    D = |dp| + a_max*|dtheta| along a linearly interpolated segment, and metric
    clearance is 1-Lipschitz in that motion, so a segment with endpoint margins
    m1 + m2 > D is collision-free throughout (free-bubble argument).  Segments
    that do not close are bisected up to max_depth.

    Returns (certified, min_margin_m, max_depth_used, n_checks)."""
    from ..common.se2 import wrap_diff
    a_max = max(robot.a, robot.b)
    stats = {"min_m": np.inf, "depth": 0, "checks": 0}

    def margin(q):
        stats["checks"] += 1
        m = metric_margin(scene, robot, q)
        stats["min_m"] = min(stats["min_m"], m)
        return m

    def certify_seg(q1, q2, m1, m2, depth):
        if m1 <= 0.0 or m2 <= 0.0:
            return False
        dth = wrap_diff(q2[2] - q1[2])
        D = np.hypot(q2[0] - q1[0], q2[1] - q1[1]) + a_max * abs(dth)
        if m1 + m2 > D:
            return True
        if depth >= max_depth:
            return False
        stats["depth"] = max(stats["depth"], depth + 1)
        qm = (0.5 * (q1[0] + q2[0]), 0.5 * (q1[1] + q2[1]), q1[2] + 0.5 * dth)
        mm = margin(qm)
        return (certify_seg(q1, qm, m1, mm, depth + 1)
                and certify_seg(qm, q2, mm, m2, depth + 1))

    ok = True
    m_prev = margin(tuple(poses[0]))
    for p, q in zip(poses[:-1], poses[1:]):
        m_next = margin(tuple(q))
        if not certify_seg(tuple(p), tuple(q), m_prev, m_next, 0):
            ok = False
            break
        m_prev = m_next
    return ok, float(stats["min_m"]), int(stats["depth"]), int(stats["checks"])


def gate_interval_numeric(scene, robot, w, n_theta=1440, dy=0.005):
    """Measured gate set on the door mid-line:
    Theta_num = {theta : exists t with free(center + t*wall_dir, theta)}.
    Supports de-aligned doors via scene.meta door_offset / door_tilt.
    Returns bool array over theta and the theta grid."""
    tilt = scene.meta.get("door_tilt", 0.0)
    offset = scene.meta.get("door_offset", 0.0)
    # symmetric grid with an odd point count so t=0 is always included —
    # near critical width the admissible t-set shrinks to {0} (worklog 08-11)
    n_y = int(round((w + 0.1) / dy))
    if n_y % 2 == 1:
        n_y += 1
    ts = np.linspace(-w / 2 - 0.05, w / 2 + 0.05, n_y + 1)
    wall_dir = np.array([-np.sin(tilt), np.cos(tilt)])
    X = 0.0 + ts * wall_dir[0]
    Y = offset + ts * wall_dir[1]
    thetas = np.arange(n_theta) * (TWO_PI / n_theta)
    passable = np.zeros(n_theta, dtype=bool)
    half = n_theta // 2 if robot.symmetric else n_theta
    for k in range(half):
        f, _ = scene.eval_points(robot, thetas[k], X, Y)
        passable[k] = bool(f.any())
    if half < n_theta:
        passable[half:] = passable[:half]
    return passable, thetas
