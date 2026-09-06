"""Independent validation battery for Sprint A (user-requested audit).

Principle: every conclusion is re-derived through a path that shares as little
code as possible with the production pipeline:

  V1  scene mirror exactness (generator coordinates are k*step)
  V2  brute-force oracle: checker #2 + triangle-inequality bounds only,
      ALL theta slices (no PW, no KD-tree, no symmetry mirror) vs production
  V3  Monte-Carlo footprint sampling as a third, dumbest checker
      (random poses + near-boundary shell poses)
  V4  free-mask symmetry: free(x,y,th) == free(x,-y,-th) == free(-x,y,pi-th)
  V5  components: hand-rolled BFS + theta-roll invariance
  V6  Dijkstra: hand-rolled heapq implementation vs scipy (coarse)
  V7  path discipline: legal steps, cost re-summation, endpoint conditions,
      conservative swept certificate
  V8  mid-plane gate interval: brute-force checker #2 vs production vs analytic

Exit code 0 iff all checks pass.
Run:  cd splatc_atlas && PYTHONPATH=src python experiments/oracle/validate_sprint_a.py
"""
import heapq
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from splatc.datasets.g1_gate import (
    make_g1_scene, robot_library, gate_half_angle,
    Q_START, GOAL, GOAL_RADIUS, ELL_R)
from splatc.gaussian_geometry.contact import (
    point_ellipse_distance, support_half_widths)
from splatc.reference.oracle import (
    make_grid, RESOLUTIONS, dense_oracle, components, shortest_path,
    certify_path_conservative, pose_to_index, gate_interval_numeric)
from splatc.common.se2 import world_to_body, wrap_diff, TWO_PI

RNG = np.random.default_rng(20260812)
FAILURES = []


def report(name, ok, detail=""):
    tag = "PASS" if ok else "FAIL"
    print(f"[{tag}] {name}  {detail}", flush=True)
    if not ok:
        FAILURES.append(name)


def all_centers(scene):
    out = []
    for R, centers in scene.disc_groups:
        out.append((R, centers))
    return out


# ---------------------------------------------------------------------------
# brute-force margin: checker #2 + provable triangle-inequality bounds only
# ---------------------------------------------------------------------------

def brute_margin_slice(scene, robot, theta, X, Y):
    """Metric margin for grid points, min over ALL discs, no KD-tree/PW."""
    m = np.full(X.size, np.inf)
    a, b = robot.a, robot.b
    for R, centers in scene.disc_groups:
        for cx, cy in centers:
            d = np.hypot(cx - X, cy - Y)
            # triangle bounds: d - (a+R) <= margin <= d - (b+R)
            lo = d - (a + R)
            need = m > lo  # only pairs that could lower current min
            amb = need & (lo < 0.05)  # ambiguous shell -> exact checker #2
            if np.any(amb):
                u1, u2 = world_to_body(theta, cx - X[amb], cy - Y[amb])
                if robot.is_circle:
                    exact = np.hypot(u1, u2) - (a + R)
                else:
                    dist = point_ellipse_distance(u1, u2, a, b)
                    inside = (np.abs(u1) / a) ** 2 + (np.abs(u2) / b) ** 2 <= 1.0
                    exact = np.where(inside, -R - 1e-12, dist - R)
                m[amb] = np.minimum(m[amb], exact)
            clear = need & ~amb
            m[clear] = np.minimum(m[clear], lo[clear])  # safe lower bound
    return m


def brute_free_slice(scene, robot, theta, X, Y):
    xmin, xmax, ymin, ymax = scene.workspace
    px, py = support_half_widths(robot.a, robot.b, theta)
    contained = ((X - px > xmin) & (X + px < xmax)
                 & (Y - py > ymin) & (Y + py < ymax))
    return contained & (brute_margin_slice(scene, robot, theta, X, Y) > 0.0), \
        brute_margin_slice.__name__


# ---------------------------------------------------------------------------

def main():
    scene = make_g1_scene(0.7)
    robots = robot_library()
    t_start = time.time()

    # V1 -- exact mirror symmetry of generated primitive coordinates
    ok = True
    for R, centers in scene.disc_groups:
        cset = {(x, y) for x, y in centers}
        ok &= all((-x, y) in cset for x, y in cset)
        ok &= all((x, -y) in cset for x, y in cset)
    report("V1 scene mirror exactness", ok,
           f"{scene.n_primitives} primitives")

    # V2 -- brute-force oracle vs production, coarse, ALL slices, 3 robots
    dx, nt = RESOLUTIONS["coarse"]
    grid = make_grid(scene.workspace, dx, nt)
    X, Y = np.meshgrid(grid.xs, grid.ys)
    Xf, Yf = X.ravel(), Y.ravel()
    coarse_free = {}
    for rid, robot in robots.items():
        free_prod, rho_prod = dense_oracle(scene, robot, grid)
        coarse_free[rid] = free_prod
        mism = knife = 0
        for k in range(nt):  # deliberately no symmetry mirror here
            theta = grid.thetas[k]
            xmin, xmax, ymin, ymax = scene.workspace
            px, py = support_half_widths(robot.a, robot.b, theta)
            contained = ((Xf - px > xmin + 1e-9) & (Xf + px < xmax - 1e-9)
                         & (Yf - py > ymin + 1e-9) & (Yf + py < ymax - 1e-9))
            margin = brute_margin_slice(scene, robot, theta, Xf, Yf)
            free_b = contained & (margin > 1e-9)  # same tie-break as contract
            diff = free_b != free_prod[k].ravel()
            if diff.any():
                knife_mask = np.abs(margin[diff]) < 1e-6
                knife += int(knife_mask.sum())
                mism += int(diff.sum() - knife_mask.sum())
        report(f"V2 brute-force oracle == production ({rid})",
               mism == 0, f"hard mismatches={mism}, knife-edge(|m|<1e-6)={knife}")

    # V3 -- Monte-Carlo footprint third checker
    robot = robots["R_long_ellipse"]
    phis = np.linspace(0, TWO_PI, 240, endpoint=False)
    rads = np.array([1.0, 0.999, 0.9, 0.6, 0.3])
    bx = np.outer(rads, robot.a * np.cos(phis)).ravel()
    by = np.outer(rads, robot.b * np.sin(phis)).ravel()

    def mc_penetrates(q):
        c, s = np.cos(q[2]), np.sin(q[2])
        Xs = q[0] + c * bx - s * by
        Ys = q[1] + s * bx + c * by
        xmin, xmax, ymin, ymax = scene.workspace
        if (Xs.min() < xmin - 1e-12 or Xs.max() > xmax + 1e-12
                or Ys.min() < ymin - 1e-12 or Ys.max() > ymax + 1e-12):
            return True
        for R, centers in scene.disc_groups:
            for cx, cy in centers:
                if np.any(np.hypot(cx - Xs, cy - Ys) < R - 1e-9):
                    return True
        return False

    n_bad = n_inconclusive = 0
    poses = [(RNG.uniform(-3.5, 3.5), RNG.uniform(-2.3, 2.3),
              RNG.uniform(0, TWO_PI)) for _ in range(600)]
    # add near-boundary shell poses from the coarse rho map
    free_prod, rho_prod = dense_oracle(scene, robot, grid)
    shell = np.argwhere(np.abs(rho_prod) < 0.02)
    if len(shell):
        sel = shell[RNG.choice(len(shell), min(200, len(shell)), replace=False)]
        poses += [(grid.xs[i], grid.ys[j], grid.thetas[k]) for k, j, i in sel]
    for q in poses:
        prod_free, prod_rho = scene.check_pose(robot, q)
        pen = mc_penetrates(q)
        if pen and prod_free:
            n_bad += 1  # sampled proof of collision but production says free
        elif (not pen) and (not prod_free):
            if abs(prod_rho) > 5e-3:
                n_bad += 1  # deep collision claimed but no sample penetrates
            else:
                n_inconclusive += 1  # thin penetration below sampling density
    report("V3 Monte-Carlo footprint checker", n_bad == 0,
           f"{len(poses)} poses, contradictions={n_bad}, "
           f"thin-shell inconclusive={n_inconclusive}")

    # V4 -- free-mask symmetries (coarse + medium, ellipse)
    for res in ("coarse", "medium"):
        dxr, ntr = RESOLUTIONS[res]
        gr = make_grid(scene.workspace, dxr, ntr)
        fr, _ = dense_oracle(scene, robot, gr)
        # y -> -y, theta -> -theta
        sym1 = fr[(-np.arange(ntr)) % ntr][:, ::-1, :]
        # x -> -x, theta -> pi - theta
        sym2 = fr[(ntr // 2 - np.arange(ntr)) % ntr][:, :, ::-1]
        report(f"V4 mask symmetry ({res})",
               bool(np.array_equal(fr, sym1) and np.array_equal(fr, sym2)),
               f"y-mirror diff={int((fr != sym1).sum())}, "
               f"x-mirror diff={int((fr != sym2).sum())}")

    # V5 -- components: hand-rolled BFS + roll invariance (coarse ellipse)
    fr = coarse_free["R_long_ellipse"]
    labels_prod, n_prod = components(fr)
    nt_, ny_, nx_ = fr.shape
    seen = np.zeros(fr.shape, dtype=bool)
    sizes_bfs = []
    from collections import deque
    for start in zip(*np.nonzero(fr)):
        if seen[start]:
            continue
        qd, size = deque([start]), 0
        seen[start] = True
        while qd:
            k, j, i = qd.popleft()
            size += 1
            for dk, dj, di in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0),
                               (0, 0, 1), (0, 0, -1)):
                k2 = (k + dk) % nt_  # periodic theta
                j2, i2 = j + dj, i + di
                if 0 <= j2 < ny_ and 0 <= i2 < nx_ and fr[k2, j2, i2] \
                        and not seen[k2, j2, i2]:
                    seen[k2, j2, i2] = True
                    qd.append((k2, j2, i2))
        sizes_bfs.append(size)
    sizes_prod = sorted(np.bincount(labels_prod.ravel())[1:], reverse=True)
    ok = (len(sizes_bfs) == n_prod
          and sorted(sizes_bfs, reverse=True) == list(sizes_prod))
    report("V5a components BFS == scipy+wrap-merge", ok,
           f"n={n_prod} sizes={sizes_prod[:3]}")
    _, n_roll = components(np.roll(fr, 7, axis=0))
    report("V5b components roll-invariance", n_roll == n_prod,
           f"{n_prod} vs rolled {n_roll}")

    # V6 -- hand-rolled Dijkstra vs scipy (coarse)
    poses_s, cost_s = shortest_path(fr, grid, Q_START, GOAL, GOAL_RADIUS, ELL_R)
    offs = [(0, 0, 1), (0, 1, 0), (0, 0, -1), (0, -1, 0),
            (0, 1, 1), (0, 1, -1), (0, -1, 1), (0, -1, -1),
            (1, 0, 0), (-1, 0, 0)]
    wcost = {o: np.hypot(o[2] * grid.dx, o[1] * grid.dy)
             + (ELL_R * grid.dtheta if o[0] else 0.0) for o in offs}
    ks, js, is_ = pose_to_index(grid, Q_START)
    dist = {(ks, js, is_): 0.0}
    pq = [(0.0, (ks, js, is_))]
    gx, gy = GOAL
    best = np.inf
    while pq:
        d, (k, j, i) = heapq.heappop(pq)
        if d > dist.get((k, j, i), np.inf):
            continue
        if (grid.xs[i] - gx) ** 2 + (grid.ys[j] - gy) ** 2 <= GOAL_RADIUS ** 2:
            best = min(best, d)
        for o in offs:
            k2 = (k + o[0]) % nt_
            j2, i2 = j + o[1], i + o[2]
            if 0 <= j2 < ny_ and 0 <= i2 < nx_ and fr[k2, j2, i2]:
                nd = d + wcost[o]
                if nd < dist.get((k2, j2, i2), np.inf):
                    dist[(k2, j2, i2)] = nd
                    heapq.heappush(pq, (nd, (k2, j2, i2)))
    report("V6 hand-rolled Dijkstra == scipy", abs(best - cost_s) < 1e-9,
           f"manual={best:.6f} scipy={cost_s:.6f}")

    # V7 -- path discipline on the medium path
    dxm, ntm = RESOLUTIONS["medium"]
    gm = make_grid(scene.workspace, dxm, ntm)
    fm, _ = dense_oracle(scene, robot, gm)
    poses_m, cost_m = shortest_path(fm, gm, Q_START, GOAL, GOAL_RADIUS, ELL_R)
    legal = True
    total = 0.0
    for p, q in zip(poses_m[:-1], poses_m[1:]):
        di = int(round((q[0] - p[0]) / gm.dx))
        dj = int(round((q[1] - p[1]) / gm.dy))
        dk = int(round(wrap_diff(q[2] - p[2]) / gm.dtheta))
        legal &= (dk, dj, di) in offs
        total += np.hypot(q[0] - p[0], q[1] - p[1]) \
            + ELL_R * abs(wrap_diff(q[2] - p[2]))
    k0, j0, i0 = pose_to_index(gm, Q_START)
    start_ok = np.allclose(poses_m[0], [gm.xs[i0], gm.ys[j0], gm.thetas[k0]])
    goal_ok = (poses_m[-1][0] - GOAL[0]) ** 2 + (poses_m[-1][1] - GOAL[1]) ** 2 \
        <= GOAL_RADIUS ** 2
    on_free = all(fm[pose_to_index(gm, q)] for q in poses_m)
    cert, min_m, depth, checks = certify_path_conservative(scene, robot, poses_m)
    report("V7 path discipline + conservative certificate",
           bool(legal and start_ok and goal_ok and on_free and cert
                and abs(total - cost_m) < 1e-9),
           f"legal_steps={legal} cost_resum_ok={abs(total-cost_m)<1e-9} "
           f"certified={cert} min_margin={min_m*1000:.2f}mm")

    # V8 -- mid-plane gate interval brute force vs production vs analytic
    for w in (0.51, 0.7):
        sc = make_g1_scene(w)
        n_th = 720
        ths = np.arange(n_th) * (TWO_PI / n_th)
        ny2 = int(round((w + 0.1) / 0.005))
        ny2 += ny2 % 2  # odd point count -> includes y=0
        ys = np.linspace(-w / 2 - 0.05, w / 2 + 0.05, ny2 + 1)
        Xz = np.zeros_like(ys)
        passable_b = np.zeros(n_th, dtype=bool)
        for k in range(n_th):  # no symmetry shortcut
            px, py = support_half_widths(robot.a, robot.b, ths[k])
            contained = ((Xz - px > sc.workspace[0] + 1e-9) & (Xz + px < sc.workspace[1] - 1e-9)
                         & (ys - py > sc.workspace[2] + 1e-9) & (ys + py < sc.workspace[3] - 1e-9))
            margin = brute_margin_slice(sc, robot, ths[k], Xz, ys)
            passable_b[k] = bool(np.any(contained & (margin > 1e-9)))
        th_f = np.where(ths >= np.pi, ths - TWO_PI, ths)
        fold = np.abs(np.where(np.abs(th_f) > np.pi / 2, np.pi - np.abs(th_f), th_f))
        num_b = np.degrees(np.max(fold[passable_b])) if passable_b.any() else 0.0
        pass_p, ths_p = gate_interval_numeric(sc, robot, w, n_theta=n_th)
        agree = bool(np.array_equal(passable_b, pass_p))
        ana = np.degrees(gate_half_angle(robot.a, robot.b, w))
        report(f"V8 mid-plane gate brute==prod (w={w})", agree,
               f"brute={num_b:.2f}° analytic={ana:.2f}° "
               f"set_diff={int((passable_b != pass_p).sum())}")

    print(f"\n== validation battery done in {time.time()-t_start:.0f}s: "
          f"{'ALL PASS' if not FAILURES else 'FAILURES: ' + ', '.join(FAILURES)}")
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main())
