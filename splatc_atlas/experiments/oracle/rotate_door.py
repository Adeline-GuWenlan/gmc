"""Sprint A decisive experiment: rotate-through-door + three morphologies.

Order follows the top-level README meta-rule: emit human-viewable artifacts
(slice mosaics, path overlay) BEFORE trusting any scalar metric.

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/oracle/rotate_door.py [medium|fine]
"""
import json
import sys
import time
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse as MplEllipse, Circle as MplCircle

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from splatc.datasets.g1_gate import (
    make_g1_scene, robot_library, Q_START, GOAL, GOAL_RADIUS, ELL_R,
    gate_half_angle)
from splatc.reference.oracle import (
    make_grid, RESOLUTIONS, dense_oracle, reachability, shortest_path,
    certify_path)
from splatc.common.se2 import wrap_diff

W = 0.7
FIGDIR = os.path.join(os.path.dirname(__file__), "..", "..", "results", "figures")
TABDIR = os.path.join(os.path.dirname(__file__), "..", "..", "results", "tables")


def draw_scene(ax, scene):
    for R, centers in scene.disc_groups:
        for cx, cy in centers:
            ax.add_patch(MplCircle((cx, cy), R, color="0.55", lw=0))
    xmin, xmax, ymin, ymax = scene.workspace
    ax.plot([xmin, xmax, xmax, xmin, xmin],
            [ymin, ymin, ymax, ymax, ymin], "k-", lw=1.5)
    ax.set_aspect("equal")
    ax.set_xlim(xmin - 0.2, xmax + 0.2)
    ax.set_ylim(ymin - 0.2, ymax + 0.2)


def robot_patch(ax, robot, q, color, alpha=0.5):
    e = MplEllipse((q[0], q[1]), 2 * robot.a, 2 * robot.b,
                   angle=np.degrees(q[2]), facecolor=color, alpha=alpha,
                   edgecolor="k", lw=0.6)
    ax.add_patch(e)


def slice_mosaic(scene, robot, free, grid, fname):
    idx = [0, len(grid.thetas) // 12, len(grid.thetas) // 8, len(grid.thetas) // 4]
    fig, axes = plt.subplots(1, len(idx), figsize=(18, 3.4))
    for ax, k in zip(axes, idx):
        ax.imshow(free[k], origin="lower", cmap="Greens",
                  extent=[grid.xs[0], grid.xs[-1], grid.ys[0], grid.ys[-1]],
                  interpolation="nearest", vmin=0, vmax=1.2)
        for R, centers in scene.disc_groups:
            ax.scatter(centers[:, 0], centers[:, 1], s=1, c="0.4")
        ax.set_title(f"{robot.robot_id}  θ={np.degrees(grid.thetas[k]):.1f}°")
        ax.set_aspect("equal")
    fig.suptitle("free(x,y | θ) slices — green = free")
    fig.tight_layout()
    fig.savefig(fname, dpi=110)
    plt.close(fig)


def main(res_name="medium"):
    os.makedirs(FIGDIR, exist_ok=True)
    os.makedirs(TABDIR, exist_ok=True)
    dx, ntheta = RESOLUTIONS[res_name]
    scene = make_g1_scene(W)
    grid = make_grid(scene.workspace, dx, ntheta)
    robots = robot_library()
    print(f"scene {scene.scene_id}: {scene.n_primitives} primitives; "
          f"grid {grid.shape} ({res_name})")

    record = {"scene": scene.meta, "resolution": res_name,
              "grid": list(grid.shape), "robots": {}}

    long_free = None
    for rid, robot in robots.items():
        t0 = time.time()
        free, rho = dense_oracle(scene, robot, grid)
        t_or = time.time() - t0
        reach, labels, ncomp, s_lab = reachability(
            free, grid, Q_START, GOAL, GOAL_RADIUS)
        print(f"{rid}: oracle {t_or:.1f}s  free%={100*free.mean():.1f} "
              f"components={ncomp} reachable={reach}")
        # visualize FIRST
        slice_mosaic(scene, robot, free, grid,
                     os.path.join(FIGDIR, f"slices_{rid}_{res_name}.png"))
        record["robots"][rid] = {
            "oracle_seconds": round(t_or, 2), "free_fraction": float(free.mean()),
            "n_components": int(ncomp), "reachable": bool(reach)}
        if rid == "R_long_ellipse":
            long_free = free

    # rotate-through-door path for the long ellipse
    robot = robots["R_long_ellipse"]
    t0 = time.time()
    poses, cost = shortest_path(long_free, grid, Q_START, GOAL, GOAL_RADIUS, ELL_R)
    t_dij = time.time() - t0
    assert poses is not None, "no path found for long ellipse — investigate!"
    ok, min_rho = certify_path(scene, robot, poses)
    print(f"path: {len(poses)} poses cost={cost:.3f} dijkstra {t_dij:.1f}s "
          f"certified={ok} min_rho={min_rho:.4f}")
    record["robots"]["R_long_ellipse"].update(
        {"path_cost": cost, "path_poses": len(poses),
         "path_certified": bool(ok), "path_min_rho": float(min_rho),
         "dijkstra_seconds": round(t_dij, 2)})

    # figure: footprints along the path
    fig, ax = plt.subplots(figsize=(11, 7))
    draw_scene(ax, scene)
    step = max(1, len(poses) // 28)
    cmap = plt.get_cmap("viridis")
    for n, q in enumerate(poses[::step]):
        robot_patch(ax, robot, q, cmap(n / max(1, len(poses[::step]) - 1)), 0.45)
    ax.plot(poses[:, 0], poses[:, 1], "r-", lw=1.2)
    robot_patch(ax, robot, Q_START, "tab:blue", 0.9)
    ax.add_patch(MplCircle(GOAL, GOAL_RADIUS, fill=False, color="tab:red", lw=2))
    th = gate_half_angle(robot.a, robot.b, W)
    ax.set_title(f"rotate-through-door  w={W}  cost={cost:.2f}  "
                 f"analytic gate ±{np.degrees(th):.1f}°  certified={ok}")
    fig.tight_layout()
    fig.savefig(os.path.join(FIGDIR, f"rotate_door_path_{res_name}.png"), dpi=130)
    plt.close(fig)

    # figure: theta and rho profile along the path
    s = np.concatenate([[0.0], np.cumsum(
        np.hypot(np.diff(poses[:, 0]), np.diff(poses[:, 1])))])
    theta_unwrapped = np.concatenate(
        [[poses[0, 2]], poses[0, 2] + np.cumsum(wrap_diff(np.diff(poses[:, 2])))])
    fig, ax = plt.subplots(figsize=(9, 3.2))
    ax.plot(s, np.degrees(theta_unwrapped), "b-")
    ax.axhspan(-np.degrees(th), np.degrees(th), color="g", alpha=0.15,
               label="analytic gate ±θmax (mod 180°)")
    in_door = np.abs(poses[:, 0]) <= 0.6
    if in_door.any():
        ax.axvspan(s[in_door][0], s[in_door][-1], color="r", alpha=0.12,
                   label="inside corridor |x|<=0.6")
    ax.set_xlabel("arclength along path (m)")
    ax.set_ylabel("heading θ (deg, unwrapped)")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGDIR, f"rotate_door_theta_{res_name}.png"), dpi=130)
    plt.close(fig)

    with open(os.path.join(TABDIR, f"rotate_door_{res_name}.json"), "w") as f:
        json.dump(record, f, indent=2)
    print("record + figures written")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "medium")
