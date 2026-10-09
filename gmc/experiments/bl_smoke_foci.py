"""bl B0 smoke test: FOCI's own `demos/stonehenge.py`, headless, on the patched copy (MA27 -> MUMPS).

Identical to the demo up to and including the six `planner.plan(start, end)` calls (same ply, same x20 scaling,
same robot_cov = 0.01 I, num_control_points=10, num_samples=40, same 12 ring points at z = 0.5). The demo's final
block opens a viser web server (`ViserVis().show()`); that is replaced by a matplotlib top view. Adds per-plan wall
time and IPOPT's return status (`planner.solver.stats()`), which the demo does not print.

Usage (foci env, GPU node): python -B bl_smoke_foci.py --repo .../foci_bl --out DIR
"""
import argparse
import json
import os
import sys
import time

sys.dont_write_bytecode = True

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--n_plans", type=int, default=6)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    os.chdir(a.out)

    import casadi
    import foci
    import warp as wp
    from foci.planners.planner import Planner  # sets warp device cuda:0 at import
    from foci.utils.ply import extract_splat_data

    assert os.path.realpath(os.path.dirname(foci.__file__)).startswith(os.path.realpath(a.repo)), foci.__file__
    info = {"foci_from": foci.__file__, "casadi": casadi.__version__, "warp": wp.config.version,
            "warp_device": str(wp.get_device())}
    print(info)

    ply_file = os.path.join(a.repo, "demos", "data", "stonehenge.ply")
    means, covs, colors, opacities = extract_splat_data(ply_file)
    means = 20 * means
    covs = 20 ** 2 * covs
    radius = max(np.linalg.norm(means, axis=1)) * 1.02
    robot_cov = np.eye(3) * 0.01
    t0 = time.perf_counter()
    planner = Planner(means, covs, robot_cov, num_control_points=10, num_samples=40)
    setup_s = time.perf_counter() - t0

    points = []
    for i in range(12):
        theta = i * 2 * np.pi / 12
        points.append([radius * np.cos(theta), radius * np.sin(theta), 0.5, np.pi / 2])

    rows, solutions = [], []
    for i in range(a.n_plans):
        start_point, end_point = points[i], points[i + 6]
        t1 = time.perf_counter()
        try:
            opt_curve, astar = planner.plan(start_point, end_point)
            st = planner.solver.stats()
            row = {"status": st.get("return_status"), "success": bool(st.get("success")),
                   "iter": st.get("iter_count"), "error": None}
            solutions.append((i, opt_curve, astar))
            row["end_error_m"] = float(np.linalg.norm(np.asarray(opt_curve)[-1, :3] - np.asarray(end_point[:3])))
            row["start_error_m"] = float(np.linalg.norm(np.asarray(opt_curve)[0, :3] - np.asarray(start_point[:3])))
        except Exception as e:
            row = {"status": None, "success": False, "error": f"{type(e).__name__}: {e}"}
        row.update({"i": i, "start": start_point, "end": end_point, "wall_s": time.perf_counter() - t1})
        rows.append(row)
        print(json.dumps(row))

    fig, ax = plt.subplots(figsize=(8, 8))
    keep = (opacities.ravel() > 0.5) & (means[:, 2] > 0) & (means[:, 2] < 1.5)
    ax.scatter(means[keep, 0], means[keep, 1], s=0.2, c="0.5", alpha=0.3, lw=0)
    for i, opt_curve, astar in solutions:
        c = f"C{i}"
        ax.plot(np.asarray(astar)[:, 0], np.asarray(astar)[:, 1], ":", color=c, lw=1)
        ax.plot(np.asarray(opt_curve)[:, 0], np.asarray(opt_curve)[:, 1], "-", color=c, lw=2, label=f"plan {i}: {rows[i]['status']}")
    P = np.array(points)
    ax.plot(P[:, 0], P[:, 1], "r.", ms=8)
    ax.set_aspect("equal"); ax.legend(fontsize=7, loc="lower left")
    ax.set_title("FOCI demos/stonehenge.py, headless (IPOPT linear solver MUMPS)\n"
                 "solid: optimised curve, dotted: A* spline init; grey: Gaussians (opacity>0.5, 0<z<1.5)")
    fig.tight_layout(); fig.savefig("foci_stonehenge.png", dpi=110)

    summary = {**info, "n_gaussians": int(len(means)), "setup_s": setup_s, "plans": rows,
               "figure": os.path.join(a.out, "foci_stonehenge.png")}
    with open("summary.json", "w") as f:
        json.dump(summary, f, indent=1, default=str)


if __name__ == "__main__":
    main()
