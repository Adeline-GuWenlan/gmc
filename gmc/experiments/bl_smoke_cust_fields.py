"""bl B0 smoke test: run cust_fields' own `test_nf.py` (plain navigation function on CONFIG/world_demo.yaml)
unchanged, headless. The only difference from `python test_nf.py` is that `plt.show()` saves the figure.

Usage (cust_fields env):  python -B bl_smoke_cust_fields.py --repo /scratch/wg2381/ext_repos/cust_fields --out DIR
"""
import argparse
import importlib.util
import json
import os
import sys
import time

sys.dont_write_bytecode = True  # keep the pristine clone free of __pycache__

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    fig_path = os.path.join(a.out, "test_nf.png")
    plt.show = lambda *args, **kw: plt.savefig(fig_path, dpi=110, bbox_inches="tight")

    spec = importlib.util.spec_from_file_location("test_nf", os.path.join(a.repo, "test_nf.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    # Same as test_nf.main(), but keep the trajectory so the summary can say whether the goal was reached.
    captured = {}
    orig_plot = mod.plot_result

    def plot_and_keep(world, trajectory):
        captured["traj"] = trajectory
        orig_plot(world, trajectory)

    mod.plot_result = plot_and_keep
    t0 = time.perf_counter()
    mod.main()
    wall = time.perf_counter() - t0

    traj = captured["traj"]
    end_err = float(((traj[-1] - mod.GOAL) ** 2).sum() ** 0.5)
    summary = {
        "demo": "cust_fields test_nf.py (CONFIG/world_demo.yaml)",
        "start": mod.START.tolist(), "goal": mod.GOAL.tolist(),
        "points": int(len(traj)), "path_length": mod.path_length(traj),
        "final_dist_to_goal": end_err, "goal_tol": mod.GOAL_TOL,
        "reached_goal": end_err < mod.GOAL_TOL, "wall_s": wall,
        "figure": fig_path, "python": sys.version.split()[0],
    }
    with open(os.path.join(a.out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1)
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
