"""Visualise what the coarsener actually did (project README meta-rule 2:
emit a human-inspectable artifact *before* trusting any scalar)."""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from gmc.coreset import CoarsenParams, coarsen_scene, uniform_cut
from gmc.coreset.hierarchy import build_hierarchy
from gmc.coreset.macro import shape_matrix
from gmc.io.robot_io import ellipse_robot
from gmc.synth import single_door


def draw(ax, scene, color, lw, alpha, fill):
    for s in scene.supports:
        eigval, eigvec = np.linalg.eigh(shape_matrix(s))
        semi = np.sqrt(np.maximum(eigval, 0.0))
        t = np.linspace(0, 2 * np.pi, 128)
        pts = (eigvec @ np.column_stack([semi[0] * np.cos(t),
                                         semi[1] * np.sin(t)]).T).T + s.mean
        ax.plot(*pts.T, color=color, lw=lw, alpha=alpha)
        if fill:
            ax.fill(*pts.T, color=color, alpha=0.10, lw=0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=float, default=0.6)
    ap.add_argument("-o", "--output", default="results/coreset/figures")
    args = ap.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)

    scene = single_door(width=args.width)
    root = build_hierarchy(scene)
    robot = ellipse_robot(0.50, 0.20)
    panels = [("original", scene, None)]
    for eps, tol in ((0.002, 0.004), (0.01, 0.004), (0.02, 0.01)):
        res = coarsen_scene(scene, robot,
                            CoarsenParams(clearance_tol=eps, passage_tol=tol),
                            hierarchy=root)
        panels.append((f"coreset eps={eps} n={res.n_macro}", res.scene, res))
    ctrl = uniform_cut(scene, panels[1][2].n_macro, hierarchy=root)
    panels.append((f"uniform control n={ctrl.n_macro}", ctrl.scene, ctrl))

    fig, axes = plt.subplots(1, len(panels), figsize=(4.2 * len(panels), 5.0))
    for ax, (title, sc, _) in zip(axes, panels):
        draw(ax, scene, "0.75", 0.6, 0.9, False)
        if sc is not scene:
            draw(ax, sc, "tab:red", 1.4, 0.95, True)
        ax.axhline(args.width / 2, color="tab:blue", ls=":", lw=1.0)
        ax.axhline(-args.width / 2, color="tab:blue", ls=":", lw=1.0)
        ax.set_title(title, fontsize=10)
        ax.set_xlim(-1.6, 1.6)
        ax.set_ylim(-1.6, 1.6)
        ax.set_aspect("equal")
    fig.suptitle("Grey = original 484 supports.  Red = coreset macro ellipses.  "
                 "Blue dotted = door edges (w=%.2f)" % args.width, fontsize=11)
    fig.tight_layout()
    path = out / "coreset_overlay.png"
    fig.savefig(path, dpi=140)
    print("wrote", path)


if __name__ == "__main__":
    main()
