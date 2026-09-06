"""Compression-Topology-Runtime Pareto (design revision 14.1).

Answers project risk R1: does a mobility compression signal exist?

For each arm we compile fixed-orientation slices and bisect for the largest
orientation at which the robot can still translate through the door in
*certified safe* free space.  The single-door family has closed-form truth
(synth.gate_half_angle), so the gate threshold is checkable against an external
analytic value, not only against the uncompressed compiler.

Arms
----
full      : uncompressed scene (reference)
coreset   : robot-conditioned clearance-bounded coarsener, swept over eps
uniform   : same hierarchy and same certified macro construction cut to the
            *same primitive count*, but with no robot conditioning -- the
            control that isolates how much of the win comes from conditioning
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from gmc.config import load_config
from gmc.coreset import CoarsenParams, coarsen_scene, uniform_cut
from gmc.coreset.hierarchy import build_hierarchy
from gmc.io.robot_io import ellipse_robot
from gmc.spatial.slice_compiler import build_slice
from gmc.synth import DISC_R, SPACING, gate_half_angle, single_door
from gmc.types import GaussianSupport2D, SceneModel2D

BISECT_ITERS = 13          # pi/2 / 2^13 = 1.9e-4 rad resolution
START = (-2.0, 0.0)
GOAL = (2.0, 0.0)


def surface_scene(scene: SceneModel2D) -> SceneModel2D:
    """Keep only boundary splats of a volume-filled scene.

    Real 3DGS puts primitives on visible *surfaces*, not through the interior
    of solids.  The synthetic families fill rectangles, which makes them an
    unrealistically compressible testbed in one direction (interior
    redundancy) and an unrealistically hard one in another (a filled rectangle
    is a poor fit for ellipses).  This variant isolates the surface regime.
    """
    means = np.array([s.mean for s in scene.supports], dtype=np.float64)
    d = np.linalg.norm(means[:, None, :] - means[None, :, :], axis=-1)
    neighbours = ((d > 0.0) & (d <= SPACING * 1.05)).sum(axis=1)
    keep = [i for i in range(len(scene.supports)) if neighbours[i] < 4]
    supports = tuple(GaussianSupport2D(scene.supports[i].mean,
                                       scene.supports[i].covariance,
                                       scene.supports[i].level, k)
                     for k, i in enumerate(keep))
    return SceneModel2D(supports=supports, workspace=scene.workspace,
                        name=scene.name + "_surface")


def connected(scene, robot, cfg, theta) -> tuple[bool, int]:
    sl = build_slice(scene, robot, theta, cfg)
    a = sl.locate(START, "safe")
    b = sl.locate(GOAL, "safe")
    ok = a is not None and b is not None and a.component_id == b.component_id
    return ok, sl.support_calls


def gate_threshold(scene, robot, cfg) -> dict:
    """Largest theta with certified-safe start-goal connectivity."""
    t0 = time.time()
    calls = 0
    open_at_zero, c = connected(scene, robot, cfg, 0.0)
    calls += c
    if not open_at_zero:
        return {"threshold": 0.0, "sealed": True,
                "support_calls": calls, "wall_seconds": time.time() - t0}
    lo, hi = 0.0, math.pi / 2.0
    top, c = connected(scene, robot, cfg, hi)
    calls += c
    if top:
        return {"threshold": hi, "sealed": False,
                "support_calls": calls, "wall_seconds": time.time() - t0}
    for _ in range(BISECT_ITERS):
        mid = 0.5 * (lo + hi)
        ok, c = connected(scene, robot, cfg, mid)
        calls += c
        if ok:
            lo = mid
        else:
            hi = mid
    return {"threshold": 0.5 * (lo + hi), "sealed": False,
            "support_calls": calls, "wall_seconds": time.time() - t0}


def run(scene, robot, cfg, targets, label, truth) -> list[dict]:
    rows = []
    root = build_hierarchy(scene)
    ref = gate_threshold(scene, robot, cfg)
    rows.append({"scene": label, "arm": "full",
                 "n": len(scene.supports), "compression": 1.0,
                 "target_gate_tol": None,
                 "min_slack": None, "coarsen_seconds": 0.0, **ref})
    print(f"[{label}] full n={len(scene.supports)} "
          f"gate={ref['threshold']:.5f} truth={truth:.5f} "
          f"calls={ref['support_calls']} {ref['wall_seconds']:.1f}s",
          flush=True)
    for tgt in targets:
        t0 = time.time()
        res = coarsen_scene(scene, robot,
                            CoarsenParams(clearance_tol=0.002,
                                          target_gate_tol=tgt),
                            hierarchy=root)
        coarsen_s = time.time() - t0
        gt = gate_threshold(res.scene, robot, cfg)
        rows.append({"scene": label, "arm": "coreset", "target_gate_tol": tgt,
                     "n": res.n_macro, "compression": res.compression,
                     "min_slack": res.min_slack,
                     "coarsen_seconds": coarsen_s, **gt})
        print(f"[{label}] coreset gate_tol={tgt:<6} n={res.n_macro:4d} "
              f"ratio={res.compression:.4f} gate={gt['threshold']:.5f} "
              f"err={gt['threshold'] - truth:+.5f} "
              f"calls={gt['support_calls']} {gt['wall_seconds']:.2f}s",
              flush=True)

        t0 = time.time()
        ctrl = uniform_cut(scene, res.n_macro, hierarchy=root)
        ctrl_s = time.time() - t0
        gtc = gate_threshold(ctrl.scene, robot, cfg)
        rows.append({"scene": label, "arm": "uniform", "target_gate_tol": tgt,
                     "n": ctrl.n_macro, "compression": ctrl.compression,
                     "min_slack": ctrl.min_slack,
                     "coarsen_seconds": ctrl_s, **gtc})
        print(f"[{label}] uniform n={ctrl.n_macro:4d} "
              f"ratio={ctrl.compression:.4f} gate={gtc['threshold']:.5f} "
              f"err={gtc['threshold'] - truth:+.5f} "
              f"calls={gtc['support_calls']} {gtc['wall_seconds']:.2f}s",
              flush=True)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/toy.yaml")
    ap.add_argument("--width", type=float, default=0.6)
    ap.add_argument("--robot", type=float, nargs=2, default=(0.50, 0.20))
    ap.add_argument("--targets", type=float, nargs="*",
                    default=[0.002, 0.005, 0.01, 0.02, 0.05, 0.10])
    ap.add_argument("-o", "--output", default="results/coreset/pareto.json")
    args = ap.parse_args()

    cfg = load_config(args.config)
    a, b = args.robot
    robot = ellipse_robot(a, b)
    truth = gate_half_angle(a, b, args.width)
    filled = single_door(width=args.width)
    surface = surface_scene(filled)
    print(f"analytic gate half-angle = {truth:.6f} rad  "
          f"(robot a={a} b={b}, door w={args.width}, disc_r={DISC_R})",
          flush=True)

    rows = []
    rows += run(filled, robot, cfg, args.targets, "filled", truth)
    rows += run(surface, robot, cfg, args.targets, "surface", truth)

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(
        {"analytic_gate": truth, "robot": {"a": a, "b": b},
         "door_width": args.width, "config": args.config,
         "bisect_iters": BISECT_ITERS, "targets": args.targets,
         "rows": rows}, indent=2))
    print("wrote", out, flush=True)


if __name__ == "__main__":
    main()
