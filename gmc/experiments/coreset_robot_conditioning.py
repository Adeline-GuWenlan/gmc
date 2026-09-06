"""Robot-conditioning test (design revision 14.3) + Pareto, final criterion.

The central risk the design revision names is R4: the coarsener might be
learning nothing more than a hand-crafted size/clearance threshold.  The test
that separates those hypotheses is *cross-application*: build the coreset for
robot i, then plan robot j on it.

If the cut is genuinely conditioned on the robot, the diagonal (i == j) must
hold the gate while off-diagonal entries degrade -- a coreset built for a
robot that barely fits keeps geometry a slimmer robot never needed, and a
coreset built for a slim robot has already merged away the geometry a wider
robot depends on.  If instead every entry is equally good, conditioning buys
nothing and the learned coarsener in phase 4 cannot beat a threshold rule.
"""
import argparse
import json
import time
from pathlib import Path

from gmc.config import load_config
from gmc.coreset import CoarsenParams, coarsen_scene, uniform_cut
from gmc.coreset.hierarchy import build_hierarchy
from gmc.io.robot_io import ellipse_robot
from gmc.synth import gate_half_angle, single_door

from coreset_pareto import gate_threshold, surface_scene

# 2b < width < 2a for all three, so each has a non-trivial finite gate angle.
ROBOTS = {"wide": (0.50, 0.20), "slim": (0.40, 0.15), "fat": (0.35, 0.28)}
# (clearance_tol, target_gate_tol) -- the second is the knob that matters, and
# it is converted per robot to a radius tolerance via the gate sensitivity.
OPERATING = [(0.02, 0.10), (0.01, 0.05), (0.005, 0.02),
             (0.002, 0.005), (0.002, 0.002)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/toy.yaml")
    ap.add_argument("--width", type=float, default=0.6)
    ap.add_argument("-o", "--output",
                    default="results/coreset/robot_conditioning.json")
    args = ap.parse_args()
    cfg = load_config(args.config)
    scene = single_door(width=args.width)
    root = build_hierarchy(scene)
    rows = []

    # ---- reference: uncompressed compiler, one per robot
    truth, ref = {}, {}
    for name, (a, b) in ROBOTS.items():
        robot = ellipse_robot(a, b)
        truth[name] = gate_half_angle(a, b, args.width)
        ref[name] = gate_threshold(scene, robot, cfg)
        print(f"[full] robot={name:5s} a={a} b={b} gate={ref[name]['threshold']:.5f} "
              f"truth={truth[name]:.5f} err={ref[name]['threshold']-truth[name]:+.5f} "
              f"calls={ref[name]['support_calls']}", flush=True)
        rows.append({"kind": "full", "built_for": None, "evaluated": name,
                     "n": len(scene.supports), "truth": truth[name], **ref[name]})

    # ---- Pareto per robot, with the geometry-only control at matched count
    coresets = {}
    for name, (a, b) in ROBOTS.items():
        robot = ellipse_robot(a, b)
        for eps, tol in OPERATING:
            t0 = time.time()
            res = coarsen_scene(scene, robot,
                                CoarsenParams(clearance_tol=eps,
                                              target_gate_tol=tol),
                                hierarchy=root)
            build_s = time.time() - t0
            g = gate_threshold(res.scene, robot, cfg)
            rows.append({"kind": "coreset", "built_for": name,
                         "evaluated": name, "eps": eps, "target_gate_tol": tol,
                         "n": res.n_macro, "compression": res.compression,
                         "min_slack": res.min_slack, "build_seconds": build_s,
                         "critical_radii": list(res.critical_radii),
                         "truth": truth[name], **g})
            ctrl = uniform_cut(scene, res.n_macro, hierarchy=root)
            gc = gate_threshold(ctrl.scene, robot, cfg)
            rows.append({"kind": "uniform", "built_for": name,
                         "evaluated": name, "eps": eps, "target_gate_tol": tol,
                         "n": ctrl.n_macro, "compression": ctrl.compression,
                         "truth": truth[name], **gc})
            print(f"[pareto] robot={name:5s} eps={eps:<6} gate_tol={tol:<6} "
                  f"n={res.n_macro:4d} err={g['threshold']-truth[name]:+.5f} "
                  f"speedup={ref[name]['support_calls']/g['support_calls']:.1f}x "
                  f"| uniform err={gc['threshold']-truth[name]:+.5f}", flush=True)
            coresets.setdefault(name, {})[(eps, tol)] = res

    # ---- cross-application matrix at the exact-accuracy operating point
    op = (0.002, 0.005)
    print(f"\n=== cross-robot matrix at eps={op[0]} target_gate_tol={op[1]} ===",
          flush=True)
    for built, res in ((k, v[op]) for k, v in coresets.items()):
        for ev, (a, b) in ROBOTS.items():
            robot = ellipse_robot(a, b)
            g = gate_threshold(res.scene, robot, cfg)
            err = g["threshold"] - truth[ev]
            rows.append({"kind": "cross", "built_for": built, "evaluated": ev,
                         "eps": op[0], "target_gate_tol": op[1], "n": res.n_macro,
                         "compression": res.compression, "truth": truth[ev],
                         "ref_threshold": ref[ev]["threshold"], **g})
            print(f"  built_for={built:5s} eval={ev:5s} n={res.n_macro:4d} "
                  f"gate={g['threshold']:.5f} truth={truth[ev]:.5f} "
                  f"err={err:+.5f} vs_full={g['threshold']-ref[ev]['threshold']:+.5f}",
                  flush=True)

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"robots": ROBOTS, "door_width": args.width,
                               "operating_point": list(op), "rows": rows},
                              indent=2))
    print("wrote", out, flush=True)


if __name__ == "__main__":
    main()
