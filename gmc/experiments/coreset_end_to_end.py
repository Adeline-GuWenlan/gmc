"""End-to-end toy: plan on the coreset, verify on the original 3DGS.

This is the safety property the whole design rests on (design revision 6.3):
the mobility graph may be compiled from macro primitives, but the returned
trajectory is re-verified against the *original leaf supports* by the
independent continuous checker.  A coreset that silently deleted an obstacle
would be caught here, because the verifier never sees a macro ellipse.

Reported for both arms: compile cost, query verdict, clearance lower bound, and
the independent verdict on the uncompressed scene.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np

from gmc.budget import WorkLedger
from gmc.config import load_config
from gmc.coreset import CoarsenParams, coarsen_scene
from gmc.io.robot_io import ellipse_robot
from gmc.mobility.graph import compile_mobility
from gmc.mobility.query import query
from gmc.orientation.slab_builder import build_slabs
from gmc.spatial.bvh import query_candidate_pairs
from gmc.synth import single_door
from gmc.types import PlanStatus, Pose2
from gmc.verification.path import verify_curve


def compile_and_query(scene, robot, cfg, start, goal, label):
    t0 = time.time()
    ledger = WorkLedger()
    pq = query_candidate_pairs(scene, robot, scene.workspace)
    oracles = list(pq.oracles)
    for o in oracles:
        o.ledger = ledger
    dec = build_slabs(scene, robot, cfg, oracles, ledger=ledger)
    mc = compile_mobility(scene, robot, cfg, oracles, dec, ledger=ledger)
    compile_s = time.time() - t0

    t1 = time.time()
    result = query(start, goal, mc)
    query_s = time.time() - t1
    totals = ledger.snapshot().get("totals", {})
    out = {
        "arm": label,
        "n_supports": len(scene.supports),
        "n_pairs": len(oracles),
        "compile_seconds": compile_s,
        "query_seconds": query_s,
        "safe_nodes": mc.M_safe.number_of_nodes(),
        "safe_edges": mc.M_safe.number_of_edges(),
        "possible_nodes": mc.M_possible.number_of_nodes(),
        "possible_edges": mc.M_possible.number_of_edges(),
        "status": result.status.name,
        "clearance_lb": result.clearance_lower_bound,
        "support_value_evals": totals.get("support_value_evals"),
        "support_point_evals": totals.get("support_point_evals"),
    }
    print(f"[{label}] n={out['n_supports']} pairs={out['n_pairs']} "
          f"compile={compile_s:.1f}s query={query_s:.1f}s "
          f"safe={out['safe_nodes']}n/{out['safe_edges']}e "
          f"status={out['status']} clearance_lb={out['clearance_lb']}",
          flush=True)
    return out, result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/toy.yaml")
    ap.add_argument("--width", type=float, default=0.6)
    ap.add_argument("--robot", type=float, nargs=2, default=(0.50, 0.20))
    ap.add_argument("--target-gate-tol", type=float, default=0.005)
    ap.add_argument("--theta", type=float, default=0.3)
    ap.add_argument("--skip-full", action="store_true",
                    help="skip the uncompressed comparison arm (it is the "
                         "expensive one); the coreset path is still verified "
                         "against the original supports either way")
    ap.add_argument("-o", "--output",
                    default="results/coreset/end_to_end.json")
    args = ap.parse_args()

    cfg = load_config(args.config)
    a, b = args.robot
    robot = ellipse_robot(a, b)
    scene = single_door(width=args.width)
    start = Pose2(np.array([-2.0, 0.0]), args.theta)
    goal = Pose2(np.array([2.0, 0.0]), args.theta)

    t0 = time.time()
    res = coarsen_scene(scene, robot,
                        CoarsenParams(clearance_tol=0.002,
                                      target_gate_tol=args.target_gate_tol))
    coarsen_s = time.time() - t0
    print(f"coreset: {res.n_original} -> {res.n_macro} supports "
          f"(ratio {res.compression:.4f}), min_slack={res.min_slack:+.2e}, "
          f"critical_radii={[round(r, 5) for r in res.critical_radii]}, "
          f"{coarsen_s:.1f}s", flush=True)

    coarse_out, coarse_result = compile_and_query(
        res.scene, robot, cfg, start, goal, "coreset")
    if args.skip_full:
        full_out = None
    else:
        full_out, _ = compile_and_query(scene, robot, cfg, start, goal, "full")

    # --- the safety check: verify the coreset's path on the ORIGINAL supports
    verdict = {"ran": False}
    if coarse_result.curve is not None:
        pq0 = query_candidate_pairs(scene, robot, scene.workspace)
        rep = verify_curve(tuple(pq0.oracles), scene.workspace,
                           coarse_result.curve, cfg.query.eps_clear,
                           cfg.orientation.theta_min,
                           expected_start=start, expected_goal=goal)
        verdict = {"ran": True, "certified": bool(rep.certified),
                   "min_clearance": float(rep.min_clearance),
                   "failed_segment": rep.failed_segment, "reason": rep.reason,
                   "n_original_pairs": len(pq0.oracles)}
        print(f"\n=== INDEPENDENT VERIFICATION OF THE CORESET PATH "
              f"AGAINST ALL {len(pq0.oracles)} ORIGINAL PAIRS ===", flush=True)
        print(f"certified={rep.certified} min_clearance={rep.min_clearance:.6f} "
              f"reason={rep.reason}", flush=True)
    else:
        print("\ncoreset query returned no curve; nothing to verify",
              flush=True)

    speedup = None
    if full_out and coarse_out["support_value_evals"] \
            and full_out["support_value_evals"]:
        speedup = (full_out["support_value_evals"]
                   / coarse_out["support_value_evals"])
        print(f"\ncompile speedup "
              f"{full_out['compile_seconds'] / max(coarse_out['compile_seconds'], 1e-9):.1f}x"
              f" wall, {speedup:.1f}x support evaluations", flush=True)

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({
        "coreset": {"n_original": res.n_original, "n_macro": res.n_macro,
                    "compression": res.compression,
                    "min_slack": res.min_slack,
                    "critical_radii": list(res.critical_radii),
                    "coarsen_seconds": coarsen_s,
                    "target_gate_tol": args.target_gate_tol},
        "arms": [a for a in (coarse_out, full_out) if a],
        "independent_verification_on_original": verdict,
        "support_eval_speedup": speedup,
    }, indent=2))
    print("wrote", out, flush=True)


if __name__ == "__main__":
    main()
