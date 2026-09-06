"""Oracle labeling for dev / validation manifests (OracleRecord wiring;
Sprint B carryover item).  BLIND IS NEVER LABELED HERE — it stays sealed
until final evaluation (plan 00 §5.2 discipline).

Per episode: coarse/medium/fine oracle (reachability + components per goal),
mid-line gate interval measurement, analytic labels via rotational
equivariance, medium reference path + conservative swept certificate for the
first reachable goal.  One JSON record per episode.

Run (one episode, for job arrays):
  PYTHONPATH=splatc_atlas/src python splatc_atlas/experiments/oracle/label_manifests.py --index N
Run (all episodes serially):
  ... --all
"""
import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "src"))

from splatc.datasets.g1_gate import (  # noqa: E402
    make_g1_scene, robot_library, blind_robot_library, gate_half_angle, ELL_R)
from splatc.reference.oracle import (  # noqa: E402
    make_grid, RESOLUTIONS, dense_oracle, components, goal_mask, pose_to_index,
    shortest_path, certify_path_conservative, gate_interval_numeric)
from splatc.evaluation.records import validate_oracle_record  # noqa: E402

DATA = os.path.join(HERE, "..", "..", "data", "splatc_gates")
OUT = os.path.join(HERE, "..", "..", "outputs", "oracle_records")


def load_episodes():
    eps = []
    for split in ("dev", "validation"):  # blind deliberately excluded
        with open(os.path.join(DATA, split, "manifest.json")) as f:
            m = json.load(f)
        for e in m["episodes"]:
            eps.append((split, e))
    return eps


def measured_half_angle(passable, thetas, center):
    if not passable.any():
        return 0.0
    if passable.all():
        return np.pi / 2
    th = np.mod(thetas - center + np.pi, 2 * np.pi) - np.pi
    fold = np.abs(np.where(np.abs(th) > np.pi / 2, np.pi - np.abs(th), th))
    return float(np.max(fold[passable]))


def label_episode(split, e):
    lib = {**robot_library(), **blind_robot_library()}
    s = e["scene"]
    tilt = np.radians(s["door_tilt_deg"])
    scene = make_g1_scene(s["door_width"], door_offset=s["door_offset"],
                          door_tilt=tilt)
    robot = lib[e["robot_id"]]
    w = s["door_width"]
    rec = {
        "query_id": e["query_id"], "split": split,
        "scene_id": scene.scene_id, "robot_id": e["robot_id"],
        "collision_checker_id": "pw_bisect_v1",
        "theta_periodic": True,
        "analytic": {
            "physically_open": bool(w > 2 * robot.b),
            "gate_half_angle_deg":
                float(np.degrees(gate_half_angle(robot.a, robot.b, w))),
            "gate_center_deg": s["door_tilt_deg"],
        },
        "resolutions": {},
    }

    for res, (dx, ntheta) in RESOLUTIONS.items():
        grid = make_grid(scene.workspace, dx, ntheta)
        t0 = time.time()
        free, rho = dense_oracle(scene, robot, grid)
        labels, ncomp = components(free)
        k, j, i = pose_to_index(grid, tuple(e["start_pose"]))
        start_free = bool(free[k, j, i])
        start_label = int(labels[k, j, i]) if start_free else 0
        goals_reach = []
        for g in e["goals"]:
            gm = goal_mask(grid, g, e["goal_radius"]) & free
            goals_reach.append(bool(start_free
                                    and np.any(labels[gm] == start_label)))
        rec["resolutions"][res] = {
            "grid_shape": list(grid.shape),
            "n_components": int(ncomp),
            "start_free": start_free,
            "reachable_per_goal": goals_reach,
            "seconds": round(time.time() - t0, 2),
        }
        if res == "medium":
            free_m, grid_m = free, grid

    # measured gate interval (dense theta on the door mid-line)
    passable, thetas = gate_interval_numeric(scene, robot, w)
    rec["measured_gate_half_angle_deg"] = float(np.degrees(
        measured_half_angle(passable, thetas, tilt)))

    # reference path on medium for the first analytically-plausible goal
    rec["reference_path"] = None
    for gi, g in enumerate(e["goals"]):
        if rec["resolutions"]["medium"]["reachable_per_goal"][gi]:
            poses, cost = shortest_path(free_m, grid_m, tuple(e["start_pose"]),
                                        g, e["goal_radius"], ELL_R)
            if poses is None:
                break
            cert, min_m, depth, checks = certify_path_conservative(
                scene, robot, poses)
            rec["reference_path"] = {
                "goal_index": gi, "cost": float(cost), "n_poses": len(poses),
                "certified": bool(cert),
                "min_metric_margin_m": float(min_m),
                "bisect_depth": int(depth),
            }
            break

    # convergence status: medium and fine must agree on all reachability bits
    agree = (rec["resolutions"]["medium"]["reachable_per_goal"]
             == rec["resolutions"]["fine"]["reachable_per_goal"])
    ana_open = rec["analytic"]["physically_open"]
    fine_reach = rec["resolutions"]["fine"]["reachable_per_goal"]
    # single-gate scenes: any false-unreachable vs analytic marks unresolved
    suspicious = ana_open and len(fine_reach) == 1 and not fine_reach[0]
    rec["convergence_status"] = ("converged" if agree and not suspicious
                                 else "ORACLE_UNRESOLVED_GRID")
    rec["uncertainty_flags"] = (
        ["grid_oracle_below_analytic_truth"] if suspicious else [])
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=None)
    ap.add_argument("--all", action="store_true")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    eps = load_episodes()
    todo = range(len(eps)) if args.all else [args.index]
    for n in todo:
        split, e = eps[n]
        t0 = time.time()
        rec = label_episode(split, e)
        validate_oracle_record(rec)  # schema drift guard (records.py is canon)
        path = os.path.join(OUT, f"{rec['query_id']}.json")
        with open(path, "w") as f:
            json.dump(rec, f, indent=2)
        print(f"[{n:02d}] {rec['query_id']} conv={rec['convergence_status']} "
              f"({time.time()-t0:.0f}s)", flush=True)


if __name__ == "__main__":
    main()
