"""Generate and seal the SplatC-Gates dev / validation / blind manifests
(03 spec §8; Gate A criterion 5).

Blind design (03 §8.3 holdouts): held-out widths in the critical region,
held-out door offsets/tilts (incl. the largest magnitudes), a morphology
holdout robot (R_blind_ellipse, only referenced here), just-unreachable
widths, and one multi-goal batch.  The blind manifest carries NO labels.
Sealing = SHA-256 of the canonical manifest JSON, recorded in
results/manifests/split_seals.json and in the Gate A report.

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/oracle/make_splits.py
"""
import hashlib
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from splatc.datasets.g1_gate import (
    make_g1_scene, robot_library, blind_robot_library,
    Q_START, GOAL, GOAL_RADIUS)

DATA = os.path.join(os.path.dirname(__file__), "..", "..", "data", "splatc_gates")
SEALS = os.path.join(os.path.dirname(__file__), "..", "..", "results", "manifests")
SEED = 20260812


def episode(eid, w, dy0, tilt_deg, robot_id, goals=None, note=""):
    return {
        "query_id": eid,
        "scene": {"family": "G1", "door_width": round(w, 4),
                  "door_offset": round(dy0, 4),
                  "door_tilt_deg": round(tilt_deg, 2)},
        "robot_id": robot_id,
        "start_pose": list(Q_START),
        "goals": goals if goals is not None else [list(GOAL)],
        "goal_radius": GOAL_RADIUS,
        "final_orientation": "free",
        "note": note,
    }


def sanity_check(eps):
    """Start pose must be free and rooms must be on opposite door sides."""
    lib = {**robot_library(), **blind_robot_library()}
    for e in eps:
        s = e["scene"]
        scene = make_g1_scene(s["door_width"], door_offset=s["door_offset"],
                              door_tilt=np.radians(s["door_tilt_deg"]))
        robot = lib[e["robot_id"]]
        ok, rho = scene.check_pose(robot, tuple(e["start_pose"]))
        assert ok, f"{e['query_id']}: start not free (rho={rho:.4f})"
        n = np.array([np.cos(np.radians(s["door_tilt_deg"])),
                      np.sin(np.radians(s["door_tilt_deg"]))])
        c = np.array([0.0, s["door_offset"]])
        side_s = np.dot(np.array(e["start_pose"][:2]) - c, n)
        far = [side_s * np.dot(np.array(g) - c, n) < 0 for g in e["goals"]]
        if len(e["goals"]) == 1:
            assert far[0], f"{e['query_id']}: start/goal on same door side"
        else:  # multi-goal batch: same-room goals allowed, need >=1 far-side
            assert any(far), f"{e['query_id']}: no far-side goal in batch"


def main():
    rng = np.random.default_rng(SEED)
    trio = list(robot_library())

    dev = []
    for w in (0.55, 0.70, 0.90, 1.10):
        for dy0, tilt in ((0.0, 0.0), (0.013, 5.0)):
            for rid in trio:
                dev.append(episode(f"dev_{len(dev):03d}", w, dy0, tilt, rid))

    val = []
    for w in (0.52, 0.62, 0.80):
        for dy0, tilt in ((0.007, -8.0), (-0.021, 12.0)):
            for rid in trio:
                val.append(episode(f"val_{len(val):03d}", w, dy0, tilt, rid))

    blind = []
    # critical-reachable, randomized de-alignment (main robot)
    for _ in range(8):
        w = float(rng.uniform(0.502, 0.530))
        dy0 = float(rng.uniform(-0.03, 0.03))
        tilt = float(rng.uniform(-14.0, 14.0))
        blind.append(episode(f"blind_{len(blind):03d}", w, dy0, tilt,
                             "R_long_ellipse", note="critical-region"))
    # just-unreachable
    for _ in range(4):
        w = float(rng.uniform(0.470, 0.499))
        blind.append(episode(f"blind_{len(blind):03d}", w,
                             float(rng.uniform(-0.03, 0.03)),
                             float(rng.uniform(-14.0, 14.0)),
                             "R_long_ellipse", note="near-closure"))
    # morphology holdout (2b = 0.6)
    for w, note in ((0.605, "holdout-critical"), (0.615, "holdout-critical"),
                    (0.640, "holdout-critical"), (0.660, "holdout-open"),
                    (0.560, "holdout-closed"), (0.590, "holdout-closed")):
        blind.append(episode(f"blind_{len(blind):03d}", w,
                             float(rng.uniform(-0.03, 0.03)),
                             float(rng.uniform(-14.0, 14.0)),
                             "R_blind_ellipse", note=note))
    # held-out extreme tilt
    for tilt in (14.0, -14.0):
        blind.append(episode(f"blind_{len(blind):03d}",
                             float(rng.uniform(0.55, 0.65)),
                             float(rng.uniform(-0.03, 0.03)), tilt,
                             "R_long_ellipse", note="extreme-tilt"))
    # multi-goal batch (E7): one scene, four goals incl. same-room + far-room
    blind.append(episode(f"blind_{len(blind):03d}", 0.58, -0.017, 9.0,
                         "R_long_ellipse",
                         goals=[[2.0, 0.0], [2.6, 1.4], [-2.6, -1.6], [2.6, -1.5]],
                         note="multi-goal-batch"))

    for eps in (dev, val, blind):
        sanity_check(eps)

    seals = {}
    for name, eps in (("dev", dev), ("validation", val), ("blind", blind)):
        d = os.path.join(DATA, name)
        os.makedirs(d, exist_ok=True)
        payload = json.dumps({"split": name, "seed": SEED, "episodes": eps},
                             indent=2, sort_keys=True)
        with open(os.path.join(d, "manifest.json"), "w") as f:
            f.write(payload)
        seals[name] = {"sha256": hashlib.sha256(payload.encode()).hexdigest(),
                       "n_episodes": len(eps)}
        print(f"{name}: {len(eps)} episodes  sha256={seals[name]['sha256'][:16]}…")

    os.makedirs(SEALS, exist_ok=True)
    with open(os.path.join(SEALS, "split_seals.json"), "w") as f:
        json.dump({"sealed_on": "2026-08-11", "seed": SEED,
                   "note": "sealed before any method tuning; blind labels "
                           "never computed at seal time", "seals": seals}, f,
                  indent=2)
    print("seals written to results/manifests/split_seals.json")


if __name__ == "__main__":
    main()
