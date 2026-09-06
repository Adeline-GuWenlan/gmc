"""Oracle-guided floor probe (Round 2, review §7 — DIAGNOSTIC ONLY, never a
method): refine only cells intersecting a tube around the certified witness
path, breadth-first inside the tube.  Everything else (tri-state
certification, isotropic 2x2x2 splits, face-center adjacency, start/goal
seeding, billing) is identical to the real policies.

Answers: what is the minimum center-query budget at which THIS representation
machinery can produce a certified start-to-goal chain, given perfect
knowledge of where to look?
  floor << 64k  -> capacity sufficient; v4's failure is an ordering problem
  floor >~ 64k  -> capacity problem (splits / certification / adjacency)

Run:  cd splatc_atlas && PYTHONPATH=src python experiments/compiler/floor_probe.py
"""
import heapq
import json
import os
import sys
import time
from collections import Counter

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from splatc.datasets.g1_gate import (
    make_g1_scene, robot_library, Q_START, GOAL, GOAL_RADIUS)
from splatc.baselines.probe_methods import ProbeTree
from splatc.common.se2 import wrap_diff

WS = [0.505, 0.51, 0.52, 0.54, 0.58]
DEALIGN = (0.013, np.radians(7.0))
TUBE_RS = [0.2, 0.4]
CHECKPOINTS = [500, 1000, 1500, 2000, 3000, 4000, 6000, 8000, 12000,
               16000, 24000, 32000, 48000, 64000, 96000, 128000]
TABDIR = os.path.join(os.path.dirname(__file__), "..", "..", "results", "tables")


def witness_path(tilt, dy0, n=600):
    """Densely sampled certified witness: rotate at start, traverse the
    tilted door axis, reach the goal (same construction as the D17 ground
    truth witnesses)."""
    ax = lambda s: (s * np.cos(tilt), dy0 + s * np.sin(tilt))
    wps = [(-2.0, 0.0, np.pi / 2), (-2.0, 0.0, tilt),
           (*ax(-1.2), tilt), (*ax(1.2), tilt), (2.0, 0.0, tilt)]
    out = []
    for p, q in zip(wps[:-1], wps[1:]):
        dth = wrap_diff(q[2] - p[2])
        seg = max(2, int(n / (len(wps) - 1)))
        for t in np.linspace(0.0, 1.0, seg, endpoint=False):
            out.append((p[0] + t * (q[0] - p[0]),
                        p[1] + t * (q[1] - p[1]),
                        p[2] + t * dth))
    out.append(wps[-1])
    return np.array(out)


class FloorTree(ProbeTree):
    """Tube-restricted breadth-first refinement (privileged; diagnostic)."""

    def __init__(self, scene, robot, witness, tube_r, **kw):
        super().__init__(scene, robot, policy="uniform", **kw)
        self.wx, self.wy, self.wth = witness[:, 0], witness[:, 1], witness[:, 2]
        self.tube_r = tube_r
        self.a_max = max(robot.a, robot.b)

    def _tube_dist(self, c):
        return float(np.min(np.hypot(self.wx - c[0], self.wy - c[1])
                            + self.a_max * np.abs(wrap_diff(self.wth - c[2]))))

    def _push(self, key, rec):
        if key[0] >= self.max_level:
            return
        c = self.cell_center(key)
        if self._tube_dist(c) <= self.tube_r + self.cell_radius(key[0]):
            self._tick += 1
            heapq.heappush(self._heap, (float(key[0]), self._tick, key))


def main():
    robot = robot_library()["R_long_ellipse"]
    dy0, tilt = DEALIGN
    wit = witness_path(tilt, dy0)
    rows = []
    for w in WS:
        scene = make_g1_scene(w, door_offset=dy0, door_tilt=tilt)
        for tr in TUBE_RS:
            t0 = time.time()
            tree = FloorTree(scene, robot, wit, tr)
            res = tree.run(CHECKPOINTS, Q_START, GOAL, GOAL_RADIUS)
            solved = [b for b, r in sorted(res.items()) if r["reachable"]]
            first = solved[0] if solved else None
            free_lv = Counter(k[0] for k, r in tree.leaves.items()
                              if r["status"] == "FREE")
            rows.append({"w": w, "tube_r": tr, "first_success": first,
                         "queries_total": tree.queries,
                         "free_levels": dict(sorted(free_lv.items())),
                         "seconds": round(time.time() - t0, 1)})
            print(f"w={w:.3f} tube_r={tr}: floor="
                  f"{first if first else 'NOT WITHIN ' + str(CHECKPOINTS[-1])}"
                  f"  (total spent {tree.queries}, {rows[-1]['seconds']}s)",
                  flush=True)
    os.makedirs(TABDIR, exist_ok=True)
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from prov import make_provenance
    out = {"provenance": make_provenance(
        __file__, "oracle-tube floor (iso ProbeTree machinery, DIAGNOSTIC "
        "lower bound; not a method arm)"), "rows": rows}
    with open(os.path.join(TABDIR, "floor_probe.json"), "w") as f:
        json.dump(out, f, indent=2)
    print("floor_probe.json written")


if __name__ == "__main__":
    main()
