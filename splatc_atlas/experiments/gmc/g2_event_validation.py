"""Gate G2 validation: certified event finder vs analytic truth and vs
uniform theta sweeps at matched budget.

A. single_door (frozen bench, analytic gate_half_angle truth), w sweep,
   R_long_ellipse, tol sweep: two-sided certified brackets must CONTAIN the
   analytic events; report bracket width, budget, and the uniform-sweep
   worst-case localization error at the SAME budget (uniform has no
   certificate at any budget — the qualitative gap).
B. double_door (two gates, no analytic truth): certified brackets vs a dense
   uniform reference sweep (0.05 deg): every reference event inside a
   certified bracket (no missed events), no spurious brackets.

Output: results/gmc_h2/g2_event_validation.json
"""
import json
import sys
import time
from pathlib import Path

import numpy as np
from shapely.geometry import LineString

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from splatc.datasets.g1_gate import (gate_half_angle, make_g1_scene,
                                     robot_library)
from splatc.gmc.events import certified_gate_intervals
from splatc.gmc.slice import build_slice, probes_connected

from g1_slice_validation import make_double_door

OUT = Path(__file__).resolve().parents[2] / "results" / "gmc_h2"
PROBES = [LineString([(-2.0, -0.3), (-2.0, 0.3)]),
          LineString([(2.0, -0.3), (2.0, 0.3)])]


def two_sided(scene, robot, tol):
    rep_o = certified_gate_intervals(scene, robot, PROBES, tol=tol, side="outer")
    rep_i = certified_gate_intervals(scene, robot, PROBES, tol=tol, side="inner")
    n = rep_o.n_slices + rep_i.n_slices
    # pair up brackets by proximity; sandwich = union extent
    sandw = []
    used = set()
    for a, b, *_ in rep_o.brackets:
        best, bestd = None, 1e9
        for j, (c, d, *_x) in enumerate(rep_i.brackets):
            if j not in used and abs((a + b) - (c + d)) / 2 < bestd:
                best, bestd = j, abs((a + b) - (c + d)) / 2
        if best is not None and bestd < np.deg2rad(2.0):
            c, d, *_x = rep_i.brackets[best]
            used.add(best)
            sandw.append((min(a, c), max(b, d)))
        else:
            sandw.append((a, b))
    return sandw, n, {"outer": rep_o, "inner": rep_i}


def uniform_reference(scene, robot, step_deg=0.05):
    thetas = np.arange(0.0, 180.0, step_deg)
    flags = []
    for d in thetas:
        sl = build_slice(scene, robot, np.deg2rad(d), 48, "outer")
        flags.append(probes_connected(sl, PROBES))
    flags = np.asarray(flags)
    sw = np.flatnonzero(flags[1:] != flags[:-1])
    return [np.deg2rad((thetas[i] + thetas[i + 1]) / 2) for i in sw], len(thetas)


def main():
    robot = robot_library()["R_long_ellipse"]
    report = {"single_door": [], "double_door": {}}

    for w in (0.55, 0.7, 0.9):
        scene = make_g1_scene(w)
        half = gate_half_angle(robot.a, robot.b, w)
        truths = [half, np.pi - half]
        for tol_deg in (0.2, 0.1, 0.05):
            t0 = time.time()
            sandw, n, _ = two_sided(scene, robot, np.deg2rad(tol_deg))
            contain = [any(a <= t <= b for a, b in sandw) for t in truths]
            widths = [np.rad2deg(b - a) for a, b in sandw]
            mid_err = [min(abs(np.rad2deg((a + b) / 2 - t))
                           for a, b in sandw) for t in truths]
            uniform_err_same_budget = 180.0 / n / 2   # worst-case, no certificate
            row = {"w": w, "tol_deg": tol_deg, "n_slices": n,
                   "truth_deg": [float(np.rad2deg(t)) for t in truths],
                   "n_brackets": len(sandw),
                   "all_truths_contained": all(contain),
                   "bracket_widths_deg": [float(x) for x in widths],
                   "midpoint_err_deg": [float(x) for x in mid_err],
                   "uniform_worstcase_err_same_budget_deg": uniform_err_same_budget,
                   "wall_s": time.time() - t0}
            report["single_door"].append(row)
            print(row, flush=True)

    scene = make_double_door()
    t0 = time.time()
    sandw, n, reps = two_sided(scene, robot, np.deg2rad(0.1))
    ref_events, n_ref = uniform_reference(scene, robot, step_deg=0.05)
    missed = [float(np.rad2deg(t)) for t in ref_events
              if not any(a - np.deg2rad(0.06) <= t <= b + np.deg2rad(0.06)
                         for a, b in sandw)]
    spurious = [(float(np.rad2deg(a)), float(np.rad2deg(b)))
                for a, b in sandw
                if not any(a - np.deg2rad(0.06) <= t <= b + np.deg2rad(0.06)
                           for t in ref_events)]
    report["double_door"] = {
        "n_slices_certified": n, "n_slices_reference": n_ref,
        "n_reference_events": len(ref_events),
        "reference_events_deg": [float(np.rad2deg(t)) for t in ref_events],
        "n_certified_brackets": len(sandw),
        "brackets_deg": [(float(np.rad2deg(a)), float(np.rad2deg(b)))
                         for a, b in sandw],
        "missed_events": missed, "spurious_brackets": spurious,
        "unresolved": len(reps["outer"].unresolved) + len(reps["inner"].unresolved),
        "wall_s": time.time() - t0}
    print(report["double_door"], flush=True)

    ok = (all(r["all_truths_contained"] for r in report["single_door"])
          and not report["double_door"]["missed_events"]
          and not report["double_door"]["spurious_brackets"])
    report["summary"] = {"gate_g2_first_leg_pass": bool(ok)}
    (OUT / "g2_event_validation.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report["summary"]), flush=True)


if __name__ == "__main__":
    main()
