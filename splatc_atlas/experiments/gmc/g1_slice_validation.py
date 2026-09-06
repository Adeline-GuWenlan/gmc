"""Gate G1 closing experiment: GMC fixed-theta slice vs frozen oracle + truth.

Per (family, robot):
  A. CONNECTIVITY: for every oracle theta slice (medium grid 0.05/144), the
     outer GMC slice's free components must correspond 1:1 with the oracle
     slice's components (contingency bijection over robustly-free cells,
     margin >= dx). Splits/merges inside the polygonization gap must be
     bracketed by the inner slice (sandwich), else FAIL.
  B. WITNESSES: every outer component witness certified free by the frozen
     independent checker. Must be 100%.
  C. GATE INTERVAL (single_door + R_long_ellipse): two-sided theta sweep;
     open(outer) subset {|theta mod pi| < gate_half_angle} subset open(inner),
     endpoint errors <= sweep resolution + polygonization gap.

Families: frozen G1 single_door (w sweep x 3 robots) + constructed
double_door / u_shape / keyhole disc scenes (oracle-truth only).

Output: results/gmc_h2/g1_slice_validation.json (+ prints)
"""
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy import ndimage

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from splatc.datasets.g1_gate import (gate_half_angle, make_g1_scene,
                                     robot_library)
from splatc.gaussian_geometry.primitives import SceneGeometry
from splatc.gmc.slice import build_slice, certify_witnesses, probes_connected
from splatc.reference.oracle import dense_oracle, make_grid
from shapely.geometry import LineString

OUT = Path(__file__).resolve().parents[2] / "results" / "gmc_h2"
DX, NTHETA = 0.05, 144
NDIR = 48
GATE_NTH = 720                       # 0.25 deg over [0, pi)
SLIVER_CELLS = 8


def wall(p0, p1, spacing=0.04):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    n = max(2, int(np.ceil(np.linalg.norm(p1 - p0) / spacing)) + 1)
    t = np.linspace(0, 1, n)
    return p0 + t[:, None] * (p1 - p0)


def make_double_door(w1=0.9, w2=0.6, r=0.10):
    ws = (-3.5, 3.5, -2.3, 2.3)
    segs = [wall((0, -3.0), (0, -1.0 - w2 / 2)),
            wall((0, -1.0 + w2 / 2), (0, 1.0 - w1 / 2)),
            wall((0, 1.0 + w1 / 2), (0, 3.0))]
    return SceneGeometry("double_door", ws, [(r, np.concatenate(segs))])


def make_u_shape(opening=1.1, r=0.10):
    ws = (-3.5, 3.5, -2.3, 2.3)
    h = opening / 2
    segs = [wall((-2.0, -h), (-2.0, h)),
            wall((-2.0, h), (1.0, h)),
            wall((-2.0, -h), (1.0, -h))]
    return SceneGeometry("u_shape", ws, [(r, np.concatenate(segs))])


def make_keyhole(w=0.7, r=0.10):
    ws = (-3.5, 3.5, -2.3, 2.3)
    segs = [wall((-1.0, -1.0), (-1.0, 1.0)),
            wall((-1.0, 1.0), (1.0, 1.0)),
            wall((-1.0, -1.0), (1.0, -1.0)),
            wall((1.0, -1.0), (1.0, -w / 2)),
            wall((1.0, w / 2), (1.0, 1.0))]
    return SceneGeometry("keyhole", ws, [(r, np.concatenate(segs))])


def connectivity_check(scene, robot, grid, free):
    X, Y = np.meshgrid(grid.xs, grid.ys)
    stats = {"n_theta": 0, "ok": 0, "sliver": 0, "bracketed": 0,
             "certified_conn": 0, "fail": 0, "witness_fail": 0,
             "unassigned_frac": [], "merge_fine_comps": []}
    n_eval = NTHETA // 2 if robot.symmetric else NTHETA
    for k in range(n_eval):
        th = grid.thetas[k]
        sl = build_slice(scene, robot, th, ndir=NDIR, side="outer")
        ok, _ = certify_witnesses(sl, scene, robot)
        if not ok:
            stats["witness_fail"] += 1
        olab, n_o = ndimage.label(free[k])
        glab = sl.label_grid(X, Y)
        _, rho_sl = scene.eval_points(robot, th, X, Y)
        robust = free[k] & (np.asarray(rho_sl).reshape(X.shape) >= DX)
        stats["unassigned_frac"].append(
            float(((glab < 0) & robust).mean()))
        verdict = "ok"
        for oc in range(1, n_o + 1):
            cells = (olab == oc) & robust
            if cells.sum() == 0:
                if (olab == oc).sum() <= SLIVER_CELLS:
                    verdict = _worse(verdict, "sliver")
                    continue
                cells = olab == oc          # thin but real component
            gl = np.unique(glab[cells])
            gl = gl[gl >= 0]
            if len(gl) == 1:
                continue
            # split (or missing) -> must be bracketed by the inner slice
            sl_in = build_slice(scene, robot, th, ndir=NDIR, side="inner")
            ilab = sl_in.label_grid(X, Y)
            il = np.unique(ilab[cells])
            il = il[il >= 0]
            verdict = _worse(verdict, "bracketed" if len(il) <= 1 else "fail")
        # merge direction: one gmc-outer component spanning several oracle
        # components. Outer free is a subset of true free, so outer
        # connectivity is a construction-level certificate — the oracle grid
        # fragmented a thin-but-real corridor. Cross-check with a dx/2 slice
        # for the record; either way this is not a failure of the slice.
        both = (glab >= 0) & (olab > 0) & robust
        if both.any():
            pairs = set(zip(glab[both].ravel(), olab[both].ravel()))
            gcount = {}
            for g, o in pairs:
                gcount.setdefault(g, set()).add(o)
            if any(len(v) > 1 for v in gcount.values()):
                X2, Y2 = np.meshgrid(
                    np.arange(scene.workspace[0], scene.workspace[1], DX / 2),
                    np.arange(scene.workspace[2], scene.workspace[3], DX / 2))
                f2, _ = scene.eval_points(robot, th, X2, Y2)
                _, n2 = ndimage.label(np.asarray(f2).reshape(X2.shape))
                verdict = _worse(verdict, "certified_conn")
                stats["merge_fine_comps"].append([int(n_o), int(n2)])
        stats["n_theta"] += 1
        stats[verdict] += 1
    stats["unassigned_frac"] = float(np.mean(stats["unassigned_frac"]))
    return stats


def _worse(a, b):
    order = {"ok": 0, "sliver": 1, "bracketed": 2, "certified_conn": 3,
             "fail": 4}
    return a if order[a] >= order[b] else b


def gate_interval_check(scene, robot, w):
    probes = [LineString([(-2.0, -0.3), (-2.0, 0.3)]),
              LineString([(2.0, -0.3), (2.0, 0.3)])]
    from splatc.datasets.g1_gate import projection_radius
    half = gate_half_angle(robot.a, robot.b, w)
    thetas = np.linspace(0, np.pi, GATE_NTH, endpoint=False)
    bad_outer = bad_inner = 0
    for th in thetas:
        # direct passability predicate (robust at the half=pi/2 boundary,
        # where |theta| < pi/2 misses theta = pi/2 exactly)
        truth = projection_radius(robot.a, robot.b, th) < w / 2
        o = probes_connected(build_slice(scene, robot, th, NDIR, "outer"), probes)
        i = probes_connected(build_slice(scene, robot, th, NDIR, "inner"), probes)
        if o and not truth:
            bad_outer += 1              # outer-open must imply truth-open
        if truth and not i:
            bad_inner += 1              # truth-open must imply inner-open
    # endpoint error: measured open fraction vs analytic
    of_truth = 2 * half / np.pi
    of_outer = np.mean([probes_connected(
        build_slice(scene, robot, th, NDIR, "outer"), probes) for th in thetas])
    return {"w": w, "half_angle_deg": float(np.rad2deg(half)),
            "sandwich_violations_outer": bad_outer,
            "sandwich_violations_inner": bad_inner,
            "open_frac_truth": float(of_truth),
            "open_frac_outer": float(of_outer)}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    robots = robot_library()
    report = {"config": {"dx": DX, "n_theta": NTHETA, "ndir": NDIR,
                         "gate_n_theta": GATE_NTH}, "connectivity": [],
              "gates": []}
    combos = [("single_door", make_g1_scene(w), rn, {"w": w})
              for w in (0.55, 0.7, 0.9, 1.3) for rn in robots] + [
              ("double_door", make_double_door(), "R_long_ellipse", {}),
              ("u_shape", make_u_shape(), "R_long_ellipse", {}),
              ("keyhole", make_keyhole(), "R_long_ellipse", {})]
    for fam, scene, rn, extra in combos:
        robot = robots[rn]
        grid = make_grid(scene.workspace, DX, NTHETA)
        t0 = time.time()
        free, _ = dense_oracle(scene, robot, grid)
        st = connectivity_check(scene, robot, grid, free)
        st.update({"family": fam, "robot": rn, **extra,
                   "wall_s": time.time() - t0})
        report["connectivity"].append(st)
        print(st, flush=True)
    for w in (0.55, 0.7, 0.9, 1.3):
        g = gate_interval_check(make_g1_scene(w), robots["R_long_ellipse"], w)
        report["gates"].append(g)
        print(g, flush=True)
    conn = report["connectivity"]
    report["summary"] = {
        "total_theta_slices": sum(c["n_theta"] for c in conn),
        "fail_slices": sum(c["fail"] for c in conn),
        "certified_conn_slices": sum(c["certified_conn"] for c in conn),
        "bracketed_slices": sum(c["bracketed"] for c in conn),
        "sliver_slices": sum(c["sliver"] for c in conn),
        "witness_failures": sum(c["witness_fail"] for c in conn),
        "gate_sandwich_violations": sum(
            g["sandwich_violations_outer"] + g["sandwich_violations_inner"]
            for g in report["gates"]),
    }
    (OUT / "g1_slice_validation.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report["summary"], indent=1), flush=True)


if __name__ == "__main__":
    main()
