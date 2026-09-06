"""Does robot conditioning actually change the cut?  (design revision 14.3, R4)

The single-door family cannot answer this.  It has one passage at one critical
radius, so every robot negotiates the same door edges and one coreset serves
them all -- which is exactly what the first cross-robot matrix showed, and it is
a property of the scene, not evidence about conditioning.

A scene with passages at *different* scales can answer it.  With a wide door and
a narrow door, a robot too fat to fit the narrow one may legitimately merge it
shut, while a robot that fits must preserve it.  The decisive probe is direct:
is the pose in the narrow doorway still free after coarsening?
"""
import argparse
import json
from pathlib import Path

import numpy as np

from gmc.config import load_config
from gmc.coreset import CoarsenParams, coarsen_scene
from gmc.coreset.hierarchy import build_hierarchy
from gmc.io.robot_io import ellipse_robot
from gmc.spatial.slice_compiler import build_slice
from gmc.synth import DISC_R, SPACING, _disc
from gmc.types import GaussianSupport2D, SceneModel2D

from shapely.geometry import box as shapely_box

WIDE_Y, NARROW_Y = 1.0, -1.0


def dual_door_thick(w_wide=0.9, w_narrow=0.45, wall_t=0.6,
                    workspace=(-3.0, 3.0, -2.5, 2.5)) -> SceneModel2D:
    """Thick vertical wall with a wide door at y=+1 and a narrow one at y=-1."""
    xmin, xmax, ymin, ymax = workspace
    half_t = wall_t / 2.0
    xs = np.arange(-half_t + DISC_R, half_t - DISC_R + 1e-9, SPACING)
    bands = [(ymin - 0.5, NARROW_Y - w_narrow / 2.0 - DISC_R),
             (NARROW_Y + w_narrow / 2.0 + DISC_R,
              WIDE_Y - w_wide / 2.0 - DISC_R),
             (WIDE_Y + w_wide / 2.0 + DISC_R, ymax + 0.5)]
    supports = []
    for y0, y1 in bands:
        for y in np.arange(y0, y1 + 1e-9, SPACING):
            for x in xs:
                supports.append(_disc((x, y), DISC_R, len(supports)))
    supports = [GaussianSupport2D(s.mean, s.covariance, s.level, i)
                for i, s in enumerate(supports)]
    return SceneModel2D(supports=tuple(supports),
                        workspace=shapely_box(xmin, ymin, xmax, ymax),
                        name=f"dual_door_{w_wide:g}_{w_narrow:g}")


def doorway_free(scene, robot, cfg, y, theta) -> bool:
    """Can the robot stand in the doorway at (0, y) with orientation theta?"""
    sl = build_slice(scene, robot, theta, cfg)
    return sl.locate((0.0, y), "safe") is not None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/toy.yaml")
    ap.add_argument("--w-wide", type=float, default=0.9)
    ap.add_argument("--w-narrow", type=float, default=0.45)
    ap.add_argument("--target-gate-tol", type=float, default=0.02)
    ap.add_argument("-o", "--output",
                    default="results/coreset/conditioning_dual.json")
    args = ap.parse_args()
    cfg = load_config(args.config)
    scene = dual_door_thick(args.w_wide, args.w_narrow)
    root = build_hierarchy(scene)
    print(f"scene {scene.name}: {len(scene.supports)} supports", flush=True)

    # small fits the narrow door at any orientation (2a < w_narrow);
    # large cannot fit it at any orientation (2b > w_narrow).
    robots = {"small": (0.20, 0.10), "large": (0.40, 0.25)}
    for name, (a, b) in robots.items():
        print(f"  robot {name}: a={a} b={b}  2a={2*a} 2b={2*b} "
              f"vs narrow={args.w_narrow} -> "
              f"{'fits' if 2*a < args.w_narrow else 'cannot fit'}", flush=True)

    params = CoarsenParams(clearance_tol=0.002,
                           target_gate_tol=args.target_gate_tol)
    coresets = {}
    for name, (a, b) in robots.items():
        res = coarsen_scene(scene, ellipse_robot(a, b), params, hierarchy=root)
        coresets[name] = res
        print(f"[coreset built_for={name}] {res.n_original} -> {res.n_macro} "
              f"(ratio {res.compression:.4f}) critical="
              f"{[round(r, 4) for r in res.critical_radii]}", flush=True)

    rows = []
    print("\n=== is each doorway still usable? (theta=0) ===", flush=True)
    for ev, (a, b) in robots.items():
        robot = ellipse_robot(a, b)
        for src_name, src in [("original", scene)] + [
                (f"coreset[{k}]", v.scene) for k, v in coresets.items()]:
            wide_ok = doorway_free(src, robot, cfg, WIDE_Y, 0.0)
            narrow_ok = doorway_free(src, robot, cfg, NARROW_Y, 0.0)
            rows.append({"evaluated": ev, "scene": src_name,
                         "wide_door_free": wide_ok,
                         "narrow_door_free": narrow_ok,
                         "n": len(src.supports)})
            print(f"  robot={ev:6s} on {src_name:16s} n={len(src.supports):4d} "
                  f"wide={'OPEN' if wide_ok else 'shut':4s} "
                  f"narrow={'OPEN' if narrow_ok else 'shut':4s}", flush=True)

    # Conditioning = asymmetric degradation.  A coreset is "adequate" for a
    # robot if every doorway that robot can use on the original scene is still
    # usable.  Conditioning exists when some coreset is adequate for the robot
    # it was built for but inadequate for another -- in either direction.
    def get(ev, sc):
        return [r for r in rows if r["evaluated"] == ev
                and r["scene"] == sc][0]

    def adequate(ev, sc):
        orig, got = get(ev, "original"), get(ev, sc)
        return all(got[k] or not orig[k]
                   for k in ("wide_door_free", "narrow_door_free"))

    names = list(robots)
    matrix = {f"{b}->{e}": adequate(e, f"coreset[{b}]")
              for b in names for e in names}
    print("\n=== adequacy matrix (coreset built_for -> evaluated robot) ===",
          flush=True)
    for b in names:
        row = "  ".join(f"{e}:{'OK ' if matrix[f'{b}->{e}'] else 'LOST'}"
                        for e in names)
        print(f"  built_for={b:6s} {row}", flush=True)
    diagonal_ok = all(matrix[f"{n}->{n}"] for n in names)
    off_diagonal_lost = [k for k, v in matrix.items()
                         if not v and k.split("->")[0] != k.split("->")[1]]
    conditioned = diagonal_ok and bool(off_diagonal_lost)
    print(f"\nCONDITIONING SIGNAL: {'YES' if conditioned else 'NO'} -- "
          f"every coreset is adequate for its own robot={diagonal_ok}; "
          f"cross-application failures={off_diagonal_lost}", flush=True)

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({
        "scene": scene.name, "n_supports": len(scene.supports),
        "robots": robots, "w_wide": args.w_wide, "w_narrow": args.w_narrow,
        "coresets": {k: {"n": v.n_macro, "compression": v.compression,
                         "critical_radii": list(v.critical_radii)}
                     for k, v in coresets.items()},
        "rows": rows, "adequacy_matrix": matrix,
        "conditioning_signal": conditioned}, indent=2))
    print("wrote", out, flush=True)


if __name__ == "__main__":
    main()
