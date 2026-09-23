"""Leak sweep: is every boundary of the booth closed for the UAV body?

For each boundary face a grid of straight swept-body edges crosses it
perpendicularly (shared production oracle, margin .05).  A closed face has
*no* free crossing edge.  The under-lamp opening is swept too, as the positive
control: it must have free crossings.  Uses a large declared known box so that
box faces cannot hide a hole.  Writes ``probes.json`` and ``probes.png``.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from gmc.gs3d.contracts import Pose3, SceneSpec
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.robots import BoxKnownSpace, crop_by_support_aabb
from gmc.gs3d import scene_uavlamp as su
from uavlamp_query import UAV


def faces(doc):
    q = doc["query"]
    lamp = q["lamp"]
    ul = (lamp["footprint_route_uv"][0][0] + lamp["footprint_route_uv"][1][0]) / 2
    zl = lamp["underside_z"]
    ub = q["box_route"]["upper"][0]
    r = np.round
    # name: (fixed-axis sweep grid, crossing axis, from, to)
    return {
        "wall_A (real)": dict(grid=[("u", r(np.arange(ul, ub - .3 + 1e-9, .1), 3)), ("z", r(np.arange(.25, 2.15 + 1e-9, .1), 3))],
                              axis="v", a=.35, b=-.70),
        "wall_B (real)": dict(grid=[("u", r(np.arange(ul, ub - .3 + 1e-9, .1), 3)), ("z", r(np.arange(.25, 2.15 + 1e-9, .1), 3))],
                              axis="v", a=1.95, b=3.10),
        "soffit": dict(grid=[("u", r(np.arange(ul, ub - .3 + 1e-9, .1), 3)), ("v", r(np.arange(.35, 1.95 + 1e-9, .1), 3))],
                       axis="z", a=2.20, b=2.80),
        "back_panel": dict(grid=[("v", r(np.arange(.35, 1.95 + 1e-9, .1), 3)), ("z", r(np.arange(1.15, 2.15 + 1e-9, .1), 3))],
                           axis="u", a=ub - .35, b=ub + .60),
        "back_panel_low (beside table)": dict(grid=[("v", r(np.arange(1.15, 1.95 + 1e-9, .1), 3)), ("z", r(np.arange(.25, 1.05 + 1e-9, .1), 3))],
                                              axis="u", a=ub - .35, b=ub + .60),
        "entrance_above_underside (lamp+bulkhead)": dict(grid=[("v", r(np.arange(.35, 1.95 + 1e-9, .1), 3)), ("z", r(np.arange(zl - .05, 2.15 + 1e-9, .1), 3))],
                                                         axis="u", a=ul - .55, b=ul + .55),
        "entrance_under_lamp (positive control)": dict(grid=[("v", r(np.arange(.35, 1.95 + 1e-9, .1), 3)), ("z", r(np.arange(.25, zl - .25 + 1e-9, .1), 3))],
                                                       axis="u", a=ul - .55, b=ul + .55),
    }


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--archive", type=Path, required=True)
    p.add_argument("--manifest", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args(argv)
    a.output.mkdir(parents=True, exist_ok=True)
    full, doc = su.load_uavlamp_derivative(a.archive, a.manifest)
    frame = su.Frame.from_dict(doc["frame"])
    lo, hi = np.array([-5., -1.5, -.3]), np.array([4.5, 3.8, 3.3])
    corners = frame.to_world(np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]))
    bmin, bmax = tuple(corners.min(0)), tuple(corners.max(0))
    cropped, crop = crop_by_support_aabb(full, bmin, bmax, level=su.LEVEL, tau=su.TAU)
    scene = SceneSpec("probe", cropped, bmin, bmax, su.TAU, su.LEVEL, BoxKnownSpace(bmin, bmax), None,
                      {"coverage_policy": "large declared probe box"})
    oracle = GaussianBodyOracle(PreparedScene(scene))
    ax_i = {"u": 0, "v": 1, "z": 2}
    report, fig_data = {}, {}
    for name, f in faces(doc).items():
        (n1, g1), (n2, g2) = f["grid"]
        occ = np.zeros((len(g1), len(g2)), dtype="U10")
        counts = {"free": 0, "occupied": 0, "unknown": 0}
        free_examples = []
        for i, x1 in enumerate(g1):
            for j, x2 in enumerate(g2):
                pa, pb = np.zeros(3), np.zeros(3)
                pa[ax_i[n1]] = pb[ax_i[n1]] = x1
                pa[ax_i[n2]] = pb[ax_i[n2]] = x2
                pa[ax_i[f["axis"]]], pb[ax_i[f["axis"]]] = f["a"], f["b"]
                rep = oracle.edge(Pose3(tuple(frame.to_world(pa))), Pose3(tuple(frame.to_world(pb))), UAV, margin_m=.05)
                occ[i, j] = rep.occupancy
                counts[rep.occupancy] += 1
                if rep.occupancy == "free" and len(free_examples) < 20:
                    free_examples.append({n1: float(x1), n2: float(x2), "clearance_lower_m": rep.clearance_lower_m})
        report[name] = {"grid_axes": [n1, n2], "grid_1": [float(x) for x in g1], "grid_2": [float(x) for x in g2],
                        "crossing_axis": f["axis"], "from": f["a"], "to": f["b"], "counts": counts,
                        "closed": counts["free"] == 0, "free_examples": free_examples}
        fig_data[name] = (n1, g1, n2, g2, occ)
        print(name, counts, flush=True)
    report["_meta"] = {"archive_sha256": doc["derivative"]["sha256"], "body": {"r": .25, "h": .10, "margin": .05},
                       "crop": crop, "oracle_stats": oracle.stats}
    (a.output / "probes.json").write_text(json.dumps(report, indent=1))
    fig, axs = plt.subplots(len(fig_data), 1, figsize=(10, 3.2 * len(fig_data)))
    cmap = {"free": 2, "unknown": 1, "occupied": 0}
    for ax, (name, (n1, g1, n2, g2, occ)) in zip(axs, fig_data.items()):
        img = np.vectorize(cmap.get)(occ).T
        ax.imshow(img, origin="lower", aspect="auto", vmin=0, vmax=2, cmap="RdYlGn",
                  extent=[g1[0] - .05, g1[-1] + .05, g2[0] - .05, g2[-1] + .05])
        c = report[name]["counts"]
        ax.set_title(f"{name}: crossing along {report[name]['crossing_axis']} — free {c['free']}, "
                     f"unknown {c['unknown']}, occupied {c['occupied']} (green = free crossing)", fontsize=9)
        ax.set_xlabel(n1); ax.set_ylabel(n2)
    fig.tight_layout(); fig.savefig(a.output / "probes.png", dpi=90)


if __name__ == "__main__":
    main()
