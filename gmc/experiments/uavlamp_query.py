"""One production UAV planner query on a derivative archive, plus plots and replay.

A query is start + goal only: exactly one ``LatticePlanner.plan`` call, no
waypoints, no per-segment altitude limits, no cost terms.  The only inputs are
the archive, the box (route-frame prism = bounds + declared known space), the
start and goal, and the planner resolution/margin/budget.  Test-only variants
(drop added rows by role, append a plug) are applied in memory and recorded.

Usage (from ``gmc/``):
    python experiments/uavlamp_query.py SPEC.json [SPEC.json ...]
Each SPEC writes ``<output>/{result,summary}.json`` and ``top.png``/``side.png``.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Rectangle
import numpy as np

from gmc.gs3d.contracts import (BodySpec, GoalRegion, PlannerConfig, Pose3,
                                SceneSpec, SearchBudget)
from gmc.gs3d.integration import RouteBoxKnownSpace
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.planner import LatticePlanner
from gmc.gs3d.robots import crop_by_support_aabb
from gmc.gs3d.trajectory import replay_plan, sample_linear_trajectory
from gmc.height.ply3d import GaussianScene3D

TAU, LEVEL = .3, 2.
UAV = BodySpec("uav", .25, .10, "uav_translation")


def sha256(path) -> str:
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(8 << 20), b""):
            h.update(block)
    return h.hexdigest()


class Frame:
    """Route frame: world = route @ R + origin (z of origin = floor)."""

    def __init__(self, origin, rotation):
        self.origin = np.asarray(origin, float)
        self.R = np.asarray(rotation, float)

    def to_world(self, local):
        return np.asarray(local, float) @ self.R + self.origin

    def to_route(self, world):
        return (np.asarray(world, float) - self.origin) @ self.R.T


def load_archive(path):
    with np.load(path, allow_pickle=False) as d:
        scene = GaussianScene3D(d["means"], d["covs"], d["opacity"], d["ids"], "archive")
        meta = json.loads(str(d["meta"])) if "meta" in d.files else {}
    return scene, meta


def _extra_rows(spec, frame):
    """Test-only rows appended in memory (plugs, probes). Never written to an archive."""
    extras = spec.get("extra_builders", [])
    if not extras:
        return None, []
    from gmc.gs3d import scene_uavlamp
    rows, records = [], []
    for item in extras:
        g = scene_uavlamp.build_test_only(item, frame)
        rows.append(g)
        records.append({**item, "rows": int(len(g))})
    return rows, records


class FaceLoggingKnownSpace:
    """Delegates to the route prism; counts which prism face(s) each rejected AABB exceeds.

    Pure bookkeeping for reporting ``map_unknown``: the planner sees identical answers.
    """

    def __init__(self, inner):
        self.inner = inner
        self.face_counts = {}

    def contains_aabb(self, lower, upper):
        ok = self.inner.contains_aabb(lower, upper)
        if not ok:
            lo, hi = np.asarray(lower, float), np.asarray(upper, float)
            corners = np.asarray([[a, b, c] for a in (lo[0], hi[0]) for b in (lo[1], hi[1])
                                  for c in (lo[2], hi[2])])
            route = (corners - np.asarray(self.inner.origin_world_m)) @ np.asarray(self.inner.world_to_route).T
            dlo, dhi = np.asarray(self.inner.lower_route_m), np.asarray(self.inner.upper_route_m)
            faces = [f"{ax}_{side}" for k, ax in enumerate("uvz")
                     for side, bad in (("min", np.any(route[:, k] < dlo[k] - 1e-12)),
                                       ("max", np.any(route[:, k] > dhi[k] + 1e-12))) if bad]
            key = "+".join(faces) or "invalid"
            self.face_counts[key] = self.face_counts.get(key, 0) + 1
        return ok


def build_scene(spec, full, manifest_doc):
    frame = Frame(spec["frame"]["origin_world_m"], spec["frame"]["world_to_route"])
    drop_roles = set(spec.get("drop_roles", []))
    dropped = []
    if drop_roles:
        if manifest_doc is None:
            raise ValueError("drop_roles needs the manifest")
        ids = [int(i) for e in manifest_doc["edits"] if e["role"] in drop_roles
               for i in range(e["id_range"][0], e["id_range"][1] + 1)]
        keep = ~np.isin(full.ids, np.asarray(ids, dtype=np.int64))
        dropped = sorted(drop_roles)
        full = full.subset(keep)
    extra, extra_records = _extra_rows(spec, frame)
    if extra:
        base_max = int(full.ids.max()) + 1_000_000
        means = [full.means] + [g.means for g in extra]
        covs = [full.covs] + [g.covs for g in extra]
        op = [full.opacity] + [g.opacity for g in extra]
        ids, nxt = [full.ids], base_max
        for g in extra:
            ids.append(np.arange(nxt, nxt + len(g), dtype=np.int64))
            nxt += len(g)
        full = GaussianScene3D(np.vstack(means), np.concatenate(covs), np.concatenate(op),
                               np.concatenate(ids), "archive+test_only")
    box = spec["box_route"]
    known = RouteBoxKnownSpace(tuple(frame.origin), tuple(map(tuple, frame.R)),
                               tuple(box["lower"]), tuple(box["upper"]))
    bmin, bmax = known.world_bounds()
    known = FaceLoggingKnownSpace(known)
    cropped, crop = crop_by_support_aabb(full, bmin, bmax, level=LEVEL, tau=TAU)
    scene = SceneSpec(spec["name"], cropped, bmin, bmax, TAU, LEVEL, known, None,
                      {"coverage_policy": "assumed_map_domain (declared route prism)",
                       "crop": crop, "dropped_roles_test_only": dropped,
                       "extra_test_only": extra_records, "seed": 0})
    return frame, scene, crop, dropped, extra_records


def footprint_mask(route_xy, lamp):
    """Centre inside the lamp's plan footprint (rect in route uv)."""
    lo, hi = np.asarray(lamp["footprint_route_uv"][0]), np.asarray(lamp["footprint_route_uv"][1])
    return np.all((route_xy >= lo) & (route_xy <= hi), axis=1)


def analyse(result, frame, spec):
    out = {"status": result["status"], "reason": result["reason"],
           "expansions": result["diagnostics"]["expansions"],
           "visited_nodes": result["diagnostics"]["visited_nodes"],
           "oracle_calls": result["diagnostics"].get("oracle_calls"),
           "occupied_rejections": result["diagnostics"]["occupied_rejections"],
           "unproven_rejections": result["diagnostics"]["unproven_rejections"],
           "map_unknown_rejections": result["diagnostics"]["map_unknown_rejections"],
           "algorithm_wall_s": result["timings"]["algorithm_wall_s"],
           "clearance_lower_m": result["clearance_lower_m"]}
    if result["trajectory"] is None:
        return out, None
    samples = sample_linear_trajectory(result["trajectory"], dt_s=.02)
    rows = np.asarray(samples["poses"], float)[:, :3]
    local = frame.to_route(rows)
    knots = frame.to_route(np.asarray(result["trajectory"]["poses"], float)[:, :3])
    out.update(path_length_m=result["diagnostics"]["path_length_m"],
               altitude_range_m=result["diagnostics"]["altitude_range_m"],
               knots_route=knots.round(4).tolist(),
               z_start=float(local[0, 2]), z_goal=float(local[-1, 2]),
               z_min=float(local[:, 2].min()), z_max=float(local[:, 2].max()))
    lamp = spec.get("lamp")
    if lamp:
        inside = footprint_mask(local[:, :2], lamp)
        under = inside & (local[:, 2] + UAV.half_height_m <= lamp["underside_z"])
        over = inside & (local[:, 2] - UAV.half_height_m >= lamp["top_z"])
        lateral = lamp["footprint_route_uv"]
        u_in = (local[:, 0] >= lateral[0][0]) & (local[:, 0] <= lateral[1][0])
        out["lamp"] = {
            "footprint_route_uv": lateral, "underside_z": lamp["underside_z"],
            "centre_samples_inside_footprint": int(inside.sum()),
            "passes_under": bool(under.any()),
            "passes_over": bool(over.any()),
            "z_inside_footprint": ([float(local[inside, 2].min()), float(local[inside, 2].max())]
                                   if inside.any() else None),
            "v_range_while_u_in_lamp_span": ([float(local[u_in, 1].min()), float(local[u_in, 1].max())]
                                              if u_in.any() else None),
            "z_range_while_u_in_lamp_span": ([float(local[u_in, 2].min()), float(local[u_in, 2].max())]
                                              if u_in.any() else None),
            "goal_minus_min_z_under_lamp": (float(local[-1, 2] - local[inside, 2].min())
                                            if inside.any() else None)}
        if under.any():
            first_under = int(np.flatnonzero(under)[0])
            high = np.flatnonzero(local[:, 2] >= local[-1, 2] - .05)
            out["lamp"]["under_before_reaching_goal_altitude"] = bool(
                len(high) == 0 or first_under < int(high[0]))
    return out, local


def plot(spec, scene, frame, local, output, summary, highlight_ids):
    g = scene.gaussians
    op = g.opacity > TAU
    loc = frame.to_route(g.means[op])
    half = LEVEL * np.sqrt(np.einsum("nii->ni", g.covs[op]))
    ids = g.ids[op]
    edit = np.isin(ids, highlight_ids)
    box = spec["box_route"]
    lo, hi = np.asarray(box["lower"]), np.asarray(box["upper"])
    views = {"top": (0, 1, "u (m, route forward)", "v (m, route lateral)"),
             "side": (0, 2, "u (m, route forward)", "z above floor (m)")}
    for name, (a, b, la, lb) in views.items():
        fig, ax = plt.subplots(figsize=(11, 6.5))
        if name == "side":
            slab = spec.get("side_slab_v", [lo[1], hi[1]])
            sel = (loc[:, 1] >= slab[0]) & (loc[:, 1] <= slab[1])
        else:
            zb = spec.get("top_band_z", [lo[2], hi[2]])
            sel = (loc[:, 2] + half[:, 2] >= zb[0]) & (loc[:, 2] - half[:, 2] <= zb[1])
        src = sel & ~edit
        ax.scatter(loc[src, a], loc[src, b], s=1.5, c=loc[src, 2], cmap="Greys", vmin=-1, vmax=4,
                   alpha=.35, linewidths=0, label="source Gaussians (opaque centres)")
        for role, colour in spec.get("role_colours", {}).items():
            rid = np.asarray(spec.get("role_ids", {}).get(role, []), dtype=np.int64)
            m = sel & np.isin(ids, rid)
            if m.any():
                ax.scatter(loc[m, a], loc[m, b], s=3, color=colour, linewidths=0, label=role)
        rest = sel & edit & ~np.isin(ids, np.concatenate(
            [np.asarray(v, dtype=np.int64) for v in spec.get("role_ids", {}).values()] or [np.empty(0, np.int64)]))
        if rest.any():
            ax.scatter(loc[rest, a], loc[rest, b], s=6, color="orange", linewidths=0, label="added edit")
        lamp = spec.get("lamp")
        if lamp:
            (u0, v0), (u1, v1) = lamp["footprint_route_uv"]
            if name == "top":
                ax.add_patch(Rectangle((u0, v0), u1 - u0, v1 - v0, fill=False, ec="gold", lw=2,
                                       label="lamp footprint"))
            else:
                ax.add_patch(Rectangle((u0, lamp["underside_z"]), u1 - u0, lamp["top_z"] - lamp["underside_z"],
                                       fill=False, ec="gold", lw=2, label="lamp (underside..top)"))
        table = spec.get("table")
        if table:
            (u0, v0), (u1, v1) = table["top_route_uv"]
            if name == "top":
                ax.add_patch(Rectangle((u0, v0), u1 - u0, v1 - v0, fill=False, ec="saddlebrown", lw=2,
                                       ls="--", label="table top footprint"))
            else:
                ax.plot([u0, u1], [table["top_z"]] * 2, color="saddlebrown", lw=2.5, label="table top")
        ax.add_patch(Rectangle((lo[a], lo[b]), hi[a] - lo[a], hi[b] - lo[b], fill=False, ec="k",
                               ls=":", lw=1, label="planning box"))
        start, goal = np.asarray(spec["start_route"]), np.asarray(spec["goal_route"])
        if local is not None:
            ax.plot(local[:, a], local[:, b], color="deepskyblue", lw=2.5, label="planned route (centre)")
            if name == "top":
                ax.fill_between([], [], [])
                for k in range(0, len(local), max(1, len(local) // 40)):
                    ax.add_patch(Ellipse((local[k, 0], local[k, 1]), .5, .5, fill=False,
                                         ec="deepskyblue", lw=.4, alpha=.5))
            else:
                ax.fill_between(local[:, 0], local[:, 2] - .10, local[:, 2] + .10, color="deepskyblue",
                                alpha=.15, lw=0)
        ax.scatter([start[a]], [start[b]], c="lime", s=80, edgecolors="k", zorder=6, label="start")
        ax.scatter([goal[a]], [goal[b]], c="red", s=80, edgecolors="k", zorder=6, label="goal")
        pad = .3
        ax.set_xlim(lo[a] - pad, hi[a] + pad); ax.set_ylim(lo[b] - pad, hi[b] + pad)
        ax.set_aspect("equal"); ax.set_xlabel(la); ax.set_ylabel(lb)
        lam = summary.get("lamp", {})
        ax.set_title(f"{spec['name']} — {name} view — status {summary['status']} ({summary['reason']})\n"
                     f"passes_under={lam.get('passes_under')} passes_over={lam.get('passes_over')} "
                     f"len={summary.get('path_length_m', float('nan')):.2f} m  exp={summary['expansions']}",
                     fontsize=9)
        ax.legend(loc="upper left", fontsize=6, markerscale=3, bbox_to_anchor=(1.01, 1))
        fig.tight_layout()
        fig.savefig(output / f"{name}.png", dpi=110)
        plt.close(fig)


def run_one(spec_path: Path, cache: dict):
    spec = json.loads(Path(spec_path).read_text())
    output = Path(spec["output"])
    output.mkdir(parents=True, exist_ok=True)
    archive = Path(spec["archive"])
    if str(archive) not in cache:
        cache.clear()
        cache[str(archive)] = (load_archive(archive), sha256(archive))
    (full, meta), digest = cache[str(archive)]
    if spec.get("archive_sha256") and spec["archive_sha256"] != digest:
        raise ValueError("archive hash differs from spec")
    manifest_doc = json.loads(Path(spec["manifest"]).read_text()) if spec.get("manifest") else None
    frame, scene, crop, dropped, extras = build_scene(spec, full, manifest_doc)
    prepared = PreparedScene(scene)
    start = Pose3(tuple(map(float, frame.to_world(spec["start_route"]))))
    goal = Pose3(tuple(map(float, frame.to_world(spec["goal_route"]))))
    config = PlannerConfig(resolution_m=spec.get("resolution_m", .10),
                           margin_m=spec.get("margin_m", .05), seed=0,
                           budget=SearchBudget(**spec.get("budget", {})))
    result = LatticePlanner(prepared).plan(scene, UAV, start, GoalRegion(goal), config)
    replay = None
    if result["status"] == "success":
        # Independent re-validation with a fresh oracle on a freshly prepared index.
        replay = replay_plan(result, GaussianBodyOracle(PreparedScene(scene)))
        replay = {k: v for k, v in replay.items() if k != "samples"}
    summary, local = analyse(result, frame, spec)
    summary["map_unknown_rejections_by_box_face"] = scene.known_space.face_counts
    summary.update(name=spec["name"], archive=str(archive), archive_sha256=digest,
                   start_route=spec["start_route"], goal_route=spec["goal_route"],
                   start_world=list(start.xyz), goal_world=list(goal.xyz),
                   box_route=spec["box_route"], bounds_world=[scene.bounds_min, scene.bounds_max],
                   resolution_m=config.resolution_m, margin_m=config.margin_m,
                   budget=spec.get("budget", {}), crop=crop, dropped_roles_test_only=dropped,
                   extra_test_only=extras, planner_calls=1,
                   independent_replay_passed=(replay or {}).get("passed"),
                   endpoint_reports={k: {kk: v.get(kk) for kk in ("occupancy", "reason", "clearance_lower_m")}
                                     for k, v in result["endpoint_reports"].items()})
    (output / "result.json").write_text(json.dumps({"result": result, "replay": replay}, allow_nan=False) + "\n")
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    highlight = []
    if manifest_doc is not None:
        highlight = [i for e in manifest_doc.get("edits", []) for i in range(e["id_range"][0], e["id_range"][1] + 1)]
        highlight += [r["gaussian_id"] for r in manifest_doc.get("manual_geometry", [])]
    if extras:
        highlight += list(range(int(full.ids.max()) + 1_000_000, int(scene.gaussians.ids.max()) + 1))
    if manifest_doc is not None and "edits" in manifest_doc:
        colours = {"lamp": "gold", "drop_ceiling": "tab:purple", "bulkhead": "tab:green",
                   "back_panel": "tab:blue", "partition": "tab:cyan"}
        spec.setdefault("role_ids", {})
        for e in manifest_doc["edits"]:
            spec["role_ids"].setdefault(e["role"], []).extend(range(e["id_range"][0], e["id_range"][1] + 1))
            spec.setdefault("role_colours", {}).setdefault(e["role"], colours.get(e["role"], "orange"))
        if extras:
            spec["role_ids"]["test_only_plug"] = list(range(int(full.ids.max()) + 1_000_000,
                                                            int(scene.gaussians.ids.max()) + 1))
            spec["role_colours"]["test_only_plug"] = "red"
    plot(spec, scene, frame, local, output, summary, np.asarray(highlight, dtype=np.int64))
    print(json.dumps({k: summary.get(k) for k in ("name", "status", "reason", "expansions",
                                                  "path_length_m", "lamp")}, default=str), flush=True)
    return summary


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("specs", nargs="+", type=Path)
    args = p.parse_args(argv)
    cache = {}
    failed = 0
    for s in args.specs:
        try:
            run_one(s, cache)
        except Exception as exc:  # keep going; record per spec
            failed += 1
            print(f"SPEC FAILED {s}: {exc!r}", file=sys.stderr, flush=True)
            import traceback; traceback.print_exc()
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
