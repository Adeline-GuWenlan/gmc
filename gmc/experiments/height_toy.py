"""T1/T2 gates for height-band projection (spec §5.1-§5.2). One gate per call."""
import argparse
import json
from pathlib import Path

import numpy as np
from shapely.geometry import box

from gmc.config import load_config
from gmc.height.pathio import crossing_thetas, curve_from_dict, path_crosses
from gmc.height.prism import robot_table
from gmc.height.project import project_scene
from gmc.height.replay3d import replay_curve
from gmc.height.run import compile_and_query, safe_areas, with_overrides
from gmc.height.synth3d import GOAL, START, TABLETOP_XY, WORKSPACE, Z_FLOOR, table_scene
from gmc.height.viz import robot_video
from gmc.synth import gate_half_angle, single_door

RESULTS = Path("results/height/toy")
RAW = Path("/scratch/wg2381/splathjb/gmc/outputs/height/toy")
TABLE = box(*TABLETOP_XY)


def _cfg():
    return load_config("configs/height_toy.yaml")


def _video(gate, scene2d, robot, res, scene3d=None):
    out = robot_video(scene2d, robot, res, RESULTS / "media" / gate, scene3d=scene3d,
                      z_floor=Z_FLOOR, title=gate)
    return {"video": str(out["video"]) if out["video"] else None,
            "frames": [str(p) for p in out["frames"]]}


def gate_T1(gate):
    width = 0.6 if gate == "T1a" else 0.35
    robot = robot_table()["ellipse_toy"]
    scene = single_door(width)
    res, _ = compile_and_query(scene, robot, _cfg(), (-2.0, 0.0, np.pi / 2),
                               (2.0, 0.0, np.pi / 2), out_dir=RAW / gate)
    if gate == "T1b":
        return {"criteria": {"not_reachable": res["status"] != "REACHABLE"}, "result": res}
    half = gate_half_angle(0.50, 0.20, width)
    thetas = crossing_thetas(curve_from_dict(res["curve"]), 0.0) if res["curve"] else []
    folded = [min(t % np.pi, np.pi - t % np.pi) for t in thetas]
    crit = {"reachable": res["status"] == "REACHABLE",
            "verify_certified": bool(res["verify"].get("certified")),
            "crossing_inside_gate": bool(folded) and all(f < half for f in folded)}
    return {"criteria": crit, "result": res,
            "extra": {"gate_half_angle": half, "crossing_thetas": thetas,
                      "media": _video(gate, scene, robot, res)}}


T2_CASES = {  # gate: (variant, robot key, expectation)
    "T2-1": ("closed", "sweeper", "reach_cross"),
    "T2-2": ("closed", "quadruped", "reach"),
    "T2-3": ("closed", "cylinder", "blocked"),
    "T2-4": ("open", "cylinder", "reach_avoid"),
    "T2-5": ("closed", "uav", "reach_cross"),
}


def gate_T2_run(gate):
    variant, key, expect = T2_CASES[gate]
    s3, _ = table_scene(variant)
    robot = robot_table(1.20)[key]
    s2, stats = project_scene(s3, robot, WORKSPACE, z_floor=Z_FLOOR)
    res, _ = compile_and_query(s2, robot, _cfg(), START, GOAL, out_dir=RAW / gate)
    rep = None
    if res["curve"] is not None:
        rep = replay_curve(s3, robot, curve_from_dict(res["curve"]), z_floor=Z_FLOOR)
        res["replay3d"] = rep
    if expect == "blocked":
        crit = {"not_reachable": res["status"] != "REACHABLE"}
    else:
        crit = {"reachable": res["status"] == "REACHABLE",
                "verify_certified": bool(res["verify"].get("certified")),
                "replay3d_passed": bool(rep and rep["passed"])}
        crosses = bool(res["curve"]) and path_crosses(curve_from_dict(res["curve"]), TABLE)
        if expect == "reach_cross":
            crit["crosses_tabletop"] = crosses
        if expect == "reach_avoid":
            crit["avoids_tabletop"] = bool(res["curve"]) and not crosses
    return {"criteria": crit, "result": res, "replay3d": rep,
            "extra": {"projection": stats, "media": _video(gate, s2, robot, res, s3)}}


def gate_T2_6():
    s3, _ = table_scene("closed")
    line = {"schema_version": 2, "segments": [
        {"kind": "TRANSLATION", "q0": list(START), "q1": list(GOAL)}]}
    rep = replay_curve(s3, robot_table()["cylinder"], curve_from_dict(line), z_floor=Z_FLOOR)
    return {"criteria": {"replay3d_rejects": not rep["passed"]}, "replay3d": rep}


def gate_T2_7():
    s3, meta = table_scene("closed")
    g = {k: set(v) for k, v in meta["groups"].items()}
    ids = {}
    for key in ("sweeper", "cylinder", "uav"):
        s2, _ = project_scene(s3, robot_table(1.20)[key], WORKSPACE, z_floor=Z_FLOOR)
        ids[key] = {s.primitive_id for s in s2.supports}
    crit = {"sweeper_legs_no_top": bool(ids["sweeper"] & g["legs"]) and not ids["sweeper"] & g["tabletop"],
            "cylinder_legs_and_top": bool(ids["cylinder"] & g["legs"]) and bool(ids["cylinder"] & g["tabletop"]),
            "uav_walls_only": bool(ids["uav"] & g["wall"]) and not ids["uav"] & (g["legs"] | g["tabletop"])}
    return {"criteria": crit, "extra": {k: len(v) for k, v in ids.items()}}


def gate_T2_8():
    s3, _ = table_scene("closed")
    robot = robot_table()["sweeper"]
    s2, _ = project_scene(s3, robot, WORKSPACE, z_floor=Z_FLOOR)
    runs = {}
    for n in (1, 16):
        res, mc = compile_and_query(s2, robot, with_overrides(_cfg(), initial_intervals=n),
                                    START, GOAL, out_dir=RAW / f"T2-8_n{n}")
        runs[n] = {"status": res["status"], "areas": safe_areas(mc),
                   "compile_seconds": res["compile_seconds"]}
    ref = runs[1]["areas"][0]["area"]
    all_areas = [a["area"] for n in (1, 16) for a in runs[n]["areas"]]
    rel = max(abs(a - ref) for a in all_areas) / ref if ref > 0 else float("inf")
    same = runs[1]["status"] == runs[16]["status"] and rel <= 1e-6
    return {"criteria": {"same_status": runs[1]["status"] == runs[16]["status"],
                         "areas_within_1e-6": rel <= 1e-6},
            "extra": {"runs": runs, "max_rel_area_diff": rel,
                      "showcase_initial_intervals": 1 if same else 16}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gate", required=True,
                    choices=["T1a", "T1b", "T2-1", "T2-2", "T2-3", "T2-4", "T2-5",
                             "T2-6", "T2-7", "T2-8"])
    gate = ap.parse_args().gate
    RESULTS.mkdir(parents=True, exist_ok=True)
    if gate.startswith("T1"):
        out = gate_T1(gate)
    elif gate in T2_CASES:
        out = gate_T2_run(gate)
    else:
        out = {"T2-6": gate_T2_6, "T2-7": gate_T2_7, "T2-8": gate_T2_8}[gate]()
    out["gate"] = gate
    out["pass"] = all(out["criteria"].values())
    (RESULTS / f"{gate}.json").write_text(json.dumps(out, indent=2, default=str))
    print(f"{gate} pass={out['pass']} criteria={out['criteria']}", flush=True)


if __name__ == "__main__":
    main()
