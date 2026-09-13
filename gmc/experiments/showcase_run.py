# gmc/experiments/showcase_run.py
"""One robot on the showcase case: project, probe, compile+query, replay3d (spec §5.4)."""
import argparse
import json
import time
from pathlib import Path

import numpy as np

from gmc.config import load_config
from gmc.height.pathio import curve_from_dict
from gmc.height.prism import robot_table
from gmc.height.project import project_scene
from gmc.height.replay3d import replay_curve
from gmc.height.run import compile_and_query, with_overrides
from showcase_scene import RES, load_processed

RAW = Path("/scratch/wg2381/splathjb/gmc/outputs/height/showcase")
PROBE_HALF = 1.0          # probe window: 2 m x 2 m around the overhang midpoint
ABORT_HOURS = 20.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", required=True, choices=["cylinder", "sweeper", "uav"])
    ap.add_argument("--tau", type=float, default=0.3)
    ap.add_argument("--budget-scale", type=int, default=1)
    ap.add_argument("--skip-probe", action="store_true")
    a = ap.parse_args()
    name = a.robot + ("" if a.tau == 0.3 else f"_tau{a.tau:g}") + ("" if a.budget_scale == 1 else f"_b{a.budget_scale}")
    case = json.loads((RES / "case.json").read_text())
    scene3d, g0 = load_processed()
    robot = robot_table(case["z_c"])[a.robot]
    cfg = load_config("configs/height_showcase.yaml")
    k = a.budget_scale
    if k > 1:
        cfg = with_overrides(cfg, max_refinement_rounds=cfg.query.max_refinement_rounds * k,
                             max_wall_seconds=cfg.query.max_wall_seconds * k,
                             max_support_calls=cfg.query.max_support_calls * k)
    out = {"robot": a.robot, "tau": a.tau, "budget_scale": k, "case": case}
    s2, stats = project_scene(scene3d, robot, case["window"], z_floor=case["z_floor"], tau=a.tau)
    out["projection"] = stats
    print("projection", json.dumps(stats), flush=True)

    probe = {"n_supports_full": len(s2.supports), "abort": False}
    if not a.skip_probe:
        ov = case.get("overhang") or {}
        st, gl = np.array(case["start"][:2]), np.array(case["goal"][:2])
        mid_s = np.mean(ov["s_interval"]) if ov.get("s_interval") else 0.5 * np.linalg.norm(gl - st)
        c = st + (gl - st) / np.linalg.norm(gl - st) * mid_s
        pw = [c[0] - PROBE_HALF, c[1] - PROBE_HALF, c[0] + PROBE_HALF, c[1] + PROBE_HALF]
        sp, _ = project_scene(scene3d, robot, pw, z_floor=case["z_floor"], tau=a.tau)
        t0 = time.time()
        pcfg = with_overrides(cfg, initial_intervals=1, max_depth=0)
        compile_and_query(sp, robot, pcfg, (pw[0] + 0.3, c[1], 0.0), (pw[2] - 0.3, c[1], 0.0))
        dt = time.time() - t0
        n_int = cfg.orientation.initial_intervals
        ratio = len(s2.supports) / max(1, len(sp.supports))
        hours = dt * ratio * n_int / 3600.0
        probe.update(n_supports_probe=len(sp.supports), probe_compile_seconds=dt,
                     projected_hours=hours, extrapolation="linear in supports x initial_intervals (rough)",
                     abort=hours > ABORT_HOURS)
    out["probe"] = probe
    print("probe", json.dumps(probe), flush=True)
    if probe["abort"]:
        out["result"] = out["replay3d"] = None
    else:
        res, _ = compile_and_query(s2, robot, cfg, case["start"], case["goal"], out_dir=RAW / name)
        out["result"] = res
        out["replay3d"] = None
        if res["curve"] is not None:
            out["replay3d"] = replay_curve(scene3d, robot, curve_from_dict(res["curve"]),
                                           z_floor=case["z_floor"], tau=a.tau)
            res["replay3d"] = out["replay3d"]
    (RES / f"{name}.json").write_text(json.dumps(out, indent=2, default=str))
    r = out["result"] or {}
    print(f"=== {name} status={r.get('status')} verify={r.get('verify')} "
          f"replay3d={(out['replay3d'] or {}).get('passed')} abort={probe['abort']} ===", flush=True)


if __name__ == "__main__":
    main()
