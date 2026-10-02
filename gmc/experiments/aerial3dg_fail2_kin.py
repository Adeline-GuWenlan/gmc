"""F2 Task 2.1: classify the cylinder ``shared_replay_failed`` rows of F1's widened-box runs (W1, W2).

Run via sbatch from ``gmc/`` (loads a read-only F1 widened compile).  Extends F1's ``fixprobe kinematics``:
for every vetoed row, re-query with the same verdict-only config on the same compile, rebuild the exported
gs3d trajectory exactly as ``api.query`` does, record which trajectory steps break ``verify_linear_trajectory``
(in-place turn = yaw-rate half, translation = speed half), then replay three exports with the shared gs3d oracle:

  original      as exported (reproduces the veto)
  turn_floor    F1's fix: every in-place turn lasts >= 1 ms
  both_floors   turn floor + every translation lasts >= 1 ms (the "translation variant" F1 left untested)

Same poses / path geometry in all three; only the timing of tiny steps changes.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from gmc.aerial3d import api as a3api
from gmc.aerial3d.api import load_compiled, query
from gmc.gs3d.contracts import Pose3
from gmc.gs3d.oracle import GaussianBodyOracle
from gmc.gs3d.planner import _limits, _row
from gmc.gs3d.trajectory import replay_plan
from gmc.gs3d.validation import angle_delta

from aerial3dg_fail_diag import _dump
from aerial3dg_fail_fixprobe import kin_steps
from aerial3dg_fail_widen import FAST


def floored_trajectory(path, body, start_yaw, end_yaw, min_turn_s=1e-3, min_trans_s=0.):
    """``planner._linear_trajectory`` with in-place turns >= ``min_turn_s`` and translations >= ``min_trans_s``."""
    limits = _limits(body)
    poses, times = [Pose3(path[0].xyz, start_yaw)], [0.]

    def append(q, seconds):
        if seconds > 1e-12:
            poses.append(q)
            times.append(times[-1] + seconds)

    for q in path[1:]:
        prev = poses[-1]
        delta = np.asarray(q.xyz) - prev.xyz
        length = float(np.linalg.norm(delta))
        if length <= 1e-12:
            continue
        yaw = prev.yaw
        turn = angle_delta(math.atan2(delta[1], delta[0]), yaw)
        yaw += turn
        append(Pose3(prev.xyz, yaw), max(abs(turn) / limits["max_yaw_rate_radps"], min_turn_s))   # as F1: every turn
        append(Pose3(q.xyz, yaw), max(length / limits["max_speed_mps"], min_trans_s))
    turn = angle_delta(end_yaw, poses[-1].yaw)
    append(Pose3(poses[-1].xyz, poses[-1].yaw + turn), max(abs(turn) / limits["max_yaw_rate_radps"], min_turn_s))
    return {"poses": [_row(q) for q in poses], "time_s": times, "interpolation": "linear_xyz_yaw",
            "segments": [], "control_dt_s": .05}


def _replay(gs, traj, oracle):
    fixed = dict(gs)
    fixed["trajectory"] = traj
    fixed["attained_goal"] = traj["poses"][-1]
    rep = replay_plan(fixed, oracle)
    return {"passed": rep["passed"], "geometry_passed": rep["geometry"].get("passed"),
            "geometry_reason": rep["geometry"].get("reason"), "kinematics": rep["kinematics"],
            "attained_matches": rep["attained_matches"]}


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--a3c", type=Path, required=True)
    p.add_argument("--rows", type=Path, required=True, help="F1 fast_00.jsonl of the widened run")
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)
    compiled = load_compiled(a.a3c)
    body, z_c = compiled.body, compiled.domain.ground_z
    oracle = GaussianBodyOracle(compiled.prepared)
    src = [json.loads(l) for l in open(a.rows)]
    rows = sorted({r["index"]: r for r in src if r["reason"] == "shared_replay_failed"}.values(), key=lambda r: r["index"])
    out = []
    for r in rows:
        s = compiled.frame.to_world([*r["start_uv"], z_c])
        g = compiled.frame.to_world([*r["goal_uv"], z_c])
        q = query(compiled, s, g, config=FAST, call_id=r["pair_id"])
        rec = {"index": r["index"], "pair_id": r["pair_id"], "status_now": q["status"], "reason_now": q["reason"]}
        if q["polyline_world"] is None:
            out.append(rec)
            continue
        dens = a3api._densify(np.asarray(q["polyline_world"], float), FAST.export_max_segment_m)
        gs = a3api._gs3d_result(compiled, dens, g, q["verification"]["own"]["clearance_lower_m"])
        bad = kin_steps(gs["trajectory"], body)
        path = [Pose3(tuple(map(float, x))) for x in dens]
        kinds = sorted({"turn" if v["translation_m"] == 0 else "translation" for v in bad})
        rec.update(violations=bad[:5], n_violations=len(bad), violation_kinds=kinds,
                   min_translation_m=min((v["translation_m"] for v in bad if v["translation_m"] > 0), default=None),
                   original=_replay(gs, gs["trajectory"], oracle),
                   turn_floor=_replay(gs, floored_trajectory(path, body, 0., 0., 1e-3, 0.), oracle),
                   both_floors=_replay(gs, floored_trajectory(path, body, 0., 0., 1e-3, 1e-3), oracle))
        out.append(rec)
        print(r["pair_id"], q["reason"], kinds, "orig/turn/both passed", rec["original"]["passed"],
              rec["turn_floor"]["passed"], rec["both_floors"]["passed"], flush=True)
    summ = {}
    for x in out:
        if "original" not in x:
            k = f"no_route_now:{x['status_now']}:{x['reason_now']}"
        else:
            k = "+".join(x["violation_kinds"]) or "no_kinematic_violation"
            k += (f"|geom={x['original']['geometry_passed']}|turn_floor={x['turn_floor']['passed']}"
                  f"|both_floors={x['both_floors']['passed']}")
        summ[k] = summ.get(k, 0) + 1
    _dump(a.out, {"a3c": str(a.a3c), "rows_source": str(a.rows), "n": len(out), "summary": summ, "rows": out})
    print(json.dumps(summ, indent=1), flush=True)


if __name__ == "__main__":
    main()
