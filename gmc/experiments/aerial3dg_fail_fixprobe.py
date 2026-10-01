"""F1 Task 2/3: evidence for the mechanism (and a candidate fix) of two UNKNOWN classes, without changing src.

Run via sbatch from ``gmc/`` (loads the read-only G2 compile).

* ``kinematics``  sweeper ``shared_replay_failed`` rows: re-query on the G2 compile, rebuild the exported
                  gs3d trajectory exactly as ``api.query`` does, and record the trajectory step that breaks
                  ``verify_linear_trajectory`` (elapsed time, yaw change, rate excess).  Then rebuild it with
                  every in-place turn given >= 1 ms (same poses and path geometry, only the timing of tiny turns changes)
                  and replay that with the shared gs3d oracle.
* ``endpoints``   ``*_not_certified_free`` rows: re-query with the endpoint query cell grown with buffer 0
                  instead of the octree buffer (monkeypatch of ``api.grow_cell`` for ``kind="query"`` only);
                  the own verifier and the shared gs3d replay still decide the verdict.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import numpy as np

from gmc.aerial3d import api as a3api
from gmc.aerial3d.api import load_compiled, query
from gmc.gs3d.contracts import Pose3
from gmc.gs3d.oracle import GaussianBodyOracle
from gmc.gs3d.planner import _limits
from gmc.gs3d.trajectory import replay_plan
from gmc.gs3d.validation import angle_delta, verify_linear_trajectory

from aerial3dg_run import QCONFIG
from aerial3dg_fail_diag import A3C, _dump, g2_rows


def kin_steps(traj, body):
    lim = _limits(body)
    rows, t = np.asarray(traj["poses"], float), np.asarray(traj["time_s"], float)
    el, dif = np.diff(t), np.diff(rows, axis=0)
    sp = np.linalg.norm(dif[:, :3], axis=1) / el
    yr = np.abs(dif[:, 3]) / el
    bad = np.flatnonzero((sp > lim["max_speed_mps"] + 1e-9) | (yr > lim["max_yaw_rate_radps"] + 1e-9))
    return [{"step": int(j), "steps": int(len(el)), "t_s": float(t[j]), "elapsed_s": float(el[j]),
             "translation_m": float(np.linalg.norm(dif[j, :3])), "yaw_change_rad": float(dif[j, 3]),
             "speed_excess": float(sp[j] - lim["max_speed_mps"]), "yaw_rate_excess": float(yr[j] - lim["max_yaw_rate_radps"]),
             "elapsed_exact_s": float(abs(dif[j, 3]) / lim["max_yaw_rate_radps"]) if np.linalg.norm(dif[j, :3]) == 0 else
             float(np.linalg.norm(dif[j, :3]) / lim["max_speed_mps"])} for j in bad]


def snapped_trajectory(path, body, start_yaw, end_yaw, min_turn=0., min_turn_s=1e-3):
    """``planner._linear_trajectory`` with every in-place turn given at least ``min_turn_s`` seconds
    (so its yaw rate stays below the limit after time rounding); ``min_turn`` > 0 instead drops tiny
    turns (yaw kept), which the replay's heading-match check rejects."""
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
        heading = math.atan2(delta[1], delta[0])
        turn = angle_delta(heading, yaw)
        if abs(turn) >= min_turn:
            yaw += turn
            append(Pose3(prev.xyz, yaw), max(abs(turn) / limits["max_yaw_rate_radps"], min_turn_s))
        append(Pose3(q.xyz, yaw), length / limits["max_speed_mps"])
    turn = angle_delta(end_yaw, poses[-1].yaw)
    if abs(turn) >= min_turn:
        append(Pose3(poses[-1].xyz, poses[-1].yaw + turn), max(abs(turn) / limits["max_yaw_rate_radps"], min_turn_s))
    from gmc.gs3d.planner import _row
    return {"poses": [_row(q) for q in poses], "time_s": times, "interpolation": "linear_xyz_yaw",
            "segments": [], "control_dt_s": .05}


def cmd_kinematics(a):
    compiled = load_compiled(a.a3c)
    body, z_c = compiled.body, compiled.domain.ground_z
    rows = [r for r in g2_rows(a.robot).values() if r["reason"] == "shared_replay_failed"]
    out = []
    for r in sorted(rows, key=lambda r: r["index"]):
        s = compiled.frame.to_world([*r["start_uv"], z_c])
        g = compiled.frame.to_world([*r["goal_uv"], z_c])
        q = query(compiled, s, g, config=QCONFIG, call_id=r["pair_id"])
        poly = np.asarray(q["polyline_world"], float)
        dens = a3api._densify(poly, QCONFIG.export_max_segment_m)
        gs = a3api._gs3d_result(compiled, dens, g, q["verification"]["own"]["clearance_lower_m"])
        bad = kin_steps(gs["trajectory"], body)
        # which knots carry the tiny turns: heading changes at densify knots vs at real vertices
        heads = np.arctan2(np.diff(dens[:, 1]), np.diff(dens[:, 0]))
        dturn = np.abs([angle_delta(b, a_) for a_, b in zip(heads[:-1], heads[1:])])
        tiny = int(np.count_nonzero((dturn > 1e-12) & (dturn < 1e-6)))
        fixed = dict(gs)
        fixed["trajectory"] = snapped_trajectory([Pose3(tuple(map(float, p))) for p in dens], body, 0., 0.)
        fixed["attained_goal"] = fixed["trajectory"]["poses"][-1]
        rep = replay_plan(fixed, GaussianBodyOracle(compiled.prepared))
        rec = {"index": r["index"], "pair_id": r["pair_id"], "status_now": q["status"], "reason_now": q["reason"],
               "densified_knots": int(len(dens)), "polyline_vertices": int(len(poly)),
               "turns_between_1e-12_and_1e-6_rad": tiny, "max_tiny_turn_rad": float(dturn[(dturn > 1e-12) & (dturn < 1e-6)].max()) if tiny else None,
               "violations": bad[:5], "n_violations": len(bad),
               "snapped_replay": {"passed": rep["passed"], "geometry_passed": rep["geometry"].get("passed"),
                                  "geometry_reason": rep["geometry"].get("reason"),
                                  "clearance_lower_m": rep["geometry"].get("clearance_lower_m"),
                                  "kinematics": rep["kinematics"], "attained_matches": rep["attained_matches"]}}
        out.append(rec)
        print(r["pair_id"], q["reason"], "violations", len(bad), bad[0] if bad else None, "tiny turns", tiny,
              "snapped passed", rep["passed"], flush=True)
    _dump(a.out, {"robot": a.robot, "n": len(out), "rows": out,
                  "snapped_passed": sum(x["snapped_replay"]["passed"] for x in out)})


def cmd_endpoints(a):
    compiled = load_compiled(a.a3c)
    z_c = compiled.domain.ground_z
    real = a3api.grow_cell

    def grow(table, domain, centre, half, *, buffer_m, kind="support_plane", **kw):
        if kind == "query":
            buffer_m = a.buffer
        return real(table, domain, centre, half, buffer_m=buffer_m, kind=kind, **kw)
    a3api.grow_cell = grow
    rows = [r for r in g2_rows(a.robot).values() if r["reason"].endswith("_not_certified_free")]
    out, t0 = [], time.perf_counter()
    from aerial3dg_batch import _Timeout, _alarm
    for r in sorted(rows, key=lambda r: r["index"]):
        s = compiled.frame.to_world([*r["start_uv"], z_c])
        g = compiled.frame.to_world([*r["goal_uv"], z_c])
        w0 = time.perf_counter()
        try:
            with _alarm(a.timeout):
                q = query(compiled, s, g, config=QCONFIG, call_id=r["pair_id"])
            st, rs = q["status"], q["reason"]
            shared = ((q["verification"] or {}).get("shared") or {}).get("passed")
        except _Timeout:
            st, rs, shared = "TIMEOUT", "timeout", None
        out.append({"index": r["index"], "pair_id": r["pair_id"], "g2_reason": r["reason"], "status": st, "reason": rs,
                    "shared_replay_passed": shared, "wall_s": time.perf_counter() - w0})
        print(r["pair_id"], r["reason"], "->", st, rs, round(out[-1]["wall_s"], 2), flush=True)
    counts = {}
    for x in out:
        counts[f"{x['status']}:{x['reason']}"] = counts.get(f"{x['status']}:{x['reason']}", 0) + 1
    _dump(a.out, {"robot": a.robot, "query_cell_buffer_m": a.buffer, "n": len(out), "counts": counts, "rows": out,
                  "wall_s": time.perf_counter() - t0})
    print(json.dumps(counts), flush=True)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("cmd", choices=["kinematics", "endpoints"])
    p.add_argument("--robot", required=True)
    p.add_argument("--a3c", type=Path, default=None)
    p.add_argument("--buffer", type=float, default=0.)
    p.add_argument("--timeout", type=float, default=120.)
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)
    a.a3c = a.a3c or A3C / f"{a.robot}.a3c"
    {"kinematics": cmd_kinematics, "endpoints": cmd_endpoints}[a.cmd](a)


if __name__ == "__main__":
    main()
