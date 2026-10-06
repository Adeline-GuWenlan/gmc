"""F3 Task 1.2-1.4: probe and classify GMC failures on confirmed-reachable pairs (sbatch for ``bufzero`` / ``requery``).

Run from ``gmc/`` with ``PYTHONPATH=src:experiments``.

  bufzero   every ``*_not_certified_free`` row of a task JSONL: re-query on the same compile with the endpoint
            query cell grown at buffer 0 (F1's ``fixprobe endpoints``, generalised to any compile / rows)
  requery   reproduce chosen rows on the same compile (full G2 QCONFIG) and dump the certificate: for UNREACHABLE
            the cut pair ids with their Gaussians' route-frame centres / 2-sigma tops, and whether the A* witness
            route of the pair passes within 0.05 m (xy) of any of them -- the soundness trace F4/F5 continue.
  classify  (inline) merge GMC rows + A* evidence + probe outputs into one class per non-REACHABLE row:
              EXPORT-KIN        shared_replay_failed, cleared by the 1 ms turn + translation floor (aerial3dg_fail2_kin)
              EXPORT-DOMAIN     shared_replay_failed, replay geometry map_unknown only (swept-AABB coverage, F2 4.2)
              REPLAY-OTHER      shared_replay_failed, anything else                                    -> genuine
              EP-TOL            *_not_certified_free and the endpoint's oracle clearance <= margin + buffer (2 mm)
              EP-GENUINE        *_not_certified_free with endpoint clearance > 2 mm                    -> genuine
              GAP-TOL           safe_graph_disconnected and the A* witness's 3-D clearance <= 2 mm (ladder)
              GAP-GENUINE       safe_graph_disconnected with witness clearance >= 3 mm                 -> genuine
              SOUNDNESS         UNREACHABLE on a confirmed-reachable pair                              -> genuine (!)
              METHOD-ERROR      TIMEOUT / ERROR / any other UNKNOWN reason                             -> genuine
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np

from aerial3dg_run import QCONFIG, _dump
from aerial3dg_fail2_sample import read_jsonl

MARGIN, BUFFER = .001, .001
TOL = MARGIN + BUFFER + 1e-6


def _rows(path):
    return {r["index"]: r for r in read_jsonl(Path(path))}


def cmd_bufzero(a):
    from gmc.aerial3d import api as a3api
    from gmc.aerial3d.api import load_compiled, query
    from aerial3dg_batch import _Timeout, _alarm
    compiled = load_compiled(a.a3c)
    z_c = compiled.domain.ground_z
    real = a3api.grow_cell

    def grow(table, domain, centre, half, *, buffer_m, kind="support_plane", **kw):
        if kind == "query":
            buffer_m = a.buffer
        return real(table, domain, centre, half, buffer_m=buffer_m, kind=kind, **kw)
    a3api.grow_cell = grow
    rows = [r for r in _rows(a.rows).values() if r["reason"].endswith("_not_certified_free")]
    out = []
    for r in sorted(rows, key=lambda r: r["index"]):
        s = compiled.frame.to_world([*r["start_uv"], z_c])
        g = compiled.frame.to_world([*r["goal_uv"], z_c])
        w0 = time.perf_counter()
        try:
            with _alarm(a.timeout):
                q = query(compiled, s, g, config=QCONFIG, call_id=r["pair_id"])
            st, rs = q["status"], q["reason"]
        except _Timeout:
            st, rs = "TIMEOUT", "timeout"
        out.append({"index": r["index"], "pair_id": r["pair_id"], "orig": f"{r['status']}:{r['reason']}",
                    "buffer0": f"{st}:{rs}", "wall_s": time.perf_counter() - w0})
        print(out[-1], flush=True)
    _dump(a.out, {"a3c": str(a.a3c), "rows_source": str(a.rows), "query_cell_buffer_m": a.buffer, "rows": out})


def cmd_requery(a):
    from gmc.aerial3d.api import load_compiled, query
    compiled = load_compiled(a.a3c)
    z_c = compiled.domain.ground_z
    pairs = {p["pair_id"]: p for p in json.loads(Path(a.pairs).read_text())["pairs"]}
    src = _rows(a.rows)
    want = [r for r in src.values() if (a.ids and r["pair_id"] in a.ids) or (not a.ids and r["status"] != "REACHABLE")]
    cp = compiled.pairs
    pos = {int(i): k for k, i in enumerate(cp.ids)}
    lev = float(cp.level)
    out = []
    for r in sorted(want, key=lambda r: r["index"]):
        s = compiled.frame.to_world([*r["start_uv"], z_c])
        g = compiled.frame.to_world([*r["goal_uv"], z_c])
        q = query(compiled, s, g, config=QCONFIG, call_id=r["pair_id"])
        rec = {"index": r["index"], "pair_id": r["pair_id"], "orig": f"{r['status']}:{r['reason']}",
               "now": f"{q['status']}:{q['reason']}",
               "reproduced": (q["status"], q["reason"]) == (r["status"], r["reason"]),
               "compile_id": compiled.compile_id}
        cert = q.get("certificate") or {}
        rec["certificate_kind"] = cert.get("kind")
        if q.get("polyline_world") is not None:
            rec["gmc_route_uv"] = compiled.frame.to_plan(np.asarray(q["polyline_world"], float))[:, :2].round(4).tolist()
        ids = [int(x) for x in cert.get("cut_pair_ids") or []]
        p = pairs.get(r["pair_id"])
        if ids:
            k = np.array([pos[i] for i in ids if i in pos])
            mu = compiled.frame.to_plan(cp.means[k])
            top = mu[:, 2] + lev * np.sqrt(np.maximum(cp.covs[k, 2, 2], 0.))
            rec["cut_gaussians"] = [[i, *np.round(m, 4).tolist(), round(float(t), 5)] for i, m, t in zip(ids, mu, top)]
            leaves = np.asarray(cert.get("cut_leaf_centres_plan") or np.zeros((0, 3)), float)
            rec["cut_leaves_sampled"] = len(leaves)
            if p is not None and p.get("astar_route_uv"):
                R = np.asarray(p["astar_route_uv"], float)
                dense = np.concatenate([np.linspace(a_, b_, max(2, int(np.ceil(np.linalg.norm(b_ - a_) / .01)) + 1))
                                        for a_, b_ in zip(R[:-1], R[1:])]) if len(R) > 1 else R
                if len(leaves):
                    d = np.min(np.linalg.norm(dense[:, None, :] - leaves[None, :, :2], axis=2), axis=0)
                    rec["astar_route_min_xy_dist_to_cut_leaf_centre_m"] = float(d.min())
                    rec["cut_leaf_centres_within_5cm_of_astar_route"] = int((d < .05).sum())
                d = np.min(np.linalg.norm(dense[:, None, :] - mu[None, :, :2], axis=2), axis=0)
                rec["astar_route_min_xy_dist_to_cut_gaussian_m"] = float(d.min())
            rec["start_component_bbox_plan"] = cert.get("start_component_bbox_plan")
        out.append(rec)
        print(rec["pair_id"], rec["orig"], "->", rec["now"], rec["reproduced"],
              rec.get("astar_route_min_xy_dist_to_cut_leaf_centre_m"), flush=True)
    _dump(a.out, {"a3c": str(a.a3c), "rows_source": str(a.rows), "rows": out})


def classify_row(r, ev, kin=None, buf0=None):
    st, rs = r["status"], r["reason"]
    if st == "REACHABLE":
        return "REACHABLE"
    if st == "UNREACHABLE":
        return "SOUNDNESS"
    if st == "UNKNOWN" and rs == "shared_replay_failed":
        if kin is None:
            return "REPLAY-UNPROBED"
        if kin.get("both_floors", {}).get("passed"):
            return "EXPORT-KIN"
        o = kin.get("original") or {}
        if o.get("geometry_passed") is False and o.get("geometry_reason") == "map_unknown":
            return "EXPORT-DOMAIN"
        return "REPLAY-OTHER"
    if st == "UNKNOWN" and rs.endswith("_not_certified_free"):
        end = "start" if rs.startswith("start") else "goal"
        c = ev.get(f"{end}_clear3d_m")
        if c is None:
            c = (ev.get("clearance_m") or {}).get(end)
        return "EP-TOL" if c is not None and c <= TOL else "EP-GENUINE"
    if st == "UNKNOWN" and rs.startswith("safe_graph_disconnected"):
        c = ev.get("clear3d_m")
        return "GAP-TOL" if c is not None and c <= TOL else "GAP-GENUINE"
    return "METHOD-ERROR"


GENUINE = {"SOUNDNESS", "REPLAY-OTHER", "EP-GENUINE", "GAP-GENUINE", "METHOD-ERROR"}
TOLERANCE = {"EP-TOL", "GAP-TOL"}
EXPORT = {"EXPORT-KIN", "EXPORT-DOMAIN"}


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("bufzero")
    b.add_argument("--a3c", type=Path, required=True)
    b.add_argument("--rows", type=Path, required=True)
    b.add_argument("--buffer", type=float, default=0.)
    b.add_argument("--timeout", type=float, default=120.)
    b.add_argument("--out", type=Path, required=True)
    q = sub.add_parser("requery")
    q.add_argument("--a3c", type=Path, required=True)
    q.add_argument("--rows", type=Path, required=True)
    q.add_argument("--pairs", type=Path, required=True)
    q.add_argument("--ids", nargs="*", default=None)
    q.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)
    {"bufzero": cmd_bufzero, "requery": cmd_requery}[a.cmd](a)


if __name__ == "__main__":
    main()
