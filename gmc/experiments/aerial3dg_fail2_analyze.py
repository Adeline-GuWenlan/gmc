"""F2 Task 1d: GMC (aerial3d) on the confirmed-reachable pair list -> headline false-negative table.

Run from ``gmc/`` with ``PYTHONPATH=src:experiments`` (light: reads JSON only).

  summary   per robot: status / reason counts over the confirmed pairs, the false-negative rate
            (UNKNOWN + UNREACHABLE + TIMEOUT/ERROR, out of N), distance-tercile breakdown, timing, and the list of
            every non-REACHABLE pair (pair id, reason, certificate kind, cut pairs) -> results/aerial3dg/f2/summary.json
  plot      map of the region: confirmed pairs' A* routes (thin), non-REACHABLE pairs highlighted, the robust lattice
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from aerial3dg_batch import read_checkpoint
from aerial3dg_run import _dump

F2 = Path("results/aerial3dg/f2")


def load(robot, pairs_n):
    rows = {}
    for p in sorted((F2 / "gmc" / robot).glob("task_*.jsonl")):
        rows.update(read_checkpoint(p))
    missing = [i for i in range(pairs_n) if i not in rows]
    return rows, missing


def _stats(x):
    x = np.asarray([v for v in x if v is not None], float)
    if not len(x):
        return None
    return {"n": int(len(x)), "sum": float(x.sum()), "mean": float(x.mean()), "median": float(np.median(x)),
            "p95": float(np.percentile(x, 95)), "max": float(x.max())}


def cmd_summary(a):
    doc = json.loads(a.pairs.read_text())
    pairs = doc["pairs"]
    d = np.array([p["dist_m"] for p in pairs])
    edges = np.quantile(d, [0, 1 / 3, 2 / 3, 1])
    tercile = np.clip(np.searchsorted(edges, d, side="right") - 1, 0, 2)
    out = {"pairs_file": str(a.pairs), "n_pairs": len(pairs), "distance_tercile_edges_m": edges.tolist(), "robots": {}}
    for robot in a.robots:
        rows, missing = load(robot, len(pairs))
        st, rs, by_t = {}, {}, {}
        fails = []
        for i, r in rows.items():
            st[r["status"]] = st.get(r["status"], 0) + 1
            rs.setdefault(r["status"], {}).setdefault(r["reason"], 0)
            rs[r["status"]][r["reason"]] += 1
            t = int(tercile[i])
            by_t.setdefault(t, {}).setdefault(r["status"], 0)
            by_t[t][r["status"]] += 1
            if r["status"] != "REACHABLE":
                p = pairs[i]
                fails.append({"index": i, "pair_id": r["pair_id"], "status": r["status"], "reason": r["reason"],
                              "dist_m": r["dist_m"], "start_uv": r["start_uv"], "goal_uv": r["goal_uv"],
                              "certificate_kind": r.get("certificate_kind"),
                              "cut_distinct_pairs": r.get("cut_distinct_pairs"),
                              "endpoint_certificate_pair": r.get("endpoint_certificate_pair"),
                              "own_verification": r.get("own_verification"),
                              "shared_replay_passed": r.get("shared_replay_passed"),
                              "astar_path_length_m": p["astar"]["path_length_m"],
                              "endpoint_clearance_m001": p["clearance_m"]["cylinder"]})
        n = len(rows)
        reach = st.get("REACHABLE", 0)
        ok = [r for r in rows.values() if r["status"] == "REACHABLE"]
        ratio = [r["path_length_m"] / pairs[r["index"]]["astar"]["path_length_m"] for r in ok
                 if r.get("path_length_m") and pairs[r["index"]]["astar"]["path_length_m"]]
        out["robots"][robot] = {
            "answered": n, "missing": len(missing), "status_counts": st, "reasons": rs,
            "false_negatives": n - reach, "false_negative_rate": (n - reach) / max(n, 1),
            "unreachable_count": st.get("UNREACHABLE", 0),
            "by_distance_tercile": {str(k): v for k, v in sorted(by_t.items())},
            "query_wall_s": _stats([r["algorithm_wall_s"] for r in rows.values()]),
            "query_wall_s_reachable": _stats([r["algorithm_wall_s"] for r in ok]),
            "gmc_len_over_astar_len": _stats(ratio),
            "clearance_lower_m_reachable": _stats([r.get("clearance_lower_m") for r in ok]),
            "compile_ids": sorted({r["compile_id"] for r in rows.values()}),
            "non_reachable": sorted(fails, key=lambda x: x["index"])}
        print(robot, json.dumps({k: out["robots"][robot][k] for k in
                                 ("answered", "missing", "status_counts", "reasons", "false_negative_rate")}), flush=True)
    _dump(a.out, out)


def cmd_plot(a):
    import matplotlib.pyplot as plt
    doc = json.loads(a.pairs.read_text())
    summ = json.loads(a.summary.read_text())
    lat = np.load(F2 / "sample/nw/lattice.npz")
    fig, axes = plt.subplots(1, len(summ["robots"]), figsize=(6.5 * len(summ["robots"]), 6), squeeze=False)
    for ax, (robot, s) in zip(axes[0], summ["robots"].items()):
        us, vs, comp = lat["us"], lat["vs"], lat["comp"]
        ax.imshow((comp >= 0).T, origin="lower", extent=[us[0] - .05, us[-1] + .05, vs[0] - .05, vs[-1] + .05],
                  cmap="Greys_r", alpha=.35, vmin=0, vmax=1)
        for p in doc["pairs"][::max(1, len(doc["pairs"]) // 400)]:
            r = np.asarray(p["astar_route_uv"])
            ax.plot(r[:, 0], r[:, 1], "-", color="tab:blue", lw=.3, alpha=.4)
        colors = {"UNKNOWN": "tab:orange", "UNREACHABLE": "tab:red", "TIMEOUT": "tab:purple", "ERROR": "k"}
        for f in s["non_reachable"]:
            c = colors.get(f["status"], "k")
            ax.plot(*zip(f["start_uv"], f["goal_uv"]), "-", color=c, lw=.8)
            ax.plot(*f["start_uv"], "o", color=c, ms=3)
            ax.plot(*f["goal_uv"], "s", color=c, ms=3)
        bu = doc["box_uv"]
        ax.add_patch(plt.Rectangle(bu[:2], bu[2] - bu[0], bu[3] - bu[1], fill=False, ec="k", lw=1))
        ax.set_aspect("equal")
        ax.set_title(f"{robot}: {s['status_counts']}\n(blue: every {max(1, len(doc['pairs']) // 400)}th A* route; "
                     f"grey: robust-body lattice nodes; orange/red: GMC non-REACHABLE)", fontsize=8)
        ax.set_xlabel("u (m)")
        ax.set_ylabel("v (m)")
    fig.tight_layout()
    fig.savefig(a.out, dpi=130)
    print("wrote", a.out)


def main(argv=None):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("summary")
    s.add_argument("--pairs", type=Path, default=F2 / "pairs_confirmed_5000.json")
    s.add_argument("--robots", nargs="+", default=["cylinder", "sweeper"])
    s.add_argument("--out", type=Path, default=F2 / "summary.json")
    pl = sub.add_parser("plot")
    pl.add_argument("--pairs", type=Path, default=F2 / "pairs_confirmed_5000.json")
    pl.add_argument("--summary", type=Path, default=F2 / "summary.json")
    pl.add_argument("--out", type=Path, default=F2 / "f2_map.png")
    a = p.parse_args(argv)
    {"summary": cmd_summary, "plot": cmd_plot}[a.cmd](a)


if __name__ == "__main__":
    main()
