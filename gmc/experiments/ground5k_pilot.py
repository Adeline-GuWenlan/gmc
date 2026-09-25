"""Ground-5k G1 Task 2: summarise the pilot runs and project the full run's cost.

Reads the per-query JSONs of pairs [0, n) for each combination and, for the arrays named on the
command line, Slurm's accounting (``sacct``: elapsed x allocated CPUs, MaxRSS).  Writes
``results/ground5k/pilot/summary.json`` and a markdown table for docs/ground5k_design.md §2.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess

import numpy as np

import ground5k_common as gc

OUT = Path("outputs/ground5k")
RES = Path("results/ground5k/pilot")


def _pct(v, q):
    return float(np.percentile(v, q)) if len(v) else None


def _sacct(job):
    rows = subprocess.run(["sacct", "-j", str(job), "-P", "-n", "--format=JobID,State,ElapsedRaw,AllocCPUS,MaxRSS"],
                          capture_output=True, text=True).stdout.splitlines()
    tasks, steps = {}, {}
    for r in rows:
        jid, state, el, cpus, rss = r.split("|")
        base = jid.split(".")[0]
        if "." not in jid:
            tasks[base] = {"state": state, "elapsed_s": int(el or 0), "cpus": int(cpus or 0)}
        elif rss:
            v = rss.rstrip("KMG")
            mult = {"K": 1 << 10, "M": 1 << 20, "G": 1 << 30}.get(rss[-1], 1)
            steps[base] = max(steps.get(base, 0), float(v) * mult)
    for k, t in tasks.items():
        t["max_rss_bytes"] = steps.get(k)
    return tasks


def summarise(scene, n, arrays):
    runs = OUT / scene / "runs"
    out = {"scene": scene, "n_pairs": n, "combos": {}}
    for combo in gc.COMBOS:
        recs = []
        for k in range(n):
            p = runs / combo / f"{gc.load_config()['scenes'][scene]['pair_prefix']}-{k:05d}.json"
            if p.exists():
                recs.append(json.loads(p.read_text()))
        oc = {o: sum(r["outcome"] == o for r in recs) for o in gc.OUTCOMES}
        causes = {}
        for r in recs:
            causes[r["cause"]] = causes.get(r["cause"], 0) + 1
        wall = np.array([r["query_wall_s"] for r in recs])
        rss = np.array([r["peak_rss_bytes"] for r in recs]) / 2 ** 30
        sup = [r.get("crop", {}).get("selected_supports") for r in recs if r.get("crop")]
        nsup = [r.get("stats", {}).get("n_supports") for r in recs if r.get("stats", {}).get("n_supports") is not None]
        row = {"n_done": len(recs), "outcomes": oc, "causes": causes,
               "query_wall_s": {"mean": float(wall.mean()) if len(wall) else None, "median": _pct(wall, 50),
                                "p90": _pct(wall, 90), "max": float(wall.max()) if len(wall) else None},
               "peak_rss_gb": {"median": _pct(rss, 50), "p90": _pct(rss, 90), "max": float(rss.max()) if len(rss) else None},
               "crop_selected_supports_median": _pct(sup, 50), "gmc_projected_supports_median": _pct(nsup, 50),
               "gmc_projected_supports_max": max(nsup) if nsup else None,
               "replay_failures": sum(r["outcome"] == "FAIL_REPLAY" for r in recs)}
        if combo in arrays:
            acct = _sacct(arrays[combo])
            cpu_s = sum(t["elapsed_s"] * t["cpus"] for t in acct.values())
            row["slurm"] = {"array": arrays[combo], "tasks": len(acct),
                            "states": {s: sum(t["state"] == s for t in acct.values()) for s in {t["state"] for t in acct.values()}},
                            "cpu_h": cpu_s / 3600, "cpu_h_per_pair": cpu_s / 3600 / max(len(recs), 1),
                            "max_rss_gb": max((t["max_rss_bytes"] or 0) for t in acct.values()) / 2 ** 30,
                            "task_overhead_s_mean": float(np.mean([t["elapsed_s"] for t in acct.values()])) - float(wall.mean() if len(wall) else 0)}
        out["combos"][combo] = row
    return out


def table(summary):
    lines = ["| combination | done | SUCCESS | NO_PATH | UNKNOWN | BUDGET | REPLAY | ERROR | query wall median / p90 / max (s) | peak RSS p90 / max (GB) | CPU·h (pilot) | CPU·h / pair |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for combo, r in summary["combos"].items():
        o = r["outcomes"]
        w = r["query_wall_s"]
        s = r.get("slurm", {})
        f = lambda v, d=0: "—" if v is None else f"{v:,.{d}f}"
        lines.append(f"| {combo} | {r['n_done']} | {o['SUCCESS_VERIFIED']} | {o['FAIL_NO_PATH']} | {o['FAIL_UNKNOWN']} | "
                     f"{o['FAIL_BUDGET']} | {o['FAIL_REPLAY']} | {o['ERROR']} | {f(w['median'])} / {f(w['p90'])} / {f(w['max'])} | "
                     f"{f(r['peak_rss_gb']['p90'], 2)} / {f(r['peak_rss_gb']['max'], 2)} | {f(s.get('cpu_h'), 2)} | {f(s.get('cpu_h_per_pair'), 3)} |")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="scene_v2")
    ap.add_argument("--n", type=int, default=50)
    ap.add_argument("--array", action="append", default=[], help="combo=arrayjobid")
    ap.add_argument("--project", type=int, nargs="*", default=[], help="prefix sizes to project")
    ap.add_argument("--jobids", default="/scratch/wg2381/claude_jobs/ground5k/jobids/G1.txt")
    args = ap.parse_args(argv)
    arrays = dict(a.split("=") for a in args.array)
    s = summarise(args.scene, args.n, arrays)
    if args.project:
        used = chain_cpu_h(args.jobids)
        s["cpu_h_used_so_far"] = used
        s["projection"] = {str(n): project(s, n, used["agent"] + used["compute"]) for n in args.project}
    RES.mkdir(parents=True, exist_ok=True)
    gc.write_json_atomic(RES / f"summary_{args.scene}.json", s)
    (RES / f"table_{args.scene}.md").write_text(table(s) + "\n")
    print(table(s))
    print(json.dumps({c: r["causes"] for c, r in s["combos"].items()}, indent=1))



# ------------------------------------------------------------------- projection --

LAYOUT = {"astar_sweeper": (250, 6), "astar_cylinder": (125, 6), "gmc_sweeper": (25, 8), "gmc_cylinder": (9, 12)}
QOS = {"cpus": 32, "mem_gb": 120, "reserved_gb": 12}     # agent jobs of this and the other chain


def chain_cpu_h(jobids_file):
    """CPU·h already used by every job listed for the stage (agents included)."""
    ids = [l.split()[0] for l in Path(jobids_file).read_text().splitlines() if l.strip()]
    rows = subprocess.run(["sacct", "-j", ",".join(ids), "-X", "-n", "-P", "--format=JobID,JobName,CPUTimeRAW"],
                          capture_output=True, text=True).stdout.splitlines()
    tot = {"agent": 0.0, "compute": 0.0}
    for r in rows:
        _, name, cpu = r.split("|")
        tot["agent" if name.startswith("agent") else "compute"] += int(cpu or 0) / 3600
    return tot


def project(summary, n_pairs, used_cpu_h, budget=8000.0, g2_reserve=300.0, load_s=15.0):
    rows, per_pair_total, gbh = {}, 0.0, 0.0
    for combo, r in summary["combos"].items():
        per_task, mem = LAYOUT[combo]
        wall = (r["query_wall_s"]["mean"] or 0.0) + load_s / per_task
        sac = r.get("slurm", {}).get("cpu_h_per_pair")
        per_pair = max(wall / 3600, sac or 0.0)
        rows[combo] = {"cpu_h_per_pair": per_pair, "from_query_wall_h": wall / 3600, "from_sacct_h": sac,
                       "cpu_h": per_pair * n_pairs, "task_mem_gb": mem, "gb_h": per_pair * n_pairs * mem,
                       "pairs_per_task": per_task, "tasks": -(-n_pairs // per_task)}
        per_pair_total += per_pair
        gbh += per_pair * n_pairs * mem
    avail = budget - used_cpu_h - g2_reserve
    cpu_h = per_pair_total * n_pairs
    wall_mem_h = gbh / (QOS["mem_gb"] - QOS["reserved_gb"])
    wall_cpu_h = cpu_h / QOS["cpus"]
    return {"n_pairs": n_pairs, "combos": rows, "cpu_h_per_pair_all_four": per_pair_total,
            "cpu_h": cpu_h, "budget_cpu_h": budget, "used_cpu_h": used_cpu_h, "g2_reserve_cpu_h": g2_reserve,
            "available_cpu_h": avail, "max_prefix_by_budget": int(min(5000, avail // per_pair_total)),
            "wall_clock_h_memory_bound": wall_mem_h, "wall_clock_h_cpu_bound": wall_cpu_h,
            "wall_clock_days_estimate": max(wall_mem_h, wall_cpu_h) / 24, "qos": QOS}


if __name__ == "__main__":
    main()
