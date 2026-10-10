"""bl B3: the full runs of the four frozen baselines on F4's 5000 confirmed pairs x 2 robots (plan §6), driven
through ``bl_harness`` unchanged. Run from ``gmc/`` under gmc-venv with ``PYTHONPATH=src:experiments``.

Subcommands:
  plan       (inline) the B3 layout, written to ``results/baselines/b3/plan.json`` BEFORE any full-run row exists:
             per method x robot x region the frozen config + its SHA, the persisted setup artifact that every task
             loads (path, SHA-256 = sidecar, setup id, the record of the job that built it), per-task cost estimates
             from the frozen-config pilot rows, the task split (``run --task t --n-tasks n``), the packing of GPU tasks
             into 4 harness streams per L40S job (longest first), the CPU array map, and plan §6's budget decision.
  streams    (sbatch, GPU job) one shell line per stream: the stream's ``bl_harness run`` commands, in order
  task       (sbatch, CPU array) the ``bl_harness run`` arguments of array task ``--index``
  collect    (sbatch) ``results/baselines/collect.json``: completeness, setup-once proof, outcome counts, sacct cost
  spotpick   (inline) the spot-check sample: 5 SUCCESS + 5 CLAIMED_* rows per method (seeded), committed first
  spotcheck  (sbatch) re-judge the picked rows' stored exact paths in a fresh gmc-venv process and compare verdicts
"""
from __future__ import annotations

import argparse
import collections
import json
import math
from pathlib import Path
import subprocess

import numpy as np

import bl_harness as H

B3 = H.RES / "b3"
METHODS = ("foci", "splatnav", "pno", "cust_fields")          # queue order: cheap first (pilot projection)
GPU_METHODS = ("foci", "splatnav", "pno")
STREAMS_PER_GPU = 4
GPU_TASK_TARGET_H = 1.4        # split a region x robot into tasks of <= this many estimated stream-hours
CPU_TASK_TARGET_H = 0.45
N_REGION = {"WWEST": 2500, "GAPW1": 1500, "S": 1000}


def config_file(method):
    return H.CONFIGS / f"{method}.json"


def pilot_estimates(method):
    """Per (region, robot): mean seconds per query (outer + judge: the judge runs inline in B3) and the mean worker
    instantiate time, from the frozen-config pilot (results/baselines/pilot/<method>)."""
    root = H.RES / "pilot" / method
    by = collections.defaultdict(list)
    for r in H.iter_rows(root):
        by[(r["region"], r["robot"])].append(r)
    inst = {}
    for f in root.rglob("task_*.summary.json"):
        s = json.loads(f.read_text())
        v = [t for t in s["setup_once_proof"]["instantiate_s_per_start"] if t]
        inst[(s["region"], s["robot"])] = float(np.mean(v)) if v else 0.
    out = {}
    for k, v in by.items():
        q = float(np.mean([(r.get("outer_wall_s") or 0.) + (r.get("judge_wall_s") or 0.) for r in v]))
        out[k] = {"pilot_rows": len(v), "mean_query_s_incl_judge": q, "instantiate_s": inst.get(k, 0.),
                  "pilot_status_counts": dict(collections.Counter(r["status"] for r in v))}
    return out


def setup_record(method, region, robot):
    cfg = H.load_config(method, robot, config_file(method))
    art = H.artifact_path(method, region, robot, cfg)
    side = json.loads(Path(f"{art}.json").read_text())
    sha = H.sha256_file(art)
    rec_path = H.RES / method / "setup" / art.parent.name / f"{region}_{robot}.json"
    rec = json.loads(rec_path.read_text()) if rec_path.exists() else {}
    return {"artifact": str(art), "artifact_sha256": sha, "sidecar_sha256": side["sha256"],
            "sha_matches_sidecar": sha == side["sha256"], "setup_id": side["setup_id"],
            "build_setup_wall_s": side.get("build_setup_wall_s"), "built_by_record": str(rec_path),
            "built_on_host": (rec.get("host") or {}), "config_sha256": H.sha_json(cfg)}


def cmd_plan(a):
    doc = {"schema": "bl.b3_plan.v1",
           "definition": __doc__.strip(),
           "pairs": str(H.F4 / "pairs_confirmed_5000.json"), "pairs_per_region": N_REGION,
           "subset": None,
           "judge": "inline in every harness stream (the pilots' validated path; deferred judging is not used)",
           "timeout_s": H.QUERY_TIMEOUT_S, "methods": {}}
    gpu_h = cpu_h = 0.
    for m in METHODS:
        cf = config_file(m)
        cdoc = json.loads(cf.read_text())
        est = pilot_estimates(m)
        md = {"config_file": str(cf), "config_file_sha256": H.sha256_file(cf),
              "frozen_candidate": cdoc.get("candidate"), "adapter_sha256_now": H.adapter_sha(m),
              "adapter_sha256_frozen": cdoc.get("adapter_sha256"),
              "resource": "gpu" if m in GPU_METHODS else "cpu", "region_robot": {}, "tasks": []}
        md["adapter_unchanged_since_freeze"] = md["adapter_sha256_now"] == md["adapter_sha256_frozen"]
        target = GPU_TASK_TARGET_H if m in GPU_METHODS else CPU_TASK_TARGET_H
        for robot in H.ROBOTS:
            for region in H.REGIONS:
                e = est[(region, robot)]
                tot_h = e["mean_query_s_incl_judge"] * N_REGION[region] / 3600
                n = max(1, math.ceil(tot_h / target))
                if m not in GPU_METHODS:
                    n = max(1, min(n, math.ceil(N_REGION[region] / 100)))
                md["region_robot"][f"{region}/{robot}"] = {"setup": setup_record(m, region, robot), "estimate": e,
                                                           "estimated_stream_h": tot_h, "n_tasks": n}
                for t in range(n):
                    md["tasks"].append({"region": region, "robot": robot, "task": t, "n_tasks": n,
                                        "est_h": tot_h / n + e["instantiate_s"] / 3600})
        md["tasks"].sort(key=lambda t: -t["est_h"])
        if m in GPU_METHODS:
            streams = [[] for _ in range(STREAMS_PER_GPU)]
            load = [0.] * STREAMS_PER_GPU
            for t in md["tasks"]:                       # longest processing time first
                i = int(np.argmin(load))
                streams[i].append(t)
                load[i] += t["est_h"]
            md["streams"] = streams
            md["est_stream_h"] = load
            md["est_job_wall_h"] = max(load)
            md["est_gpu_h"] = max(load)
            md["est_cpu_h"] = max(load) * STREAMS_PER_GPU
            gpu_h += md["est_gpu_h"]
        else:
            md["array"] = list(range(len(md["tasks"])))
            md["est_cpu_h"] = sum(t["est_h"] for t in md["tasks"])
            md["est_max_task_h"] = max(t["est_h"] for t in md["tasks"])
        cpu_h += md["est_cpu_h"]
        md["over_cpu_share"] = md["est_cpu_h"] > 600 / 4
        md["decision"] = "full 5000 x 2 (no subset: under the per-method share, plan §6)"
        doc["methods"][m] = md
    doc["estimate_total"] = {"gpu_h": gpu_h, "cpu_h": cpu_h,
                             "note": "pilot means x pair counts, judge inline, linear in streams [G]; "
                                     "jobs get ~2x these as --time"}
    doc["budget"] = {"cpu_h": 600, "gpu_h": 100, "per_method_cpu_share_h": 150}
    H._dump(B3 / "plan.json", doc)
    for m, md in doc["methods"].items():
        print(m, len(md["tasks"]), "tasks", {k: np.round(v, 3).tolist() for k, v in md.items() if k.startswith("est_")},
              "adapter unchanged", md["adapter_unchanged_since_freeze"],
              all(v["setup"]["sha_matches_sidecar"] for v in md["region_robot"].values()))


def _run_args(m, t):
    return (f"run --method {m} --region {t['region']} --robot {t['robot']} --pairs f4 "
            f"--config {config_file(m)} --task {t['task']} --n-tasks {t['n_tasks']} --out {H.RES / m}")


def cmd_streams(a):
    plan = json.loads((B3 / "plan.json").read_text())
    for s in plan["methods"][a.method]["streams"]:
        print("; ".join(f"$H {_run_args(a.method, t)}" for t in s))


def cmd_task(a):
    plan = json.loads((B3 / "plan.json").read_text())
    print(_run_args(a.method, plan["methods"][a.method]["tasks"][a.index]))


# ================================================================================================ collect
def _sacct(ids):
    """CPU-h (allocated cores x elapsed) and GPU-h per job id, from sacct (the job allocation lines only)."""
    if not ids:
        return {}
    out = subprocess.run(["sacct", "-n", "-P", "-X", "-j", ",".join(ids), "-o",
                          "JobID,JobName,State,ElapsedRaw,AllocCPUS,AllocTRES,MaxRSS,NodeList"],
                         capture_output=True, text=True).stdout
    rows = []
    for line in out.splitlines():
        f = line.split("|")
        if len(f) < 8:
            continue
        el, ncpu = int(f[3] or 0), int(f[4] or 0)
        gpus = 0
        for kv in f[5].split(","):
            if kv.startswith("gres/gpu="):
                gpus = int(kv.split("=")[1])
        rows.append({"jobid": f[0], "name": f[1], "state": f[2], "elapsed_s": el, "cpus": ncpu, "gpus": gpus,
                     "node": f[7], "cpu_h": el * ncpu / 3600, "gpu_h": el * gpus / 3600})
    return rows


def cmd_collect(a):
    pairs = {r: [p["pair_id"] for p in H.load_pairs("f4", r)] for r in H.REGIONS}
    plan = json.loads((B3 / "plan.json").read_text())
    doc = {"schema": "bl.b3_collect.v1", "plan": str(B3 / "plan.json"),
           "definition": "Completeness: every method x robot x F4 pair answered exactly once (subset: none). "
                         "Setup-once: per method x robot x region one setup id over all rows, every task "
                         "setup_builds_in_this_task = 0 and every worker start loaded the plan's artifact SHA. "
                         "Outcome classes plan §5. Cost: sacct allocated cores x elapsed (GPU jobs: 4 cores).",
           "methods": {}, "ok": True}
    for m in METHODS:
        md = {"robots": {}, "config_file_sha256_in_summaries": set(), "ok": True}
        for robot in H.ROBOTS:
            rd = {"regions": {}}
            for region in H.REGIONS:
                d = H.RES / m / region / robot
                rows = []
                for f in sorted(list(d.glob("task_*.jsonl")) + list(d.glob("task_*.jsonl.gz"))):
                    rows += _rows_of(f)
                sums = [json.loads(f.read_text()) for f in sorted(d.glob("task_*.summary.json"))]
                ids = [r["pair_id"] for r in rows]
                c = collections.Counter(ids)
                want = set(pairs[region])
                plan_rr = plan["methods"][m]["region_robot"][f"{region}/{robot}"]
                sids = sorted({r.get("setup_id") for r in rows}, key=str)
                proofs = [s["setup_once_proof"] for s in sums]
                art_shas = sorted({s["setup"]["artifact_sha256"] for s in sums})
                for s in sums:
                    md["config_file_sha256_in_summaries"].add(s.get("config_file_sha256"))
                stages_setup = sum(1 for r in rows if any("setup" in k for k in (r.get("stages") or {})))
                reg = {
                    "rows": len(rows), "expected": len(want),
                    "missing": sorted(want - set(c))[:20], "n_missing": len(want - set(c)),
                    "duplicated": sorted(k for k, v in c.items() if v > 1)[:20],
                    "n_duplicated": sum(1 for v in c.values() if v > 1),
                    "foreign": sorted(set(c) - want)[:20],
                    "tasks": len(sums), "tasks_expected": plan_rr["n_tasks"],
                    "tasks_complete": sum(bool(s.get("complete")) for s in sums),
                    "setup_ids": sids, "plan_setup_id": plan_rr["setup"]["setup_id"],
                    "artifact_sha256s": art_shas, "plan_artifact_sha256": plan_rr["setup"]["artifact_sha256"],
                    "setup_builds_in_tasks": sum(p["setup_builds_in_this_task"] for p in proofs),
                    "worker_starts": sum(p["worker_starts"] for p in proofs),
                    "restarts": sum(p["restarts"] for p in proofs),
                    "worker_start_failures": sum(p["worker_start_failures"] for p in proofs),
                    "every_start_same_artifact": all(p["every_start_loaded_the_same_artifact"] for p in proofs),
                    "every_start_setup_calls_zero": all(p["every_start_setup_calls_zero"] for p in proofs),
                    "rows_with_a_setup_stage": stages_setup,
                    "instantiate_s_per_start": [t for p in proofs for t in p["instantiate_s_per_start"]],
                    "hosts": sorted({(s.get("host") or {}).get("node", "?") for s in sums}),
                    "commits": sorted({(s.get("host") or {}).get("git_commit", "?") for s in sums}),
                    "slurm_jobs": sorted({(s.get("host") or {}).get("slurm_job_id", "?") for s in sums}),
                    "summary": H.summarize(rows),
                    "judge_fail_location": dict(collections.Counter(
                        f"{r['status']}|{r.get('judge_fail_location')}" for r in rows
                        if str(r["status"]).startswith("CLAIMED_"))),
                    "fail_reasons": dict(collections.Counter(r.get("reason") for r in rows
                                                             if r["status"] in ("FAIL", "ERROR", "SETUP_FAIL"))),
                }
                reg["complete"] = (reg["n_missing"] == 0 and reg["n_duplicated"] == 0 and not reg["foreign"]
                                   and reg["rows"] == reg["expected"])
                reg["setup_once"] = (sids == [reg["plan_setup_id"]] and art_shas == [reg["plan_artifact_sha256"]]
                                     and reg["setup_builds_in_tasks"] == 0 and reg["every_start_same_artifact"]
                                     and reg["every_start_setup_calls_zero"] and stages_setup == 0)
                reg["pending_judge"] = sum(r["status"] == "CLAIMED_PENDING_JUDGE" for r in rows)
                if not (reg["complete"] and reg["setup_once"]) or reg["pending_judge"]:
                    md["ok"] = doc["ok"] = False
                rd["regions"][region] = reg
            allrows = []
            for region in H.REGIONS:
                allrows += _rows_of_dir(H.RES / m / region / robot)
            rd["all"] = H.summarize(allrows)
            md["robots"][robot] = rd
        md["config_file_sha256_in_summaries"] = sorted(md["config_file_sha256_in_summaries"], key=str)
        md["config_file_sha256_plan"] = plan["methods"][m]["config_file_sha256"]
        md["one_config"] = md["config_file_sha256_in_summaries"] == [md["config_file_sha256_plan"]]
        if not md["one_config"]:
            md["ok"] = doc["ok"] = False
        doc["methods"][m] = md
    # cost from sacct over the B3 compute jobs (jobids/B3.txt, agent allocations excluded)
    jl = Path("/scratch/wg2381/claude_jobs/baselines/jobids/B3.txt")
    ids, purpose = [], {}
    for line in jl.read_text().splitlines():
        f = line.split(None, 3)
        if len(f) >= 2 and not f[1].startswith("agent_"):
            ids.append(f[0])
            purpose[f[0]] = f[3] if len(f) > 3 else ""
    acct = _sacct(ids)
    doc["cost"] = {"jobs": acct, "cpu_h": sum(r["cpu_h"] for r in acct), "gpu_h": sum(r["gpu_h"] for r in acct),
                   "budget": {"cpu_h": 600, "gpu_h": 100}, "job_purpose": purpose}
    if a.reruns:
        doc["infrastructure_reruns"] = json.loads(Path(a.reruns).read_text())
    H._dump(H.RES / "collect.json", doc)
    for m, md in doc["methods"].items():
        for robot, rd in md["robots"].items():
            print(m, robot, rd["all"]["n"], rd["all"]["status_counts"],
                  {r: (v["complete"], v["setup_once"]) for r, v in rd["regions"].items()})
    print("ok", doc["ok"], "cpu_h %.2f gpu_h %.2f" % (doc["cost"]["cpu_h"], doc["cost"]["gpu_h"]))


def _rows_of(f):
    import gzip
    with (gzip.open(f, "rt") if f.suffix == ".gz" else open(f)) as fh:
        return [json.loads(x) for x in fh if x.strip()]


def _rows_of_dir(d):
    rows = []
    for f in sorted(list(d.glob("task_*.jsonl")) + list(d.glob("task_*.jsonl.gz"))):
        rows += _rows_of(f)
    return rows


# ================================================================================================ storage
FULL_ROWS = H.OUTB / "b3_rows"
SLIM_DROP = ("route_polyline", "method_path_uv")


def cmd_store(a):
    """Gzip the finished task files for commit (``bl_harness archive`` semantics: the raw ``method_path_uv`` of a
    judged row is dropped, ``route_polyline`` is the exact judged path). ``--slim`` methods (SplatNav: dense exact
    Bezier polylines, ~30 kB per row) keep the full gzipped rows under ``outputs/baselines/b3_rows/`` (uncommitted,
    SHA-256 sidecar) and commit slim rows without the polylines; every slim row names its full file + SHA, and
    ``polyline_sha256`` (of the exported world polyline) stays in the slim row."""
    import gzip
    for m in a.methods:
        for f in sorted((H.RES / m).glob("*/*/task_*.jsonl")):
            rows = [json.loads(x) for x in open(f) if x.strip()]
            if any(r["status"] == "CLAIMED_PENDING_JUDGE" for r in rows):
                raise SystemExit(f"pending judge rows in {f}")
            for r in rows:
                if r.get("route_polyline") is not None:
                    r.pop("method_path_uv", None)
            if m in a.slim:
                full = FULL_ROWS / f.relative_to(H.RES).with_suffix(".jsonl.gz")
                full.parent.mkdir(parents=True, exist_ok=True)
                with gzip.open(full, "wt") as g:
                    for r in rows:
                        g.write(json.dumps(r, default=float) + "\n")
                sha = H.sha256_file(full)
                Path(f"{full}.sha256").write_text(f"{sha}  {full.name}\n")
                out = []
                for r in rows:
                    r = {k: v for k, v in r.items() if k not in SLIM_DROP}
                    r["full_row_file"], r["full_row_file_sha256"] = str(full), sha
                    out.append(r)
                rows = out
            with gzip.open(f"{f}.gz", "wt") as g:
                for r in rows:
                    g.write(json.dumps(r, default=float) + "\n")
            n = sum(1 for _ in gzip.open(f"{f}.gz", "rt"))
            if n != len(rows):
                raise SystemExit(f"{f}.gz has {n} rows, expected {len(rows)}")
            f.unlink()
            print(m, f, len(rows), "slim" if m in a.slim else "full", flush=True)


# ================================================================================================ spot check
def cmd_spotpick(a):
    rng = np.random.default_rng(a.seed)
    pick = {}
    for m in METHODS:
        rows = []
        for robot in H.ROBOTS:
            for region in H.REGIONS:
                rows += [(r["robot"], r["region"], r["pair_id"], r["status"])
                         for r in _rows_of_dir(H.RES / m / region / robot)]
        pick[m] = {}
        for cls, sel in (("SUCCESS", lambda s: s == "SUCCESS"), ("CLAIMED_*", lambda s: s.startswith("CLAIMED_"))):
            pool = sorted(r for r in rows if sel(r[3]))
            idx = rng.choice(len(pool), min(5, len(pool)), replace=False) if pool else []
            pick[m][cls] = {"pool": len(pool), "rows": [list(pool[i]) for i in sorted(idx)]}
    H._dump(B3 / "spotcheck_pick.json", {"seed": a.seed, "rule": "per method 5 SUCCESS + 5 CLAIMED_* rows, uniform "
                                         "without replacement over the sorted (robot, region, pair_id) pool", "pick": pick})
    print(json.dumps({m: {c: v["pool"] for c, v in d.items()} for m, d in pick.items()}))


def cmd_spotcheck(a):
    """Fresh process: re-export each picked row's stored exact ``route_polyline`` and judge it again."""
    pick = json.loads((B3 / "spotcheck_pick.json").read_text())["pick"]
    judges, out, ok = {}, [], True
    for m, d in pick.items():
        for cls, v in d.items():
            for robot, region, pid, status in v["rows"]:
                row = next(r for r in _rows_of_dir(H.RES / m / region / robot) if r["pair_id"] == pid)
                if "route_polyline" not in row:             # slim committed row: the full row is under outputs/
                    full = Path(row["full_row_file"])
                    if H.sha256_file(full) != row["full_row_file_sha256"]:
                        raise SystemExit(f"{full} differs from the SHA in the slim row")
                    row = next(r for r in _rows_of(full) if r["pair_id"] == pid)
                if (region, robot) not in judges:
                    judges[(region, robot)] = H.Judge(region, robot)
                J = judges[(region, robot)]
                uv = np.asarray(row["route_polyline"], float)
                P, res = J.export(uv, row["goal_uv"])
                j = J.judge(res)
                st = H.outcome(True, j)
                rec = {"method": m, "robot": robot, "region": region, "pair_id": pid, "stored_status": status,
                       "rejudged_status": st, "stored_judge_reason": row.get("judge_reason"),
                       "rejudged_judge_reason": j["judge_reason"],
                       "stored_polyline_sha256": row.get("polyline_sha256"), "rejudged_polyline_sha256": H.poly_sha(P),
                       "judge_wall_s": j["judge_wall_s"]}
                rec["agrees"] = (st == status and rec["stored_polyline_sha256"] == rec["rejudged_polyline_sha256"]
                                 and j["judge_reason"] == row.get("judge_reason"))
                ok &= rec["agrees"]
                out.append(rec)
                print(m, robot, region, pid, status, "->", st, rec["agrees"], flush=True)
    H._dump(B3 / "spotcheck.json", {"all_agree": ok, "n": len(out), "host": H.host(), "rows": out})
    print("all agree", ok, len(out))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("plan")
    s = sub.add_parser("streams")
    s.add_argument("--method", required=True)
    t = sub.add_parser("task")
    t.add_argument("--method", required=True)
    t.add_argument("--index", type=int, required=True)
    c = sub.add_parser("collect")
    c.add_argument("--reruns", help="json describing rows re-run after infrastructure loss")
    sp = sub.add_parser("spotpick")
    sp.add_argument("--seed", type=int, default=20261012)
    sub.add_parser("spotcheck")
    st = sub.add_parser("store")
    st.add_argument("--methods", nargs="+", default=list(METHODS))
    st.add_argument("--slim", nargs="*", default=["splatnav"])
    a = ap.parse_args(argv)
    {"plan": cmd_plan, "streams": cmd_streams, "task": cmd_task, "collect": cmd_collect, "spotpick": cmd_spotpick,
     "spotcheck": cmd_spotcheck, "store": cmd_store}[a.cmd](a)


if __name__ == "__main__":
    main()
