"""bl B2: combined B3 projection over all four methods (plan §6) from the two pilot projections.

Reads ``results/baselines/pilot/projection_b1.json`` (SplatNav, FOCI) and ``projection_b2.json`` (PNO, cust_fields),
both made by ``bl_harness project`` from the frozen-config pilots, and writes ``results/baselines/pilot/projection.json``:
per method the B3 array layout, concurrency, method stream-hours, GPU-h, CPU-h (GPU-job allocations + CPU judge +
setup, incl. the shared raster build for the map-based methods), wall time, and whether plan §6's budget rule
(per-method share 600/4 = 150 CPU-h; 100 GPU-h total) forces the stratified 1000-pair subset.
CPU-h of a GPU job = its allocated cores x wall (4 cores per L40S job, one per harness stream).
"""
import json
from pathlib import Path

P = Path("results/baselines/pilot")
CPU_SHARE, GPU_TOTAL, CPU_TOTAL = 600 / 4, 100., 600.
GPU_JOBS, STREAMS = 2, 4               # plan/_rules: <= 2 GPU jobs at a time; 4 harness streams per L40S job
CPU_ARRAY_CONC = 12                    # CPU array tasks at a time (4 GB each; chain cap 64 GB with the agent + slack)


def main():
    b1 = json.loads((P / "projection_b1.json").read_text())
    b2 = json.loads((P / "projection_b2.json").read_text())
    out = {"definition": __doc__.strip(), "budget": {"B3_cpu_h": CPU_TOTAL, "per_method_cpu_share_h": CPU_SHARE,
                                                     "B3_gpu_h": GPU_TOTAL},
           "assumptions": {"gpu_jobs_at_a_time": GPU_JOBS, "streams_per_gpu": STREAMS,
                           "cpu_array_tasks_at_a_time": CPU_ARRAY_CONC,
                           "judge": "deferred (`--judge defer` on the GPU, `bl_harness judge` as a CPU array) for GPU "
                                    "methods; inline for CPU-only cust_fields"},
           "methods": {}}
    rasters = {}
    for m in ("pno", "cust_fields"):
        cfg = json.loads(Path(f"configs/baselines/{m}.json").read_text())
        rasters[m] = sum(v["build_wall_s"] for v in cfg["rasters"].values()) / 3600
    tot_gpu = tot_cpu = 0.
    for src in (b1, b2):
        for m, d in src["methods"].items():
            t = d["projection_5000x2"]
            stream_h = sum(r["method_h_one_stream"] for r in d["robots"].values())
            judge = t["judge_cpu_h"]
            setup = t["setup_h"] + rasters.get(m, 0.)
            if m == "cust_fields":                       # CPU only: one core per stream, judge inline
                gpu_h = 0.
                cpu_h = stream_h + judge + setup
                wall = (stream_h + judge) / CPU_ARRAY_CONC
                layout = (f"CPU array: region x robot x chunks of 100 pairs (50 tasks), %{CPU_ARRAY_CONC}, 4 GB, "
                          "judge inline")
            else:
                gpu_h = stream_h / STREAMS
                cpu_h = gpu_h * STREAMS + judge + setup     # GPU-job cores (4 x wall) + CPU judge array + setup
                wall = gpu_h / GPU_JOBS
                layout = (f"GPU: {GPU_JOBS} L40S jobs at a time, {STREAMS} harness streams each (region x robot "
                          "tasks), --judge defer; then `bl_harness judge` CPU array")
            over_cpu = cpu_h > CPU_SHARE
            out["methods"][m] = {"layout": layout, "method_stream_h": stream_h, "gpu_h": gpu_h, "cpu_h": cpu_h,
                                 "judge_cpu_h": judge, "setup_h_incl_raster": setup, "wall_h": wall,
                                 "over_cpu_share": over_cpu,
                                 "decision": ("stratified 1000-pair subset (plan §6: over its CPU share after "
                                              "parallelising)" if over_cpu else "full 5000 x 2"),
                                 "pilot": {r: {"n": v["summary"]["n"], "status_counts": v["summary"]["status_counts"],
                                               "median_alg_s": v["summary"]["algorithm_wall_s"]["median"]}
                                           for r, v in d["robots"].items()}}
            tot_gpu += gpu_h if not over_cpu else gpu_h / 5
            tot_cpu += cpu_h if not over_cpu else cpu_h / 5
    out["totals"] = {"gpu_h": tot_gpu, "cpu_h": tot_cpu, "within_B3_budget": tot_gpu <= GPU_TOTAL and tot_cpu <= CPU_TOTAL}
    (P / "projection.json").write_text(json.dumps(out, indent=1) + "\n")
    for m, v in out["methods"].items():
        print(f"{m:12s} stream_h {v['method_stream_h']:7.2f} GPU-h {v['gpu_h']:6.2f} CPU-h {v['cpu_h']:7.2f} "
              f"wall {v['wall_h']:6.2f} h  {v['decision']}")
    print("totals", out["totals"])


if __name__ == "__main__":
    main()
