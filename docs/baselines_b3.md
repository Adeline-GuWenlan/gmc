# Baselines on the F4 benchmark — full runs (bl stage B3)

Plan: `docs/baselines_f4_plan.md` §3–§6. Adapters, tuning, pilots: `docs/baselines_adapters.md`. Branch `baselines-f4`.
Claims are **[E]** (evidence: the committed file named next to it) or **[G]** (guess / untested).
Code: `gmc/experiments/bl_b3.py` (plan / streams / task / collect / spotpick / spotcheck / store) on top of
`bl_harness.py`, which was **not changed** (8c5f2ef → HEAD: no diff in `bl_harness.py`, `bl_worker.py`, the four adapters
or `configs/baselines/`). Sbatch helpers `gmc/hpc/baselines/b3_gpu.sbatch`, `b3_cpu_array.sbatch`. Job IDs:
`/scratch/wg2381/claude_jobs/baselines/jobids/B3.txt`.

## 1. What ran

- **All four methods, all 5000 F4 pairs, both robots.** No stratified subset: the plan projected 5.3 GPU-h and 31
  CPU-h in total, far below every method's share (plan §6) **[E]** (`results/baselines/b3/plan.json`).
- **Configs:** the frozen `configs/baselines/{foci,splatnav,pno,cust_fields}.json`, unchanged since their freeze
  commits.
  - Every task summary records the config file SHA-256.
  - Per method there is exactly one config SHA across all rows: `collect.json` → `one_config: true` for all four.
  - Adapter SHA now = adapter SHA at freeze, for all four (`plan.json` → `adapter_unchanged_since_freeze`).
- **Setup ("compile") reused, not rebuilt.**
  - The 24 setup artifacts (4 methods × 2 robots × 3 regions) were built by B1/B2 from the frozen configs.
  - Before any run, each was checked three ways (plan.json → `region_robot.*.setup`):
    - SHA-256 = its sidecar;
    - path = hash(frozen config, current adapter source);
    - setup id = the one on the pilot rows.
  - So no setup job was needed.
- **Layout** (plan.json, committed in 8c5f2ef before any full-run row existed):
  - **GPU methods** (FOCI, SplatNav, PNO): one L40S job each, with 4 harness streams. Each region × robot is split into
    tasks of ≤ 1.4 estimated stream-hours, packed longest-first into the streams.
  - **cust_fields:** a CPU array of 26 tasks, 12 at a time.
  - **Judge:** inline in every stream, the pilots' validated path. Deferred judging was not used.
  - **Limits:** 120 s per query from outside; ≤ 2 GPU jobs at a time; ≤ 64 GB queued.
  - **Queue order** (cheap first): FOCI and cust_fields, then SplatNav, then PNO behind FOCI.

| job | what | node | wall | result |
|---|---|---|---|---|
| 19525656 | FOCI, 4 streams | gl026 | 0:40 | 6/6 tasks complete |
| 19525657 | SplatNav, 4 streams | gl023 | 1:54 | 8/8 tasks complete |
| 19525658 | PNO, 4 streams | gl026 | 2:08 | **CANCELLED by uid 0** at 12:31Z; 6/8 tasks complete |
| 19528737 | PNO GAPW1 cylinder tasks 0, 1, resumed (infrastructure rerun) | gl031 | 0:10 | 2/2 complete |
| 19525659 | cust_fields, array 0–25 %12 | cs6xx | 0:12–0:32 per task | 26/26 complete |
| 19529048 | collect → spotpick → spotcheck → store | cs607 | 0:02 | ok |

**One infrastructure rerun, disclosed** (`results/baselines/b3/reruns.json`) **[E]**:
- **What was lost.** sacct reports PNO job 19525658 as `CANCELLED by 0` after 2:08 of its 5 h limit. This stage did
  not cancel it. Its two GAPW1-cylinder tasks had answered 702 + 645 of 1500 pairs, with whole, fsync'd rows and no
  duplicates.
- **The rerun.** Job 19528737 resumed both tasks from that checkpoint and answered the other 153 pairs: same frozen
  config, same setup artifact and setup id, harness code unchanged (commit 03ed70c differs from 8c5f2ef only in
  `bl_b3.py`).
- **Where it shows.** These two task summaries record `resumed_from_checkpoint`.
- **Nothing else was rerun.** No row anywhere is an ERROR, TIMEOUT or SETUP_FAIL.

## 2. Completeness and setup-once (`gmc/results/baselines/collect.json`, `ok: true`) [E]

For every method × robot × region (24 cells):
- **Rows.** Rows = F4 pairs of the region (2500 / 1500 / 1000): 0 missing, 0 duplicated, 0 foreign. All 8 × 5000 rows
  are present.
- **One setup.** One `setup_id` over all rows, and it equals the plan's. One artifact SHA over all tasks, and it equals
  the plan's.
- **No setup inside a task or query.** `setup_builds_in_this_task = 0` in every task. Every worker start loaded the
  same artifact with 0 setup calls. No row has a setup stage.
- **Worker restarts:** 0.
- **Per-worker-start instantiate time** (live planner from the artifact, never charged to a query):

| method | instantiate time per worker start |
|---|---|
| FOCI | 14–25 s |
| SplatNav | 17–36 s |
| PNO | 5–15 s |
| cust_fields | 1–2 s |

## 3. Outcomes (plan §5 classes) [E]

`collect.json` → `methods.<m>.robots.<robot>.all / regions.<R>.summary`. SUCCESS = claimed **and** passed the judge.

| method | robot | SUCCESS | CLAIMED_COLLIDES | CLAIMED_UNPROVEN | CLAIMED_KINEMATICS | FAIL | TIMEOUT / ERROR | SUCCESS WWEST / GAPW1 / S |
|---|---|---|---|---|---|---|---|---|
| SplatNav | cylinder | **4958** (99.2 %) | 0 | 0 | 0 | 42 | 0 / 0 | 2459/2500, 1499/1500, 1000/1000 |
| SplatNav | sweeper | **5000** (100 %) | 0 | 0 | 0 | 0 | 0 / 0 | all |
| FOCI | cylinder | **2102** (42.0 %) | 224 | 2660 | 0 | 14 | 0 / 0 | 681/2500, 423/1500, 998/1000 |
| FOCI | sweeper | **3665** (73.3 %) | 239 | 210 | 0 | 886 | 0 / 0 | 1670/2500, 1030/1500, 965/1000 |
| PNO | cylinder | **5000** (100 %) | 0 | 0 | 0 | 0 | 0 / 0 | all |
| PNO | sweeper | **5000** (100 %) | 0 | 0 | 0 | 0 | 0 / 0 | all |
| cust_fields | cylinder | **0** | 0 | 0 | 0 | 5000 | 0 / 0 | 0, 0, 0 |
| cust_fields | sweeper | **113** (2.3 %) | 0 | 0 | 0 | 4887 | 0 / 0 | 0, 113/1500, 0 |

The full runs reproduce the pilots: FOCI cylinder 44 % → 42 %, sweeper 77 % → 73 %; SplatNav and PNO ≈ 100 %;
cust_fields 0 % / 3 % → 0 % / 2.3 %.

### SplatNav
- **Cylinder FAILs** (42): `astar_no_path` 39 (all WWEST), `qp_infeasible` 3.
- **Unsafe claims:** none.

### FOCI
- **FAILs** are all IPOPT: `Maximum_Iterations_Exceeded` 14 cylinder and 885 sweeper, plus 1 `Error_In_Step_Computation`.
- **Where the unsafe claims lie** (`judge_fail_location`):
  - **Cylinder:** 2754 of 2884 are on FOCI's own curve, 2610 of them CLAIMED_UNPROVEN (it grazes Gaussians:
    a soft overlap cost has no margin). The other 130 are on the harness's completion segments: 103 on the start
    segment, 27 on the goal segment.
  - **Sweeper:** 349 of 449 are on the appended goal segment (FOCI's soft goal stops short and the §3.3 completion is
    judged). 98 are on its own curve, 2 on the start segment.
- **By region:** S is nearly solved for both robots (998 / 965 of 1000). WWEST and GAPW1 cylinder carry the unsafe
  claims.

### PNO
- 10000/10000. Expected: grid A* is complete on a judge-sound map, and the completion segments are millimetres
  (`docs/baselines_adapters.md` §9–§10).

### cust_fields
- **Cylinder:** every endpoint lies inside a convex cover (`endpoint_in_obstacle_cover` 5000), as the B2 world check
  predicted (2500 + 1500 + 1000 swallowed).
- **Sweeper, WWEST:** 2500 swallowed.
- **Sweeper, GAPW1:** 384 swallowed, `nf_stuck` 484, `nf_max_steps` 519, **113 SUCCESS**.
- **Sweeper, S:** 121 swallowed, `nf_stuck` 527, `nf_max_steps` 352, 0 SUCCESS.
- **Unsafe claims:** none.
- **Attribution** (from B2, for B4): the swallowed endpoints are our star-world adaptation; the stuck / max-step
  descents are the method's own.

## 4. Timing (per query; judge excluded from the method) [E]

| method | robot | algorithm s median / p95 / max | judge s median | completion segments (start / goal) |
|---|---|---|---|---|
| SplatNav | cylinder | 0.60 / 2.00 / 3.7 | 0.99 | 4958 / 4958 (every claimed row: A* voxel-centre endpoints) |
| SplatNav | sweeper | 0.33 / 1.20 / 3.0 | 0.68 | 5000 / 5000 |
| FOCI | cylinder | 0.36 / 1.14 / 4.8 | 0.04 | 4986 / 4986 |
| FOCI | sweeper | 0.39 / 3.41 / 4.6 | 0.04 | 4114 / 4114 |
| PNO | cylinder | 2.13 / 5.50 / 7.9 | 0.26 | 5000 / 5000 (mm cell-centre segments) |
| PNO | sweeper | 1.57 / 3.95 / 8.1 | 0.11 | 5000 / 5000 |
| cust_fields | cylinder | 0.0001 (FAIL at the snap) | — | 0 / 0 |
| cust_fields | sweeper | 0.0002 / 43.9 / 72.1 | 0.15 | 9 / 113 |

- **Timing basis.** These were measured with 4 streams per L40S (GPU methods) or one core per task (cust_fields).
  The pilots used the same layouts.
- **Setup is separate.** Setup ("compile") times per robot × region are in the setup records (`plan.json` →
  `build_setup_wall_s`) plus the shared raster build for PNO and cust_fields (adapters doc §9.2).
- **Left for B4:** stage breakdowns are in each row's `stages`; amortised costs and the measurement sheets are B4's job.

## 5. Spot-check: stored exact paths re-judged in a fresh process (`results/baselines/b3/spotcheck.json`) [E]

- **Sample.** `bl_b3.py spotpick`, seed 20261012 (`b3/spotcheck_pick.json`): per method, 5 random SUCCESS and
  5 random CLAIMED_* rows.
- **How it was checked.** Job 19529048 ran it in a new gmc-venv process. Each picked row's stored `route_polyline` was
  re-exported through GMC's exporter and judged again by `replay_plan` on the SHA-checked F3 compile. Status, judge
  reason and `polyline_sha256` were compared with the stored row.
- **Result: 25 / 25 agree.**
  - FOCI: 5 SUCCESS + 4 CLAIMED_UNPROVEN + 1 CLAIMED_COLLIDES.
  - SplatNav, PNO, cust_fields: 5 SUCCESS each.
- **Smaller samples.** SplatNav, PNO and cust_fields have **no** CLAIMED_* rows at all (pool 0), so their CLAIMED_*
  sample is empty by necessity, not by choice.

## 6. Where the rows are

- **FOCI, PNO, cust_fields:** `gmc/results/baselines/<method>/<R>/<robot>/task_NN.jsonl.gz` + `task_NN.summary.json`,
  committed in full (`bl_b3.py store` = `bl_harness archive`: raw `method_path_uv` dropped from judged rows;
  `route_polyline` is the exact judged path).
- **SplatNav:** each row's exact path is a dense Bézier polyline of about 30 kB.
  - **Full rows:** 133 MB gzipped, under `gmc/outputs/baselines/b3_rows/splatnav/<R>/<robot>/task_NN.jsonl.gz`, with
    `.sha256` sidecars, uncommitted (plan rule for large files).
  - **Committed rows:** slim rows under `results/` without `route_polyline` / `method_path_uv`. Each slim row names its
    full file and its SHA, and keeps `polyline_sha256`.
  - `bl_b3.py spotcheck` reads the full file when it meets a slim row.
- **Other committed files:** GPU memory logs `results/baselines/b3/gpu_mem/` (peak FOCI 2.4 GB, SplatNav 5.8 GB,
  PNO 17.1 GB for 4 streams).
- **Not committed:** worker logs, under `gmc/outputs/baselines/worker_logs/`.

## 7. Budget [E]

**29.1 CPU-h of 600 and 4.88 GPU-h of 100.** These come from `sacct` over the B3 compute jobs: allocated cores ×
elapsed, with GPU jobs counted with their 4 cores (`collect.json` → `cost`).

| | CPU-h | GPU-h |
|---|---|---|
| FOCI | 2.70 | 0.68 |
| SplatNav | 7.63 | 1.91 |
| PNO (incl. the cancelled job's 2:08) | 8.56 | 2.14 |
| PNO rerun | 0.64 | 0.16 |
| cust_fields | 9.56 | 0 |
| collect | 0.01 | 0 |

The agent's own 1-core allocations are not included (19525431, 19525820, 19528751: < 0.5 CPU-h).
