# Where the wall clock goes: stage timings and GMC's efficiency (Amendment 3 §P4)

> **DRAFT: sections 2b, 4b and 5 wait on the timing sweep 17965980** (running since 2026-09-18
> 21:52 UTC). Every number below that does not say "pending" is final and comes from
> `gmc/results/height/timing/tables.md`, which is machine-written by
> `experiments/plane_timing_report.py`.

**Claims boundary (applies to every number here):**
- The plane floor is a **user-approved manual scene-definition change** (Amendment 3 P1). It is not a
  reconstruction method and not a guaranteed outer approximation. Results are sound with respect to the
  edited scene only.
- The P2/P3 cases timed here rest on criterion 3 **relaxed by user decision D1** (`r + 0.05` m on each
  robot's own certified map). Criteria 1 and 3 intersect in **0.9 m²** of the hall as built and
  **1.2 m²** with every phantom cell deleted.
- Amendment 2's three runs (marked **A2**) are on the unedited scene with the Amendment 1 floor rule.
  They predate `timing.py`, so they enter the compile and query series only.
- τ = 0.3, ρ = 2.0, the robots, `configs/height_showcase.yaml`, GMC, `verify_curve` and `replay3d` are
  untouched. P4 only timed them.

The user's question: *"每个阶段的时间 … 从降噪处理到真实的算法进行路径选择，再到证明路径的可行性、连通性，
再到证明算法本身的效率。"* Sections 1–4 answer its four parts in that order. Section 5 is the practical
statement.

## 0. What was measured

| source | what it is | units | stage timing |
|---|---|---|---|
| P2 shared case | one job per robot: project → compile → query → verify → replay3d | 4 (incl. the cylinder `BUDGET=2` retry) | all nine P1e stages |
| P3 long case + cylinder ladder | same | 8 (incl. rung 3 `BUDGET=2`) | all nine |
| A2 (Amendment 2) | same pipeline, unedited scene | 3 | `compile_seconds` / `query_seconds` only |
| P2/P3 searches | `scene_load` + many `project` calls | 94 projection records in total | `scene_load`, `project` |
| **P4 timing sweep** 17965980 | the one dedicated sweep; see §0.1 | pending | all nine, plus GMC's own query report |

- All times are **wall clock on one core**. GMC is single-threaded: the P2 cylinder used 0.99 of 8 cores.
- This cluster's `sacct` reports `TotalCPU = 0` for every job, so wall time is the only time there is.
- The 15 single-job runs landed on four node families (cs6xx, cl01x, gr104, gl060), which puts
  cross-run noise into every fit that pools them.
- The sweep ran all its lanes on one node, cs616 (Intel Xeon Platinum 8592+), four lanes side by side.
  Its calibration against P3's own runs is in §2b.

### 0.1 Why a sweep was needed

The runs alone could not separate the three axes the prompt asks for:
- **Window, supports and distance moved together.** P3's cylinder ladder grew the window and the A–B
  distance at the same time, and the other runs differ in robot, site and scene.
- **Every run was one compile followed by one query.** "Compile once, query many" was never measured.
- **The saved JSONs drop GMC's own query report**, so no run says how many refinement rounds or
  support calls its query used.

This last point matters because `mobility.query.query` is not a lookup. On an UNKNOWN attempt it calls
`refine_compiler(mc, …)`, which bisects an orientation slab and rebuilds the derived graph in place.
With `initial_intervals: 1`, a first query can therefore carry compile work.

The sweep (`experiments/plane_timing.py`, `hpc/planefloor_timing.sbatch`) is one job with four
single-core lanes:
- **prep:** the per-scene chain, timed step by step; hall-wide projections; then compile-only on the
  biggest maps.
- **cyl:** the cylinder on one pair (ladder rung 0, 2.43 m), with the window grown 7 → 116 m².
- **sweeper** and **uav:**
  - the long case's map with 7 goals at 2.43–11.24 m, run **warm** (one compile, then all 7 queries
    twice) and **cold** (a fresh compile per goal);
  - the rung-0 pair on a growing window.
- The sweeper lane ends with the **cylinder warm** run on rung 2's window, querying goals 0–2 twice.

## 1. 降噪 / scene prep: paid once per scene, not per query

| step (P4 sweep, one core) | seconds | what it does |
|---|---|---|
| PLY decode | 5.2 | 7,340,008 rows → 7,319,425 finite splats |
| floor fit, gravity rotation, crop | 2.1 | — |
| Amendment 1 floor rule | 10.6 | −71,594 splats |
| five band rasters + phantom test | 3.5 | 397,047 cells at 0.05 m, 94,780 phantom |
| plane-floor replacement | 2.1 | −146,977, +1,014 plane tiles → 7,101,868 |
| npz write | 0.7 | 0.80 GB |
| npz load (what every run pays) | 0.7 | — |
| **total, once per scene** | **≈ 25 s** | — |

- **The rebuild reproduced P1 exactly:** every count matches `p1b_build.json` (`checks.all_match: true`).
- **Projection is per robot and per window, and it is cheap.** It scans all 7.1 M splats whatever the
  window, so it is a fixed cost plus a per-support term. Linear fits over every recorded projection:

  | robot | fixed cost | per support | n | R² |
  |---|---|---|---|---|
  | cylinder | 4.0 s | 32 µs | 39 | 0.98 |
  | uav | 4.9 s | 29 µs | 27 | 0.88 |
  | sweeper | 4.8 s | 38 µs | 28 | 0.37 |

  The sweeper's supports are too few to move its time off the fixed scan.
- **Whole hall, measured:** 7.4 s (sweeper, 51,499 supports), 19.4 s (uav, 421,853) and 75.8 s
  (cylinder, 2,148,885).

**So scene prep is not where the time goes.** Scene prep is paid once per scene; projection once per
robot and window. In every single-job run, scene load + projection is **0.1–3.2 %** of the unit's
wall clock. The one exception is the tiny P2 sweeper run, at 16.8 % of 31 s.

## 2. Path selection: compile vs query

### 2a. What the single-job runs say (final)

| run | robot | supports | A–B m | compile s | query s | query ÷ compile | status |
|---|---|---|---|---|---|---|---|
| A2 | uav | 143 | 4.15 | 0.6 | 0.3 | 0.43 | certified |
| A2 | sweeper | 477 | 2.02 | 2.0 | 3.4 | 1.76 | certified |
| A2 | cylinder | 3,791 | 2.69 | 17.3 | 409.4 | **23.65** | certified |
| P2 | sweeper | 1,779 | 2.36 | 7.0 | 11.3 | 1.62 | certified |
| P2 | uav | 19,867 | 2.36 | 108.5 | 171.9 | 1.58 | certified |
| P3 | sweeper | 6,436 | 11.24 | 25.0 | 107.5 | 4.29 | certified |
| P3 | uav | 39,834 | 11.24 | 153.4 | 292.9 | 1.91 | certified |
| P3 rung 0 | cylinder | 21,144 | 2.43 | 81.0 | 174.8 | 2.16 | certified |
| P3 rung 1 | cylinder | 45,572 | 4.00 | 174.7 | 628.8 | 3.60 | certified |
| P3 rung 2 | cylinder | 46,835 | 5.45 | 185.0 | 445.0 | 2.41 | certified |
| P3 rung 3 (+`BUDGET=2`) | cylinder | 73,178 | 6.90 | 283.9 / 293.8 | ≥ 722.5 / ≥ 1,289 | ≥ 2.5 / ≥ 4.4 | UNKNOWN |
| P3 rung 4 | cylinder | 149,303 | 8.58 | 581.0 | ≥ 1,714 | ≥ 2.95 | UNKNOWN |
| P2 (+`BUDGET=2`) | cylinder | 156,426 | 2.36 | 675.7 / 766.4 | ≥ 7,968 / ≥ 15,027 | ≥ 11.8 / ≥ 19.6 | UNKNOWN |

**The prompt expects compile to be the expensive part and the query the cheap part. For the first
query on a map, the data say the opposite.** In every finished hall run the query took **1.6–4.3×**
as long as the compile, and 24× for the A2 cylinder. Only the 143-support A2 uav map queried faster
than it compiled. On the maps that failed, the query is ≥ 2.5× to ≥ 19.6× the compile and still
unfinished. §2b measures whether a second query on the same map is cheaper.

**Compile is predictable, and `compile_slabs` is all of it:**
- Fit on the 15 single-job runs, 143–156,426 supports, R² 0.998: **`t = 0.00436 · N^1.00`**.
  That is linear, at 4.4 ms per support.
- Per robot: cylinder N^1.00 (n = 9), uav N^1.00 (n = 4), sweeper N^1.06 (n = 5, incl. the sweep's
  51 k hall map).
- `compile_slabs` (`build_slabs`) is **96.0–96.7 %** of compile in all 15 timed compiles.
  `compile_pairs` and `compile_mobility` are ~2 % each.

**Query is not predictable from supports:**
- The four finished single-run cylinder queries fit `t ∝ N^0.09` with **R² 0.04**. That is no
  relation, and all five UNKNOWN runs lie above it.
- For the sweeper and the uav there are only 3 single runs each. **No fit** was made; the table above
  is the data.

### 2b. Compile once, query many (pending sweep)

*Pending.* Measured so far, sweeper on the long-case map (6,436 supports):
- compile once: 33 s;
- warm queries at 2.43 / 4.00 / 5.45 / 6.90 m: 19.9 / 55.4 / 55.2 / 74.7 s, all with **0 refinement
  rounds**.

So on this map the query's cost is not deferred compile work: nothing was refined. It is the certified
lifting of the path itself, 0.8–1.6 M support calls, and it grows with distance.

## 3. Proving the path: verify_curve and replay3d

| | verify_curve ÷ query | replay3d |
|---|---|---|
| all 18 finished units so far (9 single-job runs + 9 sweep units) | **0.49–0.51** | 1.8–4.4 s; 0.4–1.7 % of a single-job unit (6.7 % of the 31 s P2 sweeper run) |

- **`verify_curve` costs half a query, every time.** It is the independent 2D re-check of the route
  against every support pair. Across 18 units, 3 robots, 1,779–46,835 supports and 2.4–11.2 m, it
  never left 0.49–0.51 × the query.
  - The query itself runs a SAFE → verify → refine loop (`mobility/query.py`) and certifies its
    lifted path before returning it. The ratio is consistent with about half of each query being one
    certification pass of the same size as `verify_curve`'s. That is an inference from the ratio. The
    query's internals were not timed: `mobility/` is read-only.
- **`replay3d` is negligible:** 1.8–4.4 s against 7.1 M splats, growing with route length. This is the
  3D check with separating directions, the one that proves the route is collision-free in the room's
  own geometry rather than in the projection.
- **Proof is not cheap, but it is bounded by the query.** Prove (verify + replay3d) is 23–29 % of
  every finished single-job unit. It is never the dominant stage.
- **Connectivity** in the D1 sense (start and goal in one component of `dist > r`) is a raster
  pre-check, not a GMC stage. The sweep records its cost: the `d1_raster_seconds` of each
  `dist` / `project` row.

## 4. Efficiency of the algorithm

### 4a. From the runs (final)
- **Compile ∝ N^1.00.** 4.4 ms per support, n = 15, 143–156,426 supports. With the sweep's first
  compile-only point: n = 18, the same exponent.
- **Projection:** ≈ 4–5 s + ~30 µs per support.
- **Query:** no fit across runs predicts it (above). What the runs do show:
  - every cylinder map ≤ 46,835 supports certified;
  - every one ≥ 73,178 exhausted a budget, at `BUDGET=1` and at `BUDGET=2`.
- **verify_curve ≈ 0.5 × query. replay3d is a few seconds.**

### 4b. From the sweep (pending)
- query vs A–B distance on one fixed map, cold and warm;
- query and verify vs supports at one fixed pair;
- compile past 156 k supports (422 k uav hall, 703 k cylinder ladder map, if the memory guard allows);
- run-to-run noise: 7 cold compiles of the same map.

## 5. Which stage dominates where, and the day (pending sweep)

Extrapolation, rests on n = 18 compiles of 143–156,426 supports:
- 24 h of compile ≈ **21 M supports**, 134× beyond the largest map compiled so far.
- The whole hall, measured at 51 k (sweeper), 422 k (uav) and 2.15 M (cylinder) supports, would
  compile in **≈ 0.06 h, 0.5 h and 2.5 h** respectively.

**Compile is not what limits GMC here; the query is.** The sweep's big compile-only units test the
linear extrapolation directly.

## Reproduce

```
cd gmc && export PYTHONPATH=src:experiments MPLBACKEND=Agg
sbatch --job-name=pf_p4_sweep hpc/planefloor_timing.sbatch      # the sweep (self-test first)
python experiments/plane_timing_report.py --step collect         # -> results/height/timing/records.json
python experiments/plane_timing_report.py --step report          # -> timing_report.json, tables.md, figs/
```

Figures (`gmc/results/height/timing/figs/`):
- `where_time_goes.png`
- `stages_vs_supports.png`
- `stages_vs_area.png`
- `stages_vs_distance.png`
- `compile_once_query_many.png`
- `prep_per_scene.png`
