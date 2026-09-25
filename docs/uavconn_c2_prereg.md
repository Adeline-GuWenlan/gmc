# uav-conn C2 pre-registration: benchmark + extended query set, success / smoothness / time

Status: written and committed by C2 **before** the extended pairs are generated and before either
method runs on them (2026-09-25). Benchmark predictions for M1/M5/N2/N2h/C4h are C1's, in
`docs/uavconn_design.md` §5 (commit `59625e9`); they are not changed here. Rules:
`/scratch/wg2381/claude_jobs/uavconn/prompts/_rules.md`.

Methods:
- **aerial3d**: the new pair-certified support-plane cell complex (`gmc.aerial3d`). Default
  `CompileConfig` / `QueryConfig`, one compile per scene variant, `query(compiled, start, goal)` only.
  Runner: `gmc/experiments/uavconn_run.py`.
- **lattice**: the baseline `gs3d.planner.LatticePlanner` (26-neighbour xyz A*, 0.10 m, margin 0.05),
  run by the **unchanged** `gmc/experiments/uavlamp_run.py` (this branch's `gs3d`/`uavlamp_*` code is
  byte-identical to `uav-lamp` 4357962), one `plan()` call per query.

Shared for both: archive `2a3a72d6…96cc` via the manifest-checked loader; `uavlamp_query.build_scene`
of the same spec (same crop, box, τ 0.3, level 2, body r 0.25 / half-height 0.10, margin 0.05); the final
safety check is the baseline's `replay_plan` with a **fresh** `GaussianBodyOracle` on a freshly built
`PreparedScene`; ordered evidence from `uavlamp_run.ordered_evidence` on the replayed trajectory
sampled at 0.02 s.

## 1. Benchmark set (5 queries, same specs as L2)

`gmc/configs/uavlamp_l2/{M1_main_low_start, M5_high_start, N2_plug_low_start, N2h_plug_high_start,
C4h_lamp_only_high_start}.json`. Ground truth from the scene design (L1/L2): M1, M5, C4h have paths (the
L2 baseline found them); N2/N2h have none inside the box (the plug closes the only opening; L2's lattice
exhausted its reachable set). aerial3d compiles: booth (M1, M5), booth + test-only plug (N2, N2h),
lamp-only (C4h). Predictions: design doc §5.

The baseline's recorded L2 numbers were timed with 4–7 single-threaded searches sharing one node. C2
re-times M1 for the baseline in its own 2-CPU job (job `uc_base_M1`). The other L2 baseline times are
used as recorded and labelled "L2, contended".

## 2. Extended set (for a success *rate*)

Generator: `gmc/experiments/uavconn_pairs.py`, seed **20260925**, template = the M1 spec (booth scene,
M1's box). 24 pairs:

- **Class A (12), corridor → table.** Start uniform in route-frame u ∈ [−2.6, −1.3], v ∈ [0.35, 2.05],
  z ∈ [0.20, 2.20]. That is the approach corridor in front of the lamp slab, whose C-obstacle starts at
  u ≈ −1.16. Goal centre uniform over the table-top footprint u ∈ [−0.02, 2.97], v ∈ [−0.03, 0.79],
  z ∈ [1.15, 2.25], so the body bottom minus the margin is at or above the table top (0.94) at z ≥ 1.09.
- **Class B (12), random free pairs.** Start and goal uniform over the aerial3d C-space domain's
  bounding box, which the generator derives from the scene (≈ u [−2.67, 3.37], v [−0.02, 2.42],
  z [0.15, 2.28]). The two must be ≥ 1.0 m apart.
- A pose is kept **only** if the shared gs3d oracle certifies it free:
  `GaussianBodyOracle.pose(..., margin_m=0.05).occupancy == "free"`, on the same SceneSpec both
  planners receive. Rejections are counted by reason. Poses are rounded to 0.1 mm and re-certified.
- Per pair the oracle's verdict on the straight start–goal segment is recorded as a stratum
  ("direct" vs "needs search"). It is not a filter.
- Every pair becomes a spec (`gmc/configs/uavconn_c2/ext/E{A,B}NN.json`). Only name, output, start,
  goal and budget differ from the M1 spec. The spec files are committed before any planner runs on them.
- **Baseline budget:** planner `max_wall_s` = 18 000 s (5 h), other budget fields as in M1. Each pair
  runs in its own sbatch job (2 CPU / 8 GB / 5 h 40 min), at most 12 queued at once. A budget stop or a
  job timeout counts as **failure-by-budget** and is reported as such.
- **aerial3d:** one booth compile in one job, then 24 queries. Each query gets one cold call (in a process
  forked from the fresh compile) and 3 warm calls.

## 3. Ground truth and success (fixed before running)

- **GT = PATH** if either method returns a path that passes the shared fresh replay. Otherwise
  **GT = NO-PATH-EVIDENCE**, and each method's evidence is recorded: aerial3d certified UNREACHABLE
  (possible-space cut, pair ids) / UNKNOWN (reason); lattice queue exhausted / budget stop / job timeout.
- **Success(method, pair)** holds in either of two cases:
  - GT = PATH, and the method returned a path that passes the shared fresh replay;
  - GT = NO-PATH-EVIDENCE, and the method gave a no-path verdict.

  aerial3d's no-path verdict is `UNREACHABLE` with a cut certificate. The lattice's is a queue
  exhausted before any budget. It is shown in its own column as "exhausted (not a certificate)".
- **Never folded into success:** UNKNOWN, budget stops, job timeouts. These are reported per method.
- **Contradictions:** aerial3d UNREACHABLE while the lattice returns a replay-passing path, or either
  method returning a path that fails the replay. Either is counted as wrong and reported as a soundness
  bug.
- **Rates** are reported three ways: per class, overall, and for the 5 benchmark queries. Wilson 95%
  intervals are given, because n = 12 / 24 is small.

## 4. Predictions (extended set)

| # | Prediction | P |
|---|---|---|
| X1 | GT = PATH for all 12 class-A pairs. Every class-A path of both methods passes under the lamp (`passes_under_lamp`) and reaches the above-table interval after the under-lamp one. | 0.9 |
| X2 | aerial3d REACHABLE on ≥ 10/12 class A and ≥ 10/12 class B. Misses are UNKNOWN (e.g. `safe_graph_disconnected_possible_connected` in thin free space near the soffit, table edge or walls), never a false UNREACHABLE. | 0.75 |
| X3 | Lattice success within 5 h on ≥ 9/12 class A (the L2 M1/M5 searches took 1.6–2.3 h) and ≥ 10/12 class B. | 0.7 |
| X4 | No contradiction on any of the 29 queries: no replay-failing path from either method, and no aerial3d UNREACHABLE where the lattice finds a path. | 0.95 |
| X5 | Class B: ≈ 4 ± 2 pairs cross the lamp plane u = −0.85 (these need the under-lamp opening); ≥ 4 pairs have a free straight segment. At most 2 class-B pairs have GT = NO-PATH-EVIDENCE (isolated pockets, e.g. under the table). | 0.7 |
| X6 | aerial3d per-query cold time ≤ 20 s on every pair and median ≤ 10 s. Booth compile 4–15 min, measured once. | 0.8 |
| X7 | Lattice per-query time: median ≥ 3 000 s over pairs whose straight segment is not free. On pairs with a free straight segment, both methods answer in seconds (within 10× of each other, either way). | 0.8 |
| X8 | Where both succeed, the aerial3d raw path is no longer than the lattice path on every pair, and total turning is lower on ≥ 90% of them. | 0.85 |
| X9 | After the same `gs3d.smoothing.optimize_trajectory` post-process, every smoothed curve that is kept passes a fresh continuous re-verification (`verify_piecewise_bezier`). There is **no** directional prediction for smoothed integrated squared jerk: C1's synthetic suite was mixed, 2 lower / 5 higher, and the A6 smoother eases to a stop at every anchor. | 0.9 |

## 5. Measurement protocol

- **Time.**
  - aerial3d: `compile_wall_s` once per scene (amortised over N queries: compile / N + per-query).
    Per-query `algorithm_wall_s` (cold, and warm × 3; it includes the in-query gs3d replay).
    Time-to-first-path = compile + cold query.
  - Lattice: `algorithm_wall_s` of its single call, with the index prepared separately
    (`--warm 1`, as L2 did for M5/N2/N2h/C4h). Time-to-first-path = index preparation + that call.
  - Speedups are ratios, with raw seconds next to them.
  - Every job records its CPU count, the node's CPUAlloc/CPUTot at start and end, and the load average,
    so shared nodes are visible.
- **Smoothness.**
  - Raw: `gmc.aerial3d.metrics.polyline_metrics` on both methods' raw polylines (aerial3d
    `polyline_world`, lattice trajectory knots): length, vertices, turning vertices (θ > 1e-6),
    total/max turning, bending energy, vertical travel.
  - Smoothed: the same `optimize_trajectory` with A6's UAV `SmoothingConfig` (as in
    `uavlamp_run.smooth` / `aerial3d_synthetic.SMOOTH`) on both methods' replay-passing raw
    trajectories. Each candidate curve is re-verified with a fresh oracle, then
    `smooth_segments_metrics` gives integrated squared jerk, duration and length.
  - Per-query table, plus median and IQR per method over the queries where both succeed.

## 6. What would count against the claim

- Any contradiction (X4).
- aerial3d REACHABLE with a failing shared replay.
- aerial3d success rate below the lattice's on the same pairs, counting budget-limited lattice runs
  as failures.
- A benchmark verdict of aerial3d other than design §5 (M1/M5/C4h REACHABLE; N2/N2h UNREACHABLE,
  or honest UNKNOWN).

Each of these is reported in the C2 report's first section if it happens.
