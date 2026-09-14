# Sweeper, one case (Amendment 2) — agent notes

Robot `sweeper`: disc r = 0.175 m, band 0.02–0.10 m above the floor. Plan:
`docs/superpowers/plans/2026-09-14-per-robot-cases-amendment.md`. Worktree `/scratch/wg2381/splathjb-percase`.
Ledger `/scratch/wg2381/claude_jobs/percase/jobids.txt`. Times are EDT.

## 2026-09-14

- 06:05 — Read the amendment, spec §3.3/§5.3/§5.4, `height_A4_done.md`, the A2/A4 worklog sections, `showcase_scene.py`,
  `showcase_run.py` (with `--case/--out-dir/--raw-subdir`), `height/{prism,project,band_shadow,floor,run,pathio}.py`,
  both `percase_*.sbatch`, and the A2/A4 scripts `case_certified_check.py`, `case_search_table.py`, `table_frames.py`,
  `pair_search.py`, `clear_map.py`, `proj_map_diag.py`.
  - Queue: one non-`pc_` job of mine (17756204, `bash`). Ledger is empty.
  - Opened `figs/diag/a4_case_certified_check_SWbench.png`. In the sweeper panel the bench is nearly free: blobs are
    floor slivers plus a few leg shadows. The cylinder panel shows the bench as a solid block at (1.5–2.7, 9.4–11.5).
    Opened `figs/case_overview.png`: bench leg at s ≈ 1.75, seat 0.38–0.45 m over s ≈ 1.8–3.66.
  - Compile-cost evidence: T2-8 compiles in 0.30 s with 1 orientation interval and 4.5 s with 16. `configs/height_showcase.yaml`
    has `initial_intervals: 1`. T1b (506 supports, 16 slabs) took 56 s. So the sweeper's ≈2.8k supports are probably
    minutes, not hours, but the window is still kept small.
- 06:08 — Wrote `gmc/experiments/percase_search_sweeper.py`, then the session hit a usage limit and stopped (nothing submitted).
- 09:56 — Resumed.
  - Fixes: removed an unused helper; the top 3 are now distinct start/goal pairs.
  - `py_compile` OK. `--selftest` (synthetic rasters, no scene, < 1 min) OK: wall-with-gap bottleneck 0.2375, raster
    path 2.025 m with 1.0 m under the mask, section runs/top/underside, crop/window snapping, JSON cleaning of numpy
    types, SIGALRM timeout.
  - Lazy imports and `configs/height_showcase.yaml` load (`initial_intervals` 1; probe half 1.0 m; abort 20 h).
  - Design of the search:
    - Regions SW/A/E/G/N, each pre-subset by the ρ-AABB + 2.5 m. The subset is checked identical to the full scene on
      the A2 window.
    - Certified sweeper raster per region; endpoints on a 0.25 m lattice with clearance ≥ 0.225 m that are not within
      0.25 m of overhang mass.
    - Pairs 2–6 m long, connected at the region level, crossing ≥ 0.3 m of sweeper-free cells under overhang mass
      (opaque ρ-bottom 0.15–1.75 m).
    - Section check (step_case bins; "under" = ≥ 5 free-low + overhang bins).
    - Windows at margins 0.5–2.0 m cropped from the region raster; metrics are clearances, connectivity at r / r+0.025
      / r+0.05, the bottleneck, and supports.
    - Ranked by valid, then under, then tier (bottleneck, clearance, run length, raster path under overhang), then
      support bucket, then bottleneck.
    - Exact pre-check (`project_scene` + `_support_raster`) on 8 finalists plus the A2 reference. The top 3 are
      re-checked on the full scene, each with a figure (sweeper map, section, and a cylinder map of the same window as
      a diagnostic only), plus showcase_run's probe.
    - Trial compile+query on the top candidates (≤ 40 min each, SIGALRM), as a selection aid only.
- 09:57 — Envelope: 1 `pc_` job queued, ledger 2 lines. Submitted **17777515** `pc_sweeper_search` (8 CPU / 64 GB /
  4 h) and appended it to the ledger. Waiting with a background until-loop.
- 10:0x — **17777515 COMPLETED in 2 m 47 s, no errors** (log `gmc/logs/pc_sweeper_search-17777515.out`).
  - Scene: 7,247,831 splats (floor rule on). Subset check on the A2 window: full scene 2,796 supports, subset 2,796,
    identical ids/means/covs (A4 reported 2,796).
  - Regions:

    | region | supports | endpoints | valid windows |
    |---|---|---|---|
    | SW | 5,034 | 468 | 1,287 (886 tier 0) |
    | A | 7,209 | 97 | 0 (no pair crosses free cells under an overhang) |
    | E | 12,280 | 30 | 0 (disc fits in 6 % of the region) |
    | G | 3,128 | 313 | 118 |
    | N | 15,342 | 280 | 0 |

    Inline section equals `_section` in SW and G.
  - Shortlist and finalists all cross **table B**, not the SW bench. They are small windows of ≈480–550 supports with
    bottleneck 0.325 m. For comparison, the A2 reference line under the bench, pre-checked:
    - original window: 2,796 supports, bottleneck 0.30;
    - 1.0 m margin: 276 supports, bottleneck only 0.2125.
    Its raster shortest path does not go under the seat (path under = 0).
  - **Top 1 `SW-p1-m0.5`:** window [−0.85, 9.75, 1.15, 12.8] (2.0 × 3.05 m), start (0.2625, 10.2625), goal (0.0125,
    12.2625), L 2.02 m.
    - Full-scene pre-check: 477 supports; start clear 0.372, goal 0.355; connected at r, r+0.025 and r+0.05;
      bottleneck 0.325; disc fits in 53.7 %. The straight line is disc-free along its whole length, with 0.93 m of it
      under the table.
    - Section: runs 62 bins, longest 1.24 m over s 0.50–1.74; overhang underside 0.665, top 0.867.
    - Cylinder diagnostic (same window): 32,369 supports; the line is blocked for 1.74 m.
  - Probe (showcase_run logic): 368-support probe window, 1.7 s, projected 0.0006 h, no abort.
    - The probe's own query returned INVALID_GEOMETRY. Its fixed start/goal at the probe-window sides are not case
      poses; only its timing is used.
  - **Trial compile+query (selection aid):**
    - REACHABLE; verify certified, min clearance 0.0172 m; 1 slab; compile 2.0 s, query 3.5 s.
    - The path is the straight segment (4 vertices), 2.02 m long, 0.93 m of it under the table B overhang mask.
  - Figures opened:
    - `precheck_top1_trial_SW-p1-m0.5.png`:
      - left: almost empty sweeper map (a few floor blobs, wide halos), start and goal in one component, trial path
        straight under the orange table outline;
      - middle: tabletop mass 0.67–0.87 m over s 0.5–1.74 and nothing in the low band;
      - right: table B is a solid diagonal block in the cylinder map.
    - `region_SW.png`:
      - candidate lines fan across table B's outline at (−0.7…0.8, 10…12);
      - the SW bench (orange rectangle, 1.5–2.8 × 9.4–11.6) holds only a few sweeper shadows, but no bench line ranked
        above the table-B lines.
  - **Chosen:** `SW-p1-m0.5` (the trialled one), behaviour **under** (table B). Backup `SW-p7-m0.5` (goal 0.5 m
    further north, 478 supports, not trialled). Wrote `results/height/percase/sweeper/case.json`.
- 10:05 — Envelope: 0 `pc_` jobs queued, ledger 3 lines. Submitted **17777613** `pc_sweeper_run` (`ROBOT=sweeper`,
  budget 1, 8 CPU / 64 GB / 24 h) and appended it to the ledger. The trial suggests minutes (compile 2 s, query 3.5 s;
  replay3d unknown). Waiting with a background until-loop.
- 10:07 — **17777613 COMPLETED in 49 s. SUCCESS** (`results/height/percase/sweeper/sweeper.json`, log
  `gmc/logs/pc_sweeper_run-17777613.out`).
  - Projection: 477 supports (candidate2 151, floored 0).
  - Probe: 368 supports, 1.6 s, projected 0.0006 h, no abort.
  - **REACHABLE**; `verify` certified, min clearance 0.01716 m; 1 slab; compile 1.96 s, query 3.44 s. The numbers equal
    the search trial.
  - Path: rotation 0 → π in place at start, one straight translation (0.2625, 10.2625) → (0.0125, 12.2625), rotation
    back. The disc robot is unaffected by the rotations.
  - **replay3d passed:**
    - 313 samples, 3,300 pairs checked, 104 refined, no collisions;
    - lower bound 0.0268 m;
    - worst sample at (0.104, 11.530), under table B at s ≈ 1.28 (splat 4491937).
  - Raw GMC output: `/scratch/wg2381/splathjb/gmc/outputs/height/showcase/percase/sweeper/{path.json,result.json}`.
  - No retry needed; no further jobs submitted. Jobs of mine: 17777515 (search) and 17777613 (run), both in the ledger.
