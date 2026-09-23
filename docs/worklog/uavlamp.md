# UAV-under-lamp chain — worklog

## L1 Task 0 — planner knobs (read from code at base `cba1433`, no compute) — 2026-09-23

Source: `gmc/src/gmc/gs3d/planner.py`, `oracle.py`, `contracts.py`, `scene.py`,
`experiments/gs3d_integration.py`, `experiments/gs3d_core_fixtures.py`.

**Lattice.** `LatticePlanner._search` is A* on an integer lattice *anchored at the start pose*:
node `k ∈ Z³` sits at `start.xyz + resolution_m · k`. One resolution for x, y **and** z
(`PlannerConfig.resolution_m`; Codex's UAV used `0.10 m`). UAV (`motion == "uav_translation"`)
uses all 26 neighbours of `{-1,0,1}³ \ 0`; nothing fixes z. Cost = Euclidean edge length,
heuristic = Euclidean distance to goal minus goal tolerance (admissible). The goal is reached
when a closed node lies within tolerance (+1e-9 roundoff), or when a node within
`max(2·res, tol+res)` has a free straight edge to the exact goal. Before searching, the planner
tries the direct start→goal edge.

**Search box.** There is no separate box argument: the box is `SceneSpec.bounds_min/max`. A
lattice node is skipped if its centre is outside `[bounds_min + half + margin,
bounds_max − half − margin]` with `half = (r, r, h)`. Independently, every oracle call requires
the swept body AABB to be inside `SceneSpec.known_space` (else `map_unknown`) and farther than
`margin` from the bounds (else `workspace_margin_unproven`). So bounds are hard walls. Codex's
UAV box is the route-frame prism `[-.75,-1,.20]–[3,1,2.0]` m (floor-relative, route frame
origin `(9.0,5.75,z_floor)`, `u=(.6,.8,0)`), `RouteBoxKnownSpace` for coverage and its world
AABB as `bounds`.

**Budget.** `SearchBudget(max_wall_s=120, max_expansions=100 000, max_oracle_calls=1 000 000,
max_narrowphase_pairs=5 000 000)` (defaults = Codex's UAV setting). An *expansion* is one node
popped from the open list and closed; an *oracle call* is one pose or swept-edge check (≤26 per
expansion + goal connectors); narrowphase pairs are body–Gaussian exact tests. Hitting any
limit raises `BudgetExceeded` → `status="budget_exhausted"`, `reason=<limit name>`.
If instead the open list empties, `_search` returns `None` and the planner reports
`no_path_on_lattice / reachable_lattice_exhausted` — **unless** some edge during the search
was rejected as not-provably-free, in which case the same exhaustion is labelled
`verification_failed / reachable_frontier_has_unproven_edges` (or `map_unknown / ...` for
coverage). Both of these are *queue exhaustion*, not a budget stop; the distinction is the
exception path vs. the empty queue. `diagnostics.{expansions, visited_nodes,
occupied_rejections, unproven_rejections, map_unknown_rejections}` record which.
Consequence for the necessity proof (Task 3.2): near real Gaussians some edges will be
"unproven" (lower bound ≤ margin without overlap), so the honest expected label for "no path,
set exhausted" is either `no_path_on_lattice` or `verification_failed/
reachable_frontier_has_unproven_edges`; both mean every reachable lattice node was expanded.
Anything `budget_exhausted` does **not** count.

**Body and obstacles.** UAV body = upright cylinder, `r = 0.25 m`, half-height `h = 0.10 m`,
margin `0.05 m` (all edges and poses need a *proved* clearance lower bound > margin).
`PreparedScene` keeps Gaussians with `opacity > τ = 0.3`; each is the solid ellipsoid
`(x−μ)ᵀΣ⁻¹(x−μ) ≤ level²`, `level = 2.0` (2σ). Semiaxis `a` ↔ covariance eigenvalue `(a/2)²`.
`GaussianBodyOracle.edge` sweeps the cylinder linearly between two poses and tests every
candidate from a BVH with `cylinder_ellipsoid_bound`: overlap → `occupied`, bound ≤ margin →
`unknown/unproven` (fail closed), else free with that bound. Required free gap for the body to
pass *between* two obstacles: horizontally `2(r+margin) = 0.60 m`, vertically
`2(h+margin) = 0.30 m` (strictly more).

**F1 (synthetic under/over).** `gs3d_core_fixtures.under_over()`, tested by
`tests/unit/test_gs3d_core_planner.py::test_under_over_impossible_for_fixed_z` (res .2/.1/.05)
and rendered by `experiments/gs3d_core_smoke.py`. Scene box `[-2.2,-.5,0]–[2.2,.5,2.8]`: a
hanging Gaussian at `(-.7,0,2)` with semiaxes `(.35,2,1)` (spans z 1.0–3.0, i.e. through the box
top at 2.8) and a low one at `(.8,0,.3)` semiaxes `(.35,2,.95)`; start `(-1.8,0,.65)`, goal
`(1.8,0,1.65)`. **How it blocks the alternatives:** the box is only 1.0 m wide (node centres
|y| ≤ .20), and both Gaussians have lateral semiaxis 2 m ≫ corridor, so "beside" does not exist;
the hanging one reaches past the box ceiling, so "over" does not exist. The box faces play the
walls. That is the pattern to reproduce — but in the real hall the side walls and top seal must be
*scene geometry* (real wall, added partition, drop ceiling), not open-air box faces.

## L1 problems log

- Job 18322943 (T1) failed in 2 s: `/usr/bin/time` does not exist on Torch compute nodes. Removed;
  MaxRSS comes from `sacct`. Resubmitted.
- T1 run 1 (18323035): Q1a budget-stopped at 120 s after 78 expansions (~180 ms/oracle call,
  68 narrowphase pairs/call near real clutter, 204/654 edges unproven). Planner is slow in real
  clutter; any necessity/exhaustion proof must keep the reachable volume small and budget in
  hours. Q1b pre-registered start is inside a real pillar → moved to v=+.4. Resubmitted as
  18323468/18323469 with max_wall_s 5400.
- Site search from measurement rasters (`outputs/uavlamp/measure/*.png`): hall walls are
  full height to the ~5 m ceiling. Candidate A: table at world (14.1–15.3, 9.0–10.9) against the
  outer wall, but only ~1.5 m approach before a low round dais. Candidate B (chosen for
  measurement): gallery between two real walls, A (7.5,22)→(11.6,30.5) and B (10.3,22)→(15.9,32),
  ~2.8–3.0 m wide, with a real table/cabinet at (10.2–11.8, 26.8–29.6) against wall A and a
  ≥4 m free approach from the south. Job 18323467 fits the wall faces and the table.
- Site chosen: gallery between real walls A/B, real table against wall A. Frame origin moved
  0.45 m onto wall A's face (site job's 0.999 quantile was polluted by the table).
- Build 18323926 ok (sha 485df24d…). Render 18323927 looked at: reads as gallery + light box.
  Probes 18323928: all boundaries 0 free crossings; opening 85 free (control). Wall B sparse in
  the booth span, still closed.
- cpu_short rejects --time ≥ 8 h (6 h accepted): necessity budget capped at 20 700 s (job 18324321).
- Resumed 09:04Z. Q1a/Q1b run 2: both success, both UNDER Codex's lamp (Q1b falsifies my "over").
  v1 T3: M1/M5/C4/B6/B6h budget-stopped at 7200 s (~3.6 s/expansion, 19 GJK it/pair, 1/3 edges
  unproven near wall A/table); C4h success OVER the lamp; N2 queue exhausted but labelled map_unknown
  because v1 box v-faces 5 cm inside walls + rotated-AABB coverage check precedes occupancy;
  P3c/d probes mis-placed (goal body outside box). v2: box faces behind surfaces, face-logging
  coverage wrapper, nearer goal (.8,.55,1.5), start u=-2.0, 20 000 s budgets. Jobs 18329380-90.
- v2 results 11:15Z: M1/M5 success under the lamp (replay passed); N2 exhausted, unknown only at u_min (3648); B6/B6h identical routes; C4 same as M1 (low start = geodesic), C4h over. Route overlay render 18331738.

## L2 log

- 11:22Z L2 start. Review of L1 by artefacts: archive SHA-256 `2a3a72d6…96cc` recomputed = manifest;
  manifest copy in git identical; looked at `v2_M1_side`, `v2_N2_top`, `booth_v2_routes_cutaway_side`,
  `booth_v2_entrance` — consistent with L1's claims. `uavlamp_query.run_one` calls
  `LatticePlanner(prepared).plan(scene, UAV, start, GoalRegion(goal), config)`: start + goal + box
  (SceneSpec bounds/known space) + resolution/margin/budget only. Note: L1's runner loads with the
  plain `np.load` + spec hash check, not the manifest loader; L2's `uavlamp_run.py` uses
  `load_uavlamp_derivative`.
- Timing constraint: one M1 call ≈ 1.6 h, so cold + 3 warm sequentially ≈ 6.5 h > cpu_short's
  usable limit. Resolution: one job, one timed index preparation, then 4 forked processes (1 cold
  with unprepared planner, 3 warm with the prepared index), one CPU each, same node. Recorded as
  concurrent in the result JSON.
- First synthetic slice for the unit test was infeasible (lamp→table gap .45 m < body .6 m) and ran
  into the 120 s budget; lengthened the corridor. 9/9 new tests pass locally (71 s).
- 11:38Z submitted 18332115 (M1 cold+3 warm), 18332116 (M5), 18332117 (N2), 18332118 (N2h: plug,
  high start — so the necessity figure has one start/goal for all three panels), 18332119 (C4h),
  18332120 (pytest).
- 11:38Z C4h (18332119) done in 10 s: lamp-only scene, high start → direct edge OVER the lamp
  (z 1.50–1.55, replay passed) — reproduces L1. pytest 18332120: 98 passed / 0 failed (181 s;
  run on the uncommitted working tree containing uavlamp_run.py + test_uavlamp_run.py).
- Viz dry runs on L1's result files (18332187, 18332276; output in ignored `outputs/uavlamp/l2_dev`):
  first side view too wide with overlapping labels, oblique from behind the start had the bulkhead
  hiding the route → side camera moved in, keyframes spread (start/approach/under/climb/over/goal),
  oblique moved to the table end. Video frames checked (camera not behind a wall). ~1 s per frame.
- 11:47Z queued 18332346 (smoothing, afterok M1) and 18332347 (final viz, afterok M1/M5/N2/N2h).
- 14:07Z resumed. All jobs COMPLETED: M1 1h42 (4 calls 6100–6122 s, identical knots = L1), M5 2h16,
  N2 2h19, N2h 2h20 (both queue-exhausted, 5330 exp, unknown only at u_min), smoothing 33 s
  (selected, continuous re-verification passed), viz 6 min. MaxRSS 1.4–2.9 GB.
- Bug found by looking at the frames: after adding the "approach" keyframe, the flythrough still used
  idx[1] as "under the lamp", so the extracted frame showed u=-1.62 (approach). Fixed (lookup by label),
  re-rendered (18342635); frame 134 now at u=-0.83, z=.65 under the light box. All four frames looked at.
- Committed 5802592 (Task 1) and 3c5591d (Task 2). Report docs/uavlamp_report.md (Task 3).
