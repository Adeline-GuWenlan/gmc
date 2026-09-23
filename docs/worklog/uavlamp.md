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
