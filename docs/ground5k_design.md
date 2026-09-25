# Ground 5000-pair benchmark: design (stage G1)

Scope: the user's ask of 2026-09-24, as recorded in `claude_jobs/ground5k/prompts/_rules.md`. Pairs
are drawn uniformly over the whole real gallery hall (`scene_v2`, the uav-lamp derivative with the
booth). Each pair has at least 3 m between endpoints, and the same start and goal are used for the
sweeper and the cylinder. Two planners run on every pair:
- the **GMC certified planner** (the method under study);
- the **gs3d lattice A\*** (the baseline).

Failures are the focus. Paths are relative to `gmc/` unless they start with `docs/`.

Frozen parameters: `configs/ground5k.json`. They were committed before any planner run.

## 1. The two planners, pinned (Task 0)

### 1.1 GMC on a ground robot

The plane-floor chain ran GMC for ground robots through `experiments/showcase_run.py:31` (`main`).
This benchmark calls the same functions in the same order; only the per-query wall cap is new.

| step | code | what it does |
|---|---|---|
| robot | `src/gmc/height/prism.py:32` `robot_table()` | Vertical prisms. Sweeper: disc r 0.175, band floor + [0.02, 0.10]. Cylinder: disc r 0.30, band floor + [0.02, 1.75]. |
| projection | `src/gmc/height/project.py:15` `project_scene` | τ = 0.3 opacity cut. Band-overlap filter, then outer shadow ellipses at level 2 over the window ± r, then dedup. Result: `SceneModel2D` supports. |
| compile | `src/gmc/height/run.py:55,61,65`: `query_candidate_pairs` → `build_slabs` → `compile_mobility` | Certified pair oracles, orientation slabs, and the SAFE / POSSIBLE mobility graphs. No wall check inside. |
| query | `src/gmc/mobility/query.py:268` `query(start, goal, mc)` | SAFE path → verify → local refine → retry. Budget from `mc.cfg.query` (`:278`). |
| certification | `src/gmc/verification/path.py` `verify_curve` (called at `run.py:78`) | Independent continuous check of the returned `PoseCurve` on the projected map, `eps_clear` 2 mm. |
| legacy replay | `src/gmc/height/replay3d.py:80` `replay_curve` | Sampled prism-vs-ellipsoid 3D replay. **Not** used here: the shared gs3d replay (§1.4) replaces it. |

- **Inputs:** a `GaussianScene3D`; a window (the robot-*centre* workspace — `slab_builder.py:393`
  subtracts C-obstacles from it); `z_floor`; start and goal `Pose2 (x, y, θ)`.
- **Budget:** `configs/height_showcase.yaml`:
  - `max_support_calls` 50,000,000;
  - `max_wall_seconds` 7,200 (query only; compile is unbounded);
  - `eps_clear` 2e-3, `max_refinement_rounds` 8;
  - `initial_intervals` 1, `theta_min` 5e-3, `max_depth` 14;
  - `certificate_mode: prototype`.
- **Status vocabulary:** `src/gmc/types.py:26` `PlanStatus`:
  - `REACHABLE` (`query.py:594`, only after `verify_curve` certified);
  - `UNREACHABLE` (`:711`, **theorem mode only**, `:682`);
  - `UNKNOWN`, with these `report["reason"]` values:
    - `query_wall_budget_exhausted` `:310`
    - `query_support_budget_exhausted` `:512`
    - `possible_cut_is_not_a_global_certificate` `:725`
    - `safe_graph_ambiguous` `:749`
    - `finite_possible_graph_pose_coverage_missing` `:806`
    - `global_possible_cover_status_missing` `:674`
  - `INVALID_GEOMETRY` (`invalid_start_or_goal_pose` `:793`);
  - `INTERNAL_ERROR` (e.g. `certified_safe_edge_failed_verification` `:617`, `formal_cut_invariant_failed` `:704`).
- **Consequence of prototype mode:** GMC here **cannot** return `UNREACHABLE`. Its "search exhausted"
  answer is `UNKNOWN / possible_cut_is_not_a_global_certificate`: start and goal lie in different
  components of the POSSIBLE (outer) graph.

**How this benchmark calls it** (`experiments/ground5k_run.py:310` `_run_gmc`). The body is
`compile_and_query`'s two halves, call for call, as in `experiments/plane_timing.py:190,216`. Two
things differ:
- **Workspace.** The window is the query region `R` eroded by the robot's `r + 0.001` m square
  (§1.5). GMC's body therefore stays inside `R`, which is exactly where A\*'s body stays.
- **Band.** GMC projects with its robot band inflated by the shared margin, 0.001 m (sweeper
  floor + [0.019, 0.101], cylinder floor + [0.019, 1.751]). Its band test drops any splat lying wholly
  outside the band, however close. Without the inflation, GMC certified paths that pass 0.2–0.8 mm
  above floor splats, which the shared replay and A\* (margin 1 mm in every direction) reject. The
  pilot found this (§2.1). Horizontally, GMC's `eps_clear` 2 mm already exceeds the margin.
- **Wall budget.** GMC's own query deadline is set to the time left in the 1800 s cap
  (`ground5k_run.py:357`). This writes `max_wall_seconds` in place on the same config object, because
  the decomposition binding checks config *identity*: `orientation/provenance.py:50`. Compile has no
  internal deadline, so the watchdog (§1.6) enforces the cap there.

Start and goal yaw are 0; both bodies are discs.

### 1.2 gs3d lattice A\* on `ground_unicycle`

| step | code | what it does |
|---|---|---|
| bodies | `src/gmc/gs3d/robots.py:21,23` | `SWEEPER` (r 0.175, half-height 0.04, clearance 0.02) and `CYLINDER` (r 0.30, half-height 0.865, clearance 0.02). Physically the same prisms as GMC's. |
| scene | `src/gmc/gs3d/contracts.py:65` `SceneSpec` | Gaussians, bounds, τ, level, `KnownSpace`, `SupportSurface`. |
| crop | `robots.py:156` `crop_by_support_aabb` | Full 3D ellipsoid AABB overlap at level 2, opacity > τ. |
| support | `robots.py:66` `EvidenceBoundedPlaneSupport` | Built the way `experiments/gs3d_run.py:130` `_constant_floor_support` builds it. Constant plane `z_floor`; the constructor refuses a fitted-plane mismatch > 0.05 m. |
| index | `oracle.py:65` `PreparedScene` | Covariance validation plus a support-AABB BVH. |
| entry | `src/gmc/gs3d/planner.py:81` `LatticePlanner.plan(scene, body, start, GoalRegion, PlannerConfig)` | One call, start and goal only. |
| search | `planner.py:198` `_search` | 8-neighbour xy lattice anchored at the start. Every edge goes through `GaussianBodyOracle.edge` (`oracle.py:147`), a continuous swept-body bound. |
| certification | `planner.py:177-185` | `verify_path` + `verify_linear_trajectory` on the exported trajectory. |

- **Budget:** `SearchBudget` (`contracts.py:49`), with limits enforced at `oracle.py:33-46` and
  `planner.py:218`. Frozen here: `max_wall_s` = time left in the 1800 s cap, 500,000 expansions,
  5,000,000 oracle calls, 20,000,000 narrowphase pairs. Those counts are the "real diagnostic ceiling"
  of `experiments/gs3d_integration.py:244`.
- **Other settings:**
  - resolution 0.20 m and margin 0.001 m, the ground runs of `gs3d_integration.py:198,243`;
  - exact goal (tolerance 0), yaw tolerance 0.05;
  - no refinement retry.
- **Status vocabulary:** `contracts.py:20`:
  - `success` (`planner.py:191`);
  - `budget_exhausted` (`:193`, reason `max_wall_s|max_expansions|max_oracle_calls|max_narrowphase_pairs`);
  - `map_unknown` (`:152,155,171`);
  - `verification_failed` (`:173` frontier has unproven edges; `:185` post-build);
  - `no_path_on_lattice` (`:174`);
  - `start_invalid` / `goal_invalid` (`:152,155`);
  - `invalid_input` (`:195`).
- **Order of precedence** when the search exhausts: `map_unknown` if any map-unknown rejection, else
  `verification_failed` if any unproven rejection, else `no_path_on_lattice`.

**How this benchmark calls it:** `ground5k_run.py:270` `_run_astar`.
- `PreparedScene` is built before `plan`, so it counts as preparation, but it is still inside the
  1800 s cap.
- An **observation-only** subclass of `GaussianBodyOracle` is swapped into the planner module for
  that one call (`ground5k_run.py:247`). It records:
  - every non-free report's reason;
  - the expanded pose closest to the goal (where a failed search stopped).

  Every answer is `super().edge(...)` unchanged. `tests/unit/test_ground5k_run.py` runs the full
  plan with it.

### 1.3 One outcome set

| outcome | GMC | A\* |
|---|---|---|
| `SUCCESS_VERIFIED` | `REACHABLE` with `verify_curve` certified, **and** the shared replay passes | `success`, **and** the shared replay passes |
| `FAIL_NO_PATH` | `UNREACHABLE`; `UNKNOWN / possible_cut_is_not_a_global_certificate` (the outer graph separates start and goal; uncertified only because of prototype mode) | `no_path_on_lattice`; `map_unknown` / `verification_failed` when every unknown or unproven rejection came from the query region's **box faces** (the crop boundary, a hard wall for both planners) |
| `FAIL_UNKNOWN` | all other `UNKNOWN` reasons; `INVALID_GEOMETRY` (an endpoint blocked on GMC's projected map, though the 3D oracle certified it free); `REACHABLE` whose `verify_curve` did not certify | the search touched the **contact-strip faces** (support-evidence limit); any `geometry_or_margin_unproven` edge; `start_invalid` / `goal_invalid`; `post_build_verification_failed` |
| `FAIL_BUDGET` | `UNKNOWN / query_wall_budget_exhausted` or `query_support_budget_exhausted`; watchdog kill (wall or memory cap; compile included); system SIGKILL (likely OOM) | `budget_exhausted` (any cap); watchdog kill |
| `FAIL_REPLAY` | planner success, shared replay rejects | same |
| `ERROR` | exception; `INTERNAL_ERROR`; replay not completed | exception; `invalid_input` |

Code: `classify_gmc` `ground5k_run.py:161`, `classify_astar` `:136`, `with_replay` `:181`. Each
record also keeps:
- the planner's verbatim status and reason;
- the fine `cause`;
- A\*'s rejection reasons, counted per oracle reason and per region face.

G2 can therefore re-slice any row.

### 1.4 The shared replay

Both planners' paths go through `src/gmc/gs3d/trajectory.py:91` `replay_plan`:
- `verify_path` (`validation.py:34`): every closed segment is rechecked by the swept-body oracle,
  unicycle-consistency is enforced, and the goal must be reached;
- `verify_linear_trajectory`: speed and yaw-rate limits.

Settings: margin 0.001 m, τ 0.3, level 2, the same body, a **fresh `PreparedScene`** of the same
query `SceneSpec`.
- **A\*:** its own `gs3d.v1` result is replayed as-is.
- **GMC:** the `PoseCurve` knots become an xy polyline (`gmc_poses`, `:189`), then gs3d's own
  `_linear_trajectory`: rotate in place, then translate along the heading. For a disc this sweeps the
  same set as GMC's (x, y, θ)-linear segments.

A path must also start exactly at the start (≤ 1e-9 m). The replay runs after the capped phase and
has its own 1800 s timeout.

### 1.5 The per-query crop (identical for both planners, all four combinations)

One region per **pair**, shared by both robots and both planners (`ground5k_common.py`):
1. `Q` = bounding box of: start, goal, the sweeper witness and the cylinder witness (§3). It is
   padded by 1.0 m and clipped to the scene extent. It is stored in the pair file *before* any
   planner run.
2. `S` = contact strip, where `|fitted floor − z_floor| ≤ 0.05 m`. The fitted plane is tilted 0.285°,
   so this excludes the SW corner (x ≲ 0) and an east strip (x ≳ 18, y 15–25). Neither planner may
   put its body where the gs3d support rule would refuse the floor.
3. `R` = `Q ∩ S` as an **axis-aligned staircase**: 0.10 m y-bands, each with the x-range the strip
   allows over the whole band (`StairRegion`, `ground5k_common.py`). It replaced the exact convex
   cut after the pilot (§2.1). A box the strip does not cut stays exactly `Q`.
   - **A\*:** known space and support evidence. Scene bounds are `Q × z`.
   - **GMC:** workspace = `R ⊖ square(r + 0.001)`, a rectilinear polygon.
4. **Gaussians:** opacity > τ and level-2 ellipsoid AABB overlapping `Q × [z_floor − 0.10,
   z_floor + 2.00]`. This is `crop_by_support_aabb`, the conservative crop rule of the architecture
   record.
   - Anything that can touch a body inside `R` is kept: bodies reach at most `z_floor + 1.75`.
   - GMC then applies its own band test to the same set.
5. **Floor:** constant `z_floor = −1.2271749593` (the GMC band origin and the gs3d A5 manifold). The
   plane-floor tiles top out at `z_floor + 0.015 m` < chassis bottom `z_floor + 0.02 m`.

The witnesses are certified paths for both robots and lie inside `R`, except five cylinder witnesses
that touch the staircase's ≤ 1.2 cm edge slivers (§2.1). So a planner that fails on a pair has failed
where a certified route exists inside its own query region.

### 1.6 Per-query caps and resumability

- Each (pair, robot, planner) runs in a forked child under `watchdog` (`ground5k_run.py:55`).
- **Wall cap:** 1800 s over crop + preparation or projection + compile + query + certification, with
  30 s grace.
- **Memory cap:** child RSS ≤ min(12.5 GB, task memory − 1.5 GB). Task memory is pilot MaxRSS × 1.5
  (§2.3): 10.5 GB for GMC cylinder.
- A kill is `FAIL_BUDGET` with the stage it hit.
- **Output:** one JSON per query at `outputs/ground5k/<scene>/runs/<combo>/<pair_id>.json`, written
  atomically. A task skips every pair whose JSON already parses.

### 1.7 Frozen parameters (`configs/ground5k.json`)

- **Common:** τ 0.3, level 2.0, margin 0.001 m.
- **Robots:** as §1.1 / §1.2.
- **Scenes:** hash-pinned archives.
- **Contact rule:** 0.05 m / 0.05 m travel / 5°.
- **Crop:** pad 1.0 m, z −0.10 … +2.00 m.
- **Sampler:**
  - seed 20260924, 5000 pairs, ≥ 3 m separation, 0.25 m witness lattice;
  - floor evidence: 0.10 m cells; level-2 footprints of opaque source splats within −0.30 … +0.10 m
    of the fitted floor; splats with half-extent > 0.5 m ignored.
- **Per query:** 1800 s, 30 s grace, 12.5 GB.
- **A\* and GMC settings:** as above, plus the GMC band inflation `gmc.band_inflation_m` = 0.001.

No parameter differs between pairs, robots or planners, except the robot bodies.

## 2. Pilot and projection (Task 2)

**Setup.** The pilot covered the first 50 pairs of the committed list (`v2-00000` … `v2-00049`), all
four combinations, one pair per array task, with 1 CPU per task.
- Arrays (all in `claude_jobs/ground5k/jobids/G1.txt`):
  - A\*: 18500830, 18500831;
  - GMC: 18501688 + 18501874, 18501689;
  - reruns of the 7 strip-cut pairs: 18511062–18511065.
- Summary: `gmc/experiments/ground5k_pilot.py` → `results/ground5k/pilot/summary_scene_v2.json`,
  `table_scene_v2.md`.

### 2.1 Runner bugs the pilot found (fixed before the numbers below; no planner parameter changed)

1. **Budget keyword.** A\* crashed on every call: the frozen config's descriptive
   `astar.budget.max_wall_s` string was forwarded next to the computed wall budget.
   - The tests now read the planner sections of the frozen config.
   - The 33 ERROR JSONs were deleted and the arrays rerun.
2. **Vertical margin for GMC (the band).** 5 of the first 18 GMC sweeper results were `FAIL_REPLAY`.
   - `ground5k_replay_diag.py` (job 18501088) showed each was one floor splat whose level-2 top sits
     0.2–0.8 mm below the sweeper chassis. There was no collision, but the gap is inside the shared
     1 mm margin.
   - GMC's band test ignores a splat lying wholly outside the band, so GMC had been given no vertical
     margin.
   - Fix: GMC projects with its band inflated by the margin (§1.1).
   - Superseded outputs: `outputs/ground5k/scene_v2/superseded_pilot_gmc_band/`.
3. **Slanted workspace edge.** GMC `INTERNAL_ERROR / I3_graph_nesting` occurred on 5 of 7
   strip-cut regions and 0 of 43 boxes (sweeper).
   - Cause: the invariant compares SAFE and POSSIBLE overlays by exact set difference, and a slanted
     workspace edge leaves slivers.
   - Fix: the query region is an axis-aligned staircase inside the strip (§1.5, `StairRegion`), for
     both planners and the replay. Uncut boxes are unchanged.
   - The 7 cut pairs were rerun for all four combinations. The sweeper errors went: pairs 22, 28, 33
     and 36 now certify; 16 ended FAIL_BUDGET.
   - Superseded outputs: `superseded_pilot_stair/`.
   - Side effect: for 5 of the 5,000 pairs a cylinder witness touches the ≤ 1.2 cm sliver the
     staircase gives up (v2-01136, v2-01413, v2-02749, v2-02860, v2-03906). They are flagged in the
     handoff.

The runner also records the rejected replay edge (Gaussian ids, bound) and GMC's `failed_invariants`.

### 2.2 Pilot result (final code, 50 pairs)

| combination | done | SUCCESS | NO_PATH | UNKNOWN | BUDGET | REPLAY | ERROR | query wall median / p90 / max (s) | peak RSS p90 / max (GB) | CPU·h (pilot) | CPU·h / pair |
|---|---|---|---|---|---|---|---|---|---|---|---|
| gmc_sweeper | 50 | 44 | 0 | 2 | 4 | 0 | 0 | 122 / 1,640 / 1,830 | 2.51 / 5.20 | 5.05 | 0.101 |
| gmc_cylinder | 50 | 6 | 0 | 0 | 44 | 0 | 0 | 1,830 / 1,830 / 1,831 | 6.47 / 7.33 | 25.73 | 0.515 |
| astar_sweeper | 50 | 50 | 0 | 0 | 0 | 0 | 0 | 5 / 10 / 16 | 1.30 / 1.30 | 0.19 | 0.004 |
| astar_cylinder | 50 | 50 | 0 | 0 | 0 | 0 | 0 | 27 / 51 / 143 | 1.30 / 1.30 | 0.58 | 0.012 |

**Causes.**
- GMC sweeper:
  - 44 `gmc_reachable_certified`;
  - 4 `wall_cap_kill`, killed while in `query`;
  - 2 `gmc_safe_graph_ambiguous`.
- GMC cylinder:
  - 32 `wall_cap_kill` (26 while in `query`, 6 in `compile_slabs` on 387 k–672 k-support maps);
  - 11 `gmc_query_support_budget_exhausted` (maps of 30 k–313 k supports);
  - 1 `gmc_wall_cap_before_query`;
  - 6 certified.
- A\*: 100 of 100 `astar_success`. Every replay passed; there are 0 `FAIL_REPLAY` and 0 `ERROR` in
  the final pilot.

**GMC cylinder by projected map size** (GMC supports after projection):

| map size | queries | certified |
|---|---|---|
| < 50 k | 6 | 4 |
| 50–100 k | 6 | 2 |
| 100–200 k | 12 | 0 |
| 200–400 k | 19 | 0 |
| ≥ 400 k | 7 | 0 |

- The median map is 214 k supports.
- The largest certified map, 96,537 supports (v2-00046, 1,368 s), is **above** P3's 46,835-support
  ceiling. P3 had a 7200 s query cap and a smaller window, so the two are not directly comparable.

**Path length where both certify** (GMC / A\*): sweeper median 0.963 (p10 0.944, p90 1.000,
n = 44); cylinder median 0.965 (n = 6). GMC's any-angle paths are about 4 % shorter than the 0.20 m
8-neighbour lattice's.

**Time and memory.** A\* takes 5 s (sweeper) and 27 s (cylinder) median per query.
- GMC sweeper median 122 s; GMC cylinder hits the 1,830 s kill in 33 of 50 queries.
- sacct MaxRSS: A\* ≤ 2.2 GB, GMC sweeper ≤ 5.2 GB, GMC cylinder median 4.7 / p90 6.6 / max 7.4 GB.

### 2.3 Projection and the prefix

Cost per pair per combination is the larger of:
- sacct CPU time per pair in the pilot, including the scene load in one-pair tasks and the reruns (conservative);
- mean query wall plus the load amortised over the full-run task size.

| combination | CPU·h / pair | 5,000 pairs | task layout | memory |
|---|---|---|---|---|
| A\* sweeper | 0.0037 | 19 | 250 pairs/task, 20 tasks, %1 | 6 GB |
| A\* cylinder | 0.0116 | 58 | 125 pairs/task, 40 tasks, %1 | 6 GB |
| GMC sweeper | 0.101 | 505 | 25 pairs/task, 200 tasks, %2 | 8 GB |
| GMC cylinder | 0.515 | 2,573 | 9 pairs/task, 556 tasks, %8 | 12 GB |
| **all four** | **0.631** | **3,154** | | |

**Budget.**
- 8,000 CPU·h, minus 42 CPU·h used by G1 so far (agents included), minus a 300 CPU·h G2 reserve,
  leaves 7,658 CPU·h.
- **The budget allows the full list: prefix = 5,000 pairs** on scene_v2, the same prefix for all four
  combinations.

**Memory.**
- Tasks request pilot MaxRSS × 1.5, rounded up.
- The child cap is task memory − 1.5 GB, i.e. 10.5 GB for GMC cylinder.
- No pilot query exceeded 7.4 GB, so no pilot outcome would change.

**Wall-clock is bound by memory, not by CPU·h.** The cpu_short QOS allows 32 CPUs and 120 GB per user
at once (6 h wall per job), shared with the other chain and the agent jobs.
- scene_v2 needs about 30,900 GB·h for GMC cylinder plus 4,500 GB·h for the rest.
- Against about 108 GB usable, that is **about 13.6 days**. CPU-bound it would be about 4.1 days.
- The throttles (%8, %2, %1, %1) give each combination roughly its share of the GB·h, so they
  finish together.

**Plane floor (priority 2).** The budget would allow its full 5,000 pairs as well (about 3,150 CPU·h
more, total about 6,650 of 8,000). But that would add about 13.6 more days before G2 can report, and
the user put scene_v2 first "如果时间不够的话" (if time is short).
- G1 therefore queues a **1,000-pair prefix** of the plane-floor list (`pf-00000` … `pf-00999`,
  about 631 CPU·h, about 2.7 days) behind scene_v2 with `--dependency=afterany`.
- It is an unbiased prefix. G2 or the operator can extend it: the runner resumes and skips finished
  pairs.

### 2.4 Pre-registered expectations for the full scene_v2 run (from the 50-pair pilot, 95 % Wilson)

1. **A\* success** ≥ 93 % for both robots (pilot 50/50; interval 0.93–1.00). Its failures, if any, are
   `FAIL_UNKNOWN` (unproven near-contact edges) or `FAIL_NO_PATH` in narrow passages below the 0.20 m
   lattice. `FAIL_BUDGET` stays below 1 %.
2. **GMC sweeper success** 76–94 % (pilot 44/50). Its failures are mainly `FAIL_BUDGET` (query past the
   1800 s cap, pilot 8 %) and `FAIL_UNKNOWN / safe_graph_ambiguous` (pilot 4 %). No `FAIL_NO_PATH`
   (possible-graph cut) is expected, because every pair has a certified witness inside its region.
3. **GMC cylinder success** 6–24 % (pilot 6/50), and it is budget-limited.
   - About 88 % end `FAIL_BUDGET`.
   - Success falls with the projected map size. No certification is expected above about 150 k
     supports.
   - The certified maps extend past P3's 47 k.
   - GMC cylinder failures here say "certification did not finish within 1800 s", not "no path":
     A\* finds and replays a path on every pilot pair.
4. **Zero `FAIL_REPLAY`** for both planners. After the band fix any replay rejection is a safety bug
   and G2 lists every one.
5. **Where both certify**, GMC paths are 3–6 % shorter than A\*'s (lattice discretisation).
6. **GMC ERROR (`I3_graph_nesting`)** below 2 % after the staircase fix. It was 0 of 100 in the final
   pilot.

## 3. The pair sampler (Task 1)

**Code.** `gmc/experiments/ground5k_sample.py`. Steps: `evidence` → `lattice` → `sample` → `plot`, all
via `gmc/hpc/ground5k/sample.sbatch`. Tests: `gmc/tests/unit/test_ground5k_sample.py` (17). The tests
use a synthetic two-room floor with a door and an unobserved strip. They cover:
- uniform endpoint draws (KS test);
- the 3 m separation;
- filters (a), (b) and (c);
- that the rejection counts add up;
- seed determinism, the shuffle, and that the crop holds the witnesses.

**Output.**
- `gmc/outputs/ground5k/scene_v2/pairs_scene_v2.json`: 7.9 MB, SHA-256 `f598ee35…7115`.
- Committed as `gmc/results/ground5k/pairs_scene_v2.json.gz` (`gzip -n`, 0.95 MB) plus
  `results/ground5k/sampler/pairs_scene_v2_header.json`.
- Each pair records:
  - start and goal;
  - separation;
  - support z and fitted-floor z at both endpoints;
  - the known-region check;
  - the clearance lower bound of both endpoints for both robots;
  - both robots' witness component id (`<robot>:lattice0.25:comp<id>`), node count and length;
  - the crop box and region polygon (§1.5).
- Witness polylines: `outputs/ground5k/scene_v2/witness_paths_scene_v2.npz`.

**Seed and order.** Seed 20260924. Pairs are i.i.d. draws, shuffled once and numbered `v2-00000` …
`v2-04999`. Any prefix is an unbiased sample.

### 3.1 Known floor and the witness (jobs 18499321, 18499463)

**Floor evidence.** Source: the original capture `processed.npz` (read-only, SHA-256 `c9318d…8ae`).
- A 0.10 m cell is observed floor if the level-2 xy support box of an opaque (τ 0.3) splat touches it,
  where the splat is centred within −0.30 … +0.10 m of the fitted floor.
- Result: 556.8 m² observed, of which 477.9 m² lie inside the contact strip. The scene extent
  rectangle is 995.8 m².
- The first, centre-count version (job 18487348) gave 401 m² with pinholes in open floor. It was
  replaced before any sampling.
- Figure: `results/ground5k/figs/floor_evidence_scene_v2.png`.

**Witness lattice** (0.25 m, 8-neighbour, every node pose and edge certified by the shared oracle,
margin 0.001):

| robot | known-floor nodes | certified free | edges free / tested | components (largest, 2nd) | wall |
|---|---|---|---|---|---|
| sweeper | 7,642 | 4,351 | 14,865 / 14,969 | 19 (3,957, 332) | 39 s |
| cylinder | 7,642 | 3,341 | 11,144 / 11,174 | 14 (2,921, 207) | 292 s |

Components: `results/ground5k/figs/witness_components_scene_v2.png`.

### 3.2 Rejection counts (job 18499463, 1,003 s for 5,000 pairs)

| stage | drawn | rejected | rate |
|---|---|---|---|
| endpoint draws, uniform over the 25.5 × 38.8 m extent | 67,652 | (a) not on observed floor inside the contact strip, for both robots' body boxes: **39,131** | 57.8 % |
| (b) endpoint pose certified free, both robots | 28,521 | sweeper **10,180**, cylinder **4,095** | 50.1 % |
| pairs of admitted endpoints | 7,123 | separation < 3 m: **509** | 7.1 % |
| (c) certified-lattice connected, both robots | 6,614 | sweeper disconnected **931**; cylinder unattached **13**, disconnected **670** | 24.4 % |
| accepted | **5,000** | | |

Filters (b) and (c) test the sweeper first; a draw failing both is counted under the sweeper.

**The pairs themselves.**
- Separation: median 9.0 m, maximum 23.4 m.
- Cylinder witness length / separation: median 1.19.
- Query region: median 63.9 m², p90 133.4 m², maximum 239.7 m².
- 572 regions are clipped by the contact strip.

### 3.3 Spread over the hall, and the bias the filters introduce

Figure: `results/ground5k/figs/pairs_scene_v2.png` (opened). The largest 5 × 5 m block holds 10.3 % of
endpoints. Per y-band (`results/ground5k/pairs_scene_v2_yband.json`):

| y band (m) | known floor m² (share) | endpoint share | sweeper / cylinder free lattice nodes |
|---|---|---|---|
| −1–5 | 76.0 (13.6 %) | 8.7 % | 435 / 316 |
| 5–10 | 90.1 (16.2 %) | 23.4 % | 837 / 680 |
| 10–15 | 105.5 (18.9 %) | 31.5 % | 1,138 / 919 |
| 15–20 | 104.1 (18.7 %) | 25.5 % | 936 / 761 |
| 20–25 | 87.4 (15.7 %) | 10.3 % | 576 / 392 |
| 25–30 | 40.9 (7.3 %) | 0.5 % | 297 / 204 |
| 30–38 | 53.0 (9.5 %) | 0.1 % | 132 / 69 |

The endpoints cover the south and central hall (y < 25) evenly, and every room there has endpoints.
The **north wing (y ≥ 25, 17 % of the known floor) holds only 0.6 % of endpoints.** I checked why
before accepting it, using `witness_components_scene_v2.png` and the plane-floor control (§3.4):

1. **The uav-lamp booth cuts the wing off from the main hall** (a manual edit in scene_v2).
   - The booth fills the corridor between walls A and B at world x 9.4–13.6, y 25.0–30.1.
   - Its back panel runs from the floor to 2.48 m at y ≈ 29–30. Its lamp box starts 1.07 m above the
     floor, so the cylinder cannot pass under it.
   - Free lattice nodes at y ≥ 25 that lie in the main hall's component:
     - scene_v2: sweeper 109 of 429, cylinder 6 of 273;
     - the same hall without the booth (plane floor): sweeper 393 of 437, cylinder 253 of 296.
2. **At y ≥ 30 most known floor is not free for either robot, in both scenes.**
   - scene_v2, y 30–38: 16 % of known-floor lattice nodes certified free for the sweeper, 8 % for the
     cylinder. For comparison, y 10–15: 67 % and 54 %.
   - This is the round-table cluster, where P1 reported residual near-floor over-approximation that
     the plane-floor edit did not remove. Near-floor splats there overlap even the sweeper's chassis
     at +0.02 m.
3. **Why so few endpoints land there.** Pairs are independent uniform draws. A north-wing endpoint in
   scene_v2 is admitted only if its partner is in the same small island, which has probability
   ≈ (area share)².

**Biases, stated for the report.** The accepted pairs are uniform over pairs of known,
certified-free floor points ≥ 3 m apart that a coarse certified lattice connects for **both**
robots. This excludes:
- pairs linked only through gaps narrower than the 0.25 m lattice can thread;
- pairs linked through unobserved floor or across the contact-strip limit;
- in effect, the booth-isolated north wing (scene_v2 only) and the SW / E contact-limited strips.

It therefore favours open, well-connected floor, and it removes the pairs where the question is
connectivity itself. A planner failure on an admitted pair is a failure on a pair that a certified
route inside the planner's own query region connects.

### 3.4 Plane-floor pair list (priority 2, job 18500683)

Same sampler, same evidence raster, seed 20260925, prefix `pf`. `outputs/ground5k/planefloor/`
(figure `results/ground5k/figs/pairs_planefloor.png`, opened):
- **Endpoint draws:** 55,813. (a) rejected 32,163; (b) sweeper 8,360, cylinder 3,422.
- **Pair draws:** 5,934. Separation < 3 m: 414. (c) sweeper disconnected 48; cylinder unattached 11,
  disconnected 461.
- **Accepted:** 5,000.

Without the booth, the north wing up to y ≈ 31 is populated; pair separations reach 29 m. It serves
as the control for §3.3. Whether its planner arrays run depends on the projection (§2).
