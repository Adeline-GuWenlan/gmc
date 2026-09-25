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
3. `R = Q ∩ S` is a convex polygon (`Region`, `ground5k_common.py:116`):
   - **A\*:** known space and support evidence. Scene bounds are `Q × z`.
   - **GMC:** workspace = `R ⊖ square(r + 0.001)`.
4. **Gaussians:** opacity > τ and level-2 ellipsoid AABB overlapping `Q × [z_floor − 0.10,
   z_floor + 2.00]`. This is `crop_by_support_aabb`, the conservative crop rule of the architecture
   record.
   - Anything that can touch a body inside `R` is kept: bodies reach at most `z_floor + 1.75`.
   - GMC then applies its own band test to the same set.
5. **Floor:** constant `z_floor = −1.2271749593` (the GMC band origin and the gs3d A5 manifold). The
   plane-floor tiles top out at `z_floor + 0.015 m` < chassis bottom `z_floor + 0.02 m`.

The witnesses are certified paths for both robots and lie inside `R`. So a planner that fails on a
pair has failed where a certified route exists inside its own query region.

### 1.6 Per-query caps and resumability

- Each (pair, robot, planner) runs in a forked child under `watchdog` (`ground5k_run.py:55`).
- **Wall cap:** 1800 s over crop + preparation or projection + compile + query + certification, with
  30 s grace.
- **Memory cap:** 12.5 GB child RSS (tasks request 16 GB).
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
- **A\* and GMC settings:** as above.

No parameter differs between pairs, robots or planners, except the robot bodies.

## 2. Pilot and projection

(Filled in after the pilot; see below.)

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
The **north wing (y ≥ 25, 17 % of the known floor) holds only 0.6 % of endpoints**, for two reasons
in the edited map:

1. **Most of its known floor is not free for either robot** (grey in the components figure):
   - y 25–30: ≈ 45 % of known-floor lattice nodes certified free for the sweeper, ≈ 31 % for the
     cylinder.
   - y 30–38: 16 % and 8 %.
   - For comparison, y 10–15: 67 % and 54 %.

   This is the round-table cluster, where P1 already reported 28.5 m² of residual near-floor
   over-approximation that the plane-floor edit did not remove. Near-floor splats there overlap even
   the sweeper's chassis at +0.02 m.
2. **What is free forms separate components** (sweeper 332 nodes, cylinder 207). The certified 0.25 m
   lattice does not connect them to the main hall. Pairs are independent uniform draws, so a pair
   needs *both* endpoints in such an island, which has probability ≈ (area share)².

**Biases, stated for the report.** The accepted pairs are uniform over pairs of known,
certified-free floor points ≥ 3 m apart that a coarse certified lattice connects for **both**
robots. This excludes:
- pairs linked only through gaps narrower than the 0.25 m lattice can thread;
- pairs linked through unobserved floor or across the contact-strip limit;
- in effect, the north wing and the SW / E contact-limited strips.

It therefore favours open, well-connected floor, and it removes the pairs where the question is
connectivity itself. A planner failure on an admitted pair is a failure on a pair that a certified
route inside the planner's own query region connects.
