# Baseline adapters on the F4 benchmark (bl stage B1; B2 adds PNO and cust_fields)

Plan: `docs/baselines_f4_plan.md` (§3 fairness contract, §5 rows). Environments: `docs/baselines_envs.md` (B0).
Branch `baselines-f4`. Claims are **[E]** (evidence: the committed file named next to it) or **[G]** (guess / untested).
Code: `gmc/experiments/bl_harness.py` (harness, gmc-venv), `bl_worker.py` (method process), `bl_testmethods.py`
(the harness's own methods), `bl_splatnav.py`, `bl_foci.py`; tests `gmc/tests/unit/test_bl_harness.py`; sbatch
helpers `gmc/hpc/baselines/b1_{cpu,gpu}.sbatch`. Job IDs: `/scratch/wg2381/claude_jobs/baselines/jobids/B1.txt`.

## 1. Harness

### 1.1 What every method goes through

```
F4 / tuning pair ──► bl_harness (gmc-venv) ──JSON line──► bl_worker (method env, own process group, GPU)
                         │  120 s from outside (select on the pipe; on expiry killpg SIGKILL, next pair restarts the worker)
                         ◄──────────── {claimed, path (u, v[, z]), algorithm_wall_s, cpu_s, stages} ─────┘
                         ▼
     complete_path: exact start / goal appended if the method stops short (recorded) ──► GMC's exporter
     (api._gs3d_result + api._densify 0.20 m) ──► replay_plan(result, GaussianBodyOracle(C.prepared)) ──► §5 row
```

- **Pairs.** `load_pairs("f4")` = `results/aerial3dg/f4/pairs_confirmed_5000.json`; `load_pairs("tuning")` = plan §3.5
  (first 25 WWEST + 15 GAPW1 + 10 S of F3's `pilot_pairs.json`, disjoint from the 5000); or a committed id list:
  `results/baselines/pilot/pilot_pairs.json` (100) and `results/baselines/b1_sanity/sanity_pairs.json` (200), both
  stratified by region × F4 lateral-clearance band (`aerial3dg_fail3_f4.LAT_BANDS`), proportional, ≥ 1 per stratum,
  seeds 20261010 / 20261011 (`bl_harness sample`). The method's worker receives only `(start_uv, goal_uv)`; only the
  `astar_replay` sanity method is sent the pair record.
- **Judge scene.** `Judge(region, robot)` loads the persisted F3 compile after `bl_judge_check.verified_a3c` checked
  its SHA-256 against the sidecar, F3's handoff and F4's `compile_once.json`; `C.prepared` is the very object F5/J
  judged A* routes and GMC with. One `Judge` per task.
- **Scene export** (`bl_harness export`, job 19508155) — what a Gaussian-native method reads:
  `outputs/baselines/scene/<R>_<robot>.npz` (+ `.json` with SHA-256): the judge's own `PreparedScene.means/covs/ids`
  (opacity > τ = 0.3, covariances as the judge floors them), rotated into the plan frame (= the route frame:
  u, v, height above the floor; checked `known_frame_equals_plan_frame: true` for all six), plus τ, level (2.0), margin
  (0.001), body, z_c, the known-space prism. 390,716 (WWEST), 209,126 (GAPW1), 333,487 (S) Gaussians **[E]**
  (`outputs/.../scene/*.json`, log `bl_b1_export-19508155.out`).
- **Export = GMC's export, mirrored by calling it.** `Judge.export` builds the world polyline exactly as `api.query`
  does — interior vertices `frame.to_world([u, v, domain.ground_z])`, endpoints the exact
  `frame.to_world([u, v, clearance + half_height])` that `aerial3dg_batch.run_task` passes — then
  `api._gs3d_result(C, api._densify(P, QCONFIG.export_max_segment_m = 0.20), g_w, None)`: start yaw 0, turn in place
  to each heading at 1 rad/s, translate at 0.3 m/s, final turn to yaw 0, position tolerance 0, yaw tolerance 0.05,
  margin = the compile's 0.001. **Test:** GMC re-queried through the harness as a method (`gmc_requery`) gives a
  `gs3d.v1` trajectory equal to GMC's own export pose for pose and time for time, and the same `polyline_sha256` as
  GMC's re-judged F4 row (`test_gmc_reachable_route_requeried_is_success_and_export_is_byte_identical`, 3 S pairs)
  **[E]** (job 19508155, 13/13 tests pass).
- **Completion (§3.3).** `complete_path`: a method vertex within 1e-9 m of the pair's endpoint *is* the endpoint
  (frame round-off); otherwise a straight segment from the exact start / to the exact goal is added and judged like the
  rest; rows carry `prepended_start_segment`, `appended_goal_segment`, `start_gap_m`, `goal_gap_m`. The start
  prepend is our addition to §3.3 (SplatNav starts at an A* voxel centre, FOCI within 0.22 m of the start: both need
  it). A 3-column path is (u, v, z): z is dropped (the body is judged at z_c) and `max_abs_dz_m` recorded.
- **Judge** = `replay_plan` (§2), always in the harness's gmc-venv process. Outcome (§5):
  claimed ∧ passed → `SUCCESS`; geometry fails with `occupied` → `CLAIMED_COLLIDES`; geometry `unknown`/`invalid`
  (margin unproven, map_unknown, goal tolerance) → `CLAIMED_UNPROVEN`; geometry passes but kinematics/attained-goal
  fails → `CLAIMED_KINEMATICS`; method reports failure → `FAIL`; > 120 s → `TIMEOUT`; exception / worker death /
  unexportable path → `ERROR` (message + traceback kept); worker cannot start → `SETUP_FAIL`.
  `judge_wall_s` is recorded per row and never charged to the method. `run --judge defer` writes the method's exact
  path only and `bl_harness judge` fills the judge fields later on CPU (keeps GPU allocations off the CPU judge).
- **Timeout from outside.** The worker is a child process in its own session; the parent waits on the result pipe with
  `select` and a 120 s deadline, then `killpg(SIGKILL)` and records `TIMEOUT:query_exceeded_120s`; the next pair starts
  a fresh worker (which reloads the persisted setup). No signal or alarm runs inside the method process (F4's in-process
  SIGALRM surfaced as a `TypeError` inside numpy, 0a97784). The worker's own stdout (Python and C level: IPOPT) is
  redirected to its log, so method prints cannot corrupt the protocol (`test_method_prints_cannot_corrupt_the_protocol`).
- **Setup once (§3.4).** `bl_harness setup` builds the method's setup state from the scene export once per
  robot × region, in the method's env (`bl_worker --mode setup`), and persists it with a SHA-256 sidecar
  (`outputs/baselines/<method>/<config-hash>/<R>_<robot>.setup.pkl`; record `results/baselines/<method>/setup/...`).
  Each task's worker loads that artifact (hash checked, `load_s`) and instantiates the live planner (`instantiate_s`:
  e.g. SplatNav's voxel grid on the GPU, FOCI's CasADi/IPOPT problem); both are reported per worker start in the task
  summary's `setup_once_proof` (`setup_builds_in_this_task: 0`, `worker_starts`, `restarts`, same artifact SHA every
  start, same `setup_id` every row) and never charged to a query.
- **Rows** (§5): F4's fields where they apply (`index, pair_id, start_uv, goal_uv, dist_m, status, reason,
  algorithm_wall_s` (worker-measured around `plan`), `stages, cpu_s, outer_wall_s` (parent, send → receive),
  `polyline_sha256` (of the exported world polyline, `aerial3dg_run._sha`)) + `method, setup_id, claimed,
  claimed_reason, appended_goal_segment, prepended_start_segment, judge_passed, judge_reason, judge_geometry_reason,
  judge_kinematics_reason, judge_wall_s, path_length_m, vertices, route_polyline` (exact uv that was judged),
  `method_path_uv` (the method's raw output), `lateral_clearance_m, len_ratio, worker_generation, info`. Appended +
  fsync'd per pair; a killed task resumes. Task summaries carry host/commit (`aerial3dg_run.host`), setup id + SHA,
  setup-once proof, config + its SHA, judge-scene SHA, worker log path (`outputs/baselines/worker_logs/...`).

### 1.2 Harness checks
| check | result |
|---|---|
| unit tests, pure (sleep → TIMEOUT from outside + worker restart; raise → ERROR, worker survives; `os._exit` → ERROR + restart; completion; outcome map; artifact hash; protocol vs prints; resume) | 9/9 pass **[E]** (login node, and job 19508155) |
| heavy unit tests (GMC REACHABLE re-queried → SUCCESS with byte-identical export; straight line through an obstacle → CLAIMED_COLLIDES (`minkowski_interior_witness`); astar_replay → SUCCESS; sleep with inline judge → TIMEOUT) | 4/4 pass **[E]** (job 19508155, `BL_HEAVY=1`) |
| **astar_replay** on the tuning set, both robots | **100/100 SUCCESS** **[E]** (`results/baselines/b1_sanity/astar_replay/tuning/`, job 19508232) |
| **astar_replay** on the 200-pair F4 sample, both robots | **400/400 SUCCESS** **[E]** (`.../f4_sample200/`, job 19508232) |
| astar_replay on J's 4 rounding-failure pairs (F4X-00400/00692/01125, F4W-00211) | **8/8 SUCCESS** **[E]**: cylinder via the exact A* re-run (stored route `geometry_or_margin_unproven`, as in J), sweeper via the stored route (`.../rounding4/`, job 19508644) |

All 500 sample/tuning rows used the stored 0.1 mm route (none needed the re-run); no completion segment was needed
(the A* route starts and ends exactly at the endpoints).

## 2. SplatNav (Splat-Plan)

### 2.1 Design
`gmc/experiments/bl_splatnav.py`, env `splatnav`, GPU. The pristine clone (e996e22) is imported and called exactly as
`run_splatplan.py` does: `SplatPlan(gsplat, {radius, vmax, amax}, {lower_bound, upper_bound, resolution},
SplinePlanner(spline_deg, N_sec), device).generate_path(x0, xf)`; the nerfstudio loader is replaced by an object with
the fields of the repo's `DummyGSplatLoader` (B0's approach). Setup (persisted) = the Gaussian preparation below;
instantiate (per worker start) = `SplatPlan(...)`, i.e. the repo's voxel grid + collision set on the GPU.
Claimed = A* found a path and the Bézier QP was feasible (the repo's `feasible`); FAIL reasons `astar_no_path`,
`qp_infeasible`. Output = the evaluated Bézier positions (≈ 10 per polytope).

### 2.2 Body cover: a sphere through a linear change of coordinates
Splat-Plan's robot is a sphere. The plan's fallback — the sphere enclosing the cylinder — has radius
√((r+m)² + (h+m)²) = **0.917 m** for the cylinder (3.05 × its radius) and 0.181 m for the sweeper. Instead the planner
works in the plan frame recentred on the region box and squashed in z about the body centre,
x' = (u − u_c, v − v_c, ε (z − z_c)). Every collision test is invariant under an invertible linear map, Gaussians stay
Gaussians (μ' = Aμ, Σ' = AΣAᵀ), so Splat-Plan still reads Gaussians. In that space the body is a cylinder of radius
r + m and half-height ε (h + m), which Splat-Plan's sphere of radius R = √((r+m)² + (ε(h+m) + Δ)²) covers for any
centre within Δ = √2·vmax²/(2 amax) of the plane z' = 0 (Δ bounds how far the Bézier can leave the plane: its corridor
around each A* segment is the repo's local box of half-width vmax²/(2 amax), rotated randomly about the segment).
Mapped back, the cover is the ellipsoid with semi-axes (R, R, R/ε): laterally **R/r = 1.073 (cylinder) and 1.088
(sweeper)** at ε = 0.05 with the authors' vmax = amax = 0.1 **[E]** (`results/baselines/splatnav/setup/*/*.json`),
vertically very tall. The tall extent is harmless only after the obstacle cut: a Gaussian is kept iff its
judge-level (2σ) ellipsoid meets the body's slab [z_c − h − m, z_c + h + m] (exact: an ellipsoid's z-extent is
μ_z ± 2√Σ_zz) and lies within reach of the box. Dropping the others cannot make a path unsafe for the real body. A kept
Gaussian that pokes out of the slab is still seen over its whole height — the cover's conservatism; kept/extending:
WWEST 207,352 / 10,367, GAPW1 92,157 / 5,604, S 81,475 / 5,320 (cylinder) **[E]**. The voxel grid (A* init) is one
layer thick (z' ∈ [−R, R]); its xy extent is the known box shrunk by r + m + Δ (so the corridor cannot leave the
box); the harness projects the result onto z = z_c (`max_abs_dz_m`).

### 2.3 Obstacle contract
| parameter | judge | SplatNav | status |
|---|---|---|---|
| σ level | 2σ ellipsoid | `scales` = semi-axes | **matched**: scales = 2·√eig(Σ) |
| opacity | > τ = 0.3 | not used | matched by input (export holds only the judge's Gaussians) |
| margin | clearance > 0.001 | none | matched: added to r and h |
| body | vertical cylinder | sphere | cover above (ε-squash) |
| safety | exact continuous check | corridor polytopes from ellipsoid–sphere tests, Bézier in the polytopes | method's own |

### 2.4 Deviations from the paper's setup
- No nerfstudio / gsplat / Splat-Loc: Gaussians fed directly (plan §4; B0).
- The z-squash body cover and the slab cut (above); the plain enclosing sphere (ε = 1) is tuning candidate S0.
- Rotations: the repo's quaternion-to-matrix returns NaN for a zero vector part (B0 hazard); such Gaussians are
  rewritten as an equivalent 90° yaw with swapped scales (none occurred: `zero_vector_quaternions_rewritten: 0`), and
  all rotations/covariances are asserted finite.
- On an infeasible QP the repo dumps the polytopes to `infeasible.obj` (open3d + qhull) before re-raising; that dump
  raised `QhullError` on degenerate polytopes and masked the infeasibility (probe 19508258, F3S-00000). The dump is
  replaced by a no-op on the instance; control flow is the repo's (its bare `raise` follows → FAIL `qp_infeasible`).
- Per-query `torch.manual_seed(q_seed)`: the repo's corridor box draws a random axis (`torch.randn`).
- Endpoints: Splat-Plan starts/ends at A* voxel centres (≤ half a cell off); the harness adds the exact endpoints.

## 3. FOCI

### 3.1 Design
`gmc/experiments/bl_foci.py`, env `foci`, GPU (warp), code `ext_repos/foci_bl` = clone 79d8ddc +
`gmc/baselines/patches/foci_mumps.patch` (MA27 → MUMPS). `Planner(means, covs, robot_cov, num_control_points,
num_samples).plan(start, end)` as in `demos/stonehenge.py`; FOCI's own initial guess (its A* on a 0.25 m voxel grid of
Gaussian means + spline fit), never our A* route. Frame: plan frame recentred on the box, z shifted so z_c = 0.5.
Claimed = IPOPT success (`solver.stats()['success']`: `Solve_Succeeded` / `Solved_To_Acceptable_Level`); FAIL
`astar_no_path` (FOCI's `ValueError("No path found")`), `ipopt_<status>`. Output = the 40 (`num_samples`) curve samples.
Setup (persisted) = obstacle preparation; instantiate (per worker start) = `Planner(...)` (CasADi NLP + warp kernels).

### 3.2 Body cover
FOCI's robot is three body points (centre and ±0.2 m along the heading), each the Gaussian `robot_cov`. Cover:
robot_cov = diag(a², a², c²)/level², with (a, a, c) the minimum-volume ellipsoid enclosing the cylinder (r + m, h + m):
a = √1.5 (r+m), c = √3 (h+m); so each body point's 2σ ellipsoid encloses the cylinder — semi-axes
(0.369, 0.369, 1.500) m for the cylinder (1.22 × r laterally, before the ±0.2 m body-point spread) and
(0.216, 0.216, 0.071) m for the sweeper **[E]** (`results/baselines/foci/setup/*/*.json`). FOCI's obstacle term is a
soft overlap cost, so a "cover" bounds the shape the cost sees, not a guarantee.

### 3.3 Obstacle contract
| parameter | judge | FOCI | status |
|---|---|---|---|
| σ level | 2σ | none: Gaussian-overlap cost exp(−dᵀ(Σ_o+Σ_r)⁻¹d/2)·1000/det(Σ_o+Σ_r) | **mismatch** (no threshold); `sigma_level` "native" feeds Σ, "judge" feeds 4Σ (its 1σ shape = the judge's 2σ ellipsoid) |
| opacity | > 0.3 | not used | matched by input |
| margin | 0.001 | none | mismatch (soft cost) |
| safety | hard | soft cost vs jerk and goal cost | mismatch: no collision constraint |
| plane | z = z_c | 3D, z ∈ (0, 1) hard-coded | constrained: band z_c ± `z_half` (below) |

### 3.4 Deviations from the paper's setup
- IPOPT linear solver MA27 → MUMPS (licence; B0; committed patch on a copy).
- **Plane constraint.** FOCI plans in 3D with its z band hard-coded to (0, 1). With that band it moved the body the
  full ±0.5 m off the plane on every probe query (`max_abs_dz_m` ≈ 0.4999, job 19508258), and the projection of such a
  curve is not what FOCI planned. `_planner_cls` subclasses `Planner` with `__init__` verbatim except the band,
  (0.5 ± `z_half`), and optionally the body-point spacing (`kin_scale`, repo 0.2). `plan` is inherited unchanged (its
  A* init still searches z ∈ (0, 1)). `F0_repo_zband` (z_half 0.5) is run in tuning to show the effect.
- Start/goal heading: the start→goal direction for both (the demo uses π/2 for all); the body is axisymmetric, FOCI's
  three-point robot is not.
- Obstacle cut to the body slab and box (as SplatNav); the demo feeds all Gaussians of its scene.
- FOCI's goal is a soft cost and its start a soft constraint (‖·‖² ≤ 0.05, incl. heading): its curves start up to
  0.22 m and end 0.4–2.6 m from the endpoints on the probe **[E]**; the harness completes them (§1.1), and that
  completion is part of what is judged.

## 4. Tuning (both methods, same procedure)

Pre-registered rule, committed before any tuning result (`gmc/configs/baselines/tuning/RULE.md`, fbdbf18): tuning set
= plan §3.5 (50 pairs, disjoint from the 5000), both robots, unchanged harness (judge inline, 120 s); 8 candidates per
method, one pass each; per robot pick the most SUCCESS, then fewest CLAIMED_*, then lowest median
`algorithm_wall_s`; candidates that break §3 (SplatNav 1σ obstacles; FOCI without the plane constraint) are run for
information and are ineligible. Applied mechanically by `bl_harness tunetable`
(`results/baselines/tuning/<method>/tuning_table.json`). Jobs: 19508642 (all 16 candidates, 4 streams on one L40S,
43 min), 19510034 (FOCI F1 rerun, below), 19510126/19510253 (fail locations: `bl_harness locate` re-exports the stored
exact polylines and places the judge's failing edge on the method's path or on a completion segment). Rows:
`results/baselines/tuning/<method>/<candidate>/<R>/<robot>/task_00.jsonl.gz` **[E]**.

**One rerun, disclosed.** In 19508642 FOCI's `F1_zband` (config `{}`) reused a setup artifact built before the FOCI
defaults gained `kin_scale`/`z_half`, because the artifact directory was keyed on the explicit config only; all 100
rows were SETUP_FAIL (kept in `outputs/baselines/stale/`). Fix (a502739: `artifact_path` now also hashes the adapter's
source), then F1 alone was rerun unchanged (19510034). No other candidate was rerun or edited between passes.

**splatnav** (pick: cylinder `S6_eps05_noshrink`, sweeper `S1_eps05`)

| candidate | what | robot | SUCCESS | COLLIDES | UNPROVEN | FAIL | median s | unsafe claims on a completion segment |
|---|---|---|---|---|---|---|---|---|
| S0_sphere | ε = 1: the plain enclosing sphere (R = 0.98 m cylinder, 0.21 m sweeper) | cylinder | 2 | 3 | 0 | 45 | 0.00 | 3/3 |
| S0_sphere |  | sweeper | 50 | 0 | 0 | 0 | 0.32 | 0/0 |
| S1_eps05 | ε = 0.05, 2 cm cells, vmax = amax = 0.1 (authors') | cylinder | 47 | 0 | 0 | 3 | 0.57 | 0/0 |
| **S1_eps05** |  | sweeper | 50 | 0 | 0 | 0 | 0.30 | 0/0 |
| S2_eps05_cell1 | S1 + 1 cm cells | cylinder | 46 | 0 | 0 | 4 | 0.67 | 0/0 |
| S2_eps05_cell1 |  | sweeper | 50 | 0 | 0 | 0 | 0.38 | 0/0 |
| S3_eps05_cell4 | S1 + 4 cm cells | cylinder | 47 | 0 | 0 | 3 | 0.66 | 0/0 |
| S3_eps05_cell4 |  | sweeper | 49 | 0 | 0 | 1 | 0.26 | 0/0 |
| S4_eps02 | ε = 0.02 | cylinder | 48 | 0 | 0 | 2 | 0.80 | 0/0 |
| S4_eps02 |  | sweeper | 50 | 0 | 0 | 0 | 0.36 | 0/0 |
| S5_eps05_narrow | S1 + amax 0.5 (corridor half-width 0.01) | cylinder | 47 | 0 | 0 | 3 | 2.78 | 0/0 |
| S5_eps05_narrow |  | sweeper | 49 | 0 | 0 | 1 | 1.54 | 0/0 |
| **S6_eps05_noshrink** | S1, A* grid = whole known box (not shrunk by r + m + Δ) | cylinder | 49 | 0 | 0 | 1 | 0.56 | 0/0 |
| S6_eps05_noshrink |  | sweeper | 50 | 0 | 0 | 0 | 0.45 | 0/0 |
| S7_eps05_native1sigma | S1 with 1σ obstacles (contract **not** matched; ineligible) | cylinder | 38 | 6 | 6 | 0 | 0.53 | 1/12 |
| S7_eps05_native1sigma |  | sweeper | 43 | 3 | 4 | 0 | 0.30 | 0/7 |

**foci** (pick: cylinder `F3_cov05`, sweeper `F3_cov05`)

| candidate | what | robot | SUCCESS | COLLIDES | UNPROVEN | FAIL | median s | unsafe claims on a completion segment |
|---|---|---|---|---|---|---|---|---|
| F0_repo_zband | repo's z band (0, 1) (plane **not** enforced; ineligible) | cylinder | 18 | 5 | 27 | 0 | 0.21 | 2/32 |
| F0_repo_zband |  | sweeper | 27 | 7 | 16 | 0 | 0.19 | 0/23 |
| F1_zband | F-defaults: z band ±0.01, 1σ covs, cov_scale 1, 10 cp, 40 samples | cylinder | 20 | 4 | 26 | 0 | 0.23 | 2/30 |
| F1_zband |  | sweeper | 33 | 1 | 3 | 13 | 0.39 | 3/4 |
| F2_judge_sigma | F1 + obstacle covs × 4 (2σ shape) | cylinder | 19 | 4 | 27 | 0 | 0.26 | 2/31 |
| F2_judge_sigma |  | sweeper | 31 | 6 | 7 | 6 | 0.55 | 12/13 |
| **F3_cov05** | F1 + robot cov_scale 0.5 | cylinder | 23 | 2 | 24 | 1 | 0.35 | 0/26 |
| **F3_cov05** |  | sweeper | 35 | 2 | 1 | 12 | 0.47 | 2/3 |
| F4_cp20 | F1 + 20 control points | cylinder | 19 | 4 | 27 | 0 | 0.48 | 4/31 |
| F4_cp20 |  | sweeper | 34 | 2 | 7 | 7 | 0.50 | 5/9 |
| F5_kin0 | F1 + body points coincident (kin_scale 0) | cylinder | 22 | 3 | 25 | 0 | 0.24 | 2/28 |
| F5_kin0 |  | sweeper | 29 | 5 | 7 | 9 | 0.56 | 9/12 |
| F6_samples80 | F1 + 80 samples | cylinder | 20 | 3 | 27 | 0 | 0.41 | 2/30 |
| F6_samples80 |  | sweeper | 29 | 1 | 6 | 14 | 0.73 | 5/7 |
| F7_cp20_samples80 | F1 + 20 cp + 80 samples | cylinder | 17 | 3 | 29 | 1 | 0.67 | 4/32 |
| F7_cp20_samples80 |  | sweeper | 35 | 3 | 8 | 4 | 0.57 | 6/11 |

Reading the tables **[E]** (all from the files above):
- **SplatNav** is insensitive to resolution and corridor width on this set (46–49/50 cylinder, 49–50/50 sweeper,
  zero unsafe claims for every eligible candidate). Its FAILs are `qp_infeasible` / `astar_no_path`. The cylinder pick
  (`S6`) only differs from the authors'-defaults candidate S1 by letting the A* grid span the whole known box.
- The **plain enclosing sphere** (S0, 0.98 m for the cylinder) makes SplatNav fail 45/50 cylinder pairs
  (`astar_no_path` 33, `qp_infeasible` 12); its 3 unsafe claims all lie on harness completion segments (A* endpoint
  moved off an occupied voxel). For the sweeper (0.21 m sphere) it is as good as S1. So the ε-squash cover is what
  makes SplatNav viable for the tall body — an adaptation choice, stated as such.
- **Matched contract matters for SplatNav**: with 1σ obstacles (S7) 12/50 cylinder and 7/50 sweeper claims are unsafe,
  11 resp. 7 of them on SplatNav's own path; with the judge's 2σ none are.
- **FOCI** is far weaker under the judge: cylinder 17–23/50, with 26–32 unsafe claims per candidate, almost all
  `geometry_or_margin_unproven` on FOCI's own curve (it grazes Gaussians: a soft overlap cost has no margin). Sweeper
  29–35/50. The plane constraint costs nothing measurable (F0 18 vs F1 20 cylinder) while removing ±0.5 m z drift.
  Pick `F3_cov05` (robot covariance halved) for both robots.

## 5. Pilot (frozen configs, 100 F4 pairs per robot)

Configs frozen in `gmc/configs/baselines/{splatnav,foci}.json` (full effective parameters + adapter SHA-256), commit
0f0e049, **before** the pilot. Pairs: `results/baselines/pilot/pilot_pairs.json` (50 WWEST / 30 GAPW1 / 20 S,
stratified by region × lateral band, seed 20261010). Job 19510252 (one L40S, 4 concurrent streams = method × robot,
judge inline, node gl015, commit 0f0e049). Rows `results/baselines/pilot/<method>/<R>/<robot>/task_00.jsonl.gz`,
summaries `task_00.summary.json`, `report.json` **[E]**.

| method | robot | SUCCESS | CLAIMED_COLLIDES | CLAIMED_UNPROVEN | FAIL | TIMEOUT / ERROR | algorithm s median / p95 / max | judge s median / p95 |
|---|---|---|---|---|---|---|---|---|
| SplatNav | cylinder | **100** | 0 | 0 | 0 | 0 / 0 | 0.60 / 1.57 / 2.97 | 1.45 / 13.9 |
| SplatNav | sweeper | **100** | 0 | 0 | 0 | 0 / 0 | 0.40 / 1.30 / 1.88 | 0.98 / 1.64 |
| FOCI | cylinder | **44** | 2 | 54 | 0 | 0 / 0 | 0.39 / 1.07 / 1.22 | 0.07 / 0.21 |
| FOCI | sweeper | **77** | 5 | 2 | 16 | 0 / 0 | 0.46 / 2.96 / 3.38 | 0.06 / 0.08 |

Per region (SUCCESS/n), cylinder: SplatNav WWEST 50/50, GAPW1 30/30, S 20/20; FOCI WWEST 16/50, GAPW1 8/30, S 20/20.
Sweeper: SplatNav all; FOCI WWEST 37/50, GAPW1 21/30, S 19/20 **[E]**.

Where the unsafe claims are **[E]** (`judge_fail_location`, inline): FOCI cylinder 53 of 56 on FOCI's own curve
(`geometry_or_margin_unproven` 53 — clearance below the 1 mm margin — and 1 `minkowski_interior_witness`), 2 on the
prepended start segment; FOCI sweeper 6 of 7 on the appended goal segment (FOCI stops short: goal gap median 0.20 m,
p90 1.19 m) and 1 on its own curve. FOCI sweeper FAILs are all `ipopt_Maximum_Iterations_Exceeded` (16). So FOCI's
sweeper unsafe claims are mostly caused by the completion that §3.3 prescribes, its cylinder ones by FOCI itself.
Example (`fig_b1_pilot.png`, viewed): on F4X-00002 FOCI's curve cuts the corner the A* route and SplatNav go around;
the Gaussian the judge flags there is a **floor-level** one (centre z −0.042 m, 2σ top 0.020 m = the chassis bottom,
0.37 m from the edge, clearance 0.54 mm < 1 mm) **[E]** (job 19510804). Such sparse floor bumps are typical of these
regions' hard pairs **[G]** (the figure shows them; not counted).

![pilot paths](../gmc/results/baselines/pilot/fig_b1_pilot.png)

Other pilot facts **[E]**:
- **Completion**: SplatNav needed both completion segments on every row (A* voxel centres: start/goal gap median
  8.7 / 8.5 mm, cylinder) and none of them failed; FOCI on every claimed row (start gap median 0.22 m: its soft start).
- **Plane**: FOCI `max_abs_dz_m` ≤ 0.010 (the band); SplatNav ≤ 0.77 m in real units, i.e. ≤ 0.039 in squashed units,
  inside the Δ = 0.0707 the cover accounts for.
- **Paths**: SplatNav path / straight-line length median 1.19 (cylinder), 1.20 (sweeper), ≈ 700 exported vertices
  (dense Bézier); FOCI 1.02 / 1.05, ≈ 42 vertices.
- **Setup once**: every task 1 worker start, 0 restarts, same artifact SHA, same setup id on every row. Setup build
  (numpy, persisted) 0.8–2.5 s SplatNav, 0.01–0.04 s FOCI; instantiate per worker start (GPU) 17.6–31.4 s SplatNav
  (voxel grid; WWEST largest), 11.9–23.5 s FOCI (CasADi NLP + warp), under 4-stream contention.
- **Success vs lateral-clearance band** (cylinder, n per band 2–25): SplatNav 100 % in every band; FOCI 3/4 at 0 m,
  2/6 at 5 mm, 4/11 at 10 mm, 12/25 at 20 mm, 5/21 at 50 mm, 9/21 at ≥ 100 mm — no clear trend with the route's
  lateral clearance **[E]**; its failures follow the floor-level grazes instead **[G]**.

## 6. Projection for B3 (5000 pairs × 2 robots per method)

`gmc/results/baselines/pilot/projection_b1.json` (`bl_harness project ... --streams 4 --gpus 2 --tasks 1`) **[E]**:
mean per-query `outer_wall_s` and `judge_wall_s` per region from the pilot × F4's 2500/1500/1000 pairs, + setup.

| method | method GPU-h (4 streams/GPU) | judge CPU-h | wall h at 2 GPU jobs (judge deferred) | GPU-h if judge inline | wall h, judge inline |
|---|---|---|---|---|---|
| SplatNav | 0.45 | 5.98 | 0.23 | 1.95 | 0.97 |
| FOCI | 0.49 | 0.21 | 0.24 | 0.54 | 0.27 |

Recommendation for B3 **[G]**: run each method with `--judge defer` on the GPU (4 harness streams per L40S job:
method × robot, or more tasks per region) and judge with `bl_harness judge` as a CPU array (SplatNav's dense paths
make the judge ~2–5 s/query on WWEST cylinder: ≈ 6 CPU-h). Either way both methods are two orders of magnitude inside
the plan's B3 share (150 CPU-h, 100 GPU-h total). Caveats: linear scaling to 4 streams per GPU is assumed from a pilot
whose streams also spent time in the inline judge [G]; CPU-h of the GPU jobs = 4 cores × wall. SplatNav task files
are large (exact dense polylines): ≈ 100–300 kB per row before gzip; `bl_harness archive` drops the redundant raw
path and gzips (pilot: 7.1 → 3.0 MB for 400 rows), so ~0.5–1 GB gzip per robot for 5000 rows is plausible [G] — B3
should decide whether to commit them or keep them under `outputs/` with SHA sidecars.

## 7. For B2 (PNO, cust_fields) and B3
- Use the harness unchanged: write `bl_<method>.py` with `Adapter.build_setup / instantiate / plan` (see the
  `bl_worker.py` docstring), add the module to `bl_worker.ADAPTERS` and the env's python to
  `bl_harness.METHOD_PYTHON`. `plan` returns a planar path (u, v[, z]) in the plan frame; the harness does completion,
  export, judge, timing, rows. Raise for errors; return `claimed: False` with a reason for method-declared failures.
- The scene export (`outputs/baselines/scene/<R>_<robot>.npz`) is the input the shared rasteriser should read: the
  judge's Gaussians (opacity > τ = 0.3 already), plan frame, level 2.0, margin 0.001, z_c, body, known box.
- Artifacts are keyed on (config, adapter source SHA); a frozen config is `configs/baselines/<method>.json` with
  per-robot full parameter sets; `bl_harness setup --method M --config configs/baselines/M.json`.
- Tuning: `bl_harness run --pairs tuning`, `tunetable --method M --ineligible ...`; same pre-registered rule
  (`configs/baselines/tuning/RULE.md`) keeps the effort comparable.
- `astar_replay` (100 % SUCCESS) and the unit tests are the harness's regression check; rerun
  `BL_HEAVY=1 pytest tests/unit/test_bl_harness.py` in an sbatch job after any harness change.

## 8. Budget and jobs
B1 used **4.0 CPU-h and 0.96 GPU-h** of 40 / 10 (`sacct` over the 18 jobs in `jobids/B1.txt`, excluding the agent's
own 1-CPU allocation) **[E]**. Jobs: 19508155 export + tests, 19508232 sanity, 19508233 setup, 19508258 GPU probe,
19508642 tuning, 19508644 rounding sanity, 19510034 F1 rerun, 19510126 / 19510253 fail locations, 19510252 pilot,
19510647 / 19510776 (failed: script on node-local /tmp) / 19510804 / figure re-renders and diagnosis.
