# Baselines on the F4 benchmark — plan (single source of truth for the `bl` chain)

Written 2026-10-09 by the interactive orchestrator. Chain: `/scratch/wg2381/claude_jobs/baselines/`
(stages J → B0 → B1 → B2 → B3 → B4). Worktree `/scratch/wg2381/splathjb-baselines`, branch `baselines-f4`
(from `origin/aerial3dg-fail` 2a68d73; the judge fix is d757729). Every stage prompt points here; where a
prompt and this file disagree, this file wins unless the prompt says it overrides a named section.

## 0. The user's ask (their words, lightly cleaned)
> 这几个文章是我们的baseline，要把这四个都跑一下: splatnav (github.com/chengine/splatnav), foci
> (github.com/leggedrobotics/foci), PNO (github.com/ExistentialRobotics/PNO), cust_fields
> (github.com/Shuaikang-Wang/cust_fields). Reuse the start and ending point of the 5000 pairs result and follow
> the same doctrine for logging and timing etc. Arrange claude_jobs to finish them.

Decisions the user made on 2026-10-09:
- **Pair set = F4's 5000 confirmed-reachable pairs** (`gmc/results/aerial3dg/f4/pairs_confirmed_5000.json`), not G2.
- **PNO = published pretrained weights, zero-shot only.** No training, no fine-tuning.
- **Fix the replay judge first.** The user: *"there is one problem remaining with the replay judge — either it used
  different standards than the A\*, or it has issues so that even if GMC or the A\* passes, it reports some kind of
  issue. Fix that first."* Done interactively in d757729 (§2); stage J verifies it at scale before any baseline runs.

## 1. Facts to start from (do not re-derive; cite)
- **Scene:** `scene_v2` archive, SHA-256 `2a3a72d6…96cc` (`aerial3dg_run.ARCHIVE/MANIFEST/FROZEN_SHA256`). Regions,
  boxes, quotas: `pairs_confirmed_5000.json` → `regions` (WWEST 2500 `F4X-*`, GAPW1 1500 `F4W-*`, S 1000 `F4S-*`).
- **Robots** (`gmc.gs3d.robots`): `SWEEPER` r 0.175 m, half-height 0.04; `CYLINDER` r 0.30, half-height 0.865; both
  vertical cylinders, `ground_unicycle`, ground clearance 0.02 m, centre z_c = floor + clearance + half-height
  (`aerial3dg_run.ground_world`). Limits (`gs3d.planner._limits`): 0.3 m/s, 1 rad/s. Margin 0.001 everywhere.
  Axisymmetric ⇒ yaw never changes occupancy; turns are in place.
- **Pairs are confirmed for the cylinder** (real body, margin 0.001, lattice A* ROUTE + `verify_path`); each pair
  carries the A* route, length ratio, lateral/vertical clearance ladders, endpoint clearances
  (`experiments/aerial3dg_fail3_sample.py` docstring). The sweeper runs on the same pairs as context, as in F4.
- **GMC on these pairs:** rows `gmc/results/aerial3dg/f4/gmc/<REGION>/<robot>/task_NN.jsonl` (+ `.summary.json`),
  compile-once proof `f4/compile_once.json`, classes `f4/failures.csv`, `f4/summary.json`. Persisted compiles (F3, reused
  by F4, uncommitted): `/scratch/wg2381/splathjb-aerial3dg-fail/gmc/outputs/aerial3dg/f3/<REGION>/<robot>.a3c` (+ `.json`
  with SHA-256). Runner: `experiments/aerial3dg_fail3_f4.py`; trace/probe machinery: `experiments/aerial3dg_fail5_trace.py`.
- **GMC headline before the judge fix** (F4/F5): cylinder 3614/5000 REACHABLE (27.7 % fail: TIMEOUT 194, endpoint
  tolerance 656, judge vetoes 534, ERROR 1, EP-GENUINE 1); sweeper 4819/5000 (judge vetoes 4). F5 proved the 534+4
  judge vetoes are the judge's, not the geometry's (`docs/aerial3dg_failures_f5.md` §2.4–2.5).
- **Timing doctrine (G2/F4):** one compile per robot per region, persisted with SHA-256, every query row carries
  that `compile_id`, a compile-once proof per task, per-query 120 s wall limit (`TIMEOUT:query_exceeded_120s`), rows
  with `status, reason, algorithm_wall_s, stages{…}, cpu_s, outer_wall_s, polyline_sha256`, task summaries with
  host/commit (`aerial3dg_run.host()`), and the measurement sheet `measurement_template.csv` (repo root, untracked in
  the main checkout; copy it in) filled per method, `N/A` where a stage does not exist.
- **External repos:** `/scratch/wg2381/ext_repos/{splatnav,foci,PNO,cust_fields}` (clones, upstream HEADs e996e22,
  79d8ddc, 6384751, 5ca178e). Install scripts and logs: `/scratch/wg2381/ext_repos/env_setup/`. Status 2026-09-24:
  **only `/scratch/wg2381/.conda/envs/foci` exists** (missing open3d: `libEGL.so.1` absent on nodes; MA27 licensed
  solver absent → `Invalid_Option`). splatnav/pno/cust_fields were **never created**: the four `conda create` jobs ran
  in parallel and corrupted each other in the shared pkgs cache (`Stale file handle`, `InvalidArchiveError`).
- **What each baseline is, natively:** SplatNav = Splat-Plan: polytope safe corridor from GSplat ellipsoids + Bézier
  QP (Clarabel), sphere/ellipsoid robot, A*-on-voxels initialisation, torch/CUDA. FOCI = trajectory optimisation
  (CasADi/IPOPT) of a Gaussian robot against Gaussian scene overlap, orientation-aware, own A*-based initial guess,
  warp. PNO = neural operator mapping an SDF/cost map + goal to a value function; path by following its gradient;
  pretrained 2D models on Hugging Face (`lukebhan/generalizableMotionPlanningViaOperatorLearning`; data
  `lukebhan/generalizableMotionPlanning`). cust_fields = harmonic navigation function on a 2D "forest world"
  (star-shaped obstacles in a bounded workspace, `NF/`), plus homotopy customisation (`TOPO/`, needs a target class).
- **Cluster:** CPU `--account=torch_pr_527_general --partition=cpu_short`. GPU: `torch_pr_527_general` may use
  **`l40s_public`** (test-only start ≈ hours) and `h200_public` (≈ 2 days queue); h100/a100 partitions refuse this
  account. User QOS cap 120 GB memory across all jobs.

## 2. The judge (fixed in d757729; verified at scale by stage J)
"The judge" = `gmc.gs3d.trajectory.replay_plan(result, GaussianBodyOracle(<region prepared scene>))` at d757729 or
later: geometry `verify_path` (the very check the F4 A* routes were confirmed with) + kinematics
`verify_linear_trajectory` + attained-goal match. It is the **only** success authority for GMC and every baseline.

What was wrong, and the fix:
- **REPLAY-AABB** (GMC 524 vetoes). The oracle's coverage test asked whether the swept body's *world AABB* lies in the
  rotated route prism. In a 53–64° frame that box overshoots the body by r(|cos|+|sin|−1) (≈ 0.34 r). A* never trips
  it because it searches *through* the same test; GMC (and any baseline) plans against the real body. Fix:
  `RouteBoxKnownSpace.contains_swept_cylinder` tests both end cylinders in the route frame (exact: the swept body is
  the convex hull of its end cylinders, the prism is convex); the oracle prefers it when a known space offers it;
  `FaceLoggingKnownSpace` (experiments/uavlamp_query.py) delegates. Soundness: the crop keeps every Gaussian whose
  support box meets the prism's world box, so nothing that could touch a body inside the prism is missing.
- **REPLAY-RATE** (GMC 10 + 4 vetoes). The exporter times each step exactly at the limit and stores absolute float
  times, so a µs-long turn reads ~1e-9 over the yaw-rate limit. Fix: the rate check allows one ulp of time and two
  of pose per step on top of the existing 1e-9.
- Both changes only **loosen** (AABB coverage implies exact coverage; the slack is ulps). Collision checks are
  untouched. So every route that passed before still passes, and nothing that collides can start passing.
- Not judge bugs, and unchanged: endpoint-tolerance UNKNOWNs (GMC's own `*_not_certified_free`, by design), POST
  TIMEOUTs, the simplify ZeroDivisionError. Also note: re-judging a *stored, 0.1 mm-rounded* route can fail where
  the exact poses pass (F5: 5 S rows); always judge the exact poses a method produced, never a rounded copy.

**Stage J acceptance (hard gate for the whole chain):** (a) full `gmc/tests` suite green, or every failure shown to
fail identically at 2a68d73; (b) all 5000 A* routes still pass on the cylinder (exact poses where rounding bites);
(c) the 538 vetoed GMC rows re-queried on the same compiles: report how many become REACHABLE (expected ≈ all) and
why any do not; (d) a stratified ≥ 300-row sample of GMC REACHABLE rows stays REACHABLE with the same polyline SHA;
(e) a negative control: for ≥ 500 pairs, judge the straight start→goal segment with old and new code — no row may go
from occupied/unproven-by-collision to free (only `map_unknown`/rate verdicts may change). J then writes
**GMC's re-judged F4 rows** (`gmc/results/baselines/gmc_rejudged/`), which are the GMC column every later comparison
uses. If (a), (b), (d) or (e) fails: stop the chain (J writes its standup with the evidence; no handoff).

## 3. Fairness contract (binding for every baseline)
1. **Same input.** Per region, every method gets the same scene: the Gaussians of the region's judge scene (the crop
   the persisted compile was built from — means, covariances, opacities, the floor support). Gaussian-native methods
   (SplatNav, FOCI) read those Gaussians. Map-based methods (PNO, cust_fields) read a map made by **one shared
   rasteriser** (`gmc/experiments/bl_raster.py`, written once in B2, unit-tested): Gaussians with opacity ≥ the judge's
   τ, 2σ-level ellipsoids (the judge's level), intersected with the robot's vertical extent [z_c − h − margin,
   z_c + h + margin], conservatively projected to a 2D occupancy grid and inflated by r + margin (C-space of a point).
   Where a method exposes the obstacle contract (σ-level, opacity threshold, safety margin), set it to the judge's
   ("matched contract"); otherwise use its native default and record the mismatch.
2. **Same body.** Each method uses its native body model. Where it cannot represent the vertical cylinder exactly,
   use the smallest conservative cover (e.g. the enclosing ellipsoid, a Gaussian set covering the cylinder) and
   state the cover. All planning happens in the robot's plane at z_c (constrain z if the method plans in 3D; if it
   cannot, project its path to the plane and say so).
3. **Same endpoints and judge.** Start/goal = the pair's `start_uv`/`goal_uv` at z_c, exactly. A method's output is
   converted to a gs3d `gs3d.v1` result **through the same export GMC uses** (turn in place to heading, translate,
   final turn; segments split to ≤ 0.20 m as `QCONFIG.export_max_segment_m`; read `gmc.aerial3d.api` to mirror it
   exactly, including start/goal yaw and goal tolerances) and judged by §2. A path that stops short of the goal is
   completed by a straight segment to the goal, which is judged like the rest (record that it was appended).
4. **Same timing.** One-time setup per robot per region (map/voxel/SDF/rasteriser build, corridor or index prep,
   model load) is timed and reported separately as the method's "compile", reused by every query of that
   robot/region, with a setup-once proof per task (the analogue of `compile_once.json`). Per query: hard 120 s wall
   limit (enforced from outside the method, like F4), and the time the judge takes is **not** charged to the method
   (record it separately as `judge_wall_s`).
5. **No tuning on the 5000.** Start from the authors' defaults. Tune only on the **tuning set** = F3 pilot pairs
   (`gmc/results/aerial3dg/f3/sample/<R>/pilot_pairs.json`: first 25 WWEST + 15 GAPW1 + 10 S, disjoint from the 5000),
   with a comparable effort per method (≤ ~10 configurations each), then freeze the config in
   `gmc/configs/baselines/<method>.json` and commit it **before** the full run. The frozen config is what B3 runs.

## 4. Per-method adaptation (the starting plan; deviations allowed with a stated reason)
- **SplatNav** (B1): Splat-Plan only (no Splat-Loc, no Blender). Robot = ellipsoid enclosing the cylinder body (or the
  repo's sphere if that is all it supports — then the enclosing sphere, and say how conservative that is). Plane z_c.
  GPU (`l40s_public`). Bypass nerfstudio scene loading by feeding the Gaussians directly to its planner classes.
- **FOCI** (B1): robot as a set of Gaussians fitted to the cylinder; its own initial guess (never our A* route — that
  would be the "oracle initialisation" upper bound, out of scope); IPOPT linear solver MA27 → MUMPS (licence), kept as
  a committed patch `gmc/baselines/patches/foci_mumps.patch` applied to a copy, disclosed as a deviation. open3d only if
  needed (headless: `open3d-cpu`, or avoid the import).
- **PNO** (B2): published pretrained 2D model, zero-shot; the region's C-space map (§3.1) resampled to the model's
  input grid; value function → path with the paper's own extraction; then §3.3. No training of any kind (user's
  decision). CPU inference is fine if it fits the 120 s limit; else GPU.
- **cust_fields** (B2): obstacles = star-shaped approximations of the C-space map's connected components (convex hull
  or star decomposition — document which, and how often it closes a real passage); workspace = region box. The
  homotopy customisation needs a per-pair target class that this benchmark does not define, so the **main result is
  the repo's plain navigation function** (`NF/`, as `test_nf.py`); say so in the report.
If a method cannot run at all after honest effort (within its stage budget), record exactly why with evidence
(commands, errors) and continue with the others; never fake rows.

## 5. Rows, outcomes, files
Per method/robot/region: `gmc/results/baselines/<method>/<REGION>/<robot>/task_NN.jsonl` + `.summary.json`
(host, commit, setup id + SHA, setup-once proof, frozen config SHA). Row = F4's fields where they apply plus:
`method, setup_id, claimed (bool), claimed_reason, appended_goal_segment (bool), judge_passed, judge_reason,
judge_geometry_reason, judge_kinematics_reason, judge_wall_s, path_length_m, vertices, route_polyline (exact),
polyline_sha256, stages{method-specific}`. Outcome (`status`), exactly one:
`SUCCESS` (claimed and judge passed) · `CLAIMED_COLLIDES` (claimed, judge geometry `occupied`) · `CLAIMED_UNPROVEN`
(claimed, judge `unknown`/`unresolved`, e.g. margin unproven) · `CLAIMED_KINEMATICS` (geometry passed, kinematics
failed) · `FAIL` (method reports failure) · `TIMEOUT` (> 120 s) · `ERROR` (crash; message kept) · `SETUP_FAIL`.
`CLAIMED_*` are the baseline's *unsafe/unsound* rate and are reported as such — never folded into FAIL.

## 6. Stages, budgets, hand-offs
| stage | does | budget | hands the next stage |
|---|---|---|---|
| J | verify the judge fix at scale (§2 acceptance), write GMC's re-judged rows | 20 CPU-h | `gmc_rejudged/` + verdict; stops the chain on failure |
| B0 | build splatnav/pno/cust_fields envs **one at a time**, fix foci (open3d, MUMPS patch), download PNO weights, run each repo's own smoke demo | 10 CPU-h, 2 GPU-h | `docs/baselines_envs.md` (env → status, versions, smoke evidence) |
| B1 | shared harness (`gmc/experiments/bl_harness.py`: scene/body/pairs loading, export + judge, row schema, timeout, setup-once proof) + SplatNav + FOCI adapters; tuning; frozen configs; pilot on 100 F4 pairs/robot (stratified) → cost projection | 40 CPU-h, 10 GPU-h | adapters + frozen configs + projection json |
| B2 | rasteriser + PNO + cust_fields adapters on the same harness; tuning; frozen configs; pilot + projection | 40 CPU-h, 5 GPU-h | same |
| B3 | full runs: 4 methods × 2 robots × 5000 pairs as arrays; collect; completeness + setup-once checks | 600 CPU-h, 100 GPU-h | complete rows + `gmc/results/baselines/collect.json` |
| B4 | comparison vs GMC (re-judged) + report + final sweep against §0 | 20 CPU-h | `docs/baselines_f4_report.md` (Chinese), user standup |

B3 budget rule: if a method's pilot projects more than its share (600/4 CPU-h, 100 GPU-h total), first parallelise
and reorder; if still over, run that method on a **stratified 1000-pair subset** (region × lateral-clearance band) and
say so in every table. Never shrink silently, never relax §3.

## 7. Report (B4) — what "done" means, from the user's ask
1. All four baselines run on the F4 5000 pairs (or a declared stratified subset with the reason), both robots, same
   endpoints, same judge, same timing doctrine — or a documented, evidenced reason a method could not run.
2. Headline table per robot: SUCCESS / CLAIMED_* / FAIL / TIMEOUT / ERROR counts and rates for GMC (re-judged) and
   each baseline; per region; failure rate vs route lateral-clearance band and vs detour ratio (F4's bands, so it
   lines up with `docs/aerial3dg_failures_f4.md`).
3. Timing: setup ("compile") per robot per region, per-query median/p95, amortised cost for 5000 queries,
   stage breakdown; one filled `measurement_template.csv` per method (`docs/baselines_measurement_<method>.csv`).
4. Path quality on pairs both GMC and a baseline solved: length ratio vs A* route, vertices/turns.
5. Figures (Read every one you cite): top-down overview per region with a few representative pairs showing all
   methods' paths, and the clearance-band plot. Each adaptation/deviation listed in one table (body cover, contract
   matched or not, MUMPS, plain NF, PNO zero-shot, appended goal segments).
6. Claims discipline: label **[E]**/**[G]** as F1–F5 did; a baseline failure caused by our adaptation (conservative
   cover, rasteriser resolution) must be called that, not the method's failure.
