# Amendment 3 — plane floor, the shared three-robot case, long range, and stage timings

**Status:** approved by the user on 2026-09-17 (afternoon EDT) in an interactive session. Amends
`docs/superpowers/specs/2026-09-12-height-band-robots-design.md`, Amendment 1
(`2026-09-13-floor-rule-amendment.md`) and Amendment 2 (`2026-09-14-per-robot-cases-amendment.md`).
Everything not named here is unchanged and still binds you.

Worktree `/scratch/wg2381/splathjb-plane`, branch `height-planefloor` (from `height-percase` 6bcfd8e).
Chain files `/scratch/wg2381/claude_jobs/plane/`. Stages P1…P5, one unattended agent each.

## What the user asked for, in their order

1. **Same start + same goal, three robots, different routes.** Prove the algorithm plans differently
   for a sweeper (disc r=0.175, band [0.02,0.10]) and a tall cylinder (disc r=0.30, band [0.02,1.75]) —
   e.g. the sweeper goes under a table, the cylinder goes around it. The uav is the third robot.
   *If no case can be made to work, that is an acceptable outcome, but then you must **visualize why**:
   show, per robot, the map it actually sees and where its free space dies, so the reader can see that
   no subspace admits a route for both.*
2. **Replace the floor Gaussians with one large plane.** Manual denoising, explicitly a stopgap.
   This is the enabler for (1) — the user's words: do (2) first, and if it fixes (1), re-plan (1) on top.
3. **Push A and B further apart**, on a larger subspace, and check the algorithm still works.
4. **Per-stage wall-clock timing**: denoise/scene-prep → projection → compile → query →
   certification/connectivity → and what that says about the algorithm's efficiency.

## The three decisions the user made when this was planned

- **D1 — criterion 3 is relaxed, with approval.** Spec §5.3 criterion 3 ("start and goal ≥ 0.5 m from
  any obstacle in every band") is replaced, for the shared case only, by the Amendment 2 per-robot
  precheck: for each robot, start and goal clearance ≥ `robot.max_radius() + 0.05` m on that robot's own
  certified map, and start and goal in the same connected component of `dist > r`.
  Rationale is in `docs/worklog/height_map_diagnosis.md` §3: crit1 ∧ crit3 intersect in **0.9 m²** of a
  990 m² hall as built, and **1.2 m²** even with every phantom cell deleted, so 0.5 m is not a data
  problem and cannot be fixed by P1. Every report must state that this is a user-approved criterion
  change and quote both numbers.
- **D2 — target A–B distance ≥ 8 m, window ≤ 12 m × 12 m** (spec's 8×8 cap is lifted to 12×12 for
  Amendment 3 only). If the probe says 12×12 is unaffordable, shrink the **window**, never the
  semantics, and keep the ≥ 8 m separation; report the largest window that was actually affordable.
- **D3 — "floor" is identified by phantom evidence, not by a bare height threshold.** See P1.

## Claims boundary (must appear in every report, figure caption and video title)

- The plane floor is a **user-approved, manual scene-definition change**, not a reconstruction method
  and not an outer approximation with a guarantee. Results on it are sound *with respect to the edited
  scene*; the gap between that and the room is the open research question
  (`docs/worklog/height_map_diagnosis.md` §4).
- Criterion 3 was relaxed per D1. Say so next to any "the shared case passed" claim.
- If task 1 produces genuinely different routes for sweeper and cylinder from one start/goal pair,
  **that** is the morphology claim, and it is allowed — it is exactly what Amendment 2 forbade claiming
  from per-robot cases. It must rest on one window, one start, one goal, one scene, with all three maps
  shown.

## Frozen — never changed to rescue a run

τ = 0.3, ρ = 2.0, the robot table (spec §3.3), `configs/height_showcase.yaml`, GMC itself,
`verify_curve`, `replay3d`, the read-only module list in the main plan's Global Constraints.
Success is `REACHABLE` **and** `verify.certified` **and** `replay3d.passed`. On `UNKNOWN`, one
budget-only retry (`BUDGET=2`) is allowed. Anything else is reported as it is.

---

## P1 — the plane floor (task 2) + timing instrumentation (task 4 scaffolding)

**Deliverable: `gmc/src/gmc/height/planefloor.py`, tests, a rebuilt scene variant, before/after evidence.**

P1a. **Identify the phantom layer by the diagnosis test.** Rasterise the hall at 0.05 m
(`showcase_scene._occupancy`, the existing bands). A cell is *phantom* iff it is occupied in
[0.02, 0.10] m and free in **every** band from 0.10 m to 2.50 m. This is the test that already produced
94,780 cells / 236.95 m² / 367 speckle components (`gmc/results/height/diagnosis/map_diagnosis.json`);
reproduce those numbers first as a regression check, and stop and escalate if they do not reproduce.

P1b. **Replace, do not merely delete.** A splat is replaced iff its ρ-footprint lies (in xy) only in
phantom cells **and** its ρ-top is below the floor + 0.10 m. Then insert one analytic floor: a small
tiling of large, flat, opaque Gaussians lying in the fitted floor plane with a σ along the normal small
enough that, at ρ = 2, they do not reach `z_floor + 0.02` — so no robot band ever sees them, while the
scene still has a real ground surface for rendering and for `replay3d`. Verify that last property
numerically, do not assume it.

P1c. **Prove nothing real died.** Reuse the survivor checks already in the repo
(`gmc/results/height/showcase/floor_rule_legs_check.json`, `floor_rule_survivors.json` and the script
that made them). Table A/B/C/G legs and tabletops, the SW-hall bench and its end supports, plinths and
the reception counter must all survive with their support counts before/after. A figure per object.
If any of them loses mass, tighten the rule until it does not, and report the final parameters.

P1d. **Evidence.** Band-occupancy table before/after (the 40.7 / 17.4 / 13.3 / 11.7 / 11.1 % column is
the "before"), the monotonicity check (occupancy must now decrease toward the floor), projected support
counts for sweeper and cylinder on the Amendment 1 window A `[5.5, 3.5, 12.0, 10.5]` before/after, and
the crit-3-style free-area map. `gmc/results/height/plane/`.

P1e. **Timing instrumentation.** Add a per-stage timer that every downstream run writes into its
result JSON, with these stages, each measured separately and each with its input size recorded:
`scene_load`, `floor_replace`, `project` (per robot), `compile_pairs` (`query_candidate_pairs`),
`compile_slabs` (`build_slabs`), `compile_mobility` (`compile_mobility`), `query`, `verify_curve`,
`replay3d`. Put it in `gmc/src/gmc/height/timing.py` and thread it through `height/run.py`'s
`compile_and_query` and `showcase_run.py` **without changing any read-only module**. Existing callers
must keep working; `compile_seconds` and `query_seconds` keep their current meaning.

P1f. Commit per task, TDD, tests named `test_height_plane*.py`. Run the full suite once at the end and
record the summary line.

**P1 hands P2:** the scene loader to use, the parameters, the support counts, and an honest statement of
whether the near-floor map is now trustworthy.

## P2 — the shared three-robot case (task 1)

P2a. Rebuild the case search on the plane-floor scene, with criterion 3 as D1 defines it and criterion 1
(straight start→goal passes under an overhang) unchanged. Search over the tables the A4 search already
enumerated plus the scene-wide pair search; the machinery is
`gmc/experiments/showcase_scene.py step_case`, `percase_search_*.py` and the A4 search
(`gmc/results/height/showcase/case_search_a4.json`). Rank candidates by *expected morphological
contrast*: the straight line must be free for the sweeper band and blocked for the cylinder band, and
start/goal must be connected for both.

P2b. Pre-check on the **certified** map (`project_scene` → `showcase_scene._support_raster(0.025)` → EDT),
never on the AABB aid. Record support counts; `showcase_run.py`'s 20-projected-hour probe abort stands.

P2c. Run **all three robots on the identical window, start and goal**, each to success, through sbatch.
Then: `replay3d` for each, one comparison figure with the three maps and the three certified routes over
the same window, and a video per robot with the EWA renderer already in the tree
(`gmc/src/gmc/height/ewa.py`, `viz3d.py`).

P2d. **If no case passes, this is the deliverable instead** (the user asked for it explicitly):
for the best few candidate windows, a figure set showing, per robot, its projected certified map, its
free space `dist > r`, its connected components with start/goal marked, and the exact criterion that
fails — so a reader can see *why* no subspace admits routes for both. Plus a scene-wide version of the
same argument. Do not weaken any criterion further to force a pass; report the failure.

## P3 — longer range (task 3)

Same scene, same three robots, target **‖goal − start‖ ≥ 8 m**, window ≤ 12 m × 12 m (D2). Preferably
keep the shared start/goal idea; if a shared long case does not exist, run the long case per robot and
say so. Probe first; if the projected hours exceed the abort threshold, shrink the window (keeping
≥ 8 m) and, only if that is not enough, tell the orchestrator before using the Amendment 1 grid coreset.
Deliverables: certified runs, replay3d, videos, and a table comparing short-range (Amendment 2: 2.0 /
2.7 / 4.1 m) against long-range on supports, compile time, query time and clearance lower bound.

## P4 — stage timings and efficiency (task 4)

Aggregate every `timing.py` record from P1–P3 plus the Amendment 2 runs into one table and one figure
per axis: **time vs number of supports**, **time vs window area**, **time vs A–B distance**, broken down
by the P1e stages. Fit and state the empirical growth of compile and of query separately, with the
number of points and the range they cover — no fit on fewer than 4 points, and say so if that is all
there is. Add a small dedicated scaling sweep (one sbatch job) if the runs alone do not span a useful
range. Report: where the wall clock actually goes, which stage dominates at which scale, and what that
implies for the biggest scene GMC could handle in a day. `docs/worklog/height_timing.md`.

## P5 — final sweep

Build a checklist of user tasks 1–4 and every deliverable named above, judge each from artefacts and
commits only (never from a predecessor's exit code — agents in this series have finished their work and
then died on a usage limit), finish everything missing, open every figure and video frame and check it
against its JSON, run the full test suite, and write the user-facing summary. **Do not stop because of a
usage limit**: the wrapper waits the limit out inside the allocation and, if that is not enough, drop a
continuation sentinel and it will resubmit you at the next refresh slot.

---

## Compute envelope (user-approved; no need to ask again)

| stage | jobs | resources |
|---|---|---|
| case search | ≤ 2 per stage | 8 CPU / 64 GB / ≤ 4 h |
| compile+query run | 1 per robot per case, + 1 budget-only retry on UNKNOWN | 8 CPU / 64 GB / ≤ 24 h |
| render / video | 1 per robot per case | 4 CPU / 32 GB / ≤ 4 h |
| timing sweep | 1 | 8 CPU / 64 GB / ≤ 6 h |

- Job names start with `pf_`. Before each submit: at most **3** `pf_*` in the queue
  (`squeue -u wg2381 -h -o %j | grep -c '^pf_'`), at most **24** in
  `/scratch/wg2381/claude_jobs/plane/jobids/<STAGE>.txt` in total across the chain. Append
  `<jobid> <name> <utc>` immediately after submitting.
- Everything that loads the 7.3 M-splat scene goes through sbatch, never the login node. Poll with
  `squeue -j` / `sacct -j`; never `srun`, never `--wait`.
- `scancel` only job IDs you yourself appended to your own `jobids/<STAGE>.txt`. Never `--all`,
  never a wildcard.
- Install nothing. Python is `/scratch/wg2381/.conda/envs/gmc-venv/bin/python`.
