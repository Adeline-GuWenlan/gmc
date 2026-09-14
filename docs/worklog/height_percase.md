# Height-band robots, one case per robot (Amendment 2) — worklog

Plan: `docs/superpowers/plans/2026-09-14-per-robot-cases-amendment.md`. Worktree `/scratch/wg2381/splathjb-percase`,
branch `height-percase`, from `height-bands` at `074dc89`. Job ledger `/scratch/wg2381/claude_jobs/percase/jobids.txt`.
Per-robot agent notes go to `docs/worklog/height_percase_<robot>.md` and are merged here by the orchestrator.

## Setup (orchestrator, interactive session, 2026-09-14)

- 05:3x — The user chose: per-robot cases; 3D point-cloud MP4; install `imageio-ffmpeg`; cancel C2; the compute
  envelope in the amendment.
- 05:4x — `scancel 17731929` (C2 agent, pending, begin 09:03): state `CANCELLED`. No other job touched.
- 05:4x — `pip install imageio-ffmpeg` into `/scratch/wg2381/.conda/envs/gmc-venv` (user-owned venv):
  `get_ffmpeg_exe()` → bundled ffmpeg 7.0.2-static. `viz.save_animation` now writes MP4.
- 05:4x — `git worktree add -b height-percase /scratch/wg2381/splathjb-percase height-bands` (HEAD `074dc89`).
- 05:45 — Wrote Amendment 2, `gmc/hpc/percase_{search,run,render}.sbatch`, and gave `experiments/showcase_run.py`
  `--case`, `--out-dir`, `--raw-subdir` (defaults keep the shared-case behaviour).
- ~06:25 — All four in-session agents stopped on a usage limit (reset 09:40); the session was restarted.
  - On disk at 09:53: the UAV search job 17763665 COMPLETED (15.5 min; 908/1,144 prechecks pass; top candidates
    cross table A at z_c 1.20 with 0–233 UAV supports).
  - The sweeper search script was written but not submitted.
  - The render code and tests were written; no toy check yet.
  - The cylinder agent had written nothing.
- 09:55 — The user asked to resume the sub-agents; each was resumed with its on-disk state and a usage-economy note.
- 10:04 — **uav succeeded.** Run 17777506 (47 s), case A-long-46 over table A: REACHABLE, verify certified
  (min 0.456 m), replay3d passed. replay3d checked 0 pairs: no splat came within its margin, so its reported bound
  is the 0.05 m margin.
- 10:06 — **sweeper succeeded.** Run 17777613 (49 s), case under table B, window [−0.85, 9.75, 1.15, 12.8],
  477 supports: REACHABLE, verify certified (min 0.0172 m), replay3d passed. It checked 3,300 pairs and refined
  104; lower bound 0.0268 m, worst splat 4491937; 0 collisions.
  - Ledger: 4 jobs, none queued. Render waits on the render agent's verification.
- ~10:13 — Cylinder search 17777663 COMPLETED (4 m 49 s, errors 0).
  - Suggestion: P1 "around", window [0.5, 10.0, 3.5, 13.0], 3,791 supports, probe projected 0.006 h.
- ~10:15 — The session ended again. The cylinder agent stopped before `case.json`. The render agent stopped
  without submitting renders or writing notes, though the toy check outputs exist.
- 15:32 — Session restarted.
  - The cylinder agent was resumed at case selection.
  - The render agent was resumed and authorized to submit the uav and sweeper render jobs itself, inside the
    envelope. Ledger: 5 jobs, none queued.
- 15:35 — Cylinder run 17799997 submitted by its agent. Case P1 "around": window [0.5, 10, 3.5, 13],
  3,791 supports, start (1.21, 10.51), goal (3.01, 12.51); the bench's north end blocks the straight line.
  Probe projected 0.0056 h.
- ~15:45 — Render agent submitted 17801474 (uav) and 17801523 (sweeper).
- 15:51 — **sweeper video done.** 17801523, 1 m 25 s.
  - Outputs: `sweeper_3d.mp4` (158 frames, 78,465 splats, DC colour), `sweeper_2panel.mp4`, overview, PLY.
  - The projection matches the run (477).
  - The orchestrator opened `sweeper_3d_f050.png` and `sweeper_overview.png`: table B in true colour with its
    exhibits, legs and the partition behind; the prism path runs straight under the tabletop; the title carries
    the verdict and the claims boundary.
  - Sent to the user.
- 15:52 — **cylinder succeeded.** Run 17799997 (8 m 31 s: compile 17.3 s, query 409 s), case P1 "around",
  3,791 supports.
  - Verdict: REACHABLE, verify certified (min 0.0066 m), safe graph 6 nodes / 3 edges.
  - replay3d passed: 534 samples, 12,327 pairs, 438 refined, lower bound 0.0032 m, 0 collisions.
- 15:53 — **uav video done.** 17801474, 2 m 46 s.
  - Outputs: `uav_3d.mp4` (289 frames, 91,684 splats), `uav_2panel.mp4`, overview, PLY. The projection matches
    the run (143).
  - The orchestrator opened `uav_3d_f050.png` and `uav_overview.png`: the prism flies over table A (exhibits,
    legs, bench visible); the title carries the verdict and the claims boundary. The floor-path/trail offset in
    the overview is perspective (floor vs 1.2 m).
  - Sent to the user.
- 15:54 — Submitted `pc_cylinder_render` 17802084 (the cylinder's one render job). The envelope check before
  submitting showed 0 `pc_` jobs queued and 8 ledger lines; the ledger now has 9.
- 15:57 — **cylinder video done.** 17802084, 3 m 28 s.
  - Outputs: `cylinder_3d.mp4` (268 frames, 37,872 splats), `cylinder_2panel.mp4`, overview, PLY. The projection
    matches the run (3,791).
  - The orchestrator opened `cylinder_3d_f050.png` and `cylinder_overview.png`: the tall prism leaves the start by
    the bench and goes around the bench's north end to the goal; the title carries the verdict and the claims
    boundary. Overlaps in the overview are draw order, not collisions (replay3d: 0 collisions).
  - Sent to the user.
- 15:58 — **All three robots done.** Each has a REACHABLE, certified, replay3d-passed run and a 3D MP4.
  - Jobs used: 9 of 12 (3 search, 3 run, 3 render); no retries.
  - Nothing committed yet.
