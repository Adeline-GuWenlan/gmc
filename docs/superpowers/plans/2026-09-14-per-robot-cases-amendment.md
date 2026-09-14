# Amendment 2 — one case per robot, and 3D point-cloud videos (2026-09-14)

**Status:** approved by the user on 2026-09-14 (early morning EDT) in an interactive session, after reading
A4's standup. This amends `docs/superpowers/specs/2026-09-12-height-band-robots-design.md` and Amendment 1
(`2026-09-13-floor-rule-amendment.md`). Everything not named here is unchanged.

The user's choices, verbatim in substance:

1. Drop the shared case. Each robot gets its own window, start and goal, chosen so that it can succeed, and
   runs once to success.
2. Video: a 3D point-cloud animation (matplotlib, real splat colours), not a photoreal 3DGS render.
3. MP4 through `imageio-ffmpeg`, installed into the user's `gmc-venv` (done: ffmpeg 7.0.2).
4. C2 17731929 cancelled.
5. The compute envelope below may be used without asking again.

## Why

After the floor rule, no start/goal pair satisfies case criteria 1–3 together (A4: 0 under tables A/B/C/G,
0 of 1,296 scene-wide). The user wants one successful certified run and a 3D video per robot now.

## Claims boundary (must appear in every report and video caption)

- These are **per-robot showcase cases chosen for success**. They are **not** a morphology comparison at one
  place, and no "body shape changes the route" claim may rest on them.
- Morphology contrast may be shown only with maps of **one** window for all robots (e.g.
  `results/height/showcase/figs/diag/a4_case_certified_check_tableA_mid.png`).
- The floor-surface rule (Amendment 1) is on, and is stated next to the claims.

## Frozen

Scene definition (`showcase_scene.load_processed()` with its default floor rule), τ = 0.3, ρ = 2, the robot
table (spec §3.3, including the UAV z_c rule when flying over an overhang: z_c ≥ overhang top + 0.25 m and
z_c + 0.10 ≤ lowest ceiling-side obstacle − 0.30 m), `configs/height_showcase.yaml`, GMC, `verify_curve`,
`replay3d`. The route must not pass glass-adjacent openings (glass doors at x ≈ −3.9, y ≈ 0.9–2.6).

## Success

`REACHABLE` and `verify.certified` and `replay3d.passed`. Anything else is reported as it is. On `UNKNOWN`, one
budget-only retry (`BUDGET=2`) is allowed (spec §5.4); never change scene, robot, τ, ρ or filters.

## Case per robot

Try the preferred behaviour first; if none is found, take any case that will plausibly succeed, and label which.

| robot | preferred | fallback |
|---|---|---|
| sweeper | straight start→goal passes **under** an overhang (bench seat, table) that blocks the cylinder band | any connected pair |
| uav | straight start→goal passes **over** a table/counter, z_c by the §3.3 rule | any connected pair at z_c = 1.20 |
| cylinder | straight start→goal is **blocked** in its own map, start and goal in the same free component (detour) | any connected pair |

**Pre-check on the certified map, not the AABB aid** (pattern: `/scratch/wg2381/splathjb/gmc/outputs/height/a4/case_certified_check.py`):
`project_scene` for that robot on the window → `showcase_scene._support_raster(s2, window, 0.025)` →
distance transform → start and goal clearance ≥ r + 0.05 m, and start and goal in the same connected component
of `dist > r`. Window ≤ 8 m × 8 m and as small as practical. Record the support count: compile cost grows fast
with it (toy: 74–124 supports compile in 6–9 s; the 484-support door took 138 s; the cylinder map of the SW
bench window had 200,329). `showcase_run.py`'s probe aborts above 20 projected hours; if so, pick a smaller or
emptier window (spec §5.4: shrink the window, never the semantics). The grid coreset (Amendment 1 F3) stays
approved but is not the first resort; tell the orchestrator before starting it.

## Compute envelope (user-approved)

| stage | per robot | resources |
|---|---|---|
| case search | 1 job | 8 CPU / 64 GB / ≤ 4 h (`gmc/hpc/percase_search.sbatch`) |
| planning | 1 job, plus 1 budget-only retry on UNKNOWN | 8 CPU / 64 GB / ≤ 24 h (`gmc/hpc/percase_run.sbatch`) |
| render | 1 job | 4 CPU / 32 GB / ≤ 4 h (`gmc/hpc/percase_render.sbatch`) |

- At most **3** jobs of this effort in the queue at once (job names start with `pc_`), at most **12** in total.
  Before each submit: `squeue -u wg2381 -h -o %j | grep -c '^pc_'` must be < 3, and
  `/scratch/wg2381/claude_jobs/percase/jobids.txt` must have < 12 lines. Append `<jobid> <job-name> <utc time>`
  right after submitting.
- Everything heavy goes through sbatch; nothing that loads the 7.3 M-splat scene runs on the login node.
  Poll with `squeue -j` / `sacct -j`, never `srun` or `--wait`. `scancel` only IDs from the ledger that you
  submitted yourself.
- No installs.

## Video (spec §6, amended)

Per robot, MP4 (≤ 300 frames): opaque splats of the window dilated by 1 m, cropped to ≤ 2.5 m above the floor
so the ceiling does not hide the view, subsampled for speed, coloured by spherical-harmonic DC colour
`rgb = clip(0.5 + 0.28209479·f_dc, 0, 1)` read from the raw PLY through `scene.ids` (processed ids index the
raw vertex rows; the gravity rotation is the identity for this scene). The robot is drawn as its prism
(footprint × band) moving along the certified path, with a trail; an oblique camera that slowly orbits or
follows. The title carries robot, verdict, replay3d lower bound and the claims-boundary line. The existing
two-panel `viz.robot_video` output is produced as well.

## Layout

- Worktree `/scratch/wg2381/splathjb-percase`, branch `height-percase` (from `height-bands` at `074dc89`).
- Code: `gmc/experiments/percase_*.py`, `gmc/src/gmc/height/viz3d.py`; sbatch in `gmc/hpc/percase_*.sbatch`.
- Results: `gmc/results/height/percase/<robot>/{case.json, search.json, <run>.json, figs/, video/}`; raw GMC
  output under `/scratch/wg2381/splathjb/gmc/outputs/height/showcase/percase/`.
- Worklog: `docs/worklog/height_percase.md` (append-only, timestamped).
- Sub-agents do not commit or push; the orchestrator commits.
