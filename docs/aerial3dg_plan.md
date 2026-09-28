# aerial3d-ground: extend the pure-3D GMC backend to sweeper and cylinder, demo + 5000-pair timing

Written 2026-09-28 02:xx EDT by the interactive session, before any agent touched this branch. This is
the plan document the chain's `_rules.md` and stage prompts point at. Front-loaded facts below exist so
no stage burns a window rediscovering them.

## 0. What the user actually asked for (verbatim, 2026-09-28)

> assign agents to further work on the aerial3d to make sweeper and cylinder running with demostration
> video and graph, 确保起点和终点不要踩在一起，第一个起点和终点的距离至少要大于3米，最好能够体现出对于
> 不同体型的机器人产生了不同的策略，比如说一种要绕行，一种可以穿行。第三，做5000对采样，然后并且要做
> 先compile后用存储的Cache进行重复计算来节省时间的操作。然后用好一点的agents，并且要关注刷新时间。
> 每做一点就先推一点，最好要在明天早上8:45之前把这个起点终点距离大于3米的3D GMC风格的pair跑通，并且
> 给出比较好的时间估计。dont over engineer give clear context and goal use opus high and use the
> /scratch/wg2381/splathjb/measurement_template.csv 作为测试文档。

Deliverables, in the user's own priority order:
1. **Extend `aerial3d`** (the pure-3D, no-projection GMC-style backend already built and proven for the
   axisymmetric UAV in the `uav-conn` chain) so it also runs the **sweeper** and **cylinder** robots.
2. **Demonstration video + a graph/chart** for each robot.
3. **Start ≠ goal, straight-line distance ≥ 3 m**, for the first (and every) pair.
4. **Best-effort: show morphology-dependent strategy** — one robot detours, one threads through. This is
   a hope, not a requirement to force. If the geometry does not actually produce a contrast, say so
   honestly rather than stage one (§3 explains why the scene should make this likely without forcing it).
5. **5000-pair sampling**, both robots.
6. **Compile once, cache it, answer repeated queries from the cached compile** — this is the main
   technical point of the exercise. Ground-5k's GMC runs (2D projected pipeline, different branch) did
   **not** do this and it cost them: recompiling per pair only saved ~25% versus a persisted compile
   (see `docs/worklog/height_timing.md` §2b and the `ground5k-agent-chain` project memory). Do not repeat
   that mistake here.
7. **Good agents, refresh-window discipline, push after every increment** (not just at the end).
8. **Soft deadline: by 2026-09-29 08:45 America/New_York**, have ONE start/goal pair (≥ 3 m) running
   end-to-end in the new pure-3D style for **both** robots, plus a defensible time estimate for the full
   5000-pair run. This does not gate or stop the rest of the chain — G2/G3 continue regardless — but G1
   should reach and push this milestone as its first priority, before polishing anything further.
9. **Fill `/scratch/wg2381/splathjb/measurement_template.csv`** (read-only source; copy it into your
   worktree's `docs/` or `gmc/results/` before filling — never edit the original) from what you actually
   measure. Leave a field `____` only where the pipeline genuinely has no such stage — do not force-fit.

## 1. What already exists — read this before writing any code

- **`aerial3d`** (`gmc/src/gmc/aerial3d/` in the `uav-conn` branch / worktree
  `/scratch/wg2381/splathjb-uavconn`, read-only reference for you): a pair-certified, full-3D backend —
  Gaussian-pair C-obstacles, sandwich-audited outer/inner envelopes, a pair-driven octree, convex free
  cells whose faces are pair support planes, a portal graph, A* search, path lifting, and a continuous
  verifier — **never** projects the scene to 2D. It currently plans only "the axisymmetric UAV"
  (`pairs.py: domain_from_scene` raises `ValueError("aerial3d plans the axisymmetric UAV only")` if
  `body.motion != "uav_translation"`). Read `docs/uavconn_design.md` and `docs/uavconn_report.md` in that
  worktree, and skim `gmc/src/gmc/aerial3d/{api,pairs,cells,octree,envelopes,query,verify}.py`.
- **The reason this generalizes cheaply.** `gmc/src/gmc/height/prism.py: robot_table()` (the frozen
  robot definitions used by every prior chain) gives all three robots **circular footprints** —
  `ellipse_prism(a, b, z_lo, z_hi, name)` with `a == b` in every case:
  - `sweeper`: r = 0.175 m, z_lo/z_hi = 0.02/0.10 m above the floor (half_height = 0.04 m, band centre
    0.06 m — a thin 8 cm-tall disk).
  - `cylinder`: r = 0.30 m, z_lo/z_hi = 0.02/1.75 m (half_height = 0.865 m, band centre 0.885 m — a
    pole spanning nearly the full room height).
  - `uav` (for cross-check against what `aerial3d` already used): r = 0.25 m, z_c ± 0.10 m. `uavconn`'s
    actual C2 body was r = 0.25, half-height 0.10, margin 0.05 (see `docs/worklog/uavconn.md` /
    `_rules.md` in that worktree) — same order of magnitude, good sanity check.

  `aerial3d.pairs.PairSet` already takes exactly `radius` + `half_height` (a vertical-cylinder body) —
  the same parameterisation `PrismRobot` uses. **A ground robot is not a different kind of body, it is
  the same kind of body with its z-window pinned to a narrow band near the floor instead of free to move
  through the whole room.** The generalization is: replace the `motion == "uav_translation"` guard (or
  add a parallel path) with a body/domain that (a) still uses full 3D pair covariances, no shadow/2D
  projection, and (b) restricts the C-space z-extent to the robot's own absolute height band instead of
  the room's full height. Decide the cleanest seam yourselves (a new `motion` tag e.g.
  `"ground_translation"` with a fixed z-window, or a thin wrapper that clamps the domain's z rows) — this
  is a design decision for you to make and record, not for me to prescribe further.
- **Do not touch or reuse `gmc/src/gmc/height/`** (the 2D band-shadow-projection pipeline) as planning
  geometry. It is a different, already-published method (see `docs/worklog/height_timing.md`) and is not
  what "non-projection" means here. You may still reuse `gmc/src/gmc/gs3d/` oracle/geometry code for the
  **independent replay verification**, exactly as `aerial3d` already does for the UAV (final safety check
  = same shared `gs3d` replay, your own verifier is additional, not a replacement).
- **The scene.** Use the same archive `aerial3d`'s UAV work already used:
  `/scratch/wg2381/splathjb-uavlamp/gmc/outputs/uavlamp/scene_v2/uavlamp_scene.npz`
  (SHA-256 `2a3a72d610519095381a83fd741a61f32b420b20f39e0c5f60951e463fd596cc`, manifest next to it, loader
  `gmc.gs3d.scene_uavlamp.load_uavlamp_derivative`). Read-only — never rebuild or modify it. This archive
  is the real 展示厅 3DGS plus the added lamp / light box / bulkhead / soffit / back panel that the UAV
  work used to force an under/over choice (queries M1/M5 in `docs/worklog/uavconn.md`). **This is why a
  pair whose straight line passes near that structure is likely (not guaranteed) to give the contrast the
  user hopes for**: the sweeper's whole body band (2–10 cm above the floor) sits entirely below where the
  soffit/lamp was placed (~90–105 cm, see `gs3d_final_report.md`'s under-lamp numbers in the `uav-conn`
  worktree), so a sweeper can likely walk straight under it with no detour; the cylinder's band runs
  nearly floor-to-ceiling (2–175 cm) so it cannot duck under anything — the lamp/soffit/partition should
  force it around, in xy, since it has no z freedom at all. **Verify this empirically per pair from each
  robot's own pair certificates — do not assume it or stage it.** If a chosen pair does not show the
  contrast, try another before concluding it doesn't exist; if it genuinely doesn't, report that honestly.
- **`measurement_template.csv`** (`/scratch/wg2381/splathjb/measurement_template.csv`, read-only) is a
  Chinese row-per-metric template, one "current value" column per experiment. Suggested (not binding)
  correspondence to `aerial3d`'s stages, for you to correct once you see the real timing breakdown:
  BVH/scene-load ≈ archive load + crop; candidate-pair generation ≈ `pairs.PairSet` construction;
  exact geometry interaction ≈ sandwich-audited envelope evaluation; free/collision domain construction
  ≈ octree labelling + convex free-cell decomposition; adaptive refinement ≈ any octree/cell refinement
  rounds (or "N/A" if `aerial3d`'s ground path has none); graph/operator assembly ≈ portal graph build;
  factorization/preconditioner ≈ N/A (no linear solve here); global solve ≈ portal A*; path extraction ≈
  lifting; final continuous check ≈ your continuous verifier + the shared `gs3d` replay. Fill the
  robot-comparison and 5000-pair sections from real aggregate numbers in G3, not from the single demo pair.

## 2. Known failure mode to avoid (ground-5k)

`ground5k_run._run_gmc` (branch `ground-5k`, unrelated to this chain but same cluster, same user) compiles
its 2D GMC map **fresh for every one of its 5000 pairs**, on a per-pair crop. Measured cost: compile-once
would only save ~25% of wall time there, because most of the cost is in the query itself (see the
`ground5k-agent-chain` memory, 2026-09-26 entry). **Do not build the ground extension the same way.** The
whole point of this chain's compile-once requirement is architectural: for a fixed (scene, robot-body,
query-box) triple, compile the free-volume complex **once**, keep it resident (an in-process long-lived
worker, or persist `CompiledComplex` to disk and reload — your call, record which and why), and answer
every pair in the 5000-pair batch assigned to that (scene, robot) from the one compiled artifact. Prove
this works before scaling: run the same pair twice against one compiled complex and confirm identical
result and no second compile cost, exactly as `uavconn`'s C2 did cold-vs-warm for the UAV
(`docs/worklog/uavconn.md`, cold vs 3 warm calls per query).

## 3. Compute envelope and cluster hygiene

- Account `torch_pr_527_general`, partition `cpu_short` (6 h wall cap, rejects `--time` ≥ 8h).
- **The user's `ground-5k` chain is running concurrently right now** (as of 2026-09-28 02:08 EDT: ~11
  jobs running, ~172 GB mem requested across running+pending). Headroom is real but not generous. Start
  small: probes at 2–4 CPU / 8–16 GB, size up only from a measured MaxRSS, same discipline `uavconn` used
  (probe before committing to a full run). Re-check `squeue -u wg2381 -h -o "%C %m"` totals before
  submitting anything large.
- Your own orchestration allocation is 2 CPU / 6 GB / 6 h — you write code and orchestrate; all real
  compute (archive load, compile, query, render, full test suites) goes through `sbatch`.
- **Push every meaningful increment** (`git push origin aerial3d-ground`), not just at the end of a
  stage. This overrides the older `uavconn` convention of "only the last stage pushes" — the user
  explicitly asked for incremental pushes this time.
- scancel only job IDs you yourself appended to `jobids/<STAGE>.txt`. Never `--all`, never a wildcard.
- Install nothing. Python: `/scratch/wg2381/.conda/envs/gmc-venv/bin/python` (numpy 2.1, scipy 1.15,
  numba 0.61, networkx 3.4, shapely 2.1). Run from `gmc/` with `PYTHONPATH=src:experiments MPLBACKEND=Agg`.

## 4. What "done" looks like for this plan (the final sweep's checklist, G3)

Derived from §0 above, verbatim — not from what G1/G2 choose to prioritize:
- [ ] `aerial3d` runs sweeper and cylinder (full 3D pair-certified geometry, no 2D projection).
- [ ] Demo video + a timing/graph figure for each robot, on a pair that is ≥ 3 m start-goal distance,
      with start ≠ goal (not overlapping).
- [ ] An honest statement of whether the two robots' strategies differed (detour vs thread), with
      evidence (cell labels / clearance along the route), not just an assertion.
- [ ] 5000-pair sampling run for both robots (or a clearly reported, justified partial run against the
      compute envelope and deadline, exactly as `ground5k` reported when it had to seed a prefix).
- [ ] Compile-once-cache-reuse demonstrated and used for the real 5000-pair run, not just the demo pair.
- [ ] Every meaningful commit pushed as it happened, not batched at the end.
- [ ] `measurement_template.csv` filled (a copy in this worktree, not the original) from real measurements.
- [ ] A defensible 5000-pair time estimate existed **before** 2026-09-29 08:45 America/New_York, alongside
      the running single->3m demo pair for both robots.
