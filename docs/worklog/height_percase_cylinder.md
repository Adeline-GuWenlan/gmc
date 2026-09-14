# Cylinder (disc r = 0.30 m, band 0.02–1.75 m), Amendment 2 — agent worklog

Plan: `docs/superpowers/plans/2026-09-14-per-robot-cases-amendment.md`. Worktree `/scratch/wg2381/splathjb-percase`.
Ledger `/scratch/wg2381/claude_jobs/percase/jobids.txt`. Times are EDT.

- 09:55 — Resumed from step 1; there was no script, no results and no job. I read the amendment, spec §3.3/§5.3/§5.4,
  the A4 standup, the A2/A4 worklog, `showcase_scene.py`, `showcase_run.py`, `height/{prism,project,band_shadow,run,floor}.py`,
  both sbatch files, the A2/A4 diagnostic scripts, and the UAV search's pre-subset/probe pattern.
  - **Figures opened:**
    - `a2/diag/proj_map_SW.png`: 210k supports come mostly from walls, table B and the bench. Only 2,174 are centred
      below 0.10 m.
    - `a4/diag/case_certified_check_SWbench.png`: the north free area at y 12–14 is not connected to the south one.
    - `a4/diag/clear_map.png`.
  - **Compile-cost data:** toy T2-8 compiled in 0.30 s with 1 interval and 4.5 s with 16, at ≈100 supports. The UAV probe
    had 0 supports, so it gives no rate for this scene.
  - **Checked:** skimage `MCP_Geometric` treats negative costs as impassable.
- 10:06 — **Wrote `gmc/experiments/percase_search_cylinder.py`.**
  - **Global map.** A scene-wide certified raster (`project_scene` on an exact pre-subset, then `_support_raster`,
    2.5 cm). A window's distance map is min(cropped distance, border distance).
  - **Scan.** Windows of 3–6 m on a 0.5 m grid, kept only with floor and ceiling splat coverage and outside the glass
    zone, in ascending order of estimated support count. Per window: pairs in one free component whose straight line
    is blocked by ≥ 0.10 m, with bottleneck clearance from components at thresholds 0.30–0.50, and a blocker counted
    as real only with ≥ 20 supports centred > 0.30 m and top p95 ≥ 0.35 m.
  - **Refinement.** Tightened window around the raster detour (pad 0.75 m), then the amendment pre-check on the
    window's own map, then a full-scene id recheck.
  - **Probes.** `showcase_run`'s probe, spawned and killed after 40 min: top 3 plus a 1 × 1 m scaling probe, with a
    fallback round if all abort.
  - **Checks.** `py_compile` OK after fixing one unclosed parenthesis.
  - **Synthetic smoke test** (login node, 16 s, a 6 × 6 m room with ~3.7k splats) ran end to end with 0 errors:
    - scan-crop raster vs the window's own raster: 0 differing cells;
    - full-scene ids equal;
    - spawned probes returned (1 slab, REACHABLE, 0.05–0.07 s per support on 3–16 supports).
  - **Gap.** No preferred case appeared: the stacked box splats deduplicated to 16 xy shadows, below the real-object
    threshold. I added xy jitter to the synthetic box to exercise that path.
- 10:08 — **Synthetic rerun with the jittered box** (25 s, 0 errors).
  - **Scan:** 30 preferred windows in 7 clusters. Two tightened candidates pass every hard check:
    - clearance 0.575/0.60, same component, bottleneck 0.50, line min dist 0, detour ×1.11/×1.17;
    - blocker: 20–24 supports centred > 0.30 m, top p95 1.22 m;
    - scan-crop vs own-raster diff 0 cells; full-scene ids equal.
  - **Probes** (P1, P2 at 2 × 2 m; P1 scaling at 1 × 1 m): 96 supports in 4.6–4.7 s, 66 supports in 4.0 s; 1 slab.
    The probe status is `INVALID_GEOMETRY`, because `showcase_run`'s probe start and goal lie inside the box; the inner
    compile (4.1 s) ran fully, so the timing is valid.
  - **Opened `precheck_P1.png`** (scratchpad):
    - left: red box ring, the blue start component around it, the straight line through its corner, the orange detour
      around the east side;
    - middle: section with the box splats at s 1.0–1.8 m up to 1.2 m;
    - right: height map of the four faces.
    The figure is correct.
- 10:09 — **Submitted `pc_cylinder_search` as job 17777663** (8 CPU / 64 GB / ≤ 4 h).
  - Envelope before submitting: 0 `pc_` jobs queued, ledger 4 lines. The job was appended to the ledger.
  - Log: `gmc/logs/pc_cylinder_search-17777663.out`.
  - Defaults: scan budget 45 min, probe timeout 40 min, 14 refined candidates, 3 probes plus the scaling probe.
- 10:11 — Job 17777663 started at 10:09 on cs733 and loaded the scene with the floor rule in 10.7 s. The scene-wide
  projection and raster are running. Background watchers are set on the log (scan start or error) and on job end.
  Case helper ready in the scratchpad (`make_case.py`, JSON only, no `overhang` key).
- 15:32 — Resumed after a session restart. **Search job 17777663: COMPLETED** 10:09–10:13, 273 s, exit 0, 0 errors.
  - **Global map:**
    - pre-subset 2,674,453 splats; 2,308,075 supports scene-wide;
    - projection 73 s, raster 40 s; occupied 25.4 %, disc fits in 38.1 %.
  - **Scan:**
    - 78,144 windows, 16,272 after prefilter, all scanned in 22 s;
    - 2,092 windows with a preferred pair, in 40 blocker clusters;
    - estimated supports p5/p50/p95 = 14k/77k/183k.
  - **Refined preferred candidates**, ranked. Every pre-check raster matched the scene-wide crop (0 differing cells),
    and every full-scene recheck gave identical ids.

    | rank | window | supports | blocker | clear s/g | bottleneck | detour |
    |---|---|---|---|---|---|---|
    | P1 | [0.5, 10, 3.5, 13] | 3,791 | SW-hall bench end, top p95 0.43 m | 0.485/0.475 | 0.35 | ×1.31 |
    | P2 | [1.5, 5, 3.5, 9] | 5,329 | unnamed object at (2.65, 6.81), top p95 1.77 m | 0.45/0.475 | 0.375 | ×1.11 |
    | P3 | [−0.5, 10.5, 2, 13.75] | 22,439 | table B end, top 0.86 m | 0.45/0.44 | 0.35 | ×1.06 |

  - **Probes** (2 × 2 m at the midpoint; `INVALID_GEOMETRY` = probe start/goal inside obstacles, compile complete):

    | probe | probe supports | compile (s) | projected (h) |
    |---|---|---|---|
    | P1 | 1,236 | 7.1 | 0.006 |
    | P1 1 × 1 m | 780 | 4.2 | 0.006 |
    | P2 | 9,112 | 42.6 | 0.007 |
    | P3 | 17,752 | 70.4 | 0.025 |

    Compile time is ≈4–6 ms per support and close to linear.
  - **Fallbacks** (connected, not needed): 809 supports [1, 11, 4, 13.5]; 1,444; 2,482; 27,931 (x ≈ 19).
- 15:35 — **Opened `figs/precheck_P1.png`.**
  - **Left:**
    - the red bench block runs diagonally from the south window border to its north tip at ≈(2.2, 11.5);
    - the dashed straight line start (1.21, 10.51) → goal (3.01, 12.51) crosses the tip;
    - the blue start component wraps west and north around the tip, and the orange raster detour follows it;
    - the narrowest neck is at ≈(1.0–1.4, 11.2–11.5), between the tip's halo and three small shadows at
      (1.35–1.45, 11.33).
  - **Middle:** a seat surface at 0.38–0.43 m over s ≈ 0.7–1.4 m and a vertical end support from floor to seat at
    s ≈ 1.4 m. Both are in the band.
  - **Right:** the bench as a ≈0.8 × 1.6 m diagonal slab of 0.4 m tops.
  - **Choice:** P1, behaviour "around", blocker = north end of the SW-hall bench. P3's detour is marginal (×1.06)
    and P2's blocker is unidentified.
- 15:35 — **Wrote `results/height/percase/cylinder/case.json`** from refined index 0 (P1).
  - **Case:** window [0.5, 10, 3.5, 13]; start (1.2125, 10.5125, 0); goal (3.0125, 12.5125, 0);
    z_floor −1.2271749593107995; z_c 1.2 (unused); behaviour "around"; blocking obstacle = north end of the
    SW-hall bench; n_supports 3,791.
  - **Contents:** pre-check stats, the P1 probe, the full-scene check and the claims boundary. There is no
    `overhang` key, so `showcase_run`'s probe centres on the midpoint, as the search probe did.
  - **Validation:** the fields match the search record.
- 15:35 — **Submitted `pc_cylinder_run` as job 17799997** (8 CPU / 64 GB / ≤ 24 h).
  - Envelope before submitting: 0 `pc_` jobs queued, ledger 5 lines. The job was appended to the ledger.
  - Log: `gmc/logs/pc_cylinder_run-17799997.out`. Background watchers are set on the probe line and on job end.
- 15:37 — Job 17799997 is PENDING (Priority); no log yet. Watchers: probe line or error → decide under step 4
  (wait if projected ≤ 3 h); job end → read `cylinder.json` (step 5).
- 15:45 — Job 17799997 started at 15:43 on cs660.
  - **Projection** on the full scene (n_input 7,247,831): kept **3,791**, as in the search; candidate2 45; floored_2d 0.
  - **Probe line:** 1,236 probe supports, 6.6 s, projected **0.0056 h**, abort false.
  - That is ≤ 3 h, so (step 4) I wait for completion; the job-end watcher is running.
- 15:52 — **Job 17799997: COMPLETED** 15:43–15:52, 8 m 31 s, exit 0 on cs660. **Success by the amendment definition.**
  - **status REACHABLE**; `verify.certified` **true**, min clearance **0.0066 m**, reason ok; clearance_lb 0.0066.
  - **replay3d passed:**
    - 534 samples, 12,327 pairs checked, 438 refined, 0 collisions;
    - min clearance lower bound **0.0032 m**, worst at pose (1.052, 11.128), splat 4,507,323 (the west neck
      seen in the pre-check figure).
  - **Timing:** compile 17.3 s (3,791 supports, 1 slab), query 409 s. Probe 6.6 s, projected 0.0056 h.
  - Outputs: `results/height/percase/cylinder/cylinder.json`; raw GMC output under
    `/scratch/wg2381/splathjb/gmc/outputs/height/showcase/percase/cylinder/`. No retry needed and no further jobs.
  - Claims boundary: per-robot showcase case chosen for success, not a morphology comparison; floor rule on.
- 15:55 — **Certified path geometry** (`cylinder.json` result.curve):
  - 5 segments (TRANSLATION/ROTATION); start (1.2125, 10.5125) → west to ≈(0.93, 11.61) → goal (3.0125, 12.5125).
  - xy within x 0.93–3.01, y 10.51–12.51, inside the window.
  - Entirely on the NW side of the straight line, up to 0.95 m off it, around the bench's north end. Control-polygon
    length 3.41 m vs 2.69 m straight (×1.27).
  - **Behaviour "around" held.**
  - Stage done for the cylinder. Jobs submitted by this agent: 17777663 (search), 17799997 (run). Next: the render
    job, owned by the orchestrator/render agent.
