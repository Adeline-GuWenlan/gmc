# Height-band robots, Amendment 2 — uav agent worklog

Robot `uav`: disc r = 0.25 m, band z_c ± 0.10 m above the floor. Plan:
`docs/superpowers/plans/2026-09-14-per-robot-cases-amendment.md`. Ledger `/scratch/wg2381/claude_jobs/percase/jobids.txt`.
Claims boundary: per-robot showcase case chosen for success; not a morphology comparison at one place; no
"body shape changes the route" claim rests on it; floor-surface rule (Amendment 1) on.

## 2026-09-14 (EDT)

- 05:57 — Read the amendment, spec §3.3/§5.3/§5.4, `height_A4_done.md`, worklog A2/A4, `showcase_scene.py`,
  `showcase_run.py`, `height/{prism,project,band_shadow,run}.py`, percase sbatch files, A4 `case_certified_check.py`,
  `table_frames.py`, `case_search_table.py`. Ledger empty; no `pc_` jobs in the queue.
  - Toy UAV reference (T2-5): 106 supports, 16 slabs, compile 8.1 s. T2-8 with `initial_intervals = 1`: 0.3 s.
  - `showcase_run.py`'s probe is not wrapped in try/except and projects a 2 × 2 m window around the overhang
    midpoint. Over a table in the UAV band that window may have **zero** supports, so the search job also runs an
    empty-map GMC smoke test and the probe itself.
- 06:0x — Wrote `gmc/experiments/percase_search_uav.py` (design in its docstring):
  - lines per table (A, C, B, G): across the short axis, along the long axis, ±40° diagonals, and parallel
    fallback lines; z_c 1.20–1.60 with step_case's criterion-1 block plus a conservative tabletop top over the
    crossing ± r; 4 windows per line (bbox + 0.7/1.0/1.4 m, square), all ≤ 8 m, none touching the glass zone;
  - pre-check on `_support_raster` of `project_scene` (0.025 m): clearance ≥ 0.30, same component of dist > r;
  - projection on an exact pre-subset (ρ-AABB within window + 2 m, band ± 0.2 m), re-checked on the full scene
    for the top candidates by primitive ids.
- 06:06 — Login-node checks: `py_compile` OK; `--synthetic` smoke (3,949-splat toy table, 6 s, 244 MB, 0 errors):
  section equals `_section`, full-scene ids identical, empty-map GMC compile REACHABLE/certified. Opened
  `precheck_top1` of the synthetic run: raster, height map and section panels render as intended.
  Ranking tie-break changed to prefer a longer table crossing (before the submit).
- 06:07 — Envelope 0 `pc_` jobs / 0 ledger lines. **Submitted `pc_uav_search` 17763665** (ledger line appended).
- (usage-limit pause; resumed ~09:55 on the orchestrator's message)
- 09:57 — **17763665 COMPLETED** (909 s, errors 0). `results/height/percase/uav/search.json`:
  - table A: 66 lines, 62 over-valid (all strict tier); 264 candidates, 264 pass the pre-check (248 "over").
  - table C: 54 lines, 50 over-valid; 416 / 416 pass (200 over). Best C is C-long-36, z_c 1.55 (top 1.28, items on C).
  - table B: 62 lines, 44 over-valid (38 strict); 224 / 248 pass (176 over).
  - table G: 54 lines, 0 over-valid (L-wall in the section); 4 / 216 pass, fallback only.
  - Total pass 908 / 1,144. `_section` equality True for every table.
  - Top distinct lines, all tier 0 "over" across table A at z_c 1.20, top 0.901: A-long-48 (0 supports),
    **A-long-46 (143)**, A-long-49 (233), A-diag-56 (0), A-diag-58 (0). Start/goal clearance 1.00–1.02, the straight
    line ≥ 1.00 m from any shadow, 1 component.
  - Full-scene re-projection: A-long-48 0/0, A-long-46 143/143, A-long-49 233/233, A-parallel-62 0/0, ids identical.
  - Empty-map GMC smoke REACHABLE, certified 0.5. Probe of top1: 0 probe supports, 0.005 s, projected 0 h.
- 09:58 — **Figures opened.**
  - `precheck_top2_A-long-46_zc1.20.png`: certified raster has no red inside the window (5.9 × 3.65 m); the start
    component (blue) covers the whole window except the border. Height map: table A (≈0.8–0.9 m, teal) with the bench
    along its NE side (≈0.45 m); the line runs along table A's long axis from its NW end to past its SE end. Section:
    tabletop 0.75–0.90 m over s ≈ 0.55–3.55 with an end leg at s ≈ 0.6; nothing between 0.95 m and ≈4.6 m (pendant light
    splats at 4.6–4.9, ceiling ≈5.0–5.3); band [1.10, 1.30] empty; top + 0.25 = 1.15 under the band bottom.
  - `precheck_top3_A-long-49_zc1.20.png`: same picture over a 6.6 × 4.0 m window, longer line (4.97 m); section shows both
    end legs, the middle trestle and the bench frame below 0.45 m. No red inside the window either.
  - `overview_tableA.png`: all best-per-line "over" lines green (pass); fallback parallels cyan. Column box at
    (8.3–9.5, 3.3–5.2), walls NW/SW/NE/SE, SE semicircular platform; none on the lines.
  - Not opened: `precheck_top1` (A-long-48, same line as top2) and `precheck_fallback1` (not used).
- 09:59 — **Choice: A-long-46** (cand 185), window [6.0, 5.45, 11.9, 9.1], start (7.049, 8.097), goal (10.871, 6.483),
  z_c 1.20, behaviour "over". Reason: equally clean as rank 1, but with a non-empty map (143 supports), so GMC and verify
  see real obstacles instead of an empty map (orchestrator's guidance). Caveat: its shadows lie at/just outside the
  window edge (no red inside at 2.5 cm), so the path should be near-straight and the video will not show a detour;
  the "over" evidence is table A's top at 0.90 m under the band [1.10, 1.30], checked independently by replay3d.
  Wrote `results/height/percase/uav/case.json`.
- 09:56 — Envelope 0 `pc_` jobs / 1 ledger line. **Submitted `pc_uav_run` 17777506** (`ROBOT=uav`, budget 1, probe on;
  ledger line appended). Waiting for the `probe` line, then completion if `projected_hours` ≤ 3.
- 10:03 — **17777506 COMPLETED** on cs733 (47 s, exit 0). Log `gmc/logs/pc_uav_run-17777506.out`.
  - Projection on the full scene: kept **143** (matches the search), candidate2 9, floored_2d 1.
  - Probe: 0 probe supports, 0.03 s, projected 0.0013 h, no abort.
  - **Result `results/height/percase/uav/uav.json`: REACHABLE; verify certified, min clearance 0.456 m; replay3d passed**
    (574 samples, 0 pairs checked, 0 collisions, lower bound reported 0.05 = the replay margin: no splat of the
    full scene came within the margin of the prism). Compile 0.64 s, query 0.28 s; 143 pairs, 1 slab,
    M_safe = M_possible = 1 node.
  - Path: rotate in place θ 0 → π at the start (disc, no effect), one straight translation (7.049, 8.097) →
    (10.871, 6.483) over table A, rotate back. As expected, no detour: the 143 shadows lie at the window edge.
  - **Success by the amendment's definition.** No BUDGET=2 retry needed; no further jobs submitted.
  - Claims boundary: per-robot case chosen for success, not a morphology comparison; floor rule on. The "over"
    reading rests on the section (table A's top 0.90 m, band 1.10–1.30 m empty) and on replay3d finding no splat
    within 0.05 m of the prism along the path.
