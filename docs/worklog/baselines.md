# Worklog — `bl` (baselines on F4) chain

## J — judge check (2026-10-09)
- Old-code copy: `git archive 2a68d73 gmc/src gmc/experiments gmc/tests` → `gmc/outputs/baselines/j/old/` (uncommitted);
  data dirs (`configs scenes results hpc pyproject.toml`, `splatc_atlas`) symlinked to the worktree's — unchanged between
  2a68d73 and d757729 (`git diff --stat 2a68d73 HEAD` touches only src/experiments/tests/docs). Old oracle has no
  `contains_swept_cylinder` (grep count 0).
- Soundness of the fix re-read in code: `uavlamp_query.build_scene` sets the scene bounds to the prism's world AABB
  and crops by Gaussian support AABB ∩ those bounds; the oracle caps every clearance by the body-AABB distance to the
  scene bounds (`oracle.py` `boundary`). A body inside the prism therefore cannot meet an omitted Gaussian.
- Only `api.query`'s shared replay (`api.py:417`) uses the changed code inside GMC (grep of `gmc/aerial3d`), so F4
  rows that never reached it (endpoint, TIMEOUT, ERROR) or passed it keep their verdict.
- Jobs: tests 19488705 (head) / 19488711 (old); probe 19488875 (6 pairs WWEST+S: load 1 s, RSS 725 MB WWEST /
  383 MB S, ~0.3 s per pair); re-query 19488879 (11 tasks); main 19488996 (120 tasks: A* + straight, new + old).
- Script: `gmc/experiments/bl_judge_check.py` (plan/astar/straight/requery), `bl_judge_collect.py` (collect).
  Old runs use `python -P` + old PYTHONPATH; every output's `.meta.json` records the imported oracle file and
  whether it contains the fix (`code_label`), and a run aborts on mismatch.
- Results (collect, `results/baselines/j/judge_check.json`): (a) 1 HEAD-only failure,
  `test_route_prism_domain_matches_the_baseline_oracle_in_an_empty_scene`. It pins GMC domain == judge; the new judge
  is looser by r(|c|+|s|-1). `bl_judge_domcheck.py`: GMC-only 0, judge-only 980, all inside the band.
  (b) 5000/5000 (4 needed the exact-pose A* re-run).
  (c) 537/538 REACHABLE; F4X-02145 TIMEOUT (REACHABLE at 121.5 s in a 300 s re-run, job 19490619).
  (d) 368/369 same SHA; F4X-01566 TIMEOUT at 120 s, same SHA at 113.1 s with 300 s.
  (e) 0 disallowed transitions in 199,970 edges; wide crop agrees on 2993/2993 freed edges.
- J's nodes ran ~11 % slower than F4's (median wall ratio 1.107, 66 WWEST-cylinder rows): both timeouts are
  that effect.
- **Gate (a) fails as written → BLOCKED for the user** (prompt hard stop). Recommended option 1 (test-only change)
  in `docs/baselines_judge_check.md` §8. Total 5.59 CPU-h.

## B0 — baseline envs + smoke tests (2026-10-09)
- J standup: RESOLVED (user option 1, 54a4960) → proceeded. Test edit already done by the orchestrator in 54a4960.
- Installs strictly serial, fresh `CONDA_PKGS_DIRS=/scratch/wg2381/.conda/pkgs_bl/<env>/<jobid>/conda`:
  cust_fields 19506169 → foci repair 19506296 (died after its conda step: `set -u` vs base activate.d
  `QT_XCB_GL_INTEGRATION`; scripts now `set -eo pipefail`) / 19506351 → pno 19506371 → splatnav 19506844. No
  cache errors this time.
- libEGL/libGL: conda-forge libegl+libgl in-env; open3d's RUNPATH `$ORIGIN/../../../` finds them, no
  LD_LIBRARY_PATH (checked with `env -u LD_LIBRARY_PATH` + `ldd`).
- foci: `ext_repos/foci_bl` = `cp -a` of the clone (first patch attempt failed: git-lfs not on PATH for the
  smudge filter; nothing was changed; redone with the foci env's git-lfs on PATH) + `foci_mumps.patch`
  (3 hunks); editable install re-pointed.
- Smokes: cust_fields 19506295 (goal reached); FOCI 19506397 (6/6 IPOPT acceptable, curves end 0.2–4.0 m short:
  soft goal); PNO CPU 19506845 FAILED (oneMKL DFTI on stride-0 `expand` input) → contiguous-rfft2 wrapper on CPU
  → 19506958 (rel L2 0.1733/0.1636 vs paper 0.1748/0.1675); PNO cuda + SplatNav 19506846 — SplatNav part INVALID
  (identity quaternions → NaN rotations in SplatNav → wall ignored); fixed scene, rerun 19507292 (3/3 feasible,
  clearance ≥ r, negative control hits). Probe `bl_probe_splatnav_quat.py`: NaN iff vector part exactly 0.
- Budget 2.13 CPU-h, 0.054 GPU-h. Doc: `docs/baselines_envs.md`.

## 2026-10-10 — stage B1 (harness, SplatNav, FOCI, tuning, pilot) — agent job 19507514
- Harness `bl_harness.py` + `bl_worker.py` (method in its own process group, 120 s enforced by the parent via select +
  killpg, no in-process alarms), export = GMC's `_gs3d_result`/`_densify` called directly, judge `replay_plan` in
  gmc-venv. Scene exports (judge's Gaussians, plan frame) 19508155; 10 pure + 4 heavy unit tests pass (GMC
  re-queried through the harness → byte-identical gs3d export and same polyline SHA as its re-judged F4 row).
- astar_replay sanity: 500/500 SUCCESS (tuning + 200-pair F4 sample, both robots, 19508232); J's 4 rounding pairs 8/8
  (exact A* rerun path, 19508644).
- Adapters: SplatNav with a z-squash sphere cover (ellipsoid (R,R,R/ε), lateral 1.07× cylinder) + slab cut, matched
  2σ contract; FOCI with min-volume-ellipsoid robot cov, z band ±1 cm via a verbatim-`__init__` subclass. Probe
  19508258 found: SplatNav infeasible-QP debug dump raising QhullError (→ instance no-op), FOCI ±0.5 m z drift
  (→ band). Probe's 4 FOCI SETUP_FAIL rows came from my mid-run edit of bl_foci.py (probe only).
- Tuning 19508642 (8 candidates/method, pre-registered rule fbdbf18); F1_zband reused a stale artifact (artifact key
  lacked the adapter SHA) → fixed a502739, F1 rerun 19510034. Frozen 0f0e049: SplatNav cyl S6 / sweeper S1; FOCI F3
  both.
- Pilot 19510252: SplatNav 100/100 + 100/100; FOCI 44/100 cylinder (54 margin-grazes on its own curve), 77/100
  sweeper (16 IPOPT max-iter; 6 unsafe on the appended goal segment). Projection: ~0.5 GPU-h per method for 5000×2.
- Budget 4.0 CPU-h, 0.96 GPU-h. Doc `docs/baselines_adapters.md`.

## 2026-10-10 — stage B2 (shared rasteriser, PNO, cust_fields, tuning, pilots, projection) — agent jobs 19511035/19511086/19521346/19523202
- `bl_raster.py`: exact closed-form support of (2σ ellipsoid ∩ body slab) projected to the plane, rows bounded by
  rigorous chord ends (sampled + golden-section angles); known-space and world-AABB workspace layers; floor splats
  count only by their cap above 0.019 m. First version (outer 64-gon) overshot ~a·π/K on long thin splats — caught by
  the tightness unit test before any use. 9 unit tests. Judge check 19518228: 0 unsafe of 9960 non-free uniform +
  108 000 boundary samples (18 region × robot × resolution), conservatism 0.16–2.5 %.
- PNO: only the example-notebook models are published (the planning notebooks' PNO2D checkpoint is not); used
  DEEPNORM2dMultiGoal + FNOSDF inside the paper's heuristic-A* pipeline, zero-shot. Units probe: V × S/64 in cells,
  ≈ 2.8 × distance on an empty room. Tuning: GPU OOM from my 4-stream layout on S = 2048 → stale rows moved,
  P1/P3/P5 rerun unchanged (19518925); S = 4096 does not fit an L40S alone. Frozen P2_S1024_pinn (3a841e9).
  Pilot 19520299: 100/100 + 100/100.
- cust_fields: star world = disjoint squircles / chains (≤ 7) inside one squircle workspace; the repo's
  boundary-attached form crashes (`compute_virtual_ws` TypeError); +0.1 m hidden pad undone. Convex covers merge the
  big rooms into one obstacle; chain decomposition with adaptive tightening keeps 96–100 % of pairs connected except
  WWEST cylinder but the NF descent fails (stuck / max steps / 120 s). Control: our loop reproduces test_nf.py on the
  demo (step 179, 8.150 m); 15/30 on a clean synthetic world. Tuning 1/800 eligible → rule picks C0_convex (cylinder
  by the median-time tie-break). Pilot: 0/100 cylinder, 3/100 sweeper, 0 unsafe.
- Earlier world checks with a nearest-cell snap (`world_check_{convex,chain,chain_w2}.json`) superseded by
  `world_check_C*.json` (segment-checked snap, tightening) and removed.
- Agent allocation 19511086 ended OUT_OF_MEMORY (2 GB) after 2.5 h; resumed by the wrapper, no work lost.
- Projection (4 methods, 5000 × 2): 2.3 GPU-h, 26 CPU-h total; no subset needed. Budget 19.1 CPU-h, 1.01 GPU-h.

## 2026-10-10 — stage B3 (full runs, 4 methods × 5000 × 2) — agent jobs 19525431/19525820/19528751
- Setup artifacts from B1/B2 reused (24/24: SHA = sidecar, path = hash(frozen config, adapter source), setup id = pilot
  rows); plan committed before any row (8c5f2ef): judge inline, 4 streams per L40S, cust_fields CPU array 26 tasks.
- Jobs: FOCI 19525656 (0:40), SplatNav 19525657 (1:54), PNO 19525658 (CANCELLED by uid 0 after 2:08 — 2 GAPW1
  cylinder tasks cut at 702 + 645 rows; resumed by 19528737, 153 rows), cust_fields 19525659 (26/26).
- collect.json ok: 40 000 rows, every cell complete, one setup id + artifact per cell, 0 setup builds, 0 restarts,
  0 ERROR/TIMEOUT. SUCCESS cylinder/sweeper: SplatNav 4958/5000, FOCI 2102/3665 (2884/449 unsafe claims), PNO
  5000/5000, cust_fields 0/113. Spot-check 25/25 agree (only FOCI has CLAIMED_* rows).
- SplatNav full rows (133 MB gz) kept under outputs/baselines/b3_rows/ with SHA sidecars; slim rows committed.
- Budget 29.1 CPU-h, 4.88 GPU-h. Doc `docs/baselines_b3.md`.

## 2026-10-10 — stage B4 (comparison, report, final sweep) — agent job 19529143
- Audit by artifacts: 30 cells (5 methods × 2 robots × 3 regions) complete, 0 missing/dup/foreign, status counts =
  collect.json, one setup id per cell; frozen configs committed 0f0e049 / 3a841e9 / 981f5b6 before B3's first job
  (05:41 EDT), unchanged since; gmc/src 0 lines changed since d757729; harness/adapters/raster/configs unchanged since 8c5f2ef.
- New evidence for "GMC column judged by d757729": all 8970 GMC re-judged REACHABLE routes (stored 0.1 mm) through
  bl_harness.judge_row → 8970/8970 SUCCESS, 0 rounding failures (19529261, 0.29 CPU-h).
- Doctrine check (19529631 → field-name fix 19529693): every baseline row has the F4 pair's exact endpoints, all §5
  fields, exact judged polyline from/to the pair; task summaries carry host.git_commit / setup-once / config SHA / timeout 120.
- bl_analyze.py (analyze 19529399 — its figs step crashed on a double GMC status map, fixed; figs reruns 19529487
  (errorbar float clip), 19529508/19529550/19529577/19529614 (layout, S pick fallbacks, log-axis bars → dots)).
  All 5 figures viewed and checked against analysis.json.
- Headline cylinder SUCCESS: PNO 5000, SplatNav 4958, GMC 4147, FOCI 2102 (2884 unsafe claims), cust_fields 0.
  Sweeper: PNO/SplatNav 5000, GMC 4823, FOCI 3665 (449 unsafe), cust_fields 113. Every GMC failure is solved by some
  baseline; no pair is solved by nobody. Caveats in the report: PNO's 100 % = grid A* completeness on the judge-sound
  raster; SplatNav's rests on our ε-squash cover; timing on unequal hardware.
- Report docs/baselines_f4_report.md (Chinese), measurement sheets docs/baselines_measurement_<method>.csv.
- Budget: 0.36 CPU-h compute + the agent's 1-core allocation (< 1 CPU-h).
