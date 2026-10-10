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
