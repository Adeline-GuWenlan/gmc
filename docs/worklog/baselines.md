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
