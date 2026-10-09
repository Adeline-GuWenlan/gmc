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
