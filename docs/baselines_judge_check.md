# bl J — does the replay-judge fix (d757729) remove exactly the false vetoes, and nothing else?

Stage J of the `bl` chain, 2026-10-09. Plan: `docs/baselines_f4_plan.md` §2. Labels: **[E]** = backed by the committed
file named next to it; **[G]** = inference, not tested. All numbers: `gmc/results/baselines/j/judge_check.json`
(`bl_judge_collect.py`), unless another file is named.

## Verdict (one paragraph)
The fix does what it claims on the F4 data:
- it clears the judge vetoes: 537/538 vetoed GMC rows are now REACHABLE. The 538th passes the judge but needs 121.5 s on J's slower node
  (post-processing time);
- every A* route still passes: 5000/5000;
- no REACHABLE row changes: 369 stratified rows give the same path SHA (one only at 300 s, node speed);
- no collision verdict changes: in 199,970 straight-segment edges judged by both codes, only `map_unknown` edges
  change, and every edge newly called free is also free on a 1 m wider crop of the archive.

**But gate (a) fails as written**, and the chain stops for the user.
- One test, `test_aerial3d_pairs::test_route_prism_domain_matches_the_baseline_oracle_in_an_empty_scene`, passes at
  2a68d73 and fails at HEAD.
- It asserts that GMC's planning domain **equals** the judge's free set.
- The fix made the judge exact, but GMC's domain (`aerial3d.pairs.domain_from_scene`) still uses the old world-AABB
  inset. So there is now a band along rotated box faces where the judge accepts a pose and GMC's domain does not:
  r(|cos θ|+|sin θ|−1), ≈ 0.10 m for the cylinder at 64°.
- The soundness direction holds: no pose in GMC's domain is rejected by the judge **[E]**.
- What to do about the band is a decision about the comparison (GMC is now stricter than the judge it is scored
  by), not a judge bug. Options are in §8.

## 1. What changed and why
Recap of the plan, §2.
- **REPLAY-AABB.** The oracle used to ask whether the world AABB of the swept body lies inside the rotated route
  prism. In a 53–64° frame that box overshoots the body by r(|cos|+|sin|−1).
  - A* never tripped this because it searches through the same test.
  - GMC (and any baseline) plans against the real body, so 524 GMC routes were vetoed.
  - Now `RouteBoxKnownSpace.contains_swept_cylinder` tests the two end cylinders in the route frame. This is exact.
- **REPLAY-RATE.** Float round-off on µs-long turns timed exactly at the limit caused 14 vetoes.
  - Now the rate check allows one ulp of time and two of pose per step.
- **Why the fix is sound (checked in code [E]):**
  - `uavlamp_query.build_scene` sets the scene bounds to the prism's world AABB and keeps every Gaussian whose
    support AABB meets those bounds.
  - The oracle caps every clearance at the body-AABB distance to the scene bounds (`oracle.py`, `boundary`).
  - So a body inside the prism cannot meet a Gaussian that was cropped away.
- **Which F4 rows the fix can affect.** Inside GMC, only `api.query`'s shared replay (`api.py:417`) calls the
  changed code.
  - F4 rows that never reached the replay keep their verdict: endpoint failures, timeouts (alarm before or during
    the replay, which does the same work on every edge the old test passed) and the simplify crash.
  - Rows that passed the replay also keep it: the fix only loosens.
  - The 538 vetoed rows were re-queried.

Old-code comparisons use `git archive 2a68d73` (gmc/src, experiments, tests) under `gmc/outputs/baselines/j/old/`.
- Old-code runs use `python -P` with only the old tree on `PYTHONPATH`.
- Every output's `.meta.json` names the oracle file it imported and whether that file contains the fix
  (`code_label`).

## 2. (a) Full test suite, HEAD vs 2a68d73
Jobs 19488705 / 19488711. Files: `j/tests/junit_{head,old}.xml`.

| | passed | failed |
|---|---|---|
| HEAD (d757729+) | 776 | 26 |
| old (2a68d73) | 773 | 25 |

- 4 tests exist only at HEAD (`test_gs3d_judge.py`, the fix's own tests). All pass.
- 25 failures are identical on both trees, with the same message: 24 × `AtlasAssetError` (sealed atlas round
  absent in this checkout) + 1 × `FileNotFoundError` (atlas file). Pre-existing.
- **1 regression:** `test_route_prism_domain_matches_the_baseline_oracle_in_an_empty_scene`. Its assertion
  `free == (slack > 0)` fails at a pose 30 mm outside GMC's domain that the judge now calls free.
- Follow-up (`gmc/experiments/bl_judge_domcheck.py`; test scene, 20,000 poses; `j/tests/domain_vs_judge_{new,old}.json`) **[E]**:

  | code | in both | in neither | GMC domain only (judge rejects) | judge-free only |
  |---|---|---|---|---|
  | old | 9463 | 10537 | 0 | 0 |
  | new | 9463 | 9557 | **0** | 980 (all within the 82.8 mm band for the test's r = 0.25 at 64.7°) |

So GMC ⊆ judge still holds (soundness), but equality no longer holds (GMC is now the stricter of the two).

## 3. (b) All 5000 F4 A* routes, cylinder, the region's compile oracle
Job 19488996, files `j/astar/{new,old}/`.
- **`route`:** F5 (a) verbatim. The stored A* route as unicycle poses, then `verify_path`, margin 0.001, goal region.
- **`export`:** the same route through GMC's export + `replay_plan`. This is the judge every baseline will face.

| | stored route passes | stored export passes | after exact-pose A* re-run |
|---|---|---|---|
| new | 4996 | 4996 | **5000 / 5000** (both forms) |
| old (context) | 4991 (4 margin, 5 `map_unknown`) | 4991 | not re-run |

The 4 new-code failures (F4W-00211, F4X-00400, F4X-00692, F4X-01125) are `geometry_or_margin_unproven`.
- This is the 0.1 mm rounding of the stored route: the re-run A*'s exact poses pass, with clearance 1.00001–1.0016 mm (> the 1 mm margin).
- Per region: WWEST 2500/2500, GAPW1 1500/1500, S 1000/1000 **[E]**.

## 4. (c) The 538 vetoed GMC rows, re-queried
Job 19488879; files `j/requery/<R>/<robot>/vetoed.jsonl`.
- Same persisted F3 compiles. SHA-256 checked against the sidecar, F3's handoff and F4's `compile_once.json`; it
  matches for all 6 compiles.
- G2 `QCONFIG`, 120 s limit, `aerial3dg_batch.run_task`.
- Compile-once proof: 0 compiles in every task, the same compile id as F4 on every row.

| F4 class | rows | now REACHABLE | other |
|---|---|---|---|
| EXPORT-DOMAIN (REPLAY-AABB) | 524 | **523** | 1 TIMEOUT (F4X-02145) |
| EXPORT-KIN (REPLAY-RATE) | 14 | **14** | — |

- F5's 3 rows that had then failed only REPLAY-RATE (F4S-00504, F4S-00748, F4X-01569) are REACHABLE.
- **F4X-02145** (WWEST cylinder): in F4 it ran 113.4 s, then the replay vetoed it. In J it hit 120.0 s.
  - J's nodes were slower: median wall ratio J/F4 = 1.107 on 66 WWEST-cylinder regression rows; 113.4 × 1.107 ≈ 125.
  - 300 s re-run (job 19490619, `j/requery300/`): **REACHABLE in 121.5 s**, shared replay passed, same compile id.
  - Stage times:
    - F4: shortcut + tighten + merge = 113.1 s, then a replay veto at 0.0 s.
    - J (300 s run): shortcut + tighten + merge = 118.7 s, then a complete replay of 2.5 s.
  - So its time is post-processing (F5's POST-TIMEOUT mechanism), on a slower node, plus a now-complete replay.
  - On F4's node it would have finished in ≈ 116 s **[G]**.
  - Under the 120 s doctrine it stays a TIMEOUT in the re-judged rows (class METHOD-TIMEOUT).
- Not a judge cause.

## 5. (d) Regression: 369 stratified REACHABLE rows
Strata = region × robot × F4 detour band (31 strata, ≥ 3 each, proportional to 360). Files
`j/plan/regression_strata.json`, `j/requery/<R>/<robot>/regression.jsonl`.
- **368 / 369 are REACHABLE with the same `polyline_sha256` and compile id as F4.**
- The one other row, **F4X-01566** (WWEST cylinder), took 105.7 s in F4 and timed out at 120 s in J (the same
  node-speed effect).
  - 300 s re-run (job 19490619): **REACHABLE in 113.1 s, the same `polyline_sha256` (7284681a…) and compile id as F4**.
  - So the judge changes no REACHABLE row. Read strictly at 120 s, (d) is 368/369. The 369th row is a node-speed
    timeout and does not count against the judge. Its re-judged row is F4's (REACHABLE): only vetoed rows are
    re-queried.

## 6. (e) Negative control: the straight start → goal segment, old vs new
Job 19488996, files `j/straight/{old,new}/<robot>/*.jsonl.gz`.
- All 5000 pairs × both robots.
- The path is judged as a whole, and every densified 0.20 m edge is judged on its own by both codes, so a change
  after the first failing edge is seen too.

| | cylinder | sweeper |
|---|---|---|
| edges judged by both codes | 99,985 | 99,985 |
| edges with a changed verdict | 3106, **all from `unknown/map_unknown`**: → free 2993, → occupied 102, → margin-unproven 11 | 0 |
| occupied → free | **0** | **0** |
| any other change (incl. clearance values of unchanged verdicts) | 0 | 0 |
| path verdicts | `map_unknown` → PASS 515, → occupied 68, → margin-unproven 30; all others identical | identical |
| kinematics of the export | identical (5000 pass both) | identical |
| times shrunk by 1e-6 / 1e-8 (real overspeed) | rejected by both codes, 5000/5000 each | same |

**Wide-crop check** (job 19489206, `j/widecheck/<R>.json`). All 3106 edges that left `map_unknown` were re-judged
by an oracle on the archive cropped to the region box widened by 1 m in u and v (z unchanged: route z = world z, so
both codes tested the z faces exactly).
- Every edge the region crop calls free is free on the wider scene: 2993/2993.
- The occupied and margin-unproven ones stay occupied or unproven.
- This is the empirical side of the soundness argument in §1 **[E]**.

## 7. GMC's re-judged F4 rows (the GMC column for every later stage)
Files: `gmc/results/baselines/gmc_rejudged/<R>/<robot>/rows.jsonl` (5000 per robot) and `summary.json`.
- `summary.json` has the same fields as `f4/summary.json`, plus `before_f4` and `transitions`.
- Each row carries `rejudge_source` (`F4` | `J-requery`), `f4_status`, `f4_reason`, `f4_class`, `class` and `group`.

| robot | F4 REACHABLE | re-judged REACHABLE | fail rate | class transitions |
|---|---|---|---|---|
| cylinder | 3614 | **4147** | 27.7 % → **17.1 %** | EXPORT-DOMAIN → REACHABLE 523, EXPORT-KIN → REACHABLE 10, EXPORT-DOMAIN → METHOD-TIMEOUT 1 |
| sweeper | 4819 | **4823** | 3.6 % → **3.5 %** | EXPORT-KIN → REACHABLE 4 |

| region / robot | F4 → re-judged REACHABLE | remaining classes |
|---|---|---|
| WWEST / cylinder | 2049 → 2098 | METHOD-TIMEOUT 195, EP-TOL 206, METHOD-ERROR 1 |
| GAPW1 / cylinder | 1191 → 1237 | EP-TOL 262, EP-GENUINE 1 |
| S / cylinder | 374 → 812 | EP-TOL 188 |
| WWEST / sweeper | 2440 → 2442 | EP-TOL 57, EP-GENUINE 1 |
| GAPW1 / sweeper | 1413 → 1413 | EP-TOL 87 |
| S / sweeper | 966 → 968 | EP-TOL 32 |

- Remaining cylinder failures: genuine 197 (POST-TIMEOUT 194 + F4X-02145's node-speed timeout, SIMPLIFY-ZERODIV 1,
  EP-SLACK 1) and tolerance 656 (EP-FLOOR/EP-SIDE). No export class is left.
- Timing caveat: the 538 re-queried rows' wall times come from J's nodes (≈ 11 % slower than F4's on WWEST,
  see §4). Later stages that compare per-query time should prefer the 4466 + 4996 F4-sourced rows, or state the mix.

## 8. What the user needs to decide (gate (a))
The regressed test pins the old contract: GMC's domain == judge free set. With the exact judge:
- **Sound:** every pose GMC can plan through is judge-free (0 counterexamples).
- **Not complete w.r.t. the judge:** within ≈ r(|cos θ|+|sin θ|−1) of a rotated box face (≈ 0.10 m for the
  cylinder, ≈ 0.06 m for the sweeper at 64° **[G]**, computed, not measured on F4), a baseline may drive where GMC
  cannot.
- On F4's 5000 pairs this cannot cost GMC a pair: every pair's A* route was found under the old rule, i.e. inside
  GMC's domain. It can shorten a baseline's path along the domain limit (S is a thin box) **[G]**.

Options (not taken; the hard stop says the user decides):
1. **Accept the band.** Change only the test, to assert GMC ⊆ judge-free plus the band width. B4 would then name the
   band as a GMC-side conservatism. No re-runs. *(Recommended: smallest change, keeps every number above.)*
2. Make GMC's `domain_from_scene` use the exact rule too: a `gmc/src` change, re-compile + re-run GMC on all 10,000
   F4 queries (≈ 100 CPU-h **[G]**), then redo §4–7.
3. Judge every method in GMC's inset domain (a stricter known space for baselines only). This is not recommended: it
   changes the judge again.

## 9. Jobs and cost
All jobs: `cpu_short`, 1 CPU, named `bl_j_*`; IDs in `/scratch/wg2381/claude_jobs/baselines/jobids/J.txt`.

| job | what | CPU-h |
|---|---|---|
| 19488705 / 19488711 | (a) tests, HEAD / old | 0.74 + 0.74 |
| 19488875 | probe (6 pairs × 2 regions) | 0.03 |
| 19488879 | (c) + (d) re-queries | 1.45 |
| 19488996 | (b) + (e), 120 tasks | 2.52 |
| 19489206 | wide-crop check | 0.04 |
| 19490619 | 2 rows at 300 s | 0.07 |
| **total** | | **5.59 of 20** |

Reproduce from `gmc/` with `PYTHONPATH=src:experiments`:
1. `python experiments/bl_judge_check.py plan`
2. `sbatch hpc/baselines/j_tests.sbatch {head,old} <root>`
3. `sbatch --array=1-120%10 --mem=1500M hpc/baselines/j_py_array.sbatch results/baselines/j/plan/plan_main.txt`
4. `... plan_requery.txt` (11), `plan_widecheck.txt` (3), `plan_timeout300.txt` (2)
5. `python experiments/bl_judge_check.py collect`
