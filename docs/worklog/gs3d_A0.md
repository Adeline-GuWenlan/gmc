# A0 — architecture and baseline audit, completed 2026-09-22

A0 delivers the frozen design in `docs/gs3d_architecture.md` and interface-only
`gmc/src/gmc/gs3d/contracts.py`. No planner implemented, source dataset modified,
package installed, or branch published. Exact algorithm base:
`9ff0d5c2a59c95a4444e45878ef878c8d9837108`; initial plan commit `1d5891a`.

## Decisions and handoff

- Preserve `legacy_height2d` as comparison. Production uses complete 3D means/covariances,
  finite cylindrical bodies, xyz/yaw poses, coverage separate from Gaussian occupancy,
  verified swept edges, bounded lazy search, and explicit failure reasons.
- UAV radius .25 m, half-height .10 m; .05 m obstacle margin. Ground dimensions unchanged;
  explicit .001 m chassis margin and supported unicycle motion, no ground z search.
  Floor tiles follow local heights and a top clamp, so fitted plane alone is not support
  evidence. The support protocol includes conservative footprint height bounds.
- Cylinder exact/relaxed goal comparisons retain the original, default relaxation .25 m
  only when explicitly selected, maximum .50 m. Budget exhaustion is not map unknown.
- Freeze result/trajectory/goal/timing fields, seeds/budgets, cold/warm boundaries,
  physical control dt, and declared kinematic limits. Acceleration/smoothing is A6 work.
- A1 owns core oracle/index/search/validation; A2 owns scene edits/manifests/render checks.
  A3 owns robots/goals/runner, A4 timing/harness; A5 integrates them. Full path matrix in ADR.
- Read focused NeuPAN and Splat-Nav papers and pinned source (commits in ADR). Borrow
  body-aware constraints, Gaussian geometry and verified smoothing ideas; no code copied.
  GPL-3.0 and MIT source licenses recorded respectively.

## Evidence and reproducible commands

All runtime paths below are relative to
`/scratch/wg2381/codex_jobs/gs3d_20260921`. Scripts cd into A0/gmc and use
`/scratch/wg2381/.conda/envs/gmc-venv/bin/python`, `PYTHONPATH=src:experiments`,
`MPLBACKEND=Agg`, and one BLAS/OpenMP thread. Future heavy reruns use only:

```bash
python3 "$GS3D_ROOT/submit_compute.py" --stage A0 --script "$GS3D_ROOT/compute/A0_baseline.sh" --cpus 2 --mem-gb 8 --hours 1
python3 "$GS3D_ROOT/submit_compute.py" --stage A0 --script "$GS3D_ROOT/compute/A0_fixture_v2.sh" --cpus 2 --mem-gb 8 --hours 1
```

| Check | Outcome | Evidence |
|---|---|---|
| Full unchanged baseline suite, job 18230541 | 601 tests: **576 passed, 25 failed**, 1200.486 s, no errors/skips | `logs/A0/baseline.xml`, `baseline.log` |
| Failure classification | 24 missing sealed-package AtlasAssetError, 1 missing generator FileNotFoundError; 10 benchmark + 15 gate-interval tests; no unexpected failure | `logs/A0/baseline_classification.json` |
| Assets | Original PLY, decoded NPZ, edited-floor NPZ available; hashes identical across audits | `logs/A0/assets.json`, `logs/A0/fixture_v2/assets.json`; hashes in ADR |
| Legacy cylinder audit | 6.90 m/8.58 m failures exhaust support-call caps; endpoint unknown unproven; larger-map shared case exhausts wall cap | committed `gmc/results/height/plane/long/cyl_ladder/rung*/cylinder*.json`, `shared/cylinder*.json` |
| Initial fixture | **Did not pass** low approach from s=−.60: unresolved support-box overlap; rise/high legs passed | `logs/A0/scene_audit.json` retained |
| Revised fixture, job 18231762 | All three legs pass conservative swept-box bound: **.0804868341 / .0699999900 / .0699999900 m**; no source obstacle removed | `logs/A0/fixture_v2/scene_audit.json` |
| Contract smoke | Imports/defaults, xyz preservation, goal/time fields and conservative support interface pass | `logs/A0/contracts_smoke.log` |
| Final audit | Source/test base equivalence, evidence gates and whitespace check | `logs/A0/acceptance_review.json`, `logs/A0/git_diff_check.log` |

Compute 18230541 used 2 CPU/8 GiB, elapsed 20m13s, MaxRSS 3,843,544 KiB;
18231762 used 2 CPU/8 GiB, elapsed 10s, MaxRSS 3,743,388 KiB. Both finished.
The full suite is classified, not called all-green. No repetition after those results.

Revised fixture: start (8.85,5.55,−.5771749593), low traverse to
(9.30,6.15,−.5771749593), rise to (9.30,6.15,.1728250407), high traverse to
(10.50,7.75,.1728250407). Altitude swing .75 m exceeds frozen .50 m gate.
The new start replaces only an unaccepted fixture proposal, not any legacy goal.
All three light/shade/ceiling supports are included; 45,841 original support candidates
pass covariance validity checks. The enclosure bound covers motion intervals and
omitted supports are outside a proven enclosing crop; this is not just waypoint sampling.

## Visual review and limits

Opened `gmc/results/height/plane/shared_a/video/uav_3d_f050.png`: old UAV flies above
existing table A at fixed 1.20 m, agreeing with its manifest. It is not variable-z evidence.
Opened runtime `logs/A0/fixture_v2/fixture_3d.png`: low/high body rings and the blue
under-light, rise, high path agree with the specified shade/suspension/ceiling dimensions.
This is a 3D design schematic, not an edited GS render; it omits original table geometry.
A2 must build and inspect the actual edited scene, identify full table extents and
freeze its manifest. A1/A5 must validate with the production oracle and planner.
The A0 witness establishes geometric feasibility only, not dynamics/coverage/flight execution.
Ground support, all-robot runs, cylinder fix, timings and smoothing remain downstream tasks.

## Corrections recorded

Initial contract smoke had a wrong hardcoded field count; corrected to semantic field
checks, with no production fix. Git staging initially hit read-only shared metadata;
ordinary escalated approval allowed explicit-path commits. Initial fixture failed its
conservative bound; shortened only its approach and audited again. Design review fixed
UAV margin incorrectly applying to ground support and clarified local floor/clamp evidence.
No external blocker or scheduler change is needed for A0.

Implementation history: `df841b2` (contracts/design), `d7e3360` (baseline classification
and fixture/support corrections), followed by the final A0 evidence/interface commit.
Only runner publication after A7 acceptance is authorized; A0 does not push.
