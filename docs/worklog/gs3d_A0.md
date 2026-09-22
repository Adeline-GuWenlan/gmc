# A0 architecture and baseline audit

Work in progress, 2026-09-21/22. Exact base `9ff0d5c2a59c95a4444e45878ef878c8d9837108`;
starting plan commit `1d5891a`. No predecessor progress/standup existed; clean checkout.

Audited fixed-band projection, Pose2 export, robot dimensions, timing and full 3D input
container. Read NeuPAN/Splat-Nav primary papers and pinned source; decisions, citations,
licenses and disjoint downstream ownership are in `docs/gs3d_architecture.md`.
Added interface-only `gmc.gs3d.contracts`: body-centre xyz/yaw, scene/coverage/support,
continuous oracle, bounded planner, goals, trajectory/result and timing schemas.
No planner or production geometry implemented.

Cylinder evidence distinguishes 50M/100M support-call exhaustion from wall exhaustion;
endpoint unknown remains unproven. Opened the existing P5-A-0 UAV splat keyframe
`gmc/results/height/plane/shared_a/video/uav_3d_f050.png`: fixed height above table,
consistent with the old manifest and not variable-z evidence.

Read-only assets: original PLY, decoded original NPZ and existing floor-edited NPZ
are available at the shared absolute paths documented in the ADR. Do not rerun old
builder defaults into those source directories. Missing local sealed Atlas assets
must be reproduced and classified, not presumed exempt.

Pending compute: runtime `compute/A0_baseline.sh` reproduces the full unchanged base
suite and records JUnit, then `compute/A0_scene_audit.py` hashes shared inputs and
checks the proposed under/over witness conservatively in the full 3D support map.
Evidence target: `/scratch/wg2381/codex_jobs/gs3d_20260921/logs/A0/`.
AABB overlap will mean unresolved feasibility, never a collision or an accepted flight.

Next: inspect full-suite failures against 576 pass/25 asset-failure baseline; inspect
fixture audit and refine the placement/design as needed; finalize ADR and report,
verify contracts and clean commit, then write acceptance only with no live owned compute.

Local checks: `py_compile` and `git diff --check` passed. Contract import/default smoke
passed with xyz preservation and original/attained-goal and physical-time field checks;
log `logs/A0/contracts_smoke.log` under runtime root. The first smoke attempt incorrectly
asserted a hardcoded total field count and failed; replaced that test-script assertion
with checks of the semantically required fields. No production change was needed.

## Resume: baseline classified, 2026-09-22

Compute 18230541 is finished (Slurm 20m13s, MaxRSS 3,843,544 KiB). Inspected JUnit
and all 25 failure messages: 601 tests, **576 passed / 25 failed**, no errors/skips;
24 missing-package AtlasAssetError plus one missing sealed generator FileNotFoundError.
Counts agree with exact-base historical result, with no unexpected failures.
Classification: runtime `logs/A0/baseline_classification.json`; raw XML/log retained.
Input hashes and environment are recorded in `logs/A0/assets.json` and the ADR.

The first conservative scene audit did NOT pass: initial low approach s=−.60 through
−.28 had unresolved body/support-box overlap; the rise and high leg both exceeded
.05 m with a .06999999 m lower bound. No collision is inferred from box overlap.
Revised proposal begins at s=−.25, keeping all source obstacles and the original high
endpoint. Added explicit suspension and ceiling-attachment dimensions to the audit.
Submit only `compute/A0_fixture_v2.sh`, not the already-classified full suite again.
It will produce `logs/A0/fixture_v2/scene_audit.json` and `fixture_3d.png`.

Found and corrected a design inconsistency before implementation: ground chassis gap
is .02 m, so UAV's .05 m margin cannot apply to ground support. Ground chassis margin
is explicitly .001 m; all Gaussians remain collision obstacles, while permitted wheel
support contact has a separate nonpenetration test. Also documented that edited floor
tiles use local heights and a top clamp; the fitted plane is not alone terrain evidence.
A3 must check footprint support against that evidence and declare contact travel/slope.

Next: inspect revised audit and OPEN its 3D schematic (not a real-scene render); finalize
fixture manifest inputs in ADR. A2 still owns building/rendering actual edited GS scene,
and A1 owns production oracle; A0 audit is conservative design feasibility only.
