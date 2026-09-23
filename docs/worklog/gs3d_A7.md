# A7 — independent final review and publication readiness

Final outcome: R1–R8 readiness checks pass. Three real-scene robots and verified
smoothing reproduce; all **679 distinct repository tests** have passing evidence
after the Atlas path repair. No A7 compute remains live. Runner publication and
remote-SHA verification remain a separate final chain action; A7 does not push.
The Chinese user-facing report is `docs/gs3d_final_report.md`.

## Initial audit and repairs

Read the original R1–R8 before predecessor claims. Inherited clean A6 commit
`726dc09d9250c8152981d5967f3b49b4547c6c40`; no merge conflict. No nested agents,
package installs, dataset writes or publication actions.

Code review found five omissions and added adversarial regressions:

- UAV preparation omitted crop/index wall time; include that span before planner entry.
- Smooth replay matched samples but omitted yaw continuity at joins; derive each
  endpoint yaw from authoritative controls and reject discontinuities/nonfinite limits.
- Control heights alone do not prove a nonlinear ground manifold; require an
  evidenced affine plane or a constant-height bound over the full footprint.
- Ordered smooth flight used chords through samples; use monotone, constant-height,
  laterally enclosed Bezier control hulls with conservative section time bounds.
- Rendered finite UAV bodies used ideal fixture centres; use actual replay rows and
  retain timestamps/world coordinates in the render manifest.

Opened predecessor A5 `integration/render/uav_side.png` and A6
`smoothing/render/raw_vs_smoothed_3d.png`. The low/rise/high sequence and ground
detour are visible, but idealized body markers were insufficient evidence; new
rendered artifacts must be opened after reproduction. The A6 plot is thinned context,
not a full Gaussian collision or flight-control certificate.

Local tiny checks: final-audit, smoothing and integration tests completed successfully;
JUnit/logs at runtime `logs/A7/adversarial.xml` and `adversarial.log`. Governed
full-suite/reproduction results were pending at the initial checkpoint.

Reproducer `gmc/experiments/gs3d_final_audit.py` hash-checks immutable original/edited
sources and the accepted A5 raw inputs, traps legacy projection, replans all three
robots (five warm calls plus cylinder tolerance comparisons), compares raw trajectories
exactly with A5, pins newly verified JSON, smooths/revalidates, and regenerates renders.
It records computational evidence separately from final acceptance. The suite classifier
requires both baseline test identities and exact missing-asset failure categories;
new failures cannot inherit a blanket exemption.

Initial continuation required governed compute, rendered-artifact review, failure
classification and a final report; the results below close those requirements.

## First governed result and expanded asset check

Job **18297240** completed in 31m08s, peak RSS 5,145,952 KiB. All three real
scene trajectories reproduce A5 exactly; fresh smoothing passes complete safety,
kinematics and ordered Bezier gates. Full suite: **654 passed / 25 failed / 0
skipped** (679 total), including **78/78 GS3D**. The 25 failures exactly match
baseline test names and missing-package paths. JUnit SHA-256:
`d3efb058bcddc154efceaa074dc527ff358ba25f71287ae013ece8fb4a4d85bb`.

Opened all six new render views and independently reconstructed the finite UAV
body keyframes from exported timestamps. Side view is useful; wall occlusion
limits oblique/high views and ceiling occlusion makes the ground EWA overview
poor evidence of obstacle proximity. Those limitations are preserved in
`gmc/results/gs3d/final_review/visual_review.json` and the Chinese final-report draft.
All artifact and tested-source hashes were recomputed, not merely copied as claims.

A7 challenged the asset classification: the original checkout actually contains
a readable sealed Atlas package. A read-only `AtlasAssetStore.load_cases` probe
passes pinned source/manifest/record checks for dev_001. Local ignored symlinks
now expose only that package and splatc_gates, without copying datasets. The
supplementary compute script disables bytecode writes and reruns both affected
Atlas test files. Test mutations were inspected: they monkeypatch in-memory
reads/modules, not source files. Acceptance was held pending those results.

Historical Git audit found all accepted predecessors present, unchanged legacy
source/tests, no introduced blob over 1 MiB and no known credential-pattern match.
The final clean-state audit is recorded in runtime `logs/A7/final_audit.json`.

## Final closure

Supplementary job **18301714 COMPLETED** in 25 s (MaxRSS 83,152 KiB); both Atlas
files pass **31/31**, including every previously failing test. No production/test
source changed between the main and supplemental runs. Independent JUnit identity
merging gives **679 distinct passed / 0 remaining failed / 0 skipped**. This is a
full-suite run plus an affected-suite rerun, not one monolithic green invocation.
`gmc/results/gs3d/final_review/resolved_regressions.json` retains all test names,
failure history and both JUnit hashes. Supplementary JUnit SHA-256:
`501d1240f9777b5e78620265b5af2b4e0d155b5fce9279b7324c14d929f7f774`.

Frozen real paths: UAV .65–1.40 m above the floor, exact original cylinder goal
8.583851 m away, supported sweeper 1.60 m. Final smoothing reduces total turn by
48.8% UAV / 72.0% cylinder; clearances remain strictly above .05/.001 m. Timing,
analytic dynamics bounds, all goal-tolerance results, source/artifact hashes,
visual caveats and limitations are committed under `gmc/results/gs3d/final_review/`.
The report explicitly preserves stopped UAV climb corners, tiny cylinder margin,
assumed map coverage, approximate floor contact, slower smooth execution, missing
physical controller/attitude/tracking/traction guarantees, and ceiling occlusion.

Implementation commit `0f738d1f45d1d17d58c28abf8796a51e189641a8`; evidence commit
`d4fc858a0d5532ab1e450320bfca4344986a1f1b`; final closure commit is recorded in
`state/A7.done.json`. All accepted A0–A6 commits are ancestors. Only runner may
publish and confirm `state/publication.json`; `main` remains untouched.

Commands: `compute/A7_final_checks.sh` and `compute/A7_atlas_recheck.sh` through
`submit_compute.py --stage A7`; both use the prescribed interpreter from A7/gmc.
No outstanding failure, blocker, source-data write, package install or scheduler
change remains. No subagents or external collaborator messages were used.
