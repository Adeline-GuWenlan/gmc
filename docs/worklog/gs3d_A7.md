# A7 — independent final review (in progress)

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
full-suite/reproduction results remain pending. No `.done.json` has been written.

Reproducer `gmc/experiments/gs3d_final_audit.py` hash-checks immutable original/edited
sources and the accepted A5 raw inputs, traps legacy projection, replans all three
robots (five warm calls plus cylinder tolerance comparisons), compares raw trajectories
exactly with A5, pins newly verified JSON, smooths/revalidates, and regenerates renders.
It records computational evidence separately from final acceptance. The suite classifier
requires both baseline test identities and exact missing-asset failure categories;
new failures cannot inherit a blanket exemption.

Remaining: submit governed job, open/reconcile new renders, inspect metrics/failures,
finish code audit and Chinese final report with timing/quality/limitations/hashes,
audit Git history/content and accept only after all R1–R8 readiness checks pass.
Runner alone publishes and verifies remote refs after acceptance.

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
reads/modules, not source files. Acceptance remains pending those results.

Historical Git audit found all accepted predecessors present, unchanged legacy
source/tests, no introduced blob over 1 MiB and no known credential-pattern match.
The final evidence/report commit still needs an updated clean-state audit.
