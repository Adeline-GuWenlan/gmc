# A1 — direct 3D core

Started from accepted A0 `1bac9e2b206957d0f8a494df6c192d36a6b52a17` on 2026-09-22.
No previous A1 progress/standup existed; no merge conflict or applicable AGENTS.md.
Read runtime PLAN, repository plan, README, A0 report/standup and frozen architecture.

Implementation in progress. Full covariance ellipsoids and upright finite cylinders are
the geometry authority. Swept linear translation is exactly the Minkowski sum of a
cylinder and segment, enabling a separating-support bound over the whole closed edge.
Numerical nonconvergence will fail closed. Synthetic rendering/benchmarks go through
the bounded compute helper; scalar success alone will not establish visual agreement.

Implemented separate geometry, BVH, oracle, sparse planner and validation modules.
Preparation validates all input covariances in chunks, outward-floors nonnegative
tiny eigenvalues with counts, and snapshots opaque full 3D supports. BVH queries
retain oversized/distant-centred ellipsoids. Analytical support planes bound the
whole swept finite cylinder; GJK only proposes directions, and unresolved cases
stay unknown. Known space, support, workspace/body extent and search budgets are
separate authorities. Search has exact connectors and reachable goal-region tests;
ground output realizes xy edges using stopped unicycle rotations and translations.

Initial tiny test collection caught one unmatched parenthesis in the new test file;
fixed immediately. Focused local geometry check then passed **8 tests**, 10 deselected,
in 1.37 s (endpoints/vertical crossing, tangent margin, rotated covariance and counted
outward floor). This is provisional unit evidence, not stage acceptance. Full focused
tests and scaling remain submitted-compute work after opening the 3D artifact.

Prepared synthetic fixtures, tests and `experiments/gs3d_core_smoke.py`. First compute
produces analytic 3D ray renders of the exact ellipsoids and finite UAV cylinders,
interactive Plotly scene and native PLY support/body mesh. It preserves the actual
planner output; failure cannot manufacture a path. No source dataset is used.

Visual compute **18234289** completed in 31 s, MaxRSS 276,112 KiB. Opened all three
analytic ray renders (`logs/A1/under_over_{perspective,side,overhead}.png`) on resume.
The side camera shows finite cylinders below the hanging orange support, then an
ascent and high passage above the brown support. Perspective/high cameras confirm
body volume and the obstacles' lateral extent. Read the scalar evidence only after
opening the images: 1.0 m altitude range, 4.01421 m path, .08089095248 m continuous
clearance lower bound, 46 expansions/946 oracle calls/74 narrowphase pairs. This is
synthetic evidence, not the real showcase. `logs/A1/visual_review.json` records the
inspection; `visual_artifacts.json` hashes the PNG/HTML/PLY artifacts.

Safety review found a potential roundoff weakness for highly anisotropic covariance:
direct `n.T @ C @ n` can cancel before the support square root. Added an absolute
quadratic evaluation-error allowance, interior support proposals and their error bound
for overlap witnesses; outward flooring now uses a representable diagonal increment
and records the maximum addition. Tiny analytic checks passed **4 tests**, 18 deselected,
in 1.48 s (`logs/A1/roundoff_unit.log`): exact sphere/cylinder distances, rotated support
normal construction, severe cancellation and prepared-array isolation. Also reject
nonfinite kinematic limits and include final result assembly in algorithm wall time.
Focused submitted suite and scaling remain outstanding. The checks runner asserts
that numerical hardening leaves the inspected .20 m route unchanged; any changed
route requires a new visual inspection.
