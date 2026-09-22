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
