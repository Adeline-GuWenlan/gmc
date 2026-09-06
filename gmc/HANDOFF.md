# HANDOFF — GMC current state (2026-09-04)

## Authority and verdict

The authoritative specification remains
`Gaussian_Mobility_Compiler_Implementation_Guide_v0.1.docx`.

The **full Guide GOAL remains open**.  The implementation now contains the
theorem interval-cover and query-refine mechanisms that were missing in the
2026-09-01 audit, but generic arrangement/event-topology completeness, full I7
replay, query-many evidence, real-3DGS validation, and the full M9 dynamics
pipeline are not complete.

A narrower, preregistered claim is being evaluated separately:

> On the frozen non-blind Atlas G1 single-door family, for 2D hard supports,
> fixed start/goal translations, and the full orientation circle, GMC returns
> certified angular lower/upper bounds
> `Theta_safe subset Theta_true_connectivity subset Theta_possible`.

That scoped claim is **closed** by formal HPC job `17653106`: Slurm reported
`COMPLETED 0:0`, the complete log ends with
`=== GMC_CURRENT_CLOSURE_DONE ===`, all 353 tests and the CLI toy passed, and
the top gate plus both ordered formal stages passed unchanged. The authoritative
result is `artifacts/atlas_g1_full_gate_intervals_hpc_17653106.json`.

## Current verified local state

- Full suite: **353/353 passed** locally on 2026-09-03.
- HPC job `17653106`: **353/353 passed** in `1667.96s`; CLI toy compile,
  query, and independent verification also passed.
- Final schema-v4 toy:
  `outputs/door_run_local_final_budget_20260903`.
- Graphs: safe `72n/74e`, possible `128n/138e`.
- Query `(-2,0,0.3) -> (2,0,0.3)`: `REACHABLE`.
- Independent verifier: `certified=True`, clearance lower bound
  `0.02171590806152206`.
- Query proof profile `query_proof_bindings_v1`: three independently verified
  segment certificates; query artifact `missing=[]`.
- Compile manifest deliberately remains `i7_complete=false` and lists six
  missing full-run artifact/replay classes.  The toy uses prototype envelopes
  and is end-to-end CLI evidence, not theorem P3 evidence.

## Implemented mechanism boundary

- P0/P1: validated inputs, support oracle, independent overlap oracle, and
  cross-platform conservative binary64 sandwich certificates.
- P2: fixed-slice inner/outer free-space bounds and components.
- P3: theorem-mode interval pair certificates, full-circle cover provenance,
  and a certified global possible cover.  Generic arrangement combinatorics,
  critical-event completeness, and route/homotopy-class enumeration remain
  outside the proved result.
- P4: three-valued query, SAFE path lift plus independent continuous replay,
  theorem-only `UNREACHABLE`, query-relevant atomic slab refinement, hard
  support/wall budgets, failure-case artifacts, and invariant-to-
  `INTERNAL_ERROR` handling.  There is no persistent failed-edge invalidation
  table; a contradicted SAFE certificate is an internal error and other
  ambiguity is handled by refinement plus full derived-graph rebuild.
- I7 tranche: pair-pruning replay, fixed-direction tree, interval cover tree,
  components, lineage/witness records, and content-addressed query proofs are
  serialized.  Four full-run independent replay roles deliberately fail
  closed, so full I7 is not complete.
- M9: only the fail-closed verifier for an already supplied reversible-DD
  curve is implemented.  No `M_dyn` builder, directed planning integration,
  Reeds-Shepp, or local steering is present.

## P5 evidence

- Local artifacts:
  `artifacts/p5_system_scaling_current_20260903.json` and
  `artifacts/p5_atlas_g1_equal_budget_current_20260903.json`.
- HPC job `17580448` completed; pulled artifacts end in `_hpc_17580448.json`.
- BVH at 8192 supports retained 1024 candidates and pruned 7168; its candidate
  set exactly matched flat enumeration.
- One-support incremental update at N=128 used 180 envelope support calls and
  14 union operations versus 23040/254 locally (23092/254 on HPC) for a full
  rebuild, with semantically identical geometry.
- Equal-budget G1 diagnostic: both arms found 12/12 truth events over six
  cases, zero spurious brackets, and used 788 support evaluations.  This is a
  **TIE** and explicitly `formal_closure_evidence=false`; it supports runner
  fairness/correctness, not a method-efficiency advantage.

## Final bug-fix tranche

The 2026-09-03 adversarial pass fixed and regression-tested:

1. x86 binary64 outer-envelope undercoverage caused by platform-dependent
   `longdouble` behavior;
2. a small NumPy/Accelerate GEMM crash in the long path-lifting run;
3. full-I7 role aliasing, where unrelated hashed files could satisfy shallow
   role checks;
4. a public formal-profile token that could bypass the complete 42+16
   configuration check;
5. wall-deadline races before formal `UNREACHABLE` and before any terminal
   query result commit;
6. connectivity classification and artifact post-processing time omitted from
   the hard wall-budget result.
7. evaluator conflation of geometric UNKNOWN-leaf resolution with
   budget-aware completion; budget-stopped cases now remain ineligible for
   PASS, and serialized acceptance is cross-checked against compiler/report
   stop and budget state;
8. quadratic strict-convex containment auditing in the interval certificate;
   it now uses the shared exact fan search with a fail-closed fallback. An
   attempted fixed-precision bulk-union shortcut was found unsound, fully
   reverted, and replaced by a regression guard rather than shipped.

Within the adversarial and regression cases executed in this review, the final
soundness pass did not reproduce another false OPEN/CLOSED verdict, formal
gate promotion, or budget-overrun PASS.

## HPC chain

- `17573066`: diagnostic failure that exposed the x86 envelope issue; fixed.
- `17579708`: deliberately cancelled after the I7 checker flaw changed code;
  not a test failure.
- `17580448`: current P5 HPC PASS.
- `17581683`: pre-final targeted numeric run, 83/83 PASS.
- `17582190`: deliberately cancelled at 44% after final budget/profile bugs
  were found; not a scientific failure and not final evidence.
- `17584904`: final expanded targeted regression, 107/107,
  `COMPLETED 0:0` in 7:28 (MaxRSS 214238K).
- `17585021`: valid formal rejection; 14 refined partial cases exceeded the
  unchanged 1800-second per-case wall budget.
- `17620029`: valid formal rejection after the first repair; 12 refined cases
  still exceeded the unchanged wall budget. Its failed JSON is preserved.
- `17652860`: formal-config `dev_016` diagnostic PASS in `748.512s`, with 4/4
  events and brackets, maximum bracket width `4.21875` degrees, zero spurious
  and false-open/false-closed measure, and no budget exceedance. Diagnostic
  only, not the final scientific result.
- `17653106`: final formal PASS, `COMPLETED 0:0` in `05:13:42` (CPU
  `05:11:40`, MaxRSS `898777K`). The run passed 353/353 tests and the CLI toy;
  the complete log ends with the completion marker.

Formal thresholds, case lists, compiler settings, and budgets must not be
relaxed to obtain a PASS.

## Closed Atlas G1 scope

The unchanged formal profile `atlas_g1_full_gate_intervals_v1` and stage order
were validated. The top gate and both stage gates passed. Initial 42/42 cases
passed both containment directions and the structural/measure checks. Refined
16/16 cases passed acceptance and resolution; the two controls passed, and all
14 partial cases covered 56/56 analytic transitions one-to-one with 56/56
UNKNOWN brackets. Maximum merged bracket width was
`4.921875000000009 < 5.0` degrees. Spurious brackets, false-open measure, and
false-closed measure were all zero.

No budget was exceeded. The largest case wall time was
`1766.008213374007 < 1800` seconds (`val_004`); its support-value and
support-point counts, also the stage maxima, were `33,818,984` and `5,245,116`
against 50,000,000 limits. Maximum refinements were 48 against 128. These
numbers come from the final formal artifact, not the diagnostic probe or either
failed predecessor.

Do **not** infer generic arrangement completeness, general SE(2) reachability,
real 3DGS, query-many advantage, full I7, blind-split performance, or full M9.
