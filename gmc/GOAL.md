# GOAL — current acceptance state (2026-09-04)

The authoritative requirement is
`Gaussian_Mobility_Compiler_Implementation_Guide_v0.1.docx`.

## Verdict

**The full Guide GOAL is not closed.** The implementation now includes the
theorem interval-cover and query-refine mechanisms missing from the 2026-09-01
audit. What remains open is broader than a bug fix: generic arrangement and
critical-event completeness, four full-I7 independent replay roles,
compile-once/query-many validation, real-3DGS evidence, and the complete M9
dynamics graph/planner.

The narrower **Atlas G1 fixed-translation claim is closed** by formal HPC job
`17653106`. This closes only the frozen non-blind G1 scope, not the full Guide.

## Current executable evidence

- Local full suite: **353/353 passed** on 2026-09-03.
- HPC job `17653106`: **353/353 passed** and the CLI toy passed before the
  unchanged formal 42+16 gate ran.
- Final local CLI run:
  `outputs/door_run_local_final_budget_20260903`.
- Compile: safe `72n/74e`, possible `128n/138e`.
- Query `(-2,0,0.3) -> (2,0,0.3)`: `REACHABLE`.
- Independent replay: `certified=True`, clearance lower bound
  `0.02171590806152206`.
- Query proof object: profile `query_proof_bindings_v1`, three verified segment
  certificates, `artifact_scope.missing=[]`.
- Compile manifest: schema v4 and honestly `i7_complete=false`; its six listed
  omissions are compile/full-run omissions, not missing query proof files.

The toy uses `certificate_mode: prototype`. It tests the CLI and proof replay
chain; it is not evidence for the theorem-mode interval claim.

## Stage status

| Stage | Status | Evidence boundary |
|---|---|---|
| P0 Definitions | Implemented | Input/SPD/unit/serialization and binary64 representability gates are tested. No repository lockfile/CI claim. |
| P1 Pair kernel | Implemented | Support oracle, independent Perram-Wertheim oracle, theorem inner/outer envelopes, exact nesting fallback, and cross-platform error direction are tested. |
| P2 Fixed slice | Implemented core; broader validation open | Dual free-space slices/components are implemented. Atlas G1 is one external analytic family, not a complete raster/exact matrix. |
| P3 Orientation | Theorem interval cover implemented; generic event topology open | `D_safe(I) subset F(theta) subset D_possible(I)` is certified over full interval covers. This does not enumerate all arrangement/topology events or route classes. |
| P4 Mobility/query-refine | Implemented core | SAFE path lift and independent continuous replay; theorem-only global `UNREACHABLE`; atomic targeted refinement; hard budgets; failure artifacts; invariant errors. Persistent failed-edge invalidation is not a separate table. |
| P5 Scale | Current engineering evidence complete | BVH and incremental rebuild are conservative. Equal-budget Atlas diagnostic is a tie, so there is no efficiency-advantage claim. |
| M9 Dynamics | Partial checker only | An already supplied reversible-DD curve can be checked fail-closed. `M_dyn`, directed planner integration, Reeds-Shepp/local steering are absent. |
| I7 Reproducibility | Tranche complete; full I7 false | Pair pruning and fixed directions have semantic replay; interval tree/components/lineage/witness/query proofs are serialized. Four full-run replay roles deliberately return false until independent validators exist. |

## P5 result boundary

Current local and HPC artifacts are under `artifacts/` with
`current_20260903` and `hpc_17580448` suffixes.

- BVH, N=8192: 1024 candidate pairs, 7168 pruned, exact flat candidate-set
  agreement.
- Incremental, N=128: 180 support calls and 14 union operations versus full
  rebuild 23040/254 locally and 23092/254 on HPC; output geometry is
  semantically equal. The small cross-platform work-count difference comes
  from the adaptive numeric schedule and is not a bitwise-work claim.
- Equal-budget G1: both arms, six cases, 12/12 truth events, zero spurious,
  788 support evaluations. The report says
  `formal_closure_evidence=false`; the outcome is **TIE**.

## Final 2026-09-03 corrections

The latest adversarial review found and fixed eight concrete implementation or
evidence bugs: x86 outer-envelope undercoverage, long-run Accelerate GEMM
instability, I7 role aliasing, a formal-profile API bypass, query terminal
wall-deadline races, uncharged connectivity artifact post-processing, a
geometric-leaf-resolution versus budget-aware-completion mismatch in the
evaluator, and the strict-convex containment hot path. Budget-stopped cases
remain ineligible for PASS, and serialized acceptance is cross-checked against
the compiler/report budget state. The containment speedup uses an exact fan
search; an unsafe fixed-precision bulk-union shortcut was rejected, fully
reverted, and guarded by regression tests. The new attacks are included in the
353-test suite.

## Scoped formal gate closure

HPC job `17653106` completed `COMPLETED 0:0` in `05:13:42` (CPU
`05:11:40`, MaxRSS `898777K`). Its complete log ends with
`=== GMC_CURRENT_CLOSURE_DONE ===`; the same run passed all 353 tests and the
CLI toy. The formal result is
`artifacts/atlas_g1_full_gate_intervals_hpc_17653106.json`.

- Formal profile `atlas_g1_full_gate_intervals_v1` was validated unchanged;
  stage order was `initial_42_soundness`, then `refined_16_acceptance`.
- The top formal gate and both stage gates passed with empty error lists.
- Initial stage: exactly 42 declared cases; all 42 passed both containment
  directions and the structural/measure contract. No case exceeded a budget.
- Refined stage: exactly 16 declared cases; all 16 passed acceptance and
  resolution. The 14 partial cases covered all 56 analytic transitions
  one-to-one with 56 brackets; the maximum merged bracket width was
  `4.921875000000009` degrees under the fixed 5-degree tolerance.
- Spurious brackets, false-open measure, and false-closed measure were all
  zero. Budget-exceeded cases were zero. The largest per-case wall time was
  `1766.008213374007 < 1800` seconds; the largest support-value and
  support-point counts were `33,818,984` and `5,245,116`, each below the
  declared 50,000,000 limit.

Jobs `17585021` and `17620029` remain preserved as valid formal rejections:
14 and 12 refined cases, respectively, exceeded the unchanged 1800-second
wall budget. Diagnostic job `17652860` subsequently passed `dev_016`; it was
diagnostic evidence, not the formal result. No tolerance, case list, compiler
strength, or budget was relaxed for the passing run.

## Work still required for the full Guide

1. Generic arrangement/event-topology completeness and broader P2/P3 truth.
2. Four missing independent full-run I7 replay validators.
3. Compile-once/query-many throughput and break-even evaluation.
4. Real 3DGS import/geometry/robot truth validation.
5. If M9 is claimed, implement `M_dyn`, constrained local steering, and
   directed query integration.

See `HANDOFF.md` for exact jobs, artifacts, and historical failure boundaries.
