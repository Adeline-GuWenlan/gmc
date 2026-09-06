# GMC Implementation Guide v0.1 — current conformance report

Date: 2026-09-04  
Specification: `Gaussian_Mobility_Compiler_Implementation_Guide_v0.1.docx`

## Overall judgment

**Partial conformance; the full Guide GOAL remains open.**

The code now implements a conservative 2D hard-support interval compiler and
a three-valued query-refine pipeline. In theorem mode it can certify

`D_safe(I) subset F(theta) subset D_possible(I)` for every `theta` in a full
orientation cover, and therefore a fixed-translation angular connectivity
sandwich

`Theta_safe subset Theta_true_connectivity subset Theta_possible`.

This is not a proof of generic arrangement combinatorics, event completeness,
route/homotopy-class enumeration, general free-final-orientation SE(2)
queries, real 3DGS, or nonholonomic planning.

The frozen Atlas G1 scoped claim is closed by formal HPC job `17653106`.

## Output semantics

| Requirement | Current implementation | Judgment |
|---|---|---|
| `REACHABLE` only after SAFE lift and independent continuous verification | Library `query()` uses its validated in-memory compiler; CLI query first rebuilds that compiler from frozen inputs. In both paths the candidate curve is independently checked, with endpoint, C0, collision, workspace, clearance, support, and wall gates. | Implemented |
| `UNREACHABLE` only from a globally valid possible-space cut | Available only in theorem mode with certified full-circle interval covers, complete possible components, and replayed graph invariants. Prototype/partial cover abstains. | Implemented within supported theorem model |
| Budget/numeric ambiguity returns `UNKNOWN` | Support and wall budgets are hard gates, including post-operation and terminal-commit checks. Expected numeric certificate failures remain unresolved. | Implemented |
| Structural contradiction returns `INTERNAL_ERROR` | Initial graph, refined graph, and formal-cut invariants are replayed. A contradicted SAFE rotation witness is an internal error. | Implemented |
| Failure-as-data | Query ambiguity/refinement trace and configured failure artifacts are saved. Some construction/type errors still raise, as allowed for programmer/input faults. | Substantially implemented |

## M0-M9 mapping

| Module | Status | Current boundary |
|---|---|---|
| M0 input/calibration | Implemented | Immutable supports, SPD/ID/workspace/config validation, round-trip, scale and world-ULP gates. No lockfile/CI claim. |
| M1 pair oracle | Implemented | Exact support value/point; pair-local continuous separation with conservative binary64 error. |
| M2 independent oracle | Implemented | Bracketed Perram-Wertheim validation; solver failure is never interpreted as free. |
| M3 convex sandwich | Implemented in theorem mode | Binary64-normalized directions, support/point error floors, outward export audit, exact-dyadic nesting fallback, and tamper tests. Prototype mode is explicitly uncertified and avoids theorem proof helpers. |
| M4 fixed slice | Implemented | `C+`, `C-`, `F_safe`, `F_possible`, components and optional nerve; Boolean/precision failures fail closed. Broader external component truth remains open. |
| M5 orientation | Dual status | L/M/R event regularity remains only a candidate diagnostic. Separately, interval-wide free-space bounds and full-circle tiling/provenance can be theorem-certified. These must not be conflated. |
| M6 mobility graph | Implemented for current holonomic model | SAFE edges own replayable witnesses; theorem interval covers support a global upper graph. No generic route-class completeness claim. |
| M7 query-refine | Implemented core | Deterministic target selection, atomic slab split/recompile, full derived-graph rebuild, budget/no-progress UNKNOWN, failure dump, and invariant error handling. No separate persistent failed-edge invalidation table. |
| M8 continuous verifier | Implemented | Independently recomputes collision/workspace clearance over translations and rotations and binds exact request endpoints. Local steering is not implemented. |
| M9 dynamics | Partial | Fail-closed checker for an already provided reversible-DD curve. No `M_dyn`, Reeds-Shepp/local steering, or directed planning pipeline. |

## I1-I7 mapping

| Invariant | Status | Evidence boundary |
|---|---|---|
| I1 pair nesting | Strong implementation evidence | Theorem, dense/analytic, large-coordinate, 1-ULP protrusion and direction-tamper regressions. |
| I2 free-space nesting | Strong implementation evidence | Inner/outer precision direction, union/difference and possible-sliver tests. |
| I3 graph nesting | Theorem-mode strong; prototype structural only | Full interval cover and possible-component completeness are required for a formal cut. |
| I4 witness ownership | Query-level implemented | SAFE edges and content-addressed query segment proofs replay. A full-run all-witness index validator is still missing. |
| I5 no silent fallback | Implemented on verdict paths | Certificate, numeric, provenance and budget failures abstain or become internal errors; they do not become positive/negative claims. |
| I6 periodicity | Implemented for interval tiling/provenance | Exact full-circle tiling, seam and refinement revision checks exist. This is not generic event completeness. |
| I7 reproducibility | Explicit partial/fail-closed | Pair-pruning and fixed-direction roles have semantic validators. Formal-event, lineage-rebuild, all-witness and all-query full-run validators are absent and explicitly return false. File aliases cannot satisfy multiple roles. |

## Artifact contract

Schema-v4 compile bundles bind frozen scene, robot, config, ordering, numeric
policy, work ledger, candidate pairs, direction sets, interval tree,
components, graphs and witnesses. Query execution reconstructs the compiler
from frozen inputs rather than trusting GraphML, then writes a
content-addressed `proofs.json` plus independent `verify.json`.

The current toy query artifact is complete for its own verified PoseCurve, but
the compile manifest remains `i7_complete=false`. This distinction is
intentional: one complete query proof does not supply the four missing
full-run replay validators or compile-time figures/raw traces/nerve/oracle
tables/formal event artifacts.

## External evaluation and P5

`splatc_atlas` remains a frozen external input/truth asset. The GMC evaluator
loads all selected supports but recomputes G1 analytic truth from independently
pinned split/robot fields; legacy `splatc.gmc` method code is forbidden from
truth construction. The formal profile locks stage order, exact 42/16 case
IDs, 5-degree tolerance, compiler settings, and per-case budgets. Its public
API has no inherited-token bypass, and the CLI rejects any absent, malformed,
false, or error-bearing gate.

P5 current artifacts show conservative BVH pruning and semantically exact
single-support incremental rebuild. The equal-budget six-case diagnostic is
a strict tie (12/12 events, zero spurious, 788 support evaluations per arm),
so it supports correctness/fairness only and is not formal closure evidence or
an efficiency advantage.

## Current execution evidence

- Local: **353/353 tests passed**.
- Local schema-v4 CLI run:
  `outputs/door_run_local_final_budget_20260903`.
- Toy result: safe `72n/74e`, possible `128n/138e`, `REACHABLE`, independent
  verifier certified, clearance LB `0.02171590806152206`, three segment proof
  objects, query artifact `missing=[]`.
- HPC P5: job `17580448`, completed.
- HPC targeted pre-final numeric: job `17581683`, 83/83 completed.
- HPC final targeted: `17584904`, 107/107, `COMPLETED 0:0` in 7:28.
- HPC final full/toy/formal: job `17653106`, `COMPLETED 0:0` in `05:13:42`
  (CPU `05:11:40`, MaxRSS `898777K`); 353/353 tests and the CLI toy passed,
  and the complete log ends with the completion marker.
- Formal artifact:
  `artifacts/atlas_g1_full_gate_intervals_hpc_17653106.json`; formal profile,
  exact 42/16 case sets, fixed 5-degree tolerance, compiler contracts, and
  budgets validated. The top gate and both ordered stages passed with zero
  budget-exceeded cases and zero spurious brackets.

Jobs `17579708` and `17582190` were intentionally cancelled after subsequent
code-review findings made their snapshots obsolete; neither is a scientific
failure or final evidence. Jobs `17585021` and `17620029` are preserved valid
formal rejections because 14 and 12 refined cases, respectively, exceeded the
unchanged wall budget. Job `17652860` is a passing single-case diagnostic, not
formal closure evidence.

## Closed scoped claim

Job `17653106` satisfies the unchanged terminal, test, toy, profile, case,
metric, and budget gates. Therefore the following scoped claim is closed:

> Frozen non-blind Atlas G1 single-door, 2D hard supports, fixed start/goal
> translations, full S1: GMC certifies
> `Theta_safe subset Theta_true_connectivity subset Theta_possible`; the
> initial 42 cases pass containment and the refined 16-case matrix passes its
> controls and all 14 partial cases with four truth transitions each covered
> one-to-one by UNKNOWN brackets no wider than 5 degrees plus the declared
> floating-point measure tolerance, zero spurious brackets, and
> false-open/false-closed measure within that declared tolerance, all within
> the declared budgets.

Even then, the full Guide remains open for generic arrangement/event topology,
general SE(2), route classes, query-many/break-even, real 3DGS, full I7, blind
evaluation, and the full M9 dynamics pipeline.
