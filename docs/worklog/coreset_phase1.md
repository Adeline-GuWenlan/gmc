# Worklog — Mobility Coreset, Phase 1–2 (2026-09-04)

Scope: implement the deterministic half of the v2 design revision
(`docs/design/SPLATC_v2_Mobility_Coreset_Design_Revision.md`) — hierarchy,
certified macro sandwich, hand-crafted robot-conditioned coarsener,
compression/topology measurement. No learning. This is the R1 go/no-go:
*does a mobility compression signal exist at all?*

Code lives in `gmc/src/gmc/coreset/` (hierarchy, macro, coarsen), tests in
`gmc/tests/unit/test_coreset.py`, experiments in `gmc/experiments/coreset_*.py`.

## Design decisions that depart from the doc, and why

1. **The coreset emits an ordinary `SceneModel2D`.** The compiler is consumed
   completely unchanged — BVH, pair oracle, slice compiler, slabs, mobility
   graph, query and verify all run without modification. This was worth
   protecting: it means every result is comparable to the uncompressed arm by
   construction, and there is no new code on the soundness path.
2. **A group may map to more than one ellipse.** The doc's risk R2 (over-
   conservative `E+`) is real: one enclosing ellipse per wall group seals every
   doorway. Multi-ellipse falls out of the tree for free — if one ellipse is
   too coarse, descend to children.
3. **Merging is not restricted to hierarchy cuts** (relaxes doc §4.3). The
   bottom-up merge pass produces a partition of the leaves that is not always a
   cut of the tree. Nothing in the soundness argument depends on the partition
   coming from the tree, and the leaf partition remains the fallback.

## Soundness

`outer_ellipse` never returns an uncertified primitive. MVEE (Khachiyan) is
followed by a certificate over 4096 directions **minus a Lipschitz half-step
term**, so containment is proved at every direction rather than only at the
sampled ones; a negative margin triggers radial inflation, then a bounding-disc
fallback, then a hard error. MVEE quality therefore affects tightness only —
never soundness. `test_c1_survives_a_deliberately_loose_mvee` pins this by
crippling the solver to one iteration and requiring the result still certify.

Because obstacles only grow, free space only shrinks, so coarsening can convert
a decidable query to `UNKNOWN` but can never manufacture a false `SAFE`. Every
one of the ~40 measured arms had gate error <= 0, confirming this empirically.

## Three criterion failures, in order

The admissibility rule was wrong three times. Each failure is worth keeping.

**1. Erosion by the robot's minimum half-width.** Rejected merges that changed
the component count of `free.buffer(-b)`. This checks the robot's *most
favourable orientation only*. On the single door it accepted merges narrowing a
0.60 door to ~0.55 — still open at half-width 0.20, but closed at theta=0.45
where the projected half-width is 0.282. Certified gate collapsed 0.510 -> below
0.45 at n=16.

**2. Uniform grid over `[b, a]`.** Fixed the orientation blindness but made
accuracy depend on grid luck: 4 radii scored *better* than 8, because
`linspace(0.2,0.5,4)` contains 0.30 (the door half-width) and
`linspace(0.2,0.5,8)` steps over it. A criterion that depends on the grid
straddling the answer is not a criterion.

**3. Components-only topology signature.** Locating the critical radii by
bisection removed the grid dependence, but counting only connected components is
blind to route classes. On a wall with two doors, sealing one leaves free space
connected — the other door still joins the sides. Measured: the coarsener sealed
the *wide* door while reporting no topology change. Fix: count holes too, i.e.
the `(#components, #holes)` signature — the same `S3` signature the earlier G-H2
experiment used (`00_MASTER_EXECUTION_PLAN.md` Gate G-H2).

**Final rule.** A macro is admissible if its intrusion into free space is
nowhere deeper than `eps`, *or* it leaves the `(components, holes)` signature
unchanged at probe radii bracketing every critical radius found by bisection.

## Gate tolerance must be specified in gate-angle units

`tol_r` is not the unit anyone cares about. For an ellipse robot clearing a gap
of half-width `r`, `dtheta/dr = r / (sin t cos t (a^2-b^2))`, which is 3.35 rad/m
for a=0.50,b=0.20 but **15.45 rad/m** for the nearly circular a=0.35,b=0.28 — on
the same scene. One shared `tol_r=0.004` therefore gave gate errors of -0.0000
and **-0.0794** respectively. `target_gate_tol` is now the primary knob and
`tol_r` is derived per robot at the discovered critical radii. With that
conversion the contract held in 9/9 robot x tolerance cases.

## Results

- Baseline: inherited suite **353/353 passes on py3.13** under the user's
  account (job 16981865). Full compiler recovers the analytic gate to +2e-5.
- **R1 = YES.** Single door, 484 supports -> **60** (8.1x) at gate error
  +0.00002, i.e. identical to the uncompressed compiler; 7.4x fewer support
  evaluations. Below eps=0.002 nothing changes, so 60 is the limit at exact
  accuracy. With the merge pass, the two-door scene reaches 220 -> 5 (44x).
- **Beats the geometry-only control at every operating point.** At matched
  primitive count the uniform cut scores -0.17 rad where the coreset scores
  +0.00002.
- **Conditioning = YES, but only on a multi-scale scene.** `single_door` cannot
  show it: one passage at one critical radius, so one coreset serves every
  robot (the first cross-robot matrix was all-exact and is *not* evidence about
  conditioning — it is a property of the scene). On the two-door scene the
  adequacy matrix is asymmetric: `built_for=small` is inadequate for the large
  robot (it gave away width on the wide door it never needed), while
  `built_for=large` is adequate for both. **Coresets are ordered by robot size;
  one built for a smaller robot is not reusable for a larger one.**

## End-to-end toy (the safety property)

`experiments/coreset_end_to_end.py`. Compile the mobility graph from macro
primitives, query, then hand the returned curve to the independent continuous
verifier built on the **original leaf supports** — the verifier never sees a
macro ellipse, so a coreset that had deleted an obstacle would be caught here.

    coreset      484 -> 28 supports, min_slack +7.27e-05, critical radii
                 [0.30067, 0.30419]  (door half-width is 0.30)
    compile      10.5 s, 28 pairs, safe graph 72n/74e
    query        REACHABLE, clearance_lb 0.021715945291855537
    verify       certified=True, min_clearance 0.021716, against all 484
                 original pairs

The inherited uncompressed run (`outputs/door_run_local_final_budget_20260903`)
reports safe `72n/74e`, clearance LB `0.02171590806152206`, and
`compile_wall_seconds` `153.07`. So the coreset reproduces the **same mobility
graph structure** (72n/74e) and the same clearance to eight decimals, from 28
primitives instead of 484, compiling 14.6x faster.

## Final HPC confirmation (jobs 16982602, 16983029, 2026-09-05)

**Suite: 406/406 pass** on HPC (353 inherited + 53 new coreset gates), py3.13.

End-to-end, both arms in one job:

| arm | n | compile | safe graph | status | clearance LB |
|---|---|---|---|---|---|
| coreset | 28 | 8.9 s | 72n/74e | REACHABLE | 0.021715945291855537 |
| full | 484 | 137.6 s | 72n/74e | REACHABLE | 0.0217159080615251 |

15.4x wall, 14.4x support evaluations. The coreset path was independently
certified against all 484 original pairs (`certified=True`, min clearance
0.021716). Both arms produce the **same** safe graph, 72n/74e.

Cross-robot matrix on single_door with the auto-tuned tolerance: all nine
(built_for, evaluated) entries give n=28 and `vs_full = +0.00000`. So with the
tolerance bug removed, **single_door shows no conditioning at all** -- cleanly,
without the earlier confound. Conditioning is only visible on the two-door
scene, where the adequacy matrix is asymmetric (`small->large` LOST).

### The compression number does not survive the surface regime

| gate_tol | filled (484) | err | surface (124) | err |
|---|---|---|---|---|
| 0.002-0.01 | 28 (17.3x) | +0.00002 | **65 (1.9x)** | +0.00002 |
| 0.02 | 26 (18.6x) | -0.00496 | 30 (4.1x) | -0.00496 |
| 0.05 | 12 (40x) | -0.0688 | 20 (6.2x) | -0.0504 |
| 0.10 | 8 (60x) | -0.1528 | 14 (8.9x) | -0.1137 |

**At exact accuracy the surface-only scene compresses 1.9x, not 17x.** Most of
the headline 17x is interior redundancy that `_fill_rect` creates and real 3DGS
does not have, since its primitives already lie on surfaces. Accepting a 0.005
rad (0.28 degree) gate error buys 4.1x. The honest claim for real data is
therefore "a few x at exact accuracy, ~4x at sub-degree error", not 17x --
pending a real scene, which is the next milestone.

What does hold in both regimes: the coreset beats the geometry-only control at
**every** operating point, often by two orders of magnitude in gate error at
matched primitive count (surface, n=65: +0.00002 vs -0.048; filled, n=28:
+0.00002 vs -0.346), and the control seals the door outright at the aggressive
end while the coreset degrades gracefully.

## Open

- Inner (`E-`) coreset exists (`coreset/inner.py`) and the full sandwich is
  tested analytically, but no *result* yet exercises a compressed
  `UNREACHABLE`: that needs a theorem-mode possible-side compile, and every
  measurement above concerns `D_safe`.
- No CLI integration. The coreset is library + experiment scripts; there is no
  `gmc coreset` subcommand writing a scene bundle for `gmc compile`.
- `coarsen_scene` costs ~15-27 s at 484 supports, dominated by merge-pass
  admissibility tests. That is the number that decides design revision risk R3
  (teacher-label cost): fine for hundreds of teacher cuts, painful past a few
  thousand.
- Ambiguity-driven hierarchy refinement (M4) exists only as the global
  verification/split loop, not as a query-triggered local recompile.
- All scenes are synthetic. The `_fill_rect` families are volume-filled, which
  is unrepresentative of real 3DGS in both directions — see the `surface`
  variant in `coreset_pareto.py`, where dropping interior splats alone is
  484 -> 124 at *zero* gate error.
