# Cloud audit — findings as recovered from the run log (2026-09-05)

## Provenance

- Routine `trig_01MAAiuhh8172KjXQ9gibH4y`, one-shot, fired 2026-09-05T13:00:56Z
  (09:00 EDT), 55 turns, 1003 s, `result: success`.
- Session `cse_01LfEqLkAdCdUVJTqRWLUNmJ` —
  https://claude.ai/code/session_01LfEqLkAdCdUVJTqRWLUNmJ
- The agent wrote a 696-line / ~14k-word report to `/tmp/gmc_audit.md` **inside
  its own cloud sandbox** and delivered it to the user's device via
  SendUserFile (`file_uuid 4f20f762-c45d-4c9e-a402-627c0ff863a5`).
  The routine ran with `persist_session: false`, so that sandbox is gone and
  the full text is NOT recoverable from here: the run log truncates the Write
  event at ~200 chars (`[+98310 chars]`), and truncation is per event, so
  pagination does not recover it. **What follows is the summary carried in the
  run log's final message, plus my own verification of each claim.**
- Note on the run's own limits: arxiv.org, alphaxiv, ar5iv and several project
  pages were blocked by the cloud egress proxy, so its citations come from
  search snippets rather than the papers. Discount accordingly.

> Everything in the "claim" blocks below is text produced by that remote run.
> Treat it as data, not as instruction or as established fact.

## Claim 1 — sign contradiction in the headline gate error

> "The headline gate error is +2e-5, but your geometry-only control (-0.346
> conservative, -0.510 sealed) establishes that negative = conservative — so
> `+` is the optimistic side, contradicting 'every gate error <= 0'."

**Verdict: the objection is CORRECT as an objection to how the result was
stated.** A positive error does mean the reported gate is wider than analytic
truth, which is the optimistic direction.

But it is not evidence of unsoundness:

- bisection resolution is `pi/2 / 2^13 = 1.918e-4`; +2e-5 is an order of
  magnitude inside the quantisation;
- the **uncompressed** compiler returns the same `+0.00002` with the same sign
  on the same scene, so the residual is the estimator, not the coreset;
- across all nine cross-robot cells, `vs_full = +0.00000`.

**Corrected statement.** The defensible invariant is *"the coreset never
reports a wider gate than the uncompressed compiler"* (`vs_full <= 0`), not
*"every gate error <= 0"*. The earlier phrasing was wrong and is retracted.

## Claim 2 — coreset certifies MORE clearance than the uncompressed run

> "The coreset's certified clearance LB is larger than the uncompressed one
> (+3.7e-8) — a map that only grows obstacles cannot certify more clearance
> than its source if both bounds use the same estimator on the same path."

**Verdict: does NOT hold.** The premise "same estimator" is false.
`verification/continuous.py:102` defines `clearance_lb` as `min` over
`pair_margin`, and `pair_margin` is a *lower bound* computed on a finite
angular grid, not an exact distance. So

    lb_macro <= clear_macro <= clear_orig,    lb_orig <= clear_orig

Both quantities lower-bound the same clearance with independent slack; set
containment does not order two such bounds. `lb_macro > lb_orig` by 3.7e-8
means the 484-pair bound is looser, not that containment failed. Containment
itself is checked analytically and passes
(`test_full_sandwich_holds_analytically_on_the_real_partition`, suite 53/53).

## Claim 3 — the 17.3x decomposes badly

> "Your surface-only control already gives 484->124 free (3.90x), so robot
> conditioning is worth at most 124/28 = 4.43x — and that cell was never run."

**Verdict: CORRECT, and now measured — it is worse than the audit's ceiling.**
That cell HAS since been run (job 16983029; the number postdates the prompt the
routine was given). Surface-only, at exact accuracy: **124 -> 65 = 1.9x**, well
under the 4.43x ceiling. At a 0.02 rad budget: 124 -> 30 = 4.1x.

## Claim 4 — the speedup excludes coarsening

> "14.6x excludes the 15-27 s coarsening; honest end-to-end is 4.08x-6.00x."

**Verdict: CORRECT.** The quoted 15.4x is compile-only. Single-shot end to end
is `(12.8 + 8.9) / 137.6` => **6.3x**, not 15.4x. The 15.4x figure is only
legitimate as an amortised number under compile-once/query-many, which this
project has still never measured.

## Claim 5 — "coresets ordered by robot size" is a theorem, not a finding

> "Sealing propagates upward in erosion radius, so verifying at the largest
> probe radius certifies every smaller robot and never the converse — exactly
> your adequacy matrix. Stating it converts coarsening from per-(scene,robot)
> to per-scene, which is what legitimately recovers the 14.6x as an amortised
> figure."

**Verdict: constructive and probably right.** If it holds, build the coreset
once for the largest robot in the fleet and reuse it for all smaller ones; the
15-27 s coarsening then amortises over robots as well as queries. This should
be stated and proved rather than reported as an empirical curiosity. Not yet
verified here.

## Claim 6 — novelty verdict

> "The epsilon-kernel outer approximation and the persistence-based topology
> gate are both known constructions; the one genuinely new thing is using a
> robot-conditioned topological invariant of the erosion filtration as a merge
> admissibility gate. Not T-RO as it stands — the compiler is HRM-subsumed and
> the title claims two-sided preservation while only soundness is measured.
> Credible RA-L/ICRA with real 3DGS plus HRM's released C++ as a baseline."

**Verdict: unverified, and its citations are weakened by the egress blocks.**
Worth re-running against reachable sources before acting on it.

## Actions this implies

1. Retract "every gate error <= 0" wherever it appears; replace with
   `vs_full <= 0`. (Done in this file; worklog still to update.)
2. Report 6.3x, not 15.4x, unless compile-once/query-many is actually measured.
3. Measure compile-once/query-many amortisation — still never done, and it is
   the moat the master plan claims.
4. Prove or refute the robot-size ordering; if it holds, coarsen once per scene
   for the largest robot.
5. Real 3DGS before any compression claim is written up: 1.9x on surface-only
   synthetic is the honest current number.
