# aerial3d-ground: looking for regions where A* proves a path but GMC fails (F3, 2026-10-07)

**Headline.**
__HEADLINE__

The G2 audit (Task 0) is in its own document: [`aerial3dg_g2_audit.md`](aerial3dg_g2_audit.md).

Machine-readable:
- `gmc/results/aerial3dg/f3/pilot_summary.json`: per region × run × robot: funnel, verdicts, classes, costs.
- `failures_pilot.csv`: one row per GMC failure, with class, probe results and A* evidence.
- `sample/<R>/<run>_pairs.json`: the confirmed pairs, each with its A* route and clearances.
- `gmc/<R>/<run>/<robot>/task_00.jsonl`: GMC rows.
- `diag/<R>/<run>/`: probes.
- `cases/`: figure inputs and figures.
- `f4_handoff.json`: F4's input.

Process: `docs/worklog/aerial3dg_fail.md` (F3 section). Labels: **[E]** = backed by the committed file named next to it;
**[G]** = guess or inference, not tested.

## 0. What changed from F2, and the rules used here
- **Real body, no inflation.** A pair is *confirmed reachable* only if all of these hold
  (`experiments/aerial3dg_fail3_sample.py`, docstring):
  - both endpoints are oracle-free for the **real** cylinder at the shared margin 0.001;
  - the straight distance is ≥ 3 m;
  - the gs3d lattice A* (`LatticePlanner`, 0.1 m, position tolerance 0, yaw tolerance 0.05, 500k expansions) returns
    **ROUTE** for the real body at margin 0.001;
  - that route re-verifies edge by edge with `verify_path`.

  F2's +5 cm robust body is gone. A sweeper route follows from the cylinder route, because the sweeper is inside the
  cylinder (same axis, same chassis bottom, r 0.175 < 0.30, top 0.10 < 1.75).
- **Funnel**: distance → endpoints free → shared 0.1 m real-body lattice pre-filter → A* → re-verify. Rejection is
  whole-draw, so any prefix of a stream is an unbiased sample. The pre-filter only rejects. It was audited by running
  the A* on 160 recorded rejects (E 60, NMID 40, S 60): **160/160 were A* non-routes too** (NO_ROUTE 109,
  NO_ROUTE_MARGIN 51) **[E]** (`sample/{E,NMID,S}/pilot/prefilter_audit.json`).
- **Difficulty evidence per accepted pair** (on the exact re-verified poses):
  - `clear3d`: oracle 3-D clearance ladder, 0.5 … 100 mm;
  - `lateral`: radius + top growth ladder for the body with its chassis raised 3 mm, which ignores the floor splats;
  - `vertical`: geometric gap from the chassis bottom down to the floor splats under the route (a lower bound);
  - both endpoints' `clear3d`;
  - route length / straight distance.
- **Failure classes** (`experiments/aerial3dg_fail3_probe.py`, docstring):

  | group | class | rule |
  |---|---|---|
  | export | EXPORT-DOMAIN | shared replay geometry `map_unknown`, no exported pose leaves the prism, but a swept-segment world AABB does (F2 §4.2) |
  | export | EXPORT-KIN | 1 ms turn + translation floors clear the replay (F1/F2) |
  | tolerance | EP-TOL | `*_not_certified_free` with endpoint clearance ≤ margin + buffer (2 mm), or within 0.1 mm above it (fails at 2.1 mm) |
  | tolerance | GAP-TOL | `safe_graph_disconnected` and no real-body A* route at margin 0.0021 |
  | genuine | METHOD-TIMEOUT | the query exceeded G2's 120 s limit |
  | genuine | EP-GENUINE / GAP-GENUINE / REPLAY-OTHER / METHOD-ERROR / SOUNDNESS | the rest; SOUNDNESS = UNREACHABLE on a confirmed pair |

- GMC is run exactly as G2 ran it: one persisted compile per robot per region (`.a3c` + SHA-256 sidecar), then every
  pair through G2's full `QCONFIG` with the 120 s timeout. The compile-once proof holds in every task summary
  (`same_compile_id_all_rows`, no compile stage inside a query).

## 1. Candidate regions
__REGIONS__

## 2. Uniform pilots: funnel and A* rejection split
__FUNNEL__

## 3. GMC on the confirmed pairs (uniform pilots and targeted runs)
__GMC__

## 4. Failure mechanisms found
__MECH__

## 5. Genuine-failure candidates and their verification
__CANDIDATES__

## 6. Choice of F4's benchmark region(s) and the cost projection
__CHOICE__

## 7. Jobs and cost
__JOBS__
