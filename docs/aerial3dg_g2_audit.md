# G2 audit: is the cylinder really unreachable on so many pairs? (F3 Task 0, 2026-10-07)

**The question (user):** does G2 check with A* or another algorithm that the cylinder really is unreachable on so many
pairs? If not, is each failure location picking or an algorithm failure?

**Short answer.**
- `gmc/results/aerial3dg/g2/collect.json` contains GMC's own verdict counts and nothing else.
- F1 cross-checked only 30 rows per class, and those checks mostly came back NO_ROUTE_MARGIN, which proves little.
- This audit checks **every** G2 non-REACHABLE row (cylinder 4927, sweeper 301), plus 193 controls, with algorithms
  other than GMC. Results: __HEADLINE__

Machine-readable:
- `gmc/results/aerial3dg/f3/g2_audit/verdicts.csv`: one row per audited pair, with every check and the verdict.
- `summary.json`: class × verdict counts and coverage.
- `raw/` (per-row A* jsonl, lattice labellings), `raw_s2/`, `lampwidth.{json,png}`.

Labels: **[E]** = backed by the committed file named next to it; **[G]** = guess or inference, not tested.

## 1. Method: four independent checks, none of them GMC

| check | what | settings | rows |
|---|---|---|---|
| A. real-body A* in the G2 box | gs3d `LatticePlanner`, F1's oracle settings | real body, margin 0.001, 0.1 m lattice, pos tol 0, yaw tol 0.05, 500k expansions, 300 s | all 5000 cylinder rows (4927 + 73 REACHABLE controls) + 421 sweeper rows (301 + 120 controls) |
| B. real-body A* in F1's W1 box | same, on the W1 scene u[-10,4.7] v[-1.35,3.75] | as A | 1999 rows: CY-GAP 1587 + CY-EP 400 + CY-EP-LAT 12 |
| C. subset-body lattice (decisive version of NO_ROUTE_MARGIN) | connected components of a fixed 0.05 m route-frame lattice: every node a free pose, every edge one oracle edge test. This is exactly the graph the A* searches, anchored globally instead of at the start. Each row's start and goal are attached with one oracle edge. Same component = constructive route (re-verified with `verify_path` on 300 rows per labelling); different components = exhaustive A* would fail | bodies real / s2 / s10, **margin 0** (s2/s10), step 0.05 | all cylinder rows |
| C'. per-row subset-body A* | `LatticePlanner` on the s2 body, margin 0, 0.05 m lattice: a cross-check of C | as A, 0.05 m | systematic samples: CY-LAMP 1-in-9 (326), CY-GAP 1-in-16 (100), CY-EP 1-in-7 (58), all CY-EP-LAT (12), controls 1-in-4 (19) |
| D. lamp passage geometry (no search) | 1 cm occupancy of every Gaussian's 2σ cross-section, on z planes every 1 cm across the cylinder's body band [0.02, 1.75] m, and the widest disk that can cross the lamp line | window u[-2.5,0] v[-0.35,2.75] | – |

**Why subset bodies, and which.**
- In F1's sample, NO_ROUTE_MARGIN was the usual outcome. It means "exhausted, but some edges were rejected only as
  within-margin". The pilot here shows why **[E]**: 95 of the 188 Gaussians behind such rejections lie *under the
  chassis* (2σ top < 0.02 m). The real cylinder's 0.1 m lattice breaks into small start components (289-565 nodes)
  bounded by floor splats that graze the 1 mm margin.
- A body that is a **strict subset** of the real cylinder, searched at **margin 0**, removes both effects. If it
  finds no route, the real body cannot pass either (modulo lattice resolution). If it finds a route, a route exists
  up to the shrink.
- The shrinks:
  - **s2**: radius −2 mm, chassis bottom +2 mm, top −2 mm. Half-height −2 mm and clearance +2 mm keep z_c unchanged.
  - **s10**: radius −10 mm, bottom +3 mm, top −10 mm. Half-height −6.5 mm, clearance +3 mm, z_c −3.5 mm. Still inside
    the real body: bottom 0.023 ≥ 0.020, top 1.740 ≤ 1.750, r 0.29 ≤ 0.30.
- The bottom raise comes from the geometry. The plane-floor edit keeps only splats whose 2σ top is below
  z_floor + 0.02, and F1 measured the grazing tops at 0.0180–0.019990 m. So at +2 / +3 mm every floor splat is ≥ 2 mm
  below the chassis and no longer grazes at margin 0. Anything that reaches above 0.022 / 0.023 m still blocks.
- Lateral 2 mm is the "≤ 2 mm tolerance" of the verdicts. 10 mm is a generous over-approximation: if even that body
  cannot pass, the pair is blocked by a wide margin.

## 2. Results
__RESULTS__

## 3. Per-row verdict rule (`experiments/aerial3dg_fail3_audit.py verdict`)
__RULE__

## 4. Cost
__COST__
