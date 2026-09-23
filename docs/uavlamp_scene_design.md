# UAV under the lamp — real-scene design (stage L1)

All heights are metres above the plane floor `z_floor = -1.2271749593 m` unless marked "world".
Route frame (Codex A2): origin `(9.0, 5.75, z_floor)`, forward `u = (.6,.8,0)`, lateral
`v = (-.8,.6,0)`, up `z`. UAV: upright cylinder `r = .25`, half-height `.10`, margin `.05`;
`τ = .3`, `level = 2`; lattice `0.10 m`, 26-neighbour xyz A* (see `docs/worklog/uavlamp.md`).

## §1 Pre-registered prediction: Codex's lamp-only scene, one query

*Written and committed before any run.*

**Query Q1a (the one the task specifies).** Codex's derivative
`showcase_airborne_v1.npz` (SHA `1103b6a5…044d`, lamp = 3 Gaussians), start = Codex knot 0
route `(-.25, 0, .65)`, goal = Codex knot 3 route `(2.50, 0, 1.40)`, **one** `LatticePlanner.plan`,
box = Codex's own map domain (route prism `[-.75,-1,.20]–[3,1,2.0]`, which already contains
the union of its legs), resolution `.10`, margin `.05`, Codex's budget.

**Q1b (probe of the launching session's intuition).** Same goal, start moved 1.25 m further
back: route `(-1.50, 0, .65)`; box widened to route `[-2.0,-2.0,.20]–[3.0,2.0,2.0]` (the smallest
box holding the new start with a lateral choice). Same everything else.

The launching session predicts that the route does *not* pass under the lamp but climbs early and
goes beside or over it.

**My prediction, Q1a: the route passes under the lamp — trivially, not because anything forces it.**
Reasons from the numbers, not from a run:

1. The start is already at the lamp. The shade spans route `u ∈ [-.18, .18]`; the start body is at
   `u = -.25`, i.e. its footprint (radius .25 + .05) already overlaps the shade in plan view.
   Its top (`.65 + .10 + .05 = .80`) is below the shade underside (`1.05`). The straight
   start→goal line is at `z ≈ .85` when leaving the lamp zone (`u = .48`), still below `.90`
   (= underside − h − margin). So the lamp is *not an obstacle to the geodesic at all*.
2. "Beside" is closed, but only by **open-air box faces**: the shade half-width is .65, the body
   needs `|v| ≥ .95`, Codex's box stops at `|v| = 1` (effective body-AABB limit `|v| ≲ .65` after
   rotation). No wall there.
3. "Over" needs backing up to `u ≤ -.48`, climbing to `z ≥ 1.50`, then flying at 1.5–1.85 —
   roughly 1 m longer than going under.
4. The table (`u ≥ .706`, top .905) forces the climb to happen in the ~.5 m gap between the lamp's
   and the table's footprints. Codex's own rise at `u = .5` shows that gap is passable, so I
   expect success with a steep rise right after the lamp, i.e. essentially Codex's hand-made
   order — produced here only because the start sits under the lamp's edge.

Confidence: ~70 % "success, passes under the shade footprint at z ≤ .90". Main risk: the real table
geometry near `u ≈ .7` is closer than the AABB suggests and closes the rise gap, in which case the
planner must go over (success, not under) or fail.

**My prediction, Q1b: the launching session is right here — the route climbs early and goes over
the shade (≈55 %), or beside it (≈15 %), not under (≈30 %).** With the start 1.25 m back, the
geodesic to a 1.40 m goal crosses the lamp zone at `z ≈ .85–1.0`, i.e. it now intersects the
shade; staying under requires holding `z ≤ .90` to `u ≈ .45` then a near-vertical rise in the
lamp–table gap, whereas a gradual climb over the shade to 1.5 m costs only a few centimetres more
and uses the cheap diagonal moves. Beside costs a ≥ .95 m lateral detour and the table occupies
`v ≥ -.57`, so it is the longest. This is the situation the user described: the lamp is a local
obstacle and the start→goal geometry, not the scene, decides.

**What would falsify the claim "Codex's scene does not force under-the-lamp":** nothing in Q1a
can, because Q1a passing under is compatible with the lamp being irrelevant. Q1b is the real test.
If Q1b also goes under, the scene is more constraining than I think (then §1 records why).

### §1 results

*(filled in after the runs, verbatim; newest last)*

**Run 1 — job 18323035 (2 CPU, 2.9 GB MaxRSS, 2 m 18 s), Codex's own budget (120 s wall):**

- Q1a: `budget_exhausted / max_wall_s` after **78 expansions**, 654 oracle calls, 44 429 narrowphase
  pairs (≈ 180 ms per oracle call, ≈ 68 body–Gaussian pairs per call near the start). 204 of 654
  edges were rejected as *unproven* (lower bound ≤ margin), 15 occupied. No route. So under
  Codex's own budget a single start→goal query on its scene **does not return a route at all**;
  Codex's A5 legs returned because they were easy: legs 1–2 were accepted as direct edges
  (0 expansions), leg 3 searched 152 expansions at 1.40 m in open air (0.48 s, 304 pairs). The
  expensive region is exactly the low, cluttered space around the start.
- Q1b as pre-registered: `start_invalid / minkowski_interior_witness` — the start route
  `(-1.5, 0, .65)` is 0.1 m from a real hollow pillar (route `u∈[-2,-.4], v∈[-1.6,-.1]`, visible
  in `outputs/uavlamp/t1/q1b/top.png`); the body overlaps real Gaussian 4722613. My
  pre-registration missed the pillar. Re-run with the start moved the minimum amount clear,
  route `(-1.5, +.4, .65)`, everything else unchanged.

Both re-run with a 5400 s wall budget (other limits ×10–40) — the budget is a planner parameter,
not a cost term; see Run 2.

**Run 2 — jobs 18323468 / 18323469, wall budget 5400 s (other limits ×10–40), one planner call each:**

| query | status | expansions | wall | length | under the lamp? | z while under the shade footprint | clearance |
| --- | --- | ---: | ---: | ---: | --- | --- | ---: |
| Q1a (Codex start, Codex box) | success, replay passed | 703 | 1880 s | 3.11 m | **yes** | 0.65–0.75 | 0.0500076 |
| Q1b (start 1.5 m back, v=+.4, wide box) | success, replay passed | 844 | 1939 s | 4.31 m | **yes** | 0.65–0.74 | 0.0500076 |

Figures (looked at): `gmc/results/uavlamp/t1_q1a_side.png`, `t1_q1b_{side,top}.png`.
- **Q1a matches my prediction.** The route stays at 0.65 m under the shade, climbs at 45° right
  after it (u 0–0.6) and rises to 1.15 m over the table.
- **Q1b falsifies my prediction ("over", ~55 %).** The route flies level at 0.65 m from the start,
  under the shade, and starts climbing only at u ≈ 0, past the lamp. My model was wrong because the
  shade underside (1.05 m) is above the start→goal line even from 1.5 m back. And on a 26-neighbour
  lattice, "level then 45° climb" costs the same as any other monotone climb, so the lamp costs
  nothing to pass under. The launching session's prediction ("climbs early, beside or over") is also
  not what happened.
- **Conclusion.** In Codex's scene the under-lamp route appears, but it is **not forced**. It is a
  zero-cost option because the lamp is above the natural climb. The counterfactual (§3 C4h) shows
  that from a start at goal height the planner simply flies over a lamp that nothing seals. What the
  user asked for — the scene *forcing* the under-route — needs sealing (§2) and a necessity proof
  (§3 N2).

## §2 Design: a booth at the end of a real gallery, entered only under a light box

### Site (measured, job 18323467 `experiments/uavlamp_site.py`, refined by the build job)

The hall has full-height display walls (they reach the ~5.07 m ceiling). Between two of them runs a
gallery, **wall A** (world (7.5,22)→(11.6,30.5)) and **wall B** (world (10.3,22)→(15.9,32)),
≈2.3–2.4 m clear. A **real table** (top 0.92–0.94 m, legs at u≈0/1.5/2.95) stands against wall A.
Site frame: `u` along wall A (north), `v` from wall A's face into the gallery, `z` above the floor;
origin world `(9.8752, 26.9843, z_floor)`, `u = (.4314, .9022, 0)`, `v = (.9022, -.4314, 0)`.
I rejected the table at world (14.5,10) against the outer wall: only ~1.5 m approach before a low
round dais, and it has only one wall.

### Why these edits (the design argument)

Codex's lamp is a local obstacle: room beside it and above it, so nothing forces the under-route.
Here the two **real** walls already remove "beside" if the lamp spans the gallery wall to wall.
"Above" is removed by a **bulkhead** (header wall) from the lamp top to a **soffit** (lowered
ceiling) at 2.40 m, which covers the gallery section wall to wall. A **back panel** closes the
booth 0.35 m past the table end. The booth that holds the table (and the goal) is then bounded by
wall A, wall B (real), the floor (real), soffit, back panel, bulkhead + lamp (added). The only
opening is **under the lamp**. This is the F1 pattern with real walls where F1 used box faces.

```
 side section (u–z), any v in the gallery                          plan (u–v)
 z(m)                                                               v(m)
 2.40 ══════════════ soffit (drop ceiling) ════════════════╗        2.45 ▓▓▓▓▓▓▓▓▓▓▓ wall B (real, face 2.30–2.40) ▓▓▓▓
      ║ bulkhead                                           ║             │    lamp │                                 ║ back
      ║ (1.25–2.40)                                        ║ back        │  2.50 m │                     goal ●       ║ panel
 1.50 ║                          goal ● (u 1.50, v .47)    ║ panel  1.20 │ start ● │ (u −1.0…−0.7)                   ║ u=3.30
 1.25 ▄▄ lamp top                                          ║             │         │  ┌──────── table (real) ────┐   ║
 1.10 ▀▀ lamp underside (diffuser)                         ║        0.78 │         │  │ u 0…2.95, top .92–.94    │   ║
  .94      ┌─────────── table top (real) ──────────┐       ║        0.15 │         │  └───────────────────────────┘   ║
  .55 ●start (u −2.40)                             │       ║        0.00 ▓▓▓▓▓▓▓▓▓▓▓ wall A (real, face v = 0) ▓▓▓▓▓▓▓▓
  0.0 ═══════════════ floor (real) ═════════════════════════╝             u: −3.0 (box, open air)  −0.85 (lamp)  0 … 2.95  3.30
      u: −3.0     −2.40      −0.85                0 … 2.95   3.30
```

Dimensions (site frame, metres): lamp box `u ∈ [−1.00, −0.70]`, `v ∈ [−0.05, 2.45]` (ends 5 cm into
both wall bands), underside **1.10**, top 1.25; bulkhead `u = −0.85`, `z ∈ [1.25, 2.40]`; soffit
`z = 2.40`, `u ∈ [−4.20, 3.30]`; back panel `u = 3.30`, `z ∈ [0, 2.40]`. Start `(−2.40, 1.20, 0.55)`,
high start `(−2.40, 1.20, 1.55)`, goal `(1.50, 0.47, 1.50)` = 0.56 m above the table top, body
bottom 0.46 m above it.

### Gap budget (body r .25, h .10, margin .05; lattice .10)

| gap | size | closes if | status |
| --- | --- | --- | --- |
| beside lamp, wall A / wall B | ≤ 0 (lamp embedded 5 cm into each wall band) | < 0.50 | closed |
| above lamp (lamp top ↔ bulkhead) | 0 (bulkhead bottom edge on lamp top) | < 0.20 | closed |
| bulkhead / back panel ↔ soffit, soffit ↔ walls | 0 (overlapping sample rows) | < 0.20 / < 0.50 | closed |
| under lamp (floor top ≈ .03 → underside 1.10) | ≈ 1.07 m tall × ≈ 2.3 m wide | passable if > 0.30 tall, > 0.60 wide | open, ≈ 0.77 m vertical and ≈ 1.7 m lateral slack |

Centre heights under the lamp must be < 1.10 − .15 = 0.95 (lattice from the 0.55 start: ≤ 0.85),
i.e. ≥ 0.65 m below the 1.50 goal (requirement ≥ 0.50). Floor side: centres ≥ ≈ 0.19. Slack over
body+margin to the floor ≈ 0.66–0.76 m ≫ 2 cells.

### Edits are dense sheets, proven leak-free (tests, not eyes)

Each panel is a grid of flat Gaussians, spacing ≤ 8 cm (lamp 5 cm), in-plane semiaxis = spacing,
normal semiaxis 2 cm at level 2, opacity .95 (> τ). `panel_min_half_thickness` proves that the union
of supports is a slab with no holes (worst point: a grid-cell centre, half-thickness
≥ 0.707 × 2 cm). Tests in `gmc/tests/unit/test_uavlamp_scene.py`:
`test_panel_union_is_a_solid_slab_without_holes` (dense Mahalanobis check at and between samples),
`test_body_cannot_cross_panel_between_or_at_samples` (production oracle, incl. cell centres),
`test_gap_rule_closes_below_body_minus_one_cell_and_opens_above` (the .50/.20 rule vs the oracle),
`test_builder_is_deterministic_and_records_identity`, `test_loader_rejects_hash_mismatch`,
`test_opening_slice_single_query_goes_under_and_plug_exhausts` (the whole design pattern on a
synthetic slice: one query goes under; with a plug the reachable set is exhausted).

### Box

Route prism `u ∈ [−3.00, 3.30]`, `v ∈ [−0.05, 2.45]`, `z ∈ [0, 2.43]`.
- `v` faces sit **inside** the real walls (faces at v ≈ 0 and ≈ 2.30–2.40).
- `z_min` is the real floor; `z_max` is the soffit.
- `u_max` is the back panel.
- **`u_min = −3.00` is the only face in open air** (the gallery continues south). §3 B6 pushes it
  to −4.00 and compares routes.

Because the `v` faces coincide with the walls, a hole in a real wall would be hidden by the box. The
boundary leak sweep (§3 P3-sweep) checks this: it crosses every booth boundary with the body inside
a large declared box.

## §3 Evidence (booth v1: archive `outputs/uavlamp/scene/uavlamp_scene.npz`, SHA-256 `485df24d1077c313c355767ecdb0330ca7bcbba8bc986a7f47e4cc2dd7dd08c4`)

Build: job 18323926 (12 s). 7 101 868 source rows unchanged + 5 864 added rows (9 panels). The
manifest records the source SHA (`a7931eab…3c60`, checked unchanged after the build), each edit's
parameters, id range, proven minimum slab half-thickness, solid extent, and **measured contacts**
with the real walls. The soffit, lamp, bulkhead and back panel all reach 8–16 cm past the measured
wall faces (gap ≤ 0). The real table measures `u −0.02…2.97`, `v −0.03…0.79`, top q99 0.922 m /
max 0.938 m. The floor top in the approach is ≤ 0.15 m. There are 0 source supports in the approach
air and 0 in the booth air outside the table; the nearest real support to the goal is 0.53 m away.

### P3-sweep — every booth boundary is closed for this body (job 18323928, 5 min)

`experiments/uavlamp_probes.py` crosses each boundary perpendicularly with straight swept-body
edges on a 0.10 m grid. It uses the production oracle at margin .05 inside a large declared box,
so box faces cannot hide a hole. Figure: `gmc/results/uavlamp/probes_v1.png` (looked at).

| boundary | crossings | free | unknown | occupied |
| --- | ---: | ---: | ---: | ---: |
| wall A (real), u −0.85…3.0, z .25…2.15 | 780 | **0** | 0 | 780 |
| wall B (real), same grid | 780 | **0** | 0 | 780 |
| soffit | 663 | **0** | 0 | 663 |
| back panel above table / beside table | 187 / 81 | **0 / 0** | 0 | 268 |
| entrance above the lamp underside (lamp + bulkhead) | 204 | **0** | 0 | 204 |
| entrance **under** the lamp (positive control) | 119 | **85** | 28 | 6 |

Wall B is sparse in the booth span: only 350–600 supports per 0.25 m bin, versus thousands
elsewhere. It is still closed for the body at every probed point. The non-free cells of the positive
control lie at the floor (z .2–.3) and next to the table's near end (v < .9). The opening
`v ∈ [.9, 2.0], z ∈ [.3, .9]` is free throughout.

### EWA render of the edited region (job 18323927)

Rendered from the planning archive, with real DC colours for source splats and manifest colours
for the edits. Files: `gmc/results/uavlamp/booth_v1_{entrance,cutaway_side,cutaway_oblique}.png`
(all three looked at).
- The entrance view shows the real gallery (framed artworks on both walls), the black light box
  with its warm diffuser spanning wall to wall at 1.10 m, the bulkhead and soffit above it, and the
  real table seen through the opening.
- The cutaways (wall B removed, and in the oblique also the soffit) show the booth: table, back
  panel, bulkhead, lamp.
- The 8 cm sheet splats show a faint grid texture. That is cosmetic, not a hole (the P3 sweep
  above settles that).

### Planner queries on booth v1 (archive `485df24d…`): mostly budget stops

v1 query geometry: start `(−2.40, 1.20, .55)`, high start `(−2.40, 1.20, 1.55)`, goal
`(1.50, .47, 1.50)` (3.97 m from the start), box `u [−3, 3.3], v [−.05, 2.45], z [0, 2.43]`.
Budget 7200 s (N2: 20 700 s). One planner call per row.

| id | query | status / reason | expansions | oracle calls | wall |
| --- | --- | --- | ---: | ---: | ---: |
| M1 | main, low start | budget_exhausted / max_wall_s | 2020 | 7857 | 7200 s |
| M5 | high start | budget_exhausted / max_wall_s | 2809 | 11873 | 7200 s |
| C4 | lamp only, low start | budget_exhausted / max_wall_s | 2077 | 8539 | 7200 s |
| C4h | lamp only, high start | **success**, replay passed; **over** the lamp at z 1.55, len 4.26 m | 1770 | 5916 | 227 s |
| B6 / B6h | u_min pushed to −4.0 | budget_exhausted / max_wall_s | 2001 / 2768 | — | 7200 s |
| N2 | plug under the lamp | **queue exhausted**, labelled `map_unknown / reachable_frontier_meets_unknown_coverage` | 5205 | 24 493 | 3225 s |
| P3a | goal inside the bulkhead above the lamp | goal_invalid (occupied) | 0 | 2 | 0 |
| P3b | goal at the lamp-top/bulkhead seam | goal_invalid (occupied) | 0 | 2 | 0 |
| P3c/P3d | goals at v .15 / 2.20 beside the lamp | map_unknown: goal body outside the box (probe mis-placed) | 0 | 2 | 0 |

Diagnosis (from diagnostics, no new runs):
- **Cost, not a bug.** M1 needed 769 k narrowphase pairs and 14.7 M GJK iterations (19 per pair,
  i.e. near-grazing pairs). A third of its edges were *unproven* next to wall A and the table: the
  goal at v .47 is only 0.17 m of slack from wall A. A* with a Euclidean heuristic on a 26-lattice
  must close roughly the nodes inside an ellipsoid around start–goal (~3–4 k here). At ~3.6 s per
  expansion that exceeds 2 h. The run got no closer than 0.2 m to the goal.
- **N2's `map_unknown` is a coverage artefact of v1's box.** The v faces sat 5 cm *inside* the
  walls. The body's world AABB, rotated into the site frame, grows to ±0.333 m in v, and
  `contains_aabb` is checked *before* occupancy. So edges sliding toward a wall were rejected as
  unknown instead of occupied. P3c/P3d fail the same way. N2's queue did empty (not a budget stop),
  but the label cannot be claimed as clean exhaustion.

Fixes for v2 (scene edits unchanged; the planner and its cost/heuristic untouched):
1. The box's v and u_max faces move *behind* the surfaces: v ∈ [−.35, 2.75] (behind walls A/B),
   u_max 3.70 (behind the back panel). These regions are reachable only through scene surfaces the
   sweep shows closed.
2. A logging wrapper around the coverage object attributes every `map_unknown` to the box face(s)
   exceeded. It is bookkeeping only; the planner gets identical answers.
3. A nearer query: start `(−2.0, 1.2, .55)` / high `(−2.0, 1.2, 1.55)`, goal `(.80, .55, 1.50)`
   above the table's near half. Budget 20 000 s.
4. P3c/P3d move to the extreme body-feasible columns beside the lamp, v .35 and v 1.95 at lamp
   height.

### Booth v2 (archive `outputs/uavlamp/scene_v2/uavlamp_scene.npz`, SHA-256 `2a3a72d610519095381a83fd741a61f32b420b20f39e0c5f60951e463fd596cc`)

Built by job 18329380. The edits are byte-identical to v1; only the config/query block and the
archive meta hash changed. Renders: job 18329381, `gmc/results/uavlamp/booth_v2_*.png`, looked at,
same scene as v1.

**P3-sweep v2 (job 18329382):** wall A 860/860 occupied, wall B 860/860, soffit 731/731, back
panel 187 + 81, lamp+bulkhead 204. That is **0 free crossings of 2923**. The under-lamp control
has 85 free crossings. (`gmc/results/uavlamp/probes_v2.{png,json}`)

**P3 goal probes v2 (job 18329390):** the would-be bypasses are all occupied (`goal_invalid`,
clearance 0):

| probe | goal (site frame) | result |
| --- | --- | --- |
| P3a | inside the bulkhead above the lamp `(−.85, 1.2, 1.80)` | goal_invalid / occupied |
| P3b | lamp top / bulkhead seam `(−.85, 1.2, 1.27)` | goal_invalid / occupied |
| P3c | beside the lamp, first body-feasible column off wall A `(−.85, .35, 1.17)` | goal_invalid / occupied |
| P3d | beside the lamp, last column before wall B `(−.85, 1.95, 1.17)` | goal_invalid / occupied |

**C4h v2 (lamp only, high start; job 18329387):** success in 2 s, 0 expansions. The *direct*
start→goal edge is free: a straight line at z 1.53–1.55 **over** the lamp, length 2.87 m,
clearance 0.10. Without the bulkhead and soffit the lamp forces nothing.

### Booth v2 planner queries — the main evidence

Every row is **one** `LatticePlanner.plan` call on archive `2a3a72d6…`:
- inputs are only the start, the goal and the box; resolution 0.10, margin 0.05;
- budget 20 000 s / 2 M expansions (N2 20 700 s);
- no waypoints, no altitude terms, planner code unchanged from `cba1433`;
- successful routes were re-validated by `replay_plan` with a fresh oracle on a freshly prepared
  index.

Goal `(0.80, 0.55, 1.50)` is above the table's near half (body bottom 0.46 m above the 0.938 top).
Low start `(−2.0, 1.2, 0.55)`, high start `(−2.0, 1.2, 1.55)`. Box `u [−3.0, 3.7], v [−.35, 2.75],
z [0, 2.43]`. Lamp footprint `u [−1.0, −0.7]`, underside 1.10.

| id | query | job | status | exp. | planner wall | length | under lamp | centre z under footprint | goal z − min z under | clearance | replay |
| --- | --- | --- | --- | ---: | ---: | ---: | --- | --- | ---: | ---: | --- |
| **M1** | designed scene, low start | 18329383 | **success** | 1293 | 5902 s | 3.38 m | **yes** | 0.56–0.71 | **0.94** | 0.05004 | passed |
| **M5** | designed scene, high start | 18329384 | **success** | 1941 | 6595 s | 3.55 m | **yes** (descends 1.55 → 0.85, climbs back) | 0.85 | **0.65** | 0.05003 | passed |
| **N2** | + test-only plug under the lamp | 18329385 | **queue exhausted** (`map_unknown / reachable_frontier_meets_unknown_coverage`) | 5330 | 6743 s | — | — | — | — | — | — |
| P3a–d | goals at the bypasses | 18329390 | goal_invalid (occupied) ×4 | 0 | <1 s | — | — | — | — | — | — |
| C4 | lamp only, low start | 18329386 | success | 1316 | 5819 s | 3.38 m | yes (same route as M1) | 0.56–0.71 | 0.94 | 0.05004 | passed |
| C4h | lamp only, high start | 18329387 | success (direct edge) | 0 | 2 s | 2.87 m | **no — over** at 1.53 | — | — | 0.10 | passed |
| B6 | M1 with u_min pushed to −4.0 | 18329388 | success | 1293 | 5763 s | 3.38 m | yes | 0.56–0.71 | 0.94 | 0.05004 | passed |
| B6h | M5 with u_min pushed to −4.0 | 18329389 | success | 1941 | 7504 s | 3.55 m | yes | 0.85 | 0.65 | 0.05003 | passed |

Figures (all looked at): `gmc/results/uavlamp/v2_{M1,M5,N2,C4,C4h,B6,B6h}_{side,top}.png`.
- M1 side: level at 0.55 to u ≈ −1.0, a gentle climb *under* the shade (0.65 → 0.71), then 45°
  up after the lamp and a staircase over the table to 1.50.
- M5 side: a V — down from 1.55 to 0.85 before the lamp, level under it, up again over the table.
  This is the clearest demonstration that z is a search variable.
- N2 side: the red plug fills the opening; there is no route.

**How to read the results:**
1. **The scene forces the under-route (N2).** With the opening plugged, A* emptied its queue after
   5 330 expansions in 6 743 s (13 957 s short of its 20 700 s wall budget): *every* lattice node reachable from the start
   inside the box was expanded and none reaches the goal.
   - The label is `map_unknown` because the frontier touched the declared coverage. The logging
     wrapper shows that **all 3 648 unknown rejections are at `u_min`**, the open-air face behind
     the start, 2 m from the lamp. Zero are at the walls, floor, soffit or back panel.
   - A path leaving through `u_min` would have to re-enter the booth through a boundary that the
     P3 sweep shows closed (0 / 2923 free crossings). So the only connection from the start region
     to the goal region, for this body, is under the lamp.
   - This is a proof by exhaustion on the planner's lattice plus a continuous-edge sweep of the
     boundaries on a 0.10 m grid. It is not a continuous-space proof between sweep grid lines.
     §2's panel slabs are proven hole-free analytically, but the real walls are only swept.
2. **Bypasses are closed (P3).** Goals inside the bulkhead, at the lamp/bulkhead seam, and in both
   side columns beside the lamp are all occupied.
3. **The open-air box face does not matter (B6/B6h).** Pushing `u_min` out by 1 m leaves both
   routes identical: same length and same under-lamp heights.
4. **Counterfactual (C4/C4h), tying back to §1.** With only the lamp:
   - from the **high** start the planner flies straight **over** it (0 expansions, a direct edge);
   - from the **low** start it still goes under, on the same route as M1. As in §1, when the start
     is low the under-route is also the geodesic.

   So M1 alone does not prove forcing; N2 does. M5 vs C4h shows it directly: the same high start
   descends under the lamp in the designed scene and flies over it without the seal.
5. **The planner is slow in real clutter.** It takes ~1.5–4.5 s per expansion near the real table and
   walls (GJK on near-grazing pairs), so each success costs ~1.6–2.1 h of CPU. v1's farther goal
   (3.97 m, 0.17 m from wall A) did not finish in 2 h.

## §4 Limitations

- **The edits are a staged booth, not the captured hall.**
  - The 1.10 m lamp underside would be at head height for people. It is chosen because the claim
    requires the under-lamp centre to be ≥ 0.5 m below the goal while staying above the table.
  - The 2.40 m soffit, the bulkhead and the back panel are plausible exhibition construction but
    are invented.
  - The panels render as 8 cm splats with a faint grid texture.
  - Additions only: all 7 101 868 source rows are byte-identical (checked by the manifest and the
    source hash). The floor is the earlier user-approved plane-floor edit.
- **Necessity is lattice-relative.** "No path" means no path on the 0.10 m lattice anchored at the
  start, with the oracle's fail-closed "unproven" rejections counted as blocked (N2: 7 853
  unproven, 3 578 occupied). A finer lattice or a less conservative bound could in principle find a
  sliver. The P3 sweep, which crosses every boundary with the continuous swept body at 0.10 m grid
  spacing and finds no free crossing, is the stronger evidence that no sliver exists.
- **Coverage is a declared prism, not observed free space.** The box's `u_min` face is open air;
  B6/B6h show it does not change the routes. The other faces are behind scene surfaces.
- **Margins are tight.** Both successes run at clearance lower bound ≈ 0.0500 against the 0.05
  margin: the lattice route hugs obstacles. That is sound for the certificate but not a flight-grade
  buffer. Smoothing (A6) and dynamics are not applied here.
- **Cost.** One real-scene query takes 1.6–2.1 h single-threaded. L2 must budget for that. The
  timings are planner wall times from one CPU; queueing is excluded.
- **Evidence status.** Q1b's pre-registered start was inside a pillar, and the start was moved 0.4 m
  sideways. v1's budget stops and mis-placed probes are kept above and not relabelled.

**EWA render with the routes (job 18331738):** `gmc/results/uavlamp/booth_v2_routes_{cutaway_side,entrance,cutaway_oblique}.png`,
all looked at. In the cutaway side view (wall B removed) the cyan M1 route runs low beneath the light
box and climbs over the real table. The magenta M5 route dives from its high start under the box
and climbs again. The route lines are a 2-D overlay and are **not depth-tested**, so in the entrance
view they are drawn on top of the lamp. L2's photoreal renders should composite them with depth.

Handoff: `gmc/results/uavlamp/l1_handoff.json`.
