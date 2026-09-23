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
