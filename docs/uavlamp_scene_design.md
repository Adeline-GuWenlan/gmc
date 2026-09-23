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

*(filled in after the run, verbatim)*
