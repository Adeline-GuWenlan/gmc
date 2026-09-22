# A3 — robot, goal-region, and replay adapters

Status: accepted implementation and evidence, 2026-09-22.

## Delivered

- Frozen upright cylindrical bodies: UAV radius/half-height .25/.10 m, sweeper
  .175/.04 m, cylinder .30/.865 m. Ground chassis bottoms remain .02 m above
  their evidenced support manifold; ground collision margin is .001 m and UAV
  margin is .05 m.
- `EvidenceBoundedPlaneSupport` proves the whole swept footprint is inside a
  declared support domain, enforces <=5 degree slope, <=.05 m contact travel,
  and fails closed on unknown holes. The first focused test exposed a
  corner-only hole check; the implementation was corrected to require the full
  closed footprint AABB.
- Goal policy retains the original goal, permits only finite 0–.50 m position
  tolerances, records attained position/yaw error, and preserves each declared
  lattice attempt. It performs at most one finer retry and never automatically
  retries endpoint, map-unknown, or budget-exhaustion failures.
- Endpoint map status, budget stop, lattice connectivity counters, and scaling
  counters are reported separately. The cylinder matrix is exactly
  0/.10/.25/.50 m, with no target rewrite.
- Strict GS3D JSON rejects legacy Pose2 interpretation. Replay consumes stored
  `[x,y,z,yaw]` and physical times, resamples actual z, and independently
  rechecks every closed swept edge, goal, speed, vertical speed, and yaw rate.
  Non-standard JSON NaN/Infinity constants are rejected on input.
- `experiments/gs3d_run.py` is A5's timing-hook-ready integration entrypoint.
  It includes all-robot synthetic execution and the unchanged real rung-4
  cylinder start `(7.35,5.30)` to goal `(11.70,12.70)` (8.583851 m xy).
  Real-scene acceleration uses only full 3D ellipsoid-AABB overlap, never a 2D
  projection or centre-only crop.

## Acceptance evidence

- Compute job 18244075 completed in 3m20s with exit 0, 2 CPUs and peak RSS
  1,714,200 KiB. Artifact inspection, rather than that exit code, establishes:
  **58 passed, 0 failed/errors** in 62.788 s. JUnit and log are runtime
  `logs/A3/focused.xml` and `focused.log`.
- Synthetic all-robot execution and independent replay pass for UAV, sweeper and
  cylinder. UAV planned/exported/sampled/replayed z range is **0.60 m**; both
  ground bodies remain at their support height with stopped unicycle turns.
  Evidence: runtime `logs/A3/synthetic.json`.
- The real edited-floor scene conservatively selects **224,289** opacity-active
  ellipsoids by full 3D support-AABB overlap from 7,101,868 source rows. The
  prepared index retains all of them; no projection or centre crop is called.
- The unchanged cylinder endpoint separation is **8.583851117 m**. Exact mode
  reaches the original `(11.70,12.70)` goal with zero position error via a
  **12.812742 m** route. It expands 686 nodes, makes 1,532 oracle calls and
  84,466 narrowphase checks in 31.170 s. Continuous replay passes with clearance
  lower bound **.001001107 m** against the frozen **.001 m** margin. Exact,
  .10, .25 and .50 m policies all safely reach the exact original goal; no
  relaxation was needed or substituted.
- Legacy trace diagnosis remains separated: projected endpoint prechecks were
  clear and projected raster connectivity was true, but neither establishes 3D
  map coverage. Rungs 3/4 stopped specifically on the legacy query-support cap
  at 73,178/149,303 projected supports. The new sparse 3D run succeeds instead
  of weakening geometry, coverage or the target. Full attempt diagnostics and
  every trajectory are in runtime `logs/A3/cylinder_long.json`.
- Opened runtime `logs/A3/cylinder_long_routes.png`. It visibly shows all four
  paths taking the same long west/north detour and ending on the marked original
  goal; it is explicitly a route-only diagnostic without obstacle context.
  A5 still owns same-scene GS rendering and visual reconciliation.
- Committed `gmc/results/gs3d/robots/acceptance.json` freezes the metrics and
  SHA-256 hashes for the JUnit, synthetic, long-route and route-image artifacts.

Artifact SHA-256: focused JUnit `805d9e9c...3201d0`, synthetic
`4b0e1f5d...df6dc`, long cylinder `f5f25019...b8ff50`, route image
`1719663e...13a953`. Reproduce with runtime `compute/A3_acceptance.sh` through
`submit_compute.py`; never invoke it directly in the agent allocation.

## Corrections and limitations

The first support test caught a corner-only footprint check that missed an
interior unknown hole; the provider now requires the entire closed footprint
AABB to be evidenced. A failed first attempt to generate the route plot had only
a Python quoting syntax error and produced no artifact; the corrected plot was
opened and hashed. Final strict-JSON hardening was covered by a 9-test A3 rerun.

The real map domain remains assumed rather than observation-derived. The floor
is the existing user-approved manual edit; its constant contact reference differs
from the fitted plane by at most .0174994 m at the case-window corners, within
the frozen .05 m evidence bound. The manual floor top clamp leaves .005 m below
the chassis bottom. Piecewise-linear velocity changes at knots are disclosed;
A6 owns acceleration-bounded smoothing. A5 can call `run_synthetic`,
`build_long_cylinder_scene`, `run_long_cylinder`, strict trajectory readers and
replay sampling, and can pass A4's timer into the goal-policy functions.
