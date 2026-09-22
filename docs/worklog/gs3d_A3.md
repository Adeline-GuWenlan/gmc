# A3 — robot, goal-region, and replay adapters

Status: acceptance compute is running; this report is a checkpoint and will be
finalized from the produced artifacts before A3 acceptance.

## Implemented checkpoint

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
- `experiments/gs3d_run.py` is A5's timing-hook-ready integration entrypoint.
  It includes all-robot synthetic execution and the unchanged real rung-4
  cylinder start `(7.35,5.30)` to goal `(11.70,12.70)` (8.583851 m xy).
  Real-scene acceleration uses only full 3D ellipsoid-AABB overlap, never a 2D
  projection or centre-only crop.

## Checkpoint evidence

- Local focused A3 test: 9 passed after the footprint-hole repair.
- Combined A1+A3 focused test: all tests displayed passing dots and completed
  without a failure report; the compute rerun records an exact JUnit count.
- Local synthetic runner: `all_success=true`, `uav_variable_z=true`, replayed
  altitude range .6000000000000001 m.
- Compute job 18244075 (2 CPU, 16 GiB, 2 h) runs focused tests, writes synthetic
  evidence, then performs the real long-cylinder exact/relaxed matrix. Runtime
  outputs are under `logs/A3/`.

## Current limitations / next action

The real cylinder artifact is not yet available, so no long-route success is
claimed at this checkpoint. Once job 18244075 finishes, inspect the JSON by
artifact status (not process exit code), repair any algorithmic obstacle without
weakening collision/coverage rules, rerun only what changed, then finalize this
report and A3's acceptance state. Rendering remains A5 ownership; replay poses
and z samples are supplied here for that integration.
