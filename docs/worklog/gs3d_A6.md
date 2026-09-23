# A6 — constrained smooth-turn optimization

Status: accepted stage evidence, 2026-09-22. A6 consumes the exact hash-pinned
A5 trajectories and derivative Gaussian archive. It preserves a complete,
independently verified raw fallback for every robot.

## Method and safety boundary

`gmc.gs3d.smoothing` first verifies the raw body path, timing, goal and support
constraints. It then performs bounded farthest-visible shortcutting through the
same full-covariance 3-D `GaussianBodyOracle`, while hard UAV mission indices
prevent the low traverse and vertical rise from becoming a diagonal shortcut.
Natural cubic sections are progressively tensioned toward already verified
chords if necessary. The cylinder's full-tension spline left the workspace
margin and was rejected; tension 0.5 passed.

Each accepted cubic Bezier section is continuously enclosed by adaptive convex-
hull tubes around oracle-verified chords. Its hull deviation is added to the
required collision margin. Workspace, declared coverage and complete ground
footprint/support bounds are checked over each control hull. This is a
conservative floating-point argument relative to the map/oracle, not finite
collision sampling or exact-arithmetic certification.

Quintic ease timing makes position, velocity and acceleration continuous, with
zero velocity/acceleration at section joins. Speed, vertical speed,
acceleration, yaw rate and yaw acceleration upper bounds are analytically
recomputed from authoritative controls during independent verification; stored
claims are not trusted. Ground yaw follows the curve tangent, with bounded
rotate-in-place segments where needed. Jerk and wheel torque/traction are not
constrained, and the longer stop-and-go durations are an explicit tradeoff.

## Same-case results

All comparisons use A5 seed 0, unchanged endpoints, bodies, margins and scene
hash `1103b6a5124d4e858bbd03ff751b52bc10a0fd095d7b0212d3c7ac28a893044d`.
The declared turn metric is total direction variation; its finite sampling is a
quality diagnostic and never a safety authority.

| Robot | Turn raw -> smooth | Length raw -> smooth | Clearance lower bound | Duration raw -> smooth | Outcome |
|---|---:|---:|---:|---:|---|
| UAV | 7.0686 -> 3.6191 rad (**48.8% lower**) | 3.5971 -> 3.5311 m | .05277597 m > .05 m | 8.194 -> 16.353 s | Smooth, exact endpoint, all ordered gates |
| Sweeper | 0 -> 0 rad (straight input) | 1.6000 -> 1.6000 m | .00144336 m > .001 m | 8.475 -> 16.824 s | Smooth timing/support; no turn existed |
| Cylinder | 7.8540 -> 2.2003 rad (**72.0% lower**) | 12.8127 -> 12.5223 m | .00100147 m > .001 m | 52.134 -> 95.895 s | Smooth at tension .5, exact original goal |

UAV altitude range remains .75 m. Its protected low crossing precedes the rise
and high table crossing; every gate is true. Ground altitude variation is only
floating-point noise (`<=6.7e-16 m`), with no z search or lifting. Maximum
continuous bounds are .4762 m/s, .2857 m/s vertical and .4536 m/s2 for UAV;
.2858 m/s, .0838 m/s2, .9315 rad/s and .9071 rad/s2 for sweeper; and .2858
m/s, .2712 m/s2, .9524 rad/s and .9071 rad/s2 for cylinder. All are below the
frozen limits. Full precision is in `gmc/results/gs3d/smoothing_acceptance.json`.

## Verification and visual review

Governed job `18292294` completed in 2m09s (MaxRSS 2,157,020 KiB) and supersedes
the pre-hardening run `18291994`. The focused suite reports **69 passed, 0
failed/errors/skips** in 60.056 s; JUnit SHA-256 is
`3de03c9991ed5dccb3ffb309593d5a9648c7b875b9c1673eedaafdc1e72b853e`.
Tests cover curve tubes, between-waypoint collision behavior, support/unicycle
semantics, projection traps, timing, tampered replay and independently
recomputed dynamics.

I opened `raw_vs_smoothed_3d.png` and `trajectory_profiles.png`. The former uses
the exact derivative scene and shows the retained under-light/rise/over-table
UAV sequence plus a visibly rounded cylinder detour on the support plane. The
profile view reconciles the .65/1.40 m UAV levels and gate order; sweeper paths
coincide as expected. Thinned Gaussian means are visual context only; complete
indexed supports authorize numeric safety.

Reproduce the real run from the runtime root using the required governed helper:

```bash
python3 "$GS3D_ROOT/submit_compute.py" --stage A6 \
  --script "$GS3D_ROOT/compute/A6_smoothing.sh" --cpus 2 --mem-gb 12 --hours 1
```

Raw artifacts are under `gmc/results/gs3d/smoothing/`; runtime logs/JUnit are
under `logs/A6/`. The compact committed acceptance record includes hashes for
all raw JSON and rendered evidence.

## Limitations and A7 handoff

Coverage remains an assumed map domain rather than observed free space. The UAV
is an upright translation body without roll/pitch, wind, tracking error,
actuator or closed-loop controller certification. Curve safety is continuous
within the floating-point tube/oracle assumptions; turn/curvature plots remain
finite-sampling diagnostics. Every raw verified A5 route remains available as
the fail-closed fallback if a different scene or configuration rejects its
candidate. A7 should independently reproduce hashes/gates, inspect the images,
and classify the repository-wide suite before publication.
