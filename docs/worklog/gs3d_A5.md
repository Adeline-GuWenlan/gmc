# A5 — full 3-D integration baseline

Status: accepted stage evidence, 2026-09-22. This is the functioning raw
piecewise-linear baseline handed to A6; it is deliberately not called smooth.

## Integrated path

A5 combines the accepted A2 derivative scene, A3 robot/goal/replay adapters and
A4 timing hooks. The compute job first generated and hash-checked
`showcase_airborne_v1.npz`, then all three robots consumed that same derivative
through `PreparedScene`, `GaussianBodyOracle` and full 3-D ellipsoid AABB crops.
No legacy height projection is imported or called. The original edited-floor
source remains unchanged; the derivative hash is
`1103b6a5124d4e858bbd03ff751b52bc10a0fd095d7b0212d3c7ac28a893044d`.

The UAV task is an ordered three-leg mission. Its exported/replayed centre rises
from .65 m to 1.40 m above the reference floor (range .75 m). The low crossing
spans route s=[-.20,.20] at t=.100–.900 s below manual light ID 7341022. After
the rise, the high lattice path crosses table s=[.92,1.88] at t=4.849–6.826 s.
That high crossing is eight consecutive safe linear segments, analytically
clipped and unioned inside the measured table envelope; rendering interpolation
does not hide or redefine those corners. Independent continuous replay passes
with a .0527759657 m lower bound against the strict .05 m margin.

The sweeper remains on the evidenced support manifold for a 1.60 m route, with
zero z range and .0014433553 m clearance lower bound against .001 m. The cylinder
retains its unchanged 8.583851117 m original goal, takes a 12.812741700 m detour,
and reaches the exact original goal with zero position error. Its continuous
lower bound is .0010011072 m against .001 m. Exact/.10/.25/.50 m policies all
reach that exact goal; relaxation was not substituted. Legacy rung-3/rung-4
`query_support_budget_exhausted` traces remain in the raw evidence with their
hashes and are not relabelled as endpoint or coverage failures.

## Timing and reproducibility

Planner `algorithm_wall_s` begins at `LatticePlanner.plan`; preparation,
rendering, scheduler queue and physical `control_dt_s=.05` remain separate.
Preparation was 4.591 s for UAV and 1.613 s for the shared ground crop/index.
Five warm missions/calls per robot were retained. UAV warm mission p50/p95 was
.487/.490 s; sweeper per-call p50/p95 .00693/.00732 s; cylinder per-call
p50/p95 25.407/25.989 s. Nested stage spans are descriptive and never summed.
Raw timing samples and every attempt are in `gmc/results/gs3d/integration/*.json`.

Reproduce from `gmc/` using the required interpreter/environment. Full-scene
commands must run through the compute helper:

```bash
python3 "$GS3D_ROOT/submit_compute.py" --stage A5 \
  --script "$GS3D_ROOT/compute/A5_integration.sh" --cpus 2 --mem-gb 12 --hours 1
```

The accepted recheck reused the already generated exact archive and used
`compute/A5_integration_recheck.sh`. Job 18290411 regenerated all robot JSON,
five warm-call evidence, renders and 67 focused tests in 5m57s. Its batch exit
was 1 only because the trailing shell audit incorrectly applied `all()` to the
descriptive and correctly false `projection_called` field. Artifact inspection
shows the integration manifest itself is accepted, all intended gates pass,
and JUnit reports 67 passed / 0 failures / 0 errors. The source manifest now
names the positive predicate `projection_not_called` to avoid that ambiguity.

## Visual and numeric reconciliation

I opened `uav_side.png`, `uav_oblique.png`, `uav_high.png`, and
`ground_overview.png`. The side image is the non-occluding acceptance view: it
shows the orange low body beneath the gold light, the cyan vertical rise, and
the magenta high body over the retained table. Oblique/high views retain the
nearby source wall; no obstacle was removed for visibility. The ground view
shows the short blue sweeper route and red cylinder detour to the exact goal.
The interactive artifact is `trajectories_3d.html`.

The renderer loaded the same scene hash and the exact replayed JSON rows: 18 UAV,
4 sweeper and 72 cylinder poses. Recorded and recomputed hashes match for every
PNG, HTML and raw JSON. Compact committed metrics/hashes are in
`gmc/results/gs3d/integration_acceptance.json`; raw artifacts and JUnit are
retained at `gmc/results/gs3d/integration/` and runtime `logs/A5/`.

## Corrections and limitations

Job 18289564 correctly rejected an initial evidence extractor that demanded one
straight .20 m high crossing even though the verified lattice route crossed in
multiple segments. The repair unions analytically clipped consecutive segments
while enforcing body-width containment in the measured table envelope; a new
staircase regression test covers it. The first job's failed gate and the second
job's failed trailing audit are retained rather than reported as successful
process exits.

Coverage remains an assumed map domain, not sensor-observed free space. The
floor is the prior user-approved manual edit. The upright UAV body omits
roll/pitch, wind and tracking error. Raw trajectories satisfy collision,
clearance, support, speed and yaw-rate checks but have velocity discontinuities
at knots; A6 must smooth, measure turn/curvature reduction, and revalidate the
complete body trajectory with a safe fallback.
