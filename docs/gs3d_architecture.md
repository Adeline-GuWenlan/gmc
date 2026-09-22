# GS3D architecture decision record

Status: A0 accepted design; interfaces frozen, baseline classified, feasible geometric fixture audited.
Date: 2026-09-21; finalized 2026-09-22. Scope: implementation and simulation in a static edited GS map,
not a physical flight certification. Authoritative requirements: `docs/gs3d_agent_plan.md`.

## 1. Baseline and provenance

Exact algorithm base is `9ff0d5c2a59c95a4444e45878ef878c8d9837108`.
A0 starts at `1d5891a` (only planning/automation documents added; no algorithm delta).
No applicable AGENTS.md was found in the worktree or its ancestors.
Runtime: `/scratch/wg2381/codex_jobs/gs3d_20260921`; stage worktrees are independent.
Python `/scratch/wg2381/.conda/envs/gmc-venv/bin/python`, cwd `gmc`,
`PYTHONPATH=src:experiments MPLBACKEND=Agg`. No installation, GPU or training required.
Baseline full-suite reproduction ran via the bounded compute helper. Its log and
JUnit report, rather than its process exit status, determine the classification.

Audit of the exact base:

- `height/project.py:project_scene` opacity-filters (`opacity > 0.3`), selects a fixed
  height band, forms outer xy ellipses, then returns `SceneModel2D`. Eigenvalue floors
  and support deduplication happen here. This is **legacy_height2d**, not GS3D.
- `height/run.py`, `height/pathio.py` and mobility witnesses use `Pose2`. In old path
  rows the third coordinate is yaw, not z. Never reinterpret an old row as xyz.
- `height/prism.py:robot_table` gives UAV radius 0.25 m, fixed height 1.10–1.30 m;
  sweeper radius 0.175 m, height 0.02–0.10 m; cylinder radius 0.30 m, height 0.02–1.75 m.
  `height/replay3d.py` validates that fixed band; it cannot establish variable-z flight.
- `height/ply3d.py:GaussianScene3D` already retains means `(N,3)`, covariances `(N,3,3)`,
  opacity and unique IDs. Reuse that container, but add covariance validity checking
  at the GS3D preparation boundary (the container does not enforce positive definiteness).
- `height/timing.py` has useful per-stage records but sums durations and has a closed
  list containing projection/compile stages. A4 supplies GS3D timing separately.

Accessible scene inputs (all read-only):

| Input | Location / interpretation |
|---|---|
| Original copied PLY | `/scratch/wg2381/splathjb/splatc_atlas/data/gs_scenes/showcase/raw/point_cloud.ply`, 499,121,449 bytes; source capture `19017822682139126`, art gallery, LOD 0 |
| Decoded original | same directory's `processed.npz`, approximately 782 MiB; `experiments/showcase_scene.py:DATA` already points here |
| Existing edited floor scene | `/scratch/wg2381/splathjb/gmc/outputs/height/plane/processed_planefloor.npz`, approximately 759 MiB; load with `height.planefloor.load_plane_scene` |
| Decode provenance | `results/height/showcase/g0.json`; 7,319,425 retained splats, identity gravity rotation, floor reference z = −1.2271749593107995, fitted normal = (−0.00493939, −0.00058591, 0.99998763), ceiling height 5.310000826 m |
| Edited scene provenance | NPZ `meta` and `results/height/plane/shared_a/video/uav_manifest.json`; 7,101,868 splats, manual `phantom+centre` replacement |

Do not call old builder/runner defaults that write into `/scratch/wg2381/splathjb`.
Read existing assets by absolute path; new edits belong to A2's worktree output directory.
Record hashes and exact edit parameters. The floor edit is not an outer approximation
of the original capture. Guarantees apply only to the declared edited map and support level.
`level=2.0` is a geometric convention, not a 99% confidence claim or occupancy probability.
Opacity is a visibility filter, not observed-free-space evidence.

### Reproduced baseline and input hashes

Compute 18230541 produced 601 tests in 1200.486 s: **576 passed, 25 failed**, no
errors/skips. Independently inspected all failure messages: 24 `AtlasAssetError` for
missing sealed `round4_final_package`, one `FileNotFoundError` for its
`src_snapshot/src/splatc/datasets/g1_gate.py`. Counts: benchmark file 10, gate-interval
file 15. No unexpected failure. This is a classified baseline, not an all-green suite.
Evidence: runtime `logs/A0/baseline.xml`, `baseline.log`, `baseline_classification.json`.
The unchanged legacy source/tests match the exact base; new interface files are not
imported by the baseline tests. Environment: Python 3.13.5, NumPy 2.1.3, cs612.hpc.nyu.edu;
2 CPU/8 GiB requested, MaxRSS 3,843,544 KiB, elapsed 20m13s including input audit.

| Input | SHA-256 |
|---|---|
| Decoded original NPZ | `c9318d253676209b43fd0159786b74df2b628954f524c20ad1cc06d0f58c88ae` |
| Original copied PLY | `98af4928cc67523015bd32c4f438d334adb86db59bc20ef1c97227bf367cc3c2` |
| Existing edited floor NPZ | `a7931eab77e2ca2e46c86653502cd66582293b0eb01cab436792db96d0eb3c60` |

Read-only hash evidence: runtime `logs/A0/assets.json`. Inputs were not copied or modified.

### Cylinder failures: what the artifacts establish

Sources: `results/height/plane/long/cyl_ladder/rung*/cylinder*.json`, their `case.json`,
`results/height/plane/shared/cylinder*.json`, and `docs/worklog/height_planefloor.md`.

| Case | Original separation | Supports | Observed outcome |
|---|---:|---:|---|
| rung 0 | 2.433 m | 21,144 | certified legacy route |
| rung 1 | 3.996 m | 45,572 | certified legacy route |
| rung 2 | 5.454 m | 46,835 | certified legacy route; largest ladder success |
| rung 3 | 6.902 m (rounded) | 73,178 | `query_support_budget_exhausted`, 50 M calls, 722.5 s |
| rung 3 doubled | same | same | same reason, 100 M calls, 1,289 s |
| rung 4 | 8.584 m | 149,303 | same reason, 50 M calls, 1,713.9 s |
| P2 shared | short route in larger map | 156,426 | wall budget exhausted at 7,968 s and 15,027 s on retry |

Rung 3 and 4 prechecks marked endpoints clear and connected on a projected raster.
That is evidence against assuming an unknown endpoint, not proof of continuous connectivity.
The legacy UNKNOWN status describes an unresolved search, not unobserved map space.
No further multi-hour rerun of that unchanged failure is needed for A0. A3 must separately
measure endpoint occupancy, coverage, reachable component at the chosen resolution,
expansions, broadphase candidates, narrowphase pairs, and each budget stop reason.
Use rung 4's unchanged `(7.35,5.30)` to `(11.70,12.70)` pair for the >=8 m attempt;
retain all original goals, including the longer-case goal, in comparisons.

Visual audit: opened `results/height/plane/shared_a/video/uav_3d_f050.png` in an image
viewer. The splat render shows a suspended orange cylindrical UAV above the display
table, beside the bench; legend explicitly fixes 1.10–1.30 m. It supports the legacy
fixed-height interpretation, not a new under-then-over claim. The first A0 fixture audit found unresolved low-approach intervals; the revised
fixture passed geometric feasibility (§6). A2 still builds and inspects the scene before planning.

## 2. Focused primary-source research and decisions

[NeuPAN paper, v3](https://arxiv.org/html/2403.06828v3), §III–IV, formulates shape-aware
point-to-body distance constraints with robot kinematics in receding-horizon control.
Its dual distance features connect an unfolded learned encoder (DUNE) to a motion
optimization block (NRMP). Borrow the separation of geometry from motion constraints
and explicit control dt. Do not substitute point distances or learned features for the
Gaussian-body authority. This paper is not a proof of full 3D Gaussian UAV navigation.

[NeuPAN source](https://github.com/hanruihua/NeuPAN/tree/f5ae6d848b54aeae0340af1eb49e3a809dea00a5),
pinned `f5ae6d848b54aeae0340af1eb49e3a809dea00a5`: read `neupan/blocks/dune.py`,
`nrmp.py`, and `robot/robot.py`. DUNE sorts point-distance features; NRMP constructs
CVXPY-layer constraints. The robot implementation uses three state and two control
variables with differential/Ackermann models, not xyz flight. License: GPL-3.0;
no code copied, no new training/dependencies. Ideas are implemented independently.

[Splat-Nav paper, v2](https://arxiv.org/html/2403.02751v2), §IV and appendices,
uses ellipsoid collision tests, local separating corridors, and Bézier trajectories.
The convex-hull property permits continuous safety when all control points lie in a
valid body-inflated corridor. Borrow full covariance geometry, conservative local
candidate selection, and verified smoothing after a successful route. Its safety is
relative to map completeness; confidence ellipsoids do not establish observation coverage.
Our CPU budget and cylindrical ground bodies require their own verified implementation.

[Splat-Nav source](https://github.com/chengine/splatnav/tree/e996e229daca4e25703a13243541371290a6f7f8),
pinned `e996e229daca4e25703a13243541371290a6f7f8`: read `splatplan/splatplan.py`,
`spline_utils.py`, `polytopes/collision_set.py`, `ellipsoids/intersection_utils.py`.
It uses a radius-configured voxel A* seed, Gaussian halfspace filtering, spline QP,
and CUDA synchronization in the main planner. Not a drop-in CPU solution. License:
MIT; no code copied. Any future source adaptation must preserve its license notice.

## 3. Frozen representation and safety semantics

Public v1 types: `gmc/src/gmc/gs3d/contracts.py`. A1 implements these contracts;
A2 consumes them without editing them. Units metres/radians/seconds, xyz world frame,
yaw about +z, body-centre pose. Serialized row always `[x,y,z,yaw]`.
UAV planning is translation plus yaw, with upright cylindrical collision envelope;
roll/pitch dynamics, wind and tracking error are outside this initial model. If A6
introduces attitude or tracking allowances, enlarge the envelope and revalidate.

Ground chassis stays upright and its bottom follows the support height plus 0.02 m.
Sweeper half-height 0.04 m (centre support+0.06), cylinder half-height 0.865 m
(centre support+0.885). UAV half-height 0.10 m and radius 0.25 m. Preserve these
physical sizes; a radius-only sphere is not an acceptable cylinder replacement.
The existing floor's small tilt must be explicit: A2 exports the fitted plane equation
AND the actual replacement-tile metadata. `plane_tiles` follows local measured heights
and clamps tops below reference floor +.015 m, so the fitted plane alone is not a
measured terrain surface. A3 declares a contact manifold, checks it against floor evidence
(with explicit support-travel and slope bounds), and reports chassis/contact assumptions.
Use the fitted plane initially only where floor evidence agrees within declared contact
travel (maximum .05 m, slope <=5 degrees); otherwise use a supported heightfield or fail
closed. The support provider supplies `height_bounds(lower_xy, upper_xy)` over the closed footprint
box, and `supports_segment` validates the swept footprint and contact-travel/slope limits. Wheels or
support contacts bridge the chassis gap and may touch the designated support surface;
this contact exemption never applies to chassis penetration or other obstacles.
All Gaussians remain chassis collision obstacles. Ground runners explicitly use a
**.001 m chassis clearance margin**, preserving the legacy millimetre-scale route scope;
UAV uses **.05 m**. Ground support contact itself has zero separation and a separate
nonpenetration/support test; it must not be tested against the UAV clearance margin.
Whole swept footprint must have support.
No ground z search, teleportation, lateral slip, or flight. Use unicycle rotate-in-place
then forward translations initially, with finite yaw/translation times. A6 may use
curved primitives only with corresponding kinematic and continuous collision verification.

Occupied scene = union of `E_i = {mu_i + level * L_i u : ||u|| <= 1}` for `opacity > tau`.
Keep complete 3D covariance including off-diagonal entries. Covariance must be finite,
symmetric and positive definite; reject significant negative/asymmetric matrices.
Tiny nonnegative eigenvalues may be outward-floored to 1e-12 m² with counted provenance.
Nonfinite poses/margins/bounds, negative sizes, invalid budgets, duplicate IDs and NaN
are invalid input, never free. Tangency and numerically unresolved gaps fail closed.

Coverage is an independent `KnownSpace` authority. Synthetic fixtures explicitly declare
known workspace boxes. Real showcase runs use a versioned, manually declared **map-domain**
coverage volume; label it `assumed_map_domain`, not sensor-observed free space. A2 records
its bounds and holes. Outside coverage and unknown holes remain unknown even with no
Gaussians. If observation evidence later becomes available, record it independently.
`free` requires body/edge coverage AND a geometric clearance lower bound strictly above
margin; `occupied` requires an overlap witness (or a documented conservative obstacle
model); unresolved numerical geometry returns `unknown`, not a fabricated collision.
`budget_exhausted`, `map_unknown`, and `no_path_on_lattice` are distinct plan statuses.
No-path on one lattice is not a proof of continuous infeasibility.

## 4. Gaussian/body oracle and bounded search

A1 owns one `BodyOracle` for all robots. Reuse ellipsoid support functions
`h_E(n)=n·mu + level*sqrt(nᵀΣn)` and upright-cylinder support
`h_B(n)=n·q + radius*sqrt(nx²+ny²) + half_height*abs(nz)`.
For any unit n the separating interval gap is a valid Euclidean clearance lower bound.
Use a robust support-mapping distance solver (e.g. GJK with independently evaluated
separation witnesses) or a conservative enclosure with verified separation. An optimizer's
approximate distance objective alone is not a lower bound. Returning unknown on
nonconvergence is acceptable; returning free from a heuristic gap is not. Document
floating-point slack (initial 1e-8 m absolute plus scale-dependent roundoff) and subtract
it before acceptance. No unqualified formal/exact-arithmetic certification claim.

Broadphase: exact ellipsoid AABBs `mu ± level*sqrt(diag(Σ))`, indexed in 3D spatial
bins/BVH; oversized splats in a separate always-queried list. Query a swept body AABB
expanded by margin. Never use nearest-K means without a proof excluding large distant
supports. Crop by **support overlap**, not centre containment. Candidate chunks cap memory;
cache by scene hash, body, margin, known domain and config. Broadphase disjointness may
supply a conservative bound; report whether the bound is capped at the query margin.
Pose caches cannot authorize unchecked connecting edges.

Continuous edge rule: linear xyz/yaw interpolation over a closed interval. For upright
circular cylinders yaw leaves geometry invariant. A midpoint clearance lower bound d
implies segment clearance >= d − half the translation length. Adaptive subdivision
accepts only if the bound exceeds margin; otherwise subdivide, find collision, or return
unresolved on depth/work limits. Include known-space and floor/ceiling/support constraints
on the whole swept body. For future noncircular bodies add rotational displacement bound
`R*abs(delta_yaw)/2`; yaw unwrap must match replay. Finite sample checks without the bound
must be `sampled_only` and cannot authorize production success. Analytic separating swept
support functions are an allowed faster alternative. Ground edges follow the support
plane; terrain changes require a new continuous-motion model.

Search: lazy 26-neighbour 3D A* for UAV, xy lattice for ground positions with explicit
rotate/forward unicycle realization. Circular symmetry avoids enumerating redundant yaw
slabs. Search candidates are proposals; every accepted edge goes through the shared 3D
oracle, even for ground robots. Exact endpoints get verified connectors. Goal region uses
an admissible distance-to-region heuristic; verify and cost reachable candidates, rather
than picking a nearest geometrically free goal in a different component. Deterministic
tie-breaking and seed 0. A1 may use 6-neighbour fallback first, but records connectivity.

Initial resolution 0.10 m; synthetic sensitivity 0.20/0.10/0.05 m, one bounded refinement
on failure. Default per-query caps: 120 s, 100k expansions, 1M oracle calls, 5M narrowphase
pairs. Real diagnostic ceiling 600 s, 500k expansions, 5M calls, 20M pairs per run, explicitly
recorded. Stop at whichever cap binds; check wall deadline inside expensive geometry loops.
Use sparse visited/frontier storage; no global SE(2) slab compilation or N×grid arrays.
Record prep cost separately; request 2 CPU/8 GiB for tests and up to 16 GiB for full-scene
preparation if needed. The full 7M-splat scene is never reloaded for every replan.

## 5. Result, goals and timing

`PlanResult` has a schema string `gs3d.v1`, original and attained goals, body/config,
trajectory, occupancy reports, conservative safety status, scene identity, timing and
provenance. Failed plans have no executable trajectory/attained goal; debug partial paths
may live only in diagnostics. `success` requires `continuous_bound`, complete verification,
known-space coverage, goal tolerance, and kinematic validation. JSON rejects NaN/Infinity;
unknown clearance is null. Distinguish a conservative lower bound from measured minimum.

Trajectory stores poses, strictly increasing `time_s` starting at zero, interpolation,
and physical `control_dt_s` (initial 0.05 s). One-pose stationary success may have `[0]`.
For Bézier output `segments` must include complete xyz/yaw control points and duration;
poses are replay samples only. Linear output `segments=[]`, and each pose interval is the
executed motion. Replay consumes exported z/timing; it must not recover z from a robot band.

Goal default exact for the general planner and all non-cylinder comparisons. A3 cylinder
experiment explicitly opts into 0.25 m, compares 0/0.10/0.25/0.50 m, maximum 0.50 m.
Tolerance is Euclidean xyz (ground support must also hold), yaw tolerance separately 0.05 rad;
all finite and nonnegative, yaw <= pi. Never silently rewrite original goal. Report original
occupancy even when relaxed success occurs; collision at the original is not grounds to
skip a requested relaxed search. Numerical equality at exact goal permits only stated
roundoff, not an unreported grid-cell tolerance. Goal yaw may be reached by safe rotation.

`robot` serializes all BodySpec fields plus a `limits` object with keys
`max_speed_mps`, `max_vertical_speed_mps`, `max_yaw_rate_radps`,
`max_acceleration_mps2`, `max_yaw_acceleration_radps2`. Initial simulation bounds:
UAV .5/.3/1.0/.5/1.0 respectively; ground .3/0/1.0/.3/1.0, where zero vertical speed
means no independently commanded climb (support-induced z motion is reported separately).
A3 enforces speed/yaw-rate limits and explicit stopped turns; linear geometric segments
are not permission for lateral-slip turns. Its initial piecewise-linear replay has velocity
discontinuities at knots and must disclose that limitation. Acceleration constraints and
continuous smooth timing become A6 acceptance obligations; finite differences alone do not
prove continuous acceleration bounds. A6 preserves the declared speed/yaw-rate limits.
Diagnostics require `seed`, `expansions`, `oracle_calls`, `narrowphase_pairs`,
`termination`, `resolution_m`, `margin_m`, `coverage_policy`, and `config`; values on
failure are still recorded. Coverage provenance cannot be inferred from `scene_id` alone.

A4 implements `TimingSink.stage` in `gs3d/timing.py`; A1 records hooks through the protocol.
`algorithm_wall_s` starts on entry to `Planner.plan`, before endpoint checks/candidate
selection, and ends after verification/result assembly (before file serialization).
Separate outer `preparation_wall_s` includes load, scene edits if any, index build, covariance
validation; cold end-to-end = preparation + algorithm. Warm records disclose reused
identity/index/cache and reset per-call counters. Render/export/queue times never enter
algorithm timing. Stages: `scene_load`, `scene_edit`, `index_build`, `endpoint_check`,
`goal_candidates`, `search`, `edge_validation`, `trajectory_build`, `verify`, `smooth`,
`export`, `render`. Nested timings are descriptive, never summed as wall time.
Use monotonic `perf_counter`, preserve failed-stage timing. Replans each have call IDs,
wall durations and status; A4 reports count, p50/p95/max, success count, cold/warm split.
At least 5 warm calls per robot in the bounded harness; report all latency samples.
Physical dt, trajectory duration and rendering FPS have different named fields.

## 6. Frozen acceptance fixtures

All below use seed 0, level 2, tau 0.3. Covariance from desired ellipsoid semiaxes a:
`Σ = R diag((a/2)^2) Rᵀ`. Tests must include full bodies and edges, not just point centres.
A1 implements these synthetic fixtures before the real scene; document exact inputs.

| Fixture | Construction and required observation |
|---|---|
| z distinction | Known box [−2,−1,0]–[2,1,3], UAV from (−1.5,0,0.7) to (1.5,0,0.7); ellipsoid semiaxes (0.4,0.95,0.35), mean (0,0,0.7) versus (0,0,2.0). Identical xy supports; lower obstacle forces detour/height change, upper permits straight edge. Monkeypatch projection to raise. |
| between-waypoint | Edge (−1,0,1)→(1,0,1), radius .10, half-height .10; obstacle centre (0,0,1), semiaxes (.05,.3,.3). Both endpoints free; edge rejected. Include purely vertical counterpart. |
| rotated anisotropy | Ellipsoid semiaxes (.8,.08,.08), rotate 45° about y; test inside/on/outside at transformed principal-axis locations and one crossing edge. Check covariance off-diagonals affect results. |
| tangency | Sphere-shaped Gaussian radius .20, cylinder r .25; aligned centres separated .45 at same z are non-free. Separation .501 with margin .05 is free subject to roundoff; .499 is non-free. |
| support/coverage | No Gaussian does not make an unknown hole free. Test swept body crossing a hole, body exceeding floor/ceiling while its centre remains in bounds, and missing ground footprint support. |
| body distinction | Low obstacle permits UAV overflight, blocks tall cylinder; supported sweeper can pass under a tabletop whose underside is >.15 m. Cylinder may detour, cannot change support-relative height. |
| invalid inputs | NaN, Inf, negative dimensions/tolerances, tolerance >.5, invalid covariance, reversed bounds, duplicate IDs, zero-length edge and singleton path semantics. |
| goal region | Original in unknown/occupied cell, safe candidate within .25 reachable; another nearer candidate disconnected. Preserve original, select reachable candidate, reject outside .25 and outside coverage. |

Real-scene fixture uses the **existing** edited showcase, table A and added airborne
Gaussian light with suspension and ceiling attachment. Do not remove existing
obstacles. Start from the inspected P5-A-0 area: reference p=(9.0,5.75), forward
u=(0.6,0.8), lateral v=(−0.8,0.6), table along the old centreline s≈[.92,1.88],
reported top .905098 m above reference floor. That sectional top is not a whole-body
clearance proof; A2 measures all supports intersecting the swept volume.

Audited v2 fixture (A2 records it in the scene manifest before planner evaluation):

- Local coordinates xy=p+s*u+t*v. Candidate start s=−.25,t=0,height=.65; low traverse
  through s=0 to s=.50, then vertical rise at s=.50 to height=1.40, then traverse
  to s=2.50. Declare the task domain in route coordinates as
  [−.75,−1.00,.20]–[3.00,1.00,2.00] m, an assumed map domain containing the whole
  swept body with margin. Existing and added obstacle supports outside the domain
  still participate whenever they overlap it; ceiling/rod render outside it as context.
- Hanging shade ellipsoid centre s=0,t=0,height=1.20; semiaxes along (u,v,z)
  (.18,.65,.15), opacity .95. Its underside is 1.05 m; suspension stays above it.
  Suspension ellipsoid centre (s,t,height)=(0,0,3.25), semiaxes (.025,.025,1.95);
  ceiling attachment centre (0,0,5.20), semiaxes (.45,.85,.10); both opacity .95.
  Record support IDs, covariance and rendering color in the same scene manifest.
- UAV r=.25, half-height=.10. Required altitude range **>=.50 m**, witness .75 m.
  Required obstacle clearance lower bound **>.05 m** over all edges. No reducing this
  margin to obtain the flight. The low central crossing must span at least .20 m
  horizontally inside the light footprint; its body top <= light underside−.05 m.
  Later high crossing spans at least .20 m over table A with body bottom >= measured
  table-support top+.05 m. Use geometric extents, not means or camera appearance.
- Low centre crossing time precedes high table crossing. Record both intervals, min/max
  world and floor-relative z, bounds, body extents and support IDs used in the gates.
  This is a task with ordered waypoint regions; report those constraints explicitly.
  It is not a claim that the unconstrained shortest route must go under the light.
- If this placement fails, A2 may translate the light/start and choose a different rise
  location within the unchanged scene, record the failed audit and new placement, and
  freeze it **before** planner evaluation. The altitude and .05 m margin gates stay fixed.
  Existing table cannot be lowered/removed; any additional table is labelled an edit.

### Feasibility evidence and its scope

Runtime job **18231762**, `compute/A0_fixture_v2.sh` / `A0_fixture_v2.py`, completed
in 10 s with MaxRSS 3,743,388 KiB. Its `logs/A0/fixture_v2/scene_audit.json` records
45,841 original opaque support candidates selected by full ellipsoid AABB overlap,
plus the three proposed light supports. All selected covariance matrices are symmetric
positive definite (smallest eigenvalue 2.0611536e−9 m²). Both audits reproduced all
three source hashes. Original source and edited-floor datasets remain unchanged.

The fixed route-frame crop is [−1.5,−1.5,−.1]–[3.5,1.5,2.5] m. It encloses the
entire swept body with at least .65 m to its boundary. Thus omitted ellipsoid boxes
cannot reduce the reported .07 m bound. Within it, each closed motion subinterval
(length <=.02 m) is enclosed by a swept body box and tested against complete support
boxes. Box-to-box distance is a conservative bound throughout that interval; subtract
1e−8 m numerical slack. This is continuous enclosure validation, not waypoint sampling.
It is an audit calculation, not the production collision oracle or a formal numerical proof.

| Leg | World endpoints (x,y,z) m | Clearance lower bound |
|---|---|---:|
| Under light | (8.85,5.55,−.5771749593) → (9.30,6.15,−.5771749593) | .0804868341 m |
| Rise | (9.30,6.15,−.5771749593) → (9.30,6.15,.1728250407) | .0699999900 m |
| Above table | (9.30,6.15,.1728250407) → (10.50,7.75,.1728250407) | .0699999900 m |

This new UAV task explicitly starts at (8.85,5.55), replacing the **unaccepted fixture
proposal** (8.64,5.27); it does not rewrite any legacy comparison or cylinder goal.
The first attempt remains in `logs/A0/scene_audit.json`: low approach unresolved,
other legs passed. Exact light covariances follow `Rᵀ diag((axes/2)^2) R` for the
world-to-route rotation R above. Audit-only negative sentinel IDs must be replaced
by unique IDs in A2's manifest. The table is retained original geometry, not a new edit.
The top .905098 m is only the historical section observation; A2 must still identify
full tabletop support extents for the ordered-over-table gate.

Opened `logs/A0/fixture_v2/fixture_3d.png`: the shade is below its long suspension and
ceiling attachment; blue path passes below it, rises beyond its forward extent and
continues high; red body rings have different low/high heights. It is explicitly a
3D design schematic without the original table/map, so cannot establish visual agreement
with the GS scene. The earlier opened P5-A-0 splat frame supplies scene context only.
A2's real-scene visualization and A1/A5's production validation remain required.

A2 first exports/render-inspects the edited scene and a candidate feasibility witness;
A5 then plans on that frozen scene. Geometry may be visualized with support meshes plus
splat context, but a scatter/xy raster alone does not establish visual agreement.
Open side and overhead views, low-under-light and high-over-table frames, with actual
body and exported path. Renderer must use the same edit and pose files as the validator.

## 7. Disjoint ownership and downstream acceptance

| Stage | Owned paths and handoff |
|---|---|
| A0 | This ADR, `gs3d/__init__.py`, `gs3d/contracts.py`, `docs/worklog/gs3d_A0.md`; runtime audit scripts/logs. No planner implementation. |
| A1 | `gs3d/geometry.py`, `oracle.py`, `spatial.py`, `planner.py`, `validation.py`; `tests/*/test_gs3d_core*.py`, synthetic core fixtures/runner and report. May extend contracts compatibly with documented rationale, never rename v1 fields silently. |
| A2 | `gs3d/scene.py`, `experiments/gs3d_scene.py`, `configs/gs3d_scene*.json`, `tests/*/test_gs3d_scene*.py`, scene manifests/edits/visuals and report. No A1 core/type edits; use GaussianScene3D + SceneSpec. |
| A3 | `gs3d/robots.py`, `goals.py`, `trajectory.py`, `experiments/gs3d_run.py`, robot/goal tests and report. Implement ground support and timing-ready trajectory/result serialization. Core repairs allowed after A1 dependency, document them; do not edit A4 files. |
| A4 | `gs3d/timing.py`, `experiments/gs3d_benchmark.py`, timing-specific tests/config and report. No A3 runner or robot edits; provide wrapper/hooks for A5 integration. |
| A5 | Integrate branches, wire A4 into A3, render/replay via `gs3d/replay.py` and `experiments/gs3d_render.py`; all-robot real runs. |
| A6 | Smooth only after A5 success. Piecewise Bézier/corridor or constrained shortcut optimization; continuous revalidation, kinematics, velocity/acceleration limits, verified-input fallback. |
| A7 | Independent R1–R8 audit, no live compute, clean committed chain, publication readiness. Runner alone pushes and verifies refs. |

A1 acceptance: all synthetic gates, projection trap, continuous-edge counterexamples,
scaling counters on 1k/10k/100k supports, bounded failure, no unrestricted full enumeration.
A2 acceptance: reproducible input hashes and edit manifest, opened 3D images, feasible
candidate under/over witness with stated validation strength, unchanged originals.
A3 acceptance: successful all-robot synthetic and cylinder >=8 m real attempt plus fix
until a safe route succeeds, exact/relaxed comparisons, supported unicycle trajectories.
A4 acceptance: independent fake-clock boundary tests, failed-call records, all-robot
reproducible cold/warm/replan harness; A5 completes real-run timing integration.
A5 acceptance: real edited GS successes for all three, ordered UAV gates, numeric/visual
agreement, no 2D collision authority. A6 target: >=10% reduction in declared turn metric
on a nontrivial sharp-turn case, no collision/goal degradation; speed/acceleration bounds
numerically verified with physical timing. A7 classifies full-suite failures individually,
checks evidence exists and inspects frames. Missing baseline assets are classified, never
used to exempt an unrelated new failure. All reports state limitations and reproducible commands.
