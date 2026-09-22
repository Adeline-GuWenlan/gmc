# A1 — direct 3D core, accepted 2026-09-22

A1 implements full-covariance Gaussian/body collision, conservative spatial lookup,
continuous linear-edge validation and bounded sparse planning. The focused suite has
**49 passes, no failures/errors/skips**. All synthetic gates and 1k/10k/100k scaling
checks pass. Three actual 3D renders were opened before evaluating acceptance metrics.
This completes A1, not the real-scene integration, physical certification or whole project.

Base: accepted A0 `1bac9e2b206957d0f8a494df6c192d36a6b52a17`.
Implementation commits: `64e3a7e591a4cbfbbd87061475e68b898c76f8ee`,
`cbff7b329cf4949747a5e3b59d63f7a16329170c`. Final report/evidence are committed separately.
No applicable AGENTS.md, merge conflict, dataset change, package installation, scheduler
change, external message or publication. A0 contracts and all legacy implementation/tests
are unchanged. The runner alone publishes after A7 acceptance.

## Delivered APIs for A3 and A4

All modules below are under `gmc/src/gmc/gs3d/` and retain A0's v1 contracts.

| API | Purpose and caller obligations |
|---|---|
| `oracle.PreparedScene(scene)` | Validate all input covariances, snapshot `opacity > tau` means/covariances/IDs, build 3D BVH once. Coverage and support providers must remain immutable during reuse. Counts/floor adjustments and prep duration are in `.stats`. |
| `oracle.GaussianBodyOracle(scene_or_prepared, budget=None)` | `.pose(q, body, margin_m=...)` and `.edge(a, b, body, margin_m=...)` return `OracleReport`. Linear xyz motion, upright finite cylinders; yaw does not change this body geometry. Ground support is required, but this oracle alone does not authorize lateral-slip motion. |
| `oracle.QueryBudget(SearchBudget, entered_at=...)` | Wall, oracle-call and narrowphase-pair caps; `BudgetExceeded` identifies the binding cap. Planner also limits expansions. |
| `planner.LatticePlanner(prepared=None).plan(scene, body, start, goal, config, timer=None)` | A0 `PlanResult`; sparse 26-neighbour xyz for UAV and 8-neighbour supported xy for ground. Prepared reuse requires the identical `SceneSpec` object. Exact connectors and every graph/exported edge use the oracle. |
| `validation.verify_path(oracle, poses, body, margin_m=..., goal=None)` | Recheck every closed segment or singleton, goal tolerance and ground rotate/forward kinematics. Empty paths fail. |
| `validation.verify_linear_trajectory(trajectory, body, limits)` | Validate finite timing, speed, vertical speed and yaw rate. Explicitly returns `acceleration_verified=False`; A6 must handle knot discontinuities. |

Typical prepared call:

```python
prepared = PreparedScene(scene)
planner = LatticePlanner(prepared)
result = planner.plan(scene, body, start, GoalRegion(original_goal),
                      PlannerConfig(resolution_m=0.10, margin_m=margin), timer=sink)
```

Use UAV margin **.05 m**, ground chassis margin **.001 m**. Ground bodies preserve
support-relative centre height, whole-footprint support, radius and height. A3's support
provider must validate swept contact travel/slope and conformity of linear z to its
manifold; a height sample at the centre is insufficient. Missing support fails closed.
All Gaussians remain chassis obstacles. A3 owns real robot/support/goal adapters.

The minimal exported ground trajectory contains stopped rotations followed by forward
lines; UAV xyz varies. Physical `control_dt_s=.05` is separate from algorithm duration.
Default limits match A0: UAV speed/vertical/yaw .5/.3/1; ground .3/0/1, with support-induced
z distinguished from independent climb. Acceleration constraints remain downstream.

Each core call searches one declared resolution. A3 can orchestrate A0's single bounded
refinement on failure as another explicit call, preserving each result/budget/timing;
there is no hidden refinement, silent goal movement or unbounded retry. Goal relaxation
must be requested explicitly, is Euclidean xyz, and is capped at .50 m. Exact equality
allows only **1e-9 m** roundoff. Original and attained goals/occupancy stay separate;
reachable candidates are considered during search, including when the original is unsafe.

A4 receives protocol hooks for `index_build`, `endpoint_check`, `goal_candidates`,
`search`, `edge_validation`, `trajectory_build`, `verify`. `algorithm_wall_s` begins at
`plan` entry and ends after verification/final result assembly. For externally prepared
calls, prep belongs to the outer wrapper; reset counters on every call. Unprepared calls
flag `preparation_in_algorithm=True`: their index build is inside algorithm wall and is
not counted again in outer preparation. `timings.records` is empty until A4's collector
populates it; cold/warm aggregation/replan distributions belong to A4. Nested hooks must
not be summed as wall time. JSON serialization, rendering and queue time are excluded.

## Geometry and safety scope

The occupied model is the union of full 3D `mu + level*L*u` ellipsoids for opacity > tau.
It is not a projection, Gaussian probability claim or observed-free-space map. A separate
coverage provider must contain the entire swept body AABB. Bounds constrain full body
extent and margin. Unknown holes, failed geometry, invalid input, exhausted budget and
no path on one lattice remain distinct. Failed results contain no executable trajectory.

`spatial.SupportIndex` is a median-split BVH over complete ellipsoid AABBs, including
large supports centred outside the workspace. No nearest-K truncation or centre crop.
Preparation is O(N log N) time/O(N) memory; favourable queries cost O(log N + candidates),
worst-case O(N). Sparse search storage scales with visited nodes/frontier, never N×grid.
Leaves bound temporary candidate arrays, and deadlines are checked inside BVH/geometry.

The swept cylinder is exactly cylinder plus translation segment. Separating support
planes bound that whole convex sweep; GJK only proposes directions. Free requires an
independently evaluated lower bound strictly above margin. Occupied requires an interior
witness (or the declared hard workspace boundary); contact/nonconvergence otherwise
returns unknown. No sampled clearance is labelled a continuous bound.

Bounds subtract **1e-8 m + 128*machine-epsilon*coordinate scale**. Quadratic support
calculations include an absolute cancellation-error allowance; proposed Minkowski
vertices have error allowances for interior witnesses. Tiny nonnegative covariance
eigenvalues are outward-floored to at least 1e-12 m² with a representable diagonal addition
and counts/maximum addition recorded; negative/asymmetric/nonfinite inputs are rejected.
Returned clearance is capped by the broadphase padding (approximately max(.10, margin+.05)
m), and also by workspace clearance. It is a conservative lower bound, not the exact
minimum. These are numerical continuous bounds, not formal exact-arithmetic certificates.
Upright attitude, static declared map, complete coverage and ideal tracking are assumptions.

## Verified evidence

Runtime root: `/scratch/wg2381/codex_jobs/gs3d_20260921`; paths below are relative to it.
Compact committed evidence: `gmc/results/gs3d/core/acceptance.json`, `core.xml`,
`under_over_scene.json`, `under_over_plan.json`, `under_over_side.png`, `visual_review.json`.
The summary includes SHA-256 hashes for the complete runtime evidence/media.

| Check | Observed result | Evidence |
|---|---|---|
| Focused suite | **49 passed**, 64.23 s; JUnit independently parsed, zero skips/errors/failures | `logs/A1/core.log`, `core.xml` |
| Direct 3D / z distinction | Same xy supports, different z require different routes; projection trap and static import/name audit pass | focused suite; `acceptance_review.json` |
| Full bodies / closed edges | Free endpoints crossing an obstacle rejected, including vertical edge; anisotropy, tangency, body dimensions, bounds, holes, missing footprint support pass | geometry/planner tests in JUnit |
| Analytic numeric checks | Exact sphere/cylinder and rotated-support constructions bound solver output; severe covariance cancellation, covariance floor and immutable preparation checked | JUnit, `roundoff_unit.log` |
| Goals / ground / failures | Supported sweeper/cylinder stopped-turn paths; occupied/unknown original with reachable relaxed goal; disconnected nearer goal; singleton; all four budgets; no-path versus unknown; invalid inputs/timing | JUnit |
| Production projection calls | **0**, trap active throughout smoke planning/scaling; no production `project_scene`, `SceneModel2D` or `Pose2` references | `numerical_acceptance.json`, `acceptance_review.json` |
| Visual consistency | Three PNGs opened; final .20 m poses exactly equal inspected path | `visual_review.json`, `numerical_acceptance.json` |

Synthetic under/over fixture: seed 0, level 2, tau .3; workspace
[-2.2,-.5,0]–[2.2,.5,2.8] m; UAV r=.25, half-height=.10.
Light mean (-.7,0,2), semiaxes (.35,2,1); low obstacle mean (.8,0,.3),
semiaxes (.35,2,.95). Start (-1.8,0,.65), exact goal (1.8,0,1.65).
At the two obstacle centre planes, necessary free centre heights are respectively
**below .855013 m** and **above 1.395238 m**. Workspace bounds exclude flying above
the light or beneath the low obstacle, and their lateral span excludes going around
inside the domain. Thus fixed z cannot work. Every returned edge is continuously checked.

| Resolution | Clearance lower bound | z range | Expansions | Oracle / narrowphase calls | Algorithm wall |
|---|---:|---:|---:|---:|---:|
| .20 m | .0808909525 m | .65–1.65 m | 46 | 946 / 74 | .143 s |
| .10 m | .0550260163 m | .65–1.65 m | 463 | 4,313 / 870 | 6.484 s |
| .05 m | .0546035923 m | .65–1.65 m | 4,194 | 20,588 / 5,175 | 52.990 s |

All exceed .05 m clearance and .50 m required altitude swing, within default budgets.
Evidence: `logs/A1/under_over_0.20.json`, `under_over_0.10.json`, `under_over_0.05.json`.
The finest run visits 5,477 sparse nodes; tightening resolution is measurably expensive.

| Supports indexed | Preparation | 20 edge queries | BVH nodes visited across queries | Warm direct plan |
|---:|---:|---:|---:|---:|
| 1,000 | .00652 s | .00426 s | 220 | .00128 s |
| 10,000 | .01955 s | .00489 s | 380 | .00135 s |
| 100,000 | .21201 s | .00594 s | 500 | .00154 s |

Each scaling run has 20 candidates/20 narrowphase pairs in total. The spatial extent
increases at fixed density, with one local nonblocking support. This demonstrates local
index scaling, **not dense showcase or long-distance search performance**. A3 must still
diagnose those costs and budget/connectivity/map-unknown outcomes. No full-scene performance
claim follows from this sparse benchmark. Evidence: `logs/A1/scaling.json`.

Opened perspective, side and high-camera analytic ray renders of the same 3D ellipsoids
and finite red body cylinders. They show the low passage, climb in the gap and high passage;
the side view most clearly distinguishes altitude. Interactive HTML and native PLY also
retain the three-dimensional geometry. These are synthetic supports, not an edited GS
showcase image. Numeric continuous safety comes from the oracle, not apparent image gaps.

## Reproduction, compute and corrections

From this worktree's `gmc/`, exact environment is
`PYTHONPATH=src:experiments MPLBACKEND=Agg OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1`
and Python `/scratch/wg2381/.conda/envs/gmc-venv/bin/python`. Full tests/scenes/rendering
must be submitted through the runtime helper, never run in the 1 CPU agent allocation:

```bash
python3 "$GS3D_ROOT/submit_compute.py" --stage A1 --script "$GS3D_ROOT/compute/A1_visual.sh" --cpus 2 --mem-gb 8 --hours 1
# Open three generated PNGs, then record actual visual_review.json observations.
python3 "$GS3D_ROOT/submit_compute.py" --stage A1 --script "$GS3D_ROOT/compute/A1_checks.sh" --cpus 2 --mem-gb 8 --hours 1
```

The scripts run `experiments/gs3d_core_smoke.py --visual/--checks --output <evidence-dir>`
and `python -m pytest tests/unit/test_gs3d_core_geometry.py tests/unit/test_gs3d_core_planner.py
--junitxml=<evidence-dir>/core.xml`. Check mode requires actual prior visual review.
Job **18234289 completed** in 31 s, MaxRSS 276,112 KiB; **18243546 completed** in 132 s,
MaxRSS 101,372 KiB. Both used 2 CPU/8 GiB. No owned compute remains live.

Initial tiny test collection failed on an unmatched parenthesis; corrected before eight
passing focused checks. Numerical review then identified potential covariance-support
cancellation; hardened and checked with four additional analytic tests before the final
49-pass suite. Also rejected nonfinite motion limits and included final assembly in wall
time. No final failed check remains. The final inspected path is unchanged by hardening.
A0 already reproduced/classified the legacy full suite (576 pass/25 missing sealed Atlas
assets); A1 did not repeat it or exempt any new failure. A7 owns final full-suite review.

A2/A5 still supply and validate the real edited scene; A3 supplies real support, robot and
cylinder-goal integration; A4 supplies timing aggregation; A6 supplies smooth acceleration-
constrained trajectories. Their work does not weaken or replace these A1 synthetic gates.
