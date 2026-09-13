# Height-band obstacle maps: robot morphology on the showcase 3DGS — design

Date: 2026-09-12. Written on branch `k2-real-scene-test-a` at `27f6580`.
Status: design approved section by section in brainstorming; awaiting review of this written spec.

## 1. Goal

Show, with the non-learning GMC pipeline, how robot body shape changes what a real
3D Gaussian Splatting scene allows:

- a flat robot (sweeper) can pass **under** an overhang such as a table or counter;
- a tall robot (cylinder, standing in for a humanoid) has to go **around** it;
- a UAV at cruise altitude can pass **over** it.

Two phases: synthetic toy scenes first (an ellipse robot, then a 3D table), then the
showcase gallery GS with three robots (cylinder, sweeper, UAV), with videos showing
whether and how each robot got through.

"跑通" here means every run completes end to end and returns a verdict backed by its
evidence. It does not mean every robot reaches its goal. Scene semantics are never
bent to make a robot reach.

## 2. Why the current stack cannot answer this

GMC is strictly 2D (`SceneModel2D`, `RobotModel2D`, `Pose2`; "slab" in
`orientation/slab_builder.py` is an orientation interval, not a height layer). K2 Test A
flattened its 3DGS into one 2D map by taking the most solid cross-section over
0.05–0.8 m, which fuses a tabletop into the floor plan: every robot would detour.
Height has to enter before the 2D compiler.

## 3. Approach: one 2D obstacle map per robot height band

Each robot is a vertical prism: a 2D footprint over a height band `[z_lo, z_hi]`
measured from the fitted floor. For each robot, every 3D splat is projected to the 2D
obstacle it presents to that band, giving a `SceneModel2D`. The unchanged GMC compiles
and queries that map with the robot's footprint as its `RobotModel2D`.

Rejected: max-over-K conditional slices per band (Test A's pilot method; misses mass
between sample heights, i.e. false-free) and an occupancy raster with A* (not our
method, no certificates).

### 3.1 Splat decoding and scene definition

- PLY fields `x y z`; `scale_0..2` are log standard deviations; `rot_0..3` is a
  (w, x, y, z) quaternion; `opacity` is a logit. Covariance
  `Σ = R diag(exp(2·scale)) Rᵀ`. Parsed with numpy from the binary layout; no new
  dependency.
- Solid support of a splat: `{p : (p−μ)ᵀ Σ⁻¹ (p−μ) ≤ ρ²}`, `ρ = 2.0`
  (= `support_level_scene`).
- A splat is an obstacle iff `sigmoid(opacity) > τ`, `τ = 0.3`. The showcase reports one
  sensitivity arm at `τ = 0.1`; a lower τ only adds obstacle mass.
- Every filter (opacity, crop, floater removal, window) removes obstacle mass and so can
  only move a verdict toward REACHABLE. Each filter's count is recorded; none may be
  changed to rescue a run.

### 3.2 Band-clipped shadow and its ellipse bound

Blocks: `A = Σ_xy`, `c = Σ_{xy,z}`, `s = Σ_zz`, Schur complement `S = A − c cᵀ / s`.
With `t = (h − μ_z)/√s`, the cross-section at height `h` is
`{x : (x − m(t))ᵀ S⁻¹ (x − m(t)) ≤ ρ² − t²}` with centre `m(t) = μ_xy + c t / √s`.

- **Exclusion (exact).** The z-extent is `μ_z ± ρ√s`. If it is disjoint from the absolute
  band `[z_f + z_lo, z_f + z_hi]`, the splat is dropped for that robot. Touching counts
  as overlap.
- **Clipped range.** `[t_a, t_b] = ((band − μ_z)/√s) ∩ [−ρ, ρ]`.
- **Shadow support function (closed form).** For direction `u`, with `β = uᵀc/√s` and
  `γ = √(uᵀSu)`: `h_Π(u) = uᵀμ_xy + f(clip(t*, t_a, t_b))`, where
  `f(t) = βt + γ√(ρ² − t²)` and `t* = ρβ/√(β² + γ²)`. `f` is concave, so the clipped
  stationary point is the maximiser. With no clipping this equals `ρ√(uᵀAu)`, the full
  projection.
- **Outer ellipse: two sound candidates, keep the one with smaller area.**
  1. Full projection: centre `μ_xy`, shape `Q = ρ² A`.
  2. Segment ⊕ ellipse: centre `m(t_mid)`, `t_mid = (t_a + t_b)/2`,
     `d = (m(t_b) − m(t_a))/2`, `r_max² = ρ² − (min_{t∈[t_a,t_b]} |t|)²`,
     `R = r_max² S`, `Q = (1 + 1/k) d dᵀ + (1 + k) R`, `k = √(tr(d dᵀ)/tr R)`;
     if `d = 0`, `Q = R`. Valid for every `k > 0` because
     `(√a + √b)² ≤ (1 + 1/k) a + (1 + k) b`, and the shadow is contained in
     segment ⊕ `R`-ellipse.
- Support function of shape `Q` is `√(uᵀQu)`. It becomes
  `GaussianSupport2D(mean=centre, covariance=Q/ρ², level=ρ)`. `Q` is inflated by a
  relative `1e-9`, then its eigenvalues are floored so the covariance's smallest
  eigenvalue is at least `1e-9` (10× `min_cov_eigenvalue`). Inflation only grows
  obstacles; the number of floored supports is recorded.
- A splat enters a window's map iff its outer ellipse's bounding box meets the workspace
  box dilated by the robot's max rotational radius. The window boundary acts as a wall,
  which is conservative.

### 3.3 Robots

| id | footprint (robot frame) | band above floor (m) | used in |
|---|---|---|---|
| `ellipse_toy` | ellipse a = 0.50, b = 0.20 | 2D only | T1 |
| `sweeper` | disc r = 0.175 | [0.02, 0.10] | T2, G |
| `quadruped` | ellipse a = 0.35, b = 0.16 | [0.02, 0.45] | T2 |
| `cylinder` (humanoid stand-in) | disc r = 0.30 | [0.02, 1.75] | T2, G |
| `uav` | disc r = 0.25 (rotor guard) | [z_c − 0.10, z_c + 0.10] | T2, G |

- `z_lo = 0.02 m` is chassis ground clearance: nothing lower can be hit, by definition of
  the robot.
- A prism takes its widest cross-section over its whole height, which is conservative.
  Multi-part robots with a different band per part are out of scope; they would need a
  pair mask inside the certified core.
- UAV: `z_c = 1.20 m` on the toy. On the showcase, G0 chooses `z_c` at least 0.25 m above
  the overhang top and at least 0.30 m below the lowest ceiling-side obstacle over the
  window. Take-off, landing and altitude changes are not modelled; the UAV query runs at
  cruise altitude only.

### 3.4 Independent 3D replay (`replay3d`)

Every returned path is re-checked against the original 3D splats. `replay3d` shares no
code with `band_shadow`, `project` or GMC; it shares only PLY decoding and the scene
definition (τ, ρ, crop, floor fit).

- Sample each `path.json` segment (TRANSLATION and ROTATION) so that between consecutive
  samples no footprint point moves more than `δ = 0.01 m` (translation: `|Δxy|`;
  rotation: `|Δθ| · max_rotational_radius`). Each sample checks the footprint dilated to
  cover that motion: a disc `r → r + δ`; an ellipse scaled by `1 + δ/b`, which contains
  ellipse ⊕ disc(δ) because its support grows by at least `δ`. The swept volume is
  covered, not only the samples.
- Prism (dilated footprint × absolute band) vs splat: minimise `(p − μ)ᵀ Σ⁻¹ (p − μ)`
  over the prism, a convex program, after AABB pruning. Collision iff the minimum is at
  most `ρ² (1 + 1e-6)`. Report the smallest margin per segment.
- Passes iff no sample collides. Negative control T2-6 must fail.

### 3.5 When a result may be called certified

All three must hold: GMC status `REACHABLE` in prototype certificate mode; GMC's own
`verify` replay `certified=True` on the 2D map; `replay3d` passes on the 3D splats. The
claim is bounded by the scene definition (§3.1), prototype rather than theorem
certificates, and unaudited GS fidelity: glass, scan holes and reflective floors can
render as free space. The gallery has glass doors; routes through glass-adjacent
openings are flagged.

## 4. Components

New package `gmc/src/gmc/height/`. The certified core (`geometry`, `orientation`,
`mobility`, `verification`, `spatial`, `reporting`) is not modified. A bug found there
is documented and escalated, not patched in this work.

| file | responsibility |
|---|---|
| `ply3d.py` | decode 3DGS PLY → means, covariances, opacities; crop; floor-plane fit; gravity rotation |
| `band_shadow.py` | exclusion test, shadow support function, outer ellipse (§3.2) |
| `prism.py` | `PrismRobot(footprint: RobotModel2D, z_lo, z_hi, name)` and the robot table |
| `project.py` | 3D scene + robot + window → `SceneModel2D`, plus stats: kept, excluded, floored, per-filter counts, provenance ids |
| `replay3d.py` | §3.4 |
| `viz.py` | §6 |
| `synth3d.py` | synthetic 3D table scene built from 3D Gaussians (§5.2) |

- Experiments: `gmc/experiments/height_toy.py`, `showcase_scene.py`, `showcase_run.py`,
  `height_viz.py`. Sbatch: `gmc/hpc/height_*.sbatch`. Configs:
  `gmc/configs/height_toy.yaml`, `height_showcase.yaml`.
- Tests: `tests/unit/test_band_shadow.py`, `test_height_project.py`, `test_replay3d.py`;
  `tests/integration/test_table_toy.py`. The existing suite must still pass.
- Data: the showcase copy lives at the absolute path
  `/scratch/wg2381/splathjb/splatc_atlas/data/gs_scenes/showcase/`, shared by every
  checkout. Committed results (JSON, PNG, videos ≤ 20 MB) go to `gmc/results/height/`.
  Raw compile run directories, PLY exports and larger videos go to
  `/scratch/wg2381/splathjb/gmc/outputs/height/`.

## 5. Phases and hard gates

A gate passes, or the run is reported as failed with its evidence. Gates are never
loosened to pass.

### 5.1 T1 — ellipse robot on toy data (`configs/toy.yaml`)

- **T1a.** `single_door` width 0.6, `ellipse_toy`, start `(−2, 0, π/2)`, goal
  `(2, 0, π/2)`: `REACHABLE`; `verify` certified; where the path crosses the wall, the
  orientation lies within `synth.gate_half_angle(0.50, 0.20, 0.6)` of the passing
  direction. The start orientation is blocked, so a rotation is required.
- **T1b.** Negative control, width 0.35 (< 2b): status is not `REACHABLE`. Prototype mode
  does not emit global `UNREACHABLE`; `UNKNOWN` is the expected status.
- **T1c.** Video of T1a.

### 5.2 T2 — synthetic 3D table

Workspace x ∈ [−3, 3], y ∈ [−2, 2], metres. A wall of flat 3D Gaussians at x = 0, height
0–2.5 m, fills y ∈ [−2, −0.6] and y ∈ [0.6, 1.2]. The gap y ∈ [−0.6, 0.6] holds a table:
top at 0.72–0.76 m over x ∈ [−0.3, 0.3] × y ∈ [−0.6, 0.6], legs of radius 0.03 m at
(±0.25, ±0.55). Variant `open` leaves y ∈ [1.2, 2.0] as a 0.8 m detour door; variant
`closed` walls it. All gaps are measured between support boundaries at ρ = 2. Start
`(−2.2, 0, 0)`, goal `(2.2, 0, 0)`. Config `height_toy.yaml` = `toy.yaml` plus the
orientation setting from T2-8.

| gate | variant / robot | pass condition |
|---|---|---|
| T2-1 | closed / sweeper | `REACHABLE`, `replay3d` pass, path xy crosses the tabletop footprint |
| T2-2 | closed / quadruped | `REACHABLE`, `replay3d` pass |
| T2-3 | closed / cylinder | not `REACHABLE` |
| T2-4 | open / cylinder | `REACHABLE`, `replay3d` pass, path xy never meets the tabletop footprint |
| T2-5 | closed / uav (z_c = 1.20) | `REACHABLE`, `replay3d` pass, path xy crosses the tabletop footprint |
| T2-6 | closed / cylinder, hand-made straight path | `replay3d` **fails** (the checker is not vacuous) |
| T2-7 | maps | by provenance id: sweeper map holds leg shadows and no tabletop shadows; cylinder map holds both |
| T2-8 | closed / sweeper | `initial_intervals` 1 vs 16: same verdict, and each of the 16 intervals' certified-safe region has the single interval's area within 1e-6 relative. If so, disc robots use 1 on the showcase; otherwise 16 |

Property tests on `band_shadow`: support dominance over 720 directions for random
ellipsoids including needles (axis ratio up to 1e4) and tilted splats; random 3D points
inside both the ellipsoid and the band always land inside the outer ellipse; exclusion
exact, with touching counted as overlap.

### 5.3 G0 — showcase scene checks and case selection

- Copy `/scratch/sy2366/Project/LccStudio-stage-archive/point_cloud.ply` (read-only
  source; 499,121,449 bytes; 7,340,008 splats; LOD-0 of LCC capture `19017822682139126`)
  to `.../gs_scenes/showcase/raw/point_cloud.ply`, verify sha256 equality, and write
  `meta.json` in the K2 convention.
- Fit the floor plane (RANSAC on means of opaque splats in the lowest z mode). If tilt
  exceeds 0.5°, rotate means and covariances into gravity alignment. Record floor height
  `z_f`, tilt, and the distribution of floor-splat upper extents. Floor splats reaching
  above `z_lo = 0.02 m` are reported as a finding, never thresholded away.
- **Scale check.** Measure at least two of: ceiling height, counter top, desk or table
  top, chair seat, door height. Metric scale is accepted if at least two agree with
  typical values within 15%. Otherwise G1 does not run, and the chain reports the
  discrepancy for the user to decide.
- Crop floaters to `[z_f − 0.2, ceiling + 0.2]`; record counts.
- **Case selection**, from top-down maps at bands [0.02, 0.10], [0.10, 0.55],
  [0.55, 1.00], [1.00, 1.75] and the cruise band, plus a per-cell lowest-obstacle-height
  map. The case must satisfy all of:
  1. the straight segment start → goal passes under an overhang whose underside is at
     least 0.15 m above the floor (sweeper band top 0.10 m + 0.05 m margin) and whose
     top is below the UAV band;
  2. a cylinder detour plausibly exists inside the window (a cheap raster check, used
     for selection only and never reported as a result);
  3. start and goal are at least 0.5 m from any obstacle in every robot's band;
  4. the window is at most 8 m × 8 m;
  5. the route does not pass glass-adjacent openings.

  The reception counter is the first candidate. If it has no knee space from the
  approach side, use another overhang. If nothing satisfies (1), say so and run the best
  available case.
- Output `gmc/results/height/showcase/case.json` (window, start, goal, `z_f`, `z_c`,
  gravity rotation, filter counts) and annotated figures, which the agent must open and
  describe.

### 5.4 G1–G3 — three robots on the showcase

- **G1.** For `cylinder`, `sweeper` and `uav`: project; record support counts; run a
  single-orientation timing probe; then compile + query in prototype mode as three
  parallel sbatch jobs. Coreset reduction is allowed only if
  `docs/worklog/coreset_phase1.md` establishes an outer (superset) guarantee for the
  reduced obstacle set. Otherwise, if the probe projects more than 20 h, shrink the
  window, never the semantics.
- **UNKNOWN handling.** Diagnose first (M_possible vs M_safe connectivity; which budget
  ran out). Then raise only budgets (`max_refinement_rounds`, `max_wall_seconds`,
  `max_support_calls`, compile wall time), which cannot change a certified answer.
  Scene, robot, τ, ρ and filters are frozen.
- **G2.** GMC `verify` and `replay3d` on every `REACHABLE` path.
- **G3.** Videos and 3D PLY (§6) first, then the report.
- The hypotheses (sweeper under, cylinder around, UAV over) are tested, not required. The
  report says which held, with video frames as the evidence.

## 6. Visualization

Rendered with matplotlib (Agg). MP4 through `imageio-ffmpeg` if the user approves
installing it into `gmc-venv` (a wg2381-owned venv on the shared anaconda 3.13 base);
otherwise GIF through pillow. At most 300 frames per video.

- **Per-robot video**, two synchronized panels:
  - top-down: opacity-weighted splat density (grey), that robot's obstacle map (shadow
    ellipses), the certified path, the footprint at the current pose, and its trail;
    the title shows verdict and `replay3d` clearance;
  - side view: arc length along the path vs height above floor, splat mass inside the
    corridor of the footprint's half-width, and the robot band as a rectangle moving
    along the path.
- **Comparison video**: the three robots' top-down panels side by side, synchronized by
  normalized progress.
- **3D PLY** per case: splat means in the window coloured by height, plus robot prisms
  sampled along each path, one colour per robot. Opens in meshlab.
- Agents open key frames with the image reader and check them against the claims before
  writing any number (repo README: visualize first, metrics second).

## 7. Execution: unattended agent chain

Follows `/scratch/wg2381/claude_jobs`: `claude -p --model claude-opus-5
--dangerously-skip-permissions` inside an sbatch allocation, wrapped in the usage-limit
loop (on "session limit" or "usage limit", sleep 20 min and retry until the allocation
deadline minus 30 min; any other non-zero exit is not retried).

- **Worktree** `/scratch/wg2381/splathjb-height`, branch `height-bands`, created from the
  commit that adds this spec. Agents commit there with the session attribution trailer
  and never push; merging is the user's call.
- **Files** under `/scratch/wg2381/claude_jobs/height/`: a prompt and a wrapper per
  stage; `state/<stage>.json` listing passed gates (every agent reads it first and
  resumes rather than restarts); `jobids/<stage>.txt` listing every job the stage
  submitted. Standups: `claude_jobs/logs/height_<stage>_done.md`. Running worklog:
  `docs/worklog/height_bands.md`, appended as problems occur, plus a Chinese summary
  `docs/worklog/height_bands_zh.md` at the end.

| stage | kind | submitted by | starts on | work |
|---|---|---|---|---|
| A1 | worker | this session | `--begin` | `height/` package, unit and property tests, T1, T2, toy videos |
| C1 | checker | this session | `afterany:A1` + `--begin` | diagnose A1 from artifacts, finish gaps, run the full suite, confirm T1/T2 gates |
| A2 | worker | this session | `afterany:C1` + `--begin` | G0, maps, timing probes; submits the G1 compute jobs, then submits A3 with `afterany` on them |
| A3 | worker | A2 | `afterany:<G1 jobs>` | collect results, UNKNOWN handling, G2, G3, report |
| C2 | checker | this session | `afterany:A2` + `--begin` | final check, below |

C2's wrapper runs a bash pre-check before starting any agent: if A3 or any G1 job listed
in `jobids/` is pending or running, it resubmits itself with `afterany` on those jobs and
exits (at most 6 resubmissions, counted in a file). Otherwise its agent verifies every
gate across A1–A3 from artifacts, finishes anything missing (including A3's work if A3
never ran or died), and writes the final standup.

Rules written into every prompt:
- write only inside wg2381 space; `/scratch/sy2366/...` is read-only;
- `scancel` only job IDs listed in the stage's own `jobids` file;
- no package installs; no edits to the certified core;
- no gate loosening and no scene-semantics changes;
- checkers judge by artifacts, not exit codes: `FAILED 1:0` has repeatedly meant a
  usage-limit death after the work was finished.

Resources:
- Agents: `cpu_short`, 8 CPU, 48 G, 6 h.
- G1 compute jobs: at most 3 at once, each at most 8 CPU, 64 G, 24 h. No partition line,
  so they route to `cs` as `k2_door_gate_stage4.sbatch` does. Account
  `torch_pr_527_general`.
- `--begin` times are chosen with the user at submission, to line up with usage-limit
  refresh. Before the first submission this session states every job and its resources
  and waits for the user's go.

## 8. Out of scope

Theorem-mode certificates on the showcase; multi-part robots with per-part bands; UAV
altitude changes, take-off and landing; GPU-rendered GS chase-camera video; dynamics
(M9); the wall-gap/glass audit itself (only flagged); merging `height-bands` into any
other branch.
