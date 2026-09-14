# Height-band robots — worklog

Spec: `docs/superpowers/specs/2026-09-12-height-band-robots-design.md`.
Plan: `docs/superpowers/plans/2026-09-12-height-band-robots.md`.
Problems and resolutions are appended as they happen, newest last.

## Stage A1 (job 17582909, started 2026-09-13 03:03 EDT on cs672)

Times below are EDT. Entries before 03:21 were first written with guessed times that ran
ahead of the clock. `date` read 03:20:48 when the gate jobs were submitted, and those
times have been corrected to approximate values (≈).

- ≈03:06 — fresh start: no `state/A1.json`, no `[height T*]` commits on
  `height-bands` (HEAD `f19a04f`). `import imageio_ffmpeg` fails in `gmc-venv`, so every
  video in this stage is written as GIF through pillow (plan Global Constraints).
- ≈03:10 — Tasks 1–4 went through as written (6, 6, 7+6, 4 tests passed).
- ≈03:14 — **Task 5 problem:** `test_clear_path_passes_with_bounded_clearance`
  failed with `min_clearance_lb = 0.0296` (test wants ≥ 0.03; true dilated gap is
  0.55 − 0.2 − 0.31 = 0.040). Root cause, measured at pose x = 0: the best gap over the
  Fibonacci direction grid is 0.0097 (256 dirs), 0.0303 (2048), 0.0383 (32768). Any
  direction with v_z ≠ 0 pays ~0.85·|v_z| through the prism's band term, and the azimuth
  misses +y. Nelder–Mead from the xy centre direction recovers 0.040000 for every splat,
  but the plan's replay only refined pairs with gap ≤ 0, so the reported minimum kept the
  grid bound. **Fix (new code only):** before accepting a new running minimum, refine that
  pair (up to 16 per footprint sample, stopping once the argmin is already refined).
  Soundness is unchanged: `refine_gap` returns the gap of an actual direction and the
  result is `max(grid, refined)`. The test is unchanged; 7/7 pass.
- ≈03:14 — **Task 8 problem:** `test_robot_video_writes_video_and_readable_frames`
  raised `ValueError: zero-size array to reduction operation minimum` inside
  `matplotlib.collections.set_alpha`. Root cause: the test splat is 0.5 m off the path,
  outside the sweeper's 0.175 m corridor, so `corridor_profile` is empty and matplotlib
  3.10 rejects an empty per-point alpha array (reproduced standalone; a one-point array is
  fine). **Fix:** skip the side-panel scatter when the corridor is empty. The corridor
  half-width stays the footprint radius (spec §6). 4/4 pass.
  Consequence for Task 8 Step 5: with the plan's smoke inputs the side panel cannot show a
  dot at 0.74 m (the splat is outside the corridor). A second smoke render moves the splat
  to y = 0.1 to exercise the dots; both are under
  `/scratch/wg2381/splathjb/gmc/outputs/height/viz_smoke/`.
- ≈03:16 — Task 6: 4/4 pass on the first run (18 s); wall top row needed no fix.
  Task 7: 3/3 pass (single_door 0.6 compile + query + verify).
- ≈03:18 — Task 8 Step 5, frames opened with the image reader
  (`outputs/height/viz_smoke/`). `sweeper_f000/f050/f100.png` (plan inputs): left panel has a
  small red shadow ellipse at (0, 0.5) with a single dark density cell at its centre (the
  second splat at y = 3 is outside the view), the dashed blue path on y = 0 from −1 to 1, and
  the blue r = 0.175 disc at x ≈ −1.0, 0.08, 1.0 with a solid trail behind it. Right panel: the
  blue band rectangle at 0.02–0.10 m, centred at s = 0, ≈1.1, 2.0; **no dots**, as expected
  from the empty corridor (see the Task 8 problem above), so the plan's "dot at 0.74" cannot
  appear with these inputs. `sweeper_on_path_f050.png` (splat moved to y = 0.1): one black dot
  at height ≈0.74 over s ≈ 1.0, above the band, and the disc covers the splat position in
  the top-down panel. The side panel draws corridor mass correctly when there is some.
  The GIFs are written (no imageio_ffmpeg).
- ≈03:19 — Task 9 Step 3, cheap gates inline:
  - **T2-7 pass** (`gmc/results/height/toy/T2-7.json`): sweeper map keeps 74 supports (legs, no
    tabletop), cylinder 148 (legs + tabletop), uav 106 (walls only). Closed scene: wall ids
    0–949, tabletop 950–1021, legs 1022–1049.
  - **T2-6 pass** (`T2-6.json`): the hand-made straight cylinder path START→GOAL is rejected;
    441 samples, 5270 pairs, 3117 refinements, 20 collisions recorded (cap), first at pose
    x = −0.60 against tabletop splat 955 (centre (−0.22, −0.047, 0.74)), gap −0.0071; min
    lower bound −0.343.
  - A `mkdir -p gmc/logs` run with the wrong cwd created an empty `gmc/gmc/logs`; checked it was
    empty and removed it. `gmc/logs/` and `gmc/results/height/` are not gitignored.
- 03:20:48 — Task 9 Step 5: `sbatch --test-only` OK (would start on cs670, partition cs).
  Submitted eight gate jobs (4 CPU / 16 G / 12 h each), recorded in
  `/scratch/wg2381/claude_jobs/height/jobids/A1.txt`:
  T1a 17611964, T1b 17611966, T2-1 17611967, T2-2 17611968, T2-3 17611969, T2-4 17611970,
  T2-5 17611971, T2-8 17611972. T1a started at 03:21:01; the rest were pending. A1 does not
  wait for them; C1 collects them.
- 03:22 — The toy gates are fast. By 03:22:23, `sacct` showed T2-1, T2-2, T2-3, T2-5 and T2-8
  COMPLETED 0:0 (27–61 s) and T1a, T1b, T2-4 RUNNING. The JSONs and media they wrote are left
  **uncommitted** for C1 to check and commit. Key frames, opened before any numbers were
  written:
  - `media/T2-1_f050.png` (sweeper): top-down shows wall shadow discs on x = 0 for |y| ≥ 0.6,
    four tiny leg shadows at (±0.25, ±0.55) and no tabletop shadow; the r = 0.175 disc
    sits at x ≈ 0 on the straight y = 0 path. Side panel: a row of tabletop dots at 0.74 m over
    s ≈ 2.0–2.45 and the 0.02–0.10 band underneath them. Picture = "under the table".
  - `media/T2-2_f050.png` (quadruped): same map as the sweeper's; the ellipse is turned
    ≈15° at x ≈ 0 and passes between the legs; band 0.02–0.45 under the dots at 0.74.
  - `media/T2-5_f050.png` (uav z_c = 1.20): only wall shadows (no legs, no tabletop); the disc
    at x ≈ 0; side panel band 1.10–1.30 above the tabletop dots. Picture = "over the table".
  - `media/T2-3_status.png` (cylinder): the tabletop's shadow grid fills x ∈ [−0.3, 0.3],
    y ∈ [−0.6, 0.6] and joins the wall shadows, so the door is visibly closed; title UNKNOWN.
    The right axes are empty (a status-only frame still makes two subplots; cosmetic).
  Numbers (from the JSONs): T2-1 REACHABLE, verify certified (min clearance 0.0900), replay3d
  pass over 449 samples, 74 supports, compile 5.1 s. T2-2 REACHABLE, certified (0.0896),
  replay3d pass (455 samples), 84 supports. T2-5 REACHABLE, certified (0.0897), replay3d
  pass (451 samples), 106 supports. T2-3 UNKNOWN (`possible_cut_is_not_a_global_certificate`),
  148 supports, 48 possible nodes. T2-8: n = 1 and n = 16 both REACHABLE, every slab's safe area
  22.106109106084222, max relative difference 0.0 → `showcase_initial_intervals = 1`.
- Two observations for C1/A3 (neither changes a gate):
  1. `replay_curve` starts `min_clearance_lb` at `margin` (0.05) and only lowers it, so
     `lb=0.05` in T2-1/T2-2/T2-5 means "no pair came within 0.05 m", not "exactly 0.05".
  2. `floored_2d` (T2-1: 32, T2-5: 34) counts splats whose ρ-extent only touches a band end:
     there t_a = t_b = ±ρ, the clipped cross-section is one point, candidate 2 has Q = 0, and
     the eigenvalue floor turns it into a ≈6e-5 m ellipse around that point. That is sound,
     and it follows from "touching counts as overlap".
- 03:25 — T1b COMPLETED: pass (UNKNOWN, `possible_cut_is_not_a_global_certificate`, 506
  supports, compile 56 s, query 78 s). T1a COMPLETED 04:19 elapsed: pass (`reachable`,
  `verify_certified`, `crossing_inside_gate` all true).
- 03:26 — *[Superseded by the 03:40 correction. The failure is real in the first-run JSON,
  but its cause is `pathio` dropping control points, not the path or GMC's verify.]*
  **T2-4 FAILS: picture/verdict mismatch.** `media/T2-4_f050.png` (open / cylinder)
  shows title `REACHABLE replay3d=False lb=-0.343`, and the r = 0.30 disc sitting on top of the
  tabletop's shadow grid at x ≈ 0. The path is a straight line START→GOAL; it does not use the
  open door at y ∈ [1.2, 2.0]. `T2-4.json`: `reachable` true, `verify_certified` **true**
  (GMC min_clearance 0.0051), `replay3d_passed` false (first collision at x = −0.60 with splat
  630, tabletop in the open variant), `avoids_tabletop` false → `pass: false`. Path =
  ROTATION to θ = π/16, TRANSLATION (−2.2, 0) → (2.2, 0), ROTATION back; 124 supports, 124
  pairs, M_safe 16 nodes / 16 edges. The independent 3D replay caught what GMC's own 2D
  verify certified.
- 03:29 — *[Superseded by the 03:40 correction. My 2D check used the control-point-free
  sampler, so it measured a straight line. Both `verify_curve` runs used the job's real curve,
  control points included. The open map certifies it correctly. The closed map correctly
  rejects it, because the closed wall at y ∈ [1.2, 2.5] blocks the door that curve goes through.
  Kept as the record of the wrong turn.]* Debugging T2-4 (systematic-debugging, Phase 1). Reproduced locally on the committed
  code with the job's own `path.json`:
  - The projected map does contain the obstacle. Open map: 124 supports, all 72 tabletop splats
    and 4 leg splats present; the closest is support 631 at (−0.22, 0.047), radius 0.08. An
    independent 2D bounding-circle check gives clearance −0.333 m along the path (the disc
    is 0.33 m inside it).
  - Unchanged core `verify_curve` on the **open** map: `certified=True, min_clearance=0.00511,
    reason=ok`. On the **closed** map, which has the same 72 tabletop supports at identical
    positions plus 24 wall supports at y ∈ [1.2, 2.5]: `certified=False, min_clearance=0.00255,
    failed_segment=1, reason=collision`. Same path, same tabletop geometry, opposite verdicts.
    Even the rejection reports a positive min_clearance for a 0.33 m overlap.
  - Next: isolate what flips the verdict (support ids, order, count) before deciding
    whether the fault is in new height code or in the read-only core.
- 03:33 — Isolation on the open map, same path, core `verify_curve` each time:
  as-is / renumbered ids 0..N−1 / reversed order → certified, min 0.00511 (ids and order ruled
  out). Tabletop + legs only (76) → certified, min 0.106. **Single projected support 631
  alone → certified, min 0.203** (true clearance −0.333). Walls and map size ruled out.
- 03:35 — Core-only control with no height code: a hand-made `GaussianSupport2D` at
  (−0.22, 0.047), cov `0.04²·I`, level 2, robot `ellipse_robot(0.3, 0.3)`, one TRANSLATION on
  y = 0 at θ ∈ {0, π/16}, lengths 4.4 / 2.0 / 0.2 m, plus a support centred on the path. Every
  colliding case: `translation_safe` and `verify_curve` → `collision`. The clear control at
  (0, 1.0) certifies with 0.157 (true 0.620). The core verifier rejects this collision on
  well-formed input, so the certified T2-4 path needs something my control does not have:
  the projected support object itself, or the job's 3-segment curve (rotation to π/16 before
  the translation). Testing those one at a time next.
- 03:40 — **CORRECTION: the core verifier is right; the bug is in new height code
  (`pathio`).** The bullets at 03:26–03:35 above misread the evidence. The claim that GMC
  certified a straight colliding path is wrong, and so is the claim that the path "does not
  use the open door".
  - Evidence: I wrapped `translation_safe` in my own process (no core edit). Verifying the
    job's curve sweeps (−2.2, 0) → (−0.18, 1.54) → (0.78, 1.42) → (2.2, 0). T2-4's TRANSLATION
    segment carries two `control_points` (`outputs/height/toy/T2-4/path.json`). My summaries
    had printed only q0/q1. My "single translation → collision" controls were hand-made
    segments **without** control points; E1 at 03:37, which reused the job's own segment,
    certifies.
  - Root cause: `pathio.sample_curve`, `polyline_xy`, `crossing_thetas` (and thus
    `path_crosses`) use only q0 and q1 and drop `control_points`. `verify_curve` treats a
    segment as q0 → control points → q1. So replay3d, the video, the side panel and
    `avoids_tabletop` all evaluated a straight line GMC never returned.
  - Confirmation with a control-point-aware sampler patched in only for the diagnostic
    process: T2-4 knots reach y = 1.54, `crosses_tabletop` false, replay3d **passes**
    (lb 0.0044, 566 samples vs 453 before).
  - Scope: of the REACHABLE gates, only T2-4's curve has control points. T1a (92 segments),
    T2-1, T2-2 and T2-5 have none, and their replays with the patched sampler are unchanged
    (lb 0.05; 449/455/451 samples). Their evidence stands.
  - Pre-fix JSONs and media preserved in
    `/scratch/wg2381/splathjb/gmc/outputs/height/toy/prefix_control_points/`.
  - Meta-rule 1 in practice: the unexamined intermediate artifact was the raw `path.json`.
    The next move was to open it, not to extend the list of hypotheses.
  - Fix plan: TDD tests for translation and rotation control points in
    `test_height_pathio.py` (plan tests untouched), sample every q0 → cp → … → q1 piece, commit
    as a T4 fix, re-run the path-consuming gates by sbatch.
- 03:40:42 — Fix committed as `9f361b3` (`[height T4] fix: sample control points…`). The two new
  tests failed first as expected: `polyline_xy` shape (2, 2) vs (4, 2), and rotation max θ 0 vs 3.0.
  After the fix: height suite **43 passed** (41 plan tests + 2 new). `sample_curve` and
  `polyline_xy` now walk q0 → control points → q1 per segment; `crossing_thetas` checks every
  translation piece. `replay3d` and `viz` pick this up with no other change. Re-submitted the gates
  that consume path geometry (T1b, T2-3, T2-6, T2-7 and T2-8 do not):
  T1a 17618957, T2-1 17618958, T2-2 17618959, T2-4 17618960, T2-5 17618961
  (`jobids/A1.txt`, purpose `rerun_after_pathio_fix`). These overwrite the first-run JSONs and
  media in `gmc/results/height/toy/`; the first-run copies are in `prefix_control_points/`.
- 03:44 — Reruns T2-1, T2-2, T2-4, T2-5 COMPLETED 0:0 (66–86 s); T1a (17618957) still RUNNING.
  Key frame first: `media/T2-4_f050.png` (rerun, written 03:41:24) now shows the dashed path
  (−2.2, 0) → (−0.18, 1.54) → (0.78, 1.42) → (2.2, 0). The r = 0.30 disc at ≈(0, 1.5) passes
  through the open door above the wall segment that ends at y = 1.2, and the tabletop's shadow grid
  is untouched. The side panel shows the 0.02–1.75 band at s ≈ 2.75 with no corridor dots
  (nearest wall sphere centres are ≈0.38 m from the path). Title `REACHABLE replay3d=True
  lb=0.0044`. The picture shows the cylinder going **around** the table, which matches the verdict.
  Numbers from the rerun JSONs:
  - **T2-4 pass**: all four criteria true; verify min 0.00511; replay3d 566 samples, 0
    collisions, lb 0.00443.
  - T2-1, T2-2, T2-5 pass with the same numbers as the first run (verify 0.0900 / 0.0896 /
    0.0897; replay3d lb ≥ 0.05; 449 / 455 / 451 samples). Their curves have no control points, so
    this was expected.
  - T1a's rerun result is for C1 to collect.

## Stage C1 (job 17582910, started 2026-09-13 08:03 EDT on cs657)

- 08:05 — Read `_rules.md`, spec, plan, README, `state/A1.json`, `jobids/A1.txt`,
  `height_A1_done.md`, the A1 agent log. No `state/C1.json` existed (fresh start). `sacct`: all 13
  gate jobs in `jobids/A1.txt` COMPLETED 0:0 (first runs 03:21–03:25, reruns 03:40–03:46).
- 08:08 — **A1 diagnosis from artifacts.**
  - All nine task commits exist (`9a3cd4f 7d14e3d a819cf5 e34d8c9 02b4af2 3bca411 8533c61 ef901e0
    51178f2`), plus fix `9f361b3` and worklog `ff1d423`, all with the trailer.
  - Every plan code block was diffed against the file at HEAD. Identical: `__init__`, `ply3d`,
    `band_shadow`, `prism`, `project`, `synth3d`, `run`, `height_toy.py`, `height_toy.sbatch`, and
    every plan test file. `height_toy.yaml` is the header line + `toy.yaml` byte for byte.
  - The only differences are the three A1 documented: `replay3d` (refine the argmin pair), `viz`
    (skip an empty scatter), and `pathio` (walk control points, +2 tests appended to
    `test_height_pathio.py`).
  - The read-only core (`geometry … io`, `types.py`, `config.py`, `synth.py`, `configs/toy.yaml`) is
    unchanged since `f19a04f`.
- 08:09 — Gate JSONs: all ten exist with `"pass": true` (criteria listed in the table below).
  `logs/height_toy-17618957.out` prints `T1a pass=True`, and `T1a.json` (03:46) is the rerun.
  The only failing log is the first-run T2-4 (17611970, the `pathio` bug A1 fixed; rerun 17618960
  passes). No gate JSON is missing, so nothing was resubmitted.
- 08:10 — Checked A1's `pathio` fix against the core rather than against its own tests.
  `verify_curve` (`verification/path.py:118`) walks `[q0, *control_points, q1]`. It checks
  translations piece by piece with `translation_safe(a, b)`. It checks rotations piece by piece with
  `rotation_interval_safe(th0, th1)` over the literal interval `[min, max]`, not the short arc.
  `pathio._knots` + linear interpolation of raw θ is the same motion. Agrees.
- 08:11 — Challenged T1a's `crossing_inside_gate`. It looks only at translation pieces that cross
  x = 0, but the `single_door` wall is 1.2 m thick (x ∈ [−0.6, 0.6]), and the path rotates in place at
  (0, 0) twice (segments 22–23 and 67). Sampling the whole curve (δ = 0.01): all 404 poses with |x| ≤ 0.6
  have folded θ ≤ 0.442, and the 43 poses at x = 0 have ≤ 0.196; gate half-angle 0.510. The criterion
  holds over the whole wall slab, not only at x = 0. The T1a witness goes −2 → 2 → −2 → 2 (3 sign
  changes of x, length 12.45 m for a 4 m gap): certified but far from short.
- 08:12 — **Bug found in new height code: `replay3d`'s KD-tree prefilter can drop a colliding
  splat (false pass).** `reach = R + margin + big_extent` (R = dilated footprint radius) assumes a
  "small" splat reaches at most `big_extent` from its centre. But `ext` is the larger *per-axis* AABB
  half-extent, and a splat elongated along the xy diagonal reaches √2·ext. Also, per-axis AABB
  contact at a corner puts centres √2 times the per-axis bound apart. So the plan's claim "the
  KD-tree prefilter never drops a pair the AABB test would keep" is false.
  - Reproduction: needle along (1,1,0)/√2, σ = 0.35 (ext 0.495, "small"), centre 0.95 m from a
    diagonal cylinder path (reach 0.86). The needle's end is 0.25 m from the path. `refine_gap` at the
    origin pose: −0.06, a real collision. `replay_curve` → `passed=True, n_pairs_checked=0`. With
    `big_extent=0.4` (needle always checked) → `passed=False`.
  - Toy gates: every toy splat has xy half-extent and xy radius 0.08 (open and closed scenes). The old
    reach already covered every AABB-kept pair there, so no toy gate can change; an inline replay
    confirms (entry below). The showcase GS has large, arbitrarily oriented splats, so G2 would have
    been exposed.
  - TDD: two new tests in `test_height_replay3d.py`; the plan's 7 tests are unchanged.
    `test_diagonal_splat_beyond_kdtree_reach_still_collides` and
    `test_kdtree_prefilter_keeps_every_pair_the_aabb_test_keeps` compare `n_pairs_checked` with a
    `big_extent=1e9` run (no prefilter). Both failed first: `assert not True`, and `0 == 8`.
  - Fix: `reach = √2·(R + margin + big_extent)·(1 + 1e-9)`. Per axis a kept centre is within
    R + margin + big_extent of the pose; in the plane it is within √2 times that. The KD-tree now returns
    a superset of the AABB test's keep set, so replay results equal the no-prefilter results. 9/9 pass.
- 08:14 — Environment finding for the full suite: 25 of the 31 tests in
  `tests/integration/test_atlas_{benchmark,gate_intervals}.py` fail in this worktree with
  `AtlasAssetError: Atlas root does not contain the sealed round4_final_package:
  /scratch/wg2381/splathjb-height/splatc_atlas`. `splatc_atlas/outputs/` is gitignored, so a git
  worktree never has it; the main checkout does
  (`/scratch/wg2381/splathjb/splatc_atlas/outputs/round4_final_package/MANIFEST.sha256`). These tests
  and `benchmarking/atlas.py` contain no write calls. Plan: record the as-is run, then run the suite
  once more with a temporary, untracked, read-only symlink `splatc_atlas/outputs →` the main checkout's
  copy, and remove it afterwards.
- 08:16 — **Key frames opened** (`gmc/results/height/toy/media/`, f000/f050/f100 each):
  - **T1a** (ellipse_toy, top-down only). Pink disc supports fill the wall slab x ∈ [−0.6, 0.6] except
    a 0.6 m door at y = 0.
    - f000: the a = 0.5 / b = 0.2 ellipse stands upright (θ = π/2) at (−2, 0), 1.0 m tall against a
      0.6 m door.
    - f050: the ellipse is at x ≈ −1.1, turned ≈25° clockwise (θ = −0.44, segment 26), and the solid
      trail already spans −2…2 (the back-and-forth witness).
    - f100: upright again at (2, 0) (θ = −3π/2 ≡ π/2).
    - None of the three frames catches the ellipse inside the door, hence the numeric check at 08:11.
  - **T2-1** (sweeper, closed). Wall shadow discs on x = 0 for |y| ≳ 0.68, with a touching seam at
    y = 1.2 between wall segments. Four tiny leg shadows at (±0.25, ±0.55). The tabletop appears only as
    faint grey density, with no red shadow.
    - The r = 0.175 disc runs along y = 0: (−2.2, 0) → (0, 0) under the tabletop footprint → (2.2, 0).
    - Side panel: the 0.02–0.10 band slides from s = 0 to s ≈ 4.4, under a row of tabletop dots at
      0.74 m over s ≈ 1.95–2.45.
    - Title `replay3d=True lb=0.05`: no splat came within the 0.05 m margin (`n_pairs_checked` 0).
    - Picture = under the table.
  - **T2-4** (cylinder, open). The tabletop shadow block [−0.3, 0.3] × [−0.6, 0.6] joins the wall
    shadows at |y| = 0.6; the wall ends at y = 1.2, and y ∈ [1.2, 2.0] is empty. Dashed path (−2.2, 0) →
    (−0.18, 1.54) → (0.78, 1.42) → (2.2, 0).
    - f050: the r = 0.30 disc at ≈(0.03, 1.5) sits in the door. Its lower edge is on the wall end, its
      top ≈0.2 m below the workspace edge.
    - f100: the disc is at the goal, with the full trail.
    - Side panel: the 0.02–1.75 band moves s ≈ 0 → 2.75 → 5.5 with no corridor dots.
    - Title `REACHABLE replay3d=True lb=0.00443`. Picture = around the table through the door.
  - **T2-5** (uav z_c = 1.20, closed). Only wall shadows (no legs, no tabletop). The r = 0.25 disc runs
    along y = 0 and is at (0, 0) over the tabletop footprint at f050. Side panel: the 1.10–1.30 band
    passes above the tabletop dots at 0.74 m. `lb=0.05` (nothing within margin). Picture = over the table.
  - All four pictures match their verdicts and criteria. The frames can show T2-1/T2-5 `lb=0.05` only as
    "≥ 0.05" (A1 note); the synthetic scene leaves ≥ 0.26 m around those paths.
- 08:20 — Reach fix checked against the toy evidence. Replayed the committed T2-1, T2-2, T2-4 and
  T2-5 curves and the T2-6 straight line with the fixed code; every field (`passed, n_samples,
  n_pairs_checked, n_refined, min_clearance_lb, worst, collisions`) is **identical** to the gate JSON
  (T2-4: 2489 pairs, lb 0.00443; T2-6: 5270 pairs, rejects, lb −0.343). Output:
  `/scratch/wg2381/splathjb/gmc/outputs/height/c1/replay_equivalence_after_reach_fix.txt`. No toy
  gate needs a rerun; T1b, T2-3, T2-7 and T2-8 do not call `replay_curve`. Height subset with the fix:
  **45 passed in 152.17s** (41 plan + 2 A1 + 2 C1). Committed as the T5 fix below.
- 08:21 — `91ab698` `[height T5] fix: replay3d KD-tree reach covers diagonal splats`.

### Toy gates (C1, 2026-09-13)

Source: `gmc/results/height/toy/<gate>.json` (every `criteria` value true, `pass: true`); job logs
`gmc/logs/height_toy-<job>.out`. The JSONs predate `91ab698`, but the fixed replay reproduces every
replay field (08:20 entry). T1b, T2-3 and T2-8 are first runs, which the `pathio` fix cannot affect:
no curve, or no curve consumer.

| gate | case | job | criteria (all true) | pass | evidence |
|---|---|---|---|---|---|
| T1a | `single_door` 0.6, ellipse_toy, (−2,0,π/2) → (2,0,π/2) | 17618957 (rerun) | reachable, verify_certified, crossing_inside_gate | yes | REACHABLE; verify min 0.00581; 484 supports, 56 slabs; crossing θ folded 0.196 at x = 0, ≤ 0.442 over \|x\| ≤ 0.6, half-angle 0.510 |
| T1b | `single_door` 0.35, ellipse_toy | 17611966 | not_reachable | yes | UNKNOWN (`possible_cut_is_not_a_global_certificate`); 506 supports |
| T1c | video of T1a | 17618957 | — | done | `media/T1a.gif` (1.3 MB), `T1a_f000/f050/f100.png` |
| T2-1 | closed / sweeper | 17618958 (rerun) | reachable, verify_certified, replay3d_passed, crosses_tabletop | yes | verify min 0.0900; replay3d 449 samples, no pair within 0.05 m (lb ≥ 0.05); 74 supports |
| T2-2 | closed / quadruped | 17618959 (rerun) | reachable, verify_certified, replay3d_passed | yes | verify min 0.0896; replay3d 455 samples, lb ≥ 0.05; 84 supports |
| T2-3 | closed / cylinder | 17611969 | not_reachable | yes | UNKNOWN (`possible_cut_is_not_a_global_certificate`); 148 supports; M_safe = M_possible = 48 nodes / 48 edges |
| T2-4 | open / cylinder | 17618960 (rerun; first run 17611970 failed on the `pathio` bug) | reachable, verify_certified, replay3d_passed, avoids_tabletop | yes | verify min 0.00511; replay3d 566 samples, 2489 pairs, lb 0.00443; path via (−0.18, 1.54), (0.78, 1.42) |
| T2-5 | closed / uav, z_c = 1.20 | 17618961 (rerun) | reachable, verify_certified, replay3d_passed, crosses_tabletop | yes | verify min 0.0897; replay3d 451 samples, lb ≥ 0.05; 106 supports |
| T2-6 | closed / cylinder, hand-made straight line | inline, A1 `51178f2` | replay3d_rejects | yes | 441 samples, 5270 pairs, 20 collisions recorded (cap), first at x = −0.60 on tabletop splat 955; worst gap −0.343 (splat 1016) |
| T2-7 | closed maps by provenance id | inline, A1 `51178f2` | sweeper_legs_no_top, cylinder_legs_and_top, uav_walls_only | yes | supports: sweeper 74, cylinder 148, uav 106 |
| T2-8 | closed / sweeper, `initial_intervals` 1 vs 16 | 17611972 | same_status, areas_within_1e-6 | yes | both REACHABLE; 1 slab and 16 slabs, every safe area 22.106109106084222 m², max rel. diff 0.0 → **`showcase_initial_intervals = 1`** |

All T1/T2 gates pass. No gate criterion was changed and no gate was rerun in C1.
- 08:26 — Full suite, **baseline at `ff1d423` (A1's HEAD), worktree as-is**, exact command
  `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q` (+ `--junitxml` for the counts, since the
  doubled `-q` hides the summary line): **449 tests, 424 passed, 25 failed, 0 errors, 1083.9 s**. All 25
  failures are in `tests/integration/test_atlas_benchmark.py` (10) and `test_atlas_gate_intervals.py`
  (15): 24 `AtlasAssetError: Atlas root does not contain the sealed round4_final_package` and one
  `FileNotFoundError` under `/scratch/wg2381/splathjb-height/splatc_atlas/outputs/…`. Same cause for all:
  the gitignored package is absent from the worktree (08:14 entry). No failure in the height tests or in
  any other module. 449 = 353 inherited + 53 coreset (`docs/worklog/coreset_phase1.md`: 406/406) + 43
  height. Log/XML: `/scratch/wg2381/splathjb/gmc/outputs/height/c1/suite_baseline_ff1d423.{log,xml}`.
- 08:27 — Final run started on HEAD `91f11d1` (code = `91ab698`) with a temporary, untracked symlink
  `splatc_atlas/outputs → /scratch/wg2381/splathjb/splatc_atlas/outputs` (read-only use).
- 08:45 — Final run 1 on `91f11d1` with the `splatc_atlas/outputs` link: **451 tests, 450 passed,
  1 failed, 1049.9 s**. The 24 package tests now pass. The remaining failure,
  `test_atlas_benchmark.py::test_blind_opt_in_keeps_withheld_manifest_scope_explicit`, needs
  `splatc_atlas/data/splatc_gates/blind/manifest.json`, which is also gitignored. Only the two atlas test
  files reference `splatc_atlas` data. Added a second read-only link: a real, ignored
  `splatc_atlas/data/` directory holding a symlink `splatc_gates →` the main checkout's copy. Then reran
  the whole suite for a single clean summary line.
- 09:02 — **Final full suite, HEAD `91f11d1` (code `91ab698`), both read-only atlas links present:**
  exact command `cd gmc && PYTHONPATH=src:experiments $PY -m pytest -q` (+ `--junitxml`) exited 0.
  JUnit: **451 tests, 451 passed, 0 failed, 0 errors, 0 skipped, 1044.4 s**
  (353 inherited + 53 coreset + 45 height). Log/XML:
  `/scratch/wg2381/splathjb/gmc/outputs/height/c1/suite_final2_91f11d1.{log,xml}`. Both symlinks and the
  empty `splatc_atlas/data/` were removed afterwards; the main checkout's package and blind manifest
  are untouched. `git status` is clean apart from this worklog.
- 09:03 — C1 closed. All T1/T2 gates pass; one soundness fix (`91ab698`); gate artifacts committed
  (`91f11d1`). No Slurm jobs were submitted by C1, so A2 has nothing of C1's to wait for. Standup:
  `/scratch/wg2381/claude_jobs/logs/height_C1_done.md`.

## Stage A2 (job 17582911, started 2026-09-13 13:03 EDT on cs638)

- 13:04 — Read `_rules.md`, spec, plan, README, `height_C1_done.md`, `state/A1.json`, `state/C1.json`.
  There was no `state/A2.json`, so this is a fresh start. HEAD is `e4a34da` and the tree is clean.
  - **Pre-flight:** all ten gate JSONs in `gmc/results/height/toy/` (T1a, T1b, T2-1…T2-8) have
    `"pass": true`, and C1's standup reports no unresolved failure, so the showcase starts.
    `showcase_initial_intervals = 1` (`T2-8.json` `extra`).
  - The helpers Task 10 uses (`load_3dgs_ply`, `fit_floor`, `gravity_rotation`, `rotate_scene`,
    `crop_box`, `aabb`, `band_overlap_mask`, `robot_table`, `max_radius`) match the plan's script, so it
    is written verbatim.
  - The source PLY is 499,121,449 bytes (mtime 2026-09-08 16:15) with 7,340,008 vertices. Its header
    bounds are x −15.60…43.81, y −6.95…42.16, z −15.33…12.06: a ≈59 × 49 m extent, so floaters or
    neighbouring spaces are present. The floor and ceiling fits must be checked in pictures.
- 13:07 — A first launch wrapped the steps in `/usr/bin/time -v`, which is not installed on cs638.
  Nothing ran. Relaunched without the wrapper.
- 13:07 — **Task 10 Step 2 (copy + decode)**, log
  `/scratch/wg2381/splathjb/gmc/outputs/height/a2/step2_copy_decode.log`, took 16 s.
  - Copy: sha256 of source and copy equal, `98af4928cc67523015bd32c4f438d334adb86db59bc20ef1c97227bf367cc3c2`;
    `meta.json` written.
  - Decode: 7,340,008 rows, 0 non-finite dropped.
  - Floor fit: `z_floor = −1.2272`, tilt 0.285° (≤ 0.5°, so no gravity rotation; identity is stored),
    42,709 inliers. The fit's own histogram peak is `peak_z = −1.3207`, **9.4 cm below the plane**. Checking
    what these two levels are before trusting `z_f`.
  - Ceiling: z = 4.083, **5.31 m** above `z_f`.
  - Crop `[z_f − 0.2, ceiling + 0.2]`: 20,583 dropped, 7,319,425 kept.
  - **Finding (spec §5.3, reported, `z_lo` unchanged):** opaque (τ = 0.3) splats with centre within 5 cm
    of `z_f` have ρ-top above floor at p50 0.020, p90 0.062, p99 0.136, p99.9 0.236 m. **50,305** of them
    reach above `z_lo = 0.02`, and the sweeper band is [0.02, 0.10], so its map will contain floor splats.
- 13:09 — **Floor check (pictures first).** Diagnostics are in `/scratch/wg2381/splathjb/gmc/outputs/height/a2/diag/`
  (script `a2/floor_diag.py`, not committed).
  - `z_hist.png`: the floor is one thick layer of splat centres from z ≈ −1.42 to −1.15. Its densest
    5 mm bins are at −1.25…−1.20, where the red `z_f` line sits.
  - `peak_z = −1.32` is a local maximum on the lower shoulder of that layer, not a second floor. The
    fit's 2 cm histogram took the first bin ≥ 20 % of max. The RANSAC plane with the most ±2 cm inliers
    is the dense layer, which is the right floor.
  - `xy_layers.png`: splats within 2 cm of `z_f` trace wall feet over the whole floor plan. Splats at
    `peak_z` cluster under two spots, (10, 8) and (12, 34), the tables/objects below. They look like
    reflections below a glossy floor.
  - The floor's 25 cm thickness in splat centres is the reason for the 50,305 floor splats in the sweeper
    band. **`z_f = −1.2272` accepted**, and nothing is changed.
- 13:09 — **Task 10 Step 3 (maps)**, log `a2/step3_maps.log`, 10 s. Extent x −4.43…21.15,
  y −0.96…37.85; 10,064 overhang-candidate cells. Opened all five `figs/band_*.png` and
  `figs/overhang_candidates.png`. I also opened `diag/topdown_full.png` (RGB from the PLY's `f_dc`, lowest
  underside, sweeper-band hit count) and `diag/side_views.png`.
  - Building: one gallery rotated ≈62.6° (long walls along u = (0.46, 0.89)).
    - Main hall x ≈ −4…20, y ≈ 0…23; north wing/corridor up to y ≈ 38; an entrance vestibule at the SW
      corner (x ≈ −4…0, y ≈ −1…3, with cyan/blue speckle).
    - The semicircular bulge on the SE outer wall at ≈(12.5, 5.5) shows a curved low object in the
      0.10–0.55 band. Candidate glass entrance; to be checked before any route near it.
    - Freestanding partitions are 4.5 m tall. Ceiling at ≈5.0–5.3 m, with pendant lights at 4.4–4.9 m
      (`figs/section_room_long.png`).
  - Square hollow columns/plinth boxes, ≈0.8 m, visible in every band from 0.10 m up to 2.50 m:
    (−1.2, 9), (3.8, 6.5), (8.8, 4.2), (12.3, 11.5), (2.2, 16.5).
  - Round objects in the north wing at ≈(12, 34): up to 1.75 m, ring-shaped in the underside map.
    Tables or planters with chairs.
  - Objects in the 0.55–1.00 band but open underneath (green/yellow, 0.7–0.9 m, in the lowest-underside
    map) are the overhang candidates. A (9.2, 7.5) is a 2.8 × 0.9 m tan rectangle with dark items. Also
    B (0.4, 10.8), C (13.5, 14.1), D (5.8, 18) small, (2.5–3.3, 15–17.5) at the L-shaped column,
    E (11, 28), F (15.5, 10) and G (3.2, 22.5).
  - Benches (0.10–0.55 band only, underside ≈0.35–0.45 m): (10, 16), (7.8, 28.5) and (2.2, 10.5).
    `section_room_long` crosses (10, 16) lengthwise, a 2.0 m bench with a flat seat at **0.43 m** and legs
    at both ends.
  - `overhang_candidates.png`: red clusters at A, B, the L-column (2, 15), D, G and outside the west wall.
    Outside-wall hits at x < −1, y 15–20 are exterior clutter seen through glass/windows.
  - **Sweeper band (0.02–0.10) is the problem band.** The AABB occupancy map is nearly solid in the NE
    half (x > 9, y > 12) and the north wing, and speckled in the SW hall. The per-cell hit count
    (`topdown_full.png`, right) is light but everywhere, so floor splats are spread over the whole floor.
    The case start/goal must sit where this band is clear. Reported as scene fidelity; `z_lo`, τ and
    the filters are not changed.
- 13:14 — **Zoomed diagnostics** (`a2/zoom_diag.py`, `diag/zoom_<tag>.png`; 2.5 cm cells, panels RGB /
  lowest underside / highest top / lintel). Opened `full, A, B, C, G, SE, ENT`. Sections
  `figs/section_{A_u,A_v,C_u,C_v}.png` opened.
  - A `section --seg -0.52,…` call failed because argparse reads a leading `-` as a flag. Negative
    coordinates are passed as `--seg=-0.52,…`.
  - **A (9.2, 7.5)**: a long table.
    - Top surface ≈0.80 m (ρ-extent 0.73–0.88 in `section_A_v`; top-height modes 0.80/0.88), 2.95 m ×
      0.82 m. Legs at both ends plus a middle trestle at ≈(9.08, 7.56); open knee space.
    - Objects lie on it (dark items in RGB).
    - A 1.95 m bench, seat **0.42–0.44 m** (`section_A_u`, `A_v`), is attached along its NE side.
  - **C (13.5, 14.1)**: a 1.8 × 0.6 m high table with slab ends (0.1 m thick, floor to top). Top
    **≈0.92–0.94 m** (modes 0.94/0.92, `section_C_v`), underside ≈0.82, open between the slabs; objects on top.
  - **B (−0.05, 10.8)**: a 2.3 × 0.7 m table, underside ≈0.7, top ≈0.76–0.86 (`B_v` modes). Its SW end meets
    a plinth box at (−1.2, 9.2). A bench, seat ≈0.4, at (2.1, 10.5).
  - **G (3.3, 22.5)**: a 1.8 × 0.9 m table against the NW corner wall, top ≈0.95, underside ≈0.8.
  - **SE bulge (12.7, 5.7)**: a semicircular low platform (top ≈0.3 m) against the outer wall, with ≈1.2 m
    boxes on the wall line. Not an opening.
  - **Reception area (ENT)**: the counter is here, not in the hall.
    - A 3.5 m strip from ≈(−0.5, 3.0) to (0.1, −0.5): highest top ≈1.0–1.05; lowest underside ≈1.0 on the
      outer (west) edge and ≈0.3 inside.
    - A ≈0.7 m work surface runs behind it to the east.
    - The west wall x ≈ −3.9, y ≈ 0.3–2.8 has cyan/blue speckle in RGB and a lintel strip: free in
      [0.10, 1.80], with mass above whose underside is at ≈2.3–2.6 m. Outside it (x < −3.9) there is a ≈2.5 m
      canopy-like layer. **This is the glass entrance door**, 3.5–4 m from the counter.
  - Next: sections across the counter (knee space from the visitor side?) and along the door.
- 13:17 — **Reception counter and door sections** (opened `figs/section_counter_{n1,n2,len}.png` and
  `figs/section_door_W.png`).
  - **Counter:** across it (s increases eastwards, away from the entrance), the visitor-side face is a solid
    panel from ≈0.08 m up to the top ledge at **1.02–1.04 m**. The ledge overhangs the panel by only ≈0.15 m.
    Behind it, an ≈0.75 m work surface sits ≈1 m further east.
    - `counter_len` shows the panel is continuous over 3.8 m (vertical panel joints every ≈0.9 m), with a
      structure from 1.2 m to the ceiling over its south end.
    - **No knee space from the approach side, so by spec §5.3 the case uses another overhang.**
  - **Door:** along the west wall, glass double doors (each ≈0.75 m) for s ≈ 1.67–3.35, i.e. y ≈ 0.9–2.6 at
    x ≈ −3.9. Frame top at **2.37 m**, push bar at 0.97 m, bottom rail 0.08 m. Side lights at s ≈ 1.1–1.5 and
    3.5–3.9, top 2.5 m. This is the glass entrance; routes must stay away from it.
- 13:18 — **Task 10 Step 4 scale check** → `gmc/results/height/showcase/scale_check.json`. Rule: agrees iff
  measured is in [0.85·lo, 1.15·hi].

  | object | measured | typical | agrees |
  |---|---|---|---|
  | hall bench seat | 0.43 | 0.40–0.50 | yes |
  | table A top | 0.80 | 0.70–0.78 | yes |
  | bench seat at A | 0.43 | 0.40–0.50 | yes |
  | counter top | 1.03 | 0.90–1.10 | yes |
  | door clear height | 2.37 | 2.00–2.40 | yes |

  5/5 agree → **`metric_accepted: true`**, and G1 may run. Not counted: high table C (0.93, purpose
  unknown); ceiling 5.0–5.3 m (no typical range in the plan).
- 13:20 — **Task 10 Step 5, case selection. Counter rejected (no knee space); table A tried first.**
  - Selection aid `a2/case_aid.py` reuses the plan's `_occupancy`/`_section` in window
    [5.5, 3.5, 12, 10.5], z_c = 1.20. It scored 18 straight lines along u across table A, at crossing
    points s_v ∈ {1.3…3.3} and half-lengths 1.6–2.4 m. Log `a2/case_aid_A.log`; figure `diag/case_aid_A.png`
    (opened).
  - **Criterion 1 would pass on every line:** 41–63 bins that are free in [0.02, 0.15] and overhung.
  - **Criterion 3 fails on every line:**
    - Start/goal distance to an AABB-occupied cell is 0.00–0.29 m in the sweeper band, and identical in
      the cylinder band, which contains the sweeper band.
    - The sweeper-band distance map around A is almost entirely below 0.3 m. Every cell holds the AABB of
      some floor splat whose ρ-extent reaches 0.02 m.
  - Whole-scene check `a2/clear_map.py` (`diag/clear_map.png`, opened):
    - **42 %** of cells in the ≈26 × 39 m extent are sweeper-band occupied (AABB aid).
    - Cells ≥ 0.5 m clear in all three bands form 29 clusters. The three large ones lie **outside** the
      building walls.
    - Inside, the only sizeable one is ≈6.6 m² at (1.7, 13.2) in the SW hall. The rest are ≤ 0.14 m²
      specks, e.g. (2.32, 7.66), (1.82, 5.93), (6.28, 13.66), (5.0, 20.38), and a few in the reception area.
  - Possible contributor: residual floor tilt. The fitted normal (−0.00494, −0.00059, 1) is 0.285°, below the
    spec's 0.5° rotation threshold, so no rotation is applied. The plane then lies up to ≈+5 cm above
    `z_f` on the east side (x ≈ 18) and ≈−4 cm on the SW side, matching the denser clutter in the NE.
    Checking against local floor modes. **The threshold is scene definition and is not changed.**
  - Next: search every pair of interior clear cells for criteria 1–3 (`a2/pair_search.py`) before
    deciding anything else.
- 13:26 — **Local floor modes vs the fitted plane** (`a2/pair_search.log`; opacity > 0.5 centres within
  0.15 m of `z_f`, 1 cm bins, 2 × 2 m boxes).
  - East side, mode / plane offset relative to `z_f`: (13, 14) 0.005/0.021, (15, 20) 0.035/0.034,
    (18, 14) 0.045/0.046, (9, 28) 0.005/0.009, (12, 34) 0.035/0.028. The tilt is real, and floor splats in
    the east sit 2–5 cm up, inside the sweeper band.
  - SW: modes −0.045 … −0.145 against plane −0.002 … −0.041, with few opaque splats. These look like the
    reflection layer under a glossy floor rather than the surface.
- 13:27 — **Pair search: no pair meets criteria 1–3.**
  - 90 candidate endpoints (all interior clear clusters, 0.15–0.4 m sampling), 967 pairs of 2.5–7.5 m
    (`a2/pair_breakdown.log`).
  - Criterion 1 (overhang) passes for 370 pairs, criterion 2 (cylinder raster connectivity) for 167,
    **both for 0**.
  - The cylinder-band free raster (distance > 0.30) splits into 65 components. The interior candidates sit in
    17 different small ones. The only connected pairs are within one blob, which holds no overhang.
  - The inline section was asserted equal to the plan's `_section` on one line.
- 13:30 — **Visualized the certified per-robot maps** (`project_scene`, not the aid), 2.5 cm raster
  (`a2/proj_map_diag.py`; opened `diag/proj_map_A.png`, `diag/proj_map_SW.png`).
  - **Window A** [5.5, 3.5, 12, 10.5]:
    - Sweeper: 3,876 supports, 3,037 of them centred < 0.10 m above the floor. Red floor-splat shadows cover
      the window in 0.1–0.4 m blobs; the disc r = 0.175 fits in only 14.6 % of it, in isolated patches.
    - Cylinder: **112,068** supports; table + bench are one solid block; the disc fits in 2.6 %.
    - UAV: 11,012 supports; 77 % free and connected; only the column box is red.
  - **Window SW** [−2, 8.5, 5, 15] is much cleaner for the sweeper: 2,749 supports, disc fits in 58.5 %,
    scattered small blobs. Cylinder has **210,822** supports (table B and the bench as blocks, disc fits
    in 33.5 %). UAV 33,946, free.
  - The cylinder support counts (1–2 × 10⁵ in ≤ 7 m windows) mean compile time is a separate risk even
    with a valid case.
- 13:31 — **Diagnostic only, not the scene definition: gravity-aligned copy** (`a2/rot_diag.py`,
  `a2/rot_diag.log`).
  - Refit tilt 0.030°, `z_f` −1.2259. Floor splats reaching 0.02 m: 35,072 (was 50,305); tops p50 0.009,
    p90 0.054, p99 0.126. Sweeper-band AABB occupancy 43.8 % (was 41.8 %).
  - The SW clear blob shrinks to 18 cells: 35 candidates, 61 pairs, crit1 15, crit2 6, **both 0**.
  - **Rotating the 0.285° tilt away does not produce a valid case.** The limit is the floor layer's own
    thickness (ρ = 2 extents of floor splats reach 5–13 cm), not the tilt.
- 13:32 — **Task 10 Step 5 `--step case`, best available case: SW bench**. Log `a2/step5_case_SWbench.log`.
  - Inputs: window [−1.5, 6.5, 5.5, 14.2], start (2.12, 7.72, 0), goal (1.93, 13.02, 0), z_c = 1.2,
    `--glass-clear yes`. The route runs along x ≈ 2, 3.5–5 m east of the west wall, through no opening. The
    wall in B/G zooms shows no glass speckle; the glass doors are at x ≈ −3.9, y 0.9–2.6, outside the window.
  - Opened `figs/case_overview.png`. Left: the window's 0.02–1.75 AABB map, with table B as a diagonal block
    at x ≈ −1…0.5 and the bench as a block at (1.4–2.7, 9.3–11.5), plus scattered floor clutter. The red line
    runs S→N through the bench block. Right: a bench leg (0–0.40) at s ≈ 1.75 and the seat at 0.38–0.45 over
    s ≈ 1.8–3.66. The sweeper band runs under the seat, the UAV band 1.1–1.3 is empty, the cylinder band
    crosses the seat, and a pendant light hangs at 4.6.
  - `case.json`: overhang top 0.448, underside 0.326, s 1.80–3.66; ceiling-side obstacle 4.55. Every
    criterion is true **except `cylinder_detour_plausible_raster: false`**, so **`pass: false`**.
  - The plan and spec allow continuing without a passing case only when criterion 1 fails. Here criterion 2
    fails, and it fails for every candidate pair in the scene. **G1 is not submitted on this case; this is
    escalated** (see standup).
- 13:34 — **Task 11 Steps 1–3**:
  - `configs/height_showcase.yaml` is built by the plan's sed from `T2-8.json`
    (`showcase_initial_intervals = 1`). `diff` against `toy.yaml` shows exactly the header line plus
    `initial_intervals: 1`, `max_support_calls: 50_000_000` and `max_wall_seconds: 7200`.
  - `experiments/showcase_run.py` and `hpc/height_showcase_robot.sbatch` are written verbatim. The helpers
    they call (`with_overrides`, `compile_and_query`, `replay_curve`, `curve_from_dict`) match their
    signatures.
  - `sbatch --test-only --export=ALL,ROBOT=cylinder` would start on cs671 in partition `cs`. It created
    no job.
- 13:35 — **Task 11 Step 4 inline dry-check** on `case.json`, with the plan's exact one-liner
  (`a2/task11_step4_dry_check.log`). All three robots: n_input 7,319,425, dropped_opacity 3,448,924,
  floored_3d 0.

  | robot | excluded_band | outside_window | deduplicated | kept | floored_2d | candidate2 |
  |---|---|---|---|---|---|---|
  | cylinder | 1,548,256 | 2,121,916 | 0 | **200,329** | 14 | 2,288 |
  | sweeper | 3,785,433 | 82,256 | 0 | **2,812** | 2 | 1,199 |
  | uav | 3,420,899 | 413,870 | 0 | **35,732** | 18 | 2,818 |

- 13:36 — **Criteria 2–3 re-checked on the certified maps of this case** (diagnostic only, not a plan
  criterion; `a2/case_certified_check.py`, `figs/diag/case_certified_check.png`, opened).
  - Sweeper: shadows are scattered floor blobs and bench legs. Start clear 0.55 m, goal 1.18 m; start and
    goal lie in the same r = 0.175 free component, which covers most of the window.
  - **Cylinder**: table B and the bench are solid blocks.
    - The ≈0.6 m gap between them is closed by floor-splat shadows at (0.4–1.0, 9.8–10.8) under r = 0.30.
    - East of the bench, clutter at (2.5–4.5, 9.5–12) meets the partition wall.
    - The start component (y ≈ 7–9.8) does **not** reach the goal (north). On the certified map as well,
      there is no detour in this window.
  - UAV: only walls are red; start and goal are connected.
- 13:37 — **Decision: stop before G1 and escalate.**
  - The spec requires all five case criteria. It allows running the best available case only when criterion
    1 fails.
  - Criterion 2 fails for all 967 candidate pairs, and the certified map confirms it for the best case.
  - The cylinder map would carry 2 × 10⁵ supports, so the probe would likely abort (> 20 h).
  - Floor treatment is scene definition (spec §3.1, §5.3: floor splats are reported, never thresholded),
    and so is the tilt threshold. Neither is mine to change.
  - The case, the evidence and the ready job code are committed. No Slurm job is submitted and A3 is not
    submitted. Options are in the A2 standup.

## Stage A4 (job 17731928, started 2026-09-13 23:04 EDT on cs787)

Executes Amendment 1 (`docs/superpowers/plans/2026-09-13-floor-rule-amendment.md`), Tasks F1–F4.

- 23:09 — Read `_rules.md`, spec, plan (Global Constraints, bookkeeping, Tasks 10–12), README, the
  amendment, `height_A2_done.md`, `state/A2.json` and the A2 worklog section. There was no `state/A4.json`,
  so this is a fresh start. HEAD is `8340337` and the tree is clean. C2 17731929 waits on this job.
  - **Plan review, F1 test 6.** As worded (2° tilt, z offset ≈ 0.035 at x = ±1), the test would also pass
    under a *vertical* offset check, because 0.035 < 0.05. It therefore does not prove what it claims. The
    test is kept as written, with two discriminating additions:
    - on-plane splats at x = ±2 (vertical offset 0.07, normal offset 0) must be removed;
    - a flat splat at x = −1 with vertical offset 0.025 but normal offset 0.06 must be kept.
    Thresholds unchanged.
- 23:22 — **F1 `height/floor.py`** (TDD).
  - Wrote `tests/unit/test_height_floor.py` first: tests 1–8 of the amendment plus a stats test for
    `apply_floor_rule`. It failed on collection (`ModuleNotFoundError: gmc.height.floor`), as expected.
  - Then wrote `floor.py` verbatim from the amendment: 9/9 pass.
  - **Mutation check** (in-memory patch, not committed):
    - a vertical-offset variant fails `test_offset_is_measured_along_the_plane_normal` only;
    - a thickest-axis variant fails three tests (flat floor, tilted plane, chunked).
    So test 6 now discriminates the normal-offset requirement.
  - The full `tests/unit` suite printed 189 dots with no F or E (4 m 45 s).
