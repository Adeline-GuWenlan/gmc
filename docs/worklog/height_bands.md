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
