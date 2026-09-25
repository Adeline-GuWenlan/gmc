# Worklog: ground 5000-pair benchmark (chain `claude_jobs/ground5k`, branch `ground-5k`)

## G1 — sampler, pilot, projection, launch

### 2026-09-25 session 1 (job 18486030), usage-limit break, session 2 (job 18487475)

**Task 0: pin the planners.** I read:
- the plane-floor standups P2, P3 and P5;
- `docs/uavlamp_report.md` and `docs/gs3d_architecture.md`;
- the GMC ground path (`showcase_run.py` → `height/run.py` → `mobility/query.py`);
- the gs3d path (`gs3d/planner.py`, `oracle.py`, `robots.py`, `experiments/gs3d_integration.py`).

The design is in `docs/ground5k_design.md` §1. Decisions the code does not make obvious:

- **GMC in prototype mode cannot say UNREACHABLE** (`query.py:682`). Its "exhausted" answer is
  `UNKNOWN / possible_cut_is_not_a_global_certificate`, which maps to `FAIL_NO_PATH`.
- **A\* labels a search that ran into the crop boundary `map_unknown` or `verification_failed`.**
  The runner therefore counts rejections per region face (observation only) and maps crop-face-only
  exhaustion to `FAIL_NO_PATH`. GMC's workspace boundary is a hard wall too.
- **Contact strip.** The gs3d constant-floor support refuses any region where the fitted floor
  deviates by more than 0.05 m from `z_floor`. The fitted floor is tilted 0.285°, which rules out the
  SW corner and an east strip. So `R = Q ∩ strip` for both planners (GMC gets it as its workspace
  polygon), and the sampler admits endpoints only inside the strip.
- **GMC's 1800 s cap:** its query deadline is set to the time left after compile. The field is
  written in place because the decomposition binding checks config identity. A watchdog covers
  compile.

**Task 1: sampler.**
- **Process note.** I drafted `ground5k_sample.py` / `ground5k_common.py` before their tests. On
  noticing, I moved the code aside, wrote `tests/unit/test_ground5k_sample.py`, and saw it fail (module
  missing). Then I restored the code and ran six mutation checks:
  - separation, filter (a), filter (b), filter (c), evidence height band, shuffle;
  - each makes at least one test fail.
- **Floor evidence, first version (job 18487348).** Centre-count raster: 401 m² observed, speckled
  (`results/ground5k/figs/floor_evidence_scene_v2.png` of that run showed pinholes in open floor).
- **Floor evidence, frozen version (job 18499321).** Each splat marks its level-2 footprint (test
  first): 556.8 m² observed, 477.9 m² of it inside the contact strip. The figure, which I opened, shows
  one solid hall with holes only under objects. Orange (outside the strip) is the SW corner (x < 0)
  and x > 18 at y 15–25. Only 137 floor-layer splats were ignored as > 0.5 m.
- **First lattice + sample job 18499399 failed.** Strip-clipped regions have vertices exactly at
  |dev| = 0.05, so roundoff made the gs3d support constructor refuse. Fix: the strip is inset by
  1 µm (regression test added). Resubmitted as 18499463.
- **Witness lattice, sweeper** (0.25 m): 7,642 candidate nodes on known floor, 4,351 certified free.
  2,528 are occupied by Minkowski overlap witnesses, i.e. residual near-floor splats in the
  0.02–0.10 m band (P1b reported a 19.6 % low-band occupancy after the plane edit). 14,865 of 14,969
  tested edges are free. The sweeper lattice took 39 s.

**Task 2: runner** (`ground5k_run.py`, tests first; RED = module missing, then GREEN).
- 29 tests; 46 in total with the sampler's.
- On the synthetic hall, all four combinations certify and replay a detour.
- A fully walled hall is `FAIL_NO_PATH` for A\* (`no_path_on_lattice`) and for GMC (possible-graph
  cut).
- The watchdog kills at the wall and memory caps, and the runner resumes.
- **Sampler job 18499463.** Lattice: cylinder 292 s. Sampling: 5,000 pairs from 7,123 pair draws and
  67,652 endpoint draws in 1,003 s. Plot opened.
- **North wing.** It is nearly empty of endpoints (0.6 %). I checked why before accepting it:
  `witness_components_scene_v2.png` shows the wing's known floor mostly not certified free for either
  robot (near-floor residual splats: the round-table cluster P1 flagged), and its free islands
  disconnected from the main hall. This is the stated bias of filter (c), not a sampler bug.
  Recorded in design §3.3.
- **Pair list committed before any planner run:** 7.9 MB, so committed gzipped (SHA-256 of the plain
  JSON `f598ee358ae740d0a9b719e7f3024d87fce58598e680ce7133e3441cf6fd7115`).
- **Pilot, first attempt: runner bug.** A\* arrays 18500647 / 18500649 crashed on every call. The frozen
  config's `astar.budget` has a descriptive `max_wall_s` string, and I forwarded it next to the computed
  wall budget (TypeError). I cancelled both arrays and deleted the 33 ERROR JSONs they wrote; each had
  exactly that TypeError. The runner tests now take their planner sections from the frozen config
  (RED, then GREEN). A\* pilot resubmitted.
- **Pilot, GMC sweeper: 5 of the first 18 were FAIL_REPLAY**, all `geometry_or_margin_unproven`.
  Diagnostic job 18501088 (`ground5k_replay_diag.py`) found:
  - each rejected edge is held up by one floor splat whose level-2 top is at floor + 0.0199–0.0200 m;
  - the true gap to the sweeper chassis (+0.020) is 0.2–0.8 mm;
  - no collision, but below the shared 1 mm margin.

  GMC's band test drops a splat lying wholly outside the band, so the runner had given GMC zero
  vertical margin. That was a runner inconsistency, not a GMC parameter. Fix: GMC projects with its
  band inflated by the margin (regression test: a splat 0.5 mm under the chassis across the straight
  line; RED showed the same FAIL_REPLAY, GREEN makes both planners go round).

  Pilot GMC arrays 18500657 / 18500659 were cancelled. Their 40 JSONs went to
  `outputs/ground5k/scene_v2/superseded_pilot_gmc_band/`, and the GMC pilot was resubmitted
  (18501688, 18501689).
- The runner now also records the rejected replay edge (poses, implicated Gaussian ids, bound) and
  GMC's `failed_invariants` for INTERNAL_ERROR. Two superseded GMC sweeper runs ended
  `INTERNAL_ERROR / structural_invariant_failed`; their invariant names were not recorded then.
- **Slurm QOS.** cpu_short allows 32 CPUs and 120 GB per user at once, and 6 h wall. The 16 GB
  pilot tasks therefore ran about 7 at a time. Memory is now sized per combination from sacct
  MaxRSS × 1.5: A\* ≤ 2.2 GB, GMC sweeper ≤ 3.5 GB (tasks 11–49 resubmitted at 6 GB as 18501874).
- **Correction to the north-wing explanation.** The plane-floor pair list (job 18500683, same hall
  without the booth) populates the wing up to y ≈ 31. Its lattices connect most north free nodes to
  the main hall: sweeper 393 of 437, cylinder 253 of 296. In scene_v2 the corresponding counts are
  109 of 429 and 6 of 273. The uav-lamp booth (back panel floor to 2.48 m at y ≈ 29–30, lamp box from
  1.07 m, in the corridor between walls A and B) cuts the wing off. Design §3.3 is corrected; the
  y ≥ 30 near-floor occupancy is common to both scenes.
- **Pilot, GMC INTERNAL_ERROR `I3_graph_nesting`: 6 of 100 GMC queries** (5 sweeper, 1 cylinder),
  every one on a region cut by the contact strip.
  - On the GMC sweeper, 5 of 7 cut regions failed against 0 of 43 plain boxes. All polygons had the
    same (clockwise) orientation, so orientation is not the cause.
  - `verification/invariants.py:92` checks SAFE ⊆ POSSIBLE by exact shapely difference. With a slanted
    workspace edge, the two separately computed overlays leave floating-point slivers.
  - Fix (runner): the query region is now an axis-aligned staircase of 0.10 m y-bands inside the
    strip (`StairRegion`), used identically by A\*, GMC and the replay. Uncut boxes are unchanged.
    Tests added: rectilinear, inside the strip, contains == polygon, erosion == square fit. RED, then
    GREEN.
  - The staircase gives up ≤ 1.2 cm slivers at the strip edge. For 5 of the 5,000 pairs a cylinder
    witness touches such a sliver (v2-01136, v2-01413, v2-02749, v2-02860, v2-03906); they are
    listed in the handoff.
  - The 7 cut pairs of the pilot (14, 16, 22, 28, 33, 34, 36) were archived to
    `superseded_pilot_stair/` and rerun for all four combinations (18511062–65).
- **Full suite (18501925):** 722 passed, 25 failed. The 25 are exactly the baseline's missing sealed
  Atlas package (`test_atlas_benchmark` 10, `test_atlas_gate_intervals` 15). All 48 ground5k tests
  passed; they are 55 now.
- **GMC cylinder pilot memory:** sacct MaxRSS median 4.7 GB, p90 6.6 GB, max 7.4 GB. Tasks get 12 GB
  and the child cap is task memory − 1.5 GB (10.5 GB). GMC sweeper MaxRSS max 5.2 GB → 8 GB. A\* → 6 GB.
- **Final pilot** (after the three runner fixes):
  - A\* 100 of 100 certified and replayed.
  - GMC sweeper 44 of 50: 4 FAIL_BUDGET, 2 `safe_graph_ambiguous`.
  - GMC cylinder 6 of 50: 44 FAIL_BUDGET; certified maps up to 96,537 supports.
  - 0 FAIL_REPLAY and 0 ERROR.

  Projection: 0.63 CPU·h per pair for all four combinations, so 3,154 CPU·h for all 5,000 scene_v2
  pairs, which the budget allows. Wall-clock is about 13.6 days, bound by the 120 GB per-user QOS.
  Design §2, commit 2079e94.

### Task 3: launch (2026-09-25 13:29 UTC)

- **scene_v2, pairs [0, 5000):**

  | array | combination | tasks | pairs/task | memory |
  |---|---|---|---|---|
  | 18512654 | A\* sweeper | 0-19%1 | 250 | 6 GB |
  | 18512655 | A\* cylinder | 0-39%1 | 125 | 6 GB |
  | 18512656 | GMC sweeper | 0-199%2 | 25 | 8 GB |
  | 18512657 | GMC cylinder | 0-555%8 | 9 | 12 GB |

  All have `--time 05:55:00`. The pilot pairs 0–49 are skipped (same final semantics), which the
  first tasks' logs confirm.
- **Plane floor, pairs [0, 1000):** 18512671–18512674, same layout, `--dependency=afterany` on the
  four scene_v2 arrays.
- **Handoff:** `gmc/results/ground5k/g1_handoff.json`.
