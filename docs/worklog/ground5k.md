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
