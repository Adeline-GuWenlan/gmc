# Amendment 3 — plane floor, the shared case, long range, timings

Worklog for the P1–P5 chain on branch `height-planefloor`. Append as things happen.

## P1 — the plane floor and the timing module

**Standing caveat, repeated in every artefact this stage produces:** the plane floor is a
**user-approved manual scene-definition change** ("地面的高斯用大平面替代，手动解决噪音 for now").
It is surgery on the scene, not a reconstruction improvement and not an outer approximation with a
guarantee. Results on the edited scene are sound *with respect to the edited scene*; the gap between
that and the room is the open research question (`height_map_diagnosis.md` §4).

### 2026-09-17 19:1x — start

- Read `_rules.md`, Amendment 3, the main plan's Global Constraints + Stage bookkeeping,
  `height_map_diagnosis.md`, `README.md`. No `state/P1.json` existed, so this is a first run, not a
  resubmission.
- Inventory of what P1 builds on:
  - `showcase_scene._occupancy` / `BANDS` / `CELL = 0.05` — the band rasteriser the diagnosis used
    (through the cached `maps.npz`, which `--step maps` writes from the same function).
  - `gmc/results/height/diagnosis/map_diagnosis.json` — the numbers to reproduce.
  - `/scratch/wg2381/splathjb/gmc/outputs/height/a4/legs_check.py` and `table_frames.py` — the
    (uncommitted) A4 scripts behind `floor_rule_legs_check.json`; P1c reuses their method.
  - `showcase_scene._support_raster` + `project_scene` — the certified-map raster for P1c/P1d.

### 2026-09-17 ~20:0x — P1a: what the diagnosis number actually measures

- The session was resumed without its history (`state/P1.json` existed with `passed: []`, notes
  saying the reading was done). Judged the predecessor by artefacts, per `_rules.md`: it had left
  `planefloor.py`, `test_height_planefloor.py`, `planefloor_build.py` and `planefloor_step.sbatch`
  **uncommitted and never run**. Kept the code, split it into the per-task commits P1 requires.
- Found while reading `_occupancy`: the bands are **horizontal slabs above the scalar `z_floor`**,
  but the floor of this scene is a *tilted plane* (0.285° residual after the gravity rotation).
  tan(0.285°) × 25 m ≈ ±6.4 cm, i.e. most of the [0.02, 0.10] band. So wherever the fitted plane
  rises above `z_floor + 0.02`, the sweeper band contains **the floor surface itself**. That is a
  mechanism for the 40.7 %, it is not "reconstruction noise", and it predicts the plane-floor edit
  will work: the inserted plane is clamped flat below `z_floor + 0.02` everywhere by construction.
  `step_diagnose` now reports phantom rate binned by fitted-plane height so the mechanism is
  measured, not asserted.

### 2026-09-17 ~20:2x — P1b: the literal rule cannot work, and the figure says why

- P1a reproduced exactly on the first try (job 17925842, 38 s): 94,780 cells / 236.95 m² / 367
  components, and all five recomputed bands agree with the cached `maps.npz` **cell for cell**
  (`recomputed_vs_cached_maps_npz_differing_cells` all 0). No stop condition.
- Then opened `diagnosis/fig1_phantom_floor.png` before writing any more metrics (README rule 2),
  and it changed the design. The phantom layer is **salt-and-pepper dust spread over the open
  floor**, not a rim or a slab: 367 components, median blob 0.0425 m², 30 % of blobs ≤ 100 cm².
- That breaks P1b's literal rule. "Replace a splat iff its ρ-footprint lies only in phantom cells"
  needs every cell under a splat to be *occupied in the low band and free above*. In a dust field
  almost every 2–3 cell footprint straddles a cell that is merely free, so the rule keeps the dust
  it was written to delete. Arithmetic says how little slack there is: phantom is 58.65 % of the
  low band, so a perfect clear gives 40.70 % → 16.83 %, against 17.37 % in the band above —
  **97.7 % of the phantom cells have to go** for the map to become monotone.
- Fix, and it is a deviation from the literal wording that I am flagging rather than burying:
  use the **other half of the same conjunction** as the footprint test. `phantom = low &
  open_floor`, so `open_floor` (free in every band 0.10–2.50 m) is a superset, it is solid exactly
  where the dust is, and it keeps the guarantee that does the protecting — a splat goes only if
  **nothing at all** stands over its footprint between 0.10 and 2.50 m. Everything with mass above
  0.10 m (table legs, bench supports, plinths, the counter) is therefore kept *by construction*,
  which is a stronger statement than P1c's empirical check, and there is a unit test for it
  (`test_replaced_splats_never_have_mass_above_the_overhead_test_height`).
- `step_build` runs **both** masks and reports both columns; the choice is the least aggressive one
  that makes the low band stop being the most occupied. Nothing is tuned by hand.
- Consequence of the test's 0.10 m base worth recording: `max_top > 0.10` is **inert**. A splat
  reaching above 0.10 m occupies the band above in its own cells, so those cells are not open
  floor and the splat can never be a candidate, whatever `max_top` says. The ~2,899 splats that
  straddle 0.10 m on open floor (`floor_rule_survivors.json`) are outside what this rule family
  can touch, and they are the expected residual.
- `DATA` (`splatc_atlas/.../showcase/`) is read-only for this chain per `_rules.md`, so the edited
  scene is written to `gmc/outputs/height/plane/processed_planefloor.npz` with its floor metadata
  inside the archive, and P2/P3 load it with `planefloor.load_plane_scene` (one call, no need to
  reload the unedited scene to find the floor).

### 2026-09-17 ~20:5x — P1b, what the sweep actually found (three wrong guesses, then the data)

Two of my own hypotheses were wrong and the sweep killed both. Recording them because the third
one is only trustworthy *because* the first two were measured rather than believed.

1. **"`open_floor` will remove far more than `phantom`."** It removes twice as many splats
   (180,462 vs 88,750) and produces a **bit-identical map** (low band 0.2423 both, phantom cleared
   69.0 % both). The two rules are *exactly equivalent* on the splats that matter, and the proof
   is one line: any splat that reaches into the sweeper band marks its whole footprint occupied in
   the low band, so `footprint ⊆ open_floor` already implies `footprint ⊆ (low & open_floor) =
   phantom`. The extra 91,712 splats `open_floor` deletes all sit *below* `z_floor + 0.02` and
   were invisible to the band either way. Same argument shows **`max_top` is never binding**:
   every splat covering a phantom cell has a ρ-top under `z_floor + 0.10`, or the cell would not
   have been free in the band above.
2. **"the shadow footprint is the principled fix."** It is the footprint the *certified* projector
   builds, so I expected it to win. It came **last** (65.8 % cleared, worse than the literal
   rule's 69.0 %) while deleting 50 % more splats. Reason: the occupancy map is AABB-based, so
   clearing a cell needs *every* splat whose AABB covers it gone, and the band-clipped shadow is
   both smaller *and* displaced off the AABB, so it removes a different set rather than a
   superset. Its residual is also the worst placed — only 78 % within 0.30 m of real geometry,
   99th percentile 1.35 m, i.e. it leaves dust out in the open.
3. **What binds is the footprint, and `centre` wins: 88.0 % of the phantom layer cleared,
   40.70 % → 19.55 %.** A splat is dust iff nothing at all stands above *the cell it sits in* and
   it is confined below 0.10 m. That still protects everything with mass above 0.10 m over its own
   centre — every leg, support, plinth and the counter — which is what P1c has to confirm.

**The honest limit, measured.** 19.55 % is *near* the 17.37 % band above but not below it, so the
strict monotonicity test fails. The gap is attributed, not hand-waved: the 28.5 m² that survives
(from 237 m²) is **99.5 % within 0.30 m of geometry that has mass above 0.10 m**, median distance
0.05 m — one cell — 90th percentile 0.15 m, median blob 0.015 m². It is a thin rim hugging walls
and furniture feet: the AABB rasteriser over-approximating *real* near-floor objects, not leftover
phantom. Clearing it means deleting splats at the base of walls and tables, which is precisely the
P1c failure the stage is forbidden to buy a clean map with. So 19.55 % is the floor of what any
rule in this family can reach without eating real geometry.

- Supporting check without the band-thickness confound (the required column compares 0.08 m
  against 0.45 m, which catches every tabletop): at **equal 0.08 m thickness**, 0.02–0.10 goes
  40.70 % → 19.55 % against 15.47 % in 0.10–0.18. The inversion drops 2.63× → 1.26×.
- The trap in P1b, measured on the built scene with `aabb(2.0)` and again after a round trip
  through the npz: inserted ρ-top max **0.0150 m** above the floor, `z_lo` 0.02, headroom
  0.0050 m. Invisible to every robot band.
- Cost of clamping the plane flat, and it is a real visual cost: the fitted floor spans −0.071 to
  +0.075 m about `z_floor`, so **276 of 1014 tiles** are clamped off the fitted plane, the worst
  by **6.9 cm**. In the high quarter of the hall the rendered ground now sits up to 6.9 cm below
  where the reconstruction put the floor. Rendering artefact only — it cannot reach a robot band.

### 2026-09-17 ~21:0x — P1e, the timing module

- `gmc/src/gmc/height/timing.py`: `StageTimer` with the nine stages P1e names, closed list (an
  unknown name raises — a typo like `compile_paris` would otherwise become a silent tenth stage
  that P4 aggregates on its own and nobody notices). Every record carries its **input sizes**,
  because a stage measured without them is one P4 can say nothing about; `stage()` yields the
  record so a size only known after the work can be added to it (`project`'s support count).
  Repeats are labelled (`project` runs once per robot) and still sum in `by_stage`.
- Threaded through `height/run.py` (`compile_pairs` / `compile_slabs` / `compile_mobility` /
  `query` / `verify_curve`) and `experiments/showcase_run.py` (`scene_load`, `project`,
  `replay3d`). **No read-only module touched** — the sub-stages wrap calls `run.py` already made.
- `compile_seconds` and `query_seconds` are measured over exactly the same intervals as before and
  a caller that passes no timer gets a byte-identical result dict; there is a test for each.
- `showcase_run.py` also gains `--scene planefloor`, which is the P1→P2 hand-off: it loads the
  edited scene through `planefloor.load_plane_scene` and stamps the claims boundary into the
  result JSON, so a downstream run cannot use the edited scene without saying so in its own
  artefact. Default stays `processed`, so Amendment 2's runs are unaffected.

### 2026-09-17 ~21:2x — P1c: the gate fired, and the gate was wrong

First run failed 5 of 11 objects (table C, plinths S/E/NW, the reception counter). Before touching
the rule I checked what the gate was measuring, and it was measuring the wrong thing.

- `no_mass_lost_above_0.10m` was **True for all 11 objects** — not one opaque splat above 0.10 m
  was lost anywhere. That is the construction guarantee holding exactly.
- What failed was my "the member's disc must not lose occupied fraction" condition. A member disc
  is a *dilated blob of 4 cm cells plus a margin*, so its radius is 0.36–0.68 m for the plinths and
  **1.74 m** for the counter's main run: the disc is mostly floor, and the dust removed inside it
  is the edit working, not damage.
- The tell, and the reason I trust this reading rather than my own explanation of it: discs tight
  enough to actually *be* the member come out **bit-identical**. plinth_E's 0.239 m member,
  0.7003 → 0.7003; the counter's 0.23 m member, 1.0 → 1.0; its 0.22 m member, 0.8058 → 0.8058.
  Only the wide, floor-containing discs moved.
- Rewrote the gate to measure the member instead of its neighbourhood: (a) no mass lost above
  0.10 m, (b) the three-bin column test finds the **same member cells** on the edited scene,
  (c) opaque splats at 0.02–0.10 m whose centre lies **in a member cell** must not decrease — the
  member's own near-floor mass, (d) every member keeps ≥ 1 sweeper support. Disc occupied fraction
  is still reported, as information, with the reason it is not a gate.
- Opened `p1c_table_C.png` before writing any of this up, and it is the best artefact of the
  stage: **before**, the sweeper map is a solid dust field and the table is invisible in it;
  **after**, the dust is gone and table C's outline stands out — both ends inside the cyan member
  circles, long edges on the dashed footprint. The edit did not eat the table, it revealed it.
  A sweeper could not have told that table from the floor noise before.

### 2026-09-17 ~21:4x — P1c passes 11/11, and P1d is the number that matters

- Second gate run failed only the **reception counter**, and again on my condition rather than on
  the scene: the culprit was a **1-cell "member"** at (−1.94, 0.27) with `0 → 0` sweeper supports.
  It never blocked the sweeper *before* the edit, so it cannot have lost anything; my condition was
  an absolute `≥ 1` instead of "if it blocked before, it still blocks". Every mass band of the
  counter was identical (276/218/2250/8532 before and after) and its member cells were unchanged.
  Fixed the condition, and **11/11 objects now pass all four**: no mass lost above 0.10 m, member
  cells identical, member near-floor mass kept, every previously-blocking member still blocking.
- Hall-wide collateral, and this is the reassuring one: of 146,977 removed splats (46,507 opaque),
  the opaque ones have centre heights **p50 = −0.081 m** (8 cm *below* `z_floor`), p90 = −0.003 m,
  p99 = +0.031 m, and ρ-tops p50 = **−0.029 m**, p90 = +0.041 m, max **+0.0999 m** (just inside the
  0.10 m cap, as the rule requires). So the median thing deleted is the floor surface itself and
  what sits under it; under 10 % of removed opaque splats even reach above floor level. The edit
  deleted the floor and the dust lying on it, not anything that stands up.
- **P1d, window A, certified maps** (`project_scene` shadow ellipses → `_support_raster` 0.025 m):

  | | sweeper before | after | cylinder before | after |
  |---|---|---|---|---|
  | supports | 3764 | 3028 | 111,941 | 111,162 |
  | occupied | 17.9 % | **5.8 %** | 26.6 % | **14.5 %** |
  | disc fits | 17.0 % | **73.3 %** | 3.1 % | **57.7 %** |
  | free components | 77 | **3** | 11 | **2** |
  | largest free blob | 2.44 m² | **33.03 m²** | 0.49 m² | **26.22 m²** |
  | criterion-3 clear (≥ 0.5 m) | 0.1 % | **41.5 %** | 0.1 % | **40.4 %** |

  The shattering the diagnosis blamed for A4's failure is gone: the sweeper's free space goes from
  77 fragments whose largest is 2.4 m² to 3 whose largest is 33 m².
- **The edit added nothing to either certified map.** The difference panels show 8,787 (sweeper) and
  8,786 (cylinder) cells cleared and **0 newly occupied** — the inserted plane is invisible to both,
  as measured.
- Certified equal-thickness bands on window A: 0.02–0.10 goes **16.68 % → 4.43 %** while 0.10–0.18
  (2.98 %) and 0.18–0.26 (3.00 %) are *bit-identical* before and after. Ratio 5.60× → 1.49×.
- **A finding P2 must act on.** D1's rationale in `height_map_diagnosis.md` §3 (crit1 ∧ crit3 =
  0.9 m² as built, 1.2 m² with every phantom cell deleted) was computed on the **AABB selection
  aid**, hall-wide. On the *certified* raster, criterion-3-clear area in window A goes from 0.1 % to
  41.5 % of the window. Those are not directly comparable — mine is one window and one band at a
  time — but the premise "0.5 m is not a data problem and cannot be fixed by P1" was derived on the
  looser rasteriser and **should be re-derived on the certified map before P2 assumes criterion 3 is
  unreachable.** D1 remains user-approved and available either way.

### 2026-09-17 ~22:0x — P1f: run the hand-off rather than describe it

I had written the P2 hand-off into the standup without ever executing that code path, which is the
"verified list is not a completeness proof" trap in the README — the loader had unit tests on
synthetic scenes but had never opened the real 0.8 GB archive. Added `--step handoff` and ran it
(job 17931446):

- `load_plane_scene` opens the built scene; **all eight meta keys `showcase_run.py --scene
  planefloor` reads are present** (`meta_keys_missing: []`).
- `project_scene` runs for all three robots on the loaded scene: sweeper 3,028 supports (identical
  to P1d's window-A number, so the two paths agree), cylinder 111,162, uav 11,012.
- **`inserted_plane_splats_in_map: 0` for all three robots**, including the uav. This is the
  strongest direct form of the P1b trap check: not "the tops are below `z_lo`" but "no inserted
  splat is in any robot's projected map at all".
- The timing block records `scene_load` and `project` and lists the other seven stages as missing,
  which is correct for a load-and-project run and is what P4 will see.

### 2026-09-17 ~22:1x — the one region that still looks busy, and what it means for P2

Opened the final `p1b_low_band_before_after.png`. The chosen panel reads as a clean architectural
plan — walls as crisp outlines, furniture as distinct blocks, floor essentially empty — with the
red confined to thin rims, which is what the 99.5 %-within-0.30 m attribution predicts.

One region is a visible exception and a reader will ask about it: the **north corridor, x 8–15,
y 32–37**, stays dense. That is the round-tables cluster at (12.0, 34.0) from the object
catalogue. It is consistent with the measurement rather than an exception to it — the region
breakdown puts **18 m² of the 28.5 m² residual in the north** (`residual_m2_by_region`, split at
y = 20), and in a dense cluster of small round tables nearly every floor cell really is within
0.30 m of furniture, so the rim covers the floor. **Advice for P2/P3: do not site a window in
y > 32.** The south half of the hall (y < 20) is where the map is cleanest, and window A sits in it.

## P2 — the shared three-robot case

**Standing caveat, unchanged from P1 and repeated on every artefact:** the plane floor is a
**user-approved manual scene-definition change**. Results are sound with respect to the edited scene;
the gap between that and the room is the open research question (`height_map_diagnosis.md` §4).
**Criterion 3 is relaxed by user decision D1** to `r + 0.05` m on each robot's own certified map, plus
start and goal in one connected component of `dist > r`. That is a user-approved criterion change, and
the reason it was needed is not a data defect P1 could have fixed: criteria 1 and 3 intersect in
**0.9 m²** of the 990 m² hall as built and **1.2 m²** with every phantom cell deleted
(`height_map_diagnosis.md` §3). Criterion 1 is **unchanged**.

### 2026-09-18 ~00:1x — the machinery, and a synthetic scene to test it on before spending a job

- No `state/P2.json` existed, so this is a first run. Read `_rules.md`, Amendment 3, the main plan's
  Global Constraints and Stage bookkeeping, `height_map_diagnosis.md`, `README.md`, the P1 standup,
  `state/P1.json` and the P1 worklog above.
- Put the search machinery in `gmc/src/gmc/height/casesearch.py` rather than copying
  `percase_search_*.py` a fourth time — P3 needs to reuse it, and the pure functions (EDT clearance,
  free components, D1, the straight line's clearance profile, geodesics, route separation) are
  testable on hand-built rasters without loading 7.3 M splats. 23 unit tests.
- **Criterion 1 is extracted, not paraphrased.** `overhang_on_line` is `showcase_scene.step_case`'s
  criterion-1 code bin for bin, and a test replays step_case's own inline version on a random section
  and compares run counts and interval. "Unchanged" is a test here, not a claim.
- Before spending a Slurm job on the hall I ran the whole pipeline on `synth3d.table_scene("open")`,
  which has exactly the morphology P2 is hunting: a wall at x = 0 with two gaps, the south one filled
  by a table on four thin legs, the north one empty. From one start to one goal it produced
  **sweeper 4.43 m straight under the table, uav 4.43 m straight over it, cylinder 5.65 m round the
  north gap**, separation 1.48 m, 1.14 m of the straight line unusable by the cylinder. Opened the
  figure: the sweeper's map has four dots where the legs are and the tabletop is simply absent; the
  cylinder's has a solid block. That proves nothing about the hall — it proves the search can
  recognise the picture when the picture is there, which is what I wanted to know **before** reading a
  null result on real data. It is now an integration test (`test_height_plane_case.py`).

### 2026-09-18 ~00:2x — region A, and the A4 null result is gone

First region of the real search (job 17932736), table A `[4.5, 2.5, 13.5, 11.5]`, all numbers on the
**certified** map (`project_scene` → `_support_raster` 0.025 m → EDT), never on the AABB aid:

- 608 lattice points pass D1's clearance clause for **all three robots at once**;
- **7,356 pairs** are connected for all three, free on the straight line for the sweeper and blocked
  on it for the cylinder;
- of the 120 best by contrast, **120 pass criterion 1**.

A4's scene-wide search on the unedited scene got `pairs_dist_ok 1296 → crit1 490 → crit2 171 → all
three 0`. The count that was zero is no longer zero, in one region.

Best pair so far: start (9.00, 5.75), goal (10.50, 7.75), 2.5 m apart. The sweeper can drive the
straight line with 0.21 m of clearance to spare; **1.51 m of that same 2.5 m line is unusable by the
cylinder**, whose own route is 1.33× longer and runs 0.94 m away from it.

**Support density, which is the compute risk and what P3 needs to size a 12 × 12 m window**: region A
holds **3,356 cylinder supports per m²** against 75 for the sweeper and 628 for the uav. At that
density a 12 × 12 m window would be ~480,000 cylinder supports, so P3 cannot assume window area is
free. Amendment 2's cylinder case ran 3,791 supports (421/m², a clear part of the SW hall) in 409 s of
query, so the density varies by ~8× between regions and the window has to be chosen for it.

### 2026-09-18 ~00:3x — P2a/P2b: a shared case exists

Job 17932736, 6 m 16 s over five regions. **`shared_case_exists = true`**, and all six finalists pass —
this is not one lucky pair.

| region | area | sweeper sup/m² | cylinder sup/m² | uav sup/m² | endpoints passing D1 for all three | contrasting pairs | of top 120, passing crit 1 |
|---|---|---|---|---|---|---|---|
| A | 81.0 | 75 | 3356 | 628 | 608 | 7356 | 120 |
| SW | 94.5 | 50 | 3784 | 763 | 675 | 10516 | 120 |
| E | 90.0 | 111 | 4880 | 1005 | 615 | 9337 | 4 |
| G | 64.0 | 43 | 2524 | 583 | 361 | 914 | 0 |
| N | (see JSON) | | | | | | |

"Contrasting pair" means all three robots connected **and** the straight line usable by the sweeper
**and** blocked for the cylinder, all on the certified map. A4's scene-wide search on the unedited scene
ended `pairs_dist_ok 1296 → crit1 490 → crit2 171 → all three 0`. The zero is gone.

**The chosen case, `results/height/plane/case_shared.json` (label P2-SW-0):**

window `[-2.5, 7.45, 1.8, 13.05]` (4.30 × 5.60 m), start **(−1.25, 11.25)**, goal **(0.00, 9.25)**,
2.36 m apart, `z_c` 1.20 — **identical for all three robots**.

| | sweeper | cylinder | uav |
|---|---|---|---|
| band (m above floor) | 0.02–0.10 | 0.02–1.75 | 1.10–1.30 |
| supports in the window | 1,779 | **156,426** | 19,867 |
| start / goal clearance | 0.451 / 0.437 m | 0.451 / 0.437 m | 1.250 / 0.459 m |
| D1 needs | 0.225 | 0.350 | 0.300 |
| straight line unusable | **0.00 m** | **2.02 m of 2.36** | 0.32 m |
| free-space route | **2.52 m** | **5.77 m** | 2.52 m |

Detour ratio **2.29×**, route separation **1.98 m**.

**Opened `p2a_finalist0_SW.png` before writing any of this.** In the sweeper's map table B is not there
at all — just five or six isolated dots, its legs and the bench feet, each with a white halo, and the
route threads between two of them almost straight. In the cylinder's map the same patch of floor is one
solid red block lying across the straight line, and the route swings north and east around it. In the
uav's map the block is gone again and it flies straight. Same window, same start, same goal, same scene.
That is the picture the user described on 2026-09-12: **the sweeper goes under the table, the cylinder
has to go around it.**

**The compute risk is the cylinder, and it is real.** 156,426 supports against the 3,791 of Amendment 2's
cylinder run (which took 17 s to compile and 409 s to query). Every passing window is in the same
density band (153–159 k), because the contrast needs a table in the window and the cylinder's band
[0.02, 1.75] takes the whole of it plus the wall behind it. Submitted all three to `showcase_run.py`
with the probe **on** (jobs 17934013/14/15) rather than guessing: if the 20-projected-hour probe aborts,
the answer is to shrink the window, never the semantics.
