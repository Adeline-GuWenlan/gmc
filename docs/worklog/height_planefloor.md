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

### 2026-09-18 ~00:4x — P2c: two of three certified, the cylinder ran out of query budget

All three ran on the **identical** window, start and goal (`results/height/plane/shared/case.json`),
plane-floor scene, config and gates untouched.

| | supports | status | compile | query | verify / replay3d |
|---|---|---|---|---|---|
| sweeper (17934071) | 1,779 | **REACHABLE** | 7.0 s | 11.3 s | certified, replay3d passed, lb 0.0059 m |
| uav (17934072) | 19,867 | **REACHABLE** | 108 s | 172 s | certified (verify 85 s), replay3d passed |
| cylinder (17934263) | 156,426 | **UNKNOWN** `query_wall_budget_exhausted` | 676 s | 7,968 s | not run |

The cylinder hit the config's 7,200 s query cap. Amendment 3 allows exactly one budget-only retry on
UNKNOWN (`BUDGET=2`, which doubles the query wall to 14,400 s and nothing else); submitted as
**17938411**. If that is UNKNOWN too, the case is reported as it stands — two certified routes and a
cylinder that the planner could neither route nor refute within budget — and the window is **not**
shrunk after the fact to rescue it: the window was chosen, pre-checked and run; changing it now would
be changing a case to fit a result.

**The probe under-estimated by at least 13×, and P3 needs to know.** `showcase_run.py`'s probe put
45,288 supports in its 2 × 2 m window (29 % of the full 156,426), compiled them in 192 s, and projected
**0.18 h** by multiplying by the support ratio (3.5×). The real run took **2.4 h** and still did not
finish. At this density the probe window already holds a large share of the supports, so the ratio it
extrapolates over is small, and the query is not linear in supports. The 20 h abort is frozen and I
left it alone; the lesson is only that a sub-hour projection at ~6,500 cylinder supports/m² is not an
assurance. For sizing: sweeper 74, uav 825, cylinder 6,496 supports/m² in this window.

**My own bug, recorded because it cost a job:** the first evidence job (17934271) died in 18 s with
`NameError: step_selftest`. When I rewrote `step_evidence` into the window-shrink series I replaced the
text from `def step_evidence` to `def step_compare`, and `step_selftest` sat between them. Restored it
from the previous commit, re-ran the self-test (passes) and the 34 P2 tests (pass), resubmitted as
17938423. The lesson is the obvious one: after a slice-replace, `grep '^def '` before submitting.

The sweeper video (17938424) is queued now that its run is certified; the uav video waits for a queue
slot; the cylinder video, and the three-routes figure (`--step compare`), wait for 17938411.

### 2026-09-18 ~13:2x — maintenance restart; the three queued jobs resubmitted as they were

Cluster maintenance cancelled all three of P2's queued jobs at 07:24 before any of them started:
17938411 (cylinder `BUDGET=2`), 17938423 (evidence), 17938424 (sweeper video). The user restarted the
chain on 09-18 and recorded the resource decisions in the plan's "Update 2026-09-18" section, D5.
Per that section this is **not a failure and not a used retry**, so all three were resubmitted
unchanged. Only their sizing changed, and every size comes from `sacct` on the 09-17 runs:

| job | was | now | measured basis |
|---|---|---|---|
| cylinder `BUDGET=2` → **17944446** | 8 CPU / 64 G / 10 h | **1 CPU / 22 G / 8 h** | first cylinder run: MaxRSS 14.2 GB, TotalCPU 2 h 27 m over 2 h 28 m, i.e. one core |
| evidence → **17944447** | 2 CPU / 16 G | **2 CPU / 8 G** (template) | the whole five-region search peaked at 3.3 GB |
| sweeper video → **17944448** | 4 CPU / 32 G | **4 CPU / 12 G / 2 h** | Amendment 2 renders peaked at 2.3–3.3 GB; the EWA splat pack at scene radius 14 m is unmeasured, hence the margin. The uav render will be sized from this job's MaxRSS. |

Also new from the rules: this agent's own allocation is 1 CPU / 2 GB and only orchestrates, so the full
test suite runs as its own sbatch job instead of in-allocation. P3 now commits in this worktree
concurrently, so every P2 commit stages named paths only.

## P3 — longer range, option (a)

**Standing caveats, repeated on every P3 artefact:** the plane floor is a **user-approved manual
scene-definition change** (results are sound w.r.t. the edited scene only). **Criterion 3 is relaxed by
user decision D1** to `r + 0.05` m on each robot's own certified map, plus start and goal in one component
of `dist > r` (crit1 ∧ crit3 = **0.9 m²** as built, **1.2 m²** with every phantom cell deleted,
`height_map_diagnosis.md` §3). **Criterion 1 is recorded, not required**, for the long case. Scope is
**D4, option (a)**: sweeper and uav at ‖goal − start‖ ≥ 8 m in ≤ 12 × 12 m; the cylinder on the largest
window it actually certifies. Option (b) (all three at ≥ 8 m via the Amendment 1 grid coreset) is the
next step and is **not** run here.

### 2026-09-18 ~15:1x — sizing before submitting anything

- First session (13:15, job 17944389) read the rules, both plans, the diagnosis, P1/P2 standups and
  state, then died on the usage limit before submitting anything. Resumed as 17944654.
- **What is already measured**, from the result JSONs, not from the standups:

| run | robot | window (m) | supports | sup/m² | compile s | query s | status |
|---|---|---|---|---|---|---|---|
| A2 (unedited scene) | sweeper | 2.00 × 3.05 | 477 | 78 | 2.0 | 3.4 | REACHABLE, lb 0.027 |
| A2 (unedited scene) | cylinder | 3.00 × 3.00 | 3,791 | 421 | 17.3 | 409 | REACHABLE, lb 0.0032 |
| A2 (unedited scene) | uav | 5.90 × 3.65 | 143 | 7 | 0.6 | 0.3 | REACHABLE, lb 0.05 |
| P2 shared | sweeper | 4.30 × 5.60 | 1,779 | 74 | 7.0 | 11.3 | REACHABLE, lb 0.0059 |
| P2 shared | uav | 4.30 × 5.60 | 19,867 | 825 | 108.5 | 171.9 | REACHABLE, lb 0.0127 |
| P2 shared | cylinder | 4.30 × 5.60 | 156,426 | 6,496 | 675.7 | 7,968 (cap 7,200) | UNKNOWN |

  Compile is close to linear in supports for every robot (3.9–5.4 ms per support). Query is not: per
  support it ranges 0.0064 s (sweeper) to 0.108 s (A2 cylinder), and the A2 cylinder's 409 s on only
  3,791 supports goes with the tightest clearance of all (lb 0.0032 m). So supports bound the compile,
  and they bound the query only loosely. A 12 × 12 window can't be priced from one number.
- **The density varies 15× within a few metres.** A2's cylinder window [0.5, 10.0, 3.5, 13.0] (421/m²)
  overlaps P2's shared window (6,496/m²). So region averages (2,500–4,900/m²) do not size a window;
  exact counts do.
- **Sizing tool, instead of the probe.** `gmc/height/longrange.py`: `shadow_table` does
  `project_scene`'s window-independent half once per robot (opacity, eigen floor, band, certified outer
  shadow, de-dup); `count_supports` applies its two window tests. It is pinned to `project_scene`'s
  `kept` count for 3 robots × 6 windows on the synthetic scene (unit test). On the hall it is checked
  again against P2's three known counts and two P2 regions before anything uses it.
- **Where the cylinder ladder sits, and why not literally on P2's window.** The prompt says to step the
  window up from P2's shared case. P2's window is the densest the cylinder has been measured on (6,496/m²),
  and it already runs past the query cap at its own size. Every bigger window around it holds more
  supports, so a ladder there cannot certify at `BUDGET=1`; it would only say so again. So the ladder
  **shares the long case's start** instead. Its separations **step up from P2's ~2.4 m**: 2.5, 4, 5.5,
  7, 8.5 … up to the long case's own distance. Every rung is sized by its exact count, one `pf_` job
  each, run smallest first. Its map is the long window grown by 3 m. That way, if the cylinder cannot
  use the sweeper/uav corridor (say a table it cannot pass under), the ladder is stopped by compute and
  not by the crop.
- `plane_long_search.py --step selftest` runs every code path on `synth3d.table_scene("open")` in 3.5 s.
  The first attempt failed on my own self-test parameters: a 0.3 m margin is below the cylinder's
  0.35 m D1 need. That failure also exposed the crop problem above, which is why the ladder map is now
  grown by 3 m. Now it passes: a long case, plus 4 rungs whose counts equal `count_supports`. I opened
  both figures before trusting them.
- Submitted **17948423** `pf_p3a_search` (2 CPU / 12 G / 3 h; P2's region search peaked at 3.3 GB, this
  one also projects the uav over the whole hall and a grown cylinder map).

### 2026-09-18 ~15:2x — the search (17948423, 5 m 49 s, MaxRSS 3.6 GB): a long shared case exists

- **Sizing tool validated on the hall:** `count_supports` equals `project_scene`'s `kept` **exactly** on
  all 9 checks (P2's window and P2 regions A and SW × 3 robots; e.g. cylinder 156,426 / 271,848 /
  357,631). Hall-wide, below and above y = 32: sweeper **68,454**, uav **449,602**, cylinder
  **2,305,616** supports.
- Screening funnel (0.1 m block-min graph, lattice 0.5 m): 1,940 endpoints pass D1 for sweeper and uav →
  351 k pairs ≥ 8 m in 12 × 12 reach → 181 k connected for both → 16,459 whose two routes fit a
  ≤ 12 × 12 window → 13,438 under the screening caps (uav ≤ 60 k, sweeper ≤ 20 k) → **7,547 with routes
  ≥ 0.5 m apart**. Of 6 finalists re-checked on their own projection, **5 pass**; they all sit around
  table A and the long display wall (region A), so they are one place, not five.
- **The long case, P3-long-0** (`results/height/plane/long/case.json`): window `[6.45, 4.7, 14.15,
  15.6]` (**7.70 × 10.90 m, 83.9 m²**), start **(7.35, 5.30)**, goal **(13.35, 14.80)**, **11.24 m**
  apart. Supports: sweeper **6,436**, uav **39,834**, cylinder 269,036 (not run, D4). Free-space routes:
  sweeper 14.67 m, uav 12.45 m, **5.73 m apart**. Criterion 1 not met (recorded, not required).
- **Opened `p3_long_finalist0.png` before writing this.** The uav's map has neither table A nor the low
  clutter north-east of the display case, so it cuts diagonally south-east of the display case. The
  sweeper's map is speckled with floor-level clutter there, so it goes round the north-west end of the
  long display wall instead. The cylinder's map agrees with the sweeper on the NE clutter and adds
  table A as a solid block.
- **The cylinder ladder** (`cyl_ladder.json`, `figs/p3_cyl_ladder.png`, opened): anchor = the long
  case's start (cylinder clearance 0.575 m). The long goal itself fails the cylinder's D1 (0.335 m <
  0.35 m), so the top rung ends at the nearest admissible point, 11.28 m out. All 7 rungs pass their own
  pre-check:

| rung | separation m | window m | supports | sup/m² |
|---|---|---|---|---|
| 0 | 2.43 | 2.00 × 3.62 | 21,144 | 2,917 |
| 1 | 4.00 | 3.61 × 5.12 | 45,572 | 2,462 |
| 2 | 5.45 | 4.36 × 5.87 | 46,835 | 1,828 |
| 3 | 6.90 | 6.33 × 5.87 | 73,178 | 1,970 |
| 4 | 8.58 | 5.88 × 9.22 | 149,303 | 2,755 |
| 5 | 9.83 | 7.80 × 9.62 | 235,620 | 3,139 |
| 6 | 11.28 | 7.38 × 11.00 | 259,588 | 3,200 |

  Rung 3's window takes in the dense curved structure at (13, 6.5), which is why it jumps to 73 k. Rung 4
  (149 k) is already the size of P2's 156 k map that ran past the query cap.
- Submitted **17948811** sweeper (1 CPU / 6 G / 2 h) and **17948812** uav (1 CPU / 9 G / 4 h) on the long
  case. Rungs go next, smallest first, at most two in flight; a rung that returns UNKNOWN stops the climb.

### 2026-09-18 ~15:3x — the core result: sweeper and uav certified at 11.24 m; ladder rung 0 certified

Same window, start and goal for both (`results/height/plane/long/case.json`), plane-floor scene, τ, ρ,
robots, config, `verify_curve`, `replay3d` untouched.

| run | supports | compile s | query s (cap 7,200) | verify s | status | verify min clr m | replay3d lb m | wall | MaxRSS |
|---|---|---|---|---|---|---|---|---|---|
| sweeper 17948811 | 6,436 | 25.0 | 107.5 | 52.3 | **REACHABLE, certified, replay3d passed** | 0.00278 | **0.00015** | 3 m 37 s | 2.76 GB |
| uav 17948812 | 39,834 | 153.4 | 292.9 | 145.5 | **REACHABLE, certified, replay3d passed** | 0.00704 | 0.01769 | 10 m 44 s | 2.22 GB |
| cylinder rung 0 17948988 (2.43 m) | 21,144 | 81.0 | 174.8 | 85.2 | **REACHABLE, certified, replay3d passed** | 0.0325 | 0.00144 | 6 m 46 s | 2.24 GB |

- **The clearance lower bound, honestly:** positive in all three, so all three are certified. But the
  sweeper's is **0.15 mm** at 11.24 m, against 0.027 m (A2, 2.0 m) and 0.0059 m (P2, 2.36 m). GMC finds a
  certified route, not a max-clearance one. A longer route is constrained by more obstacles, so the
  minimum over it can only get smaller or stay the same. The uav's did not shrink (0.0127 → 0.0177 m).
  The certificate holds; the margin is thin, and I report that rather than a wider figure.
- **MaxRSS is far below my sizing** (2.2–2.8 GB, requested 6–9 GB). The ~73 KB/support I took from P2's
  156 k cylinder does not hold at 20–40 k supports; that run's 14.2 GB must come from its query, not its
  compile. Later rungs are sized from these measurements.
- Rung 1 (**17949332**, 45,572 supports) and rung 2 (**17949336**, 46,835) are running.

### 2026-09-18 ~15:4x — rung 2 certified; rung 1, same size, still running

- **Rung 2** (17949336, 5.45 m, 46,835 supports, window 4.36 × 5.87 m): **REACHABLE, certified, replay3d
  passed**. Compile 185 s, query 445 s, verify 220 s; 16 m 28 s; MaxRSS 2.30 GB.
- **Rung 1** (17949332, 4.00 m, 45,572 supports) is still running at 17 min. It has fewer supports and
  a shorter separation than rung 2, and it is slower. Same lesson as A2's 3,791-support cylinder that
  queried for 409 s: supports bound the compile, not the query.
- Submitted **rung 3** (17949914, 6.90 m, 73,178 supports, 1 CPU / 10 G / 3 h).

### P3 — why only this is doable, in numbers (FINAL for the ladder; 2026-09-18 ~20:4x)

Generated by `plane_long_search.py --step report` → `results/height/plane/long/p3_report.json` and
`figs/p3_scaling.png`. I opened the figure: three log-log panels, one y-axis each. Hollow markers are
UNKNOWN runs, whose time is a lower bound. Every support count is `project_scene`'s `kept`.

**1. Supports vs window area, per robot.**

| source | robot | window m² | supports | sup/m² |
|---|---|---|---|---|
| A2 (run) | sweeper / cylinder / uav | 6.1 / 9.0 / 21.5 | 477 / 3,791 / 143 | 78 / 421 / 7 |
| P2 shared (run) | sweeper / cylinder / uav | 24.1 | 1,779 / **156,426** / 19,867 | 74 / **6,496** / 825 |
| P2 shrink series, smallest (projection) | cylinder | 6.2 | 64,294 | 10,300 |
| **P3 long (run; cylinder projection only)** | sweeper / uav / cylinder | 83.9 | **6,436 / 39,834** / 269,036 | 77 / 475 / 3,205 |
| P3 rungs 0 / 1 / 2 (run, certified) | cylinder | 7.2 / 18.5 / 25.6 | 21,144 / 45,572 / 46,835 | 2,917 / 2,462 / 1,828 |
| P3 rungs 3 / 4 (run, UNKNOWN) | cylinder | 37.2 / 54.2 | 73,178 / 149,303 | 1,970 / 2,755 |
| P3 rungs 5 / 6 (projection, not run) | cylinder | 75.1 / 81.1 | 235,620 / 259,588 | 3,139 / 3,200 |
| **every 12 × 12 window below y = 32** (1,204 of them, 0.5 m step; min / median / max) | sweeper | 144 | 3,745 / 9,444 / 22,737 | 26 / 66 / 158 |
| same | uav | 144 | 24,278 / 91,889 / 154,200 | 169 / 638 / 1,071 |
| same | **cylinder** | 144 | **118,856 / 477,726 / 752,899** | 825 / 3,318 / 5,228 |

The 12 × 12 row is summed from the 0.5 m density grid by shadow centre, so it under-counts `kept`
slightly: the r-dilated rim is left out. **The cheapest 12 × 12 window in the hall holds 76 % of P2's
156 k-support map and 1.6× rung 3's 73 k map, which is the smallest cylinder map that did not certify.**
The median window holds 3× P2's map. The prompt's "~900 k" is P2's 6,500/m² applied to the whole window.
Measured, the densest window holds 753 k.

**2. Compile and query per cylinder run, against the caps.** The config allows 7,200 s of query and
50,000,000 support calls; `BUDGET=2` doubles both.

| run | sep m | supports | compile s | query s | status | which cap | MaxRSS |
|---|---|---|---|---|---|---|---|
| A2 | 2.69 | 3,791 | 17.3 | 409.4 | certified | — | — |
| P3 rung 0 | 2.43 | 21,144 | 81.0 | 174.8 | certified | — | 2.24 GB |
| P3 rung 1 | 4.00 | 45,572 | 174.7 | 628.8 | certified | — | 3.22 GB |
| P3 rung 2 | 5.45 | 46,835 | 185.0 | 445.0 | **certified** | — | 2.30 GB |
| P3 rung 3 | 6.90 | 73,178 | 283.9 | ≥ 722.5 | UNKNOWN | 50 M support calls | 3.37 GB |
| P3 rung 3, `BUDGET=2` | 6.90 | 73,178 | 293.8 | ≥ 1,289.0 | UNKNOWN | 100 M support calls | 4.52 GB |
| P3 rung 4 | 8.58 | 149,303 | 581.0 | ≥ 1,713.9 | UNKNOWN | 50 M support calls | 4.34 GB |
| P2 shared | 2.36 | 156,426 | 675.7 | ≥ 7,968.1 | UNKNOWN | 7,200 s wall | 14.2 GB |
| P2 shared, `BUDGET=2` | 2.36 | 156,426 | 766.4 | ≥ 15,026.8 | UNKNOWN | 14,400 s wall | (P2's job) |

Sweeper and uav, same caps: the long runs used 107.5 s and 292.9 s of the 7,200 s query budget. None of
their runs came near either cap.

**3. The cylinder at 12 × 12.**
- **Compile, fitted on 9 runs** (3,791–156,426 supports; UNKNOWN runs included, since their compile
  finished): `t = 0.0043 · N^1.00`, i.e. linear at **4.3 ms per support**. At the 12 × 12 min / median /
  max that is **488 s / 1,953 s / 3,074 s**. The median and max lie 3.1× and 4.8× beyond the largest
  compiled map. Compile is not what stops the cylinder.
- **Query: no fit is predictive, so there is no extrapolation.** The 4 finished runs (3,791–46,835
  supports) give exponent 0.09, i.e. flat. That fit is contradicted by **all five** UNKNOWN runs: it
  under-predicts rung 3 by ≥ 1.7× (≥ 3.0× at `BUDGET=2`), rung 4 by ≥ 3.8×, and P2 by ≥ 17.6× (≥ 33× at
  `BUDGET=2`). What the runs do show:
  - Every cylinder map ≤ 47 k supports certified.
  - Every map ≥ 73 k ran out of a query budget: the support-call cap at 73 k and 149 k, the wall cap at
    156 k.
  - Doubling the budget did not rescue either run it was tried on.
  - Every 12 × 12 window holds ≥ 119 k.

  Supports alone don't set the query cost: rungs 1 and 2 are the same size and differ by 1.4× in query
  time. They mark only where the budget stops binding: somewhere between 47 k and 73 k on this route.
- **The probe** (`showcase_run.py`, linear in supports × intervals), actual over projected:
  - A2 cylinder **21×**
  - P2 cylinder **≥ 13×**, and **≥ 20×** on its retry
  - rungs 0–2: 1.7× / 4.6× / 3.4×
  - rungs 3, 3-b2, 4: ≥ 3.6× / ≥ 5.5× / ≥ 1.4×
  - long sweeper 3.6×, long uav 2.0×

  It never over-estimated a cylinder run.

**4. What (b) would change.** Option (b) runs all three robots at ≥ 8 m via the Amendment 1 grid
coreset. That shrinks the cylinder's map itself, which is the quantity everything above says is the
binding one, where option (a) could only shrink the window. **It was not run** (D4: the user asked for
(a) first). It is the next step.

**Short range vs long range.**

| case | robot | A–B m | window m | supports | compile s | query s | replay3d lb m | result |
|---|---|---|---|---|---|---|---|---|
| A2 | sweeper | 2.02 | 2.00 × 3.05 | 477 | 2.0 | 3.4 | 0.0268 | certified |
| A2 | cylinder | 2.69 | 3.00 × 3.00 | 3,791 | 17.3 | 409.4 | 0.0032 | certified |
| A2 | uav | 4.15 | 5.90 × 3.65 | 143 | 0.6 | 0.3 | 0.0500 | certified |
| P2 shared | sweeper | 2.36 | 4.30 × 5.60 | 1,779 | 7.0 | 11.3 | 0.0059 | certified |
| P2 shared | uav | 2.36 | 4.30 × 5.60 | 19,867 | 108.5 | 171.9 | 0.0127 | certified |
| P2 shared | cylinder | 2.36 | 4.30 × 5.60 | 156,426 | 675.7 | ≥ 15,027 | — | UNKNOWN (wall, ×2) |
| **P3 long** | **sweeper** | **11.24** | 7.70 × 10.90 | 6,436 | 25.0 | 107.5 | **0.00015** | **certified** |
| **P3 long** | **uav** | **11.24** | 7.70 × 10.90 | 39,834 | 153.4 | 292.9 | 0.0177 | **certified** |
| P3 rung 0 | cylinder | 2.43 | 2.00 × 3.62 | 21,144 | 81.0 | 174.8 | 0.0014 | certified |
| P3 rung 1 | cylinder | 4.00 | 3.61 × 5.12 | 45,572 | 174.7 | 628.8 | 0.0014 | certified |
| **P3 rung 2** | **cylinder** | **5.45** | 4.36 × 5.87 | 46,835 | 185.0 | 445.0 | 0.0013 | **certified (largest)** |
| P3 rung 3 | cylinder | 6.90 | 6.33 × 5.87 | 73,178 | 293.8 | ≥ 1,289 | — | UNKNOWN (support calls, ×2) |
| P3 rung 4 | cylinder | 8.58 | 5.88 × 9.22 | 149,303 | 581.0 | ≥ 1,714 | — | UNKNOWN (support calls) |

**Reading it.**
- **Does GMC still work at longer range?** For the sweeper and the uav, yes, and cheaply. From
  2.0–4.1 m (A2) and 2.36 m (P2) to 11.24 m, compile and query stay under 5 minutes each, and both
  routes are certified and replayed in 3D.
- **The clearance lower bound holds but shrinks** for the sweeper: 0.027 → 0.0059 → 0.00015 m. That
  follows from GMC certifying *a* route, not a max-clearance one: a longer route has more chances to
  pass close to something. For the uav it did not shrink (0.0127 → 0.0177 m).
- **Where it stops.** For the cylinder, the same start supports a certified route out to 5.45 m, in a
  25.6 m² window, and no further: the next map, at 73 k supports, exhausts the query budget twice. The
  sweeper/uav case shows range alone is not the limit. The cylinder's limit is its map's support count
  and the query effort that count forces, and a 12 × 12 cylinder window holds 119 k–753 k supports.

### 2026-09-18 ~20:0x — rungs 3 and 4: UNKNOWN on the support-call budget, not the wall clock

Resumed at 20:03 after a usage-limit wait. The wrapper started this session only once both rungs had ended.

| rung | sep m | supports | compile s | query s | status | reason | MaxRSS |
|---|---|---|---|---|---|---|---|
| 3 (17949914) | 6.90 | 73,178 | 283.9 | 722.5 | **UNKNOWN** | `query_support_budget_exhausted` | 3.37 GB |
| 4 (17949948) | 8.58 | 149,303 | 581.0 | 1,713.9 | **UNKNOWN** | `query_support_budget_exhausted` | 4.34 GB |

- **This is a different cap from P2's.** Both rungs spent the config's 50,000,000 support calls
  (`query.max_support_calls`) well inside the 7,200 s wall budget: 10 % and 24 % of it. P2's 156 k map
  hit the *wall* budget instead, at 7,968 s. Its `BUDGET=2` retry (P2's job 17944446) is now back,
  **UNKNOWN again** on the wall budget: 15,027 s against 14,400 s. So P2's point is a lower bound of
  **≥ 15,027 s**.
- **At `BUDGET=1` the ladder stops between rung 2 (5.45 m, 46,835 supports, certified) and rung 3
  (6.90 m, 73,178, UNKNOWN).** The support budget doubles, not the semantics, under the frozen rule's
  one budget-only retry. That is exactly the question this retry answers, so rung 3 gets it:
  **17960231** `pf_p3c_rung3_b2`. Rung 4 gets its own retry only if rung 3's certifies; otherwise the
  climb stops at rung 2.
- MaxRSS stays small (3.4 / 4.3 GB). The 21 G I requested for rung 4 was 5× too much; P2's 14.2 GB at
  156 k must come from the length of its query, not from its support count.
- Videos for the two certified long runs: **17960232** sweeper, **17960233** uav.

### 2026-09-18 ~20:3x — rung 3's one budget retry is UNKNOWN too: the ladder stops at rung 2

- **Rung 3 at `BUDGET=2`** (17960231): **UNKNOWN**, `query_support_budget_exhausted` again. The doubled
  100 M support calls ran out after a 1,289 s query (compile 294 s, MaxRSS 4.5 GB). The budget-only
  retry does not rescue it, so this is not a matter of a slightly-too-small budget. Doubling the calls
  also doubled the query time (723 → 1,289 s) and still ended short. Rung 4 does not get its retry: the
  plan was to retry it only if rung 3's certified.
- **Result for the cylinder, option (a): the largest window it certifies is rung 2** — 4.36 × 5.87 m
  (25.6 m²), 46,835 supports, **5.45 m** of separation, from the long case's start. The next rung, at
  6.90 m / 37.2 m² / 73,178 supports, fails at `BUDGET=1` and at `BUDGET=2`.
- **Video captions.** The first sweeper render (17960232) finished, and its title read "one window, one
  start, one goal, **three robots**". That is P2's caption, hard-wired for every plane-floor video, and
  it is wrong for P3's two-robot long case. The title also printed `replay3d lb = 0.000 m` for a
  certified 0.00015 m bound. Fixed in `percase_render.py` (6bdd6db): `--claim p3long|p3rung` presets
  (free text would be word-split by the sbatch `ARGS` export), and the bound at 3 significant figures.
  Each robot also gets its own manifest now, next to the shared `manifest.json` that the last render
  overwrites. Tested (`test_height_plane_render_claims.py`); P2's and A2's defaults are unchanged.
  Cancelled the in-flight uav render (17960233, my own) and re-rendered both: uav **17961624**,
  sweeper (see jobids).

### 2026-09-18 ~20:5x — compare figure, suite, and the video captions (second fix)

- **Compare figure** (17963195, 95 s, 3.1 GB) → `figs/p3_long_compare.png` + `p3_compare.json`. Opened.
  - Sweeper's GMC-certified route: **14.26 m**, north along x ≈ 7.2, round the west end of the display
    wall, then NE.
  - uav's: **12.13 m**, SE diagonal, then north along x ≈ 13.5.
  - Certified routes **5.73 m apart**. Not a copy of the pre-check's 5.728 m: that came from other
    curves, and the farthest points sit at the same corner.
  - Cylinder panel: rungs 0–2 solid (certified), rungs 3–4 dashed (UNKNOWN), rungs 5–6 not run.
- **Full suite** (17961361, 22 min, 0.55 GB): **555 tests, 530 passed, 25 failed**.
  - The 25: `test_atlas_benchmark.py` 10 and `test_atlas_gate_intervals.py` 15. All are
    `AtlasAssetError` / `FileNotFoundError`: the sealed Atlas package is absent from this worktree.
    Same class P2 reported; neither file nor `src/gmc/benchmarking/` was touched by P3.
  - Why there is no totals line: `pyproject.toml` already sets `addopts = "-ra -q"`, so an extra `-q`
    makes `-qq`, which suppresses pytest's final totals line. That is also why P2's suite log has none.
    I counted the progress characters instead.
  - The suite collected before `test_height_plane_render_claims.py` existed. Those tests (pure
    string/format checks) were run red → green during TDD.
- **Captions, second fix** (6657ab3). With the `p3long` preset, the first corrected sweeper render
  (17961697) got the right caption and `lb = 0.000147 m`. I opened frame 150. The caption is **clipped at
  both frame edges**, and "D1" falls off: the 3D title band holds the headline plus one caption line,
  ~135 characters at 13 pt on 1280 px. P2's own "three robots" caption is clipped the same way. The
  presets are now 129 / 125 characters, and a test pins the limit. Wrapping to a third line would
  collide with the legend row.
- Renders: sweeper **17963352**, uav **17963528**, cylinder rung 2 (the largest certified rung,
  `--claim p3rung`) **17963359**. Cancelled my own superseded uav render 17961624.
- 21:2x: the final sweeper render (17963352) has the full caption on its 3D frames. On the 2-panel frame
  the claims part (manual scene edit, D1, D4) is visible, but the first ~20 characters run off the left
  edge. That is `viz.robot_video`'s title layout, which P2's videos share; not re-rendered. uav
  (17963528) and cylinder rung 2 (17963359) are still on their 2-panel stage. The watchdog is 7 min out,
  so the rest goes to `state/P3.continue`.

### 2026-09-18 ~21:3x — uav and rung 2 videos checked; P3 done

- uav (17963528, 32 min, 2.9 GB) and cylinder rung 2 (17963359, 34 min, 3.8 GB). The 2-panel stage is
  the slow part: 22–25 min each.
- Opened the uav's 3D frame 151 and 2-panel frame 100, and the cylinder's 3D frame 300 and 2-panel frame 50.
  - **uav side view:** table A + bench (0.75–0.90 m) and the display case (~0.9 m) pass under its
    1.10–1.30 m band. That is the long case's morphology, seen from the side.
  - **cylinder:** it goes round table A (one solid block in its 0.02–1.75 m band) and stops at the rung
    goal (10.20, 9.95).
  - Captions are full on the 3D frames. The 2-panel titles lose their first ~20 characters, as recorded
    above.
- P3 standup finalized: `claude_jobs/logs/plane_P3_done.md`.

## P4 — where the wall clock goes

**Standing caveats, as for P1–P3:** the plane floor is a **user-approved manual scene-definition change**
(results sound w.r.t. the edited scene only); the case pairs P4 reuses come from P3, whose criterion 3 is
**relaxed by user decision D1** (crit1 ∧ crit3 = **0.9 m²** as built, **1.2 m²** with every phantom cell
deleted). P4 changes nothing it times: τ, ρ, robots, config, GMC, `verify_curve`, `replay3d` untouched.

### 2026-09-18 ~21:4x — what the existing records can and cannot say

- 12 runs carry `timing.py` blocks (P2 ×4 incl. the cylinder retry, P3 ×8); the three Amendment 2 runs
  carry only `compile_seconds` / `query_seconds`. P2's cylinder `BUDGET=2` result
  (`results/height/plane/shared/cylinder_b2.json`) is **still untracked** in git — P2's continuation
  wrote it but never committed it; P4 reads it and leaves the commit to P5 (it is P2's directory).
- **`compile_slabs` is the compile.** On rung 0 it is 78.0 of 81.0 s; `compile_pairs` and
  `compile_mobility` are ~1–2 % each.
- **`floor_replace` was never timed by `StageTimer`**: P1b ran before P1e existed, and every later run
  loads the finished npz. The only P1 number is `sacct` of a 4-variant sweep job (1 m 17 s).
- **The query is not a lookup.** Reading `mobility/query.py` (read-only, not touched): on an UNKNOWN
  attempt `query()` calls `refine_compiler(mc, …)`, which bisects an orientation slab and rebuilds the
  derived graph *in place*. With `initial_intervals: 1` a first query can therefore carry compile work.
  **Every P1–P3 run was one compile followed by one query**, so "compile once, query cheaply" has never
  been measured. And `run.compile_and_query` drops `result.report`, so no existing run says how many
  refinement rounds or support calls its query used.
- The runs landed on four node families (cs6xx, cl01x, gr10x, gl060) — cross-run timing noise the fits
  inherit. `sacct` `TotalCPU` reads 0 for every job, so CPU time is not available; wall time is.

### 2026-09-18 ~21:5x — the one timing sweep (17965980)

`experiments/plane_timing.py` (cfca4dc), `hpc/planefloor_timing.sbatch`: one job, four single-core lanes
(4 CPU / 48 G / 6 h; lane memory from `sacct` of the runs each repeats, worst case ~39 GB):
- **prep** — PLY decode → floor fit/rotate/crop (showcase `step_decode`, no writes), npz load, the
  Amendment 1 floor rule, 5 band rasters + phantom test, the P1b replacement (`phantom`+`centre`), the
  npz write (to a temp file, deleted), the plane-floor npz load; hall-wide projections; then compile-only
  on 51 k / 422 k / 703 k-support maps (memory- and time-guarded) to push the compile curve past 156 k.
  Counts are checked against P1's (7,319,425 → 7,247,831; 94,780 phantom cells; 7,101,868).
- **cyl** — the cylinder on **one** short pair (rung 0, 2.43 m) while the window grows 7 → 116 m².
  P3's ladder grew window and distance together; this separates them. Stops at the first UNKNOWN.
- **sweeper**, **uav** — the long case's window, 7 goals 2.43 → 11.24 m (the ladder's rung goals + the
  long goal): **warm** (compile once, query all 7, then all 7 again) and **cold** (fresh compile per
  goal; the 11.24 m goal first, as a calibration against P3's own run). Then the rung-0 pair on a
  growing window. The sweeper lane ends with the **cylinder warm** experiment (rung 2's window, rung 0–2
  goals) — the robot whose query dominates.
- Every query row records GMC's own `refinement_rounds`, `query_support_calls` and graph revision
  before/after, which the P1–P3 JSONs never kept.
- Self-test on `synth3d.table_scene("open")` passes in 25 s: all lanes certified + replayed; warm rows
  carry no compile stage. On that scene no query refined (0 rounds) and warm ≈ cold, i.e. the query's own
  cost there is certified path lifting, not deferred compile work.

### 2026-09-19 ~02:0x — the sweep is back (17965980: COMPLETED, 4 h 08 m, MaxRSS 18.5 GB)

Judged by its rows, not its exit code: all four lanes wrote `lane_done`; the cylinder window series
stopped at its first UNKNOWN by design (98,037 supports, 49,999,993 of 50 M support calls).
- **Warm = cold.** 61 of 63 sweep queries refined nothing, and their support-call counts are
  identical whether the map is fresh or has served 13 earlier queries. The query's cost is its own
  certified lifting, not deferred compile work. The one pair that refined (uav, 8.58 m: 1 round,
  2,214–2,454 s) re-queried in 1,323 s on the kept refinement.
- **The query is linear in the whole map.** At a fixed 2.43 m pair, calls per support stay constant
  as the window grows (248–252 sweeper, 124–133 uav, 993 cylinder); time ∝ N^0.98–1.00, R² 1.000.
  This is the mechanism behind D4. The cylinder's 50 M budget allows ≈ 50 k supports on that pair,
  and P3's ladder stopped between 46,835 and 73,178. Its limit is map size, not distance.
- **Compile linear to 703 k** (n = 50 in total). Compile-only memory is 0.47–1.32 GiB per 100 k
  supports: 3 points, no fit.
- **Calibration:** 1.11–1.30× slower than P3's cs601 for the identical runs; cold-compile CV 0.5–1.4 %.

**My own error, caught before it shipped:** the first draft of the final report said "43 of 44 hall
queries refined nothing". I had counted in my head. A recount from `records.json` gives 63 sweep
queries, 2 of which refined, both on the same pair. The earlier commit message (497a93d) carries
the wrong "43 of 44"; the report and this worklog are corrected. Also corrected before commit:
- the `compile_slabs` share over all 47 compiles is 95.7–96.8 %, not the 15-run 96.0–96.7 %;
- the uav warm ÷ cold range is 0.95–1.13, not 0.98–1.13.

Figures opened and fixed before any number was quoted from them:
- the prep title was clipped and the grid drew over the bars;
- the distance figure had an empty `project` panel (a projection has no A–B);
- the refinement label floated away from its point;
- a legend clipped a line.

`verify_curve ÷ query` is 0.47–0.52 in 66 of 69 finished units. The three outliers (0.14–0.26) are
the refining pair, where the query does work verification does not repeat.

## P5 — final sweep

**Standing caveats, repeated on every P5 artefact:** the plane floor is a **user-approved manual
scene-definition change**; results are sound with respect to the edited scene only. **Criterion 3 is
relaxed by user decision D1** to `r + 0.05` m on each robot's own certified map, plus start and goal in
one component of `dist > r` (crit1 ∧ crit3 = **0.9 m²** as built, **1.2 m²** with every phantom cell
deleted, `height_map_diagnosis.md` §3). Criterion 1 is **unchanged** for the shared cases.

### 2026-09-18 ~22:4x — the checklist, judged from artefacts

No `state/P5.json` existed; the two earlier P5 jobs (17982321, 17982482's first minutes) only waited
on P4's suite. Read the rules, Amendment 3 incl. Update 09-18, the main plan's constraints, the
diagnosis, the README, all four standups, all state and jobid files and every agent log.

What the artefacts say, not the exit codes:
- **Task 1 is not done.** P2's standup is still the *interim* one; P2's continuation never ran to the
  end (usage limit, then P3/P4 took the slots). Its jobs did finish: the cylinder's one budget retry
  (17944446) is **UNKNOWN again** — `query_wall_budget_exhausted` after **15,027 s** — so P2-SW-0 has two
  certified routes, not three. The compare figure (`--step compare`) was never run. The retry JSON,
  the P2d shrink series, both P2 videos and five P2 job logs were left **untracked**.
- Tasks 2, 3 and 4 are done and committed (P1, P3, P4 standups; commits listed there).
- P4's suite 17981839 is "FAILED" by exit code: **558 passed, 25 failed**, all 25 the missing sealed
  Atlas package (`test_atlas_benchmark.py` 10, `test_atlas_gate_intervals.py` 15), as in P3's run.

### ~22:5x — finishing task 1: P2's ranking again, with one measured filter

P2-SW-0's cylinder map is 156,426 supports. P3's ladder measured where the cylinder certifies: every
map up to **46,835** supports (rung 2) did, every map from **73,178** up did not, and doubling the budget
rescued neither map it was tried on. The user already approved sizing the cylinder's window by that
measurement for P3 (D4). So `plane_case_search.py --step affordable` walks **P2's own ranked candidate
list** (`p2a_search.json`, the 40 windows P2 screened, in P2's order) and takes the first window whose
exact cylinder map is ≤ 46,835 supports **and** whose exact pre-check passes every P2 criterion,
unchanged. The filter only removes candidates. P2-SW-0 stays reported as it stands.

Job 17986668, 3 m 48 s: the 18 SW windows all hold 153–171 k cylinder supports (over the cap); the
first region-A window over the cap too; the next, **P5-A-0**, passes on its first exact check:

| | sweeper | cylinder | uav |
|---|---|---|---|
| window `[8.1, 4.85, 11.4, 8.65]` (3.3 × 3.8 m), start (9.00, 5.75), goal (10.50, 7.75), 2.50 m | | | |
| supports | 915 | **38,473** | 1,856 |
| start / goal clearance (D1 needs) | 0.652 / 0.679 (0.225) | 0.615 / 0.637 (0.350) | 0.652 / 0.900 (0.300) |
| straight line unusable | **0.00 m** | **1.51 m** | 0.00 m |
| free-space route | 2.62 m | **3.48 m** | 2.62 m |

Opened `figs/p5_affordable0_A.png` first: the sweeper's map shows table A as leg dots and the route
goes straight under it; the cylinder's map shows the same table as one solid diagonal block and its
route swings round the table's SE end; the uav's map has no table at all. Criterion 1 passes (the
straight line runs under table A's top). Separation 0.94 m, detour ratio 1.33.

Sweeper (17988324) and uav (17988325) on P5-A-0: **REACHABLE, verify.certified, replay3d passed**,
46 s and 48 s wall. Cylinder 17988323 running.

### ~23:0x — P2-SW-0's compare figure, and a colour bug in its key panel

`--step compare` for P2-SW-0 (17986669, 40 s) confirms the three runs shared window, start and goal,
and draws the sweeper's certified 2.69 m route and the uav's 2.37 m; the cylinder panel shows its map and
"UNKNOWN". The fourth panel promised "grey = what the cylinder sees, red = what the sweeper sees" and
drew **yellow and no grey**: `imshow` of a masked all-True boolean array normalises the constant to 0,
the colormaps' zero colour (white for Greys, yellow for autumn_r). Fixed with explicit RGBA layers
(51c1b40, test pins the colours); the figure is re-run.

### ~23:1x — P2's 3D videos never show table B: the camera is behind a wall

Opened every P2 video frame (they had never been checked; P2 died before its continuation). The
2-panel frames are right: the side view shows table B's top at 0.68–0.86 m over 0.45–1.95 m of the
sweeper's path and the sweeper under it; the uav at 1.10–1.30 m over it. **The 3D frames are not:**
at frames 0, 97 and 193 the trail and the robot are drawn over a tall white gallery wall hung with
prints, the robot in x-ray, and table B never appears.

Investigated rather than excused. The renderer depth-tests its overlays (ghost where occluded, x-ray
robot), so the frames are consistent with a camera that has a wall between it and the route, not a robot
inside a wall. The route itself is clear: the certified maps, the 2-panel side view and replay3d
(lb 0.0059 m) agree, and a floor-to-ceiling wall on the route would be in the uav's 1.10–1.30 m band,
which is empty there. The default azimuth is travel − 50° = **−108°**, i.e. the camera stands SSW of the
route, behind the tall square enclosure at (−2.2…−0.3, 7.9…9.9) — the same enclosure f193 shows, with
the goal just east of its corner.

`experiments/plane_camera_scan.py` (15d19fb) replays the renderer's own follow camera over 60 poses of
each certified curve and counts frames whose sight line crosses a ≥ 1.1 m obstacle (P3's hall density
raster, uav band). It reproduces what the frames show — P2 default **60/60** blocked; P3 long sweeper
8/60, whose mid-clip frame (pose 7.22, 12.42) shows the robot behind the long display wall — and says:

| clip | default | chosen | why |
|---|---|---|---|
| P2-SW-0 sweeper, uav | 60/60 | **−20° → 0/60** | perpendicular to table B's long side (≈ 58°) |
| P5-A-0 sweeper, uav, cylinder | 0/60 | **+65° → 0/60** | perpendicular to table A's long side (≈ −25°) |
| P3 long sweeper / uav | 8/60 / 18/60 | not re-rendered | no azimuth does better than 8/60 / 17/60 on an 11 m route through partitions |
| P3 rung 2 cylinder | 0/60 | not re-rendered | already clear |

`percase_render.py --azim` (d7614c7) exposes the azimuth `pointcloud_video` always accepted; default
None leaves every earlier render unchanged. P2's re-render also gets its own caption, preset `p2shared`
("sweeper + uav certified, cylinder UNKNOWN") instead of the clipped "three robots" line. The superseded
P2 frames are kept in `shared/video/superseded_default_camera/` as the evidence for this entry.

P1's 14 figures opened: all match `p1b_build.json` / `p1c_survivors.json` / `p1d_evidence.json`
(only nit: p1b's suptitle overlaps its panel titles). P2d's five shrink figures match `p2d_evidence.json`
(64,294 → 156,426 cylinder supports; every shrink fails `connected_cylinder`, table B's block cuts start
from goal once the window cannot reach round it).

### 2026-09-18 ~23:1x → 09-19 03:0x — P5-A-0's cylinder certifies: task 1 has three certified routes

(The session hit a usage limit at ~23:08 and resumed at 03:03 in a new allocation, 17990543.)

Cylinder 17988323 on P5-A-0: **REACHABLE, verify.certified, replay3d passed** (lb 0.00096 m, 109,725
pairs checked). 38,473 supports; project 5.8 s, compile 152.5 s (`build_slabs` 146.6), query 419.6 s,
verify 196.0 s, replay3d 10.6 s. The probe projected 0.04 h against ~0.16 h of compile + query, a ~4×
under-estimate, in line with P2/P3's cylinder record. Certified routes on the one window, start and goal:

| robot | supports | certified route | replay3d lb | how |
|---|---|---|---|---|
| sweeper | 915 | 2.50 m | 0.00078 m | straight, **under** table A's vitrine |
| uav | 1,856 | 2.50 m | ≥ 0.05 m ¹ | straight, **over** it |
| cylinder | 38,473 | **3.44 m** | 0.00096 m | **round** the vitrine's SE end |

¹ 0.05 m is `replay_curve`'s search margin, the value `min_lb` starts at: the replay checked 0 pairs
because no opaque splat's ρ-AABB comes within 5 cm of the uav's 1.10–1.30 m band on this route. It is a
floor, not a measured clearance (A2's uav shows the same 0.0500).

P2-SW-0's re-rendered videos (17990174, 17990377, `--azim -20`) opened: table B is a glass-topped vitrine
on four legs; frame 1 has the sweeper's start beneath it, frame 97 the sweeper emerging from under it,
frame 193 the goal beside the enclosure; the uav's prism floats at 1.10–1.30 m above the vitrine. No
frame is covered by a wall any more. The `p2shared` caption loses its first and last character at the
frame edges (131 characters; the 135-character test bound is not tight enough for this font) — the
claims part is readable, so it was not re-rendered.
