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

### P3 — why only this is doable, in numbers (INTERIM, as of 15:5x: rungs 3–6 pending)

Generated by `plane_long_search.py --step report` → `results/height/plane/long/p3_report.json` and
`figs/p3_scaling.png` (opened; three log-log panels, one y-axis each). Every support count below is
`project_scene`'s `kept`.

**1. Supports vs window area, per robot.** Filled rows are GMC runs; "projection" rows were never run.

| source | robot | window m² | supports | sup/m² |
|---|---|---|---|---|
| A2 | sweeper / cylinder / uav | 6.1 / 9.0 / 21.5 | 477 / 3,791 / 143 | 78 / 421 / 7 |
| P2 shared | sweeper / cylinder / uav | 24.1 | 1,779 / **156,426** / 19,867 | 74 / **6,496** / 825 |
| P2 shrink series, smallest (projection) | cylinder | 6.2 | 64,294 | 10,300 |
| **P3 long** | sweeper / uav (cylinder: projection) | 83.9 | **6,436 / 39,834** / 269,036 | 77 / 475 / 3,205 |
| P3 rungs 0 / 1 / 2 | cylinder | 7.2 / 18.5 / 25.6 | 21,144 / 45,572 / 46,835 | 2,917 / 2,462 / 1,828 |
| P3 rungs 3 / 4 / 5 / 6 (projection until run) | cylinder | 37.2 / 54.2 / 75.1 / 81.1 | 73,178 / 149,303 / 235,620 / 259,588 | 1,970 / 2,755 / 3,139 / 3,200 |
| **every 12 × 12 window below y = 32** (1,204, 0.5 m step; min / median / max) | sweeper | 144 | 3,745 / 9,444 / 22,737 | 26 / 66 / 158 |
| same | uav | 144 | 24,278 / 91,889 / 154,200 | 169 / 638 / 1,071 |
| same | **cylinder** | 144 | **118,856 / 477,726 / 752,899** | 825 / 3,318 / 5,228 |

The 12 × 12 row counts shadow centres on the 0.5 m density grid, so it slightly under-counts `kept`
(the r-dilated rim is omitted). **The cheapest 12 × 12 window anywhere in the hall holds 76 % of P2's
156 k-support map, and the median one 3×.** The prompt's "~900 k" (6,500/m² × 144) is P2's density
applied to the whole window. Measured, the densest 12 × 12 window holds 753 k.

**2. Compile and query per run, against the 7,200 s query cap.**

| run | robot | dist m | supports | compile s | query s | verify s | status | replay3d lb m |
|---|---|---|---|---|---|---|---|---|
| A2 | sweeper | 2.02 | 477 | 2.0 | 3.4 | — | REACHABLE | 0.0268 |
| A2 | cylinder | 2.69 | 3,791 | 17.3 | 409.4 | — | REACHABLE | 0.0032 |
| A2 | uav | 4.15 | 143 | 0.6 | 0.3 | — | REACHABLE | 0.05 |
| P2 | sweeper | 2.36 | 1,779 | 7.0 | 11.3 | 5.6 | REACHABLE | 0.0059 |
| P2 | uav | 2.36 | 19,867 | 108.5 | 171.9 | 84.8 | REACHABLE | 0.0127 |
| P2 | cylinder | 2.36 | 156,426 | 675.7 | **7,968 ≥ cap** | — | **UNKNOWN** | — |
| **P3 long** | **sweeper** | **11.24** | 6,436 | 25.0 | 107.5 | 52.3 | **REACHABLE** | 0.00015 |
| **P3 long** | **uav** | **11.24** | 39,834 | 153.4 | 292.9 | 145.5 | **REACHABLE** | 0.0177 |
| P3 rung 0 | cylinder | 2.43 | 21,144 | 81.0 | 174.8 | 85.2 | REACHABLE | 0.0014 |
| P3 rung 1 | cylinder | 4.00 | 45,572 | 174.7 | 628.8 | 312.1 | REACHABLE | 0.0014 |
| P3 rung 2 | cylinder | 5.45 | 46,835 | 185.0 | 445.0 | 219.8 | REACHABLE | 0.0013 |

**3. The cylinder at 12 × 12: what can and cannot be extrapolated.**
- **Compile, fitted on 5 points** (3,791–156,426 supports): `t = 0.0051 · N^0.98`, i.e. linear at
  ~5 ms per support. At the 12 × 12 min / median / max (119 k / 478 k / 753 k) that is **472 s /
  1,844 s / 2,879 s**. The median and max are 3.1× and 4.8× beyond the largest compiled map.
- **Query, fitted on the 4 finished points** (3,791–46,835 supports): exponent **0.09**, i.e. flat: 409 s
  at 3.8 k, 175 s at 21 k, 629 s at 46 k, 445 s at 47 k. **This fit is not predictive and I do not use
  it.** At 156 k it predicts 454 s; P2 measured > 7,968 s there, **under by at least 17.6×**. Query time
  is set by the route's tightest passages and the refinement they force, not by the support count. The
  only query measured above 47 k supports ran past the cap. So there is **no query extrapolation to
  12 × 12**: every 12 × 12 window holds more supports than that failed map, and nothing measured says
  such a query would finish. Rungs 3–6 (73 k–260 k) fill in the missing range.
- **The probe** (`showcase_run.py`, linear in supports × intervals), actual/projected: A2 cylinder
  **21×**, P2 cylinder **≥ 13×** (a lower bound: that run never finished), rungs 0/1/2 **1.7× / 4.6× /
  3.4×**, P3 long sweeper 3.6×, uav 2.0×. It is never an over-estimate for the cylinder, and its error
  is largest exactly where the query is slowest.

**4. Option (b)**: all three robots at ≥ 8 m via the Amendment 1 grid coreset would shrink the
cylinder's map itself, so the 12 × 12 support count is no longer the obstacle. **It was not run** (D4:
the user asked for (a) first). It is the next step.
