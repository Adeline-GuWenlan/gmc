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
