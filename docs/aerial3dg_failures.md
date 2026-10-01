# aerial3d-ground: why the 5000-pair run fails where it fails (F1, 2026-10-01)

**Summary (6 lines)**
1. **Compile range = placement box, not the hall.** One prism u[-9,3.7] v[-0.35,2.75] (582,372 of 7.1 M Gaussians) is both the compile domain and the sampling box. Its faces are walls to the planner, and they are 0.1 m more conservative than the geometry for a cylinder in this 64°-rotated frame.
2. **The box *was* the cause of the 1587 cylinder `safe_graph_disconnected` UNKNOWNs.** Widened by 1 m, W1_GAP_SUMMARY. The way round passes at body-centre v = 2.45, inside the old box, which the old domain forbade (v ≤ 2.35).
3. **The "structure at u≈-5.6" north of v 1.1 is three sub-floor Gaussians**, 2σ tops 0.0198–0.0200 m, 0.01–0.2 mm below the chassis bottom. They block the cylinder only through the 1 mm margin, and they graze the ±1 mm C-space slab, so no cut certificate can exist there (UNKNOWN is structural, not resolution).
4. **All 651 `*_not_certified_free` endpoints (239 sweeper, 412 cylinder) have one mechanism.** Each endpoint is oracle-free by 1–2 mm over a sub-floor Gaussian (3 Gaussians own 551 of them). The certificate needs margin + buffer = 2 mm. A buffer-0 endpoint cell fixes 238/239 sweeper rows.
5. **All 62 sweeper `shared_replay_failed` rows are float round-off in the exported trajectory's yaw-rate check (excess 1–3e-9), not geometry.** Replay geometry passed on all 62; with a 1 ms floor on in-place turns all 62 pass. G2/G3's "replay was more conservative" was wrong.
6. **The 2928 cylinder UNREACHABLEs are real and box-independent**: certified lamp + bulkhead cuts, unchanged in W1/W2; ORACLE_LINE.

Machine-readable: `gmc/results/aerial3dg/f1/classes.csv` (one row per non-REACHABLE pair), `classes.json`,
`class_crosstabs.json`. Process: `docs/worklog/aerial3dg_fail.md`. Every number below comes from a committed file
under `gmc/results/aerial3dg/f1/` produced by a job listed in §7. "(unverified)" marks claims I did not test.

## 1. Compile range (Task 1)

**(a) What enters the compile, and what the box edge means.**
- *Crop:* `uavlamp_query.build_scene` (experiments/uavlamp_query.py:134-138) builds the route prism as a
  `RouteBoxKnownSpace`. It keeps every opacity > 0.3 Gaussian whose 2σ **world** AABB overlaps the prism's
  **world** AABB (`gs3d/robots.py:169`). That is a superset of the prism, with no extra pad for the robot.
  None is needed: the body itself must stay inside the prism (next point), so any Gaussian that can touch it
  overlaps the prism.
- *Pair pruning* adds `pad_m = 1e-3` (`aerial3d/pairs.py:301-311`).
- *Domain:* the planner's C-space domain is "the body's world-axis-aligned AABB lies inside the prism"
  (`pairs.py:211-216` via `_known_box_rows`, rows at `pairs.py:249`), plus the ±1 mm ground slab (`pairs.py:264`).
  Leaves outside it are OUTSIDE (`octree.py:140-141`) and never enter the possible graph (`api.py:160`). The cut
  certificate says so explicitly ("remaining boundary ... is the C-space domain boundary", `api.py:268`).
- **So the box edge is a wall.** In the 64°-rotated route frame the world AABB of an r = 0.30 m cylinder spans
  0.30·(|cos|+|sin|) = 0.40 m in v. The cylinder's centre is therefore confined to v ∈ [0.050, 2.350], not the
  geometric [-0.05, 2.45]. The sweeper's is v ∈ [-0.117, 2.517].

**(b) Compile range vs placement range.** They are the same box.
- G1's demo compile, which G2 reused, is `box_route` u[-9,3.7] v[-0.35,2.75] z[0,2.43]
  (`results/aerial3dg/g1/runs/demo/demo_cylinder.json`).
- G2 sampled every endpoint uniformly in exactly that box (`pairs_5000.json → box_uv`, `aerial3dg_batch.py:53`).
- Endpoints were checked free by the gs3d oracle on that box's scene, which also requires the body inside the
  prism (`gs3d/oracle.py:165`).
- My re-run of the crop rule on this box returns 582,372 supports, the same as G2 (job 18979876).
- The compile is **not** the whole scene.

**(c) Where the box edges sit relative to the failing structures, and what lies outside**
(`results/aerial3dg/f1/range/archive_raster*.png`, inspected; full archive, job 18979876):
- *Lamp, u -1.03..-0.67:* spans wall A (v≈0) to wall B (v≈2.4) with the bulkhead above it. Both walls are real
  full-height walls in the booth, and the back panel closes u = 3.3. No way round exists inside or outside the
  box short of passing through walls.
- *"Structure at u≈-5.6":* it is two different things.
  - A real partition at u≈-5.9 for v < 1.05. It continues south far beyond the box (to v < -3.5).
  - North of v≈1.1, nothing but **three captured sub-floor Gaussians** in the cylinder band: ids 5492xxx, see
    §3.2, at v 1.19 / 1.59 / 2.15. Their 2σ tops are 0.01996 / 0.01999 / 0.01981 m.
  - Wall B is **absent** between u -7.3 and -4.3: no wall Gaussians at v≈2.6, and the hall continues to v≈4.
  - So the cylinder's escape north of the last blob lies just inside the old box face, where the old domain
    could not reach, with open floor beyond.
- *u_min = -9:* open hall continues west (the raster shows no wall). *v_min = -0.35:* wall A for u > -6; open
  for u < -6.

**Decisive experiment: re-compile on widened boxes, same 5000 pairs.**
- **W1** = +1 m in u and v: u[-10,4.7] v[-1.35,3.75], 783,547 supports.
- **W2** = +1.5 m u, +2 m v: u[-10.5,5.2] v[-2.35,4.75], 1,070,899+ supports. +2 m in u as well was not
  possible: at the +2/+2 box's world corners the fitted floor plane deviates 0.0500 m from `z_floor`, and the
  shared floor-support contract allows 0.05 (`aerial3dg_run.floor_support`; job 18981509 failed on exactly
  this). u+1.5 gives 0.0487.
- Same archive, crop rule, `CompileConfig` and code; one compile per robot per box.
- Cylinder: all 5000 pairs in **verdict-only** query mode (`aerial3dg_fail_widen.py`). Same
  locate / cut / portal search / lifting / own verifier / shared gs3d replay; shortcut / tighten / merge are
  skipped. Those took > 95 % of the 12-70 s per long detour at full config.
  - Control: the G2 compile in the same mode reproduces all 4927 G2 non-REACHABLE verdicts exactly. It is
    slightly pessimistic on REACHABLE: 8/73 get a round-off replay veto (§3.3).
- Sweeper: full G2 config on its 301 non-REACHABLE + the unbiased slice 0-499 (770 pairs).

WIDEN_TABLE

## 2. Taxonomy: every non-REACHABLE row in exactly one class (Task 2)

TAXONOMY_TABLE

## 3. Mechanisms ("why UNKNOWN"), traced through code and per-pair diagnostics

### 3.1 SW-EP / CY-EP / CY-EP-LAT: the endpoint cannot be grown into a certified-free cell
ENDPOINT_TEXT

### 3.2 CY-GAP: safe graph disconnected while the possible graph is connected
GAP_TEXT

### 3.3 SW-KIN: own verifier passed, shared replay vetoed
KIN_TEXT

### 3.4 CY-LAMP: certified UNREACHABLE
LAMP_TEXT

## 4. Independent truth check: gs3d lattice A* on a stratified sample
ORACLE_TEXT

## 5. What would fix each class

FIX_TABLE

## 6. Corrections to earlier reports
CORRECTIONS

## 7. Jobs, files, reproduction
REPRO
