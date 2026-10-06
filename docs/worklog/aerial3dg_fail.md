# Worklog: aerial3dg failure analysis (F1, 2026-10-01)

## Task 1: compile-range audit
- 21:45Z  Code read: G2's compile box = route prism u[-9,3.7] v[-0.35,2.75] z[0,2.43]. Crop
  (`gs3d.robots.crop_by_support_aabb`, called from `uavlamp_query.build_scene`) keeps opacity>0.3
  Gaussians whose 2-sigma world AABB overlaps the prism's WORLD AABB (a superset of the prism). The
  planner domain (`aerial3d.pairs.domain_from_scene` / `_known_box_rows`) = body world-AABB inside the
  prism, so the prism faces are walls for the planner: cylinder centres v in [0.050, 2.350], sweeper
  v in [-0.117, 2.517].
- 21:49Z  Job 18979876 (a3f_archive, 14 s, 1.36 GB): full-archive raster. Crop rule re-run on the G2 box
  gives 582,372 supports (= G2). Widened boxes: +1 m v 676,680; +1 m u,v 783,547; +2 m v 883,977;
  +2 m u,v 1,224,232.
- 21:52Z  Raster (`results/aerial3dg/f1/range/archive_raster_gap.png`, inspected): the "structure at
  u~-5.6" is a real partition at u~-5.9 only for v < 1.05. For v 1.1-2.6 the cylinder band contains just
  three sub-floor captured Gaussians (centres z -0.041/-0.045/-0.042 at v 1.19/1.59/2.15) whose 2-sigma tops
  are 0.01996 / 0.01999 / 0.01981 m: about 1 mm into the 0.001 m margin below the chassis bottom (0.02).
  Wall B is ABSENT at u -7.3..-4.3 (no wall Gaussians at v~2.6 there); the hall continues to v~4.
  Hypothesis to test: the box edge v=2.75 is what closes the cylinder's way round at u~-5.6.
- 21:54Z  Submitted widened compiles W1 (+1 m u and v): cylinder 18980207 (all 5000 pairs), sweeper
  18980208 (301 non-REACHABLE + slice 0-499), sequential (QOS headroom ~4 GB).
- 22:00Z  W1 cylinder compile (job 18980207): 320 s, in-process peak 1.38 GB, .a3c 570 MB. The first u~-5.6
  straddle pairs come back REACHABLE (G2-00001 5.95 m for 4.92 m straight), but each takes 12-70 s, >95 % in
  shortcut/tighten/merge. Projected ~13 CPU-h per variant, so I cancelled 18980207 (rows kept as
  `widen/w1/cylinder/fullconfig_partial.jsonl` for the agreement check) and re-ran the queries with a
  verdict-only QueryConfig (no shortcut/tighten/merge; same locate/cut/search/lift/own verifier/shared replay):
  job 18980787. Control: the G2 compile in the same mode, job 18980789.

## Task 2: diagnostics
- 22:02Z  Job 18980440 (sweeper diag):
  - 239/239 sweeper `*_not_certified_free` endpoints are in UNKNOWN leaves, inside the domain, oracle-free,
    and blocked from `grow_cell` by ONE (219) or two (20) captured Gaussians lying UNDER the chassis: 2-sigma
    top z 0.0180-0.0189, i.e. 1.1-2.0 mm below the chassis bottom (0.02). Refined gap to the margin-inflated
    C-obstacle 0.26-1.00 mm <= octree buffer 1 mm. So: sampler accepted clearance > margin (1 mm); the
    certificate needs clearance > margin + buffer (2 mm).
  - 62/62 `shared_replay_failed`: the shared replay's GEOMETRY passed on every one (clearance >= 1.03 mm);
    it failed `kinematics: speed_or_yaw_rate_exceeded`. G2/G3 called this "the replay was more
    conservative" (geometry); that was wrong. Rebuilding from 4-decimal route polylines passes, so the
    violation depends on exact float values. Hypothesis: tiny in-place turns (1e-12..1e-6 rad) at the
    collinear densify knots, timed as turn/1.0 rad/s and added to t ~ 30 s, so the elapsed time is rounded
    and yaw_rate exceeds 1 + 1e-9. Probe job 18981043 (exact polylines + replay with sub-µrad turns dropped).
- 22:09Z  Concurrency: 4 of my jobs ran at once for ~10 min (QOS had room; g5 arrays are throttled by task count,
  not by QOS memory). `scontrol update Dependency` / `hold` are refused here, so I serialise with dependencies at
  submit time from now on.
- 22:12Z  Kinematics probe v1 (18981043): violation confirmed on all 62 (one in-place turn of 0.4-2.6 µrad,
  yaw-rate excess 1.0-3.3e-9 at t 8-39 s), but dropping the tiny turn makes the replay's heading-match check
  fail (`ground_lateral_slip...`): not a valid fix. v2 (18981341): keep the turn, floor its duration at 1 ms ->
  62/62 replays pass (geometry + kinematics).
- 22:18Z  Buffer-0 endpoint cells (18981044): 238/239 sweeper endpoint rows become REACHABLE (own verifier +
  shared replay); 1 hits a replay veto.
- 22:19Z  Cylinder diag (18981090): 412 endpoint rows = 400 under-chassis + 12 lateral blockers; 3 sub-floor
  Gaussians (5337920, 5496444, 5739701; 2-sigma tops 0.0180-0.0182) own 342/412 (and 209/239 sweeper rows).
  Corridor: all 1587 CY-GAP pairs join cell components 0 (u -8.6..-5.67) and 1 (u -5.46..-1.33); the shortest
  UNKNOWN corridor (900 leaves, u -5.82..-5.28, v 1.39..2.25) has exactly 2 unresolved pairs: sub-floor
  Gaussians 5522629 (top 0.019990) and 5526162 (top 0.019805). The oracle at z_c says `unknown`
  (geometry_or_margin_unproven), not occupied: a margin violation, not a contact. 1 cm oracle grid: no free path
  across. Their C-obstacle top in body-centre z is 0.88599 / 0.88581 < slab top 0.886: a free sliver of
  0.01-0.19 mm remains at the top of the +-1 mm slab, so these leaves can never be BLOCKED (no single- or
  multi-pair certificate can exist in the slab), and they cannot be SAFE either.
- 22:24Z  W1 sweeper done (18980208, 18 min, MaxRSS 1.38 GB). W2 (+2 m u and v) failed at scene build: the
  fitted floor plane deviates 0.0500 m from z_floor at the box's world corners, the floor-support contract
  allows 0.05. Largest box with v+2 that passes: u+1.5 (0.0487 m). Resubmitted W2 = u[-10.5,5.2] v[-2.35,4.75].
- 22:41Z  G2-fast control (18980789): verdict-only mode on the G2 compile reproduces all 4927 G2 non-REACHABLE
  cylinder verdicts exactly; 8/73 REACHABLE get a replay veto (round-off). Verdict-only is faithful for this test.
- 22:44Z  W1 cylinder complete (18980787 + resume 18986134). Pair 4217 (G2 REACHABLE) raised ZeroDivisionError
  in `aerial3d/query.py:217 simplify` (coincident lifted points) on the W1 compile: a latent src bug, recorded as an
  ERROR row by my wrapper (src untouched). Result: 2928 lamp cuts unchanged; 412 endpoint rows unchanged; all 1587
  u~-5.6 rows find a route: 1122 REACHABLE + 465 own-verified/replay-vetoed. First 396 vetoes: replay geometry
  passed on all, kinematics failed on all (339 tiny turn, 57 sub-µm translation); the 1 ms turn floor clears 330.
  The W1 routes cross u=-5.6 at body-centre v = 2.4497..2.4704, INSIDE the old box (v <= 2.75) but above the old
  cylinder domain limit 2.35 (world-AABB inset 0.40 m in the rotated frame vs 0.30 geometric).
- 22:59Z  Full W1 veto breakdown (18987018): 468 vetoes, replay geometry passed on all; 398 turn (387 cleared by
  the 1 ms floor), 70 translation.
- 23:12Z  A* (18985512/13, 0.1 m lattice, G2 box): sweeper SW-EP 30/30, SW-KIN 30/30, controls 30/30 ROUTE.
  Cylinder: CY-LAMP 0/30 route, CY-GAP 0/30, CY-EP 3/30 (exactly the sampled buffer-0 REACHABLE ones),
  CY-EP-LAT 0/12, controls 30/30. My first outcome rule called "exhausted with within-margin frontier edges"
  UNSURE; I split it into NO_ROUTE_MARGIN post hoc from the stored counters (`oracle.reclassify`), because
  under the shared margin contract those edges are not traversable. No query hit the budget.
- 23:22Z  W2 cylinder (18985509, compile 392 s, MaxRSS 2.86 GB): same picture as W1: CY-GAP 1068 REACHABLE +
  519 vetoed, 0 disconnected; lamp 2928 and endpoints 412 unchanged. One G2-REACHABLE pair (G2-00917) becomes
  safe_graph_disconnected on W2 (not investigated).
- 23:25Z  Corrected my own wording: the W1 routes' body centres are inside the old box, but the body's edge
  reaches the old face v=2.75. The 1 cm scan shows the passage starts at v=2.45, so any box with that face
  closes it.

# Worklog: F2 (new region, confirmed-reachable cylinder benchmark, 2026-10-02)

- 12:39Z  Job 19029374 (a3f2_survey, 83 s, 2.35 GB): 5 candidate boxes (NW, NSTRIP, NE, S, E) cropped with G2's rule +
  a 5 cm oracle status / clearance map per robot (`results/aerial3dg/f2/region/survey.*`, inspected). Cylinder-free
  share of the grid: NW 51 %, NSTRIP 32 %, E 27 %, S 23 %, NE 20 %. The north strip / NE are cut by many floater
  blobs (oracle-occupied disks in the cylinder band, the faint squares of F1's raster). **Oracle clearance >= 0.02 m on
  0 % of every box, both robots**: clearance is 3-D body<->ellipsoid distance and the chassis is 0.02 m over the
  floor, so floor splats cap it. A literal ">= 5 cm oracle clearance" route exists nowhere.
- 12:52Z  Chose NW u[-11.5,-6.4] v[2.85,7.5] (200,755 supports, floor-contract deviation 0.0288 < 0.05). East face
  0.7 m west of the u~-5.7 wall line, south face north of wall B and of F1's three CY-GAP floor splats (v<=2.15);
  not the lamp booth. One obstacle (exhibit table ~(-10.5, 4.8)) in an otherwise open floor.
- 12:52Z  Probe compile, small NW crop u[-11.5,-9] v[2.85,5.2] (job 19029955): 54,503 supports; cylinder 1251
  candidate pairs, 2.2 s; sweeper 108 pairs, 3.3 s; 1.38 GB.
- 12:55Z  "Confirmed reachable" = robust body (cylinder r+0.05, top+0.05, same chassis bottom) at margin 0.003 gets an A*
  ROUTE, re-verified with the real cylinder at 0.001 (`experiments/aerial3dg_fail2_sample.py` docstring). Cheap
  pre-filter: shared 0.1 m robust-body lattice components, built once per region. Pilot job 19029956 (150 candidates,
  audit = A* also on pre-filter rejects).
- 13:00Z  Pilot 19029956: 150 cand -> 12 accepted, all A* ROUTE + real-cylinder re-verify; pre-filter never rejected
  (lattice 913 free nodes, components 912 + 1); ~2.8 s per accepted -> ~4 CPU-h projected for 5000, kept N=5000.
  NW probe 19029988: 200,755 supports, cylinder 2029 cand pairs / 11.6 s. Smoke 19030112: compile 8.5 s, 12/12 REACHABLE.
  Note: oracle `clearance_lower_m` is a lower bound (AABB distance for pairs outside the margin), so the real-cylinder
  re-verify reports 1.1 mm while the robust body certified > 3 mm; not a contradiction.
- 14:48Z  Streams 19030110/111 done (1.8 h each): 187,464 draws, 67,292 distance-passing, 5000 accepted; stage pass rates
  17.0 / 44.7 / 98.1 (pre-filter, 98 rejects unaudited) / 99.7 (13 NO_ROUTE_MARGIN) / 100 %. Distances 3.00-4.92 m.
- 17:00Z  GMC (19030276 cylinder 2.1 h, 19030277 sweeper 1.2 h): cylinder 4985 REACHABLE + 15 UNKNOWN
  shared_replay_failed, 0 UNREACHABLE; sweeper 5000/5000.
- 17:12Z  Veto probe 19050913 (full config): 14 = µrad in-place turn round-off (1 ms turn floor clears 14/14); 1
  (F2-03140) = replay geometry `map_unknown`: every exported pose inside the prism, but the swept-segment world AABB of
  the first 0.196 m segment exceeds the north face by 3.55 mm in the rotated route frame (`aerial3dg_fail2_diag.py`).
- 17:15Z  Task 2.1 (19030278/279): W1 468 / W2 531 vetoes, replay geometry passed on all; turn floor clears 387 / 520,
  turn + translation floor clears 468/468 and 531/531. W1's 81 undiagnosed = 64 translation + 17 translation+turn.
  Task 2.2: already in F1's fix/endpoints_buffer0_cylinder.json (all 412 rows); CY-EP-LAT 12/12 -> safe_graph_disconnected.
  Report: docs/aerial3dg_failures_f2.md. Total F2 compute 7.4 CPU-h.

# Worklog: F3 (G2 audit + hard-region search, 2026-10-06)

- 23:22Z  Task 0 pilot submitted first: 19309136 (real-body A* m0.001 r0.1 on 50 cylinder rows, 1 in 100), 19309137
  (2 mm subset body "s2" m0 r0.05 on 10 rows), 19309138/39 (global 0.05 m lattice labelling for s2 / s10 on all rows),
  19309140 (lamp passage width). Subset bodies (strict subsets of the real cylinder r 0.30, bottom 0.02, top 1.75):
  s2 = r -2 mm, bottom +2 mm, top -2 mm (z_c unchanged); s10 = r -10 mm, bottom +3 mm, top -10 mm. Bottom raise: every
  surviving floor splat has a 2-sigma top < 0.020 (plane-floor rule; F1 measured 0.0180-0.019990), so +2/+3 mm gives
  >= 2 mm vertical clearance at margin 0 and the floor stops grazing.
- 23:30Z  Pilot: real body ~13 s/row (CY-LAMP 25: 17 NO_ROUTE_MARGIN + 8 NO_ROUTE; CY-GAP 11 NO_ROUTE_MARGIN; CY-EP 5 NRM + 1
  ROUTE). Start components are tiny (289-565 expansions): the 0.1 m real-body lattice is fragmented; 95/188 Gaussians behind
  "unproven" rejections lie under the chassis. s2 A* r0.05: CY-LAMP clean NO_ROUTE (4575 expansions, 0 unproven) at ~50 s,
  CY-GAP ROUTE inside the G2 box at ~28 s. Global labelling s10: CY-LAMP 2928/2928 SEPARATED, CY-GAP/EP/EP-LAT/controls all
  CONNECTED (397 s for all rows). s2: same, 1 CY-LAMP NO_ATTACH.
- 23:34Z  Lamp width (19309372): the lamp + bulkhead close the crossing completely in the cylinder band (max-min clearance 0;
  widest free v-run per column 3 cm). Sub-bands: [0.02,0.5] and [0.5,1.0] leave a 0.48 m disk (< 0.60); [1.0,1.75] 0.04 m.
- 23:34Z  Task 0 arrays: 19309567 (real-body A*, all 5000 cylinder rows, 24 shards), 19309569 (sweeper 421 rows), 19309570 (W1
  real-body A* on CY-GAP+CY-EP+CY-EP-LAT, 8 shards), 19309571-75 (s2 A* r0.05 sample: CY-LAMP 1-in-9 = 326, CY-GAP 1-in-16,
  CY-EP 1-in-7, all CY-EP-LAT, controls 1-in-4). Per-row s2 A* on every non-ROUTE row would be ~60 CPU-h (lamp rows 50 s
  each), so the global 0.05 m labelling answers every row and the per-row A* sample cross-checks it.
- 23:36Z  Region survey (19309414, `results/aerial3dg/f3/region/survey_maps.png`): cylinder-free share ROUND 3 %, NORTH 7 %
  (both dropped), E 27 %, S 23 %, NMID 25 %, G2MID 37 %, GAPW1 44 % (with a chain of within-margin floor-splat disks).
- 23:38Z  Task 1: persisted compiles of both robots per region (a3f3_compile chain) + uniform pilots (quota 200, real body).
  scontrol hold/update refused on this cluster; to stay near the 16 GB share I cancelled two of my own not-yet-started
  pilot jobs (19309690, 19309698) and resubmitted them chained (19309716, 19309718). Transient peak ~16.5 GB requested for
  < 20 min while the sweeper audit finished.
