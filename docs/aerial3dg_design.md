# aerial3d-ground design note (G1 Task 0)

**Seam: no new motion tag and no wrapper package. `aerial3d.pairs.domain_from_scene` accepts the
`ground_unicycle` motion that `gmc.gs3d` already defines, and adds a z-window to the C-space domain.**
The bodies are the frozen `gmc.gs3d.robots.SWEEPER` (r 0.175, half-height 0.04, ground clearance 0.02)
and `CYLINDER` (r 0.30, half-height 0.865, clearance 0.02). These are exactly `robot_table()`'s prisms
(z_lo = clearance, z_hi = clearance + 2·half-height). For a ground body the domain gets two extra rows,
`z_c − δ ≤ z ≤ z_c + δ` (labels `ground:z_window_min/max`), with
`z_c = h_floor + ground_clearance + half_height`, taken from the scene's own `SupportSurface`.
Everything downstream only sees domain rows and `PairSet(radius, half_height, margin)`: the full-3D pair
C-obstacles, envelopes, octree, support-plane cells, portal A* and the continuous verifier. That code is
unchanged, and nothing is projected. The band restriction reuses the existing pair pruning: a Gaussian
whose C-obstacle lies wholly above `z_c + δ` or below `z_c − δ` fails the existing domain-row test and is
never a candidate. I rejected a new `"ground_translation"` tag because gs3d's independent replay
(`replay_plan` + `GaussianBodyOracle`) already implements ground semantics for `ground_unicycle`: the
body centre must be on the support manifold to within 1e-7, turns happen stopped with no lateral slip,
and swept-footprint support evidence is required. Reusing that tag means the shared final check audits
the real ground contract and no second convention is created. `gmc/src/gmc/height/` is not touched.

**Why a 1 mm slab (δ = `CompileConfig.ground_slab_m` = 1e-3) and not z = z_c exactly.** The octree
grid and the cell/bbox LPs need a full-dimensional domain. The octree tiles each axis separately, so a
thin slab only costs a few z-levels, not a 3D room's worth. The slab is conservative: a SAFE leaf or
cell certifies the body at *every* centre height within ±1 mm. Start and goal must lie on `z = z_c` (a
query off the manifold returns UNKNOWN `start_off_ground_manifold`). After lifting, every polyline vertex
is pinned to `z_c`. Then the own continuous verifier and the gs3d replay decide, as for the UAV.

**z-window and floor convention.** z is height above the scene's floor, and there is only one floor.
The archive's `meta.z_floor` (−1.2271749593107995 m world) is also `origin_world_m[2]` of the site
("route") frame in `uavlamp_scene.npz`'s manifest, and `scene_uavlamp` defines route z as the height
above the plane floor. The support is the same constant plane `gs3d_run._constant_floor_support` used
for the earlier ground runs: normal (0,0,1) through `z_floor`, with an evidence box equal to the query
prism's world bounds. The fitted plane's deviation from `z_floor` at the window corners is recorded as
`height_error_m` (it must be ≤ 0.05). In route coordinates:

| robot | r (m) | body band above floor (m) | body centre z_c (m) | domain z-window (m) |
|---|---|---|---|---|
| sweeper | 0.175 | 0.02 – 0.10 | 0.060 | 0.059 – 0.061 |
| cylinder | 0.300 | 0.02 – 1.75 | 0.885 | 0.884 – 0.886 |

**Margin.** Ground bodies use `margin_m = 0.001`, which is gs3d's ground convention (`PlannerConfig`,
`run_long_cylinder`). The UAV uses 0.05. The planefloor tiles are clamped to top ≤ z_floor + 0.015 and the
chassis bottom is at z_floor + 0.02. With a 0.05 margin the floor itself would block the sweeper
everywhere. With margin 0.001 plus the octree buffer 0.001, the body stays ≥ 2 mm from every level-2
support, and the tiles leave 5 mm − δ = 4 mm.

**Compile once, query many.** `compile_complex(scene, body)` / `query(compiled, start, goal)` stay
separate, as they are for the UAV. Repeat queries against one in-memory `CompiledComplex` pay no compile.
For G2's 5000 pairs, the array tasks are separate processes, so the compile is also **persisted**:
`save_compiled(compiled, path)` pickles it and writes a SHA-256 sidecar, and `load_compiled(path)` checks
the hash and returns the same object graph. The chain compiles once per (scene, robot, box), and every
array task loads that one artefact. G1 proves both paths on the demo pair: identical cold and warm
in-process results, with no compile stage in the warm call's timing records, and an identical result from
a reloaded artefact.

**Known risk carried from uavconn.** The shared gs3d oracle tests the *world-axis-aligned* AABB of a whole
swept segment against the rotated route prism. Long diagonal segments near a box face therefore come back
`map_unknown` even when every configuration on them is inside the domain (uavconn EA03/EB02/EB04). Those
results stay UNKNOWN (`shared_replay_failed`) and are counted as such. G1 does not change the shared
verifier.
