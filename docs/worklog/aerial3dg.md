# Worklog: aerial3d-ground (sweeper + cylinder on the pure-3D aerial3d backend)

## G1

- 2026-09-28T06:23Z agent job 18700054 started fresh (no G1 state). Read _rules.md, plan, aerial3d code (pairs/api/octree/cells), gs3d contracts/robots/oracle ground branch, uavlamp loader + manifest, uavconn worklog.
- Floor convention found, not invented: archive meta `z_floor` = -1.2271749593107995 = site frame `origin_world_m[2]`; route z = height above floor; planefloor tiles clamped to top <= z_floor+0.015 (gs3d_run `_constant_floor_support`, chassis bottom z_floor+0.02).
- Scene geometry relevant to the contrast: lamp + bulkhead span the corridor's full width (v -0.08..2.48) from z 1.07 up to the soffit, so the cylinder (top 1.75 m) can never cross u~-0.85 inside the box; the sweeper (top 0.10 m) passes under. Table against wall A (u -0.02..2.97, v -0.03..0.79, top 0.94).
