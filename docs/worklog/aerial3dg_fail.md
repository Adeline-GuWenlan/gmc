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
