# A2 — deterministic airborne showcase scene, completed 2026-09-22

A2 adds a reproducible derivative of the existing manually floor-edited showcase,
without touching its read-only source. The generator appends exactly three opaque,
explicitly `manual_edit: true` Gaussian primitives: a hanging shade, its suspension,
and a ceiling attachment. The existing table-A arrangement is retained as source
geometry and labelled separately (`manual_edit: false`). This stage builds scene data
and a bounded feasibility witness; it does **not** claim planner success, coverage,
dynamic flight, or execution.

## Frozen handoff for A5

From `gmc/`, reproduce through the bounded compute helper:

```bash
python3 "$GS3D_ROOT/submit_compute.py" --stage A2 \
  --script "$GS3D_ROOT/compute/A2_scene_build.sh" --cpus 2 --mem-gb 8 --hours 1
```

- Archive: `gmc/results/gs3d_scene/showcase_airborne_v1.npz`
- Planning/render manifest: `gmc/results/gs3d_scene/showcase_airborne_v1.manifest.json`
- EWA render manifest: `gmc/results/gs3d_scene/render_manifest.json`
- Archive SHA-256: `1103b6a5124d4e858bbd03ff751b52bc10a0fd095d7b0212d3c7ac28a893044d`

The archive has 7,101,871 rows: 7,101,868 source rows plus three frozen manual IDs
7,341,022–7,341,024. `load_showcase_derivative(archive, manifest)` fails closed if
the archive hash, manual means, or manual covariances differ. A5 must use this loader
and archive rather than recreate the light from a sketch.

| Read-only source | SHA-256 |
| --- | --- |
| Existing floor-edited source | `a7931eab77e2ca2e46c86653502cd66582293b0eb01cab436792db96d0eb3c60` |
| Decoded original showcase | `c9318d253676209b43fd0159786b74df2b628954f524c20ad1cc06d0f58c88ae` |
| Original PLY | `98af4928cc67523015bd32c4f438d334adb86db59bc20ef1c97227bf367cc3c2` |

World units are metres (covariance m²), z is up, `tau=.3`, `level=2`. Route origin is
`(9.0, 5.75, -1.2271749593107995)` m; world-to-route rows are `(.6,.8,0)`,
`(-.8,.6,0)`, `(0,0,1)`. Frozen UAV body: radius `.25` m, half-height `.10` m,
strict margin `.05` m. Route domain `[-.75,-1,.20]`–`[3,1,2]` is explicitly
`assumed_map_domain`, not observed free space.

Every manual row records its mean, covariance, semiaxis, opacity (.95), RGB, ID, role,
frame, units and note. The existing table selection contains 18,010 opaque source
supports intersecting route envelope `[.65,-1.2,.15]`–`[2.15,1.2,1.25]`. Its measured
union is `[.70593,-.56978,-.17859]`–`[2.27524,1.40112,.90510]` m and its original
table top is `.9050975954` m above floor. No table geometry was lowered, removed, or
manually edited.

## Candidate feasibility bounds and visual review

The exact archive audit tests all 45,843 opaque 2-sigma source/edit AABBs overlapping
route crop `[-1.5,-1.5,-.1]`–`[3.5,1.5,2.5]`. It encloses every `.02` m interval in an
upright `.25 × .25 × .10` m body box and subtracts `1e-8` m slack. This is a
conservative A0-style continuous enclosure witness, not A1's production
ellipsoid/body-oracle certificate.

| Ordered leg | Route endpoints (m) | Clearance lower bound |
| --- | --- | ---: |
| low under light | `(-.25,0,.65)` → `(.50,0,.65)` | `.0804868341` m |
| rise after light | `(.50,0,.65)` → `(.50,0,1.40)` | `.0699999900` m |
| high over table | `(.50,0,1.40)` → `(2.50,0,1.40)` | `.0699999900` m |

Altitude swing is `.75` m (frozen requirement: at least `.50` m). Low crossing
`s=[-.20,.20]` precedes high table crossing `s=[.92,1.88]`. Low body top `.75` m is
below light underside less margin (`1.00` m), and high body bottom `1.30` m exceeds
measured tabletop plus margin (`.95510` m). These are frozen geometry gates, not an
unconstrained-shortest-path assertion.

I opened the tiny companion and all actual EWA renders. The side view
`showcase_airborne_side.png` visibly shows original table top/legs, gold shade underside,
and cyan low-rise-high witness. Oblique/high views retain the same archive and show the
gold light/path/table relation, although a real nearby wall occludes part of their
context; it was retained, not deleted. The EWA render selected 87,461 archive Gaussians
by full 2-sigma AABB overlap; camera and culling counters are in `render_manifest.json`.

## Evidence and checks

- Opened tiny fixture: `gmc/results/gs3d_scene/tiny_companion_scale.png`.
- Build job `18232908` successfully built/hash-checked the archive, then failed only due
  to an EWA dictionary-unpack error; retained in `logs/gs3d_compute_A2-18232908.err`.
- Commit `1772bf9` fixes that isolated API use. Render/verification job `18243418`
  completed in 22 s (MaxRSS 3,066,900 KiB), confirmed archive identity, all three PNGs,
  and `all_legs_pass: true`.
- `PYTHONPATH=src:experiments MPLBACKEND=Agg /scratch/wg2381/.conda/envs/gmc-venv/bin/python -m pytest -q tests/unit/test_gs3d_scene.py` → **2 passed**.

Limitations remain deliberate: opacity is not coverage, table selection is a labelled
support measurement rather than semantic reconstruction, and cyan is not a planner
trajectory. A1/A5 must apply the shared 3-D oracle, declared coverage, and ordered
regions when producing actual trajectory/replay evidence.
