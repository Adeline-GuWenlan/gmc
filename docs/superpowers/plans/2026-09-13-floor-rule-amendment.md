# Amendment 1 — floor-surface rule and grid coreset (2026-09-13)

**Status:** approved by the user on 2026-09-13 at ≈21:15 EDT, after A2 escalated in
`/scratch/wg2381/claude_jobs/logs/height_A2_done.md`. This document amends
`docs/superpowers/specs/2026-09-12-height-band-robots-design.md` and
`docs/superpowers/plans/2026-09-12-height-band-robots.md`. Everything not named here is unchanged. It is
executed by stage **A4**; A3 and C2 read it too.

## Why

A2 found that the GS floor is a fuzzy layer of splats. 50,305 opaque splats centred within 5 cm of the
fitted floor have ρ-extents reaching above `z_lo = 0.02` (p99 top 0.136 m), and a glossy-floor
reflection layer sits below. Every ground robot's map is speckled with floor shadows. In window A,
3,037 of the sweeper's 3,876 supports are floor splats. No start/goal pair in the scene satisfies
case criteria 1–3.

## Spec addendum

### §3.1a Floor-surface rule (user-approved scene-definition change)

With the floor fit in `results/height/showcase/g0.json` (`floor.normal` = n, `floor.centroid` = p0),
an opaque splat is **floor surface** iff all of the following hold:

- (a) plane offset `|(μ − p0)·n| ≤ 0.05 m`, measured along the plane normal, so the 0.285° tilt is
  accounted for;
- (b) its thinnest principal axis `e_min` satisfies `|e_min·n| ≥ cos 15°` (the splat lies flat);
- (c) flatness `σ_min ≤ 0.5·σ_mid` (a disc, not a blob).

Floor-surface splats are removed from the one scene definition that every showcase projection,
`replay3d` and `viz` call uses, for all robots. Legs, walls, table edges and blobs are kept: they fail
(b) or (c), or lie outside (a).

The rule moves verdicts toward REACHABLE, and every report states it next to the claims boundary.
Known cost: flat floor-level objects (rugs, flat cables) are removed too. Parameters, counts and
before/after figures go to `results/height/showcase/floor_rule.json` and
`results/height/showcase/figs/floor_rule/`.

If the residual clutter still leaves no valid case, the chain **stops and reports the numbers**. The
thresholds are not tuned further without the user.

### §5.4a Grid coreset (user-approved conditional)

If a robot's timing probe projects more than 20 h, its projected map may be reduced as follows:

1. Group supports by the 0.10 m cell containing their centre.
2. Replace every multi-member group with `gmc.coreset.macro.outer_ellipse(members)`. It certifies
   containment over all directions (`slack ≥ 0`, else `RuntimeError`).
3. Keep singletons unchanged.

Obstacles only grow, so the reduction can turn an answer into UNKNOWN or not-reachable but never into
a false safe. It is recorded as `reduction` in the robot's result JSON (cell, counts, min slack,
fallbacks). `replay3d` still checks against the 3D scene.

## Tasks for stage A4 (in order; TDD; one commit each, tagged `[height F<n>]`)

### Task F1: `height/floor.py` — floor-surface mask

**Files:** create `gmc/src/gmc/height/floor.py`, `gmc/tests/unit/test_height_floor.py`.
**Produces:**
- `floor_surface_mask(scene3d, normal, centroid, *, max_offset=0.05, max_tilt_deg=15.0, max_flatness=0.5, chunk=1_000_000) -> np.ndarray[bool]`
- `apply_floor_rule(scene3d, floor: dict, **kw) -> tuple[GaussianScene3D, dict]`, where `floor` is `g0["floor"]`.
  Stats keys: `n_input, removed, removed_opaque_tau0.3, params`.

```python
# gmc/src/gmc/height/floor.py
"""Floor-surface rule (Amendment 1, spec §3.1a): drop flat splats lying on the fitted floor."""
import numpy as np


def floor_surface_mask(scene3d, normal, centroid, *, max_offset=0.05, max_tilt_deg=15.0,
                       max_flatness=0.5, chunk=1_000_000):
    n = np.asarray(normal, float)
    n = n / np.linalg.norm(n)
    p0 = np.asarray(centroid, float)
    out = np.zeros(len(scene3d), dtype=bool)
    cos_max = np.cos(np.deg2rad(max_tilt_deg))
    for s in range(0, len(scene3d), chunk):
        m = scene3d.means[s:s + chunk]
        w, v = np.linalg.eigh(scene3d.covs[s:s + chunk])        # ascending eigenvalues
        near = np.abs((m - p0) @ n) <= max_offset
        lying = np.abs(v[:, :, 0] @ n) >= cos_max               # thinnest axis ~ normal
        flat = np.sqrt(np.maximum(w[:, 0], 0)) <= max_flatness * np.sqrt(np.maximum(w[:, 1], 0))
        out[s:s + chunk] = near & lying & flat
    return out


def apply_floor_rule(scene3d, floor, **kw):
    mask = floor_surface_mask(scene3d, floor["normal"], floor["centroid"], **kw)
    params = {"max_offset": kw.get("max_offset", 0.05), "max_tilt_deg": kw.get("max_tilt_deg", 15.0),
              "max_flatness": kw.get("max_flatness", 0.5)}
    stats = {"n_input": len(scene3d), "removed": int(mask.sum()),
             "removed_opaque_tau0.3": int((mask & (scene3d.opacity > 0.3)).sum()), "params": params}
    return scene3d.subset(~mask), stats
```

Tests, written first, must all hold:
1. Floor splats σ = (0.05, 0.05, 0.005) with centres at z = 0 ± 0.01 and random tilt ≤ 5° are removed.
2. Vertical leg needles σ = (0.01, 0.01, 0.1) centred at z = 0.04 are kept (fail b).
3. An isotropic blob σ = 0.02 centred at z = 0.02 is kept (fails c).
4. A flat splat at z = 0.20 is kept (fails a).
5. A flat splat tilted 30° at z = 0 is kept (fails b).
6. On a floor plane tilted 2°, flat splats that follow the plane (z offset up to 0.04 at x = ±1) are
   removed, which proves the offset is measured along the plane normal.
7. `table_scene("closed")` with normal (0, 0, 1) and centroid 0 removes 0 splats.
8. Chunked and unchunked masks are equal.

### Task F2: apply the rule in `showcase_scene.py`; floor-rule figures; maps and case again

**Files:** modify `gmc/experiments/showcase_scene.py`; outputs `results/height/showcase/floor_rule.json`
and `figs/floor_rule/*`; regenerate `figs/band_*.png`, `overhang_candidates.png`, `case.json` and
`case_overview.png`. The old versions stay in git history.

1. Replace `load_processed` with:
   ```python
   def load_processed(floor_rule=True):
       d = np.load(DATA / "processed.npz")
       g0 = json.loads((RES / "g0.json").read_text())
       scene = GaussianScene3D(d["means"], d["covs"], d["opacity"], d["ids"], "showcase")
       if floor_rule:
           from gmc.height.floor import apply_floor_rule
           scene, st = apply_floor_rule(scene, g0["floor"])
           g0["floor_rule"] = st
       return scene, g0
   ```
   Every existing caller (maps, section, case, `showcase_run.py`, and `height_viz.py` later) now gets
   the amended scene definition.
2. Add `--step floor_rule`. It writes `floor_rule.json` with:
   - the rule stats;
   - opaque splats within 5 cm of the plane that reach above 0.02, before and after;
   - for window A `[5.5, 3.5, 12, 10.5]`, sweeper and cylinder `project_scene` support counts, before
     (`load_processed(False)`) and after;
   - the fraction of the window where an r = 0.175 disc fits, before and after, using the method of
     `/scratch/wg2381/splathjb/gmc/outputs/height/a2/proj_map_diag.py`.

   It saves `figs/floor_rule/proj_map_A_before_after.png` (sweeper and cylinder, before and after) and
   `figs/floor_rule/removed_z_hist.png`. **Open both figures and describe them in the worklog** before
   writing numbers.

   Check that table A's legs and tabletop are still red in the after-maps. If a leg vanished, the rule
   removed a real obstacle: stop and report.
3. Re-run `--step maps` and open the figures.
4. Case search, **table A first** (the intended demo: top 0.80 m, open knee space). Reuse the logic of
   `a2/case_aid.py` and `a2/pair_search.py` against the amended scene, which they get through
   `load_processed()`, then `--step case`.
   - Case criteria are unchanged.
   - If table A has no passing case, try tables B, C and G, then the pair search over the whole scene.
   - If nothing passes, commit, write the standup with the numbers, and **stop** (no G1).
5. Commit.

### Task F3: `height/reduce.py` — grid coreset; wire into the robot job

**Files:** create `gmc/src/gmc/height/reduce.py`, `gmc/tests/unit/test_height_reduce.py`; modify
`gmc/experiments/showcase_run.py` and `gmc/hpc/height_showcase_robot.sbatch`.

```python
# gmc/src/gmc/height/reduce.py
"""Grid coreset (Amendment 1, spec §5.4a): certified outer ellipse per 0.10 m centre cell."""
from collections import defaultdict

import numpy as np

from ..coreset.macro import outer_ellipse
from ..types import SceneModel2D


def grid_outer_reduce(scene2d, cell=0.10):
    groups = defaultdict(list)
    for s in scene2d.supports:
        groups[tuple(np.floor(np.asarray(s.mean) / cell).astype(np.int64))].append(s)
    out, merged, fallbacks, min_slack = [], 0, 0, float("inf")
    for members in groups.values():
        if len(members) == 1:
            out.append(members[0])
            continue
        me = outer_ellipse(members, primitive_id=min(m.primitive_id for m in members))
        if not me.certified:
            raise RuntimeError("uncertified macro ellipse")
        out.append(me.support)
        merged += 1
        fallbacks += int(me.fallback)
        min_slack = min(min_slack, me.slack)
    stats = {"cell": cell, "n_in": len(scene2d.supports), "n_out": len(out),
             "groups_merged": merged, "fallbacks": fallbacks, "min_slack": min_slack}
    return SceneModel2D(tuple(out), scene2d.workspace, scene2d.name + f"_grid{cell:g}"), stats
```

Tests, written first:
1. 300 random small ellipses in a 1 m square reduce to fewer supports.
2. For every merged group, `gmc.coreset.macro.certify_outer_containment(members, macro) >= 0`.
3. Primitive ids stay unique.
4. A scene where every support is a singleton comes back unchanged.

Wiring in `showcase_run.py`:
- **Flags:** add `--reduce-cell` (float, default 0.0) and `--auto-reduce`.
- **Explicit reduction:** if `--reduce-cell > 0`, reduce `s2` right after projection, before the probe.
- **Automatic reduction:** after the probe sets `probe["abort"] = True`, and only if `--auto-reduce`:
  1. reduce `s2` and the probe map `sp` with cell 0.10;
  2. re-time the probe on the reduced `sp` with the same `pcfg` and endpoints;
  3. recompute `projected_hours` with the reduced counts;
  4. set `probe["abort"]` from the new value and continue with the reduced `s2`.
- **Recording:** store `out["reduction"]` (stats plus the reduced probe numbers) whenever a reduction
  happened.
- **sbatch:** add `[ "${AUTO_REDUCE:-1}" = "1" ] && ARGS+=(--auto-reduce)` and
  `[ -n "${REDUCE_CELL:-}" ] && ARGS+=(--reduce-cell "${REDUCE_CELL}")`.

A3 rule: a follow-up run (`BUDGET`, `TAU`) for a robot whose first run was reduced must pass
`REDUCE_CELL=<same cell> SKIP_PROBE=1`, so the maps it compares are the same.

### Task F4: projection counts and submission (plan Task 11, Steps 4–6)

1. Run the inline projection dry-check on the new `case.json` and record the counts.
2. Submit the three robot jobs with `hpc/height_showcase_robot.sbatch` (probe + auto-reduce on by
   default).
3. Record the IDs in `/scratch/wg2381/claude_jobs/height/jobids/A4.txt`.
4. Submit A3: `sbatch --parsable --dependency=afterany:<ids> /scratch/wg2381/claude_jobs/height/a3.slurm`.
   Its script carries `--begin=2026-09-14T04:03:00`. Record its ID in `A4.txt`.
5. Commit, then write the standup `logs/height_A4_done.md`.
