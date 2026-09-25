# uav-conn design: uav.md connectivity / graph-search backend for the axisymmetric UAV

Status: C1 design, written and committed **before** any backend code (2026-09-25).
Spec: `/scratch/wg2381/splathjb/uav.md` (Verified v2). Binding rules:
`/scratch/wg2381/claude_jobs/uavconn/prompts/_rules.md`. Baseline: `LatticePlanner`
(`gmc/src/gmc/gs3d/planner.py`) as run by `gmc/experiments/uavlamp_run.py`, report
`docs/uavlamp_report.md`.

**One-paragraph honest summary.** The backend is *not* the exact 3-D Boolean arrangement of
uav.md §7.3–§7.4 (no robust C++ Boolean kernel is available here: no pybind11/CGAL). It is a
**pair-certified support-plane cell complex**: a pair-driven adaptive octree over the query box
(uav.md §13.3) labels configuration-space boxes SAFE / BLOCKED / UNKNOWN only from Gaussian-pair
certificates; SAFE boxes seed **convex free cells whose facets are support planes of certified outer
pair polytopes** (IRIS/GCS-style, one greedy pass, no ellipsoid iteration); cells are joined by
**certified portals**; graph search runs over cells and portals; the path is lifted through the portals,
shortened inside them, and verified by an own direct pair verifier and then by the baseline's
independent `gs3d` replay. UNREACHABLE is claimed only from the possible side: start and goal lie in
different components of the union of non-BLOCKED boxes, where every BLOCKED box lies inside the inner
polytope of one named pair. The octree is therefore part of the method, not only a fallback; §2 says
exactly which uav.md claims this does and does not earn.

## 1. Mapping from uav.md to code

New package `gmc/src/gmc/aerial3d/` (reuses `gs3d` oracle/geometry/timing/trajectory/smoothing
unchanged; forks none of it). Status enum reused from `gmc.types.PlanStatus`.

| uav.md | What is implemented | Module |
|---|---|---|
| §3.1 hard-support semantics | Identical to the baseline oracle: level-κ ellipsoid `E_i = {μ_i + κ L_i v, ‖v‖≤1}`, κ = `SceneSpec.level` (2.0), only opacity > τ (0.3); body = analytic upright cylinder (r, h); free ⇔ distance > margin m (0.05). See §3. | `pairs.py` |
| §4.1 Phase U0 | q = p ∈ R³ (upright cylinder is yaw invariant). Yaw (§9) and dynamics (Phase 6) **not** implemented. | — |
| §5.1–5.4 pair C-obstacle, support value/point | `O_i = μ_i + κE(Σ_i) ⊕ Cyl(r,h) ⊕ B(m)`, exact closed-form support `h_i(u) = u·μ_i + κ√(uᵀΣ_i u) + r‖u_xy‖ + h|u_z| + m‖u‖` and support point. The body is one convex primitive, so "pair" = (scene Gaussian i, body cylinder). | `pairs.py` |
| §6.1 direction set | Level-2 icosphere (162 unit rows = 81 antipodal pairs; the ±axes are among them, snapped exact); stored binary64 rows are the rows consumed. Adaptive refinement adds directions per pair when a fixed direction does not separate. | `envelopes.py` |
| §6.2 outer polytope | `P⁺_i = ∩_k {u_k·x ≤ h_i(u_k) + ε}` (ε = directed fp allowance). Because every `O_i` is centrally symmetric about μ_i, `h_i(u) = u·μ_i + ρ_i(u)` with even `ρ_i`; only the 81 half-directions' ρ are stored. | `envelopes.py` |
| §6.3 inner polytope | `P⁻_i = conv{x_i(u_k)}` (support points; computed lazily with qhull, H-rep shrunk by ε). | `envelopes.py` |
| §6.4 sandwich + audit | `P⁻ ⊆ O ⊆ P⁺` audit on analytic, random and adversarial (needle/pancake, rotated) pairs: inner vertices satisfy dense-direction exact support; exact support points satisfy P⁺ rows; P⁻ vertices ⊆ P⁺. Adaptive direction refinement; pair provenance on every facet. **Not** exact/dyadic predicates: floating-point conservative slack, as in `gs3d` (`numerical_slack`). | `envelopes.py`, tests |
| §7.2 safe/possible unions | Never materialised as Boolean unions. F_safe is under-approximated by the union of SAFE boxes and convex cells (each certified disjoint from every `P⁺_i`); F_possible is over-approximated by the union of non-BLOCKED boxes (each BLOCKED box ⊆ one `P⁻_i`). So `cells ⊆ F_safe ⊆ F_true ⊆ F_possible ⊆ ∪ non-BLOCKED boxes`. | `octree.py`, `cells.py` |
| §7.3 arrangement / complement dual | **Partially.** Connected free volumes = connected components of the cell graph; shared portals = certified boxes/faces inside two cells; boundary patches = cell facets, each a pair support plane (pair id, direction, outer side, slack) or a workspace face; lineage = seed leaf of each cell. Not a Boolean union, no tetrahedralisation, no exact complement. | `cells.py`, `graph.py` |
| §7.4 C++ exact Boolean backend | **Not implemented** (no pybind11/CGAL; numba/ctypes only). Replaced by §2's construction; numerical doubt → UNKNOWN. | — |
| §7.5 complexity control | Exact C-obstacle AABB pruning against the domain and per octree node; symmetric-ρ storage; lazy inner hulls; convex-cell budget with box-cell fallback; query-local refinement of UNKNOWN boxes. All prunes conservative (§2.3). No learning. | `pairs.py`, `octree.py` |
| §8 U0 pipeline | compile (scene + body + box) → query (start + goal). | `api.py` |
| §10.1 portal-based lifting | start → portal → … → goal, every segment between two points of one convex cell (hence inside it). | `query.py` |
| §10.2 query-local routing | Dijkstra/A* over portal points; then path shortening with each vertex constrained to its portal box (convex; L-BFGS-B box bounds); then greedy shortcut accepted only by the direct verifier. | `query.py` |
| §10.3 smoothing | Optional: the **same** `gs3d/smoothing.optimize_trajectory` used for the baseline, with its own continuous re-verification. Never inherits the raw path's safety flag. | `metrics.py` (runner) |
| §11.1–11.2 continuous verification | Own verifier: every segment vs every candidate pair (fresh AABB query over all pairs), separated by an exact support plane of `O_i` (fixed + refined directions) with adaptive segment bisection; unresolved → UNKNOWN; point inside some `P⁻_i` → collision. Then the **shared** baseline check `gs3d.validation.verify_path` / `gs3d.trajectory.replay_plan` with a fresh `GaussianBodyOracle` (same body, margin, τ, level, archive). | `verify.py` |
| §11.5 REACHABLE rule | graph path ∧ lifted polyline ∧ own verifier CERTIFIED ∧ shared gs3d replay passed. | `query.py` |
| §13.3 octree conditions | scene never voxelised; every box label replayable from pair oracles; each box stores its pair IDs; SAFE proves the whole box free; BLOCKED proves the whole box inside one `P⁻_i`; unresolved stays UNKNOWN; final path direct-verified. | `octree.py` |
| §17 / §18 U0 rows | Synthetic acceptance tests (Task 2). Yaw rows (suspended ring with yaw, yaw slot, yaw monotonicity) and "incomplete high sky" are out of scope. | tests |
| §21.3 outputs | REACHABLE with graph path, polyline, clearance certificate, pair provenance, workspace certificate; UNREACHABLE only with a possible-space cut; UNKNOWN with reason. | `api.py` |

Not implemented: §9 (yaw product complex), §11.3–11.4 (yaw segments; the body is yaw invariant),
§12.2 deployment augmentation, Phase 6 dynamics, §7.4 exact Boolean kernel, exact/dyadic predicates.

## 2. The free-volume complex: construction and its honest name

**Name: pair-certified support-plane cell complex.** Four layers, all in the *planning frame* — the
box's own frame (for the gallery, the route frame `x_F = R(x_W − o)`, a rotation about z plus a
translation; covariances rotate as `RΣRᵀ`, nothing is projected or dropped).

1. **Pair envelopes** (§5–§6): for every candidate pair, μ_i, Σ_i, the 81 stored ρ-values (outer), and
   on demand the inner hull.
2. **Pair-driven adaptive octree** over the bounding box of the C-space domain D (§3). A node box B
   (centre c, half extents d) is
   - OUTSIDE if one domain row separates B from D;
   - **SAFE** if B ⊆ D (strictly) and for every candidate pair some direction u gives
     `|u·(μ_i − c)| − ρ_i(u) − |u|·d > buffer + ε` — i.e. B lies strictly on the far side of a support
     plane of `P⁺_i` (fixed directions first, then adaptive pattern-search refinement of u, which refines
     `P⁺_i` per §6.4). Stores, per pair, the witness direction;
   - **BLOCKED** if all 8 corners of B are inside `P⁻_i` (H-rep shrunk by ε) for one pair i — a cheap
     necessary test `B ⊆ P⁺_i` ranks candidates first. Stores that pair id;
   - otherwise split into 8 children down to `min_cell_m`; unsplittable mixed boxes are **UNKNOWN** and
     store the unseparated pair IDs (first 32 + count) and whether D's boundary crosses them.
3. **Support-plane convex free cells.** SAFE leaves, largest first, that are not yet inside an existing
   cell seed a new cell. Growth is a single greedy pass (IRIS without the ellipsoid iteration): sort
   candidate pairs by their best separation gap from the seed box; for the nearest pair not yet
   excluded, add the halfspace `n·x ≤ n·μ_i − ρ_i(n) − buffer` whose normal n is the best separating
   direction from the seed (a support plane of `P⁺_i`, so `O_i` is outside it and the seed inside it);
   mark every pair excluded that this plane already puts outside; repeat until no pair is left.
   The cell is `D ∩ (those halfspaces)`: every facet is a pair support plane (pair id, direction index,
   outer side, buffer, ε) or a domain face. A cell budget bounds the work; SAFE leaves left uncovered
   when it runs out become *box cells* (the leaf itself; its certificate is the leaf's pair separation,
   its facets are box faces — reported separately so the traceable-facet fraction is honest).
4. **Portal graph.** Nodes are cells. Two cells are joined when they share a SAFE leaf (portal = that
   leaf box, contained in both) or contain two face-adjacent SAFE leaves (portal = the shared face
   rectangle, contained in both). Hence **the cell graph is at least as connected as the SAFE-leaf
   graph** (every SAFE leaf is in some cell). Query-time cells grown from the start/goal point are
   joined by LP (HiGHS) Chebyshev points of cell intersections.
5. **Possible-side graph** (for UNREACHABLE): non-BLOCKED, non-OUTSIDE leaves with *touch* adjacency
   (face, edge or vertex contact of closed boxes, from exact integer leaf coordinates). Every point of
   F_true lies in such a leaf; a finite union of closed boxes has the touch-graph components as its
   connected components. Therefore start and goal in different components ⇒ no continuous
   collision-free path exists in D (certified, up to floating-point slack). The certificate lists the
   BLOCKED leaves bounding the start component with their pair IDs.

**What this earns (uav.md claims).** Full 3DGS input, no projection (§1.1, §14 unchanged loader);
collision only from Gaussian pairs (§1.2); every free-cell facet traceable to a pair support plane or
the workspace, every BLOCKED/UNKNOWN box traceable to pair IDs (§1.3; fractions reported); the sandwich
`P⁻ ⊆ O ⊆ P⁺` with audit (§6.4); the three-valued discipline (§21.3): REACHABLE only after direct
continuous Gaussian verification, UNREACHABLE only from a possible-space cut, everything else UNKNOWN.

**What it does not earn.** It is not an exact Boolean arrangement of `∪P⁺` / `∪P⁻` (§7.3–7.4): F_safe
is covered only down to the octree's SAFE leaves (plus whatever the convex cells add), so a free passage
narrower than about two `min_cell_m` boxes can be missed (→ UNKNOWN, never a false UNREACHABLE); the
possible-space cut uses boxes each inside a *single* `P⁻_i`, so a separating wall that is only closed by
the union of several inner polytopes at sub-`min_cell_m` scale is not certified (→ UNKNOWN). Connectivity
is resolution-bounded exactly as §13.2 warns for octrees; the difference from "3DGS → octree labels →
A*" is that the searched graph's nodes are pair-bounded convex cells (not grid cells), the negative
certificate is an explicit pair cut, and every label is pair-certified over the whole box.
No exact arithmetic: all certificates are floating-point conservative with declared slack, like `gs3d`.

### 2.3 Pruning, and why each prune is conservative

| Prune | Rule | Why it cannot drop a relevant pair |
|---|---|---|
| τ | opacity ≤ τ removed | identical to the baseline oracle's collision semantics (§3), not a planning shortcut |
| domain AABB | keep pair iff `AABB(O_i)` meets D's bounding box and no row of D separates `O_i` (`min_{O_i} a·x > b`) | `AABB(O_i) = μ_i ± (κ√diag Σ_i + (r+m, r+m, h+m))` is the exact AABB of the convex `O_i` (support values on ±axes), inflated by ε; a pair outside D cannot touch any configuration in D |
| node AABB | a child keeps a parent candidate iff `AABB(O_i)` meets the child box | same exact AABB; a pair whose AABB misses the box cannot intersect it |
| cell growth exclusion | pair skipped once a previously added plane puts all of `O_i` outside (`n·μ_i − ρ_i(n) > β`) | `ρ_i(n) ≥ true support` (outward-rounded), so `O_i` is certified outside the cell |
| verifier AABB | segment vs pair tested iff `AABB(O_i)` meets the segment's AABB inflated by `pad` | pairs outside are farther than `pad`; the reported clearance lower bound is capped at `m + pad` |

The real input is already the baseline's crop (support-AABB overlap with the box's world AABB, 359 201
Gaussians for M1); the domain prune then keeps only pairs whose C-obstacle can reach D.

## 3. Collision semantics: identical to the baseline oracle

Baseline (`gs3d/oracle.py`, `GaussianBodyOracle.edge`): a closed segment of body centres is *free* iff
(a) the body's world AABB is inside the known-space prism and more than `margin` inside the scene's world
bounds, and (b) for **every** scene Gaussian with opacity > τ, a support-plane gap between the swept
cylinder and the level-κ ellipsoid, minus slack, exceeds `margin`. Unresolved → unknown (fail closed).

Per pair, the baseline's forbidden set of centres is
`F_i^base = {p : its gap certificate fails} ⊇ O_i := {p : dist(Cyl_{r,h}(p), E_i) ≤ m}`
and `O_i = E_i ⊕ Cyl_{r,h}(0) ⊕ B(m)` (the cylinder is centrally symmetric, so `−Cyl = Cyl`). Its
support function is the §1 formula. The two sets differ only on the band where the baseline's
GJK-direction search cannot certify a true gap (floating-point slack ~1e-8 m, or its 40-iteration limit
near tangency).

- **Outer ⊇ baseline-forbidden (in practice).** `P⁺_i ⊇ O_i` exactly (each halfspace is a support
  halfspace, outward-rounded). My SAFE boxes and cells keep a `buffer` (default 1e-3 m ≫ 1e-8 m slack)
  from `P⁺_i`, so they avoid the band where the baseline might refuse. This is not a proof about GJK
  convergence; it does not need to be: REACHABLE additionally requires the baseline replay to pass, and a
  replay failure is reported (status UNKNOWN, reason `shared_replay_failed`), never hidden.
- **Inner ⊆ baseline-forbidden (proved).** `x_i(u) = μ_i + κΣ_i u/√(uᵀΣ_i u) + cyl(u) + m u` is a sum of
  points of the three summands, so `x_i(u) ∈ O_i`; convexity gives `P⁻_i ⊆ O_i ⊆ F_i^base`. A BLOCKED
  box is therefore inside the baseline's forbidden set too; a possible-space cut is a cut for the
  baseline as well.
- **Domain.** D is the exact set the baseline accepts for condition (a): body world AABB
  `p ± (r, r, h)` inside the prism (planning-frame rows `x_k ± e_k ∈ [lo_k, hi_k]`,
  `e_k = Σ_j |R_kj|·(r,r,h)_j`) and at distance > m from the world bounds (6 rotated rows). D is a convex
  polytope (12 rows); SAFE needs strict inclusion with slack; OUTSIDE needs one row violated by the
  whole box. For synthetic `KnownBox` scenes (no holes) the frame is the identity and D is a box.
- Same τ (0.3), level (2.0), body (r 0.25, h 0.10), margin (0.05), archive, box, start and goal as the
  baseline run JSONs. The shared final check is Codex's replay for **both** methods.

## 4. Complexity and runtime estimate for the real scene

Domain (M1 box): prism u [−3.0, 3.7], v [−0.35, 2.75], z [0, 2.43] shrinks to D ≈ 6.03 × 2.43 ×
2.13 m (e_u = e_v = 0.25·(0.431 + 0.902) = 0.333 m; z ∈ (0.15, 2.28) from world bounds + margin).

| Step | Estimate | Basis |
|---|---|---|
| load + hash + crop (not algorithm time) | ~3 s, ~1 GB RSS peak | baseline: 1.37 s load, 0.60 s crop |
| domain prune | 359 k → ~150–250 k pairs, < 5 s | prism is ~½ of its world AABB area |
| ρ table (81 half-directions) | ~250 k × 81 × 8 B ≈ 160 MB, ~1 s | vectorised |
| octree | 5·10⁴–3·10⁵ nodes, ≤ 10³–10⁴ candidates each, 1–15 min | C-obstacle union boundary ~50–100 m² at 5 cm leaves |
| inner hulls | 10⁴–5·10⁴ qhulls × ~0.5 ms ≈ 5–30 s | lazy, cached |
| convex cells | 10²–10³ cells × 0.2–1 s = 1–20 min (budgeted) | one O(N) exclusion pass per plane |
| adjacency | < 1 min | KD-tree over leaf centres, exact integer test |
| query (locate, search, lift, shorten) | < 5 s | graph of ≤ 10⁴ nodes |
| own verification | 1–30 s | vectorised, AABB-pruned |
| shared gs3d replay of ~5–15 segments | 10–300 s | pure-Python GJK; long edges have large swept AABBs |

Peak memory ≈ 2–4 GB (archive arrays during load dominate). **Plan if too large**: (1) coarser
`min_cell_m` (0.10) and larger root cells; (2) numba kernels for the node classification loop;
(3) cell budget with box-cell fallback; (4) query-local refinement: compile coarse, then refine only the
UNKNOWN boxes in the start/goal possible component (§7.5.7); (5) restrict the compile to the box
(already the case — the box *is* the query-local compile region; no per-query box is ever chosen).
Task 3 measures the real numbers before C2 runs anything.

## 5. Pre-registered predictions for C2's real-scene queries

Same archive (`2a3a72d6…96cc`), same box, same body/margin/τ/level, start + goal only. M1/M5/C4h use
the booth (C4h: lamp-only variant) compile; N2/N2h use the plug-variant compile (compile is per scene,
reused across its queries).

| Query | Expected verdict | Under the lamp? | Expected length | Expected time |
|---|---|---|---|---|
| M1 low start | REACHABLE (P ≈ 0.9) | yes: the u ≈ −0.85 slab (lamp + header) spans D except the tunnel under the lamp (tunnel in C-space ≈ 0.9 m long, z ≈ 0.17–0.93, v ≈ 0.3–2.0) | 2.9–3.35 m (< baseline 3.383 m; continuous vs 0.1 m lattice); ≤ 10 vertices vs 29 knots | compile 2–30 min once; query ≤ 10 s + shared replay 10–300 s; ≫ 10× faster than 6 100 s |
| M5 high start | REACHABLE (P ≈ 0.9) | yes, descends below ≈ 0.93 then climbs | 3.0–3.5 m (< 3.549 m) | same compile (reused); query ≤ 10 s + replay |
| N2 low start + plug | **certified UNREACHABLE** (P ≈ 0.8), else UNKNOWN; never REACHABLE | — | — | plug compile 2–30 min; query < 5 s |
| N2h high start + plug | certified UNREACHABLE (P ≈ 0.8), else UNKNOWN | — | — | as N2 |
| C4h lamp only, high start | REACHABLE (P ≈ 0.95), **over** the lamp, near straight (1–3 segments) | no (over) | ≈ 2.875 m (straight-line distance) | lamp-only compile 2–30 min; query ≤ 10 s + replay |

Why N2 should be certified, not just exhausted: the plug (u = −0.85, v −0.05…2.45, z 0…1.10), the lamp
(bottom 1.10) and the header (z 1.25…2.40) are hole-free panel slabs (`panel_min_half_thickness`); their
pair C-obstacles form a slab ≈ 0.6 m thick in u that spans D's whole v range (−0.017…2.417) and z range.
A 5 cm box in its core lies inside a single panel pair's `P⁻`, so a closed BLOCKED layer should exist
without relying on the real walls. Risk: junctions between panels or real clutter could leave only
UNKNOWN boxes → honest UNKNOWN. Both methods must agree on the verdict class (no false REACHABLE).

## 6. Implementation notes added after the design commit (C1)

§1–§5 above are the pre-registered design (commit `59625e9`) and are left unchanged except for the
direction count (the level-2 icosphere already contains the ±axes: K = 81, not 81 + 3). What the
implementation (`55bd6d1` onward) added or changed, and why:

1. **Grid tiling.** The octree root grid tiles the domain bbox exactly (per-axis leaf edge ≤
   `min_cell_m`), inset by 2·tol. Straddling roots produced thousands of partial/OUTSIDE boxes. The
   uncovered sliver (2·tol ≈ 2e-8 m) is thinner than the inner-hull shrink ε_in (4e-8 m): a sliver point
   next to a BLOCKED box is still inside that pair's inner polytope, so the cut argument covers the sliver.
2. **Refinement skip bound.** `g(u) = |u·(μ−c)| − ρ(u) − |u|·d` is Lipschitz on S² with constant
   `‖μ−c‖ + R_i + ‖d‖` (R_i = circumradius of O_i − μ_i) and every unit vector is within the icosphere's
   covering radius (10.81°) of ±U. Refinement is skipped only where this bound proves it cannot succeed.
3. **Portals.** Up to 8 portal regions per cell pair (best 3-D shared leaf / 2-D shared face per
   0.5 m bin), so A* sees several crossing locations between two large overlapping cells.
4. **Lifting.** After A* over portal points, portal points are first optimised inside their portal
   boxes (L-BFGS-B), then over the full intersection of their two convex cells (SLSQP; a convex
   program for the fixed cell sequence, GCS-style). Every lifted segment is checked to lie in one cell.
5. **Post-processing (uav.md §10.3).** Greedy shortcut, then certified taut-string tightening: a vertex
   moves toward its neighbours' chord only if the own direct verifier certifies both new segments;
   midpoints are inserted between rounds so the path can bend around corners; collinear points
   dropped. Without it the low-wall route was 3.75 m (analytic ≈ 3.43 m, lattice 3.66 m); with it
   < 3.55 m. The cell-certified polyline is kept in the result (`cell_polyline_plan`).
6. **Corner merging.** A run of vertices wrapping an obstacle corner is replaced by the intersection of
   its end tangents if both new segments certify with the buffer and the path grows by ≤ 3 % (global
   default, chosen on the synthetic suite). The gs3d A6 smoother eases to a stop at every anchor, so a
   taut string with many short segments scored a high smoothed jerk; merging fixes most of that.
7. **Buffer in post-processing.** Shortcut, tightening and merging accept a segment only if the own
   verifier's clearance ≥ margin + buffer (1 mm), like the cells, so the shared replay never has to
   certify a near-tangent segment.
8. **Status strings.** `gmc.types` imports shapely (2-D backend); the 3-D backend uses the same
   names as strings and never loads shapely (tested).

## 7. Measured real-scene cost (C1 Task 3; sbatch, cpu_short, 2 CPU)

Real archive `2a3a72d6…96cc` through the manifest-checked loader and the baseline's `build_scene`
(359 201 cropped Gaussians; the domain prune keeps 284–289 k pairs). Per-stage numbers are in
`gmc/results/uavconn/probe/*.json`; job accounting in `gmc/results/uavconn/c1_handoff.json`.

| Probe (job, commit) | compile | of which octree / cells+portals | MaxRSS | query | result |
|---|---:|---:|---:|---:|---|
| M1 (18511509, 5ee328c) | 582 s | 51 s / 528 s | 1.37 GB | 8.95 s (tighten 6.1 s, shared replay 0.07 s) | REACHABLE, 3.056 m, 4 vertices, under the lamp then above the table |
| N2 plug (18513500, 87f14c3) | 507 s | 51 s / 453 s | 2.93 GB | 0.14 s | certified UNREACHABLE (cut: plug + lamp + header slab and walls/floor) |
| C4h lamp only (18513501, 87f14c3) | 256 s | 40 s / 213 s | 2.18 GB | 1.55 s | REACHABLE, 2.8749 m straight line over the lamp |

Against the §4 estimate: compile at the upper-middle of 1–30 min, memory below the 2–4 GB estimate,
query faster than estimated because the shared replay of 1–3 long segments is cheap. These are
C1 de-risking runs; the pre-registered §5 predictions stand and C2 evaluates them.
