"""Hand-crafted robot-conditioned coarsener (design revision phase 2).

This is the deterministic baseline the learned coarsener must later beat, and
simultaneously the teacher that generates its labels.  It answers the project's
own go/no-go question R1: *does a mobility compression signal exist at all?*

Admissibility rule
------------------
A hierarchy node may collapse to one macro ellipse only if the obstacle mass it
*adds to free space* is nowhere deeper than ``clearance_tol`` (eps):

    excess(v) = (E_plus(v) cap W) \\ union_j E_j        (all scene obstacles)
    admissible(v)  <=>  excess(v).buffer(-eps) is empty

Overshoot that lands inside another obstacle is free, so the interior of a solid
wall collapses at no cost while its boundary stays fine.

A uniform ``eps`` is wasteful, though: it spends the same clearance budget on a
harmless bulge in the middle of a wall face as it does at a door edge.  A macro's
overshoot is always adjacent to the obstacle it came from, so "how far is this
from an obstacle" cannot separate the two cases.  What separates them is whether
the overshoot *narrows a passage*.

So a macro that fails the eps test gets a second chance from a *passage
persistence* test.  As the robot rotates its projected half-width sweeps the
range ``[b, a]``, and a passage admits it at half-width ``r`` exactly when free
space eroded by ``r`` stays connected.  The staircase

    r  |-->  (#components, #holes) of erode(free, r),     r in [b, a]

is therefore a signature of every passage in the scene, and its jump locations
are the scene's critical half-widths -- the translational analogue of the gate
angle.  Those radii are found once by bisection, and a macro is admissible if it
leaves the staircase unchanged on both sides of every jump:

    admissible(v)  <=>  excess thinner than eps
                   or   #comp(erode(free \\ E_plus, r)) == #comp(erode(free, r))
                        for all r in probe_radii

    probe_radii = union over critical r_i of {r_i - tol_r, r_i + tol_r}

``tol_r`` is then a knob with a physical meaning: the passage-width error the
coreset may introduce.  It is not, however, the unit anyone cares about -- the
quantity a user wants bounded is the *gate angle* error, and the conversion
between them is strongly robot dependent.  For an ellipse robot of semi-axes
``(a, b)`` clearing a gap of half-width ``r``, the legal orientation satisfies
``r = sqrt(a^2 sin^2 t + b^2 cos^2 t)``, so

    dtheta/dr = r / (sin t cos t (a^2 - b^2)),   sin^2 t = (r^2-b^2)/(a^2-b^2)

which is 3.35 rad/m for a=0.50,b=0.20 at r=0.30 but 15.45 rad/m for the nearly
circular a=0.35,b=0.28 -- a 4.6x difference on the *same* scene.  Setting one
``tol_r`` for both therefore leaves the near-circular robot 4.6x worse: measured
gate errors were -0.0000 and -0.0794 respectively at ``tol_r = 0.004``.  So
``target_gate_tol`` is the primary knob and ``tol_r`` is derived from it at the
critical radii the bisection actually found.  With that conversion all three
test robots reach exact gate accuracy at the same 60 primitives.

The signature must count holes, not only components.  With two parallel doors
through one wall, sealing either door leaves free space connected -- the other
door still joins the two sides -- so a component count alone cannot see it.  What
changes is the first Betti number: two routes make the free space an annulus, and
closing one makes it simply connected.  Measured on a wide/narrow two-door scene,
a components-only signature let the coarsener seal the wide door while reporting
no topology change at all.  Counting holes is also what makes this criterion see
*route classes*, which the components-only version silently discarded.

Two earlier versions of this test were wrong in instructive ways.  A single
scale ``r = b`` checks the robot's most favourable orientation only; on the
single-door toy it accepted merges narrowing a 0.60 door to ~0.55 -- open at
half-width 0.20, closed at theta=0.45 where the projected half-width is 0.282.
Replacing it with a uniform grid over ``[b, a]`` made accuracy depend on whether
the grid happened to straddle the critical radius: 4 radii scored better than 8
purely because ``linspace(0.2, 0.5, 4)`` contains 0.30, the door half-width, and
``linspace(0.2, 0.5, 8)`` steps over it.  Locating the critical radii instead of
guessing them removes that dependence.

An earlier version of this rule tested instead whether free space eroded by the
robot's *minimum* half-width kept its component count.  That is the wrong
instrument: it preserves topology at the robot's most favourable orientation
only.  On the single-door toy it accepted merges that narrowed a 0.60 door to
~0.55 -- still open for the 0.20 half-width it tested, but closed at theta=0.45
where the robot's projected half-width is 0.282.  The certified gate angle
collapsed from 0.510 to below 0.45.  Bounding clearance loss is the criterion
that actually controls orientation-dependent gate structure; the component count
is kept only as a cheap global guard.

This is a heuristic and is deliberately allowed to be one: soundness is not its
job.  The macro sandwich (C1) makes coarsening monotone, so the worst a bad cut
can do is turn a decidable query into ``UNKNOWN``.  Only *quality* is at stake
here, never correctness.
"""
from dataclasses import dataclass

import numpy as np
from shapely.geometry import MultiPolygon, Polygon
from shapely.ops import unary_union
from shapely import STRtree

from ..types import GaussianSupport2D, RobotModel2D, SceneModel2D
from .hierarchy import HierarchyNode, build_hierarchy
from .macro import MacroEllipse, ellipse_polygon, outer_ellipse

# Vertex counts for the shapely admissibility test only.  They never enter a
# certificate; the macro containment proof lives in macro.certify_*.
OBSTACLE_VERTICES = 48
MACRO_VERTICES = 96


@dataclass(frozen=True)
class CoarsenParams:
    """Knobs for the hand-crafted coarsener.

    ``clearance_tol``:     eps above.  The Pareto knob: it upper-bounds the
                           clearance the coreset may destroy near tight geometry.
    ``target_gate_tol``:   the gate-angle error budget in radians.  When set
                           (the default), tol_r is derived from it per robot at
                           each discovered critical radius.
    ``passage_tol``:       explicit tol_r, used when ``target_gate_tol`` is
                           None or the robot is not a single ellipse.  0
                           disables the persistence test entirely.
    ``robustness_scale``:  erosion radius for the cheap global connectivity
                           guard; defaults to the robot's minimum semi-axis.
    ``max_refine_rounds``: global verification/split iterations.
    ``cert_directions``:   directions used to certify each macro.  Fewer
                           directions widen the Lipschitz term and so force
                           *more* inflation -- looser primitives, never unsound
                           ones.
    ``merge_rounds``:      bottom-up agglomeration sweeps after the top-down
                           descent.  The descent alone cannot merge two groups
                           that are only jointly admissible -- on a two-door
                           wall, any node spanning the narrow door also swallows
                           the wide one, is rejected for the wide door's sake,
                           and after splitting no candidate proposes sealing the
                           narrow door alone.  The merge pass reaches those
                           partitions.  0 disables it.
    """
    clearance_tol: float = 0.01
    target_gate_tol: float | None = 0.005
    passage_tol: float = 0.001
    robustness_scale: float | None = None
    max_refine_rounds: int = 24
    merge_rounds: int = 6
    cert_directions: int = 1024
    leaf_size: int = 1


@dataclass(frozen=True)
class CoresetResult:
    scene: SceneModel2D
    macros: tuple
    cut_node_ids: tuple
    cut_members: tuple
    n_original: int
    n_macro: int
    min_slack: float
    refine_rounds: int
    robustness_scale: float
    clearance_tol: float
    critical_radii: tuple
    admissibility_tests: int

    @property
    def compression(self) -> float:
        return self.n_macro / float(self.n_original)


def _topology_signature(geom) -> tuple[int, int]:
    """``(#components, #holes)`` -- Betti numbers 0 and 1 of a planar region.

    Holes are essential: they are what distinguishes "two routes through this
    wall" from "one route", and a components-only signature is blind to a
    coarsener sealing one of a pair of parallel doors.
    """
    if geom.is_empty:
        return (0, 0)
    if isinstance(geom, Polygon):
        return (1, len(geom.interiors))
    if isinstance(geom, MultiPolygon):
        return (len(geom.geoms), sum(len(g.interiors) for g in geom.geoms))
    parts = [g for g in getattr(geom, "geoms", [])
             if isinstance(g, Polygon) and g.area > 0.0]
    return (len(parts), sum(len(g.interiors) for g in parts))


def robot_min_half_width(robot: RobotModel2D) -> float:
    """Smallest half-width of the robot over all body supports.

    A passage narrower than twice this cannot admit the robot at any
    orientation.
    """
    values = []
    for s in robot.supports:
        eig = np.linalg.eigvalsh(s.covariance) * (s.level ** 2)
        values.append(float(np.sqrt(max(eig[0], 0.0))))
    return min(values)


def robot_max_half_width(robot: RobotModel2D) -> float:
    """Largest half-width over all body supports (the semi-major axis).

    A corridor of width ``2a`` admits the robot at *every* orientation, so
    clearance above this threshold cannot carry gate structure.
    """
    values = []
    for s in robot.supports:
        eig = np.linalg.eigvalsh(s.covariance) * (s.level ** 2)
        values.append(float(np.sqrt(max(eig[1], 0.0))
                            + np.linalg.norm(np.asarray(s.mean, float))))
    return max(values)


def robot_footprint_area(robot: RobotModel2D) -> float:
    total = 0.0
    for s in robot.supports:
        eig = np.linalg.eigvalsh(s.covariance) * (s.level ** 2)
        semi = np.sqrt(np.maximum(eig, 0.0))
        total += float(np.pi * semi[0] * semi[1])
    return total


def gate_sensitivity(robot: RobotModel2D, r: float) -> float | None:
    """``dtheta/dr`` for a single-ellipse robot clearing a gap of half-width r.

    Returns None when the robot is not a single centred ellipse or when ``r``
    is outside ``(b, a)``, where the gate is either always open or always shut
    and the derivative carries no information.
    """
    if len(robot.supports) != 1:
        return None
    only = robot.supports[0]
    if float(np.linalg.norm(np.asarray(only.mean, dtype=float))) > 0.0:
        return None
    eig = np.linalg.eigvalsh(only.covariance) * (only.level ** 2)
    b, a = float(np.sqrt(max(eig[0], 0.0))), float(np.sqrt(max(eig[1], 0.0)))
    if not (b < r < a):
        return None
    s2 = (r * r - b * b) / (a * a - b * b)
    sin_t, cos_t = np.sqrt(s2), np.sqrt(max(1.0 - s2, 0.0))
    denom = sin_t * cos_t * (a * a - b * b)
    if denom <= 0.0:
        return None
    return float(r / denom)


def _critical_radii(free, lo: float, hi: float, tol: float,
                    max_depth: int = 14) -> list[float]:
    """Erosion radii in ``[lo, hi]`` where the free-space topology signature jumps.

    Recursive bisection.  A bracket whose endpoints agree is assumed to contain
    no jump; that can miss a pair of jumps that cancel, which costs compression
    (an admissible macro is rejected) but never accuracy, because a missed
    probe only ever makes the test easier to fail via the surviving probes.
    """
    cache: dict[float, tuple[int, int]] = {}

    def count_at(r: float) -> tuple[int, int]:
        hit = cache.get(r)
        if hit is None:
            hit = _topology_signature(free.buffer(-r))
            cache[r] = hit
        return hit

    out: list[float] = []

    def recurse(a: float, b: float, depth: int):
        ca, cb = count_at(a), count_at(b)
        if ca == cb:
            return
        if b - a <= tol or depth >= max_depth:
            out.append(0.5 * (a + b))
            return
        mid = 0.5 * (a + b)
        recurse(a, mid, depth + 1)
        recurse(mid, b, depth + 1)

    recurse(lo, hi, 0)
    return sorted(out)


def _robust_free(workspace: Polygon, obstacles, rho: float):
    free = workspace.difference(unary_union(obstacles)) if obstacles \
        else workspace
    return free, (free.buffer(-rho) if rho > 0.0 else free)


def coarsen_scene(scene: SceneModel2D, robot: RobotModel2D,
                  params: CoarsenParams | None = None,
                  hierarchy: HierarchyNode | None = None) -> CoresetResult:
    """Compress ``scene`` into the coarsest representation that preserves the
    robot-robust connectivity of free space."""
    params = params or CoarsenParams()
    root = hierarchy or build_hierarchy(scene, leaf_size=params.leaf_size)
    rho = params.robustness_scale
    if rho is None:
        rho = robot_min_half_width(robot)
    if rho <= 0.0:
        raise ValueError("robustness_scale must be positive")
    eps = float(params.clearance_tol)
    if eps <= 0.0:
        raise ValueError("clearance_tol must be positive")

    leaf_polys = [ellipse_polygon(s, OBSTACLE_VERTICES, outer=False)
                  for s in scene.supports]
    free_ref, robust_ref = _robust_free(scene.workspace, leaf_polys, rho)
    target_signature = _topology_signature(robust_ref)
    obstacle_union = unary_union(leaf_polys)
    a_max = robot_max_half_width(robot)
    tol_r = float(params.passage_tol)
    if tol_r > 0.0 and a_max > rho:
        # Locate the critical radii finely first, then choose the probe offset
        # per radius from the robot's own gate sensitivity there.
        critical = _critical_radii(free_ref, rho, a_max, min(tol_r, 1e-4))
        probes_set = {rho, a_max}
        for r in critical:
            width = tol_r
            if params.target_gate_tol is not None:
                sens = gate_sensitivity(robot, r)
                if sens is not None and sens > 0.0:
                    width = float(params.target_gate_tol) / sens
            width = max(width, 1e-6)
            probes_set.add(max(rho, r - width))
            probes_set.add(min(a_max, r + width))
        probes = sorted(probes_set)
    else:
        critical, probes = [], []
    probe_ref = [_topology_signature(free_ref.buffer(-float(r))) for r in probes]
    # Only obstacles overlapping the macro can absorb its overshoot, so union
    # a local neighbourhood per test instead of differencing against all of
    # them.  Same answer, and it keeps the descent linear in scene size.
    leaf_index = STRtree(leaf_polys)

    tests = [0]
    macro_cache: dict[int, MacroEllipse] = {}

    def macro_for(node: HierarchyNode) -> MacroEllipse:
        hit = macro_cache.get(node.node_id)
        if hit is None:
            members = [scene.supports[i] for i in node.members]
            hit = outer_ellipse(members, primitive_id=node.node_id,
                                n_dirs=params.cert_directions)
            macro_cache[node.node_id] = hit
        return hit

    def admissible(macro: MacroEllipse) -> bool:
        """Intrusion depth of the macro into free space must stay below eps."""
        tests[0] += 1
        poly = ellipse_polygon(macro.support, MACRO_VERTICES, outer=True)
        clipped = poly.intersection(scene.workspace)
        if clipped.is_empty:
            return True
        near = leaf_index.query(clipped)
        local = unary_union([leaf_polys[i] for i in near]) if len(near) else None
        excess = clipped.difference(local) if local is not None else clipped
        if excess.is_empty:
            return True
        if excess.buffer(-eps).is_empty:
            return True
        # Second chance: a large overshoot is still harmless if it leaves the
        # passage-persistence staircase intact.
        if not probes:
            return False
        reduced = free_ref.difference(poly)
        for r, want in zip(probes, probe_ref):
            if _topology_signature(reduced.buffer(-float(r))) != want:
                return False
        return True

    def descend(node: HierarchyNode, out: list):
        macro = macro_for(node)
        if node.is_leaf or admissible(macro):
            out.append((node, macro))
            return
        for child in node.children:
            descend(child, out)

    selected: list = []
    descend(root, selected)

    # Bottom-up agglomeration.  Groups here are a partition of the leaves but no
    # longer necessarily a cut of the hierarchy: merging two non-sibling groups
    # deliberately relaxes design revision 4.3.  Nothing in the soundness
    # argument depends on the partition coming from the tree -- C1 certifies
    # each macro against its own members whatever they are -- and the leaf
    # partition remains the fallback.  A learned coarsener restricted to tree
    # cuts can still be trained against these groups by projecting back.
    groups = [(tuple(n.members), m) for n, m in selected]
    next_pid = [max((n.node_id for n, _ in selected), default=0) + 1]

    # A pair that failed stays failed until one of its two groups changes, so
    # remember rejections.  Without this the sweep rescans every pair after each
    # successful merge and the pass is O(merges * k^2) -- on the door scene that
    # is ~6e5 macro constructions and it dominates everything else.
    rejected: set[tuple] = set()

    def try_merge(max_merges: int) -> None:
        for _ in range(max_merges):
            merged = False
            for i in range(len(groups)):
                for j in range(i + 1, len(groups)):
                    key = (groups[i][0], groups[j][0])
                    if key in rejected:
                        continue
                    mi, mj = groups[i][1].support, groups[j][1].support
                    gap = float(np.linalg.norm(mi.mean - mj.mean))
                    if gap > 1.5 * (mi.bounding_radius()
                                    + mj.bounding_radius()):
                        rejected.add(key)
                        continue
                    members = tuple(sorted(set(groups[i][0])
                                           | set(groups[j][0])))
                    cand = outer_ellipse([scene.supports[t] for t in members],
                                         primitive_id=next_pid[0],
                                         n_dirs=params.cert_directions)
                    if not admissible(cand):
                        rejected.add(key)
                        continue
                    next_pid[0] += 1
                    groups[i] = (members, cand)
                    groups.pop(j)
                    merged = True
                    break
                if merged:
                    break
            if not merged:
                return

    if params.merge_rounds > 0:
        try_merge(params.merge_rounds * max(1, len(groups)))
    selected = [(None, m) for _, m in groups]
    selected_members = [g for g, _ in groups]

    # Global verification: independent per-node tests can miss the cumulative
    # effect of several accepted merges.  Re-check the assembled coreset and
    # split the worst offender until the robust connectivity matches.
    rounds = 0
    while rounds < params.max_refine_rounds:
        polys = [ellipse_polygon(m.support, MACRO_VERTICES, outer=True)
                 for _, m in selected]
        _, robust = _robust_free(scene.workspace, polys, rho)
        if _topology_signature(robust) == target_signature:
            break
        splittable = [k for k, g in enumerate(selected_members) if len(g) > 1]
        if not splittable:
            break
        worst = max(splittable, key=lambda k: selected[k][1].excess_area)
        members = selected_members.pop(worst)
        selected.pop(worst)
        # Split spatially, not by index.  After the merge pass a group can hold
        # supports whose ids are unrelated to their positions, and halving the
        # id-sorted list would then cut the group into two interleaved clouds
        # whose macro ellipses both still span the original extent -- a split
        # that costs a primitive and buys no accuracy.
        means = np.array([scene.supports[t].mean for t in members],
                         dtype=np.float64)
        spread = means.max(axis=0) - means.min(axis=0)
        axis = int(np.argmax(spread))
        order = sorted(members,
                       key=lambda t: (scene.supports[t].mean[axis],
                                      scene.supports[t].mean[1 - axis], t))
        half = len(order) // 2
        halves = [tuple(sorted(order[:half])), tuple(sorted(order[half:]))]
        for off, part in enumerate(halves):
            macro = outer_ellipse([scene.supports[t] for t in part],
                                  primitive_id=next_pid[0],
                                  n_dirs=params.cert_directions)
            next_pid[0] += 1
            selected_members.insert(worst + off, part)
            selected.insert(worst + off, (None, macro))
        rounds += 1

    supports = tuple(
        GaussianSupport2D(m.support.mean, m.support.covariance,
                          m.support.level, i)
        for i, (_, m) in enumerate(selected))
    coarse = SceneModel2D(supports=supports, workspace=scene.workspace,
                          name=f"{scene.name}_coreset{len(supports)}")
    return CoresetResult(
        scene=coarse,
        macros=tuple(m for _, m in selected),
        cut_node_ids=tuple(m.support.primitive_id for _, m in selected),
        cut_members=tuple(selected_members),
        n_original=len(scene.supports),
        n_macro=len(supports),
        min_slack=min((m.slack for _, m in selected), default=float("inf")),
        refine_rounds=rounds,
        robustness_scale=float(rho),
        clearance_tol=eps,
        critical_radii=tuple(critical),
        admissibility_tests=tests[0],
    )


def uniform_cut(scene: SceneModel2D, k: int,
                hierarchy: HierarchyNode | None = None,
                leaf_size: int = 1) -> CoresetResult:
    """Geometry-only control: split the widest node until ``k`` groups remain.

    Same hierarchy, same certified macro construction, no robot conditioning
    and no clearance test.  This is the baseline the learned/heuristic
    coarsener must beat (design revision 14.2), and it isolates how much of any
    win comes from *conditioning* rather than from merging per se.
    """
    root = hierarchy or build_hierarchy(scene, leaf_size=leaf_size)
    frontier = [root]
    while len(frontier) < k:
        candidates = [i for i, n in enumerate(frontier) if not n.is_leaf]
        if not candidates:
            break
        worst = max(candidates, key=lambda i: (frontier[i].radius,
                                               frontier[i].size))
        node = frontier.pop(worst)
        frontier[worst:worst] = list(node.children)
    macros = [outer_ellipse([scene.supports[i] for i in n.members],
                            primitive_id=n.node_id) for n in frontier]
    supports = tuple(GaussianSupport2D(m.support.mean, m.support.covariance,
                                       m.support.level, i)
                     for i, m in enumerate(macros))
    coarse = SceneModel2D(supports=supports, workspace=scene.workspace,
                          name=f"{scene.name}_uniform{len(supports)}")
    return CoresetResult(
        scene=coarse, macros=tuple(macros),
        cut_node_ids=tuple(n.node_id for n in frontier),
        cut_members=tuple(n.members for n in frontier),
        n_original=len(scene.supports), n_macro=len(supports),
        min_slack=min((m.slack for m in macros), default=float("inf")),
        refine_rounds=0, robustness_scale=0.0, clearance_tol=0.0,
        critical_radii=(), admissibility_tests=0)
