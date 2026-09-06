"""Robot-conditioned Gaussian mobility coreset (SPLATC v2 design revision).

The coreset layer sits *before* the certified compiler and only ever changes
the scene's representation resolution, never its semantics:

    scene supports -> hierarchy -> legal cut -> macro supports -> compiler

The compiler is consumed unchanged.  A coarsened scene is emitted as an
ordinary :class:`~gmc.types.SceneModel2D`, so every downstream stage (BVH,
pair oracle, slice compiler, slabs, mobility graph, query, verify) runs
without modification.

Soundness contract (design revision C1/C2):
  outer coreset:  union(members) subset E_plus   -> obstacles only grow
                  -> free space only shrinks     -> SAFE stays sound
  inner coreset:  E_minus subset union(members)  -> obstacles only shrink
                  -> possible space only grows   -> UNREACHABLE stays sound

Coarsening can therefore only convert a true verdict into UNKNOWN.  It can
never manufacture a false SAFE or a false UNREACHABLE.
"""
from .hierarchy import HierarchyNode, build_hierarchy, iter_nodes
from .macro import MacroEllipse, outer_ellipse, certify_outer_containment
from .coarsen import (CoresetResult, CoarsenParams, coarsen_scene,
                       uniform_cut)
from .inner import InnerMacro, inner_ellipse, inner_scene

__all__ = [
    "HierarchyNode", "build_hierarchy", "iter_nodes",
    "MacroEllipse", "outer_ellipse", "certify_outer_containment",
    "CoresetResult", "CoarsenParams", "coarsen_scene", "uniform_cut",
    "InnerMacro", "inner_ellipse", "inner_scene",
]
