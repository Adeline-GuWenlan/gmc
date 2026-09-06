"""GMC (Gaussian Mobility Complex) — fixed-theta contact-complex layer.

Modules:
  geometry   convex kernels: ellipse polygonization, support/tangent polygons,
             exact convex Minkowski sum
  cluster    M1 conservative splat clustering -> MergedScene
  hierarchy  two-level certified slice queries with adaptive refinement
"""
from .cluster import MergedScene, bucket_indices, build_merged, major_angle
from .geometry import (circum_factor, ellipse_polys, forbidden_ellipse_polys,
                       minkowski_sum, robot_cov, rot2, support_ellipses,
                       tangent_polys, unit_dirs)
from .hierarchy import (coarse_forbidden, free_topology, gate_query,
                        hot_buckets, raw_forbidden, robot_poly)

__all__ = [
    "MergedScene", "bucket_indices", "build_merged", "major_angle",
    "circum_factor", "ellipse_polys", "forbidden_ellipse_polys",
    "minkowski_sum", "robot_cov", "rot2", "support_ellipses", "tangent_polys",
    "unit_dirs", "coarse_forbidden", "free_topology", "gate_query",
    "hot_buckets", "raw_forbidden", "robot_poly",
]
