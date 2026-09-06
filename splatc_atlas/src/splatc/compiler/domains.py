"""Cell-domain predicates for the OpenChart builder (P4a.1, round-8).

Round-8 finding: the builder's domain mask tested only the CELL CENTER,
so a cell whose center was inside the (rigidly transformed) workspace
rectangle entered a chart as a full FREE member even when its corners
stuck out of the domain — the "union of certified cells lies inside the
restricted domain" claim was unsound (114 such cells at w=1.10,
phi=30deg).  The fix is a three-way CELL predicate:

  IN     the whole xy footprint of the cell lies inside the domain
  OUT    the cell's xy footprint does not intersect the domain
  CROSS  partial overlap — the cell may never become a chart member;
         it is refined further (its fully-inside children can)

The domain is an xy region (theta is never restricted).  For a rigidly
transformed rectangle both tests are EXACT:
  IN    <=>  all four cell corners inside (the rectangle is convex)
  OUT   <=>  separating axis exists (SAT over the 4 face normals of an
             axis-aligned box vs an oriented rectangle)
"""
from __future__ import annotations

import numpy as np


class RigidRectDomain:
    """The rectangle `rect` = (xmin, xmax, ymin, ymax) mapped by the
    rigid transform p -> R(phi) p + t (same convention as
    splatc.datasets.transforms.rigid_transform_scene)."""

    def __init__(self, rect, phi=0.0, t=(0.0, 0.0)):
        self.rect = tuple(float(v) for v in rect)
        self.phi = float(phi)
        self.t = (float(t[0]), float(t[1]))

    # -- point test (exact): inverse-transform, compare to the rectangle
    def contains_point(self, x, y):
        c, s = np.cos(self.phi), np.sin(self.phi)
        xr, yr = x - self.t[0], y - self.t[1]
        x0, y0 = c * xr + s * yr, -s * xr + c * yr
        xmin, xmax, ymin, ymax = self.rect
        return bool(xmin <= x0 <= xmax and ymin <= y0 <= ymax)

    # -- cell test (exact): lo/hi are cell bounds arrays (x, y[, theta]);
    # only the xy footprint is classified
    def classify_cell(self, lo, hi):
        xmin, xmax, ymin, ymax = self.rect
        c, s = np.cos(self.phi), np.sin(self.phi)
        corners = np.array([[lo[0], lo[1]], [lo[0], hi[1]],
                            [hi[0], lo[1]], [hi[0], hi[1]]])
        inv = (corners - self.t) @ np.array([[c, -s], [s, c]])
        inside = ((inv[:, 0] >= xmin) & (inv[:, 0] <= xmax)
                  & (inv[:, 1] >= ymin) & (inv[:, 1] <= ymax))
        if inside.all():
            return "IN"
        # SAT: axis-aligned box axes (x, y) + rectangle axes (u, v)
        rc = np.array([[xmin, ymin], [xmin, ymax],
                       [xmax, ymin], [xmax, ymax]])
        rect_world = rc @ np.array([[c, s], [-s, c]]) + self.t
        for ax in (np.array([1.0, 0.0]), np.array([0.0, 1.0]),
                   np.array([c, s]), np.array([-s, c])):
            pb = corners @ ax
            pr = rect_world @ ax
            if pb.max() < pr.min() or pr.max() < pb.min():
                return "OUT"
        return "CROSS"

    # -- robot-support margin (round-10 blocker fix) ---------------------
    # Round-10 review: classify_cell constrains only the ROBOT-CENTER
    # footprint, and the collision checker's box margin uses the
    # transformed scene's AABB — nobody certified that the robot BODY
    # stays inside the true rotated workspace rectangle (measured: 4,672
    # of 11,116 member-cell centers had the ellipse sticking out at
    # w=1.10, R30, 60k).  This margin closes that hole: in the domain's
    # LOCAL frame the robot support half-widths at local orientation
    # theta - phi give four side margins; the minimum is 1-Lipschitz in
    # the certificate motion metric D = |dxy| + a_max*|dtheta| (support
    # half-widths are a_max-Lipschitz in theta), so the standard cell /
    # bubble arguments apply unchanged.
    def support_margin(self, x, y, th, robot):
        from ..gaussian_geometry.contact import support_half_widths
        c, s = np.cos(self.phi), np.sin(self.phi)
        xr, yr = x - self.t[0], y - self.t[1]
        xl, yl = c * xr + s * yr, -s * xr + c * yr
        px, py = support_half_widths(robot.a, robot.b, th - self.phi)
        xmin, xmax, ymin, ymax = self.rect
        return float(min(xl - px - xmin, xmax - (xl + px),
                         yl - py - ymin, ymax - (yl + py)))

    def to_dict(self):
        return {"kind": "rigid_rect", "rect": list(self.rect),
                "phi_rad": self.phi, "t": list(self.t)}

    def domain_hash(self):
        import hashlib
        import json
        return hashlib.sha256(json.dumps(
            self.to_dict(), sort_keys=True).encode()).hexdigest()[:12]
