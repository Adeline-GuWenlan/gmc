"""Deterministic synthetic core fixtures (not the showcase scene generator)."""
from dataclasses import dataclass
import numpy as np

from gmc.height.ply3d import GaussianScene3D
from gmc.gs3d.contracts import BodySpec, Pose3, SceneSpec


@dataclass(frozen=True)
class KnownBox:
    lower: tuple
    upper: tuple
    holes: tuple = ()  # closed AABB holes; any intersection fails closed

    def contains_aabb(self, lower, upper):
        lo, hi = np.asarray(lower), np.asarray(upper)
        if np.any(lo < self.lower) or np.any(hi > self.upper):
            return False
        return not any(np.all(hi >= a) and np.all(lo <= b) for a, b in self.holes)


@dataclass(frozen=True)
class FlatSupport:
    z: float = 0.
    known: KnownBox | None = None

    def height(self, x, y):
        if self.known is not None and not self.known.contains_aabb((x, y, self.z), (x, y, self.z)):
            return None
        return self.z

    def height_bounds(self, lower_xy, upper_xy):
        if self.known is not None and not self.known.contains_aabb((*lower_xy, self.z), (*upper_xy, self.z)):
            return None
        return self.z, self.z

    def supports_segment(self, a, b, radius_m):
        lower = np.minimum(a[:2], b[:2]) - radius_m
        upper = np.maximum(a[:2], b[:2]) + radius_m
        return self.height_bounds(tuple(lower), tuple(upper)) is not None


UAV = BodySpec("uav", .25, .10, "uav_translation")
SWEEPER = BodySpec("sweeper", .175, .04, "ground_unicycle")
CYLINDER = BodySpec("cylinder", .30, .865, "ground_unicycle")


def make_scene(means=(), axes=(), *, rotations=None, lower=(-2., -1., 0.),
               upper=(2., 1., 3.), known=None, support=None, name="synthetic"):
    means = np.asarray(means, dtype=float).reshape(-1, 3)
    axes = np.asarray(axes, dtype=float).reshape(-1, 3)
    covs = np.array([np.diag((row / 2.) ** 2) for row in axes]).reshape(-1, 3, 3)
    if rotations is not None:
        rotations = np.asarray(rotations)
        covs = rotations @ covs @ rotations.transpose(0, 2, 1)
    gaussians = GaussianScene3D(means, covs, np.ones(len(means)), np.arange(len(means)))
    return SceneSpec(name, gaussians, lower, upper, .3, 2., known or KnownBox(lower, upper),
                     support, {"coverage_policy": "synthetic_declared_known_box", "seed": 0})


def z_distinction(z=.7):
    return make_scene([(0., 0., z)], [(.4, .95, .35)], name=f"z_distinction_{z}")


def under_over():
    # Both supports span the available lateral corridor. At x=-.7 all free
    # centres have z<.856; at x=.8 all free centres have z>1.395 (r=.25,
    # margin=.05). Thus no single fixed-z traversal can satisfy both gates.
    return make_scene([(-.7, 0., 2.), (.8, 0., .3)],
                      [(.35, 2., 1.), (.35, 2., .95)],
                      lower=(-2.2, -.5, 0.), upper=(2.2, .5, 2.8), name="synthetic_under_over"), \
        Pose3((-1.8, 0., .65)), Pose3((1.8, 0., 1.65))
