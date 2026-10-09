"""The shared replay judge must not veto a route for its own conservatism or round-off.

F5 (docs/aerial3dg_failures_f5.md 2.4-2.5) traced 538 vetoes of GMC routes that the judge's geometry
otherwise accepted: REPLAY-AABB (the swept body's *world AABB* leaves a rotated route prism although
the body does not) and REPLAY-RATE (a microsecond turn timed exactly at the yaw-rate limit reads
1e-9 over it after float time accumulation). The guards below pin that nothing real is loosened.
"""
import numpy as np

from gmc.gs3d.contracts import Pose3
from gmc.gs3d.integration import RouteBoxKnownSpace
from gmc.gs3d.oracle import GaussianBodyOracle, PreparedScene
from gmc.gs3d.robots import SWEEPER, UAV
from gmc.gs3d.scene import route_frame
from gmc.gs3d.validation import verify_linear_trajectory
from gs3d_core_fixtures import make_scene

GROUND_LIMITS = {"max_speed_mps": .3, "max_vertical_speed_mps": 0., "max_yaw_rate_radps": 1.,
                 "max_acceleration_mps2": .3, "max_yaw_acceleration_radps2": 1.}


def _prism_oracle():
    """Route prism u [0, 4], v [-1, 1], z [0, 2] in a frame rotated 53 deg (|cos|+|sin| = 1.4)."""
    origin, rotation = route_frame(0.)
    known = RouteBoxKnownSpace(tuple(origin), tuple(map(tuple, rotation)), (0., -1., 0.), (4., 1., 2.))
    lower, upper = known.world_bounds()
    far = np.array([.5, -.5, .3]) @ rotation + origin          # one small Gaussian, far from the body
    scene = make_scene([far], [(.05, .05, .05)], lower=lower, upper=upper, known=known)
    return GaussianBodyOracle(PreparedScene(scene)), origin, rotation


def _pose(route_xyz, origin, rotation):
    return Pose3(tuple(map(float, np.asarray(route_xyz, float) @ rotation + origin)))


def test_body_inside_rotated_prism_is_covered_though_its_world_aabb_is_not():
    oracle, origin, rotation = _prism_oracle()
    v = 1. - UAV.radius_m - .002                               # 2 mm inside the v-max face
    a, b = _pose((1.5, 0., 1.), origin, rotation), _pose((2.5, v, 1.), origin, rotation)
    for edge in ((b, b), (a, b)):                              # a pose at the face, a diagonal swept edge
        report = oracle.edge(*edge, UAV, margin_m=.01)
        assert report.reason != "map_unknown"
        assert (report.occupancy, report.safety) == ("free", "continuous_bound")


def test_body_leaving_rotated_prism_is_still_rejected():
    oracle, origin, rotation = _prism_oracle()
    r, h = UAV.radius_m, UAV.half_height_m
    for route in ((2., 1. - r + .002, 1.), (2., -1. + r - .002, 1.), (.0 + r - .002, 0., 1.),
                  (2., 0., 2. - h + .002)):                     # 2 mm out through v-max, v-min, u-min, top
        q = _pose(route, origin, rotation)
        report = oracle.edge(q, q, UAV, margin_m=.01)
        # map_unknown, or body_exceeds_workspace_bounds where the prism face meets its world AABB
        assert report.occupancy != "free", (route, report.reason)
    inside, outside = _pose((2., 0., 1.), origin, rotation), _pose((2., 1. - r + .002, 1.), origin, rotation)
    assert oracle.edge(inside, outside, UAV, margin_m=.01).occupancy != "free"


def _ground_trajectory(times, yaws, xs=None):
    xs = xs if xs is not None else [0.] * len(times)
    return {"poses": [[x, 0., .06, yaw] for x, yaw in zip(xs, yaws)], "time_s": list(times),
            "interpolation": "linear_xyz_yaw", "segments": [], "control_dt_s": .05}


def _microsecond_turn_that_reads_over_the_limit(t0=40.):
    """A turn appended exactly as planner._linear_trajectory times it (times[-1] + turn / rate)."""
    for k in range(100_000):
        turn = 1e-6 * (1. + k * 1e-4)
        t1 = t0 + turn / GROUND_LIMITS["max_yaw_rate_radps"]
        if turn / (t1 - t0) > GROUND_LIMITS["max_yaw_rate_radps"] + 1e-9:
            return turn, t1
    raise AssertionError("no round-off case found")


def test_microsecond_turn_timed_at_the_limit_passes_despite_time_roundoff():
    turn, t1 = _microsecond_turn_that_reads_over_the_limit()
    long_leg = 40. * GROUND_LIMITS["max_speed_mps"]
    trajectory = _ground_trajectory([0., 40., t1], [0., 0., turn], xs=[0., long_leg, long_leg])
    assert verify_linear_trajectory(trajectory, SWEEPER, GROUND_LIMITS)["passed"]


def test_real_overspeed_is_still_rejected():
    turn = 1e-6                                                # same microsecond turn, 1e-6 too fast
    long_leg = 40. * GROUND_LIMITS["max_speed_mps"]
    fast_turn = _ground_trajectory([0., 40., 40. + turn / (1. + 1e-6)], [0., 0., turn],
                                   xs=[0., long_leg, long_leg])
    assert not verify_linear_trajectory(fast_turn, SWEEPER, GROUND_LIMITS)["passed"]
    fast_leg = _ground_trajectory([0., 1.], [0., 0.], xs=[0., .3 * (1. + 1e-8)])  # 1 s leg, 3e-9 m/s over
    assert not verify_linear_trajectory(fast_leg, SWEEPER, GROUND_LIMITS)["passed"]
