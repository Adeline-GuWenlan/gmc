"""Gate 0 analytic collision/contact cases (problem_spec.md §3, plan 00 §6).

Run:  cd splatc_atlas && PYTHONPATH=src python -m pytest tests -q
"""
import numpy as np
import pytest

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from splatc.gaussian_geometry.contact import (
    pw_ellipse_disc, pw_ellipse_ellipse, point_ellipse_distance,
    checker2_signed, support_half_widths)
from splatc.gaussian_geometry.primitives import Robot, SceneGeometry
from splatc.common.se2 import world_to_body, rot
from splatc.datasets.g1_gate import (
    make_g1_scene, robot_library, gate_half_angle, projection_radius)

RNG = np.random.default_rng(20260811)
TOL = 1e-9


def _h_disc(u1, u2, a, b, R):
    return float(pw_ellipse_disc(np.array([u1]), np.array([u2]), a, b, R)[0])


# 1. circle-circle closed form: h = d/(r1+r2) - 1
def test_circle_circle_closed_form():
    for d, r1, r2 in [(3.0, 1.0, 0.5), (1.5, 1.0, 0.5), (0.2, 0.3, 0.4)]:
        h = _h_disc(d, 0.0, r1, r1, r2)
        assert abs(h - (d / (r1 + r2) - 1.0)) < 1e-7


# 2. circle-circle exact tangency -> h = 0
def test_circle_circle_tangency():
    assert abs(_h_disc(1.5, 0.0, 1.0, 1.0, 0.5)) < 1e-7


# 3. ellipse-disc tangency along the major axis: d = a + R
def test_ellipse_disc_tangency_major():
    a, b, R = 0.6, 0.25, 0.1
    assert abs(_h_disc(a + R, 0.0, a, b, R)) < 1e-7
    assert _h_disc(a + R + 0.01, 0.0, a, b, R) > 0
    assert _h_disc(a + R - 0.01, 0.0, a, b, R) < 0


# 4. ellipse-disc tangency along the minor axis: d = b + R
def test_ellipse_disc_tangency_minor():
    a, b, R = 0.6, 0.25, 0.1
    assert abs(_h_disc(0.0, b + R, a, b, R)) < 1e-7
    assert _h_disc(0.0, b + R + 0.01, a, b, R) > 0
    assert _h_disc(0.0, b + R - 0.01, a, b, R) < 0


# 5. PW sign agrees with independent checker #2 on random configurations
def test_pw_vs_checker2_sign():
    a, b = 0.6, 0.25
    n = 4000
    u1 = RNG.uniform(-1.2, 1.2, n)
    u2 = RNG.uniform(-1.2, 1.2, n)
    R = RNG.uniform(0.05, 0.4, n)
    h1 = np.array([_h_disc(x, y, a, b, r) for x, y, r in zip(u1, u2, R)])
    h2 = checker2_signed(u1, u2, a, b, R)
    # exclude near-tangent points where both are within numerical noise of 0
    definite = np.abs(h2) > 1e-6
    assert np.all(np.sign(h1[definite]) == np.sign(h2[definite]))


# 6. general ellipse-ellipse PW reduces to the disc fast path
def test_pw_general_matches_disc_path():
    a, b, R = 0.6, 0.25, 0.3
    for _ in range(50):
        u = RNG.uniform(-1.5, 1.5, 2)
        Ainv = np.diag([a * a, b * b])
        Binv = np.eye(2) * R * R
        hg = float(pw_ellipse_ellipse(u[0], u[1], Ainv, Binv))
        hd = _h_disc(u[0], u[1], a, b, R)
        assert abs(hg - hd) < 1e-7


# 7. rigid-transform equivariance: rotating scene + robot together leaves h fixed
def test_rigid_equivariance():
    a, b, R = 0.6, 0.25, 0.15
    for _ in range(50):
        c = RNG.uniform(-1.0, 1.0, 2)
        theta = RNG.uniform(0, 2 * np.pi)
        phi = RNG.uniform(0, 2 * np.pi)  # global rotation
        u1, u2 = world_to_body(theta, c[0], c[1])
        h_ref = _h_disc(u1, u2, a, b, R)
        c2 = rot(phi) @ c
        v1, v2 = world_to_body(theta + phi, c2[0], c2[1])
        h_rot = _h_disc(v1, v2, a, b, R)
        assert abs(h_ref - h_rot) < 1e-9


# 8. theta periodicity + centro-symmetry: h(theta) = h(theta + pi) = h(theta + 2pi)
def test_theta_periodicity_symmetry():
    a, b, R = 0.6, 0.25, 0.2
    for _ in range(30):
        c = RNG.uniform(-1.0, 1.0, 2)
        theta = RNG.uniform(0, 2 * np.pi)
        hs = []
        for t in (theta, theta + np.pi, theta + 2 * np.pi):
            u1, u2 = world_to_body(t, c[0], c[1])
            hs.append(_h_disc(u1, u2, a, b, R))
        assert abs(hs[0] - hs[1]) < 1e-9 and abs(hs[0] - hs[2]) < 1e-9


# 9. point-agent limit: shrinking robot -> free iff point outside disc
def test_point_agent_limit():
    R = 0.5
    eps = 1e-4
    for d in (0.4, 0.499, 0.501, 0.7):
        h = _h_disc(d, 0.0, eps, eps, R)
        assert (h > 0) == (d > R + eps)


# 10. body-inclusion monotonicity: small circle (r=b) inside long ellipse
def test_body_inclusion_monotonicity():
    a, b = 0.6, 0.25
    n = 2000
    u1 = RNG.uniform(-1.0, 1.0, n)
    u2 = RNG.uniform(-1.0, 1.0, n)
    R = np.full(n, 0.12)
    h_small = np.hypot(u1, u2) / (b + 0.12) - 1.0     # circle r=b closed form
    h_long = np.array([_h_disc(x, y, a, b, r) for x, y, r in zip(u1, u2, R)])
    collide_small = h_small < -1e-9
    collide_long = h_long < 1e-9
    assert np.all(collide_long[collide_small])  # small collides => long collides


# 11. support projection formula vs sampled footprint extent
def test_support_projection_vs_sampling():
    a, b = 0.6, 0.25
    phis = np.linspace(0, 2 * np.pi, 20000, endpoint=False)
    bx, by = a * np.cos(phis), b * np.sin(phis)
    for theta in (0.0, 0.3, np.pi / 4, 1.2, np.pi / 2):
        c, s = np.cos(theta), np.sin(theta)
        wx = np.max(np.abs(c * bx - s * by))
        wy = np.max(np.abs(s * bx + c * by))
        px, py = support_half_widths(a, b, theta)
        assert abs(wx - px) < 1e-6 and abs(wy - py) < 1e-6


# 12. projection radius r(theta) endpoints (gate formula ingredients)
def test_projection_radius_endpoints():
    a, b = 0.6, 0.25
    assert abs(projection_radius(a, b, 0.0) - b) < TOL
    assert abs(projection_radius(a, b, np.pi / 2) - a) < TOL


# 13. G1 scene: mid-corridor free condition matches analytic corridor formula
def test_g1_midplane_matches_analytic():
    w = 0.7
    scene = make_g1_scene(w)
    robot = robot_library()["R_long_ellipse"]
    th_max = gate_half_angle(robot.a, robot.b, w)
    for theta, expect in [(0.0, True),
                          (th_max - 0.02, True),
                          (th_max + 0.02, False),
                          (np.pi / 2, False)]:
        ys = np.arange(-w / 2, w / 2 + 1e-9, 0.002)
        f, _ = scene.eval_points(robot, theta, np.zeros_like(ys), ys)
        assert bool(f.any()) == expect, f"theta={theta}"


# 14. G1 scene: checker #1 and checker #2 agree on random poses
def test_g1_checkers_agree_on_random_poses():
    scene = make_g1_scene(0.7)
    for robot in robot_library().values():
        for _ in range(300):
            q = (RNG.uniform(-3.5, 3.5), RNG.uniform(-2.3, 2.3),
                 RNG.uniform(0, 2 * np.pi))
            f1, r1 = scene.check_pose(robot, q)
            f2, m2 = scene.check_pose_independent(robot, q)
            if abs(m2) > 1e-5:  # skip knife-edge tangencies
                assert f1 == f2, f"{robot.robot_id} q={q} r1={r1} m2={m2}"


# 15. workspace containment: boundary-sampled footprint agrees with margins
def test_workspace_containment():
    scene = make_g1_scene(0.7)
    robot = robot_library()["R_long_ellipse"]
    phis = np.linspace(0, 2 * np.pi, 4000, endpoint=False)
    for _ in range(100):
        q = (RNG.uniform(-3.6, 3.6), RNG.uniform(-2.4, 2.4),
             RNG.uniform(0, 2 * np.pi))
        c, s = np.cos(q[2]), np.sin(q[2])
        bx, by = robot.a * np.cos(phis), robot.b * np.sin(phis)
        X = q[0] + c * bx - s * by
        Y = q[1] + s * bx + c * by
        inside = (X.min() > -3.5) and (X.max() < 3.5) \
            and (Y.min() > -2.3) and (Y.max() < 2.3)
        px, py = support_half_widths(robot.a, robot.b, q[2])
        margin_ok = (q[0] - px > -3.5) and (q[0] + px < 3.5) \
            and (q[1] - py > -2.3) and (q[1] + py < 2.3)
        if inside != margin_ok:  # only disagree exactly on the boundary
            assert min(abs(X.min() + 3.5), abs(3.5 - X.max()),
                       abs(Y.min() + 2.3), abs(2.3 - Y.max())) < 1e-3


# 16. de-aligned G1: tilted/offset door matches the rotated analytic formula
def test_g1_dealigned_matches_rotated_analytic():
    w, tilt, dy0 = 0.7, np.radians(11.0), 0.117
    scene = make_g1_scene(w, door_offset=dy0, door_tilt=tilt)
    robot = robot_library()["R_long_ellipse"]
    th_max = gate_half_angle(robot.a, robot.b, w)
    wall_dir = np.array([-np.sin(tilt), np.cos(tilt)])
    ts = np.arange(-w / 2, w / 2 + 1e-9, 0.002)
    for dth, expect in [(0.0, True), (th_max - 0.02, True),
                        (-(th_max - 0.02), True), (th_max + 0.02, False),
                        (np.pi / 2, False)]:
        theta = tilt + dth
        f, _ = scene.eval_points(robot, theta,
                                 ts * wall_dir[0], dy0 + ts * wall_dir[1])
        assert bool(f.any()) == expect, f"dth={np.degrees(dth):.1f}deg"


# 17. rotational equivariance at scene level: tilted-scene h equals aligned-scene
#     h at the counter-rotated pose (interior poses; workspace differs so
#     containment is excluded from the comparison)
def test_g1_tilt_equivariance_interior():
    w, tilt = 0.7, np.radians(9.0)
    aligned = make_g1_scene(w)
    tilted = make_g1_scene(w, door_tilt=tilt)
    robot = robot_library()["R_long_ellipse"]
    ct, st = np.cos(tilt), np.sin(tilt)
    for _ in range(60):
        q = (RNG.uniform(-2.0, 2.0), RNG.uniform(-1.2, 1.2),
             RNG.uniform(0, 2 * np.pi))
        _, rho_t = tilted.check_pose(robot, q)
        q0 = (ct * q[0] + st * q[1], -st * q[0] + ct * q[1], q[2] - tilt)
        _, rho_a = aligned.check_pose(robot, q0)
        if min(abs(rho_t), abs(rho_a)) < 0.2:  # inside the exact shell
            assert abs(rho_t - rho_a) < 1e-9, f"q={q}"


# 18. Gaussian overlap c_ij: analytic gradients match finite differences
def test_overlap_gradients_fd():
    from splatc.gaussian_geometry.overlap import overlap, overlap_grad
    robot = robot_library()["R_long_ellipse"]
    S = np.diag([0.01, 0.04])
    for _ in range(20):
        mu = RNG.uniform(-1, 1, 2)
        q = (RNG.uniform(-1, 1), RNG.uniform(-1, 1), RNG.uniform(0, 2 * np.pi))
        c, gx, gy, gth = overlap_grad(S, mu, robot, q)
        h = 1e-6
        fdx = (overlap(S, mu, robot, (q[0] + h, q[1], q[2]))
               - overlap(S, mu, robot, (q[0] - h, q[1], q[2]))) / (2 * h)
        fdy = (overlap(S, mu, robot, (q[0], q[1] + h, q[2]))
               - overlap(S, mu, robot, (q[0], q[1] - h, q[2]))) / (2 * h)
        fdt = (overlap(S, mu, robot, (q[0], q[1], q[2] + h))
               - overlap(S, mu, robot, (q[0], q[1], q[2] - h))) / (2 * h)
        for g, fd in ((gx, fdx), (gy, fdy), (gth, fdt)):
            assert abs(g - fd) < 1e-5 * (1 + abs(fd)), (g, fd)


# 19. active-pair side signature: bilateral only inside the door corridor
def test_side_signature_discriminates_gate():
    w, tilt = 0.54, np.radians(7.0)
    scene = make_g1_scene(w, door_offset=0.013, door_tilt=tilt)
    robot = robot_library()["R_long_ellipse"]
    s_gate = scene.side_rho(robot, tilt, np.array([0.0]), np.array([0.013]))
    assert max(s_gate[1][0], s_gate[-1][0]) < 0.2  # both sides near: gate cue
    s_wall = scene.side_rho(robot, 0.0, np.array([-1.05]), np.array([1.2]))
    s_far = scene.side_rho(robot, 0.0, np.array([-2.0]), np.array([-1.5]))
    for s in (s_wall, s_far):
        assert max(s[1][0], s[-1][0]) >= 0.2  # unilateral / far: no gate cue


# 20. G5 remote closure: start-local rho identical, global reachability flips
def test_g5_remote_closure():
    from splatc.reference.oracle import (make_grid, dense_oracle, reachability)
    from splatc.datasets.g1_gate import Q_START, GOAL, GOAL_RADIUS
    robot = robot_library()["R_long_ellipse"]
    open_s = make_g1_scene(0.7)
    closed_s = make_g1_scene(0.7, door_plug=True)
    # start-local contact values bitwise identical (plug beyond broad phase)
    for _ in range(200):
        q = (RNG.uniform(-3.0, -1.2), RNG.uniform(-1.8, 1.8),
             RNG.uniform(0, 2 * np.pi))
        _, r_open = open_s.check_pose(robot, q)
        _, r_closed = closed_s.check_pose(robot, q)
        assert r_open == r_closed, q
    # global reachability flips (coarse oracle is sufficient at w=0.7)
    grid = make_grid(open_s.workspace, 0.10, 72)
    for scene, expect in ((open_s, True), (closed_s, False)):
        free, _ = dense_oracle(scene, robot, grid)
        reach, _, _, _ = reachability(free, grid, Q_START, GOAL, GOAL_RADIUS)
        assert reach == expect, scene.scene_id


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
