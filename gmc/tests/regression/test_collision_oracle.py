"""Independent ellipse ground truth must fail closed on ill-conditioned SPD."""
import numpy as np

from gmc.geometry.c_obstacle import body_world, ellipses_collide, pose_collides
from gmc.geometry.support import PairOracle
from gmc.types import GaussianSupport2D, PairID


def test_ill_conditioned_deep_collision_is_not_solver_failure_free_space():
    c1 = np.array([1.6622182016901923, 0.7946154923673476])
    Q1 = np.array([
        [0.07957502614115981, 0.7928437882492885],
        [0.7928437882492885, 7.899479325951508],
    ])
    l1 = 2.6372754291588003
    c2 = np.array([0.18114942163414738, 2.297421263622958])
    Q2 = np.array([
        [0.8560596213475984, 0.08941904027751842],
        [0.08941904027751843, 0.009340196324032358],
    ])
    l2 = 2.454962443323366
    assert np.linalg.eigvalsh(Q1)[0] > 1e-10
    assert np.linalg.eigvalsh(Q2)[0] > 1e-10
    assert ellipses_collide(c1, Q1, l1, c2, Q2, l2)


def test_high_condition_analytic_contacts_and_sided_offsets():
    scene = GaussianSupport2D(
        np.array([1.6622182016901923, 0.7946154923673476]),
        np.array([[0.07957502614115981, 0.7928437882492885],
                  [0.7928437882492885, 7.899479325951508]]),
        2.6372754291588003, 0,
    )
    body = GaussianSupport2D(
        np.array([-0.8305473149190914, 1.2851183937428448]),
        np.array([[0.022990143530407233, 0.13916579736271195],
                  [0.13916579736271195, 0.8424096741412235]]),
        2.454962443323366, 1,
    )
    theta = -1.3029985403523714
    oracle = PairOracle(PairID(0, 1), scene, body)
    angles = np.linspace(0.0, 2.0 * np.pi, 256, endpoint=False)
    directions = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    contacts = oracle.support_points(theta, directions)
    for direction, contact in zip(directions, contacts):
        center, covariance, level = body_world(body, contact, theta)
        assert ellipses_collide(
            scene.mean, scene.covariance, scene.level,
            center, covariance, level,
        )
        center_in, _, _ = body_world(
            body, contact - 1e-10 * direction, theta)
        assert ellipses_collide(
            scene.mean, scene.covariance, scene.level,
            center_in, covariance, level,
        )
        center_out, _, _ = body_world(
            body, contact + 1e-10 * direction, theta)
        assert not ellipses_collide(
            scene.mean, scene.covariance, scene.level,
            center_out, covariance, level,
        )


def test_pose_collision_oracle_is_large_translation_equivariant():
    scene = GaussianSupport2D(
        np.array([100000000000001.5, -36999999999999.96]),
        np.array([[0.007072179650791499, -0.045338341869854304],
                  [-0.045338341869854304, 0.2997922083545874]]),
        1.2402931244920603, 0,
    )
    body = GaussianSupport2D(
        np.array([-0.5307777604318488, -1.2299328564105472]),
        np.array([[0.04536384809681399, 0.0713476769633011],
                  [0.0713476769633011, 0.18297345884517036]]),
        0.9392729656339766, 0,
    )
    theta = 15.75610582123733
    world_t = np.array([100000000000001.14, -37000000000000.32])
    local_t = np.asarray(
        np.asarray(world_t, dtype=np.longdouble)
        - np.asarray(scene.mean, dtype=np.longdouble),
        dtype=float,
    )
    local_scene = GaussianSupport2D(
        np.zeros(2), scene.covariance, scene.level, scene.primitive_id,
    )

    # The old world-centre add-then-subtract path returned True here.  The
    # scene-local and translated formulations must be identical and this pose
    # is strictly separated.
    assert not pose_collides(local_scene, body, local_t, theta)
    assert pose_collides(scene, body, world_t, theta) == \
        pose_collides(local_scene, body, local_t, theta)
