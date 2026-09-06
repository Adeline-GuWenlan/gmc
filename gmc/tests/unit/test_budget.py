import numpy as np
import pytest
from concurrent.futures import ThreadPoolExecutor

from gmc.budget import BudgetExceeded, WorkLedger
from gmc.geometry.c_obstacle import scene_pose_collides
from gmc.geometry.support import PairOracle
from gmc.io.robot_io import ellipse_robot
from gmc.types import GaussianSupport2D, PairID, SceneModel2D
import shapely


def _support(pid, x=0.0):
    return GaussianSupport2D(np.array([x, 0.0]), np.eye(2) * 0.04,
                             1.0, pid)


def test_support_budget_is_atomic_and_batch_counts_scalar_evals():
    ledger = WorkLedger(limits={"support_value_evals": 3})
    oracle = PairOracle(PairID(0, 0), _support(0), _support(0),
                        ledger=ledger)
    U = np.array([[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0]])

    oracle.support_values(0.0, U)
    assert ledger.total("support_value_evals") == 3
    assert oracle.value_calls == 3
    with pytest.raises(BudgetExceeded):
        oracle.support_values(0.0, U[:1])
    assert ledger.total("support_value_evals") == 3
    assert oracle.value_calls == 3


def test_phases_and_oracle_currencies_stay_separate():
    ledger = WorkLedger()
    oracle = PairOracle(PairID(0, 0), _support(0), _support(0),
                        ledger=ledger)
    with ledger.phase("compile"):
        oracle.support_values(0.0, np.array([[1.0, 0.0]]))
    with ledger.phase("verification"):
        oracle.support_points(0.0, np.array([[1.0, 0.0], [0.0, 1.0]]))

    snap = ledger.snapshot()
    assert snap["by_phase"]["compile"]["support_value_evals"] == 1
    assert snap["by_phase"]["verification"]["support_point_evals"] == 2
    assert snap["totals"]["support_value_evals"] == 1
    assert snap["totals"]["support_point_evals"] == 2


def test_scene_collision_counts_pose_and_actual_early_exit_pairs():
    ledger = WorkLedger()
    scene = SceneModel2D((_support(0, 0.0), _support(1, 5.0)),
                         shapely.box(-10, -2, 10, 2))
    robot = ellipse_robot(0.2, 0.2)

    assert scene_pose_collides(scene, robot, np.array([0.0, 0.0]), 0.0,
                               ledger=ledger)
    assert ledger.total("pose_collision_queries") == 1
    assert ledger.total("pair_collision_evals") == 1


def test_hard_cap_check_and_increment_are_atomic_across_threads():
    ledger = WorkLedger(limits={"support_value_evals": 50})

    def attempt(_):
        try:
            ledger.charge("support_value_evals")
            return True
        except BudgetExceeded:
            return False

    with ThreadPoolExecutor(max_workers=8) as pool:
        accepted = list(pool.map(attempt, range(200)))
    assert sum(accepted) == 50
    assert ledger.total("support_value_evals") == 50


def test_phase_context_is_isolated_between_threads():
    ledger = WorkLedger()

    def charge_phase(name):
        with ledger.phase(name):
            for _ in range(10):
                ledger.charge("intersection_tests")

    with ThreadPoolExecutor(max_workers=2) as pool:
        list(pool.map(charge_phase, ("left", "right")))
    snapshot = ledger.snapshot()
    assert snapshot["by_phase"]["left"]["intersection_tests"] == 10
    assert snapshot["by_phase"]["right"]["intersection_tests"] == 10


def test_pair_oracle_fourth_positional_argument_remains_calls():
    oracle = PairOracle(PairID(0, 0), _support(0), _support(0), 7)
    assert oracle.calls == 7
    assert oracle.revision == 0


def test_topology_currencies_are_independent_and_visible_in_snapshots():
    ledger = WorkLedger(limits={
        "union_operations": 2,
        "difference_operations": 1,
        "spatial_index_builds": 1,
        "spatial_index_queries": 1,
    })
    ledger.charge("union_operations", 2)
    ledger.charge("difference_operations")
    ledger.charge("spatial_index_builds")
    ledger.charge("spatial_index_queries")

    snapshot = ledger.snapshot()
    assert snapshot["totals"]["union_operations"] == 2
    assert snapshot["totals"]["difference_operations"] == 1
    assert snapshot["totals"]["spatial_index_builds"] == 1
    assert snapshot["totals"]["spatial_index_queries"] == 1
    assert snapshot["totals"]["intersection_tests"] == 0

    with pytest.raises(BudgetExceeded):
        ledger.charge("union_operations")
    assert ledger.total("union_operations") == 2
