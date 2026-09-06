"""Exact, separately budgeted accounting for GEOS/topology work."""
from types import SimpleNamespace

import pytest
import shapely

from gmc.budget import BudgetExceeded, WorkLedger
from gmc.io.robot_io import ellipse_robot
from gmc.orientation.slab_builder import build_slabs
from gmc.spatial.bvh import candidate_pairs
from gmc.spatial.nerve import build_contact_nerve
from gmc.spatial.slice_compiler import assemble_slice, build_slice
from gmc.synth import empty_scene, triple_contact

from ..conftest import make_cfg


def _common_triple():
    return tuple(SimpleNamespace(outer=geometry) for geometry in (
        shapely.box(0.0, 0.0, 2.0, 2.0),
        shapely.box(0.5, 0.0, 2.5, 2.0),
        shapely.box(1.0, 0.0, 3.0, 2.0),
    ))


def test_nerve_counts_index_query_and_recursive_intersections():
    ledger = WorkLedger()

    nerve = build_contact_nerve(_common_triple(), ledger=ledger)

    assert frozenset({0, 1, 2}) in nerve.simplices
    assert ledger.total("spatial_index_builds") == 1
    assert ledger.total("spatial_index_queries") == 1
    # Three explicit pair-candidate predicates build the graph, two
    # intersections establish the triple, and two reconstruct its witness.
    assert ledger.total("intersection_tests") == 7


def test_nerve_index_budget_fails_before_query_and_does_not_cross_charge():
    ledger = WorkLedger(limits={"spatial_index_queries": 0})

    with pytest.raises(BudgetExceeded):
        build_contact_nerve(_common_triple(), ledger=ledger)

    assert ledger.total("spatial_index_builds") == 1
    assert ledger.total("spatial_index_queries") == 0
    assert ledger.total("intersection_tests") == 0


def test_nerve_intersection_budget_is_atomic():
    ledger = WorkLedger(limits={"intersection_tests": 1})

    with pytest.raises(BudgetExceeded):
        build_contact_nerve(_common_triple(), ledger=ledger)

    assert ledger.total("spatial_index_builds") == 1
    assert ledger.total("spatial_index_queries") == 1
    assert ledger.total("intersection_tests") == 1


def test_nerve_counts_bbox_candidates_even_when_they_are_not_edges():
    # Bounding boxes overlap, but the two triangles are separated by the
    # diagonal gap 2 < x+y < 2.75.
    a = shapely.Polygon(((0.0, 0.0), (2.0, 0.0), (0.0, 2.0)))
    b = shapely.Polygon(((2.0, 2.0), (2.0, 0.75), (0.75, 2.0)))
    ledger = WorkLedger()

    nerve = build_contact_nerve(
        (SimpleNamespace(outer=a), SimpleNamespace(outer=b)),
        ledger=ledger,
    )

    assert nerve.edges == ()
    assert ledger.total("spatial_index_queries") == 1
    assert ledger.total("intersection_tests") == 1


def test_slice_counts_both_workspace_differences_even_without_obstacles():
    ledger = WorkLedger()
    scene = empty_scene(workspace=(-2.0, 2.0, -2.0, 2.0))

    assemble_slice(scene, 0.0, make_cfg(), (), ledger=ledger)

    assert ledger.total("union_operations") == 0
    assert ledger.total("difference_operations") == 2


def test_slice_difference_budget_fails_before_over_budget_geos_call():
    ledger = WorkLedger(limits={"difference_operations": 1})
    scene = empty_scene(workspace=(-2.0, 2.0, -2.0, 2.0))

    with pytest.raises(BudgetExceeded):
        assemble_slice(scene, 0.0, make_cfg(), (), ledger=ledger)

    assert ledger.total("difference_operations") == 1


def test_base_slice_ledger_records_all_topology_currency_families():
    ledger = WorkLedger()
    scene = triple_contact(sep=0.6, r=0.5)
    robot = ellipse_robot(0.15, 0.15)

    build_slice(scene, robot, 0.0, make_cfg(), build_nerve=True,
                ledger=ledger)

    snapshot = ledger.snapshot()["totals"]
    assert snapshot["support_value_evals"] > 0
    assert snapshot["union_operations"] > 0
    assert snapshot["difference_operations"] == 2
    assert snapshot["spatial_index_builds"] == 1
    assert snapshot["spatial_index_queries"] == 1
    assert snapshot["intersection_tests"] > 0


def test_slab_builder_forwards_ledger_to_every_base_slice():
    ledger = WorkLedger()
    cfg = make_cfg(initial_intervals=1)
    scene = triple_contact(sep=0.6, r=0.5)
    robot = ellipse_robot(0.15, 0.15)
    oracles = candidate_pairs(scene, robot, scene.workspace)

    decomposition = build_slabs(scene, robot, cfg, oracles, ledger=ledger)

    assert decomposition.n_slices == 2
    assert ledger.total("union_operations") > 0
    assert ledger.total("difference_operations") == 2 * decomposition.n_slices
