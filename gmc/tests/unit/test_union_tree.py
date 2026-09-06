import shapely
import pytest

from gmc.budget import BudgetExceeded, WorkLedger
from gmc.spatial.union_tree import IncrementalUnionTree


def test_union_tree_replacement_matches_clean_union_and_is_logarithmic():
    items = [(i, shapely.box(i * 0.8, 0.0, i * 0.8 + 1.0, 1.0))
             for i in range(8)]
    tree = IncrementalUnionTree(items)
    replacement = shapely.box(3.1, -0.5, 4.2, 0.8)
    operations = tree.update({3: replacement})
    expected = shapely.union_all([
        replacement if key == 3 else geometry
        for key, geometry in items
    ])

    assert tree.geometry.symmetric_difference(expected).area < 1e-12
    assert operations == 3
    assert operations < tree.build_operations


def test_union_tree_clone_is_transactionally_independent():
    items = [(0, shapely.box(0, 0, 1, 1)),
             (1, shapely.box(2, 0, 3, 1))]
    original = IncrementalUnionTree(items)
    clone = original.clone()
    clone.update({0: shapely.box(10, 10, 11, 11)})

    assert original.geometry.equals(shapely.union_all([g for _, g in items]))
    assert not clone.geometry.equals(original.geometry)


def test_union_ledger_counts_only_actual_geos_calls_not_empty_padding():
    ledger = WorkLedger()
    items = [(i, shapely.box(2 * i, 0, 2 * i + 1, 1))
             for i in range(3)]

    tree = IncrementalUnionTree(items, ledger=ledger)

    # A three-leaf power-of-two tree has three internal nodes, but one has an
    # empty padding child and therefore needs no GEOS union call.
    assert tree.build_operations == 2
    assert ledger.total("union_operations") == 2


def test_union_counts_real_calls_with_explicit_empty_leaves():
    ledger = WorkLedger()
    items = [
        (0, shapely.box(0, 0, 1, 1)),
        (1, shapely.Polygon()),
        (2, shapely.box(4, 0, 5, 1)),
        (3, shapely.Polygon()),
    ]

    tree = IncrementalUnionTree(items, ledger=ledger)

    # Both leaf parents short-circuit; only the root invokes GEOS union.
    assert tree.build_operations == 1
    assert ledger.total("union_operations") == 1
    assert tree.update({1: shapely.box(2, 0, 3, 1)}) == 2
    assert ledger.total("union_operations") == 3


def test_union_update_budget_failure_leaves_tree_unchanged():
    ledger = WorkLedger()
    items = [(i, shapely.box(2 * i, 0, 2 * i + 1, 1))
             for i in range(4)]
    tree = IncrementalUnionTree(items, ledger=ledger)
    before = tree.geometry
    assert ledger.total("union_operations") == 3
    # Replacing one leaf recomputes its parent and the root.  Permit exactly
    # the first operation; the second must fail before GEOS runs.
    ledger.limits["union_operations"] = 4

    with pytest.raises(BudgetExceeded):
        tree.update({0: shapely.box(20, 0, 21, 1)})

    assert ledger.total("union_operations") == 4
    assert tree.geometry.equals_exact(before, 0.0)
