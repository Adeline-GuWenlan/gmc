"""Possible-graph lineage must preserve every non-excluded transition."""
from types import SimpleNamespace

import numpy as np
import shapely

from gmc.budget import WorkLedger
from gmc.io.robot_io import ellipse_robot
from gmc.mobility.graph import compile_mobility, node_id
from gmc.mobility.lineage import candidate_links
from gmc.orientation.intervals import Interval
from gmc.orientation.slab_builder import Slab
from gmc.spatial.arrangement import FreeComponent
from gmc.types import CertStatus

from ..conftest import make_cfg


def _component(name, geometry):
    p = geometry.representative_point()
    return FreeComponent(
        component_id=name,
        geometry=geometry,
        representative=np.array([p.x, p.y]),
        area=float(geometry.area),
        boundary_signature=(),
        causing_pairs=(),
        clearance_lb=None,
        status=CertStatus.APPROX_UNCERTIFIED,
    )


def _slice(possible=()):
    return SimpleNamespace(D_safe=(), D_possible=tuple(possible))


def _three_box_slabs():
    # A overlaps the left of connected X; B overlaps its right.  There is no
    # point common to A, X and B, so no fixed rotation anchor exists even
    # though the possible lineage through X is real.
    a = _component("A", shapely.box(0.0, 0.0, 2.0, 1.0))
    x = _component("X", shapely.box(1.0, 0.0, 4.0, 1.0))
    b = _component("B", shapely.box(3.0, 0.0, 5.0, 1.0))
    assert a.geometry.intersection(x.geometry).area > 0
    assert x.geometry.intersection(b.geometry).area > 0
    assert a.geometry.intersection(x.geometry).intersection(
        b.geometry).is_empty

    x_slice = _slice((x,))
    empty_wrap = _slice(())
    sa = Slab(
        Interval(0.0, np.pi), CertStatus.UNKNOWN, "uncertain",
        empty_wrap, _slice((a,)), x_slice,
        predicates=SimpleNamespace(certifies_no_event=False), slab_id=0,
    )
    sb = Slab(
        Interval(np.pi, 2.0 * np.pi), CertStatus.UNKNOWN, "uncertain",
        x_slice, _slice((b,)), empty_wrap,
        predicates=SimpleNamespace(certifies_no_event=False), slab_id=1,
    )
    return sa, sb


def test_three_box_lineage_without_single_anchor_is_still_possible():
    sa, sb = _three_box_slabs()
    ledger = WorkLedger()

    links = candidate_links(sa, sb, "possible", ledger=ledger)

    assert len(links) == 1
    assert links[0].common_points == ()
    assert not links[0].is_geometrically_excluded
    assert "without_single_rotation_anchor" in links[0].reasons[0]
    # A-X, X-B, then both intersections in A-X-B.
    assert ledger.total("intersection_tests") == 4


def test_three_box_transition_is_retained_in_possible_graph():
    sa, sb = _three_box_slabs()
    scene = SimpleNamespace(workspace=shapely.box(-1.0, -1.0, 6.0, 2.0))
    robot = ellipse_robot(0.2, 0.1)
    dec = SimpleNamespace(slabs=[sa, sb])

    # This test isolates graph lineage with synthetic slices rather than a
    # compiled scene, so it explicitly opts out of the production provenance
    # gate. Certified compilation never does this.
    mc = compile_mobility(scene, robot, make_cfg(), [], dec,
                          validate_oracles=False)

    assert mc.M_possible.has_edge(node_id(0, 0, "possible"),
                                  node_id(1, 0, "possible"))
    edge = mc.M_possible.get_edge_data(node_id(0, 0, "possible"),
                                       node_id(1, 0, "possible"))
    assert edge["status"] == "POSSIBLE"
    assert "without_single_rotation_anchor" in edge["reasons"][0]
