"""Invariant gates must reject corrupt graph evidence and periodic glue."""
from gmc.io.robot_io import ellipse_robot
from gmc.mobility.graph import compile_mobility
from gmc.orientation.slab_builder import build_slabs
from gmc.spatial.bvh import candidate_pairs
from gmc.synth import empty_scene
from gmc.verification.invariants import (check_graph_nesting,
                                         check_periodic_coverage,
                                         check_witness_ownership)


def _compiler(cfg):
    scene = empty_scene(workspace=(-1.0, 1.0, -1.0, 1.0))
    robot = ellipse_robot(0.2, 0.1)
    oracles = candidate_pairs(scene, robot, scene.workspace)
    decomposition = build_slabs(scene, robot, cfg, oracles)
    return compile_mobility(scene, robot, cfg, oracles, decomposition)


def test_garbage_safe_edge_witness_fails_i4(cfg):
    mc = _compiler(cfg)
    edge = next(iter(mc.M_safe.edges))
    mc.M_safe.edges[edge]["witness"] = "not a pose-arc certificate"
    assert not check_witness_ownership(mc, replay=False)


def test_missing_possible_counterpart_fails_i3(cfg):
    mc = _compiler(cfg)
    left, right = next(iter(mc.M_safe.edges))
    dl, dr = mc.M_safe.nodes[left], mc.M_safe.nodes[right]
    possible_edge = (
        mc.safe_to_possible[(dl["slab"], dl["comp"])],
        mc.safe_to_possible[(dr["slab"], dr["comp"])],
    )
    mc.M_possible.remove_edge(*possible_edge)
    assert not check_graph_nesting(mc)


def test_missing_periodic_wrap_edge_fails_i6(cfg):
    mc = _compiler(cfg)
    slabs = sorted(mc.decomposition.slabs,
                   key=lambda slab: slab.interval.lo)
    from gmc.mobility.graph import node_id
    wrap = (node_id(slabs[-1].slab_id, 0, "possible"),
            node_id(slabs[0].slab_id, 0, "possible"))
    assert mc.M_possible.has_edge(*wrap)
    mc.M_possible.remove_edge(*wrap)
    assert not check_periodic_coverage(mc.decomposition, mc)
