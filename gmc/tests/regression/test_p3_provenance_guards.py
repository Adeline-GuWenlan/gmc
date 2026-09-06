"""P3 proof objects must not be rebound, truncated, or tolerance-glued."""

from dataclasses import replace

import numpy as np
import pytest
import shapely

from gmc.io.robot_io import ellipse_robot
from gmc.mobility.graph import compile_mobility
from gmc.mobility.lineage import certified_cover_slice
from gmc.mobility.query import query
from gmc.orientation.intervals import Interval
from gmc.orientation.slab_builder import (
    _full_circle_tiling,
    _set_global_cover_status,
    build_slabs,
)
from gmc.spatial.bvh import candidate_pairs
from gmc.synth import empty_scene, single_door
from gmc.types import CertStatus, PlanStatus, Pose2

from ..conftest import make_cfg


@pytest.fixture(scope="module")
def sealed_compilation():
    cfg = make_cfg(
        mode="theorem", eps_pair=1e-2, theta_min=1e-2,
        initial_intervals=16,
    )
    scene = single_door(
        0.05, wall_t=0.18, workspace=(-2.0, 2.0, -1.2, 1.2),
    )
    robot = ellipse_robot(0.5, 0.2)
    oracles = candidate_pairs(scene, robot, scene.workspace)
    decomposition = build_slabs(scene, robot, cfg, oracles)
    assert decomposition.global_possible_cover_status is CertStatus.CERTIFIED
    return cfg, scene, robot, oracles, decomposition


def test_certified_decomposition_cannot_be_reused_for_another_scene(
        sealed_compilation):
    cfg, _, robot, _, decomposition = sealed_compilation
    replacement = empty_scene(workspace=(-2.0, 2.0, -1.2, 1.2))
    replacement_oracles = candidate_pairs(
        replacement, robot, replacement.workspace,
    )

    with pytest.raises(ValueError, match="decomposition provenance"):
        compile_mobility(
            replacement, robot, cfg, replacement_oracles, decomposition,
        )


def test_substituting_a_midpoint_slice_for_interval_proof_is_rejected(
        sealed_compilation):
    cfg, scene, robot, oracles, decomposition = sealed_compilation
    slabs = list(decomposition.slabs)
    slabs[0] = replace(slabs[0], cover_slice=slabs[0].mid_slice)
    corrupted = replace(decomposition, slabs=slabs)

    assert certified_cover_slice(corrupted.slabs[0]) is None
    with pytest.raises(ValueError, match="decomposition structure"):
        compile_mobility(scene, robot, cfg, oracles, corrupted)


def test_certified_cover_cannot_omit_a_pair_outer_obstacle(
        sealed_compilation):
    cfg, scene, robot, oracles, decomposition = sealed_compilation
    slabs = list(decomposition.slabs)
    original = slabs[0]
    cover = original.cover_slice
    assert cover is not None and cover.sandwiches

    # Removing the complete outer union enlarges the declared all-theta SAFE
    # region.  The status/provenance fields alone must not authenticate it.
    corrupted_cover = replace(cover, C_plus=shapely.Polygon())
    slabs[0] = replace(original, cover_slice=corrupted_cover)
    corrupted = replace(decomposition, slabs=slabs)

    assert certified_cover_slice(corrupted.slabs[0]) is None
    with pytest.raises(ValueError, match="cover_outer_union_binding"):
        compile_mobility(scene, robot, cfg, oracles, corrupted)


def test_sub_tolerance_angular_gap_never_certifies_full_circle(
        sealed_compilation):
    cfg, _, _, _, decomposition = sealed_compilation
    slabs = list(decomposition.slabs)
    original = slabs[1]
    shifted_lo = np.nextafter(float(original.interval.lo), np.inf)
    assert 0.0 < shifted_lo - original.interval.lo < 1e-12
    slabs[1] = replace(
        original,
        interval=Interval(shifted_lo, original.interval.hi),
    )
    corrupted = replace(decomposition, slabs=slabs)

    assert not _full_circle_tiling(corrupted.slabs)
    _set_global_cover_status(corrupted, cfg)
    assert corrupted.global_possible_cover_status is CertStatus.UNKNOWN
    assert not corrupted.global_possible_cover_provenance[
        "full_circle_tiling"
    ]


def test_missing_possible_lineage_turns_formal_cut_into_internal_error(
        sealed_compilation):
    cfg, scene, robot, oracles, decomposition = sealed_compilation
    compiler = compile_mobility(
        scene, robot, cfg, oracles, decomposition,
    )
    assert compiler.M_possible.number_of_edges() > 0
    missing_edge = next(iter(compiler.M_possible.edges))
    compiler.M_possible.remove_edge(*missing_edge)

    result = query(
        Pose2(np.array([-1.5, 0.0]), 0.0),
        Pose2(np.array([1.5, 0.0]), 0.0),
        compiler,
    )

    assert result.status is PlanStatus.INTERNAL_ERROR
    assert "possible_adjacency_completeness" in result.report.get(
        "failed_invariants", []
    )
