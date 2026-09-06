"""Focused contracts for the fixed-translation connectivity slab path."""
import time

import pytest

from gmc.budget import WorkLedger
from gmc.io.robot_io import ellipse_robot
from gmc.orientation import slab_builder
from gmc.spatial.bvh import candidate_pairs
import gmc.spatial.slice_compiler as slice_compiler
from gmc.synth import quad_overlap, single_obstacle
from gmc.types import CertStatus

from ..conftest import make_cfg


def _single_pair_problem(*, initial_intervals=2):
    cfg = make_cfg(
        mode="theorem", eps_pair=2e-2, theta_min=2e-2,
        initial_intervals=initial_intervals,
    )
    scene = single_obstacle(r=0.3)
    robot = ellipse_robot(0.22, 0.12)
    oracles = candidate_pairs(scene, robot, scene.workspace)
    return scene, robot, cfg, oracles


def test_connectivity_build_uses_every_pair_and_only_cover_assemblies(
        monkeypatch):
    cfg = make_cfg(
        mode="theorem", eps_pair=2e-2, theta_min=2e-2,
        initial_intervals=4,
    )
    scene = quad_overlap(r=0.3)
    robot = ellipse_robot(0.22, 0.12)
    oracles = candidate_pairs(scene, robot, scene.workspace)
    expected_pairs = tuple(oracle.pair_id for oracle in oracles)
    calls = {"pair_slices": 0, "assemblies": 0}
    real_pair_builder = slab_builder.build_pair_sandwich_slice
    real_assemble = slab_builder.assemble_slice

    def count_pair_slices(*args, **kwargs):
        calls["pair_slices"] += 1
        return real_pair_builder(*args, **kwargs)

    def count_assemblies(*args, **kwargs):
        calls["assemblies"] += 1
        return real_assemble(*args, **kwargs)

    def sampled_regularity_is_forbidden(*_args, **_kwargs):
        raise AssertionError("connectivity build evaluated sampled regularity")

    monkeypatch.setattr(
        slab_builder, "build_pair_sandwich_slice", count_pair_slices)
    monkeypatch.setattr(slab_builder, "assemble_slice", count_assemblies)
    monkeypatch.setattr(
        slab_builder, "collect_predicates", sampled_regularity_is_forbidden)

    decomposition = slab_builder.build_connectivity_slabs(
        scene, robot, cfg, oracles,
    )

    # One pair-envelope set and one interval-cover assembly per configured
    # leaf.  No endpoint or fixed-midpoint free-space assembly is performed.
    assert calls == {"pair_slices": 4, "assemblies": 4}
    assert len(decomposition.slabs) == 4
    assert decomposition.n_slices == 4
    assert len(decomposition.slice_cache) == 4
    assert decomposition.construction_mode == \
        "fixed_translation_connectivity"
    assert decomposition.global_possible_cover_status is CertStatus.CERTIFIED
    assert slab_builder.decomposition_structure_failures(
        decomposition, cfg) == ()
    for slab in decomposition.slabs:
        assert isinstance(slab.mid_slice, slice_compiler.PairSandwichSlice)
        assert not isinstance(slab.mid_slice, slice_compiler.CertifiedSlice)
        assert tuple(item.pair_id for item in slab.mid_slice.sandwiches) == \
            expected_pairs
        assert tuple(item.pair_id for item in slab.cover_slice.sandwiches) == \
            expected_pairs
        assert slab.status is CertStatus.UNKNOWN
        assert slab.kind == "uncertain"
        assert not slab.predicates.regular_candidate
        assert not slab.predicates.certifies_no_event
        assert not slab.predicates.details["regularity_evaluated"]


def test_connectivity_refinement_builds_only_two_children_transactionally(
        monkeypatch):
    scene, robot, cfg, oracles = _single_pair_problem()
    decomposition = slab_builder.build_connectivity_slabs(
        scene, robot, cfg, oracles,
    )
    before_slab_ids = tuple(id(slab) for slab in decomposition.slabs)
    before_cache = dict(decomposition.slice_cache)
    before_history = tuple(decomposition.refinement_history)
    calls = {"pair_slices": 0, "assemblies": 0}
    real_pair_builder = slab_builder.build_pair_sandwich_slice
    real_assemble = slab_builder.assemble_slice

    def count_pair_slices(*args, **kwargs):
        calls["pair_slices"] += 1
        return real_pair_builder(*args, **kwargs)

    def count_assemblies(*args, **kwargs):
        calls["assemblies"] += 1
        return real_assemble(*args, **kwargs)

    monkeypatch.setattr(
        slab_builder, "build_pair_sandwich_slice", count_pair_slices)
    monkeypatch.setattr(slab_builder, "assemble_slice", count_assemblies)

    refined, record = slab_builder.refine_connectivity_slab(
        scene, robot, cfg, oracles, decomposition, 0,
    )

    assert calls == {"pair_slices": 2, "assemblies": 2}
    assert record["changed"]
    assert record["new_slices_built"] == 2
    assert len(record["child_slab_ids"]) == 2
    assert refined.revision == decomposition.revision + 1
    assert refined.n_slices == decomposition.n_slices + 2
    assert len(refined.slabs) == len(decomposition.slabs) + 1
    assert slab_builder.decomposition_structure_failures(refined, cfg) == ()

    # The returned revision is new; the caller-owned input was never edited.
    assert decomposition.revision == 0
    assert tuple(id(slab) for slab in decomposition.slabs) == before_slab_ids
    assert tuple(decomposition.refinement_history) == before_history
    assert decomposition.slice_cache.keys() == before_cache.keys()
    assert all(decomposition.slice_cache[key] is value
               for key, value in before_cache.items())


def test_connectivity_refinement_discards_first_child_on_deadline(
        monkeypatch):
    scene, robot, cfg, oracles = _single_pair_problem()
    decomposition = slab_builder.build_connectivity_slabs(
        scene, robot, cfg, oracles,
    )
    before_slab_ids = tuple(id(slab) for slab in decomposition.slabs)
    before_cache = dict(decomposition.slice_cache)
    calls = {"pair_slices": 0, "assemblies": 0}
    real_pair_builder = slab_builder.build_pair_sandwich_slice
    real_assemble = slab_builder.assemble_slice
    real_deadline_check = slab_builder.check_compilation_deadline

    def count_pair_slices(*args, **kwargs):
        calls["pair_slices"] += 1
        return real_pair_builder(*args, **kwargs)

    def count_assemblies(*args, **kwargs):
        calls["assemblies"] += 1
        return real_assemble(*args, **kwargs)

    def expire_between_children(deadline, stage):
        if stage == "connectivity_refinement_between_children":
            raise slice_compiler.CompilationDeadlineExceeded(stage, deadline)
        return real_deadline_check(deadline, stage)

    monkeypatch.setattr(
        slab_builder, "build_pair_sandwich_slice", count_pair_slices)
    monkeypatch.setattr(slab_builder, "assemble_slice", count_assemblies)
    monkeypatch.setattr(
        slab_builder, "check_compilation_deadline",
        expire_between_children,
    )

    with pytest.raises(
            slice_compiler.CompilationDeadlineExceeded) as caught:
        slab_builder.refine_connectivity_slab(
            scene, robot, cfg, oracles, decomposition, 0,
            deadline=time.perf_counter() + 3600.0,
        )

    assert caught.value.stage == "connectivity_refinement_between_children"
    assert calls == {"pair_slices": 1, "assemblies": 1}
    assert decomposition.revision == 0
    assert tuple(id(slab) for slab in decomposition.slabs) == before_slab_ids
    assert decomposition.slice_cache.keys() == before_cache.keys()
    assert all(decomposition.slice_cache[key] is value
               for key, value in before_cache.items())
    assert decomposition.refinement_history == []


def test_build_slice_forwards_the_incoming_ledger(monkeypatch):
    scene, robot, cfg, oracles = _single_pair_problem()
    ledger = WorkLedger()
    forwarded = []
    real_assemble = slice_compiler.assemble_slice

    def record_ledger(*args, **kwargs):
        forwarded.append(kwargs.get("ledger"))
        return real_assemble(*args, **kwargs)

    monkeypatch.setattr(slice_compiler, "assemble_slice", record_ledger)

    result = slice_compiler.build_slice(
        scene, robot, 0.25, cfg, oracles, ledger=ledger,
    )

    assert result.status is CertStatus.CERTIFIED
    assert forwarded == [ledger]


def test_expired_connectivity_build_starts_no_pair_work(monkeypatch):
    scene, robot, cfg, oracles = _single_pair_problem()

    def forbidden_pair_work(*_args, **_kwargs):
        raise AssertionError("pair work began after the absolute deadline")

    monkeypatch.setattr(
        slab_builder, "build_pair_sandwich_slice", forbidden_pair_work)
    with pytest.raises(
            slice_compiler.CompilationDeadlineExceeded) as caught:
        slab_builder.build_connectivity_slabs(
            scene, robot, cfg, oracles, deadline=0.0,
        )

    assert caught.value.stage == "connectivity_slabs_entry"
