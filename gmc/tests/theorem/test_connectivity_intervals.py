"""Certified fixed-translation connectivity intervals over orientation."""

from types import SimpleNamespace

import numpy as np
import pytest
from shapely.geometry import box

import gmc.orientation.connectivity_intervals as connectivity
import gmc.orientation.slab_builder as slab_builder
from gmc.budget import BudgetExceeded
from gmc.io.robot_io import ellipse_robot
from gmc.orientation.connectivity_intervals import (
    ConnectivityInterval,
    ConnectivityIntervalInvariantError,
    ConnectivityIntervalReport,
    ConnectivityVerdict,
    FixedTranslationConnectivityQuery,
    certified_connectivity_intervals,
    certify_fixed_translation_sweep,
    classify_translation_interval,
    validate_connectivity_bounds,
)
from gmc.orientation.intervals import TWO_PI, Interval
from gmc.spatial.bvh import candidate_pairs
from gmc.synth import gate_half_angle, single_door, single_obstacle
from gmc.types import GaussianSupport2D, RobotModel2D

from ..conftest import make_cfg


def _contains(intervals, theta):
    theta = float(theta) % TWO_PI
    for interval in intervals:
        if interval.width >= TWO_PI:
            return True
        lo = interval.lo % TWO_PI
        hi = lo + interval.width
        lifted = theta if theta >= lo else theta + TWO_PI
        if lo <= lifted <= hi:
            return True
    return False


def _compile(scene, robot, cfg):
    oracles = candidate_pairs(scene, robot, scene.workspace)
    decomposition = slab_builder.build_slabs(
        scene, robot, cfg, oracles,
    )
    return oracles, decomposition


def test_door_above_major_diameter_is_certified_open_for_full_s1():
    scene = single_door(
        1.1, wall_t=0.18, workspace=(-1.6, 1.6, -0.85, 0.85),
    )
    robot = ellipse_robot(0.5, 0.2)
    cfg = make_cfg(
        mode="theorem", eps_pair=2e-2, theta_min=0.1,
        initial_intervals=4,
    )
    oracles, decomposition = _compile(scene, robot, cfg)

    report = certified_connectivity_intervals(
        scene, robot, cfg, oracles, decomposition,
        (-1.0, 0.0), (1.0, 0.0), max_refinements=16,
    )

    assert all(item.verdict is ConnectivityVerdict.OPEN
               for item in report.intervals)
    assert report.theta_safe == (Interval(0.0, TWO_PI),)
    assert report.theta_possible == (Interval(0.0, TWO_PI),)
    assert not report.unresolved
    assert report.stop_reason == "resolved"


def test_door_below_minor_diameter_is_certified_closed_for_full_s1():
    scene = single_door(
        0.18, wall_t=0.18, workspace=(-2.0, 2.0, -1.2, 1.2),
    )
    robot = ellipse_robot(0.5, 0.2)
    cfg = make_cfg(
        mode="theorem", eps_pair=2e-2, theta_min=2e-2,
        initial_intervals=8,
    )
    oracles, decomposition = _compile(scene, robot, cfg)

    report = certified_connectivity_intervals(
        scene, robot, cfg, oracles, decomposition,
        (-1.2, 0.0), (1.2, 0.0), max_refinements=16,
    )

    assert all(item.verdict is ConnectivityVerdict.CLOSED
               for item in report.intervals)
    assert report.theta_safe == ()
    assert report.theta_possible == ()
    assert report.stop_reason == "resolved"


@pytest.fixture(scope="module")
def partial_gate_report():
    scene = single_door(
        0.8, wall_t=0.3, workspace=(-1.6, 1.6, -0.85, 0.85),
    )
    robot = ellipse_robot(0.5, 0.2)
    cfg = make_cfg(
        mode="theorem", eps_pair=3e-2, theta_min=8e-2,
        initial_intervals=4,
    )
    oracles, decomposition = _compile(scene, robot, cfg)
    report = certified_connectivity_intervals(
        scene, robot, cfg, oracles, decomposition,
        (-1.0, 0.0), (1.0, 0.0), max_refinements=8,
    )
    return (
        scene, robot, cfg, oracles, decomposition, report,
        gate_half_angle(0.5, 0.2, 0.8),
    )


def test_partial_gate_bounds_contain_the_analytic_truth(partial_gate_report):
    *_, report, half_angle = partial_gate_report
    angles = np.linspace(0.0, TWO_PI, 721, endpoint=False)

    for theta in angles:
        # The generator's analytic fixed-orientation pass condition is
        # |theta mod pi| < half_angle.  Stay one grid step away from equality
        # when asserting truth so this is not a floating tangency test.
        distance = abs(float(np.remainder(theta + np.pi / 2, np.pi)
                             - np.pi / 2))
        analytically_open = distance < half_angle - 1e-6
        if _contains(report.theta_safe, theta):
            assert distance < half_angle
        if analytically_open:
            assert _contains(report.theta_possible, theta)

    assert report.theta_safe
    assert report.theta_possible
    assert any(item.verdict is ConnectivityVerdict.OPEN
               for item in report.intervals)
    assert any(item.verdict is ConnectivityVerdict.CLOSED
               for item in report.intervals)
    assert any(item.verdict is ConnectivityVerdict.UNKNOWN
               for item in report.intervals)


def test_tangent_angles_abstain_and_seam_is_merged(partial_gate_report):
    *_, report, half_angle = partial_gate_report

    # Closed-contact convention: the analytic equality is not promoted into
    # the certified lower set, while the upper set retains it.
    for theta in (half_angle, np.pi - half_angle,
                  np.pi + half_angle, TWO_PI - half_angle):
        assert not _contains(report.theta_safe, theta)
        assert _contains(report.theta_possible, theta)

    seam_intervals = [interval for interval in report.theta_safe
                      if interval.hi > TWO_PI]
    assert len(seam_intervals) == 1
    assert _contains(seam_intervals, 0.0)
    assert _contains(seam_intervals, np.nextafter(TWO_PI, 0.0))


def test_opposite_certified_neighbours_only_guarantee_a_transition(
        partial_gate_report):
    *_, report, _ = partial_gate_report

    assert report.transition_brackets
    for bracket in report.transition_brackets:
        assert bracket.left_verdict is not bracket.right_verdict
        assert bracket.minimum_transitions == 1
        assert bracket.isolated is False
        assert any(
            unresolved.interval == bracket.interval
            for unresolved in report.unresolved
        )


def test_explicit_sweep_api_preserves_leaf_lower_upper_contract(
        partial_gate_report):
    scene, robot, cfg, oracles, decomposition, _, _ = partial_gate_report
    query = FixedTranslationConnectivityQuery(
        np.array([-1.0, 0.0]), np.array([1.0, 0.0]), "partial_gate",
    )

    certificate = certify_fixed_translation_sweep(
        scene, robot, cfg, oracles, decomposition, query,
        event_tolerance=0.4,
        max_splits=16,
        max_support_calls=1_000_000,
        max_wall_seconds=60.0,
    )

    assert certificate.resolution_met
    assert certificate.theta_safe
    assert certificate.theta_possible
    assert certificate.leaf_certificates
    assert all(not leaf.lower_connected or leaf.upper_connected
               for leaf in certificate.leaf_certificates)
    assert all(not leaf.implies_event_free_slab
               for leaf in certificate.leaf_certificates)
    assert all(bracket.multiplicity_lower_bound == 1
               and not bracket.isolated
               for bracket in certificate.transition_brackets)


def test_explicit_sweep_charges_leaf_artifact_postprocessing_to_wall_budget(
        monkeypatch):
    slab = SimpleNamespace(
        interval=Interval(0.0, TWO_PI), slab_id=0, depth=0,
    )
    decomposition = SimpleNamespace(slabs=[slab], revision=0)
    interval = ConnectivityInterval(
        slab.interval, ConnectivityVerdict.OPEN, 0, 0, "injected_open",
    )
    inner_report = ConnectivityIntervalReport(
        period=TWO_PI,
        start_xy=(0.0, 0.0),
        goal_xy=(1.0, 0.0),
        intervals=(interval,),
        theta_safe=(slab.interval,),
        theta_possible=(slab.interval,),
        unresolved=(),
        transition_brackets=(),
        refinement_trace=(),
        initial_revision=0,
        final_revision=0,
        stop_reason="resolved",
        decomposition=decomposition,
        support_calls=0,
        max_support_calls=100,
        wall_seconds=0.1,
        max_wall_seconds=1.0,
        resolution_met=True,
        event_tolerance=0.1,
    )
    monkeypatch.setattr(
        connectivity, "certified_connectivity_intervals",
        lambda *args, **kwargs: inner_report,
    )
    clock = [0.0]
    monkeypatch.setattr(connectivity.time, "perf_counter", lambda: clock[0])

    def slow_leaf(*args, **kwargs):
        clock[0] = 2.0
        return SimpleNamespace(lower_connected=True, upper_connected=True)

    monkeypatch.setattr(
        connectivity, "classify_fixed_translation_interval", slow_leaf,
    )
    query = FixedTranslationConnectivityQuery(
        np.array([0.0, 0.0]), np.array([1.0, 0.0]), "slow_artifact",
    )

    certificate = certify_fixed_translation_sweep(
        None, None, SimpleNamespace(), (), decomposition, query,
        event_tolerance=0.1,
        max_splits=0,
        max_support_calls=100,
        max_wall_seconds=1.0,
    )

    assert certificate.stop_reason == "wall_budget_exhausted"
    assert certificate.wall_seconds == 2.0
    assert certificate.max_wall_seconds == 1.0
    assert not certificate.resolution_met
    assert certificate.sweep_report.stop_reason == certificate.stop_reason
    assert not certificate.sweep_report.resolution_met


@pytest.mark.parametrize(
    ("support_budget", "wall_budget", "expected"),
    [
        (0, 60.0, "support_budget_exhausted"),
        (1_000_000, 0.0, "wall_budget_exhausted"),
    ],
)
def test_zero_independent_budget_accepts_no_refinement(
        partial_gate_report, support_budget, wall_budget, expected):
    scene, robot, cfg, oracles, decomposition, _, _ = partial_gate_report

    report = certified_connectivity_intervals(
        scene, robot, cfg, oracles, decomposition,
        (-1.0, 0.0), (1.0, 0.0),
        max_refinements=4,
        max_support_calls=support_budget,
        max_wall_seconds=wall_budget,
        event_tolerance=1e-3,
    )

    assert report.stop_reason == expected
    assert report.n_refinements == 0
    assert report.final_revision == report.initial_revision
    assert report.unresolved
    assert not report.resolution_met


def test_cross_scene_decomposition_binding_is_rejected(partial_gate_report):
    _, robot, cfg, _, decomposition, _, _ = partial_gate_report
    replacement_scene = single_door(
        0.8, wall_t=0.3, workspace=(-1.6, 1.6, -0.85, 0.85),
    )
    replacement_oracles = candidate_pairs(
        replacement_scene, robot, replacement_scene.workspace,
    )

    with pytest.raises(
            ConnectivityIntervalInvariantError,
            match="scene_identity|candidate_pair_identity"):
        certified_connectivity_intervals(
            replacement_scene, robot, cfg, replacement_oracles,
            decomposition, (-1.0, 0.0), (1.0, 0.0),
            max_refinements=0,
        )


def test_split_finishing_after_wall_deadline_is_not_accepted(monkeypatch):
    edges = (0.0, np.pi, TWO_PI)
    slabs = [SimpleNamespace(
        interval=Interval(edges[i], edges[i + 1]), slab_id=i, depth=0,
    ) for i in range(2)]
    decomposition = SimpleNamespace(slabs=slabs, revision=3)
    refined = SimpleNamespace(slabs=slabs, revision=4)
    cfg = SimpleNamespace(
        query=SimpleNamespace(
            max_refinement_rounds=1,
            max_support_calls=100,
            max_wall_seconds=1.0,
        ),
        orientation=SimpleNamespace(theta_min=1e-3, max_depth=10),
    )
    monkeypatch.setattr(connectivity, "_compiler_input_failures",
                        lambda *args: ())
    monkeypatch.setattr(
        connectivity, "validate_candidate_oracles",
        lambda scene, robot, workspace, oracles: list(oracles),
    )
    monkeypatch.setattr(
        connectivity, "classify_translation_interval",
        lambda slab, start, goal: ConnectivityInterval(
            slab.interval, ConnectivityVerdict.UNKNOWN,
            slab.slab_id, slab.depth, "injected_unknown",
        ),
    )
    clock = [0.0]

    def late_refinement(*args, **kwargs):
        clock[0] = 2.0
        return refined, {
            "changed": True,
            "revision_before": 3,
            "revision_after": 4,
        }

    monkeypatch.setattr(connectivity, "refine_slab", late_refinement)
    monkeypatch.setattr(connectivity.time, "perf_counter", lambda: clock[0])

    report = certified_connectivity_intervals(
        None, None, cfg, (), decomposition,
        (0.0, 0.0), (1.0, 0.0),
        max_refinements=1,
        max_support_calls=100,
        max_wall_seconds=1.0,
    )

    assert report.stop_reason == "wall_budget_exhausted"
    assert report.n_refinements == 0
    assert report.final_revision == 3


def test_expired_wall_budget_skips_deep_validation_and_classification(
        monkeypatch):
    edges = (0.0, np.pi, TWO_PI)
    slabs = [SimpleNamespace(
        interval=Interval(edges[index], edges[index + 1]),
        slab_id=index,
        depth=0,
    ) for index in range(2)]
    decomposition = SimpleNamespace(slabs=slabs, revision=0)
    cfg = SimpleNamespace(
        query=SimpleNamespace(
            max_refinement_rounds=1,
            max_support_calls=100,
            max_wall_seconds=0.0,
        ),
        orientation=SimpleNamespace(theta_min=1e-3, max_depth=10),
    )

    def forbidden(*args, **kwargs):
        raise AssertionError("expired sweep started deep proof work")

    monkeypatch.setattr(connectivity, "_compiler_input_failures", forbidden)
    monkeypatch.setattr(
        connectivity, "classify_translation_interval", forbidden,
    )
    monkeypatch.setattr(connectivity, "refine_slab", forbidden)
    monkeypatch.setattr(connectivity.time, "perf_counter", lambda: 7.0)

    report = certified_connectivity_intervals(
        None, None, cfg, (), decomposition,
        (0.0, 0.0), (1.0, 0.0),
        max_refinements=1,
        max_support_calls=100,
        max_wall_seconds=0.0,
    )

    assert report.stop_reason == "wall_budget_exhausted"
    assert all(item.verdict is ConnectivityVerdict.UNKNOWN
               for item in report.intervals)
    assert all(item.reason == "wall_budget_exhausted"
               for item in report.intervals)
    assert report.final_revision == report.initial_revision


def test_specialized_split_classifies_only_new_children(monkeypatch):
    parent = SimpleNamespace(
        interval=Interval(0.0, np.pi), slab_id=0, depth=0,
    )
    unchanged = SimpleNamespace(
        interval=Interval(np.pi, TWO_PI), slab_id=1, depth=0,
    )
    decomposition = SimpleNamespace(
        slabs=[parent, unchanged], revision=0, ledger=None,
        input_binding=None,
        construction_mode="fixed_translation_connectivity",
    )
    left = SimpleNamespace(
        interval=Interval(0.0, np.pi / 2.0), slab_id=0, depth=1,
    )
    right = SimpleNamespace(
        interval=Interval(np.pi / 2.0, np.pi), slab_id=1, depth=1,
    )
    renumbered_unchanged = SimpleNamespace(
        interval=unchanged.interval, slab_id=2, depth=0,
    )
    refined = SimpleNamespace(
        slabs=[left, right, renumbered_unchanged], revision=1, ledger=None,
        input_binding=None,
        construction_mode="fixed_translation_connectivity",
    )
    record = {
        "changed": True,
        "revision_before": 0,
        "revision_after": 1,
        "parent_slab_id": 0,
        "parent_interval": [0.0, np.pi],
        "child_slab_ids": [0, 1],
        "child_intervals": [
            [0.0, np.pi / 2.0], [np.pi / 2.0, np.pi],
        ],
    }
    cfg = SimpleNamespace(
        query=SimpleNamespace(
            max_refinement_rounds=1,
            max_support_calls=100,
            max_wall_seconds=60.0,
        ),
        orientation=SimpleNamespace(theta_min=1e-3, max_depth=10),
    )
    validations = []
    classifications = []
    candidate_validations = []
    bound_oracles = []

    def validate(*args):
        validations.append(args[4].revision)
        return ()

    def classify(slab, start, goal):
        classifications.append((slab.interval.lo, slab.interval.hi))
        return ConnectivityInterval(
            slab.interval, ConnectivityVerdict.UNKNOWN,
            slab.slab_id, slab.depth, "injected_unknown",
        )

    def validate_candidates(scene, robot, workspace, oracles):
        candidate_validations.append(tuple(oracles))
        return bound_oracles

    def refine(*args, **kwargs):
        assert args[3] is bound_oracles
        return refined, record

    monkeypatch.setattr(connectivity, "_compiler_input_failures", validate)
    monkeypatch.setattr(
        connectivity, "validate_candidate_oracles",
        validate_candidates,
    )
    monkeypatch.setattr(connectivity, "classify_translation_interval", classify)
    monkeypatch.setattr(
        connectivity, "refine_connectivity_slab", refine,
    )

    report = certified_connectivity_intervals(
        None, None, cfg, (), decomposition,
        (0.0, 0.0), (1.0, 0.0),
        max_refinements=1,
        max_support_calls=100,
        max_wall_seconds=60.0,
    )

    assert validations == [0]
    assert candidate_validations == [()]
    assert classifications == [
        (0.0, np.pi),
        (np.pi, TWO_PI),
        (0.0, np.pi / 2.0),
        (np.pi / 2.0, np.pi),
    ]
    assert report.n_refinements == 1
    assert report.final_revision == 1
    assert [item.slab_id for item in report.intervals] == [0, 1, 2]


def test_resolved_specialized_revision_gets_one_final_full_validation(
        monkeypatch):
    parent = SimpleNamespace(
        interval=Interval(0.0, TWO_PI), slab_id=0, depth=0,
    )
    decomposition = SimpleNamespace(
        slabs=[parent], revision=0, ledger=None, input_binding=None,
        construction_mode="fixed_translation_connectivity",
    )
    children = [
        SimpleNamespace(
            interval=Interval(0.0, np.pi), slab_id=0, depth=1,
        ),
        SimpleNamespace(
            interval=Interval(np.pi, TWO_PI), slab_id=1, depth=1,
        ),
    ]
    refined = SimpleNamespace(
        slabs=children, revision=1, ledger=None, input_binding=None,
        construction_mode="fixed_translation_connectivity",
    )
    record = {
        "changed": True,
        "revision_before": 0,
        "revision_after": 1,
        "parent_slab_id": 0,
        "parent_interval": [0.0, TWO_PI],
        "child_slab_ids": [0, 1],
        "child_intervals": [[0.0, np.pi], [np.pi, TWO_PI]],
    }
    cfg = SimpleNamespace(
        query=SimpleNamespace(
            max_refinement_rounds=1,
            max_support_calls=100,
            max_wall_seconds=60.0,
        ),
        orientation=SimpleNamespace(theta_min=1e-3, max_depth=10),
    )
    validation_revisions = []

    def validate(*args):
        revision = args[4].revision
        validation_revisions.append(revision)
        return (() if revision == 0 else ("injected_final_corruption",))

    def classify(slab, start, goal):
        verdict = (ConnectivityVerdict.UNKNOWN if slab.depth == 0
                   else ConnectivityVerdict.OPEN)
        return ConnectivityInterval(
            slab.interval, verdict, slab.slab_id, slab.depth,
            "injected_verdict",
        )

    monkeypatch.setattr(connectivity, "_compiler_input_failures", validate)
    monkeypatch.setattr(
        connectivity, "validate_candidate_oracles",
        lambda scene, robot, workspace, oracles: list(oracles),
    )
    monkeypatch.setattr(connectivity, "classify_translation_interval", classify)
    monkeypatch.setattr(
        connectivity, "refine_connectivity_slab",
        lambda *args, **kwargs: (refined, record),
    )

    with pytest.raises(
            ConnectivityIntervalInvariantError,
            match="injected_final_corruption"):
        certified_connectivity_intervals(
            None, None, cfg, (), decomposition,
            (0.0, 0.0), (1.0, 0.0),
            max_refinements=1,
            max_support_calls=100,
            max_wall_seconds=60.0,
        )

    assert validation_revisions == [0, 1]


def test_resolved_classification_crossing_wall_deadline_is_not_accepted(
        monkeypatch):
    slab = SimpleNamespace(
        interval=Interval(0.0, TWO_PI), slab_id=0, depth=0,
    )
    decomposition = SimpleNamespace(slabs=[slab], revision=0)
    cfg = SimpleNamespace(
        query=SimpleNamespace(
            max_refinement_rounds=0,
            max_support_calls=100,
            max_wall_seconds=1.0,
        ),
        orientation=SimpleNamespace(theta_min=1e-3, max_depth=0),
    )
    monkeypatch.setattr(connectivity, "_compiler_input_failures",
                        lambda *args: ())
    monkeypatch.setattr(
        connectivity, "validate_candidate_oracles",
        lambda scene, robot, workspace, oracles: list(oracles),
    )
    clock = [0.0]

    def late_classification(item, start, goal):
        clock[0] = 2.0
        return ConnectivityInterval(
            item.interval, ConnectivityVerdict.OPEN,
            item.slab_id, item.depth, "injected_open",
        )

    monkeypatch.setattr(
        connectivity, "classify_translation_interval", late_classification,
    )
    monkeypatch.setattr(connectivity.time, "perf_counter", lambda: clock[0])

    report = certified_connectivity_intervals(
        None, None, cfg, (), decomposition,
        (0.0, 0.0), (1.0, 0.0),
        max_refinements=0,
        max_support_calls=100,
        max_wall_seconds=1.0,
    )

    assert report.stop_reason == "wall_budget_exhausted"
    assert report.wall_seconds > report.max_wall_seconds
    assert not report.resolution_met


def test_support_budget_rejects_batch_before_oracle_work(monkeypatch):
    scene = single_obstacle(r=0.3)
    robot = ellipse_robot(0.2, 0.1)
    oracle = candidate_pairs(scene, robot, scene.workspace)[0]
    slabs = [SimpleNamespace(
        interval=Interval(0.0, TWO_PI), slab_id=0, depth=0,
    )]
    decomposition = SimpleNamespace(slabs=slabs, revision=0, ledger=None)
    cfg = SimpleNamespace(
        query=SimpleNamespace(
            max_refinement_rounds=1,
            max_support_calls=1,
            max_wall_seconds=10.0,
        ),
        orientation=SimpleNamespace(theta_min=1e-3, max_depth=10),
    )
    monkeypatch.setattr(connectivity, "_compiler_input_failures",
                        lambda *args: ())
    monkeypatch.setattr(
        connectivity, "validate_candidate_oracles",
        lambda scene, robot, workspace, oracles: list(oracles),
    )
    monkeypatch.setattr(
        connectivity, "classify_translation_interval",
        lambda slab, start, goal: ConnectivityInterval(
            slab.interval, ConnectivityVerdict.UNKNOWN,
            slab.slab_id, slab.depth, "injected_unknown",
        ),
    )

    def support_hungry_refinement(*args, **kwargs):
        angles = np.linspace(0.0, TWO_PI, 16, endpoint=False)
        directions = np.stack((np.cos(angles), np.sin(angles)), axis=1)
        oracle.support_values(0.0, directions)
        raise AssertionError("hard budget should reject before this line")

    monkeypatch.setattr(connectivity, "refine_slab",
                        support_hungry_refinement)
    calls_before = oracle.calls
    report = certified_connectivity_intervals(
        scene, robot, cfg, (oracle,), decomposition,
        (0.0, 0.0), (1.0, 0.0),
        max_refinements=1,
        max_support_calls=1,
        max_wall_seconds=10.0,
    )

    assert report.stop_reason == (
        "connectivity_sweep_support_evals_budget_exhausted"
    )
    assert oracle.calls - calls_before <= 1
    assert report.support_calls <= 1
    assert report.n_refinements == 0


def test_non_support_ledger_budget_preserves_its_currency(monkeypatch):
    slab = SimpleNamespace(
        interval=Interval(0.0, TWO_PI), slab_id=0, depth=0,
    )
    decomposition = SimpleNamespace(slabs=[slab], revision=0, ledger=None)
    cfg = SimpleNamespace(
        query=SimpleNamespace(
            max_refinement_rounds=1,
            max_support_calls=100,
            max_wall_seconds=10.0,
        ),
        orientation=SimpleNamespace(theta_min=1e-3, max_depth=10),
    )
    monkeypatch.setattr(connectivity, "_compiler_input_failures",
                        lambda *args: ())
    monkeypatch.setattr(
        connectivity, "validate_candidate_oracles",
        lambda scene, robot, workspace, oracles: list(oracles),
    )
    monkeypatch.setattr(
        connectivity, "classify_translation_interval",
        lambda item, start, goal: ConnectivityInterval(
            item.interval, ConnectivityVerdict.UNKNOWN,
            item.slab_id, item.depth, "injected_unknown",
        ),
    )

    def union_budget_exhausted(*_args, **_kwargs):
        raise BudgetExceeded("union_operations", 0, 0, 1)

    monkeypatch.setattr(
        connectivity, "refine_slab", union_budget_exhausted,
    )
    report = certified_connectivity_intervals(
        None, None, cfg, (), decomposition,
        (0.0, 0.0), (1.0, 0.0),
        max_refinements=1,
        max_support_calls=100,
        max_wall_seconds=10.0,
    )

    assert report.stop_reason == "union_operations_budget_exhausted"
    assert not report.resolution_met
    assert report.support_calls == 0
    assert report.n_refinements == 0


@pytest.mark.parametrize("kind", ["support_value_evals", "support_point_evals"])
def test_ledger_support_budgets_share_public_stop_reason(kind):
    exc = BudgetExceeded(kind, 0, 0, 1)
    assert connectivity._budget_stop_reason(exc) == "support_budget_exhausted"


def test_pre_rotated_hidden_narrow_gate_is_not_lost_by_upper_bound():
    width = 0.42
    scene = single_door(
        width, wall_t=0.18, workspace=(-2.0, 2.0, -1.2, 1.2),
    )
    base = ellipse_robot(0.5, 0.2).supports[0]
    alpha = -np.pi / 16.0
    rotation = np.array([
        [np.cos(alpha), -np.sin(alpha)],
        [np.sin(alpha), np.cos(alpha)],
    ])
    body = GaussianSupport2D(
        mean=base.mean,
        covariance=rotation @ base.covariance @ rotation.T,
        level=base.level,
        primitive_id=base.primitive_id,
    )
    robot = RobotModel2D((body,), name="pre_rotated_hidden_gate")
    cfg = make_cfg(
        mode="theorem", eps_pair=2e-2, theta_min=2e-3,
        initial_intervals=8,
    )
    oracles, decomposition = _compile(scene, robot, cfg)

    report = certified_connectivity_intervals(
        scene, robot, cfg, oracles, decomposition,
        (-1.2, 0.0), (1.2, 0.0), max_refinements=0,
    )
    half_angle = gate_half_angle(0.5, 0.2, width)
    true_angles = [
        theta for theta in np.linspace(0.0, TWO_PI, 4000, endpoint=False)
        if abs(float(np.remainder(theta + alpha + np.pi / 2, np.pi)
                     - np.pi / 2)) < half_angle - 1e-4
    ]

    assert true_angles
    assert all(_contains(report.theta_possible, theta)
               for theta in true_angles)
    assert report.unresolved
    assert report.stop_reason == "refinement_budget_exhausted"


def _component(geometry):
    return SimpleNamespace(geometry=geometry)


def test_safe_boundary_and_touching_upper_components_both_abstain(monkeypatch):
    slab = SimpleNamespace(
        interval=Interval(0.0, 1.0), slab_id=0, depth=0,
    )

    tangent_cover = SimpleNamespace(
        D_safe=(_component(box(0.0, 0.0, 1.0, 1.0)),),
        D_possible=(_component(box(-1.0, -1.0, 2.0, 2.0)),),
    )
    monkeypatch.setattr(connectivity, "_certified_cover",
                        lambda unused: tangent_cover)
    tangent = classify_translation_interval(
        slab, (0.0, 0.5), (0.5, 0.5),
    )
    assert tangent.verdict is ConnectivityVerdict.UNKNOWN

    closure_cover = SimpleNamespace(
        D_safe=(),
        D_possible=(
            _component(box(0.0, 0.0, 1.0, 1.0)),
            _component(box(1.0, 0.0, 2.0, 1.0)),
        ),
    )
    monkeypatch.setattr(connectivity, "_certified_cover",
                        lambda unused: closure_cover)
    closure = classify_translation_interval(
        slab, (0.5, 0.5), (1.5, 0.5),
    )
    assert closure.verdict is ConnectivityVerdict.UNKNOWN
    assert closure.reason == "lower_and_upper_connectivity_bounds_disagree"


def test_periodic_unknown_runs_merge_and_form_nonisolating_bracket(monkeypatch):
    edges = np.linspace(0.0, TWO_PI, 5)
    verdicts = (
        ConnectivityVerdict.UNKNOWN,
        ConnectivityVerdict.OPEN,
        ConnectivityVerdict.CLOSED,
        ConnectivityVerdict.UNKNOWN,
    )
    slabs = [SimpleNamespace(
        interval=Interval(float(edges[i]), float(edges[i + 1])),
        slab_id=i, depth=0,
    ) for i in range(4)]
    decomposition = SimpleNamespace(slabs=slabs, revision=0)
    cfg = SimpleNamespace(
        query=SimpleNamespace(
            max_refinement_rounds=0,
            max_support_calls=100,
            max_wall_seconds=10.0,
        ),
        orientation=SimpleNamespace(theta_min=1e-3, max_depth=10),
    )

    def classify(slab, start, goal):
        return ConnectivityInterval(
            interval=slab.interval,
            verdict=verdicts[slab.slab_id],
            slab_id=slab.slab_id,
            depth=slab.depth,
            reason="injected_test_verdict",
        )

    monkeypatch.setattr(connectivity, "classify_translation_interval",
                        classify)
    monkeypatch.setattr(connectivity, "_compiler_input_failures",
                        lambda *args: ())
    monkeypatch.setattr(
        connectivity, "validate_candidate_oracles",
        lambda scene, robot, workspace, oracles: list(oracles),
    )
    report = certified_connectivity_intervals(
        None, None, cfg, (), decomposition,
        (0.0, 0.0), (1.0, 0.0), max_refinements=0,
    )

    assert len(report.unresolved) == 1
    unresolved = report.unresolved[0].interval
    assert unresolved.lo == edges[3]
    assert unresolved.hi == edges[1] + TWO_PI
    assert len(report.transition_brackets) == 1
    bracket = report.transition_brackets[0]
    assert bracket.interval == unresolved
    assert bracket.minimum_transitions == 1
    assert not bracket.isolated


@pytest.mark.parametrize(
    ("theta_min", "max_depth", "expected"),
    [
        (TWO_PI, 10, "theta_min_reached"),
        (0.0, 0, "max_depth_reached"),
    ],
)
def test_resolution_limits_leave_full_s1_unresolved(
        monkeypatch, theta_min, max_depth, expected):
    edges = np.linspace(0.0, TWO_PI, 3)
    slabs = [SimpleNamespace(
        interval=Interval(float(edges[i]), float(edges[i + 1])),
        slab_id=i, depth=0,
    ) for i in range(2)]
    decomposition = SimpleNamespace(slabs=slabs, revision=0)
    cfg = SimpleNamespace(
        query=SimpleNamespace(
            max_refinement_rounds=4,
            max_support_calls=100,
            max_wall_seconds=10.0,
        ),
        orientation=SimpleNamespace(
            theta_min=theta_min, max_depth=max_depth,
        ),
    )

    monkeypatch.setattr(
        connectivity,
        "classify_translation_interval",
        lambda slab, start, goal: ConnectivityInterval(
            slab.interval, ConnectivityVerdict.UNKNOWN,
            slab.slab_id, slab.depth, "injected_unknown",
        ),
    )
    monkeypatch.setattr(connectivity, "_compiler_input_failures",
                        lambda *args: ())
    monkeypatch.setattr(
        connectivity, "validate_candidate_oracles",
        lambda scene, robot, workspace, oracles: list(oracles),
    )
    report = certified_connectivity_intervals(
        None, None, cfg, (), decomposition,
        (0.0, 0.0), (1.0, 0.0), max_refinements=4,
    )

    assert report.stop_reason == expected
    assert report.theta_safe == ()
    assert report.theta_possible == (Interval(0.0, TWO_PI),)
    assert report.unresolved[0].interval == Interval(0.0, TWO_PI)


def test_injected_interval_certificate_failure_remains_unknown(monkeypatch):
    real_builder = slab_builder.build_interval_pair_certificate
    failed = False

    def fail_once(*args, **kwargs):
        nonlocal failed
        if not failed:
            failed = True
            raise FloatingPointError("injected interval certificate failure")
        return real_builder(*args, **kwargs)

    monkeypatch.setattr(
        slab_builder, "build_interval_pair_certificate", fail_once,
    )
    scene = single_obstacle(
        r=0.3, workspace=(-2.0, 2.0, -1.5, 1.5),
    )
    robot = ellipse_robot(0.2, 0.1)
    cfg = make_cfg(
        mode="theorem", eps_pair=2e-2, theta_min=0.1,
        initial_intervals=4,
    )
    oracles, decomposition = _compile(scene, robot, cfg)
    report = certified_connectivity_intervals(
        scene, robot, cfg, oracles, decomposition,
        (-1.0, 0.0), (1.0, 0.0), max_refinements=0,
    )

    assert failed
    failed_intervals = [
        item for item in report.intervals
        if item.reason == "interval_cover_not_certified"
    ]
    assert len(failed_intervals) == 1
    failed_midpoint = failed_intervals[0].interval.midpoint
    assert not _contains(report.theta_safe, failed_midpoint)
    assert _contains(report.theta_possible, failed_midpoint)


def test_lower_exceeding_upper_is_an_internal_invariant_error(monkeypatch):
    with pytest.raises(
            ConnectivityIntervalInvariantError,
            match="theta_safe_not_subset_theta_possible"):
        validate_connectivity_bounds(
            (Interval(0.0, 1.0),), (Interval(2.0, 3.0),),
        )

    cover = SimpleNamespace(
        D_safe=(_component(box(0.0, 0.0, 1.0, 1.0)),),
        D_possible=(
            _component(box(0.1, 0.1, 0.4, 0.9)),
            _component(box(0.6, 0.1, 0.9, 0.9)),
        ),
    )
    monkeypatch.setattr(connectivity, "_certified_cover",
                        lambda unused: cover)
    slab = SimpleNamespace(
        interval=Interval(0.0, 1.0), slab_id=9, depth=0,
    )
    with pytest.raises(
            ConnectivityIntervalInvariantError,
            match="lower_exceeds_upper"):
        classify_translation_interval(
            slab, (0.25, 0.5), (0.75, 0.5),
        )
