"""T-Event-01 (Guide §8.4): single-door closure events are bracketed by
uncertain slabs containing the analytic critical angles."""
import numpy as np

from gmc.io.robot_io import ellipse_robot
from gmc.orientation.slab_builder import build_slabs
from gmc.spatial.bvh import candidate_pairs
from gmc.synth import gate_half_angle, single_door

from ..conftest import SMALL_WS, make_cfg


class TestEventIsolation:
    def test_uncertain_slabs_bracket_analytic_events(self):
        cfg = make_cfg(theta_min=5e-3, initial_intervals=8)
        a, b, w = 0.5, 0.2, 0.55
        scene = single_door(w, workspace=SMALL_WS)
        robot = ellipse_robot(a, b)
        half = gate_half_angle(a, b, w)
        events = [half, np.pi - half, np.pi + half, 2 * np.pi - half]
        oracles = candidate_pairs(scene, robot, scene.workspace)
        dec = build_slabs(scene, robot, cfg, oracles)
        uncertain = [s.interval for s in dec.slabs if s.kind == "uncertain"]
        assert uncertain, "no uncertain slabs found at all"
        tol = cfg.pair_approx.eps_pair * 6.0   # polygonization shift allowance
        for ev in events:
            hit = any(iv.lo - tol <= ev <= iv.hi + tol for iv in uncertain)
            assert hit, (f"analytic event {np.degrees(ev):.2f}deg not "
                         f"bracketed; uncertain="
                         f"{[(np.degrees(i.lo), np.degrees(i.hi)) for i in uncertain]}")

    def test_regular_slabs_have_stable_counts(self):
        cfg = make_cfg(theta_min=5e-3, initial_intervals=8)
        scene = single_door(0.55, workspace=SMALL_WS)
        robot = ellipse_robot(0.5, 0.2)
        dec = build_slabs(scene, robot, cfg)
        for s in dec.slabs:
            if s.kind == "regular":
                assert (len(s.left_slice.D_safe) == len(s.mid_slice.D_safe)
                        == len(s.right_slice.D_safe))
