"""T-Slice-04 (Guide §14.4): pair graph unchanged, workspace-boundary pinch
changes complement connectivity."""
from gmc.io.robot_io import ellipse_robot
from gmc.spatial.slice_compiler import build_slice
from gmc.synth import boundary_pinch


class TestBoundaryPinch:
    def test_pinch_splits_free_space(self, cfg):
        robot = ellipse_robot(0.25, 0.25)   # center-only workspace semantics:
        # top passage needs gap > disc_r + robot_r = 0.33
        open_scene = boundary_pinch(gap=1.0)     # same wall, higher ceiling
        closed_scene = boundary_pinch(gap=0.25)
        sl_open = build_slice(open_scene, robot, 0.0, cfg, build_nerve=True)
        sl_closed = build_slice(closed_scene, robot, 0.0, cfg,
                                build_nerve=True)
        # identical scene => identical obstacle-side pair graph
        assert sl_open.nerve.edges == sl_closed.nerve.edges
        assert sl_open.nerve.simplices == sl_closed.nerve.simplices
        # but complement connectivity differs via the workspace boundary
        assert len(sl_open.D_safe) == 1
        assert len(sl_closed.D_safe) == 2
        assert len(sl_closed.D_possible) == 2
