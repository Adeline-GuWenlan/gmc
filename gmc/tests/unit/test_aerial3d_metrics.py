"""Smoothness metrics shared by C1 and C2 (both methods, same definitions)."""
import numpy as np
import pytest

from gmc.aerial3d.metrics import bezier_ease5_jerk_sq, polyline_metrics, smooth_segments_metrics


def test_straight_line_has_no_turning():
    m = polyline_metrics([[0, 0, 0], [1, 0, 0], [3, 0, 0]])
    assert m["path_length_m"] == pytest.approx(3.)
    assert m["n_vertices"] == 3 and m["n_segments"] == 2
    assert m["total_turning_rad"] == pytest.approx(0.) and m["max_turning_rad"] == pytest.approx(0.)
    assert m["bending_energy_per_m"] == pytest.approx(0.)


def test_right_angle_turn():
    m = polyline_metrics([[0, 0, 0], [1, 0, 0], [1, 0, 2]])
    assert m["total_turning_rad"] == pytest.approx(np.pi / 2)
    assert m["max_turning_rad"] == pytest.approx(np.pi / 2)
    assert m["curvature_integral_rad"] == pytest.approx(np.pi / 2)
    assert m["vertical_travel_m"] == pytest.approx(2.)
    assert m["altitude_range_m"] == pytest.approx(2.)


def test_polygonal_circle_converges_to_the_continuous_curvature():
    R, n = 2., 720
    t = np.linspace(0, 2 * np.pi, n + 1)
    pts = np.c_[R * np.cos(t), R * np.sin(t), np.zeros_like(t)]
    pts = np.vstack([pts, pts[1]])  # close the turn at the seam
    m = polyline_metrics(pts)
    assert m["total_turning_rad"] == pytest.approx(2 * np.pi, rel=1e-3)
    assert m["bending_energy_per_m"] == pytest.approx(2 * np.pi / R, rel=1e-2)  # int kappa^2 ds


def test_zero_length_segments_are_ignored():
    m = polyline_metrics([[0, 0, 0], [0, 0, 0], [1, 0, 0], [1, 0, 0], [1, 1, 0]])
    assert m["n_vertices"] == 3
    assert m["total_turning_rad"] == pytest.approx(np.pi / 2)


def test_jerk_of_a_straight_eased_bezier_matches_the_closed_form():
    a, b, T = np.array([0., 0, 0]), np.array([2., 1, 0]), 3.
    control = np.array([a, a + (b - a) / 3, a + 2 * (b - a) / 3, b])
    # x(t) = a + (b-a) e(t/T), e quintic ease: int_0^T |x'''|^2 dt = 720 |b-a|^2 / T^5
    assert bezier_ease5_jerk_sq(control, T) == pytest.approx(720 * 5. / T ** 5, rel=1e-10)


def test_smoothed_segment_metrics_sum_segments():
    a, b, T = np.array([0., 0, 0]), np.array([1., 0, 0]), 2.
    control = np.array([a, a + (b - a) / 3, a + 2 * (b - a) / 3, b])
    seg = {"type": "cubic_bezier_ease5", "control_points_xyz": control.tolist(), "duration_s": T}
    m = smooth_segments_metrics([seg, seg])
    assert m["duration_s"] == pytest.approx(2 * T)
    assert m["integrated_squared_jerk"] == pytest.approx(2 * 720 / T ** 5, rel=1e-10)
    assert m["smooth_path_length_m"] == pytest.approx(2.)
