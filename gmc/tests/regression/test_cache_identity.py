"""Cache keys must not merge geometrically distinct floating-point inputs."""
import numpy as np
import pytest

from gmc.budget import WorkLedger
from gmc.geometry.support import PairOracle
from gmc.types import GaussianSupport2D, PairID


def _support(mean, pid):
    return GaussianSupport2D(
        mean=np.asarray(mean, dtype=float), covariance=np.eye(2),
        level=1.0, primitive_id=pid,
    )


def test_pair_frame_cache_does_not_quantize_close_angles():
    scene = _support([0.0, 0.0], 1)
    body = _support([1.0e16, 0.0], 2)
    cached = PairOracle(PairID(1, 2), scene, body)
    fresh = PairOracle(PairID(1, 2), scene, body)
    direction = np.array([[0.0, 1.0]])

    at_zero = cached.support_values(0.0, direction)[0]
    close_theta = 4.0e-16
    after_cache = cached.support_values(close_theta, direction)[0]
    independently = fresh.support_values(close_theta, direction)[0]

    assert at_zero == 2.0
    assert after_cache == independently
    assert abs(after_cache - at_zero) > 1.0


def test_pair_frame_is_exactly_periodic_even_with_large_body_offset():
    scene = _support([0.0, 0.0], 1)
    body = _support([1.0e16, 0.0], 2)
    direction = np.array([[0.0, 1.0]])
    values = []
    for theta in (0.0, 2.0 * np.pi, -2.0 * np.pi, 4.0 * np.pi):
        oracle = PairOracle(PairID(1, 2), scene, body)
        values.append(oracle.support_values(theta, direction)[0])
    assert values == [2.0, 2.0, 2.0, 2.0]


@pytest.mark.parametrize("bad", [
    np.array([0.0, 0.0]),
    np.array([np.nan, 1.0]),
    np.array([[1.0, 0.0, 0.0]]),
])
def test_support_oracle_rejects_degenerate_directions_without_nan(bad):
    oracle = PairOracle(PairID(1, 2), _support([0.0, 0.0], 1),
                        _support([0.0, 0.0], 2))
    with pytest.raises(ValueError):
        oracle.support_values(0.0, bad)
    with pytest.raises(ValueError):
        oracle.support_points(0.0, bad)


def test_pair_frame_cache_hits_are_accounted():
    ledger = WorkLedger()
    oracle = PairOracle(PairID(1, 2), _support([0.0, 0.0], 1),
                        _support([0.0, 0.0], 2), ledger=ledger)
    oracle.center(0.25)
    oracle.center(0.25)
    assert ledger.total("cache_hits") == 1
