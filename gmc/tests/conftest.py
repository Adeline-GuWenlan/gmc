import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from gmc.config import (Config, GeometryCfg, LoggingCfg, OrientationCfg,
                        PairApproxCfg, QueryCfg)


def make_cfg(mode="prototype", eps_pair=2e-3, theta_min=2e-3,
             initial_intervals=8) -> Config:
    return Config(
        length_unit="meter",
        geometry=GeometryCfg(workspace_precision=1e-8,
                             min_cov_eigenvalue=1e-10,
                             support_level_scene=2.0,
                             support_level_robot=1.0),
        pair_approx=PairApproxCfg(initial_directions=16, max_directions=128,
                                  eps_pair=eps_pair, certificate_mode=mode),
        orientation=OrientationCfg(initial_intervals=initial_intervals,
                                   theta_min=theta_min, max_depth=14),
        query=QueryCfg(max_support_calls=5_000_000, max_wall_seconds=360.0,
                       eps_clear=2e-3),
        logging=LoggingCfg(save_intermediate_geometry=False,
                           save_failed_cases=True),
        source_path="<test>")


@pytest.fixture
def cfg():
    return make_cfg()


@pytest.fixture
def cfg_theorem():
    return make_cfg(mode="theorem", eps_pair=1e-3)


SMALL_WS = (-2.2, 2.2, -1.6, 1.6)
