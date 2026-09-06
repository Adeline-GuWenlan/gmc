"""Robot model I/O (Guide §4.2)."""
from pathlib import Path

import numpy as np
import yaml

from ..types import GaussianSupport2D, RobotModel2D


def save_robot(robot: RobotModel2D, path: str | Path) -> None:
    data = {
        "name": robot.name,
        "supports": [
            {"primitive_id": s.primitive_id,
             "mean": [float(s.mean[0]), float(s.mean[1])],
             "covariance": [[float(s.covariance[0, 0]), float(s.covariance[0, 1])],
                            [float(s.covariance[1, 0]), float(s.covariance[1, 1])]],
             "level": float(s.level)}
            for s in robot.supports
        ],
    }
    Path(path).write_text(yaml.safe_dump(data, sort_keys=False))


def load_robot(path: str | Path) -> RobotModel2D:
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict):
        raise ValueError("robot root must be a mapping")
    supports = tuple(
        GaussianSupport2D(mean=np.array(s["mean"]),
                          covariance=np.array(s["covariance"]),
                          level=s["level"],
                          primitive_id=s["primitive_id"])
        for s in data["supports"])
    return RobotModel2D(supports=supports, name=data["name"])


def ellipse_robot(a: float, b: float, level: float = 1.0,
                  name: str = "ellipse_robot") -> RobotModel2D:
    """Single-body ellipse robot with support semi-axes exactly (a, b)."""
    a, b, level = float(a), float(b), float(level)
    if (not np.all(np.isfinite([a, b, level]))
            or a <= 0.0 or b <= 0.0 or level <= 0.0):
        raise ValueError("ellipse axes and level must be finite and positive")
    cov = np.diag([(a / level) ** 2, (b / level) ** 2])
    return RobotModel2D(
        supports=(GaussianSupport2D(mean=np.zeros(2), covariance=cov,
                                    level=level, primitive_id=0),),
        name=name)
