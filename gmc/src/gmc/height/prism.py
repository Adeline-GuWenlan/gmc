"""Robots as vertical prisms: a 2D footprint over a height band (spec §3.3)."""
from dataclasses import dataclass

from ..io.robot_io import ellipse_robot
from ..types import RobotModel2D


@dataclass(frozen=True)
class PrismRobot:
    footprint: RobotModel2D
    z_lo: float | None
    z_hi: float | None
    name: str

    @property
    def is_2d(self) -> bool:
        return self.z_lo is None

    def band_abs(self, z_floor: float) -> tuple[float, float]:
        if self.is_2d:
            raise ValueError(f"{self.name} is a 2D-only robot with no band")
        return (float(z_floor) + self.z_lo, float(z_floor) + self.z_hi)

    def max_radius(self) -> float:
        return float(self.footprint.max_rotational_radius())


def ellipse_prism(a, b, z_lo, z_hi, name) -> PrismRobot:
    return PrismRobot(ellipse_robot(a, b, name=name), z_lo, z_hi, name)


def robot_table(z_c: float = 1.20) -> dict[str, PrismRobot]:
    return {
        "ellipse_toy": ellipse_prism(0.50, 0.20, None, None, "ellipse_toy"),
        "sweeper": ellipse_prism(0.175, 0.175, 0.02, 0.10, "sweeper"),
        "quadruped": ellipse_prism(0.35, 0.16, 0.02, 0.45, "quadruped"),
        "cylinder": ellipse_prism(0.30, 0.30, 0.02, 1.75, "cylinder"),
        "uav": ellipse_prism(0.25, 0.25, z_c - 0.10, z_c + 0.10, "uav"),
    }
