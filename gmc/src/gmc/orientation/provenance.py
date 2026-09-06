"""Runtime provenance binding for orientation decompositions.

An interval cover is a theorem about one exact scene, robot, configuration,
and canonical candidate-pair set.  Reusing the resulting slabs with another
input invalidates that theorem even when the replacement happens to have the
same workspace bounds.  This module keeps that ownership explicit without
introducing a serialized hash protocol: in-memory certified objects are bound
to the immutable model/config objects and to the exact support objects used by
their pair oracles.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True, slots=True)
class DecompositionBinding:
    """Exact in-memory inputs that own a slab decomposition."""

    scene: object = field(repr=False, compare=False)
    robot: object = field(repr=False, compare=False)
    cfg: object = field(repr=False, compare=False)
    workspace_wkb: bytes = field(repr=False)
    scene_supports: tuple = field(repr=False, compare=False)
    robot_supports: tuple = field(repr=False, compare=False)
    oracle_records: tuple = field(repr=False, compare=False)

    @classmethod
    def capture(cls, scene, robot, cfg, oracles) -> "DecompositionBinding":
        return cls(
            scene=scene,
            robot=robot,
            cfg=cfg,
            workspace_wkb=bytes(scene.workspace.wkb),
            scene_supports=tuple(scene.supports),
            robot_supports=tuple(robot.supports),
            oracle_records=tuple(
                (oracle.pair_id, oracle.scene, oracle.body)
                for oracle in oracles
            ),
        )

    def mismatch_reasons(self, scene, robot, cfg, oracles) -> tuple[str, ...]:
        reasons = []
        if scene is not self.scene:
            reasons.append("scene_identity")
        if robot is not self.robot:
            reasons.append("robot_identity")
        if cfg is not self.cfg:
            reasons.append("config_identity")
        if bytes(scene.workspace.wkb) != self.workspace_wkb:
            reasons.append("workspace_geometry")

        current_scene_supports = tuple(scene.supports)
        if (len(current_scene_supports) != len(self.scene_supports)
                or any(actual is not expected for actual, expected in zip(
                    current_scene_supports, self.scene_supports))):
            reasons.append("scene_support_identity")
        current_robot_supports = tuple(robot.supports)
        if (len(current_robot_supports) != len(self.robot_supports)
                or any(actual is not expected for actual, expected in zip(
                    current_robot_supports, self.robot_supports))):
            reasons.append("robot_support_identity")

        supplied = tuple(
            (oracle.pair_id, oracle.scene, oracle.body) for oracle in oracles
        )
        if (len(supplied) != len(self.oracle_records)
                or any(
                    pair_id != expected_pair_id
                    or scene_support is not expected_scene_support
                    or body_support is not expected_body_support
                    for (pair_id, scene_support, body_support),
                    (expected_pair_id, expected_scene_support,
                     expected_body_support)
                    in zip(supplied, self.oracle_records)
                )):
            reasons.append("candidate_pair_identity")
        return tuple(reasons)


def capture_decomposition_binding(scene, robot, cfg, oracles):
    """Create the binding stored by :func:`build_slabs`."""
    return DecompositionBinding.capture(scene, robot, cfg, tuple(oracles))


def decomposition_binding_failures(decomposition, scene, robot, cfg,
                                   oracles) -> tuple[str, ...]:
    """Return fail-closed provenance mismatches for a compiled decomposition."""
    binding = getattr(decomposition, "input_binding", None)
    if not isinstance(binding, DecompositionBinding):
        return ("decomposition_input_binding_missing",)
    return binding.mismatch_reasons(scene, robot, cfg, tuple(oracles))


def validate_decomposition_binding(decomposition, scene, robot, cfg,
                                   oracles) -> None:
    """Raise when slabs are not owned by the supplied compiler inputs."""
    failures = decomposition_binding_failures(
        decomposition, scene, robot, cfg, oracles,
    )
    if failures:
        raise ValueError(
            "orientation decomposition provenance does not match compiler "
            "inputs: " + ", ".join(failures)
        )


__all__ = [
    "DecompositionBinding",
    "capture_decomposition_binding",
    "decomposition_binding_failures",
    "validate_decomposition_binding",
]
