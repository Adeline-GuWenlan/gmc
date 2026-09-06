"""Frozen compile/query API (P4a, round-5 review risk #2).

    atlas = compile_scene_robot(scene, robot, compile_config)
    path  = query_atlas(atlas, start, goal, query_config)

CONTRACTS (frozen now, implementations land in P4b):
  - compile_scene_robot NEVER receives start, goal, start-goal geometry,
    goal-side hints, or goal-conditioned ranking of any kind; passing such
    keys raises ValueError (enforced below, tested).
  - query_atlas performs NO gate discovery: atlas.atlas_hash() and
    atlas.compile_query_count must be identical before and after any
    number of queries with any goals (tests/test_atlas_api.py).
  - Every path returned by query_atlas passes the same conservative
    continuous certificate as today's kernel output.
"""
from .atlas_types import Atlas

_GOAL_KEYS = {"start", "goal", "goals", "start_pose", "goal_pose",
              "goal_radius", "goal_side", "start_goal_line",
              "goal_bias", "goal_ranking"}


def compile_scene_robot(scene, robot, compile_config=None):
    if compile_config:
        bad = _GOAL_KEYS & set(compile_config)
        if bad:
            raise ValueError(
                f"goal-independence contract violation: compile_config "
                f"contains {sorted(bad)}")
    raise NotImplementedError(
        "P4b: gate-branch compiler lands here (kernel wrapped as one "
        "branch producer, multi-hypothesis seeds, no goal input)")


def query_atlas(atlas, start, goal, query_config=None):
    if not isinstance(atlas, Atlas):
        raise TypeError("query_atlas requires a compiled Atlas")
    raise NotImplementedError(
        "P4b: min-plus routing over AtlasEdges + certified readout")
