"""
Provides a registry of path planners for instantiation and saving to file.

If adding your own path planners, you should register them in this file.
"""

from typing import Any

from .types import PathPlanner

# Planners
from .a_star import AStarPlanner
from .rrt import RRTPlanner
from .prm import PRMPlanner
from .world_graph import WorldGraphPlanner

PATH_PLANNERS_MAP = {
    "astar": AStarPlanner,
    "rrt": RRTPlanner,
    "prm": PRMPlanner,
    "world_graph": WorldGraphPlanner,
}


def get_planner_class(planner_type: str) -> Any:
    """
    Helper function that returns a path planner.

    :param planner_type: The type of path planner.
    :return: The class corresponding to the planner type specified.
    :raises ValueError: if the planner type is invalid.
    """
    if planner_type not in PATH_PLANNERS_MAP:
        raise ValueError(f"{planner_type} is not a supported planner type.")

    return PATH_PLANNERS_MAP[planner_type]


def get_planner_string(planner: PathPlanner) -> str:
    """
    Helper function that returns the planner string from a planner instance.

    :param planner: The path planner instance.
    :return: The string corresponding to the entry in `PATH_PLANNERS_MAP`.
    :raises ValueError: if the planner has no registered key.
    """
    for planner_str, planner_type in PATH_PLANNERS_MAP.items():
        if isinstance(planner, planner_type):
            return planner_str
    raise ValueError(
        f"Could not find registered key for planner type: {type(planner).__name__}"
    )
