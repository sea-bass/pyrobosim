""" Navigation utilities.

This module contains tools associated with navigation, including
localization, mapping, path planning, and path following.
"""

# Utilities
from .execution import *
from .occupancy_grid import *

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


def get_planner_class(planner_type):
    """
    Helper function that returns a path planner.

    :param planner_type: The type of path planner.
    :type planner_type: str
    :return: The class corresponding to the planner type specified.
    :rtype: PathPlanner
    :raises ValueError: if the planner type is invalid.
    """
    if planner_type not in PATH_PLANNERS_MAP:
        raise ValueError(f"{planner_type} is not a supported planner type.")

    return PATH_PLANNERS_MAP[planner_type]
