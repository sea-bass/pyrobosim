#!/usr/bin/env python3

"""Unit tests for the RRT planner"""

import os

from pyrobosim.core import WorldYamlLoader
from pyrobosim.navigation import PathPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


def test_rrt_short_distance_connect():
    """Tests if direct connection works if goal is within max_connection_distance"""
    world = WorldYamlLoader().from_yaml(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "world": world,
        "bidirectional": False,
        "rrt_connect": False,
        "rrt_star": False,
    }
    rrt = PathPlanner("rrt", **planner_config)
    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=-1.6, y=3.0)

    path = rrt.plan(start, goal)
    assert len(path.poses) == 2
    assert path.poses[0] == start
    assert path.poses[1] == goal
