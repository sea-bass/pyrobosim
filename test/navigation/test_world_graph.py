#!/usr/bin/env python3

"""Unit tests for the World Graph planner"""

import os

from pyrobosim.core import WorldYamlLoader
from pyrobosim.navigation import PathPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


def test_world_graph_default():
    """Tests planning with default world graph planner settings."""
    world = WorldYamlLoader().from_yaml(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "world": world,
    }
    planner = PathPlanner("world_graph", **planner_config)
    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=2.5, y=3.0)

    path = planner.plan(start, goal)
    assert len(path.poses) >= 2
    assert path.poses[0] == start
    assert path.poses[-1] == goal


def test_world_graph_short_connection_distance():
    """Tests planning with short connection distance, which should fail."""
    world = WorldYamlLoader().from_yaml(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "world": world,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 1.0,
    }
    planner = PathPlanner("world_graph", **planner_config)
    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=2.5, y=3.0)

    path = planner.plan(start, goal)
    assert len(path.poses) == 0
