#!/usr/bin/env python3

"""Unit tests for the World Graph planner"""

import os
from pytest import LogCaptureFixture

from pyrobosim.core import WorldYamlLoader
from pyrobosim.navigation.world_graph import WorldGraphPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


def test_world_graph_default() -> None:
    """Tests planning with default world graph planner settings."""
    world = WorldYamlLoader().from_file(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner = WorldGraphPlanner()
    world.robots[0].set_path_planner(planner)

    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=2.5, y=3.0)
    path = planner.plan(start, goal)

    assert len(path.poses) >= 2
    assert path.poses[0] == start
    assert path.poses[-1] == goal


def test_world_graph_short_connection_distance(caplog: LogCaptureFixture) -> None:
    """Tests planning with short connection distance, which should fail."""
    world = WorldYamlLoader().from_file(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 1.0,
    }
    planner = WorldGraphPlanner(**planner_config)
    world.robots[0].set_path_planner(planner)

    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=2.5, y=3.0)
    path = planner.plan(start, goal)

    assert len(path.poses) == 0
    assert "Could not find a path from start to goal." in caplog.text
