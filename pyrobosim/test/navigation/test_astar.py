#!/usr/bin/env python3

"""Unit tests for A-star planner"""

import os

from pyrobosim.core import WorldYamlLoader
from pyrobosim.navigation import OccupancyGrid, PathPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


def test_astar():
    """Test A* planner with and without path compression"""

    world = WorldYamlLoader().from_yaml(
        os.path.join(get_data_folder(), "test_world.yaml")
    )

    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=2.5, y=3.0)

    robot = world.robots[0]
    planner_config = {
        "grid": OccupancyGrid.from_world(
            world, resolution=0.05, inflation_radius=1.5 * robot.radius
        ),
        "diagonal_motion": True,
        "heuristic": "euclidean",
        "compress_path": False,
    }
    astar_planner = PathPlanner("astar", **planner_config)

    full_path = astar_planner.plan(start, goal).poses

    assert len(full_path) >= 2
    assert full_path[0] == start
    assert full_path[-1] == goal

    # Plan for same start and goal with path compression enabled
    planner_config = {
        "grid": OccupancyGrid.from_world(
            world, resolution=0.05, inflation_radius=1.5 * robot.radius
        ),
        "diagonal_motion": True,
        "heuristic": "euclidean",
        "compress_path": True,
    }
    astar_planner = PathPlanner("astar", **planner_config)

    compressed_path = astar_planner.plan(start, goal).poses

    assert len(compressed_path) >= 2
    assert len(compressed_path) <= len(full_path)
    assert compressed_path[0] == start
    assert compressed_path[-1] == goal
