#!/usr/bin/env python3

import os

from pyrobosim.core import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.navigation import AStarPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose

# Load a test world.
world_file = os.path.join(get_data_folder(), "test_world.yaml")
world = WorldYamlLoader().from_file(world_file)


def demo_astar():
    """Creates an occupancy grid based A* planner and plans a path."""
    robot = world.robots[0]
    planner_config = {
        "world": world,
        "grid_resolution": 0.05,
        "grid_inflation_radius": 1.5 * robot.radius,
        "diagonal_motion": True,
        "heuristic": "euclidean",
        "compress_path": False,
    }

    planner = AStarPlanner(**planner_config)
    start = Pose(x=-0.5, y=-0.5)
    goal = Pose(x=3.0, y=3.0)
    robot.set_pose(start)
    robot.set_path_planner(planner)
    path = robot.plan_path(start, goal)
    if path:
        path.print_details()


if __name__ == "__main__":
    demo_astar()
    start_gui(world)
