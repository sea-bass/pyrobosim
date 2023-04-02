#!/usr/bin/env python3
import os

from pyrobosim.core import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.navigation import PathPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose

# Load a test world.
data_folder = get_data_folder()
loader = WorldYamlLoader()
world = loader.from_yaml(os.path.join(data_folder, "test_world.yaml"))


def test_rrt():
    """Creates an RRT planner and plans"""
    planner_config = {
        "grid": None,
        "world": world,
        "bidirectional": True,
        "rrt_connect": True,
        "rrt_star": True,
        "compress_path": False,
    }
    rrt = PathPlanner("rrt", planner_config)
    start = Pose(x=-0.5, y=-0.5)
    goal = Pose(x=3.0, y=3.0)

    robot = world.robots[0]
    robot.set_pose(start)
    robot.set_path_planner(rrt)
    robot.current_path = robot.plan_path(start, goal)
    rrt.info()


if __name__ == "__main__":
    test_rrt()
    start_gui(world)
