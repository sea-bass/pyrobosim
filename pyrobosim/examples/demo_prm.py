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


def test_prm():
    """Creates a PRM planner and plans"""
    planner_config = {
        "grid": None,
        "world": world,
        "max_nodes": 100,
        "max_connection_dist": 1.0,
        "compress_path": False,
    }
    prm = PathPlanner("prm", planner_config)
    start = Pose(x=-0.5, y=-0.5)
    goal = Pose(x=3.0, y=3.0)

    robot = world.robots[0]
    robot.set_pose(start)
    robot.set_path_planner(prm)
    robot.current_path = robot.plan_path(start, goal)
    prm.info()


if __name__ == "__main__":
    test_prm()
    start_gui(world)
