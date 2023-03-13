#!/usr/bin/env python3
import os

from pyrobosim.core.yaml import WorldYamlLoader
from pyrobosim.gui.main import start_gui
from pyrobosim.navigation.prm_grid import PRMGridPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose

# Load a test world.
data_folder = get_data_folder()
loader = WorldYamlLoader()
world = loader.from_yaml(os.path.join(data_folder, "test_world.yaml"))


def test_prm():
    """Creates a PRM planner and plans"""
    prm = PRMGridPlanner(world, max_nodes=100, max_connection_dist=1.5)
    start = Pose(x=-0.5, y=-0.5)
    goal = Pose(x=3.0, y=3.0)

    robot = world.robots[0]
    robot.set_pose(start)
    robot.set_path_planner(prm)
    robot.current_path = robot.plan_path(start, goal)
    # prm.print_metrics()
    print(robot.current_path)


if __name__ == "__main__":
    test_prm()
    start_gui(world)
