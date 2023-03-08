#!/usr/bin/env python3
import os

from pyrobosim.core.yaml import WorldYamlLoader
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose

# Load a test world.
data_folder = get_data_folder()
loader = WorldYamlLoader()
world = loader.from_yaml(os.path.join(data_folder, "test_world.yaml"))


def test_rrt():
    """Creates an RRT planner and plans"""
    rrt = RRTPlanner(world, bidirectional=True, rrt_connect=False, rrt_star=True)
    start = Pose(x=-0.5, y=-0.5)
    goal = Pose(x=3.0, y=3.0)

    robot = world.robots[0]
    robot.set_pose(start)
    robot.set_path_planner(rrt)
    robot.current_path = robot.plan_path(start, goal)
    rrt.print_metrics()


if __name__ == "__main__":
    test_rrt()

    import sys
    from pyrobosim.gui.main import start_gui

    start_gui(world, sys.argv)
