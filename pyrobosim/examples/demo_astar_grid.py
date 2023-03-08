#!/usr/bin/env python3
import os

from pyrobosim.core.yaml import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim.navigation.astar_grid import AStarGridPlanner

# Load a test world.
data_folder = get_data_folder()
loader = WorldYamlLoader()
world = loader.from_yaml(os.path.join(data_folder, "test_world.yaml"))


def test_astar_grid():
    robot = world.robots[0]

    astar = AStarGridPlanner(
        world=world,
        resolution=0.05,
        inflation_radius=robot.radius,
        distance_metric="manhattan",
        diagonal_motion=True,
    )
    start = Pose(x=-0.5, y=-0.5)
    goal = Pose(x=3.0, y=3.0)

    robot.set_pose(start)
    robot.set_path_planner(astar)
    robot.current_path = robot.plan_path(start, goal)
    astar.print_metrics()


if __name__ == "__main__":
    test_astar_grid()

    import sys
    from pyrobosim.gui.main import start_gui

    start_gui(world, sys.argv)
