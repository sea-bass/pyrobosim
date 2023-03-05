#!/usr/bin/env python3
import os

from pyrobosim.core.yaml import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim.navigation.astar_grid import AstarGrid

# Load a test world.
data_folder = get_data_folder()
loader = WorldYamlLoader()
w = loader.from_yaml(os.path.join(data_folder, "test_world.yaml"))


def test_astar_grid(resolution=0.05):
    astar = AstarGrid(
        w=w, resolution=resolution, inflation_radius=2 * w.robots[0].radius
    )
    start = Pose(x=-0.5, y=-0.5)
    goal = Pose(x=3.0, y=3.0)
    astar.plan(start, goal)
    robot = w.robots[0]
    print(robot.radius)
    robot.set_pose(start)
    robot.set_path_planner(astar)
    robot.current_path = robot.plan_path(start, goal)


if __name__ == "__main__":
    test_astar_grid()

    import sys
    from pyrobosim.gui.main import PyRoboSimGUI

    app = PyRoboSimGUI(w, sys.argv)
    sys.exit(app.exec_())
