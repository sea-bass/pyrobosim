#!/usr/bin/env python
import os

from pyrobosim.core.yaml import WorldYamlLoader
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose

# Load a world
data_folder = get_data_folder()
loader = WorldYamlLoader()
w = loader.from_yaml(os.path.join(data_folder, "test_world.yaml"))


def test_rrt():
    # Create an RRT planner and plan
    rrt = RRTPlanner(w, bidirectional=True, rrt_connect=False, rrt_star=True)
    start = Pose(x=-0.5, y=-0.5)
    goal = Pose(x=3.0, y=3.0)
    w.robot.set_pose(start)
    w.robot.set_path_planner(rrt)
    w.current_path = w.robot.plan_path(start, goal)
    rrt.print_metrics()


if __name__=="__main__":
    test_rrt()

    import sys
    from pyrobosim.gui.main import PyRoboSimGUI
    app = PyRoboSimGUI(w, sys.argv)
    sys.exit(app.exec_())
    