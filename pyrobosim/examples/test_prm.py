#!/usr/bin/env python3
import os

from pyrobosim.core.yaml import WorldYamlLoader
from pyrobosim.navigation.prm import PRMPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose

# Load a test world.
data_folder = get_data_folder()
loader = WorldYamlLoader()
w = loader.from_yaml(os.path.join(data_folder, "test_world.yaml"))


def test_prm():
    """ Creates a PRM planner and plans """
    prm = PRMPlanner(w, max_nodes=100, max_connection_dist=1.5)
    start = Pose(x=-0.5, y=-0.5)
    goal = Pose(x=3.0, y=3.0)
    w.robot.set_pose(start)
    w.robot.set_path_planner(prm)
    w.current_path = w.robot.plan_path(start, goal)
    prm.print_metrics()


if __name__=="__main__":
    test_prm()

    import sys
    from pyrobosim.gui.main import PyRoboSimGUI
    app = PyRoboSimGUI(w, sys.argv)
    sys.exit(app.exec_())
    