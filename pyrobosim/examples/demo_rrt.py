#!/usr/bin/env python3

import argparse

from pyrobosim.core import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose

# Load a test world.
world_file = get_data_folder() / "test_world.yaml"
world = WorldYamlLoader().from_file(world_file)


def test_rrt() -> None:
    """Creates an RRT planner and plans"""
    planner_config = {
        "bidirectional": True,
        "rrt_connect": True,
        "rrt_star": True,
        "compress_path": False,
    }
    rrt = RRTPlanner(**planner_config)
    start = Pose(x=-0.5, y=-0.5)
    goal = Pose(x=3.0, y=3.0)

    robot = world.robots[0]
    robot.set_pose(start)
    robot.set_path_planner(rrt)
    path = robot.plan_path(start, goal)
    if path:
        path.print_details()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="RRT planner demo.")
    parser.add_argument(
        "--web",
        action="store_true",
        help="Launch the browser-based web GUI instead of the Qt GUI.",
    )
    args = parser.parse_args()

    test_rrt()
    if args.web:
        from pyrobosim.web.app import run

        run(world)
    else:
        start_gui(world)
