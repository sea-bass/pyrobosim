#!/usr/bin/env python3

"""
Test script for PDDLStream planning with manipulation streams.
"""
import os
import sys
import numpy as np

from pyrobosim.core.robot import Robot
from pyrobosim.core.room import Room
from pyrobosim.core.world import World
from pyrobosim.navigation.execution import ConstantVelocityExecutor
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.planning.pddlstream.planner import PDDLStreamPlanner
from pyrobosim.planning.pddlstream.utils import get_default_domains_folder
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


def create_test_world(add_alt_desk=True):
    w = World(object_radius=0.1)

    # Set the location and object metadata
    data_folder = get_data_folder()
    w.set_metadata(
        locations=os.path.join(data_folder, "example_location_data.yaml"),
        objects=os.path.join(data_folder, "example_object_data.yaml"),
    )

    # Add rooms
    home_coords = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    home = Room(home_coords, name="home", color=[1, 0, 0])
    w.add_room(home)
    storage_coords = [(2, -2), (5, -2), (5, 2), (2, 2)]
    storage = Room(storage_coords, name="storage", color=[0, 0, 1])
    w.add_room(storage)
    w.add_hallway(home, storage, width=0.75, conn_method="auto")

    # Add locations and objects
    table0 = w.add_location("table", "home", Pose(x=0.0, y=0.5, yaw=np.pi / 2))
    desk0 = w.add_location("desk", "storage", Pose(x=2.5, y=-1.5, yaw=0.0))
    if add_alt_desk:
        desk1 = w.add_location("desk", "storage", Pose(x=4.5, y=1.5, yaw=0.0))
    w.add_object("banana", table0)
    w.add_object("water", desk0, pose=Pose(x=2.4, y=-1.4, yaw=np.pi / 4.0))
    w.add_object("water", desk0, pose=Pose(x=2.575, y=-1.57, yaw=-np.pi / 4.0))

    # Add a robot
    r = Robot(radius=0.1, path_executor=ConstantVelocityExecutor())
    w.add_robot(r, loc="home")

    # Create a search graph and motion planner
    w.create_search_graph(max_edge_dist=3.0, collision_check_dist=0.05)
    rrt = RRTPlanner(w, bidirectional=True, rrt_star=True)
    w.robot.set_path_planner(rrt)

    return w


def start_planner(world, domain_name="04_nav_manip_stream", interactive=False):
    domain_folder = os.path.join(get_default_domains_folder(), domain_name)
    planner = PDDLStreamPlanner(world, domain_folder)

    goal_literals = [("Has", "desk", "banana")]

    if interactive:
        input("Press Enter to start planning.")
    plan = planner.plan(goal_literals, focused=True, verbose=interactive)
    if interactive:
        world.robot.execute_plan(plan, blocking=True)
    return plan


#####################
# ACTUAL UNIT TESTS #
#####################
def test_plan_single_desk():
    w = create_test_world(add_alt_desk=False)
    plan = start_planner(w)
    assert plan is not None


def test_plan_double_desk():
    w = create_test_world(add_alt_desk=True)
    plan = start_planner(w)
    assert plan is not None


if __name__ == "__main__":
    w = create_test_world(add_alt_desk=False)

    # Start task and motion planner in separate thread.
    import threading

    domain_name = "04_nav_manip_stream"
    t = threading.Thread(target=start_planner, args=(w, domain_name, True))
    t.start()

    from pyrobosim.gui.main import PyRoboSimGUI

    app = PyRoboSimGUI(w, sys.argv)
    sys.exit(app.exec_())
