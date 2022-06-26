#!/usr/bin/env python3

"""
Test script for PDDLStream planning with navigation streams.
"""
import os
import sys

from pyrobosim.core.robot import Robot
from pyrobosim.core.room import Room
from pyrobosim.core.world import World
from pyrobosim.navigation.execution import ConstantVelocityExecutor
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.planning.pddlstream.planner import PDDLStreamPlanner
from pyrobosim.planning.pddlstream.utils import get_default_domains_folder
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


def create_test_world(add_hallway=True):
    w = World()

    # Set the location and object metadata
    data_folder = get_data_folder()
    w.set_metadata(
        locations=os.path.join(data_folder, "example_location_data.yaml"),
        objects=os.path.join(data_folder, "example_object_data.yaml"),
    )

    # Add rooms
    main_room_coords = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    main_room = Room(main_room_coords, name="main_room", color=[1, 0, 0])
    w.add_room(main_room)
    unreachable_coords = [(2, -1), (4, -1), (4, 1), (2, 1)]
    unreachable_room = Room(unreachable_coords, name="unreachable", color=[0, 0, 1])
    w.add_room(unreachable_room)
    goal_room_coords = [(2, 2), (4, 2), (4, 4), (2, 4)]
    goal_room = Room(goal_room_coords, name="goal_room", color=[0, 0.6, 0])
    w.add_room(goal_room)

    # Add hallway, if enabled.
    if add_hallway:
        hallway_points = [(0.0, 0.0), (0.0, 5.0), (3.0, 5.0), (3.0, 3.0)]
        w.add_hallway(
            "main_room",
            "goal_room",
            width=0.7,
            conn_method="points",
            conn_points=hallway_points,
        )

    # Add locations and objects
    table0 = w.add_location("table", "unreachable", Pose(x=3.5, y=-0.25, yaw=0.0))
    w.add_object("apple", table0)
    table1 = w.add_location("table", "goal_room", Pose(x=3.5, y=2.75, yaw=0.0))
    w.add_object("apple", table1)

    # Add a robot
    r = Robot(radius=0.1, path_executor=ConstantVelocityExecutor())
    w.add_robot(r, loc="main_room")

    # Create a search graph and motion planner
    w.create_search_graph(max_edge_dist=3.0, collision_check_dist=0.05)
    rrt = RRTPlanner(w, bidirectional=True, rrt_star=True)
    w.robot.set_path_planner(rrt)

    return w


def start_planner(world, domain_name="03_nav_stream", interactive=False):
    domain_folder = os.path.join(get_default_domains_folder(), domain_name)
    planner = PDDLStreamPlanner(world, domain_folder)

    get = lambda entity: world.get_entity_by_name(entity)
    goal_literals = [("Has", get("robot"), "apple")]

    if interactive:
        input("Press Enter to start planning.")
    plan = planner.plan(goal_literals, focused=True, verbose=interactive)
    if interactive:
        world.robot.execute_plan(plan, blocking=True)
    return plan


#####################
# ACTUAL UNIT TESTS #
#####################
def test_symbolic_plan():
    w = create_test_world()
    plan = start_planner(w, domain_name="02_derived")
    assert plan is not None


def test_stream_plan_no_hallway():
    w = create_test_world(add_hallway=False)
    plan = start_planner(w, domain_name="03_nav_stream")
    assert plan is None


def test_stream_plan_with_hallway():
    w = create_test_world(add_hallway=True)
    plan = start_planner(w, domain_name="03_nav_stream")
    assert plan is not None


if __name__ == "__main__":
    w = create_test_world(add_hallway=True)

    # Start task and motion planner in separate thread.
    import threading

    # domain_name = "02_derived" # Will get infeasible plan
    domain_name = "03_nav_stream"  # Will get feasible plan
    t = threading.Thread(target=start_planner, args=(w, domain_name, True))
    t.start()

    from pyrobosim.gui.main import PyRoboSimGUI

    app = PyRoboSimGUI(w, sys.argv)
    sys.exit(app.exec_())
