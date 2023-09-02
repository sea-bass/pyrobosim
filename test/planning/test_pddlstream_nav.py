#!/usr/bin/env python3

"""
Tests for PDDLStream planning with navigation streams.
"""
import os
import threading

from pyrobosim.core import Robot, World
from pyrobosim.gui import start_gui
from pyrobosim.navigation import ConstantVelocityExecutor, PathPlanner
from pyrobosim.planning import PDDLStreamPlanner
from pyrobosim.planning.pddlstream.utils import get_default_domains_folder
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


def create_test_world(add_hallway=True):
    world = World()

    # Set the location and object metadata
    data_folder = get_data_folder()
    world.set_metadata(
        locations=os.path.join(data_folder, "example_location_data.yaml"),
        objects=os.path.join(data_folder, "example_object_data.yaml"),
    )

    # Add rooms
    main_room_coords = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    world.add_room(name="main_room", footprint=main_room_coords, color=[1, 0, 0])
    unreachable_coords = [(2, -1), (4, -1), (4, 1), (2, 1)]
    world.add_room(name="unreachable", footprint=unreachable_coords, color=[0, 0, 1])
    goal_room_coords = [(2, 2), (4, 2), (4, 4), (2, 4)]
    world.add_room(name="goal_room", footprint=goal_room_coords, color=[0, 0.6, 0])

    # Add hallway, if enabled.
    if add_hallway:
        hallway_points = [(0.0, 0.0), (0.0, 5.0), (3.0, 5.0), (3.0, 3.0)]
        world.add_hallway(
            room_start="main_room",
            room_end="goal_room",
            width=0.7,
            conn_method="points",
            conn_points=hallway_points,
        )

    # Add locations and objects
    table0 = world.add_location(
        category="table", parent="unreachable", pose=Pose(x=3.5, y=-0.25, z=0.0)
    )
    world.add_object(category="apple", parent=table0)
    table1 = world.add_location(
        category="table", parent="goal_room", pose=Pose(x=3.5, y=2.75, z=0.0)
    )
    world.add_object(category="apple", parent=table1)

    # Add a robot
    robot = Robot(radius=0.1, path_executor=ConstantVelocityExecutor())
    world.add_robot(robot, loc="main_room")

    # Create a search graph and motion planner
    planner_config = {
        "world": world,
        "bidirectional": False,
        "rrt_connect": False,
        "rrt_star": False,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 1.0,
    }
    rrt = PathPlanner("rrt", **planner_config)
    robot.set_path_planner(rrt)

    return world


def start_planner(world, domain_name="03_nav_stream", interactive=False):
    domain_folder = os.path.join(get_default_domains_folder(), domain_name)
    planner = PDDLStreamPlanner(world, domain_folder)
    goal_literals = [("Has", "robot", "apple")]

    if interactive:
        input("Press Enter to start planning.")
    robot = world.robots[0]
    plan = planner.plan(
        robot,
        goal_literals,
        max_attempts=3,
        verbose=interactive,
        planner="ff-astar",
        max_planner_time=30.0,
    )
    if interactive:
        robot.execute_plan(plan, blocking=True)
    return plan


#####################
# ACTUAL UNIT TESTS #
#####################
def test_symbolic_plan():
    world = create_test_world()
    plan = start_planner(world, domain_name="02_derived")
    assert plan is not None


def test_stream_plan_no_hallway():
    world = create_test_world(add_hallway=False)
    plan = start_planner(world, domain_name="03_nav_stream")
    assert plan is None


def test_stream_plan_with_hallway():
    world = create_test_world(add_hallway=True)
    plan = start_planner(world, domain_name="03_nav_stream")
    assert plan is not None


if __name__ == "__main__":
    world = create_test_world(add_hallway=True)

    # Start task and motion planner in separate thread.
    # domain_name = "02_derived" # Will get infeasible plan
    domain_name = "03_nav_stream"  # Will get feasible plan
    planner_thread = threading.Thread(
        target=start_planner, args=(world, domain_name, True)
    )
    planner_thread.start()

    start_gui(world)
