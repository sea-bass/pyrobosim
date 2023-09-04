#!/usr/bin/env python3

"""
Tests for PDDLStream planning with manipulation streams.
"""
import numpy as np
import os
import pytest
import threading

from pyrobosim.core import Robot, World
from pyrobosim.gui import start_gui
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties
from pyrobosim.navigation import ConstantVelocityExecutor, PathPlanner
from pyrobosim.planning import PDDLStreamPlanner
from pyrobosim.planning.pddlstream.utils import get_default_domains_folder
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


def create_test_world(add_alt_desk=True):
    world = World(object_radius=0.0375)

    # Set the location and object metadata
    data_folder = get_data_folder()
    world.set_metadata(
        locations=os.path.join(data_folder, "example_location_data.yaml"),
        objects=os.path.join(data_folder, "example_object_data.yaml"),
    )

    # Add rooms
    home_coords = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    world.add_room(name="home", footprint=home_coords, color=[1, 0, 0])
    storage_coords = [(2, -2), (5, -2), (5, 2), (2, 2)]
    world.add_room(name="storage", footprint=storage_coords, color=[0, 0, 1])
    world.add_hallway(
        room_start="home", room_end="storage", width=0.75, conn_method="auto"
    )

    # Add locations and objects
    table0 = world.add_location(
        category="table", parent="home", pose=Pose(x=0.0, y=0.5, z=0.0, yaw=np.pi / 2.0)
    )
    desk0 = world.add_location(
        category="desk", parent="storage", pose=Pose(x=2.5, y=-1.5, z=0.0, yaw=0.0)
    )
    if add_alt_desk:
        desk1 = world.add_location(
            category="desk", parent="storage", pose=Pose(x=4.5, y=1.5, z=0.0, yaw=0.0)
        )
    world.add_object(category="banana", parent=table0)
    world.add_object(
        category="water",
        parent=desk0,
        pose=Pose(x=2.375, y=-1.375, z=0, yaw=np.pi / 4.0),
    )
    world.add_object(
        category="water",
        parent=desk0,
        pose=Pose(x=2.575, y=-1.6, z=0, yaw=-np.pi / 4.0),
    )

    # Add a robot
    grasp_props = ParallelGraspProperties(
        max_width=0.175,
        depth=0.1,
        height=0.04,
        width_clearance=0.01,
        depth_clearance=0.01,
    )
    robot = Robot(
        radius=0.1,
        path_executor=ConstantVelocityExecutor(),
        grasp_generator=GraspGenerator(grasp_props),
    )
    world.add_robot(robot, loc="home", pose=Pose(x=0.0, y=-0.5))

    # Create a motion planner
    planner_config = {
        "world": world,
        "bidirectional": True,
        "rrt_connect": False,
        "rrt_star": True,
        "compress_path": False,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 1.0,
    }
    rrt = PathPlanner("rrt", **planner_config)
    robot.set_path_planner(rrt)

    return world


def start_planner(
    world, domain_name, interactive=False, max_attempts=1, search_sample_ratio=0.5
):
    domain_folder = os.path.join(get_default_domains_folder(), domain_name)
    planner = PDDLStreamPlanner(world, domain_folder)

    goal_literals = [("Has", "desk", "banana")]

    if interactive:
        input("Press Enter to start planning.")
    robot = world.robots[0]
    plan = planner.plan(
        robot,
        goal_literals,
        search_sample_ratio=search_sample_ratio,
        max_attempts=max_attempts,
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
domains_to_test = ["04_nav_manip_stream", "05_nav_grasp_stream"]


@pytest.mark.parametrize("domain_name", domains_to_test)
def test_manip_single_desk(domain_name):
    world = create_test_world(add_alt_desk=False)
    plan = start_planner(
        world, domain_name=domain_name, max_attempts=3, search_sample_ratio=0.5
    )
    assert plan is not None


@pytest.mark.parametrize("domain_name", domains_to_test)
def test_manip_double_desk(domain_name):
    world = create_test_world(add_alt_desk=True)
    plan = start_planner(
        world, domain_name=domain_name, max_attempts=3, search_sample_ratio=0.2
    )
    assert plan is not None


if __name__ == "__main__":
    domain_name = "04_nav_manip_stream"
    add_alt_desk = False
    interactive = True
    max_attempts = 3

    # Start task and motion planner in separate thread.
    world = create_test_world(add_alt_desk)
    planner_thread = threading.Thread(
        target=start_planner, args=(world, domain_name, interactive, max_attempts)
    )
    planner_thread.start()

    start_gui(world)
