#!/usr/bin/env python3

"""
Test script showing how to build a world and use it with pyrobosim
"""
import os
import argparse

from pyrobosim.core import Robot, World, WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties
from pyrobosim.navigation import (
    ConstantVelocityExecutor,
    AStarPlanner,
    PRMPlanner,
    RRTPlanner,
)
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose

from shapely.geometry import Polygon


data_folder = get_data_folder()


def create_world(multirobot=False):
    """Create a test world"""
    world = World()

    # Set the location and object metadata
    world.add_metadata(
        locations=[
            os.path.join(data_folder, "example_location_data_furniture.yaml"),
            os.path.join(data_folder, "example_location_data_accessories.yaml"),
        ],
        objects=[
            os.path.join(data_folder, "example_object_data_food.yaml"),
            os.path.join(data_folder, "example_object_data_drink.yaml"),
        ],
    )

    # Add rooms

    r1coords = [(0, 0), (5.3, 0), (5.3, 3.73), (0, 3.73)]
    world.add_room(
        name="Office_1",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r1coords,
        color="green",
        wall_width=0.1,
    )

    r2coords = [(5.5, 0), (10.8, 0), (10.8, 3.73), (5.5, 3.73)]
    world.add_room(
        name="Office_2",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r2coords,
        color="blue",
        wall_width=0.1,
        nav_poses=[Pose(x=0.75, y=0.75, z=0.0, yaw=0.0)],
    )

    r3coords = [(11, 0), (16.3, 0), (16.3, 3.73), (11, 3.73)]
    world.add_room(
        name="Office_3",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r3coords,
        color="green",
        wall_width=0.1,
    )

    r4coords = [(16.5, 0), (21.8, 0), (21.8, 3.73), (16.5, 3.73)]
    world.add_room(
        name="Office_4",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r4coords,
        color="blue",
        wall_width=0.1,
    )

    r5coords = [(22, 0), (27.3, 0), (27.3, 3.73), (22, 3.73)]
    world.add_room(
        name="Office_5",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r5coords,
        color="green",
        wall_width=0.1,
    )

    r6coords = [(27.5, 0), (32.8, 0), (32.8, 3.73), (27.5, 3.73)]
    world.add_room(
        name="Office_6",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r6coords,
        color="blue",
        wall_width=0.1,
    )

    r7coords = [(33, 6.02), (38.67, 6.02), (38.67, 8.51), (33, 8.51)]
    world.add_room(
        name="Office_7",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r7coords,
        color="green",
        wall_width=0.1,
    )

    r8coords = [(33, 8.71), (38.67, 8.71), (38.67, 15.05), (33, 15.05)]
    world.add_room(
        name="Office_8",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r8coords,
        color="blue",
        wall_width=0.1,
    )

    r9coords = [(5.5, 6.02), (10.8, 6.02), (10.8, 9.27), (5.5, 9.27)]
    world.add_room(
        name="Sensor_Lab",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r9coords,
        color="red",
        wall_width=0.1,
    )

    r10coords = [(13.34, 6.02), (16.3, 6.02), (16.3, 9.27), (13.34, 9.27)]
    world.add_room(
        name="Store_Room",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r10coords,
        color="red",
        wall_width=0.1,
    )

    r11coords = [(13.34, 9.47), (16.3, 9.47), (16.3, 11.81), (13.34, 11.81)]
    world.add_room(
        name="Copy_Room",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r11coords,
        color="red",
        wall_width=0.1,
    )

    r12coords = [(16.5, 6.02), (21.8, 6.02), (21.8, 11.81), (16.5, 11.81)]
    world.add_room(
        name="Bathroom",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r12coords,
        color="red",
        wall_width=0.1,
    )

    #################################### Hallways ########################################

    r13coords = [(0, 3.93), (5.3, 3.93), (5.3, 15.05), (0, 15.05)]
    world.add_room(
        name="Hallway_1",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r13coords,
        color="grey",
        wall_width=0.1,
    )

    r14coords = [(5.5, 9.47), (13.14, 9.47), (13.14, 15.05), (5.5, 15.05), (5.5, 9.47)]
    world.add_room(
        name="Hallway_2",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r14coords,
        color="grey",
        wall_width=0.1,
    )

    r15coords = [
        (13.34, 12.01),
        (21.8, 12.01),
        (21.8, 15.05),
        (13.34, 15.05),
        (13.34, 12.01),
    ]
    world.add_room(
        name="Hallway_3",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r15coords,
        color="grey",
        wall_width=0.1,
    )

    r16coords = [(11, 6.02), (13.14, 6.02), (13.14, 9.27), (11, 9.27)]
    world.add_room(
        name="Hallway_4",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r16coords,
        color="grey",
        wall_width=0.1,
    )

    r17coords = [(5.5, 3.93), (32.8, 3.93), (32.8, 5.82), (5.5, 5.82)]
    world.add_room(
        name="Hallway_5",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r17coords,
        color="grey",
        wall_width=0.1,
    )

    r18coords = [(33, 0), (38.67, 0), (38.67, 5.82), (33, 5.82)]
    world.add_room(
        name="Hallway_6",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r18coords,
        color="grey",
        wall_width=0.1,
    )

    r19coords = [(27.5, 6.02), (32.8, 6.02), (32.8, 15.05), (27.5, 15.05)]
    world.add_room(
        name="Hallway_7",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r19coords,
        color="grey",
        wall_width=0.1,
    )

    r20coords = [(22, 6.02), (27.3, 6.02), (27.3, 8), (22, 8)]
    world.add_room(
        name="Entrance",
        # pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r20coords,
        color="grey",
        wall_width=0.1,
    )

    ################## Connect Hallways #################

    world.add_hallway(
        room_start="Hallway_1",
        room_end="Hallway_2",
        width=5.58,
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(5, 12.26), (5.5, 12.26), (6, 12.26)],
    )

    world.add_hallway(
        room_start="Hallway_2",
        room_end="Hallway_4",
        width=2.14,
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(12.07, 9), (12.07, 9.5), (12.07, 10)],
    )

    world.add_hallway(
        room_start="Hallway_1",
        room_end="Hallway_5",
        width=1.89,
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(5, 4.875), (6, 4.875), (7, 4.875)],
    )

    ### The one that gives error
    world.add_hallway(
        room_start="Hallway_2",
        room_end="Hallway_3",
        width=1.0,
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(12, 13.53), (13, 13.53), (14, 13.53)],
    )

    ############################ Doors ############################

    ### This one gives error too
    world.add_hallway(
        room_start="Entrance",
        room_end="Hallway_5",
        width=0.1,
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(23, 5), (23, 6), (23, 7)],
    )

    # Add robots
    grasp_props = ParallelGraspProperties(
        max_width=0.175,
        depth=0.1,
        height=0.04,
        width_clearance=0.01,
        depth_clearance=0.01,
    )

    robot0 = Robot(
        name="IDOG-1",
        radius=0.1,
        path_executor=ConstantVelocityExecutor(
            linear_velocity=1.0,
            dt=0.1,
            max_angular_velocity=4.0,
            validate_during_execution=True,
        ),
        grasp_generator=GraspGenerator(grasp_props),
        partial_observability=args.partial_observability,
        color="#CC00CC",
    )
    planner_config_rrt = {
        "world": world,
        "bidirectional": True,
        "rrt_connect": False,
        "rrt_star": True,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 0.5,
        "rewire_radius": 1.5,
        "compress_path": False,
    }
    rrt_planner = RRTPlanner(**planner_config_rrt)
    robot0.set_path_planner(rrt_planner)
    world.add_robot(robot0, loc="Hallway_1")

    if multirobot:
        robot1 = Robot(
            name="IDOG-2",
            radius=0.08,
            color=(0.8, 0.8, 0),
            path_executor=ConstantVelocityExecutor(),
            grasp_generator=GraspGenerator(grasp_props),
            partial_observability=args.partial_observability,
        )
        planner_config_prm = {
            "world": world,
            "collision_check_step_dist": 0.025,
            "max_connection_dist": 1.5,
            "max_nodes": 100,
            "compress_path": False,
        }
        prm_planner = PRMPlanner(**planner_config_prm)
        robot1.set_path_planner(prm_planner)
        world.add_robot(robot1, loc="Hallway_7")

    return world


def create_world_from_yaml(world_file):
    return WorldYamlLoader().from_file(os.path.join(data_folder, world_file))


def parse_args():
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(description="Main pyrobosim demo.")
    parser.add_argument(
        "--multirobot",
        action="store_true",
        help="If no YAML file is specified, this option will add "
        "multiple robots to the world defined in this file.",
    )
    parser.add_argument(
        "--world-file",
        default="",
        help="YAML file name (should be in the pyrobosim/data folder). "
        + "If not specified, a world will be created programmatically.",
    )
    parser.add_argument(
        "--partial-observability",
        action="store_true",
        help="If True, robots have partial observability and must detect objects.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    # Create a world or load it from file.
    if args.world_file == "":
        world = create_world(args.multirobot)
    else:
        world = create_world_from_yaml(args.world_file)

    # Start the program either as ROS node or standalone.
    start_gui(world)
