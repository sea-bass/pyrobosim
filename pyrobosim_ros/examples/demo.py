#!/usr/bin/env python3

"""
Test script showing how to build a world and use it with pyrobosim,
additionally starting up a ROS interface.
"""
import os
import rclpy
import threading
import numpy as np

from pyrobosim.core import Robot, World, WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.navigation import ConstantVelocityExecutor
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim_ros.ros_interface import WorldROSWrapper


data_folder = get_data_folder()


def create_world():
    """Create a test world"""
    world = World()

    # Set the location and object metadata
    world.set_metadata(
        locations=os.path.join(data_folder, "example_location_data.yaml"),
        objects=os.path.join(data_folder, "example_object_data.yaml"),
    )

    # Add rooms
    r1coords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
    world.add_room(name="kitchen", footprint=r1coords, color=[1, 0, 0])
    r2coords = [(1.75, 2.5), (3.5, 2.5), (3.5, 4), (1.75, 4)]
    world.add_room(name="bedroom", footprint=r2coords, color=[0, 0.6, 0])
    r3coords = [(-1, 1), (-1, 3.5), (-3.0, 3.5), (-2.5, 1)]
    world.add_room(name="bathroom", footprint=r3coords, color=[0, 0, 0.6])

    # Add hallways between the rooms
    world.add_hallway(room_start="kitchen", room_end="bathroom", width=0.7)
    world.add_hallway(
        room_start="bathroom",
        room_end="bedroom",
        width=0.5,
        conn_method="angle",
        conn_angle=0,
        offset=0.8,
    )
    world.add_hallway(
        room_start="kitchen",
        room_end="bedroom",
        width=0.6,
        conn_method="points",
        conn_points=[(1.0, 0.5), (2.5, 0.5), (2.5, 3.0)],
    )

    # Add locations
    table = world.add_location("table", "kitchen", Pose(x=0.85, y=-0.5, yaw=-np.pi / 2))
    desk = world.add_location("desk", "bedroom", Pose(x=3.15, y=3.65, yaw=0))
    counter = world.add_location(
        "counter", "bathroom", Pose(x=-2.45, y=2.5, yaw=np.pi / 2 + np.pi / 16)
    )

    # Add objects
    world.add_object("banana", table, pose=Pose(x=1.0, y=-0.5, yaw=np.pi / 4))
    world.add_object("apple", desk, pose=Pose(x=3.2, y=3.5, yaw=0))
    world.add_object("apple", table)
    world.add_object("apple", table)
    world.add_object("water", counter)
    world.add_object("banana", counter)
    world.add_object("water", desk)

    # Add a robot
    robot = Robot(name="robot", radius=0.1, path_executor=ConstantVelocityExecutor())
    world.add_robot(robot, loc="kitchen")

    # Create a search graph
    world.create_search_graph(
        max_edge_dist=3.0, collision_check_dist=0.05, create_planner=True
    )
    return world


def create_world_from_yaml(world_file):
    loader = WorldYamlLoader()
    return loader.from_yaml(os.path.join(data_folder, world_file))


def create_ros_node():
    """Initializes ROS node"""
    rclpy.init()
    node = WorldROSWrapper(state_pub_rate=0.1)
    node.declare_parameter("world_file", value="")

    # Set the world
    world_file = node.get_parameter("world_file").value
    if world_file == "":
        node.get_logger().info("Creating demo world programmatically.")
        world = create_world()
    else:
        node.get_logger().info(f"Using world file {world_file}.")
        world = create_world_from_yaml(world_file)

    node.set_world(world)

    return node


if __name__ == "__main__":
    node = create_ros_node()

    # Start ROS node in separate thread
    ros_thread = threading.Thread(target=node.start)
    ros_thread.start()

    # Start GUI in main thread
    start_gui(node.world)
