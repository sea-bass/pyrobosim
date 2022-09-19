#!/usr/bin/env python3

"""
Test script showing how to build a world and use it with pyrobosim,
additionally starting up a ROS interface.
"""
import os
import subprocess
import sys
import numpy as np

from pyrobosim.core.robot import Robot
from pyrobosim.core.room import Room
from pyrobosim.core.world import World
from pyrobosim.navigation.execution import ConstantVelocityExecutor
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


data_folder = get_data_folder()


def create_world():
    """ Create a test world """
    w = World()

    # TODO(andyz): load test_world.yaml, since that's what exports to Gazebo

    # Set the location and object metadata
    w.set_metadata(locations=os.path.join(data_folder, "example_location_data.yaml"),
                   objects=os.path.join(data_folder, "example_object_data.yaml"))

    # Add rooms
    r1coords = [(-2.0, -6.0), (5.0, -6.0), (5.0, 1.5), (-2.0, 1.5)]
    w.add_room(Room(r1coords, name="kitchen", color=[1, 0, 0],
               nav_poses=[Pose(x=0.75, y=0.75, yaw=0)]))
    r2coords = [(1.75, 2.5), (7.0, 2.5), (7.0, 6.0), (1.75, 6.0)]
    w.add_room(Room(r2coords, name="bedroom", color=[0, 0.6, 0]))
    r3coords = [(0.0, 2.0), (0.0, 7.0), (-5.0, 7.0), (-5.0, 2.0)]
    w.add_room(Room(r3coords, name="bathroom", color=[0, 0, 0.6]))

    # Add hallways between the rooms
    w.add_hallway("kitchen", "bathroom", width=2.0,
                  conn_method="auto")
    w.add_hallway("bathroom", "bedroom", width=2.0,
                  conn_method="auto")
    w.add_hallway("kitchen", "bedroom", width=2.0,
                  conn_method="auto")

    # Add locations
    table = w.add_location("table", "kitchen", Pose(
        x=0.85, y=-0.5, yaw=-np.pi/2))
    desk = w.add_location("desk", "bedroom", Pose(x=3.15, y=3.65, yaw=0))
    counter = w.add_location("counter", "bathroom", Pose(
        x=-2.45, y=2.5, yaw=np.pi/2 + np.pi/16))

    # Add objects
    w.add_object("banana", table, pose=Pose(x=1.0, y=-0.5, yaw=np.pi/4))
    w.add_object("apple", desk, pose=Pose(x=3.2, y=3.5, yaw=0))
    w.add_object("apple", table)
    w.add_object("apple", table)
    w.add_object("water", counter)
    w.add_object("banana", counter)
    w.add_object("water", desk)

    # Add a robot
    r = Robot(radius=0.1, path_executor=ConstantVelocityExecutor())
    w.add_robot(r, loc="kitchen")

    # Create a search graph
    w.create_search_graph(
        max_edge_dist=3.0, collision_check_dist=0.05, create_planner=True)
    return w


def create_world_from_yaml(world_file):
    from pyrobosim.core.yaml import WorldYamlLoader
    loader = WorldYamlLoader()
    return loader.from_yaml(os.path.join(data_folder, world_file))


def start_gui(world, args):
    """ Initializes GUI """
    from pyrobosim.gui.main import PyRoboSimGUI
    app = PyRoboSimGUI(world, args)
    sys.exit(app.exec_())


def create_ros_node():
    """ Initializes ROS node """
    import rclpy
    from pyrobosim.core.ros_interface import WorldROSWrapper

    rclpy.init()
    node = WorldROSWrapper(state_pub_rate=0.1)
    node.declare_parameter("world_file", value="")
    
    # Set the world
    world_file = node.get_parameter("world_file").value
    if world_file == "":
        node.get_logger().info("Creating demo world programmatically.")
        w = create_world()
    else:
        node.get_logger().info(f"Using world file {world_file}.")
        w = create_world_from_yaml(world_file)
        
    node.set_world(w)

    return node


if __name__ == "__main__":
    n = create_ros_node()

    # Start ROS node in separate thread
    import threading
    node_thread = threading.Thread(target=n.start)
    node_thread.start()

    # Start GUI in main thread
    start_gui(n.world, sys.argv)
