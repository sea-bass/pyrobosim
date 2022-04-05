#!/usr/bin/env python

"""
Test script showing how to build a world and use it with pyrobosim
"""
import os
import sys
import argparse
import numpy as np

from pyrobosim.navigation.execution import ConstantVelocityExecutor
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim.world.robot import Robot
from pyrobosim.world.room import Room
from pyrobosim.world.world import World


data_folder = get_data_folder()


def create_world():
    """ Create a test world """
    w = World()

    # Set the location and object metadata
    w.set_metadata(locations=os.path.join(data_folder, "example_location_data.yaml"),
                   objects=os.path.join(data_folder, "example_object_data.yaml"))

    # Add rooms
    r1coords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
    w.add_room(Room(r1coords, name="kitchen", color=[1, 0, 0],
               nav_poses=[Pose(x=0.75, y=0.75, yaw=0)]))
    r2coords = [(1.75, 2.5), (3.5, 2.5), (3.5, 4), (1.75, 4)]
    w.add_room(Room(r2coords, name="bedroom", color=[0, 0.6, 0]))
    r3coords = [(-1, 1), (-1, 3.5), (-3.0, 3.5), (-2.5, 1)]
    w.add_room(Room(r3coords, name="bathroom", color=[0, 0, 0.6]))

    # Add hallways between the rooms
    w.add_hallway("kitchen", "bathroom", width=0.7)
    w.add_hallway("bathroom", "bedroom", width=0.5,
                  conn_method="angle", conn_angle=0, offset=0.8)
    w.add_hallway("kitchen", "bedroom", width=0.6,
                  conn_method="points",
                  conn_points=[(1.0, 0.5), (2.5, 0.5), (2.5, 3.0)])

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
    w.create_search_graph(max_edge_dist=3.0, collision_check_dist=0.05)
    return w


def create_world_from_yaml():
    from pyrobosim.world.yaml import WorldYamlLoader
    loader = WorldYamlLoader()
    return loader.from_yaml(os.path.join(data_folder, "test_world.yaml"))


def start_gui(world, args):
    """ Initializes GUI """
    from pyrobosim.gui.main import PyRoboSim
    app = PyRoboSim(world, args)
    sys.exit(app.exec_())


def start_ros_node(world):
    """ Initializes ROS node """
    import rclpy
    from pyrobosim.world.ros_interface import WorldROSWrapper

    rclpy.init()
    world_node = WorldROSWrapper(world, name="test_world", state_pub_rate=0.1)
    world_node.start()


def parse_args():
    """ Parse command-line arguments """
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros", action="store_true")
    parser.add_argument("--from-yaml", action="store_true")
    return parser.parse_args()


def main(world):
    """ Main for standalone operation """
    start_gui(world, sys.argv)


def main_ros(world):
    """ Main for ROS operation """
    # Start ROS Node in separate thread
    import threading
    t = threading.Thread(target=start_ros_node, args=(world,))
    t.start()

    # Start GUI in main thread
    start_gui(world, sys.argv)


if __name__ == "__main__":
    args = parse_args()

    # Create a world
    if args.from_yaml:
        w = create_world_from_yaml()
    else:
        w = create_world()

    # Start the program
    if args.ros:
        main_ros(w)
    else:
        main(w)
