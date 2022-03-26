#!/usr/bin/env python

"""
Test script showing how to build a world and use it with pyrobosim
"""
import os
import sys
import argparse
import numpy as np

from pyrobosim.utils.pose import Pose
from pyrobosim.world.robot import Robot
from pyrobosim.world.room import Room
from pyrobosim.world.world import World

# Set the data folder for location and object information.
try:
    # If running as a ROS2 node, get the data folder from the package share directory.
    from ament_index_python.packages import get_package_share_directory
    data_folder = os.path.join(
        get_package_share_directory("pyrobosim"), "data")
except:
    # Else, assume it's relative to the file's current directory.
    data_folder = os.path.join(os.path.dirname(
        os.path.abspath(__file__)), "..", "data")


def create_world():
    # Create a world
    w = World()
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
    table = w.add_location("table", "kitchen", Pose(x=0.85, y=-0.5, yaw=-np.pi/2))
    desk = w.add_location("desk", "bedroom", Pose(x=3.15, y=3.65, yaw=0))
    counter = w.add_location("counter", "bathroom", Pose(x=-2.45, y=2.5, yaw=np.pi/2 + np.pi/16))

    # Add objects
    w.add_object("banana", table, pose=Pose(x=1.0, y=-0.5, yaw=np.pi/4))
    w.add_object("apple", desk, pose=Pose(x=3.2, y=3.5, yaw=0))
    w.add_object("apple", table)
    w.add_object("apple", table)
    w.add_object("water", counter)
    w.add_object("banana", counter)
    w.add_object("water", desk)

    # Add a robot
    w.add_robot(Robot(radius=0.1), loc="kitchen")

    # Create a search graph
    w.create_search_graph(max_edge_dist=3.0, collision_check_dist=0.05)
    return w


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
    world_node = WorldROSWrapper(world)
    rclpy.spin(world_node)
    
    world_node.destroy_node()
    rclpy.shutdown()


def parse_args():
    """ Parse command-line arguments """
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros", action="store_true")
    return parser.parse_args()


def main():
    """ Main for standalone operation """
    w = create_world()
    start_gui(w, sys.argv)

def main_ros():
    """ Main for ROS operation """
    import threading
    w = create_world()

    # Start ROS Node in separate thread
    t = threading.Thread(target=start_ros_node, args=(w,))
    t.start()
    
    # Start GUI in main thread
    start_gui(w, sys.argv)

if __name__ == "__main__":
    args = parse_args()
    if args.ros:
        main_ros()
    else:
        main()
