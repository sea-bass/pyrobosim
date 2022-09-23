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


def create_world_from_yaml(world_file):
    from pyrobosim.core.yaml import WorldYamlLoader
    loader = WorldYamlLoader()
    return loader.from_yaml(os.path.join(data_folder, world_file + "_world.yaml"))


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
        node.get_logger().error("A world_file parameter was not given.")
        sys.exit()
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
