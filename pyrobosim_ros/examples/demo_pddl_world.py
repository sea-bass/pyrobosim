#!/usr/bin/env python3

"""
Test script showing how to start a pyrobosim world that receives a plan from a
Task and Motion Planner such as PDDLStream.
"""

import os
import rclpy
import threading

from pyrobosim.core.ros_interface import WorldROSWrapper
from pyrobosim.core.yaml import WorldYamlLoader
from pyrobosim.gui.main import start_gui
from pyrobosim.utils.general import get_data_folder


def load_world():
    """Load a test world."""
    loader = WorldYamlLoader()
    world_file = "pddlstream_simple_world.yaml"
    data_folder = get_data_folder()
    return loader.from_yaml(os.path.join(data_folder, world_file))


def create_ros_node():
    """Initializes ROS node"""
    rclpy.init()
    world = load_world()
    node = WorldROSWrapper(world=world, name="pddl_demo", state_pub_rate=0.1)
    return node


if __name__ == "__main__":
    node = create_ros_node()

    # Start ROS Node in separate thread
    ros_thread = threading.Thread(target=node.start)
    ros_thread.start()

    # Start GUI in main thread
    start_gui(node.world)
