#!/usr/bin/env python3

"""
Test script showing how to start a pyrobosim world that receives a plan from a
Task and Motion Planner such as PDDLStream.
"""

import os
import rclpy
import threading

from pyrobosim.core import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.utils.general import get_data_folder
from pyrobosim_ros.ros_interface import WorldROSWrapper


def load_world():
    """Load a test world."""
    world_file = os.path.join(get_data_folder(), "pddlstream_simple_world.yaml")
    return WorldYamlLoader().from_yaml(world_file)


def create_ros_node():
    """Initializes ROS node"""
    rclpy.init()
    world = load_world()
    node = WorldROSWrapper(world=world, name="pddl_demo", state_pub_rate=0.1)
    return node


if __name__ == "__main__":
    node = create_ros_node()

    # Start ROS Node in separate thread
    ros_thread = threading.Thread(target=lambda: node.start(wait_for_gui=True))
    ros_thread.start()

    # Start GUI in main thread
    start_gui(node.world)
