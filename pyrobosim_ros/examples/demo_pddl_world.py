#!/usr/bin/env python3

"""
Example showing how to start a PyRoboSim world that receives a plan from a
Task and Motion Planner such as PDDLStream.
"""

import os
import threading

import rclpy

from pyrobosim.core import World, WorldYamlLoader
from pyrobosim.utils.general import get_data_folder
from pyrobosim.web import start_ui
from pyrobosim_ros.ros_interface import WorldROSWrapper


def load_world() -> World:
    """Load a test world."""
    world_file = os.path.join(get_data_folder(), "pddlstream_simple_world.yaml")
    return WorldYamlLoader().from_file(world_file)


def create_ros_node() -> WorldROSWrapper:
    """Initializes ROS node"""
    rclpy.init()
    world = load_world()
    return WorldROSWrapper(world=world, name="pddl_demo", state_pub_rate=0.1)


if __name__ == "__main__":
    node = create_ros_node()

    # Start ROS Node in separate thread
    ros_thread = threading.Thread(target=node.start)
    ros_thread.start()

    # Start the web UI in main thread
    start_ui(node.world)
