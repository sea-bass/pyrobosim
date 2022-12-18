#!/usr/bin/env python3

"""
Test script showing how to start a pyrobosim world that receives a plan from a
Task and Motion Planner such as PDDLStream.
"""

import os
import sys
import threading

from pyrobosim.core.yaml import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder


def load_world():
    """ Load a test world. """
    loader = WorldYamlLoader()
    world_file = "pddlstream_simple_world.yaml"
    data_folder = get_data_folder()
    w = loader.from_yaml(os.path.join(data_folder, world_file))
    return w


def start_gui(world, args):
    """ Initializes GUI """
    from pyrobosim.gui.main import PyRoboSimGUI
    app = PyRoboSimGUI(world, args)
    sys.exit(app.exec_())


def create_ros_node():
    """ Initializes ROS node """
    import rclpy
    rclpy.init()

    from pyrobosim.core.ros_interface import WorldROSWrapper
    w = load_world()
    node = WorldROSWrapper(world=w, name="pddl_demo", state_pub_rate=0.1)
    return node


if __name__ == "__main__":
    n = create_ros_node()

    # Start ROS Node in separate thread
    import threading
    t = threading.Thread(target=n.start)
    t.start()

    # Start GUI in main thread
    start_gui(n.world, sys.argv)
