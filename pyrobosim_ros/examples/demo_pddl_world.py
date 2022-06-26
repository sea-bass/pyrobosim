#!/usr/bin/env python3

"""
Test script showing how to start a pyrobosim world that receives a plan from a
Task and Motion Planner such as PDDLStream.
"""

import os
import sys
import argparse
import threading

from pyrobosim.core.yaml import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder


def parse_args():
    """ Parse command-line arguments """
    parser = argparse.ArgumentParser(description="PDDLStream demo world node.")
    parser.add_argument("--example", default="01_simple",
                        help="Example name (01_simple, 02_derived)")
    return parser.parse_args()


def load_world(args):
    """ Load a test world. """
    loader = WorldYamlLoader()
    if (args.example == "01_simple") or (args.example == "02_derived"):
        world_file = "pddlstream_simple_world.yaml"
    else:
        print(f"Invalid example: {args.example}")
        return

    data_folder = get_data_folder()
    w = loader.from_yaml(os.path.join(data_folder, world_file))
    return w


def start_gui(world, args):
    """ Initializes GUI """
    from pyrobosim.gui.main import PyRoboSimGUI
    app = PyRoboSimGUI(world, args)
    sys.exit(app.exec_())


def start_ros_node(world):
    """ Initializes ROS node """
    import rclpy
    from pyrobosim.core.ros_interface import WorldROSWrapper

    rclpy.init()
    world_node = WorldROSWrapper(world, name="pddl_demo", state_pub_rate=0.1)
    world_node.start()


if __name__ == "__main__":
    args = parse_args()
    w = load_world(args)

    # Start ROS Node in separate thread
    import threading
    t = threading.Thread(target=start_ros_node, args=(w,))
    t.start()

    # Start GUI in main thread
    start_gui(w, sys.argv)
