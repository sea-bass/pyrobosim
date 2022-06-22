#!/usr/bin/env python3

"""
Test script showing how to perform task and motion planning with PDDLStream.
"""

import os
import sys
import argparse
import threading

from pyrobosim.core.yaml import WorldYamlLoader
from pyrobosim.planning.pddlstream.planner import PDDLStreamPlanner
from pyrobosim.planning.pddlstream.utils import get_default_domains_folder
from pyrobosim.utils.general import get_data_folder


def parse_args():
    """ Parse command-line arguments """
    parser = argparse.ArgumentParser(description="PDDLStream planning demo.")
    parser.add_argument("--example", default="01_simple",
                        help="Example name (01_simple, 02_derived)")
    parser.add_argument("--verbose", action="store_true",
                        help="Print planning output")
    return parser.parse_args()


def load_world(args):
    """ Load a test world. """
    loader = WorldYamlLoader()
    if (args.example == "01_simple") or (args.example == "02_derived"):
        world_file = "pddlstream_simple_world.yaml"
    elif (args.example == "03_nav_stream"):
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


def start_planner(world, args):
    """ Test PDDLStream planner under various scenarios. """
    domain_folder = os.path.join(
        get_default_domains_folder(), args.example)
    planner = PDDLStreamPlanner(world, domain_folder)

    get = lambda entity : world.get_entity_by_name(entity)
    if args.example == "01_simple":
        # Task specification for simple example.
        goal_literals = [
            ("At", get("robot"), get("bedroom")),
            ("At", get("apple0"), get("table0_tabletop")),
            ("At", get("banana0"), get("counter0_left")),
            ("Holding", get("robot"), get("water0"))
        ]
    elif args.example == "02_derived":
        # Task specification for derived predicate example.
        goal_literals = [
            ("Has", get("desk0_desktop"), get("banana0")),
            ("Has", "counter", get("apple1")),
            ("HasNone", get("bathroom"), "banana"),
            ("HasAll", "counter", "water")
        ]
    elif args.example == "03_nav_stream":
        # Task specification for navigation stream example.
        goal_literals = [
            ("Has", get("desk0_desktop"), get("banana0")),
            ("Has", "counter", get("apple1")),
            ("HasNone", get("bathroom"), "banana"),
            ("HasAll", "counter", "water")
        ]
    else:
        print(f"Invalid example: {args.example}")
        return

    input("Press Enter to start planning.")
    plan = planner.plan(goal_literals, focused=True, verbose=args.verbose)
    world.robot.execute_plan(plan, blocking=True)


if __name__ == "__main__":
    args = parse_args()
    w = load_world(args)

    # Start ROS Node in separate thread.
    t = threading.Thread(target=start_planner, args=(w, args))
    t.start()

    # Start the program either as ROS2 node or standalone.
    start_gui(w, sys.argv)
