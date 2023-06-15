#!/usr/bin/env python3

"""
Test script showing how to perform task and motion planning with PDDLStream.
"""

import os
import argparse
import threading

from pyrobosim.core import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.planning import PDDLStreamPlanner
from pyrobosim.planning.pddlstream.utils import get_default_domains_folder
from pyrobosim.utils.general import get_data_folder


def parse_args():
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(description="PDDLStream planning demo.")
    parser.add_argument(
        "--example",
        default="01_simple",
        help="Example name (01_simple, 02_derived, 03_nav_stream, 04_nav_manip_stream, 05_nav_grasp_stream)",
    )
    parser.add_argument("--verbose", action="store_true", help="Print planning output")
    parser.add_argument(
        "--search-sample-ratio",
        type=float,
        default=0.5,
        help="Search to sample ratio for planner",
    )
    return parser.parse_args()


def load_world():
    """Load a test world."""
    world_file = os.path.join(get_data_folder(), "pddlstream_simple_world.yaml")
    return WorldYamlLoader().from_yaml(world_file)


def start_planner(world, args):
    """Test PDDLStream planner under various scenarios."""
    domain_folder = os.path.join(get_default_domains_folder(), args.example)
    planner = PDDLStreamPlanner(world, domain_folder)

    if args.example == "01_simple":
        # Task specification for simple example.
        goal_literals = [
            ("At", "robot", "bedroom"),
            ("At", "apple0", "table0_tabletop"),
            ("At", "banana0", "counter0_left"),
            ("Holding", "robot", "water0"),
        ]
    elif args.example in [
        "02_derived",
        "03_nav_stream",
        "04_nav_manip_stream",
        "05_nav_grasp_stream",
    ]:
        # Task specification for derived predicate example.
        goal_literals = [
            ("Has", "desk0_desktop", "banana0"),
            ("Has", "counter", "apple1"),
            ("HasNone", "bathroom", "banana"),
            ("HasAll", "table", "water"),
        ]
    else:
        print(f"Invalid example: {args.example}")
        return

    input("Press Enter to start planning.")
    robot = world.robots[0]
    plan = planner.plan(
        robot,
        goal_literals,
        verbose=args.verbose,
        max_attempts=3,
        search_sample_ratio=args.search_sample_ratio,
        planner="ff-astar",
        max_planner_time=10.0,
        max_time=60.0,
    )
    robot.execute_plan(plan, blocking=True)


if __name__ == "__main__":
    args = parse_args()
    world = load_world()

    # Start task and motion planner in separate thread.
    planner_thread = threading.Thread(target=start_planner, args=(world, args))
    planner_thread.start()

    # Start GUI in main thread.
    start_gui(world)
