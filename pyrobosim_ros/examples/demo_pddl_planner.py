#!/usr/bin/env python3

"""
Test script showing how to have a PDDLStream planning ROS node..
"""

import os
import sys
import argparse
import rclpy
from rclpy.node import Node
import time

from pyrobosim.core.yaml import WorldYamlLoader
from pyrobosim.planning.pddlstream.planner import PDDLStreamPlanner
from pyrobosim.planning.pddlstream.utils import get_default_domains_folder
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.ros_conversions  import task_plan_to_ros
from pyrobosim_msgs.msg import TaskPlan


def parse_args():
    """ Parse command-line arguments """
    parser = argparse.ArgumentParser(description="PDDLStream planning demo.")
    parser.add_argument("--example", default="01_simple",
                        help="Example name (01_simple, 02_derived)")
    parser.add_argument("--receive-goal", action="store_true",
                        help="If True, receives a goal from a ROS service.")
    parser.add_argument("--verbose", action="store_true",
                        help="Print planning output")
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


class PlannerNode(Node):
    def __init__(self, args, name="pyrobosim"):
        self.name = name
        super().__init__(self.name + "_pddlstream_planner", namespace=self.name)

        # Publisher for a task plan
        self.plan_pub = self.create_publisher(
            TaskPlan, "commanded_plan", 10)

        # Need a delay to ensure publishers are ready
        time.sleep(1.0)

        # Create the world and planner
        self.world = load_world(args)
        domain_folder = os.path.join(
            get_default_domains_folder(), args.example)
        self.planner = PDDLStreamPlanner(self.world, domain_folder)

        get = lambda entity : self.world.get_entity_by_name(entity)
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
        else:
            print(f"Invalid example: {args.example}")
            return

        plan = self.planner.plan(goal_literals, focused=True, verbose=args.verbose)
        plan_msg = task_plan_to_ros(plan)
        self.plan_pub.publish(plan_msg)
        

def main():
    rclpy.init()
    args = parse_args()
    planner_node = PlannerNode(args, name="pddl_demo")

    rclpy.spin(planner_node)
    planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
