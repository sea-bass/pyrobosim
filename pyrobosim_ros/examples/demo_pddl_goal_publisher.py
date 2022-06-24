#!/usr/bin/env python3

"""
Test script showing how to publish a goal specification to a PDDLStream planner node.
"""

import os
import argparse
import rclpy
from rclpy.node import Node
import time

from pyrobosim_msgs.msg import GoalPredicate, GoalSpecification


def parse_args():
    """ Parse command-line arguments """
    parser = argparse.ArgumentParser(
        description="PDDLStream demo goal specification publisher node.")
    parser.add_argument("--example", default="01_simple",
                        help="Example name (01_simple, 02_derived, 03_nav_stream)")
    return parser.parse_args()


class GoalPublisher(Node):
    def __init__(self, args, name="pyrobosim"):
        self.name = name
        super().__init__(self.name + "_pddlstream_goal_publisher", namespace=self.name)

        # Publisher for a task plan
        self.goalspec_pub = self.create_publisher(
            GoalSpecification, "goal_specification", 10)

        # Need a delay to ensure publishers are ready
        time.sleep(1.0)

        # Create goal specifications for different examples
        if args.example == "01_simple":
            # Goal specification for simple example.
            goal_predicates = [
                GoalPredicate(type="At", args=("robot", "bedroom")),
                GoalPredicate(type="At", args=("apple0", "table0_tabletop")),
                GoalPredicate(type="At", args=("banana0", "counter0_left")),
                GoalPredicate(type="Holding", args=("robot", "water0"))
            ]
        elif args.example == "02_derived":
            # Goal specification for derived predicate example.
            goal_predicates = [
                GoalPredicate(type="Has", args=("desk0_desktop", "banana0")),
                GoalPredicate(type="Has", args=("counter", "apple1")),
                GoalPredicate(type="HasNone", args=("bathroom", "banana")),
                GoalPredicate(type="HasAll", args=("table", "water"))
            ]
        elif args.example == "03_nav_stream":
            # Goal specification for derived predicate example.
            goal_predicates = [
                GoalPredicate(type="Has", args=("desk0_desktop", "banana0")),
                GoalPredicate(type="Has", args=("counter", "apple1")),
                GoalPredicate(type="HasNone", args=("bathroom", "banana")),
                GoalPredicate(type="HasAll", args=("table", "water"))
            ]
        else:
            print(f"Invalid example: {args.example}")
            return

        goal_msg = GoalSpecification(predicates=goal_predicates)
        self.goalspec_pub.publish(goal_msg)
        
        print(f"Published goal specification:")
        for pred in goal_msg.predicates:
            print(pred)
        

def main():
    rclpy.init()
    args = parse_args()
    goal_node = GoalPublisher(args, name="pddl_demo")

    rclpy.spin(goal_node)
    goal_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
