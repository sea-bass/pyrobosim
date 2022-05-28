#!/usr/bin/env python

"""
Test script showing how to publish actions and plans
"""

import argparse
import rclpy
from rclpy.node import Node
import time

from pyrobosim_msgs.msg import TaskAction, TaskPlan


class Commander(Node):
    def __init__(self, name="pyrobosim"):
        self.name = name
        super().__init__(self.name + "_command_publisher", namespace=self.name)

        # Publisher for a single action
        self.action_pub = self.create_publisher(
            TaskAction, "commanded_action", 10)

        # Publisher for a task plan
        self.plan_pub = self.create_publisher(
            TaskPlan, "commanded_plan", 10)

        # Need a delay to ensure publishers are ready
        time.sleep(1.0)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test publishing commands to pyrobosim.")
    parser.add_argument("--mode", default="plan",
                        help="Command mode (action or plan)")
    return parser.parse_args()


def main():
    rclpy.init()
    cmd = Commander(name="test_world")
    args = parse_args()

    if args.mode == "action":
        print("Publishing sample task action...")
        action_msg = TaskAction(type="navigate", target_location="desk")
        cmd.action_pub.publish(action_msg)

    elif args.mode == "plan":
        print("Publishing sample task plan...")
        task_actions = [
            TaskAction(type="navigate", target_location="desk"),
            TaskAction(type="pick", object="water"),
            TaskAction(type="navigate", target_location="counter"),
            TaskAction(type="place"),
            TaskAction(type="navigate", target_location="kitchen")
        ]
        plan_msg = TaskPlan(actions=task_actions)
        cmd.plan_pub.publish(plan_msg)

    rclpy.spin(cmd)
    cmd.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
