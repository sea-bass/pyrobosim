#!/usr/bin/env python

"""
Test script showing how to publish actions and plans
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time

from pyrobosim.msg import TaskAction, TaskPlan


class CommandPublisher(Node):
    def __init__(self):
        super().__init__("pyrobosim_action_publisher")

        # Publisher for a single action
        self.action_pub = self.create_publisher(
            TaskAction, "/commanded_action", 10)
        
        # Publisher for a task plan
        self.plan_pub = self.create_publisher(
            TaskPlan, "/commanded_plan", 10)

def main():
    rclpy.init()
    cmd_pub = CommandPublisher()
    time.sleep(1.0) # Need a delay to ensure publisher is ready

    print("Publishing sample task plan...")
    task_actions = [
        TaskAction(type="navigate", target_location="desk"),
        TaskAction(type="pick"),
        TaskAction(type="navigate", target_location="counter"),
        TaskAction(type="place"),
        TaskAction(type="navigate", target_location="kitchen")
    ]
    plan_msg = TaskPlan(actions=task_actions)
    cmd_pub.plan_pub.publish(plan_msg)

    rclpy.spin(cmd_pub)
    cmd_pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
