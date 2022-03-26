#!/usr/bin/env python

"""
Test script showing how to publish actions and plans
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time

from pyrobosim.msg import TaskAction


class CommandPublisher(Node):
    def __init__(self):
        super().__init__("pyrobosim_action_publisher")

        # Publisher for a single action
        self.action_pub = self.create_publisher(
            TaskAction, "/commanded_action", 1)
        
        # TODO: Add publisher for an action plan

def main():
    rclpy.init()
    cmd_pub = CommandPublisher()
    time.sleep(1.0) # Need a delay to ensure publisher is ready

    # Publish a single action
    print("Publishing sample action...")
    act_msg = TaskAction()
    act_msg.type = "navigate"
    act_msg.target_location = np.random.choice(
        ["desk", "table", "counter"])
    cmd_pub.action_pub.publish(act_msg)

    rclpy.spin(cmd_pub)
    cmd_pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
