#!/usr/bin/env python3

"""
Test script showing how to publish a goal specification to a PDDLStream planner node.
"""

import rclpy
from rclpy.node import Node
import time

from pyrobosim_msgs.msg import GoalPredicate, GoalSpecification


class GoalPublisher(Node):
    def __init__(self):
        super().__init__("demo_pddlstream_goal_publisher")

        # Declare parameters
        self.declare_parameter("example", value="01_simple")
        self.declare_parameter("verbose", value=True)

        # Publisher for a goal specification
        self.goalspec_pub = self.create_publisher(
            GoalSpecification, "goal_specification", 10)
        self.get_logger().info("Waiting for subscription")
        while self.goalspec_pub.get_subscription_count() < 1:
            time.sleep(2.0)

        # Create goal specifications for different examples
        example = self.get_parameter("example").value
        if example == "01_simple":
            # Goal specification for simple example.
            goal_predicates = [
                GoalPredicate(type="At", args=("robot", "bedroom")),
                GoalPredicate(type="At", args=("apple0", "table0_tabletop")),
                GoalPredicate(type="At", args=("banana0", "counter0_left")),
                GoalPredicate(type="Holding", args=("robot", "water0"))
            ]
        elif example in ["02_derived", "03_nav_stream", "04_nav_manip_stream"]:
            # Goal specification for derived predicate example.
            goal_predicates = [
                GoalPredicate(type="Has", args=("desk0_desktop", "banana0")),
                GoalPredicate(type="Has", args=("counter", "apple1")),
                GoalPredicate(type="HasNone", args=("bathroom", "banana")),
                GoalPredicate(type="HasAll", args=("table", "water"))
            ]
        else:
            self.get_logger().info(f"Invalid example: {example}")
            return

        # Publish and optionally display the goal specification message.
        goal_msg = GoalSpecification(predicates=goal_predicates)        
        if self.get_parameter("verbose").value == True:
            msg = "Published goal specification:"
            for pred in goal_msg.predicates:
                msg += f"\n{pred}"
            self.get_logger().info(msg)
        self.goalspec_pub.publish(goal_msg)


def main():
    rclpy.init()
    goal_node = GoalPublisher()

    rclpy.spin(goal_node)
    goal_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
