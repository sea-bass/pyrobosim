#!/usr/bin/env python3

"""
Example showing how to publish velocity commands to a pyrobosim robot.
"""

import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__("demo_velocity_publisher")

        # Declare parameters
        self.declare_parameter("robot_name", value="robot")
        self.declare_parameter("pub_period", value=0.1)
        self.declare_parameter("lin_vel", value=0.2)
        self.declare_parameter("ang_vel", value=0.5)

        # Publisher for velocity commands
        robot_name = self.get_parameter("robot_name").value
        self.vel_pub = self.create_publisher(Twist, f"{robot_name}/cmd_vel", 10)
        self.get_logger().info("Waiting for subscription")
        while rclpy.ok() and self.vel_pub.get_subscription_count() < 1:
            time.sleep(2.0)
        if self.vel_pub.get_subscription_count() < 1:
            self.get_logger().error(
                f"Error while waiting for a velocity command subscriber to {self.vel_pub.topic_name}."
            )
            return

        # Create a publisher timer
        pub_period = self.get_parameter("pub_period").value
        self.timer = self.create_timer(pub_period, self.vel_pub_callback)

    def vel_pub_callback(self):
        """Publisher callback for velocity commands."""
        lin_vel = self.get_parameter("lin_vel").value
        ang_vel = self.get_parameter("ang_vel").value

        pub_cmd = Twist()
        pub_cmd.linear.x = lin_vel
        pub_cmd.angular.z = ang_vel
        self.vel_pub.publish(pub_cmd)


def main():
    rclpy.init()
    pub_node = VelocityPublisher()
    rclpy.spin(pub_node)
    pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
