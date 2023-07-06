#!/usr/bin/env python3

"""
Test script showing how to publish actions and plans
"""

import rclpy
from rclpy.node import Node
import time

from pyrobosim_msgs.msg import TaskAction, TaskPlan
from pyrobosim_msgs.srv import RequestWorldState


class Commander(Node):
    def __init__(self):
        super().__init__("demo_command_publisher")

        self.declare_parameter("mode", value="plan")

        # Publisher for a single action
        self.action_pub = self.create_publisher(TaskAction, "commanded_action", 10)

        # Publisher for a task plan
        self.plan_pub = self.create_publisher(TaskPlan, "commanded_plan", 10)

        # Call world state service to ensure node is running
        self.world_state_client = self.create_client(
            RequestWorldState, "request_world_state"
        )
        while rclpy.ok() and not self.world_state_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info("Waiting for world state server...")
        future = self.world_state_client.call_async(RequestWorldState.Request())
        rclpy.spin_until_future_complete(self, future)


def main():
    rclpy.init()
    cmd = Commander()

    # Choose between action or plan command, based on input parameter.
    mode = cmd.get_parameter("mode").value
    if mode == "action":
        cmd.get_logger().info("Publishing sample task action...")
        action_msg = TaskAction(robot="robot", type="navigate", target_location="desk")
        cmd.action_pub.publish(action_msg)

    elif mode == "plan":
        cmd.get_logger().info("Publishing sample task plan...")
        task_actions = [
            TaskAction(type="navigate", target_location="desk"),
            TaskAction(type="pick", object="water"),
            TaskAction(type="navigate", target_location="counter"),
            TaskAction(type="place"),
            TaskAction(type="navigate", target_location="kitchen"),
        ]
        plan_msg = TaskPlan(robot="robot", actions=task_actions)
        cmd.plan_pub.publish(plan_msg)

    elif mode == "multirobot-plan":
        cmd.get_logger().info("Publishing sample multirobot task plan...")
        task_actions = [
            TaskAction(type="navigate", target_location="desk"),
            TaskAction(type="pick", object="water"),
            TaskAction(type="navigate", target_location="counter"),
            TaskAction(type="place"),
            TaskAction(type="navigate", target_location="kitchen"),
        ]
        plan_msg = TaskPlan(robot="robot0", actions=task_actions)
        cmd.plan_pub.publish(plan_msg)

        time.sleep(2.0)

        task_actions = [
            TaskAction(type="navigate", target_location="table"),
            TaskAction(type="pick", object="apple"),
            TaskAction(type="navigate", target_location="desk"),
            TaskAction(type="place"),
            TaskAction(type="navigate", target_location="bedroom"),
        ]
        plan_msg = TaskPlan(robot="robot1", actions=task_actions)
        cmd.plan_pub.publish(plan_msg)

        time.sleep(2.0)

        task_actions = [
            TaskAction(type="navigate", target_location="table"),
            TaskAction(type="pick", object="banana"),
            TaskAction(type="navigate", target_location="counter0_left"),
            TaskAction(type="place"),
        ]
        plan_msg = TaskPlan(robot="robot2", actions=task_actions)
        cmd.plan_pub.publish(plan_msg)

    else:
        cmd.get_logger().error(f"Invalid mode specified: {mode}")

    rclpy.spin(cmd)
    cmd.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
