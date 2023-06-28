#!/usr/bin/env python3

"""
Test script showing how to use a PDDLStream planner as a ROS 2 node.
"""

import os
import time
import rclpy
from rclpy.node import Node

from pyrobosim.core import WorldYamlLoader
from pyrobosim.planning import PDDLStreamPlanner
from pyrobosim.planning.pddlstream.utils import get_default_domains_folder
from pyrobosim.utils.general import get_data_folder

from pyrobosim_ros.ros_interface import update_world_from_state_msg
from pyrobosim_ros.ros_conversions import (
    goal_specification_from_ros,
    task_plan_to_ros,
)
from pyrobosim_msgs.msg import GoalSpecification, TaskPlan
from pyrobosim_msgs.srv import RequestWorldState


def load_world():
    """Load a test world."""
    loader = WorldYamlLoader()
    world_file = "pddlstream_simple_world.yaml"
    data_folder = get_data_folder()
    return loader.from_yaml(os.path.join(data_folder, world_file))


class PlannerNode(Node):
    def __init__(self):
        self.latest_goal = None
        self.planning = False
        super().__init__("demo_pddlstream_planner")

        # Declare parameters
        self.declare_parameter("example", value="01_simple")
        self.declare_parameter("subscribe", value=True)
        self.declare_parameter("verbose", value=True)
        self.declare_parameter("search_sample_ratio", value=0.5)

        # Publisher for a task plan
        self.plan_pub = self.create_publisher(TaskPlan, "commanded_plan", 10)

        # Service client for world state
        self.world_state_client = self.create_client(
            RequestWorldState, "request_world_state"
        )
        self.world_state_future_response = None
        while rclpy.ok() and not self.world_state_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info("Waiting for world state server...")
        if not self.world_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Error while waiting for the world state server.")
            return

        # Create the world and planner
        self.world = load_world()
        example = self.get_parameter("example").value
        domain_folder = os.path.join(get_default_domains_folder(), example)
        self.planner = PDDLStreamPlanner(self.world, domain_folder)

        self.get_logger().info("Planning node ready.")

        if self.get_parameter("subscribe").value == True:
            self.get_logger().info("Waiting for goal specification...")
            # Subscriber to task plan
            self.goalspec_sub = self.create_subscription(
                GoalSpecification, "goal_specification", self.goalspec_callback, 10
            )
        else:
            if example == "01_simple":
                # Task specification for simple example.
                self.latest_goal = [
                    ("At", "robot", "bedroom"),
                    ("At", "apple0", "table0_tabletop"),
                    ("At", "banana0", "counter0_left"),
                    ("Holding", "robot", "water0"),
                ]
            elif example in [
                "02_derived",
                "03_nav_stream",
                "04_nav_manip_stream",
                "05_nav_grasp_stream",
            ]:
                # Task specification for derived predicate example.
                self.latest_goal = [
                    ("Has", "desk0_desktop", "banana0"),
                    ("Has", "counter", "apple1"),
                    ("HasNone", "bathroom", "banana"),
                    ("HasAll", "table", "water"),
                ]
            else:
                print(f"Invalid example: {example}")
                return
            time.sleep(2.0)

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if (not self.planning) and self.latest_goal:
            self.request_world_state()

        if self.world_state_future_response and self.world_state_future_response.done():
            self.do_plan()

    def request_world_state(self):
        """Requests a world state from the world."""
        self.planning = True
        self.get_logger().info("Requesting world state...")
        self.world_state_future_response = self.world_state_client.call_async(
            RequestWorldState.Request()
        )

    def goalspec_callback(self, msg):
        """
        Handle goal specification callback.

        :param msg: Goal specification message to process.
        :type msg: :class:`pyrobosim_msgs.msg.TaskPlan`
        """
        print("Received new goal specification!")
        self.latest_goal = goal_specification_from_ros(msg, self.world)

    def do_plan(self):
        """Search for a plan and publish it."""
        if not self.latest_goal:
            return

        # Unpack the latest world state.
        try:
            result = self.world_state_future_response.result()
            update_world_from_state_msg(self.world, result.state)
        except Exception as e:
            self.get_logger().info("Failed to unpack world state.")

        # Once the world state is set, plan using the first robot.
        self.get_logger().info("Planning...")
        robot = self.world.robots[0]
        plan = self.planner.plan(
            robot,
            self.latest_goal,
            max_attempts=3,
            search_sample_ratio=self.get_parameter("search_sample_ratio").value,
            planner="ff-astar",
            max_planner_time=10.0,
            max_time=60.0,
        )
        if self.get_parameter("verbose").value == True:
            self.get_logger().info(f"{plan}")

        if plan:
            plan_msg = task_plan_to_ros(plan)
            self.plan_pub.publish(plan_msg)
        self.latest_goal = None
        self.planning = False


def main():
    rclpy.init()
    planner_node = PlannerNode()

    rclpy.spin(planner_node)

    planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
