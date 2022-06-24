#!/usr/bin/env python3

"""
Test script showing how to use a PDDLStream planner as a ROS2 node.
"""

import os
import argparse
import rclpy
from rclpy.node import Node

from pyrobosim.core.ros_interface import update_world_from_state_msg
from pyrobosim.core.yaml import WorldYamlLoader
from pyrobosim.planning.pddlstream.planner import PDDLStreamPlanner
from pyrobosim.planning.pddlstream.utils import get_default_domains_folder
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.ros_conversions import goal_specification_from_ros, task_plan_to_ros
from pyrobosim_msgs.msg import GoalSpecification, TaskPlan
from pyrobosim_msgs.srv import RequestWorldState


def parse_args():
    """ Parse command-line arguments """
    parser = argparse.ArgumentParser(description="PDDLStream demo planner node.")
    parser.add_argument("--example", default="01_simple",
                        help="Example name (01_simple, 02_derived, 03_nav_stream, 04_nav_manip_stream)")
    parser.add_argument("--subscribe", action="store_true",
                        help="If True, waits to receive goal on a subscriber.")
    parser.add_argument("--verbose", action="store_true",
                        help="Print planning output")
    return parser.parse_args()


def load_world():
    """ Load a test world. """
    loader = WorldYamlLoader()
    world_file = "pddlstream_simple_world.yaml"
    data_folder = get_data_folder()
    w = loader.from_yaml(os.path.join(data_folder, world_file))
    return w


class PlannerNode(Node):
    def __init__(self, args, name="pyrobosim"):
        self.name = name
        self.verbose = args.verbose
        self.latest_goal = None
        self.planning = False
        super().__init__(self.name + "_pddlstream_planner", namespace=self.name)

        # Publisher for a task plan
        self.plan_pub = self.create_publisher(
            TaskPlan, "commanded_plan", 10)

        # Service client for world state
        self.world_state_client = self.create_client(
            RequestWorldState, "request_world_state")
        self.world_state_future_response = None
        while not self.world_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for world state server...")

        # Create the world and planner
        self.world = load_world()
        domain_folder = os.path.join(
            get_default_domains_folder(), args.example)
        self.planner = PDDLStreamPlanner(self.world, domain_folder)

        self.get_logger().info("Planning node ready.")

        if args.subscribe:
            self.get_logger().info("Waiting for goal specification...")
            # Subscriber to task plan
            self.goalspec_sub = self.create_subscription(
                GoalSpecification, "goal_specification", self.goalspec_callback, 10)
        else:
            get = lambda entity : self.world.get_entity_by_name(entity)
            if args.example == "01_simple":
                # Task specification for simple example.
                self.latest_goal = [
                    ("At", get("robot"), get("bedroom")),
                    ("At", get("apple0"), get("table0_tabletop")),
                    ("At", get("banana0"), get("counter0_left")),
                    ("Holding", get("robot"), get("water0"))
                ]
            elif args.example in ["02_derived", "03_nav_stream", "04_nav_manip_stream"]:
                # Task specification for derived predicate example.
                self.latest_goal = [
                    ("Has", get("desk0_desktop"), get("banana0")),
                    ("Has", "counter", get("apple1")),
                    ("HasNone", get("bathroom"), "banana"),
                    ("HasAll", "table", "water")
                ]
            else:
                print(f"Invalid example: {args.example}")
                return


    def request_world_state(self):
        """ Requests a world state from the world. """
        self.planning = True
        self.get_logger().info("Requesting world state...")
        self.world_state_future_response = \
            self.world_state_client.call_async(RequestWorldState.Request())


    def goalspec_callback(self, msg):
        """
        Handle goal specification callback.

        :param msg: Goal specification message to process.
        :type msg: :class:`pyrobosim_msgs.msg.TaskPlan`
        """
        print("Received new goal specification!")
        self.latest_goal = goal_specification_from_ros(msg, self.world)
        
    
    def do_plan(self):
        """ Search for a plan and publish it. """
        if not self.latest_goal:
            return

        # Unpack the latest world state.
        try:
            result = self.world_state_future_response.result()
            update_world_from_state_msg(self.world, result.state)
        except Exception as e:
            self.get_logger().info("Failed to unpack world state.")

        # Once the world state is set, plan.
        plan = self.planner.plan(self.latest_goal, focused=True, verbose=self.verbose)
        plan_msg = task_plan_to_ros(plan)
        self.plan_pub.publish(plan_msg)
        self.latest_goal = None
        self.planning = False
        

def main():
    rclpy.init()
    args = parse_args()
    planner_node = PlannerNode(args, name="pddl_demo")

    while rclpy.ok():
        if (not planner_node.planning) and planner_node.latest_goal:
            planner_node.request_world_state()

        if planner_node.world_state_future_response and planner_node.world_state_future_response.done():
            planner_node.do_plan()

        rclpy.spin_once(planner_node)

    planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
