"""
ROS interface to world model
"""

import time
from rclpy.node import Node

from pyrobosim.msg import TaskAction, TaskPlan


class WorldROSWrapper(Node):
    def __init__(self, world):
        super().__init__("world_ros_wrapper")

        # Connect the ROS node to the world
        self.world = world
        self.world.ros_node = self
        self.world.has_ros_node = True

        # Subscriber to single action
        self.action_sub = self.create_subscription(
            TaskAction, "/commanded_action", self.action_callback, 10)

        # Subscriber to task plan
        self.plan_sub = self.create_subscription(
            TaskPlan, "/commanded_plan", self.plan_callback, 10)


        print("World ROS node started")

    def handle_action(self, msg):
        """ Handles an action and returns its success """
        if msg.type == "navigate":
            if self.world.has_gui:
                success = self.world.gui.navigate(msg.target_location)
            else:
                path = self.world.find_path(msg.target_location)
                success = self.world.execute_path(path, dt=0.1, realtime_factor=1.0,
                                                  linear_velocity=1.0, max_angular_velocity=None)
        elif msg.type == "pick":
            if self.world.has_gui:
                success = self.world.gui.pick_object(None)
            else:
                success = self.world.pick_object(None)
        elif msg.type == "place":
            if self.world.has_gui:
                success = self.world.gui.place_object(None)
            else:
                success = self.world.place_object(None)
        else:
            success = False
        
        if not success:
            self.get_logger().info(f"Failed to execute action {msg.type}")
        return success


    def action_callback(self, msg):
        """ Handle single action callback """
        self.get_logger().info(f"Executing action {msg.type}")
        success = self.handle_action(msg)


    def plan_callback(self, msg):
        """ Handle task plan callback """
        self.get_logger().info(f"Executing task plan...")
        for act_msg in msg.actions:
            success = self.handle_action(act_msg)
            if not success:
                return
            time.sleep(0.5)
