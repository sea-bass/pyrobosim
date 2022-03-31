"""
ROS interface to world model
"""

import time
import threading
import rclpy
from rclpy.node import Node
from transforms3d.euler import euler2quat

from pyrobosim.msg import RobotState, TaskAction, TaskPlan


class WorldROSWrapper(Node):
    def __init__(self, world, name="pyrobosim", state_pub_rate=0.1):
        self.name = name
        self.state_pub_rate = state_pub_rate
        super().__init__(self.name + "_world", namespace=self.name)

        # Connect the ROS node to the world
        self.world = world
        self.world.ros_node = self
        self.world.has_ros_node = True

        # Subscriber to single action
        self.action_sub = self.create_subscription(
            TaskAction, "commanded_action", self.action_callback, 10)

        # Subscriber to task plan
        self.plan_sub = self.create_subscription(
            TaskPlan, "commanded_plan", self.plan_callback, 10)

        # Robot state publisher
        self.robot_state_pub = self.create_publisher(
            RobotState, "robot_state", 10)
        self.robot_state_pub_thread = threading.Thread(
            target=self.create_timer, 
            args=(self.state_pub_rate, self.publish_robot_state))

        self.get_logger().info("World node started")


    def start(self):
        """ Starts the node """
        self.robot_state_pub_thread.start()
        rclpy.spin(self)
        self.destroy_node()
        rclpy.shutdown()


    def action_callback(self, msg):
        """ Handle single action callback """
        self.get_logger().info(f"Executing action {msg.type}")
        success = self.execute_action(msg)
        self.get_logger().info(f"Action completed with success: {success}")


    def execute_action(self, msg):
        """ Executes an action and returns its success """
        if msg.type == "navigate":
            if self.world.has_gui:
                success = self.world.gui.navigate(msg.target_location)
            else:
                path = self.world.find_path(msg.target_location)
                success = self.world.execute_path(path, dt=0.1, realtime_factor=1.0,
                                                  linear_velocity=1.0, max_angular_velocity=None)
        elif msg.type == "pick":
            if self.world.has_gui:
                success = self.world.gui.pick_object(msg.object)
            else:
                success = self.world.pick_object(msg.object)
        elif msg.type == "place":
            if self.world.has_gui:
                success = self.world.gui.place_object(None)
            else:
                success = self.world.place_object(None)
        else:
            self.get_logger().info(f"Invalid action type: {msg.type}")
            success = False
        return success


    def plan_callback(self, msg):
        """ Handle task plan callback """
        self.get_logger().info(f"Executing task plan...")
        num_acts = len(msg.actions)
        for n, act_msg in enumerate(msg.actions):
            self.get_logger().info(
                f"Executing action {act_msg.type} [{n+1}/{num_acts}]")
            success = self.execute_action(act_msg)
            if not success:
                self.get_logger().info(f"Task plan failed to execute on action {n+1}")
                return
            time.sleep(0.5) # Artificial delay between actions
        self.get_logger().info(f"Task plan executed successfully")


    def publish_robot_state(self):
        robot = self.world.robot
        if robot:
            state_msg = RobotState()
            state_msg.pose.position.x = robot.pose.x
            state_msg.pose.position.y = robot.pose.y
            state_msg.pose.position.z = robot.pose.z
            quat = euler2quat(0, 0, robot.pose.yaw)
            state_msg.pose.orientation.w = quat[0]
            state_msg.pose.orientation.x = quat[1]
            state_msg.pose.orientation.y = quat[2]
            state_msg.pose.orientation.z = quat[3]
            state_msg.executing_action = robot.executing_action
            if robot.manipulated_object is not None:
                state_msg.holding_object = True
                state_msg.manipulated_object = robot.manipulated_object.name
            state_msg.last_visited_location = robot.location.name

            self.robot_state_pub.publish(state_msg)
