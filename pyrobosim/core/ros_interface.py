""" ROS interfaces to world model. """

import rclpy
from rclpy.node import Node
import threading
from transforms3d.euler import euler2quat

from pyrobosim.msg import RobotState, TaskAction, TaskPlan
from pyrobosim.planning.ros_utils import task_action_from_ros, task_plan_from_ros

class WorldROSWrapper(Node):
    """ ROS2 wrapper node for pyrobosim worlds. """
    def __init__(self, world, name="pyrobosim", state_pub_rate=0.1):
        """
        Creates a ROS2 world wrapper node.

        Given a node name (default is ``"pyrobosim"``), this node will:
            * Subscribe to single actions on the ``pyrobosim/commanded_action`` topic.
            * Subscribe to task plans on the ``pyrobosim/commanded_plan`` topic
            * Publish robot state on the ``pyrobosim/robot_state`` topic

        :param world: World model instance.
        :type world: :class:`pyrobosim.core.world.World`
        :param name: Node name prefix and namespace, defaults to ``"pyrobosim"``.
        :type name: str, optional
        :param state_pub_rate: Rate, in seconds, to publish robot state.
        :type state_pub_rate: float, optional.
        """
        self.name = name
        self.state_pub_rate = state_pub_rate
        super().__init__(self.name + "_world", namespace=self.name)

        # Connect the ROS node to the world
        self.world = world
        self.world.ros_node = self
        self.world.has_ros_node = True

        # Internal state
        self.executing_plan = False
        self.last_command_status = None

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
        """ Starts the node. """
        self.robot_state_pub_thread.start()
        rclpy.spin(self)
        self.destroy_node()
        rclpy.shutdown()


    def action_callback(self, msg):
        """ 
        Handle single action callback. 
        
        :param msg: Task action message to process.
        :type msg: :class:`pyrobosim.msg.TaskAction`
        """
        if self.is_robot_busy():
            self.get_logger().info(f"Currently executing action(s). Discarding this one.")
            return
        t = threading.Thread(target=self.world.robot.execute_action,
                             args=(task_action_from_ros(msg),))
        t.start()


    def plan_callback(self, msg):
        """ 
        Handle task plan callback.
        
        :param msg: Task plan message to process.
        :type msg: :class:`pyrobosim.msg.TaskPlan`
        """
        if self.is_robot_busy():
            self.get_logger().info(f"Currently executing action(s). Discarding this one.")
            return
        t = threading.Thread(target=self.world.robot.execute_plan,
                             args=(task_plan_from_ros(msg),))
        t.start()


    def is_robot_busy(self):
        """
        Check if the robot is currently executing an action or plan. 
        
        :return: True if the robot is busy, else False.
        :rtype: bool
        """
        return self.world.robot.executing_action or self.world.robot.executing_plan


    def publish_robot_state(self):
        """ Publishes the robot state (this function runs on a timer). """
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
