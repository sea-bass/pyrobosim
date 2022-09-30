""" ROS interfaces to world model. """

import rclpy
from rclpy.node import Node
import threading

from pyrobosim_msgs.msg import RobotState, LocationState, ObjectState, WorldState, TaskAction, TaskPlan
from pyrobosim_msgs.srv import RequestWorldState
from pyrobosim.utils.ros_conversions import pose_from_ros, pose_to_ros, task_action_from_ros, task_plan_from_ros


class WorldROSWrapper(Node):
    """ ROS2 wrapper node for pyrobosim worlds. """
    def __init__(self, world=None, name="pyrobosim", state_pub_rate=0.1):
        """
        Creates a ROS2 world wrapper node.

        Given a node name (default is ``"pyrobosim"``), this node will:
            * Subscribe to single actions on the ``pyrobosim/commanded_action`` topic.
            * Subscribe to task plans on the ``pyrobosim/commanded_plan`` topic
            * Publish robot state on the ``pyrobosim/robot_state`` topic

        :param world: World model instance.
        :type world: :class:`pyrobosim.core.world.World`, optional
        :param name: Node name prefix and namespace, defaults to ``"pyrobosim"``.
        :type name: str, optional
        :param state_pub_rate: Rate, in seconds, to publish robot state.
        :type state_pub_rate: float, optional.
        """
        self.name = name
        self.state_pub_rate = state_pub_rate
        super().__init__(self.name)

        # Set world, if one is specified.
        if world:
            self.set_world(world)

        # Internal state
        self.executing_plan = False
        self.last_command_status = None

        # Subscriber to single action
        self.action_sub = self.create_subscription(
            TaskAction, "commanded_action", self.action_callback, 10)

        # Subscriber to task plan
        self.plan_sub = self.create_subscription(
            TaskPlan, "commanded_plan", self.plan_callback, 10)

        # World state service server
        self.world_state_srv = self.create_service(
            RequestWorldState, "request_world_state", self.world_state_callback)
        
        # Initialize robot state publisher and thread list
        self.robot_state_pubs = []
        self.robot_state_pub_threads = []

        self.get_logger().info("World node started.")


    def set_world(self, world):
        """
        Sets a world.

        :param world: World model instance.
        :type world: :class:`pyrobosim.core.world.World`, optional
        """
        self.world = world
        self.world.ros_node = self
        self.world.has_ros_node = True


    def start(self):
        """ Starts the node. """
        if not self.world:
            self.get_logger().error("Must set a world before starting node.")

        for robot in self.world.robots:
            self.add_robot_state_publisher(robot)
        rclpy.spin(self)
        self.destroy_node()
        rclpy.shutdown()


    def add_robot_state_publisher(self, robot):
        """
        Adds a state publisher for a specific robot.
        
        :param robot: Robot instance.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        """
        pub = self.create_publisher(
            RobotState, f"{robot.name}/robot_state", 10)
        self.robot_state_pubs.append(pub)

        pub_fn = lambda: pub.publish(self.package_robot_state(robot))
        pub_thread = threading.Thread(
            target=self.create_timer, 
            args=(self.state_pub_rate, pub_fn))
        self.robot_state_pub_threads.append(pub_thread)
        pub_thread.start()


    def remove_robot_state_publisher(self, robot):
        """
        Removes a state publisher for a specific robot.
        
        :param robot: Robot instance.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        """
        for i, r in self.world.robots:
            if r == robot:
                pub = self.robot_state_pubs.pop(i)
                del(pub)
                pub_thread = self.robot_state_pub_threads.pop(i)
                del(pub_thread)


    def action_callback(self, msg):
        """ 
        Handle single action callback. 
        
        :param msg: Task action message to process.
        :type msg: :class:`pyrobosim_msgs.msg.TaskAction`
        """
        robot = self.world.get_robot_by_name(msg.robot)
        if not robot:
            self.get_logger().info(f"Invalid robot name: {msg.robot}")
            return   
        if self.is_robot_busy(robot):
            self.get_logger().info(f"Currently executing action(s). Discarding this one.")
            return
        t = threading.Thread(target=robot.execute_action,
                             args=(task_action_from_ros(msg),))
        t.start()


    def plan_callback(self, msg):
        """ 
        Handle task plan callback.
        
        :param msg: Task plan message to process.
        :type msg: :class:`pyrobosim_msgs.msg.TaskPlan`
        """
        robot = self.world.get_robot_by_name(msg.robot)
        if not robot:
            self.get_logger().info(f"Invalid robot name: {msg.robot}")
            return   
        if self.is_robot_busy(robot):
            self.get_logger().info(f"Currently executing action(s). Discarding this one.")
            return
        t = threading.Thread(target=robot.execute_plan,
                             args=(task_plan_from_ros(msg),))
        t.start()


    def is_robot_busy(self, robot):
        """
        Check if a robot is currently executing an action or plan. 

        :param robot: Robot instance to check.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        :return: True if the robot is busy, else False.
        :rtype: bool
        """
        return robot.executing_action or robot.executing_plan


    def package_robot_state(self, robot):
        """ 
        Creates a ROS message containing a robot state.
        This state can be published standalone or packaged into the overall world state.
      
        :param robot: Robot instance from which to extract state.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        :return: ROS message representing the robot state.
        :rtype: :class:`pyrobosim_msgs.msg.RobotState`
        """
        state_msg = RobotState(name=robot.name)
        state_msg.pose = pose_to_ros(robot.pose)
        state_msg.executing_action = robot.executing_action
        if robot.manipulated_object is not None:
            state_msg.holding_object = True
            state_msg.manipulated_object = robot.manipulated_object.name
        state_msg.last_visited_location = robot.location.name
        return state_msg


    def world_state_callback(self, request, response):
        """ 
        Returns the world state as a response to a service request. 
        
        :param request: The service request.
        :type request: :class:`pyrobosim_msgs.srv._request_world_state.RequestWorldState_Request`
        :param response: The unmodified service response.
        :type response: :class:`pyrobosim_msgs.srv._request_world_state.RequestWorldState_Response`
        :return: The modified service response containing the world state.
        :rtype: :class:`pyrobosim_msgs.srv._request_world_state.RequestWorldState_Response`
        """
        self.get_logger().info("Received world state request.")

        # Add the object and location states.
        for loc in self.world.locations:
            loc_msg = LocationState(
                name=loc.name, category=loc.category, 
                parent=loc.get_room_name(), pose=pose_to_ros(loc.pose))
            response.state.locations.append(loc_msg)
        for obj in self.world.objects:
            obj_msg = ObjectState(
                name=obj.name, category=obj.category, 
                parent=obj.parent.name, pose=pose_to_ros(obj.pose))
            response.state.objects.append(obj_msg)

        # Add the robot states.
        for robot in self.world.robots:
            response.state.robots.append(self.package_robot_state(robot))

        return response


def update_world_from_state_msg(world, msg):
    """
    Updates a world given a state message.
    
    :param world: World object to update.
    :type world: :class:`pyrobosim.core.world.World`
    :param msg: ROS message describing the desired world state.
    :type msg: :class:`pyrobosim_msgs.msg.WorldState`
    """
    # Update the robot states
    for robot_state in msg.robots:
        robot = world.get_robot_by_name(robot_state.name)
        if robot is not None:
            robot.set_pose(pose_from_ros(robot_state.pose))
        else:
            print(f"Tried to update state for invalid robot {robot_state.name}")

    # Update the object states
    for obj_state in msg.objects:
        world.update_object(obj_state.name, loc=obj_state.parent, 
                            pose=pose_from_ros(obj_state.pose))

    # Update the location states
    for loc_state in msg.locations:
        world.update_location(loc_state.name, room=loc_state.parent, 
                              pose=pose_from_ros(loc_state.pose))
