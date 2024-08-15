""" ROS interfaces to world model. """

from functools import partial
import numpy as np
import os
import time

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from pyrobosim.core.hallway import Hallway
from pyrobosim.core.locations import Location
from pyrobosim_msgs.action import (
    ExecuteTaskAction,
    ExecuteTaskPlan,
    FollowPath,
    PlanPath,
)
from pyrobosim_msgs.msg import ExecutionResult, RobotState, LocationState, ObjectState
from pyrobosim_msgs.srv import RequestWorldState, SetLocationState
from .ros_conversions import (
    execution_result_to_ros,
    path_from_ros,
    path_to_ros,
    pose_from_ros,
    pose_to_ros,
    ros_duration_to_float,
    task_action_from_ros,
    task_plan_from_ros,
)


class WorldROSWrapper(Node):
    """ROS 2 wrapper node for pyrobosim worlds."""

    def __init__(
        self,
        world=None,
        name="pyrobosim",
        num_threads=os.cpu_count(),
        state_pub_rate=0.1,
        dynamics_rate=0.01,
        dynamics_latch_time=0.5,
        dynamics_ramp_down_time=0.5,
        dynamics_enable_collisions=True,
    ):
        """
        Creates a ROS 2 world wrapper node.

        This node will:
            * Publish states for each robot on ``<robot_name>/robot_state`` topics.
            * Subscribe to velocity commands for each robot on ``<robot_name>/cmd_vel`` topics.
            * Allow path planning and following for each robot on ``<robot_name>/plan_path`` and ``<robot_name>/follow_path`` action servers, respectively.
            * Serve a ``request_world_state`` service to retrieve the world state for planning.
            * Serve a ``execute_action`` action server to run single actions on a robot.
            * Serve a ``execute_task_plan`` action server to run entire task plans on a robot.

        :param world: World model instance.
        :type world: :class:`pyrobosim.core.world.World`
        :param name: Node name, defaults to ``"pyrobosim"``.
        :type name: str
        :param num_threads: Number of threads in the multi-threaded executor. Defaults to number of CPUs on the host OS.
        :type num_threads: int
        :param state_pub_rate: Rate, in seconds, to publish robot state.
        :type state_pub_rate: float
        :param dynamics_rate: Rate, in seconds, to update dynamics.
        :type dynamics_rate: float
        :param dynamics_latch_time: Time, in seconds, to latch the latest published velocity commands for a robot.
        :type dynamics_latch_time: float
        :param dynamics_ramp_down_time: Time, in seconds, to ramp down the velocity command to zero. This is applied after the latch time expires.
        :type dynamics_ramp_down_time: float
        :param dynamics_enable_collisions: If true (default), enables collision checking when updating robot dynamics.
        :type dynamics_enable_collisions: bool
        """
        self.name = name
        self.num_threads = num_threads
        self.state_pub_rate = state_pub_rate
        super().__init__(self.name)

        # Set world, if one is specified.
        if world:
            self.set_world(world)

        # Internal state
        self.executing_plan = False
        self.last_command_status = None

        # Server for executing single action
        self.action_server = ActionServer(
            self,
            ExecuteTaskAction,
            "execute_action",
            execute_callback=self.action_callback,
            cancel_callback=self.action_cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # Server for executing task plan
        self.plan_server = ActionServer(
            self,
            ExecuteTaskPlan,
            "execute_task_plan",
            execute_callback=self.plan_callback,
            cancel_callback=self.plan_cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # World state service servers
        self.world_state_srv = self.create_service(
            RequestWorldState,
            "request_world_state",
            self.world_state_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.set_location_state_srv = self.create_service(
            SetLocationState,
            "set_location_state",
            self.set_location_state_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # Initialize robot specific interface lists
        self.robot_command_subs = []
        self.robot_state_pubs = []
        self.robot_state_pub_threads = []
        self.robot_plan_path_servers = []
        self.robot_follow_path_servers = []
        self.latest_robot_cmds = {}

        # Start a dynamics timer
        self.dynamics_rate = dynamics_rate
        self.dynamics_latch_time = dynamics_latch_time
        self.dynamics_ramp_down_time = dynamics_ramp_down_time
        self.dynamics_latch_and_ramp_down_time = (
            self.dynamics_latch_time + self.dynamics_ramp_down_time
        )
        self.dynamics_enable_collisions = dynamics_enable_collisions

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

    def start(self, wait_for_gui=False, auto_spin=True):
        """
        Starts the node.

        :param wait_for_gui: If True, waits for the GUI to come up before starting.
        :type wait_for_gui: bool
        :param auto_spin: If True, creates an executor and spins it indefinitely.
            If you want to handle your own node execution, set this to False.
        :type auto_spin: bool
        """
        if auto_spin:
            executor = MultiThreadedExecutor(num_threads=self.num_threads)
            executor.add_node(self)

        if not self.world:
            self.get_logger().error("Must set a world before starting node.")

        # Create robot specific interfaces
        for robot in self.world.robots:
            self.add_robot_ros_interfaces(robot)

        # Create dynamics timer
        self.dynamics_timer = self.create_timer(
            self.dynamics_rate, self.dynamics_callback
        )

        while wait_for_gui and not self.world.has_gui:
            self.get_logger().info("Waiting for GUI...")
            time.sleep(1.0)
        self.get_logger().info("PyRoboSim ROS node ready!")

        if auto_spin:
            try:
                executor.spin()
            finally:
                executor.remove_node(self)
                self.destroy_node()
                executor.shutdown()
                rclpy.shutdown()

    def add_robot_ros_interfaces(self, robot):
        """
        Adds a velocity command subscriber and state publisher for a specific robot.

        :param robot: Robot instance.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        """
        self.latest_robot_cmds[robot.name] = None

        # Create subscriber
        sub = self.create_subscription(
            Twist,
            f"{robot.name}/cmd_vel",
            lambda msg: self.velocity_command_callback(msg, robot),
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        self.robot_command_subs.append(sub)

        # Create robot state publisher timer
        pub = self.create_publisher(
            RobotState,
            f"{robot.name}/robot_state",
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        pub_fn = partial(self.publish_robot_state, pub=pub, robot=robot)
        pub_timer = self.create_timer(self.state_pub_rate, pub_fn)
        self.robot_state_pubs.append(pub)
        self.robot_state_pub_threads.append(pub_timer)

        # Specialized action servers for path planning and execution.
        plan_path_server = ActionServer(
            self,
            PlanPath,
            f"{robot.name}/plan_path",
            execute_callback=partial(self.robot_path_plan_callback, robot=robot),
            callback_group=ReentrantCallbackGroup(),
        )
        self.robot_plan_path_servers.append(plan_path_server)

        follow_path_server = ActionServer(
            self,
            FollowPath,
            f"{robot.name}/follow_path",
            execute_callback=partial(self.robot_path_follow_callback, robot=robot),
            cancel_callback=partial(self.robot_path_cancel_callback, robot=robot),
            callback_group=ReentrantCallbackGroup(),
        )
        self.robot_follow_path_servers.append(follow_path_server)

    def remove_robot_ros_interfaces(self, robot):
        """
        Removes ROS interfaces for a specific robot.

        :param robot: Robot instance.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        """
        for idx, r in enumerate(self.world.robots):
            if r == robot:
                sub = self.robot_command_subs.pop(idx)
                self.destroy_subscription(sub)
                del sub
                pub = self.robot_state_pubs.pop(idx)
                self.destroy_publisher(pub)
                del pub
                pub_timer = self.robot_state_pub_threads.pop(idx)
                pub_timer.destroy()
                del pub_thread
                plan_path_server = self.robot_plan_path_servers.pop(idx)
                plan_path_server.destroy()
                del plan_path_server
                plan_follow_server = self.robot_follow_path_servers.pop(idx)
                plan_follow_server.destroy()
                del plan_follow_server

    def dynamics_callback(self):
        """
        Updates the dynamics of all spawned robots based on the latest received velocity commands.
        """
        cur_time = self.get_clock().now()

        for robot in self.world.robots:
            cmd = self.latest_robot_cmds.get(robot.name)
            if cmd is not None:
                cmd_time, cmd_vel = cmd

                # Process the velocity commands based on their time.
                elapsed_time = ros_duration_to_float(cur_time - cmd_time)
                if elapsed_time > self.dynamics_latch_and_ramp_down_time:
                    # Complete command timeout, velocity is zero.
                    cmd_vel = np.array([0.0, 0.0, 0.0])
                elif elapsed_time > self.dynamics_latch_time:
                    # Ramping down velocity to zero
                    scaling_factor = (
                        elapsed_time - self.dynamics_latch_time
                    ) / self.dynamics_latch_and_ramp_down_time
                    cmd_vel *= scaling_factor
                # The "else" case is implicit and means the latest velocity command is used as is.

                # Step the dynamics
                robot.dynamics.step(
                    cmd_vel,
                    self.dynamics_rate,
                    self.world,
                    check_collisions=self.dynamics_enable_collisions,
                )

    def velocity_command_callback(self, msg, robot):
        """
        Handle single velocity command callback.

        :param msg: The incoming velocity message.
        :type msg: :class:`geometry_msgs.msg.Twist`
        :param robot: The robot instance corresponding to this message.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        """
        self.latest_robot_cmds[robot.name] = (
            self.get_clock().now(),
            np.array([msg.linear.x, msg.linear.y, msg.angular.z]),
        )

    def action_callback(self, goal_handle):
        """
        Handle single action callback.

        :param goal_handle: Task action goal handle to process.
        :type goal_handle: :class:`pyrobosim_msgs.action.ExecuteTaskAction.Goal`
        :return: The action execution action result.
        :rtype: :class:`pyrobosim_msgs.action.ExecuteTaskAction.Result`
        """
        robot = self.world.get_robot_by_name(goal_handle.request.action.robot)
        if not robot:
            message = f"Invalid robot name: {goal_handle.request.action.robot}"
            self.get_logger().error(message)
            goal_handle.abort()
            return ExecuteTaskAction.Result(
                execution_result=ExecutionResult(
                    status=ExecutionResult.INVALID_ACTION, message=message
                )
            )
        if self.is_robot_busy(robot):
            message = "Currently executing action(s). Discarding this one."
            self.get_logger().warn(message)
            goal_handle.abort()
            return ExecuteTaskAction.Result(
                execution_result=ExecutionResult(
                    status=ExecutionResult.CANCELED, message=message
                )
            )

        # Execute the action
        robot_action = task_action_from_ros(goal_handle.request.action)
        self.get_logger().info(f"Executing action with robot {robot.name}...")
        execution_result = robot.execute_action(robot_action)
        self.get_logger().info(
            f"Action finished with status: {execution_result.status.name}"
        )

        # Package up the result
        goal_handle.succeed()
        return ExecuteTaskAction.Result(
            execution_result=execution_result_to_ros(execution_result)
        )

    def action_cancel_callback(self, goal_handle):
        """
        Handle cancellation for single action goals.

        :param goal_handle: Task action goal handle to process.
        :type goal_handle: :class:`pyrobosim_msgs.action.ExecuteTaskAction.Goal`
        """
        robot = self.world.get_robot_by_name(goal_handle.request.action.robot)
        if robot is not None:
            self.get_logger().info(f"Canceling action for robot {robot.name}.")
            robot.cancel_actions()
        return CancelResponse.ACCEPT

    def plan_callback(self, goal_handle):
        """
        Handle task plan action callback.

        :param goal_handle: Task plan action goal handle to process.
        :type goal_handle: :class:`pyrobosim_msgs.action.ExecuteTaskPlan.Goal`
        :return: The plan execution action result.
        :rtype: :class:`pyrobosim_msgs.action.ExecuteTaskPlan.Result`
        """
        plan_msg = goal_handle.request.plan
        robot = self.world.get_robot_by_name(plan_msg.robot)
        if not robot:
            message = f"Invalid robot name: {plan_msg.robot}"
            self.get_logger().error(message)
            goal_handle.abort()
            return ExecuteTaskAction.Result(
                execution_result=ExecutionResult(
                    status=ExecutionResult.INVALID_ACTION, message=message
                ),
                num_completed=0,
                num_total=len(plan_msg.actions),
            )
        if self.is_robot_busy(robot):
            message = "Currently executing action(s). Discarding this plan."
            self.get_logger().warn(message)
            goal_handle.abort()
            return ExecuteTaskAction.Result(
                execution_result=ExecutionResult(
                    status=ExecutionResult.CANCELED, message=message
                ),
                num_completed=0,
                num_total=len(plan_msg.actions),
            )

        # Execute the plan
        self.get_logger().info(f"Executing task plan with robot {robot.name}...")
        robot_plan = task_plan_from_ros(plan_msg)
        execution_result, num_completed = robot.execute_plan(robot_plan)
        self.get_logger().info(
            f"Plan finished with status: {execution_result.status.name} (completed {num_completed}/{robot_plan.size()} actions)"
        )

        # Package up the result
        goal_handle.succeed()
        return ExecuteTaskPlan.Result(
            execution_result=execution_result_to_ros(execution_result),
            num_completed=num_completed,
            num_total=robot_plan.size(),
        )

    def plan_cancel_callback(self, goal_handle):
        """
        Handle cancellation for task plans.

        :param goal_handle: Task plan goal handle to process.
        :type goal_handle: :class:`pyrobosim_msgs.action.ExecuteTaskPlan.Goal`
        """
        robot = self.world.get_robot_by_name(goal_handle.request.plan.robot)
        if robot is not None:
            self.get_logger().info(f"Canceling plan for robot {robot.name}.")
            robot.cancel_actions()
        return CancelResponse.ACCEPT

    def robot_path_plan_callback(self, goal_handle, robot=None):
        """
        Handle path planning action callback for a specific robot.

        :param goal_handle: Path planning action goal handle to process.
        :type goal_handle: :class:`pyrobosim_msgs.action.PlanPath.Goal`
        :param robot: The robot instance corresponding to this request.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        :return: The path planning action result.
        :rtype: :class:`pyrobosim_msgs.action.PlanPath.Result`
        """
        goal = goal_handle.request.target_location or pose_from_ros(
            goal_handle.request.target_pose
        )

        path = robot.plan_path(goal=goal)
        goal_handle.succeed()

        if path is None:
            return PlanPath.Result(
                execution_result=ExecutionResult(
                    status=ExecutionResult.PLANNING_FAILURE,
                    message="Path planning failed.",
                )
            )

        return PlanPath.Result(
            execution_result=ExecutionResult(status=ExecutionResult.SUCCESS),
            path=path_to_ros(path),
        )

    def robot_path_follow_callback(self, goal_handle, robot=None):
        """
        Handle path following action callback for a specific robot.

        :param goal_handle: Path following action goal handle to process.
        :type goal_handle: :class:`pyrobosim_msgs.action.FollowPath.Goal`
        :param robot: The robot instance corresponding to this request.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        :return: The path following action result.
        :rtype: :class:`pyrobosim_msgs.action.FollowPath.Result`
        """
        path = path_from_ros(goal_handle.request.path)
        robot.follow_path(path, blocking=False)

        while robot.executing_nav and not (
            goal_handle.status == GoalStatus.STATUS_CANCELED
        ):
            if goal_handle.is_cancel_requested:
                robot.cancel_actions()
                goal_handle.canceled()
            time.sleep(0.5)

        if self.world.has_gui:
            self.world.gui.set_buttons_during_action(True)

        if goal_handle.status == GoalStatus.STATUS_CANCELED:
            return FollowPath.Result(
                execution_result=ExecutionResult(status=ExecutionResult.CANCELED),
                message="Path following canceled.",
            )
        goal_handle.succeed()
        return FollowPath.Result(
            execution_result=execution_result_to_ros(robot.last_nav_result)
        )

    def robot_path_cancel_callback(self, goal_handle, robot=None):
        """
        Handle a cancel request for the robot path following action.

        :param goal_handle: Path following action goal handle to cancel.
        :type goal_handle: :class:`pyrobosim_msgs.action.FollowPath.Goal`
        :param robot: The robot instance corresponding to this request.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        :return: The goal handle cancellation response.
        :rtype: :class:`rclpy.action.CancelResponse`
        """
        self.get_logger().info(f"Canceling path following for {robot}.")
        return CancelResponse.ACCEPT

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
        state_msg.pose = pose_to_ros(robot.get_pose())
        state_msg.battery_level = robot.battery_level
        state_msg.executing_action = robot.executing_action
        if robot.manipulated_object is not None:
            state_msg.holding_object = True
            state_msg.manipulated_object = robot.manipulated_object.name
        if isinstance(robot.location, str):
            state_msg.last_visited_location = robot.location
        else:
            state_msg.last_visited_location = robot.location.name
        return state_msg

    def publish_robot_state(self, pub, robot):
        """
        Helper function to publish robot state.

        :param pub: The publisher on which to publish the robot state information.
        :type pub: :class:`rclpy.publisher.Publisher`
        :param robot: Robot instance from which to extract state.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        """
        pub.publish(self.package_robot_state(robot))

    def world_state_callback(self, request, response):
        """
        Returns the world state as a response to a service request.

        :param request: The service request.
        :type request: :class:`pyrobosim_msgs.srv.RequestWorldState.Request`
        :param response: The unmodified service response.
        :type response: :class:`pyrobosim_msgs.srv.RequestWorldState.Response`
        :return: The modified service response containing the world state.
        :rtype: :class:`pyrobosim_msgs.srv.RequestWorldState.Response`
        """
        self.get_logger().info("Received world state request.")

        # Determine whether to use a robot's local observations or the full world state.
        if request.robot:
            robot = self.world.get_robot_by_name(request.robot)
            if not robot:
                return response  # empty response
            objects = robot.get_known_objects()
        else:
            objects = self.world.objects

        # Add the object and location states.
        # TODO: Support hallway states as well.
        for loc in self.world.locations:
            loc_msg = LocationState(
                name=loc.name,
                category=loc.category,
                parent=loc.get_room_name(),
                pose=pose_to_ros(loc.pose),
                is_open=loc.is_open,
                is_locked=loc.is_locked,
            )
            response.state.locations.append(loc_msg)
        for obj in objects:
            obj_msg = ObjectState(
                name=obj.name,
                category=obj.category,
                parent=obj.parent.name,
                pose=pose_to_ros(obj.pose),
            )
            response.state.objects.append(obj_msg)

        # Add the robot states.
        for robot in self.world.robots:
            response.state.robots.append(self.package_robot_state(robot))

        return response

    def set_location_state_callback(self, request, response):
        """
        Sets the state of a location in the world as a response to a service request.

        :param request: The service request.
        :type request: :class:`pyrobosim_msgs.srv.SetLocationState.Request`
        :param response: The unmodified service response.
        :type response: :class:`pyrobosim_msgs.srv.SetLocationState.Response`
        :return: The modified service response containing result of setting the location state.
        :rtype: :class:`pyrobosim_msgs.srv.RequestWorldState.Response`
        """
        self.get_logger().info("Received location state setting request.")

        # Check if the entity exists.
        entity = self.world.get_entity_by_name(request.location_name)
        if not entity:
            message = f"No location matching query: {request.location_name}"
            self.get_logger().warn(message)
            response.result.status = ExecutionResult.INVALID_ACTION
            response.result.message = message
            return response

        if isinstance(entity, Location):
            # Try open or close the location if its status needs to be toggled.
            if request.open != entity.is_open:
                if request.open:
                    result = self.world.open_location(entity)
                else:
                    result = self.world.close_location(entity)

                if not result.is_success():
                    response.result = execution_result_to_ros(result)
                    return response

            # Try lock or unlock the location if its status needs to be toggled.
            if request.lock != entity.is_locked:
                if request.lock:
                    result = self.world.lock_location(entity)
                else:
                    result = self.world.unlock_location(entity)

                if not result.is_success():
                    response.result = execution_result_to_ros(result)
                    return response

            response.result.status = ExecutionResult.SUCCESS
            return response

        elif isinstance(entity, Hallway):
            # Try open or close the hallway if its status needs to be toggled.
            if request.open != entity.is_open:
                if request.open:
                    result = self.world.open_hallway(entity)
                else:
                    result = self.world.close_hallway(entity)

                if not result.is_success():
                    response.result = execution_result_to_ros(result)
                    return response

            # Try lock or unlock the hallway if its status needs to be toggled.
            if request.lock != entity.is_locked:
                if request.lock:
                    result = self.world.lock_hallway(entity)
                else:
                    result = self.world.unlock_hallway(entity)

                if not result.is_success():
                    response.result = execution_result_to_ros(result)
                    return response

            response.result.status = ExecutionResult.SUCCESS
            return response

        # If no valid entity type is reached, we should fail here.
        message = f"Cannot set state for {entity.name} since it is of type {type(entity).__name__}."
        self.get_logger().warn(message)
        response.result.status = ExecutionResult.INVALID_ACTION
        response.result.message = message
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
        world.update_object(
            obj_state.name, loc=obj_state.parent, pose=pose_from_ros(obj_state.pose)
        )

    # Update the location states
    for loc_state in msg.locations:
        world.update_location(
            loc_state.name,
            room=loc_state.parent,
            pose=pose_from_ros(loc_state.pose),
            is_open=loc_state.is_open,
            is_locked=loc_state.is_locked,
        )

    # TODO: Update the hallway states once this is supported.
