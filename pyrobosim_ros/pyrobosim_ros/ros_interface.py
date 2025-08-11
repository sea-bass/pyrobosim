"""ROS interfaces to world model."""

from functools import partial
import numpy as np
import numpy.typing as npt
import os
import time
from threading import Thread

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import Executor, MultiThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.service import Service
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.time import Time
from rclpy.timer import Timer

from geometry_msgs.msg import Twist
from pyrobosim.core import Robot, World
from pyrobosim.utils.logging import set_global_logger
from pyrobosim_msgs.action import (  # type: ignore[attr-defined]
    DetectObjects,
    ExecuteTaskAction,
    ExecuteTaskPlan,
    FollowPath,
    PlanPath,
)
from pyrobosim_msgs.msg import (  # type: ignore[attr-defined]
    ExecutionResult,
    HallwayState,
    LocationState,
    ObjectState,
    RobotState,
    WorldState,
)
from pyrobosim_msgs.srv import (  # type: ignore[attr-defined]
    RequestWorldInfo,
    RequestWorldState,
    ResetWorld,
    SetLocationState,
)
from std_srvs.srv import Trigger

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


class WorldROSWrapper(Node):  # type: ignore[misc]
    """ROS 2 wrapper node for PyRoboSim worlds."""

    def __init__(
        self,
        world: World | None = None,
        name: str = "pyrobosim",
        num_threads: int | None = os.cpu_count(),
        state_pub_rate: float = 0.1,
        dynamics_rate: float = 0.01,
        dynamics_latch_time: float = 0.5,
        dynamics_ramp_down_time: float = 0.5,
        dynamics_enable_collisions: bool = True,
    ) -> None:
        """
        Creates a ROS 2 world wrapper node.

        This node will:
            * Publish states for each robot on ``<robot_name>/robot_state`` topics.
            * Subscribe to velocity commands for each robot on ``<robot_name>/cmd_vel`` topics.
            * Allow path planning and following for each robot on ``<robot_name>/plan_path`` and ``<robot_name>/follow_path`` action servers, respectively.
            * Allow path planner reset on a ``<robot_name>/reset_path_planner`` service server.
            * Allow object detection for each robot on a ``<robot_name>/detect_objects`` action server.
            * Serve ``request_world_info`` and ``request_world_state`` services to retrieve the world information and state, respectively, for planning.
            * Serve a ``set_location_state`` service to set the state of the location.
            * Serve a ``reset_world`` service to reset the world.
            * Serve a ``execute_action`` action server to run single actions on a robot.
            * Serve a ``execute_task_plan`` action server to run entire task plans on a robot.

        :param world: World model instance.
        :param name: Node name, defaults to ``"pyrobosim"``.
        :param num_threads: Number of threads in the multi-threaded executor. Defaults to number of CPUs on the host OS.
        :param state_pub_rate: Rate, in seconds, to publish robot state.
        :param dynamics_rate: Rate, in seconds, to update dynamics.
        :param dynamics_latch_time: Time, in seconds, to latch the latest published velocity commands for a robot.
        :param dynamics_ramp_down_time: Time, in seconds, to ramp down the velocity command to zero. This is applied after the latch time expires.
        :param dynamics_enable_collisions: If true (default), enables collision checking when updating robot dynamics.
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
        self.executor: Executor | None = None

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

        # World info and state service servers
        self.world_state_callback_group = ReentrantCallbackGroup()

        self.world_info_srv = self.create_service(
            RequestWorldInfo,
            "request_world_info",
            self.world_info_callback,
            callback_group=self.world_state_callback_group,
        )

        self.world_state_srv = self.create_service(
            RequestWorldState,
            "request_world_state",
            self.world_state_callback,
            callback_group=self.world_state_callback_group,
        )

        self.set_location_state_srv = self.create_service(
            SetLocationState,
            "set_location_state",
            self.set_location_state_callback,
            callback_group=self.world_state_callback_group,
        )

        self.reset_world_srv = self.create_service(
            ResetWorld,
            "reset_world",
            self.reset_world_callback,
            callback_group=self.world_state_callback_group,
        )

        # Initialize robot specific interface dictionaries
        self.robot_command_subs: dict[str, Subscription[Twist]] = {}
        self.robot_state_pubs: dict[str, Publisher[RobotState]] = {}
        self.robot_state_pub_timers: dict[str, Timer] = {}
        self.robot_plan_path_servers: dict[
            str, ActionServer[PlanPath.Goal, PlanPath.Result, PlanPath.Feedback]
        ] = {}
        self.robot_follow_path_servers: dict[
            str, ActionServer[FollowPath.Goal, FollowPath.Result, FollowPath.Feedback]
        ] = {}
        self.robot_object_detection_servers: dict[
            str,
            ActionServer[
                DetectObjects.Goal, DetectObjects.Result, DetectObjects.Feedback
            ],
        ] = {}
        self.robot_reset_path_planner_servers: dict[
            str, Service[Trigger.Request, Trigger.Response]
        ] = {}
        self.latest_robot_cmds: dict[
            str, tuple[Time, npt.NDArray[np.float32]] | None
        ] = {}

        # Start a dynamics timer
        self.dynamics_rate = dynamics_rate
        self.dynamics_latch_time = dynamics_latch_time
        self.dynamics_ramp_down_time = dynamics_ramp_down_time
        self.dynamics_latch_and_ramp_down_time = (
            self.dynamics_latch_time + self.dynamics_ramp_down_time
        )
        self.dynamics_enable_collisions = dynamics_enable_collisions

        self.get_logger().info("World node started.")

    def set_world(self, world: World) -> None:
        """
        Sets a world.

        :param world: World model instance.
        """
        self.world = world
        self.world.ros_node = self

        set_global_logger(get_logger("pyrobosim"))
        self.world.logger = get_logger(self.world.name)
        self.world.logger.info("Configured ROS node.")

    def start(self, wait_for_gui: bool = False, auto_spin: bool = True) -> None:
        """
        Starts the node.

        :param wait_for_gui: If True, waits for the GUI to come up before starting.
        :param auto_spin: If True, creates an executor and spins it indefinitely.
            If you want to handle your own node execution, set this to False.
        """
        if auto_spin:
            executor = MultiThreadedExecutor(num_threads=self.num_threads)
            executor.add_node(self)
            self.executor = executor

        if not self.world:
            self.get_logger().error("Must set a world before starting node.")

        # Create robot specific interfaces
        for robot in self.world.robots:
            self.add_robot_ros_interfaces(robot)

        # Create dynamics timer
        self.dynamics_timer = self.create_timer(
            self.dynamics_rate, self.dynamics_callback
        )

        while wait_for_gui and not self.world.gui is not None:
            self.get_logger().info("Waiting for GUI...")
            time.sleep(1.0)
        self.get_logger().info("PyRoboSim ROS node ready!")

        if auto_spin:
            try:
                executor.spin()
            finally:
                self.shutdown()

    def shutdown(self) -> None:
        """Shuts down cleanly."""
        if self.executor:
            self.executor.remove_node(self)
            self.destroy_node()
            self.executor.shutdown()
            self.executor = None
        if rclpy.ok():
            rclpy.shutdown()

    def add_robot_ros_interfaces(self, robot: Robot) -> None:
        """
        Adds a velocity command subscriber and state publisher for a specific robot.

        :param robot: Robot instance.
        """
        self.latest_robot_cmds[robot.name] = None

        # Create subscriber
        if robot.name not in self.robot_command_subs:
            self.robot_command_subs[robot.name] = self.create_subscription(
                Twist,
                f"{robot.name}/cmd_vel",
                lambda msg: self.velocity_command_callback(msg, robot),
                10,
                callback_group=ReentrantCallbackGroup(),
            )

        # Create robot state publisher timer
        if robot.name not in self.robot_state_pubs:
            self.robot_state_pubs[robot.name] = self.create_publisher(
                RobotState,
                f"{robot.name}/robot_state",
                10,
                callback_group=ReentrantCallbackGroup(),
            )

        if robot.name not in self.robot_state_pub_timers:
            pub_fn = partial(
                self.publish_robot_state,
                pub=self.robot_state_pubs[robot.name],
                robot=robot,
            )
            pub_timer = self.create_timer(self.state_pub_rate, pub_fn)
            self.robot_state_pub_timers[robot.name] = pub_timer

        # Specialized action servers for path planning and execution.
        robot_action_callback_group = ReentrantCallbackGroup()

        if robot.name not in self.robot_plan_path_servers:
            self.robot_plan_path_servers[robot.name] = ActionServer(
                self,
                PlanPath,
                f"{robot.name}/plan_path",
                execute_callback=partial(self.robot_path_plan_callback, robot=robot),
                callback_group=robot_action_callback_group,
            )

        if robot.name not in self.robot_follow_path_servers:
            self.robot_follow_path_servers[robot.name] = ActionServer(
                self,
                FollowPath,
                f"{robot.name}/follow_path",
                execute_callback=partial(self.robot_path_follow_callback, robot=robot),
                cancel_callback=partial(self.robot_path_cancel_callback, robot=robot),
                callback_group=robot_action_callback_group,
            )

        # Service server for resetting path planner.
        if robot.name not in self.robot_reset_path_planner_servers:
            self.robot_reset_path_planner_servers[robot.name] = self.create_service(
                Trigger,
                f"{robot.name}/reset_path_planner",
                partial(self.robot_path_planner_reset_callback, robot=robot),
                callback_group=ReentrantCallbackGroup(),
            )

        # Specialized action server for object detection.
        if robot.name not in self.robot_object_detection_servers:
            self.robot_object_detection_servers[robot.name] = ActionServer(
                self,
                DetectObjects,
                f"{robot.name}/detect_objects",
                execute_callback=partial(
                    self.robot_detect_objects_callback, robot=robot
                ),
                callback_group=robot_action_callback_group,
            )

        # Set up logger interface for robot
        robot.logger = get_logger(robot.name)
        robot.logger.info("Configured ROS logger.")

        if self.executor is not None:
            self.executor.wake()

    def remove_robot_ros_interfaces(self, robot: Robot | str) -> None:
        """
        Removes ROS interfaces for a specific robot.

        :param robot: Robot instance or name.
        """
        name = robot if isinstance(robot, str) else robot.name

        sub = self.robot_command_subs.get(name)
        if sub:
            self.destroy_subscription(sub)
            del self.robot_command_subs[name]

        pub = self.robot_state_pubs.get(name)
        if pub:
            self.destroy_publisher(pub)
            del self.robot_state_pubs[name]

        pub_timer = self.robot_state_pub_timers.get(name)
        if pub_timer:
            pub_timer.destroy()
            del self.robot_state_pub_timers[name]

        plan_path_server = self.robot_plan_path_servers.get(name)
        if plan_path_server:
            plan_path_server.destroy()
            del self.robot_plan_path_servers[name]

        plan_follow_server = self.robot_follow_path_servers.get(name)
        if plan_follow_server:
            plan_follow_server.destroy()
            del self.robot_follow_path_servers[name]

        reset_path_planner_server = self.robot_reset_path_planner_servers.get(name)
        if reset_path_planner_server:
            self.destroy_service(reset_path_planner_server)
            del self.robot_reset_path_planner_servers[name]

        detect_objects_server = self.robot_object_detection_servers.get(name)
        if detect_objects_server:
            detect_objects_server.destroy()
            del self.robot_object_detection_servers[name]

        if self.executor is not None:
            self.executor.wake()

    def dynamics_callback(self) -> None:
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
                target_pose = robot.dynamics.step(cmd_vel, self.dynamics_rate)
                if robot.is_in_collision(pose=target_pose):
                    robot.dynamics.velocity = np.array([0.0, 0.0, 0.0])
                    continue
                robot.set_pose(target_pose)

    def velocity_command_callback(self, msg: Twist, robot: Robot) -> None:
        """
        Handle single velocity command callback.

        :param msg: The incoming velocity message.
        :param robot: The robot instance corresponding to this message.
        """
        self.latest_robot_cmds[robot.name] = (
            self.get_clock().now(),
            np.array([msg.linear.x, msg.linear.y, msg.angular.z]),
        )

    def action_callback(
        self, goal_handle: ExecuteTaskAction.Goal
    ) -> ExecuteTaskAction.Result:
        """
        Handle single action callback.

        :param goal_handle: Task action goal handle to process.
        :return: The action execution action result.
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
        if robot.is_busy():
            message = "Currently executing action(s). Discarding this one."
            self.get_logger().warning(message)
            goal_handle.abort()
            return ExecuteTaskAction.Result(
                execution_result=ExecutionResult(
                    status=ExecutionResult.CANCELED, message=message
                )
            )

        # Execute the action
        robot_action = task_action_from_ros(goal_handle.request.action)
        realtime_factor = goal_handle.request.realtime_factor
        self.get_logger().info(
            f"Executing action {robot_action.type} with robot {robot.name}..."
        )
        execution_result = robot.execute_action(robot_action, realtime_factor)
        self.get_logger().info(
            f"Action {robot_action.type} finished with status: {execution_result.status.name}"
        )

        # Package up the result
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return ExecuteTaskAction.Result(
            execution_result=execution_result_to_ros(execution_result)
        )

    def action_cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:  # type: ignore[type-arg] # Cannot add type args in Humble and Jazzy
        """
        Handle cancellation for single action goals.

        :param goal_handle: Task action goal handle to cancel.
        :return: The goal handle cancellation response.
        """
        robot = self.world.get_robot_by_name(goal_handle.request.action.robot)
        if robot is not None:
            if not robot.is_busy():
                self.get_logger().info(
                    f"Robot {robot.name} is not busy. Not canceling action."
                )
                return CancelResponse.REJECT

            self.get_logger().info(f"Canceling action for robot {robot.name}.")
            Thread(target=robot.cancel_actions).start()
        return CancelResponse.ACCEPT

    def plan_callback(
        self, goal_handle: ExecuteTaskPlan.Goal
    ) -> ExecuteTaskPlan.Result:
        """
        Handle task plan action callback.

        :param goal_handle: Task plan action goal handle to process.
        :return: The plan execution action result.
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
        if robot.is_busy():
            message = "Currently executing action(s). Discarding this plan."
            self.get_logger().warning(message)
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
        delay = goal_handle.request.delay
        realtime_factor = goal_handle.request.realtime_factor
        execution_result, num_completed = robot.execute_plan(
            robot_plan, delay=delay, realtime_factor=realtime_factor
        )
        self.get_logger().info(
            f"Plan finished with status: {execution_result.status.name} (completed {num_completed}/{robot_plan.size()} actions)"
        )

        # Package up the result
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return ExecuteTaskPlan.Result(
            execution_result=execution_result_to_ros(execution_result),
            num_completed=num_completed,
            num_total=robot_plan.size(),
        )

    def plan_cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:  # type: ignore[type-arg] # Cannot add type args in Humble and Jazzy
        """
        Handle cancellation for task plans.

        :param goal_handle: Task plan goal handle to cancel.
        :return: The goal handle cancellation response.
        """
        robot = self.world.get_robot_by_name(goal_handle.request.plan.robot)
        if robot is not None:
            if not robot.is_busy():
                self.get_logger().info(
                    f"Robot {robot.name} is not busy. Not canceling action."
                )
                return CancelResponse.REJECT

            self.get_logger().info(f"Canceling plan for robot {robot.name}.")
            Thread(target=robot.cancel_actions).start()
        return CancelResponse.ACCEPT

    def robot_path_plan_callback(self, goal_handle: ServerGoalHandle, robot: Robot) -> PlanPath.Result:  # type: ignore[type-arg] # Cannot add type args in Humble and Jazzy
        """
        Handle path planning action callback for a specific robot.

        :param goal_handle: Path planning action goal handle to process.
        :param robot: The robot instance corresponding to this request.
        :return: The path planning action result.
        """
        goal = goal_handle.request.target_location or pose_from_ros(
            goal_handle.request.target_pose
        )

        path = robot.plan_path(goal=goal)
        goal_handle.succeed()

        if path is None or path.num_poses == 0:
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

    def robot_path_follow_callback(self, goal_handle: ServerGoalHandle, robot: Robot) -> FollowPath.Result:  # type: ignore[type-arg] # Cannot add type args in Humble and Jazzy
        """
        Handle path following action callback for a specific robot.

        :param goal_handle: Path following action goal handle to process.
        :param robot: The robot instance corresponding to this request.
        :return: The path following action result.
        """

        # Follow path in a separate thread so we can check for cancellation in parallel.
        path = path_from_ros(goal_handle.request.path)
        Thread(target=robot.follow_path, args=(path,)).start()

        if self.world.gui is not None:
            self.world.gui.set_buttons_during_action(False)

        while robot.executing_nav and goal_handle.status != GoalStatus.STATUS_CANCELED:
            if goal_handle.is_cancel_requested:
                robot.cancel_actions()
                goal_handle.canceled()
                break
            time.sleep(0.1)

        if self.world.gui is not None:
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

    def robot_path_cancel_callback(self, goal_handle: ServerGoalHandle, robot: Robot) -> CancelResponse:  # type: ignore[type-arg] # Cannot add type args in Humble and Jazzy
        """
        Handle a cancel request for the robot path following action.

        :param goal_handle: Path following action goal handle to cancel.
        :param robot: The robot instance corresponding to this request.
        :return: The goal handle cancellation response.
        """
        self.get_logger().info(f"Canceling path following for {robot}.")
        return CancelResponse.ACCEPT

    def robot_path_planner_reset_callback(
        self, request: Trigger.Request, response: Trigger.Response, robot: Robot
    ) -> Trigger.Response:
        """
        Resets a robot's path planner as a response to a service request.

        :param request: The service request.
        :param response: The unmodified service response.
        :param robot: The robot for which to reset the path planner.
        :return: The modified service response containing the reset result.
        """
        self.get_logger().info(f"Resetting path planner for {robot}.")

        if not robot.path_planner:
            message = f"{robot} does not have a path planner. Cannot reset."
            self.get_logger().warning(message)
            return Trigger.Response(success=False, message=message)

        robot.reset_path_planner()
        return Trigger.Response(success=True)

    def robot_detect_objects_callback(self, goal_handle: ServerGoalHandle, robot: Robot) -> DetectObjects.Result:  # type: ignore[type-arg] # Cannot add type args in Humble and Jazzy
        """
        Handle object detection action callback for a specific robot.

        :param goal_handle: Object detection action goal handle to process.
        :param robot: The robot instance corresponding to this request.
        :return: The object detection action result.
        """
        execution_result = robot.detect_objects(
            target_object=goal_handle.request.target_object
        )

        detected_objects_msg = [
            ObjectState(
                name=obj.name,
                category=obj.category,
                parent=obj.parent.name,
                pose=pose_to_ros(obj.pose),
            )
            for obj in robot.last_detected_objects
        ]

        goal_handle.succeed()
        return DetectObjects.Result(
            execution_result=execution_result_to_ros(execution_result),
            detected_objects=detected_objects_msg,
        )

    def package_robot_state(self, robot: Robot) -> RobotState:
        """
        Creates a ROS message containing a robot state.
        This state can be published standalone or packaged into the overall world state.

        :param robot: Robot instance from which to extract state.
        :return: ROS message representing the robot state.
        """
        state_msg = RobotState(name=robot.name)
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.pose = pose_to_ros(robot.get_pose())
        state_msg.battery_level = robot.battery_level
        state_msg.executing_action = robot.executing_action
        if robot.manipulated_object is not None:
            state_msg.holding_object = True
            state_msg.manipulated_object = robot.manipulated_object.name
        if robot.location is not None:
            if isinstance(robot.location, str):
                state_msg.last_visited_location = robot.location
            else:
                state_msg.last_visited_location = robot.location.name
        return state_msg

    def publish_robot_state(self, pub: Publisher, robot: Robot) -> None:  # type: ignore[type-arg] # Cannot add type args in Humble and Jazzy
        """
        Helper function to publish robot state.

        :param pub: The publisher on which to publish the robot state information.
        :param robot: Robot instance from which to extract state.
        """
        pub.publish(self.package_robot_state(robot))

    def world_info_callback(
        self, request: RequestWorldInfo.Request, response: RequestWorldInfo.Response
    ) -> RequestWorldState.Response:
        """
        Returns the world information as a response to a service request.

        :param request: The service request.
        :param response: The unmodified service response.
        :return: The modified service response containing the world information.
        """
        self.get_logger().info("Received world information request")

        response.info.name = self.world.name
        response.info.location_categories = (
            self.world.get_location_metadata().get_categories()
        )
        response.info.object_categories = (
            self.world.get_object_metadata().get_categories()
        )
        return response

    def world_state_callback(
        self, request: RequestWorldState.Request, response: RequestWorldState.Response
    ) -> RequestWorldState.Response:
        """
        Returns the world state as a response to a service request.

        :param request: The service request.
        :param response: The unmodified service response.
        :return: The modified service response containing the world state.
        """
        self.get_logger().info("Received world state request.")

        # Determine whether to use a robot's local observations or the full world state.
        if request.robot:
            robot = self.world.get_robot_by_name(request.robot)
            if not robot:
                return response  # empty response
            objects = robot.get_known_objects()
            known_closed_hallways = robot.get_known_closed_hallways()
        else:
            objects = self.world.objects
            known_closed_hallways = [
                hall for hall in self.world.hallways if not hall.is_open
            ]

        # Add location, hallway, and object states.
        for loc in self.world.locations:
            loc_msg = LocationState(
                name=loc.name,
                category=loc.category,
                spawns=[spawn.name for spawn in loc.children],
                parent=loc.get_room_name(),
                pose=pose_to_ros(loc.pose),
                is_open=loc.is_open,
                is_locked=loc.is_locked,
            )
            response.state.locations.append(loc_msg)
        for hall in self.world.hallways:
            hall_msg = HallwayState(
                name=hall.name,
                room_start=hall.room_start.name,
                room_end=hall.room_end.name,
                is_open=(hall not in known_closed_hallways),
                is_locked=hall.is_locked,
            )
            response.state.hallways.append(hall_msg)
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

    def set_location_state_callback(
        self, request: RequestWorldState.Request, response: RequestWorldState.Response
    ) -> RequestWorldState.Response:
        """
        Sets the state of a location in the world as a response to a service request.

        :param request: The service request.
        :param response: The unmodified service response.
        :return: The modified service response containing result of setting the location state.
        """
        self.get_logger().info("Received location state setting request.")

        # Check if the entity exists.
        entity = self.world.get_entity_by_name(request.location_name)
        if not entity:
            message = f"No location matching query: {request.location_name}"
            self.get_logger().warning(message)
            response.result.status = ExecutionResult.INVALID_ACTION
            response.result.message = message
            return response

        # Initialize the result in case no actions need to happen.
        response.result = ExecutionResult(
            status=ExecutionResult.SUCCESS, message="No action taken."
        )

        # Try open or close the location if its status needs to be toggled.
        result = None
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

        # If we made it here, return the latest result.
        if result is not None:
            response.result = execution_result_to_ros(result)
        return response

    def reset_world_callback(
        self, request: ResetWorld.Request, response: ResetWorld.Response
    ) -> ResetWorld.Response:
        """
        Reset the world as a response to a service request.

        :param request: The service request.
        :param response: The service response indicating success or failure.
        :return: The modified service response containing the result of the reset operation.
        """
        # Wait to cancel all the robot actions.
        cancel_threads = [
            Thread(target=robot.cancel_actions) for robot in self.world.robots
        ]
        for thread in cancel_threads:
            thread.start()
        for thread in cancel_threads:
            thread.join()

        response.success = self.world.reset(
            deterministic=request.deterministic, seed=request.seed
        )
        return response


def update_world_from_state_msg(world: World, msg: WorldState) -> None:
    """
    Updates a world given a state message.

    :param world: World object to update.
    :param msg: ROS message describing the desired world state.
    """
    # Update the robot states
    for robot_state in msg.robots:
        robot = world.get_robot_by_name(robot_state.name)
        if robot is not None:
            robot.set_pose(pose_from_ros(robot_state.pose))
        else:
            world.logger.error(
                f"Tried to update state for invalid robot {robot_state.name}"
            )

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

    # Update the hallway states
    for hallway_state in msg.hallways:
        hallway = world.get_hallway_by_name(hallway_state.name)
        hallway.is_open = hallway_state.is_open
        hallway.is_locked = hallway_state.is_locked
