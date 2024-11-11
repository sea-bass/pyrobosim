# Integration tests for the PyRoboSim ROS interface.

import os
import pytest
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.action.client import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from pyrobosim.core import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder

from pyrobosim_msgs.action import (
    DetectObjects,
    ExecuteTaskAction,
    ExecuteTaskPlan,
    FollowPath,
    PlanPath,
)
from pyrobosim_msgs.msg import ExecutionResult, RobotState, TaskAction, TaskPlan
from pyrobosim_msgs.srv import RequestWorldState, SetLocationState
from pyrobosim_ros.ros_interface import WorldROSWrapper


def execute_ros_action(
    goal_future, spin_timeout=0.1, goal_timeout=1.0, result_timeout=30.0
):
    """Helper function to execute a ROS action and wait for a result."""
    start_time = time.time()
    while not goal_future.done():
        TestRosInterface.executor.spin_once(timeout_sec=spin_timeout)
        if time.time() - start_time > goal_timeout:
            print("Timed out waiting for goal to be accepted.")
            break
    assert goal_future.done()

    result_future = goal_future.result().get_result_async()
    while not result_future.done():
        TestRosInterface.executor.spin_once(timeout_sec=spin_timeout)
        if time.time() - start_time > result_timeout:
            print("Timed out waiting for result to be received.")
            break

    return result_future


class TestRosInterface:
    @staticmethod
    @pytest.mark.dependency(name="test_start_ros_interface")
    def test_start_ros_interface():
        rclpy.init()

        # Load world from file.
        world_file_path = os.path.join(get_data_folder(), "test_world_multirobot.yaml")
        world = WorldYamlLoader().from_file(world_file_path)

        # Create ROS interface.
        TestRosInterface.ros_interface = WorldROSWrapper(
            state_pub_rate=0.1, dynamics_rate=0.01
        )
        TestRosInterface.ros_interface.set_world(world)
        TestRosInterface.ros_interface.start(auto_spin=False)

        TestRosInterface.node = Node("test_ros_interface")

        TestRosInterface.executor = MultiThreadedExecutor()
        TestRosInterface.executor.add_node(TestRosInterface.ros_interface)
        TestRosInterface.executor.add_node(TestRosInterface.node)
        time.sleep(1.0)

    @staticmethod
    @pytest.mark.dependency(
        name="test_robot_pub_sub", depends=["test_start_ros_interface"]
    )
    def test_robot_pub_sub():
        """Test that we can publish and subscribe to the robots."""

        data = {"latest_state": None}

        def state_cb(msg):
            data["latest_state"] = msg

        state_sub = TestRosInterface.node.create_subscription(
            RobotState, "robot0/robot_state", state_cb, 10
        )
        vel_pub = TestRosInterface.node.create_publisher(Twist, "robot0/cmd_vel", 10)

        start_time = time.time()
        while not data["latest_state"]:
            TestRosInterface.executor.spin_once(timeout_sec=0.1)
            if time.time() - start_time > 2.0:
                break

        assert data["latest_state"] is not None
        init_state = data["latest_state"]

        vel_msg = Twist()
        vel_msg.linear.x = 0.1
        vel_msg.angular.z = 0.5
        for _ in range(25):
            vel_pub.publish(vel_msg)
            TestRosInterface.executor.spin_once(timeout_sec=0.1)
            time.sleep(0.01)  # Give the publisher a break

        assert data["latest_state"] != init_state

        TestRosInterface.node.destroy_subscription(state_sub)
        TestRosInterface.node.destroy_publisher(vel_pub)

    @staticmethod
    @pytest.mark.dependency(name="test_get_world_state", depends=["test_robot_pub_sub"])
    def test_get_world_state():
        """Test that we can retrieve the world state via service call."""

        client = TestRosInterface.node.create_client(
            RequestWorldState, "request_world_state"
        )

        # Full world state.
        future = client.call_async(RequestWorldState.Request())
        start_time = time.time()
        while not future.done():
            TestRosInterface.executor.spin_once(timeout_sec=0.1)
            if time.time() - start_time > 2.0:
                break

        assert future.done()
        result = future.result()
        assert len(result.state.robots) == 3
        assert len(result.state.locations) == 5
        assert len(result.state.hallways) == 3
        assert len(result.state.objects) == 8

        # Partial robot state.
        # Since the world is not configured for partial observability, we make manual modifications to robot1.
        world = TestRosInterface.ros_interface.world
        robot1 = world.get_robot_by_name("robot1")
        robot1.partial_observability = True
        robot1.known_objects = set(
            [world.get_object_by_name("apple0"), world.get_object_by_name("banana0")]
        )

        future = client.call_async(RequestWorldState.Request(robot="robot1"))
        start_time = time.time()
        while not future.done():
            TestRosInterface.executor.spin_once(timeout_sec=0.1)
            if time.time() - start_time > 2.0:
                break

        assert future.done()
        result = future.result()
        assert len(result.state.robots) == 3
        assert len(result.state.locations) == 5
        assert len(result.state.hallways) == 3
        assert len(result.state.objects) == 2

        TestRosInterface.node.destroy_client(client)

    @staticmethod
    @pytest.mark.dependency(
        name="test_set_location_state", depends=["test_get_world_state"]
    )
    def test_set_location_state():
        """Test that we can set location states."""
        client = TestRosInterface.node.create_client(
            SetLocationState, "set_location_state"
        )

        # Try to close the table, which should fail as it is locked.
        future = client.call_async(
            SetLocationState.Request(
                location_name="table0",
                open=False,
                lock=False,
            )
        )
        start_time = time.time()
        while not future.done():
            TestRosInterface.executor.spin_once(timeout_sec=0.1)
            if time.time() - start_time > 2.0:
                break

        assert future.done()
        assert future.result().result.status == ExecutionResult.PRECONDITION_FAILURE
        assert future.result().result.message == "Location: table0 is locked."

        # Try to only unlock the table, which should succeed.
        future = client.call_async(
            SetLocationState.Request(
                location_name="table0",
                open=True,
                lock=False,
            )
        )
        start_time = time.time()
        while not future.done():
            TestRosInterface.executor.spin_once(timeout_sec=0.1)
            if time.time() - start_time > 2.0:
                break

        assert future.done()
        assert future.result().result.status == ExecutionResult.SUCCESS

        TestRosInterface.node.destroy_client(client)

    @staticmethod
    @pytest.mark.dependency(
        name="test_execute_action", depends=["test_set_location_state"]
    )
    def test_execute_action():
        """Test that we can execute a single action on a robot."""

        action_client = ActionClient(
            TestRosInterface.node, ExecuteTaskAction, "execute_action"
        )

        # This is an invalid action.
        goal = ExecuteTaskAction.Goal()
        goal.action = TaskAction(robot="robot0", type="nonexistent_action")
        goal_future = action_client.send_goal_async(goal)
        result_future = execute_ros_action(goal_future)

        assert result_future.done()
        exec_result = result_future.result().result.execution_result
        assert exec_result.status == ExecutionResult.INVALID_ACTION
        assert exec_result.message == "Invalid action type: nonexistent_action."

        # This action has bad parameters.
        goal = ExecuteTaskAction.Goal()
        goal.action = TaskAction(
            robot="robot0",
            type="navigate",
            target_location="counter42",
        )
        goal_future = action_client.send_goal_async(goal)
        result_future = execute_ros_action(goal_future)

        assert result_future.done()
        exec_result = result_future.result().result.execution_result
        assert exec_result.status == ExecutionResult.PLANNING_FAILURE
        assert exec_result.message == "Failed to plan a path."

        # This action should succeed.
        goal = ExecuteTaskAction.Goal()
        goal.action = TaskAction(
            robot="robot0",
            type="navigate",
            target_location="my_desk",
        )
        goal_future = action_client.send_goal_async(goal)
        result_future = execute_ros_action(goal_future)

        assert result_future.done()
        assert (
            result_future.result().result.execution_result.status
            == ExecutionResult.SUCCESS
        )

        world = TestRosInterface.ros_interface.world
        robot = world.get_robot_by_name("robot0")
        assert robot.location == world.get_entity_by_name("my_desk_desktop")

    @staticmethod
    @pytest.mark.dependency(
        name="test_execute_task_plan", depends=["test_execute_action"]
    )
    def test_execute_task_plan():
        """Test that we can execute a task plan on a robot."""

        action_client = ActionClient(
            TestRosInterface.node, ExecuteTaskPlan, "execute_task_plan"
        )

        task_actions = [
            TaskAction(type="pick", object="water"),
            TaskAction(type="navigate", target_location="counter"),
            TaskAction(type="place"),
        ]
        goal = ExecuteTaskPlan.Goal(plan=TaskPlan(robot="robot0", actions=task_actions))
        goal_future = action_client.send_goal_async(goal)
        result_future = execute_ros_action(goal_future)

        assert result_future.done()
        assert (
            result_future.result().result.execution_result.status
            == ExecutionResult.SUCCESS
        )
        assert result_future.result().result.num_completed == 3
        assert result_future.result().result.num_total == 3

        world = TestRosInterface.ros_interface.world
        robot = world.get_robot_by_name("robot0")
        assert robot.location.parent == world.get_entity_by_name("counter0")
        assert world.get_entity_by_name("water1").parent == robot.location

    @staticmethod
    @pytest.mark.dependency(
        name="test_specialized_actions", depends=["test_execute_task_plan"]
    )
    def test_specialized_actions():
        """
        Test that we can run the specialized path planning, path following,
        and object detection actions.
        """

        path_plan_action_client = ActionClient(
            TestRosInterface.node, PlanPath, "robot0/plan_path"
        )
        path_follow_action_client = ActionClient(
            TestRosInterface.node, FollowPath, "robot0/follow_path"
        )
        detect_objects_action_client = ActionClient(
            TestRosInterface.node, DetectObjects, "robot0/detect_objects"
        )

        # Verify that path planning succeeds.
        goal = PlanPath.Goal(target_location="my_desk")
        goal_future = path_plan_action_client.send_goal_async(goal)
        result_future = execute_ros_action(goal_future)

        assert result_future.done()
        action_result = result_future.result().result
        assert action_result.execution_result.status == ExecutionResult.SUCCESS
        assert len(action_result.path.poses) >= 2

        # Verify that path following succeeds.
        goal = FollowPath.Goal(path=action_result.path)
        goal_future = path_follow_action_client.send_goal_async(goal)
        result_future = execute_ros_action(goal_future)

        assert result_future.done()
        action_result = result_future.result().result
        assert action_result.execution_result.status == ExecutionResult.SUCCESS

        # Check that the robot actually reached its destination.
        world = TestRosInterface.ros_interface.world
        robot = world.get_robot_by_name("robot0")
        assert robot.location.parent == world.get_entity_by_name("my_desk")

        # Verify that object detection works without filtering.
        goal = DetectObjects.Goal()
        goal_future = detect_objects_action_client.send_goal_async(goal)
        result_future = execute_ros_action(goal_future)

        assert result_future.done()
        action_result = result_future.result().result
        assert action_result.execution_result.status == ExecutionResult.SUCCESS
        assert len(action_result.detected_objects) == 2

        # Verify that object detection works with filtering.
        goal = DetectObjects.Goal(target_object="water")
        goal_future = detect_objects_action_client.send_goal_async(goal)
        result_future = execute_ros_action(goal_future)

        assert result_future.done()
        action_result = result_future.result().result
        assert (
            action_result.execution_result.status == ExecutionResult.EXECUTION_FAILURE
        )
        assert (
            action_result.execution_result.message
            == "Failed to detect any objects matching the query 'water'."
        )
        assert len(action_result.detected_objects) == 0

    @pytest.mark.dependency(
        name="test_shutdown_ros_interface", depends=["test_specialized_actions"]
    )
    def test_shutdown_ros_interface(self):
        """Shuts down the interface node and rclpy at the end of all other tests."""
        TestRosInterface.ros_interface.shutdown()
