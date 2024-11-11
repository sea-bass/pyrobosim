#!/usr/bin/env python3

"""
Unit tests for robot class.
"""
import os
import numpy as np
import pytest
import time
import threading

from pyrobosim.core import Pose, Robot, WorldYamlLoader
from pyrobosim.navigation import ConstantVelocityExecutor, WorldGraphPlanner
from pyrobosim.planning.actions import (
    ExecutionStatus,
    ExecutionOptions,
    TaskAction,
    TaskPlan,
)
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.motion import Path


class TestRobot:
    @pytest.fixture(autouse=True)
    def create_test_world(self):
        self.test_world = WorldYamlLoader().from_file(
            os.path.join(get_data_folder(), "test_world.yaml")
        )

    def create_test_plan(self):
        actions = [
            TaskAction(
                "navigate",
                source_location="kitchen",
                target_location="my_desk",
            ),
            TaskAction("detect", object="apple"),
            TaskAction("pick", object="apple"),
            TaskAction("place", object="apple"),
            TaskAction(
                "navigate",
                source_location="my_desk",
                target_location="hall_kitchen_bedroom",
            ),
            TaskAction("close", target_location="hall_kitchen_bedroom"),
            TaskAction("open", target_location="hall_kitchen_bedroom"),
        ]
        return TaskPlan(actions=actions)

    def test_create_robot_default_args(self, capsys):
        """Check that a robot can be created with all default arguments."""
        robot = Robot()

        assert robot.name == "robot"
        pose = robot.get_pose()
        assert pose.x == pytest.approx(0.0)
        assert pose.y == pytest.approx(0.0)
        assert pose.z == pytest.approx(0.0)
        assert pose.eul == pytest.approx([0.0, 0.0, 0.0])
        assert robot.radius == pytest.approx(0.0)
        assert robot.height == pytest.approx(0.0)
        assert robot.color == pytest.approx((0.8, 0.0, 0.8))
        assert robot.path_planner == None
        assert robot.path_executor == None
        assert robot.grasp_generator == None
        assert robot.world == None
        assert not robot.partial_observability
        assert robot.battery_level == 100.0

        robot.print_details()
        out, _ = capsys.readouterr()
        expected_details_str = (
            "Robot: robot\n"
            + "\tPose: [x=0.00, y=0.00, z=0.00, qw=1.000, qx=0.000, qy=-0.000, qz=0.000]\n"
            + "\tBattery: 100.00%\n"
        )
        assert out == expected_details_str

    def test_create_robot_nondefault_args(self, capsys):
        """Check that a robot can be created with nondefault arguments."""
        robot = Robot(
            name="test_robot",
            pose=Pose(x=1.0, y=2.0, yaw=np.pi / 2.0),
            radius=0.1,
            height=0.15,
            color=(0.5, 0.5, 0.5),
            partial_observability=True,
            initial_battery_level=50.0,
        )

        assert robot.name == "test_robot"
        pose = robot.get_pose()
        assert pose.x == pytest.approx(1.0)
        assert pose.y == pytest.approx(2.0)
        assert pose.z == pytest.approx(0.0)
        assert pose.eul == pytest.approx([0.0, 0.0, np.pi / 2.0])
        assert robot.radius == pytest.approx(0.1)
        assert robot.height == pytest.approx(0.15)
        assert robot.color == pytest.approx((0.5, 0.5, 0.5))
        assert robot.partial_observability
        assert robot.battery_level == 50.0

        robot.print_details()
        out, _ = capsys.readouterr()
        expected_details_str = (
            "Robot: test_robot\n"
            + "\tPose: [x=1.00, y=2.00, z=0.00, qw=0.707, qx=0.000, qy=-0.000, qz=0.707]\n"
            + "\tBattery: 50.00%\n"
            "\tPartial observability enabled\n"
        )
        assert out == expected_details_str

    def test_robot_bad_name(self):
        """Check that a robot with an invalid name raises an exception."""
        with pytest.raises(ValueError) as exc_info:
            Robot(name="world")
        assert exc_info.value.args[0] == "Robots cannot be named 'world'."

    def test_robot_path_planner(self, caplog):
        """Check that path planners can be used from a robot."""
        init_pose = Pose(x=1.0, y=0.5, yaw=0.0)
        goal_pose = Pose(x=2.5, y=3.0, yaw=np.pi / 2.0)

        path_planner = WorldGraphPlanner(world=self.test_world)
        robot = Robot(pose=init_pose, path_planner=path_planner)
        robot.world = self.test_world
        robot.location = self.test_world.get_entity_by_name("kitchen")

        # Explicitly provide start pose
        path = robot.plan_path(start=init_pose, goal=goal_pose)
        assert isinstance(path, Path)
        assert path.poses[0] == init_pose
        assert path.poses[-1] == goal_pose

        # Assumes start pose is already the robot pose
        path = robot.plan_path(goal=goal_pose)
        assert isinstance(path, Path)
        assert path.poses[0] == init_pose
        assert path.poses[-1] == goal_pose

        # Provide no goal pose
        path = robot.plan_path()
        assert path is None
        assert "Did not specify a goal. Returning None." in caplog.text
        caplog.clear()

        # Provide no path planner
        robot.path_planner = None
        path = robot.plan_path()
        assert path is None
        assert "No path planner attached to robot." in caplog.text
        caplog.clear()

        # Try and reset the path planner with no planner set
        robot.reset_path_planner()
        assert "Robot has no path planner to reset." in caplog.text

        # Re-add the path planner and reset it. There should be no warnings.
        robot.path_planner = path_planner
        robot.reset_path_planner()

    def test_robot_path_executor(self):
        """Check that path executors can be used from a robot."""
        init_pose = Pose(x=1.0, y=0.5, yaw=0.0)
        goal_pose = Pose(x=2.5, y=3.0, yaw=np.pi / 2.0)

        robot = Robot(
            pose=init_pose,
            path_planner=WorldGraphPlanner(world=self.test_world),
            path_executor=ConstantVelocityExecutor(linear_velocity=5.0, dt=0.1),
        )
        robot.world = self.test_world
        robot.location = "kitchen"

        # Non-threaded option -- blocks
        robot.set_pose(init_pose)
        path = robot.plan_path(goal=goal_pose)
        result = robot.follow_path(path)

        assert result.status == ExecutionStatus.SUCCESS
        assert not robot.executing_nav
        assert robot.last_nav_result.is_success()
        pose = robot.get_pose()
        assert pose.x == pytest.approx(goal_pose.x)
        assert pose.y == pytest.approx(goal_pose.y)
        assert pose.q == pytest.approx(goal_pose.q)

        # Provide no path and no path executor.
        result = robot.follow_path(None)
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "No path to execute."

        robot.path_executor = None
        result = robot.follow_path(path)
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "No path executor. Cannot follow path."

    def test_robot_nav_validation(self, caplog):
        """Check that the robot can abort execution with runtime collision validation."""
        init_pose = Pose(x=1.0, y=0.75, yaw=0.0)
        goal_pose = Pose(x=2.5, y=3.0, yaw=np.pi / 2.0)

        robot = Robot(
            pose=init_pose,
            path_planner=WorldGraphPlanner(world=self.test_world),
            path_executor=ConstantVelocityExecutor(
                linear_velocity=1.0,  # Move slowly to give time to cancel.
                dt=0.1,
                validate_during_execution=True,
            ),
        )
        robot.world = self.test_world

        # Plan a path.
        robot.set_pose(init_pose)
        path = robot.plan_path(goal=goal_pose)

        # Schedule a thread that closes all hallways after a slight delay.
        def close_all_hallways():
            time.sleep(0.5)
            for hallway in self.test_world.hallways:
                self.test_world.close_location(hallway, ignore_robots=[robot])

        threading.Thread(target=close_all_hallways).start()

        # Follow the path and check that it fails.
        result = robot.follow_path(path)
        assert "Remaining path is in collision. Aborting execution." in caplog.text
        assert "Trajectory execution aborted." in caplog.text
        assert result.status == ExecutionStatus.EXECUTION_FAILURE

        # Now open the hallways and try again, which should succeed.
        for hallway in self.test_world.hallways:
            self.test_world.open_location(hallway)
        path = robot.plan_path(goal=goal_pose)
        result = robot.follow_path(path)
        assert result.status == ExecutionStatus.SUCCESS

    def test_robot_manipulation(self):
        """Check that the robot can manipulate objects."""
        # Spawn the robot near the kitchen table
        robot = Robot(
            pose=Pose(x=1.0, y=0.5, yaw=0.0),
            action_execution_options={
                "pick": ExecutionOptions(battery_usage=5.0),
                "place": ExecutionOptions(battery_usage=10.0),
            },
        )
        robot.world = self.test_world
        robot.location = self.test_world.get_entity_by_name("table0_tabletop")

        # Try pick with a object query that cannot not be found.
        result = robot.pick_object("nonexistent")
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "Found no object nonexistent to pick."

        # Try pick with a object query that can be found, but is not in the robot's location.
        result = robot.pick_object("apple0")
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert (
            result.message
            == "apple0 is at my_desk_desktop and robot is at table0_tabletop. Cannot pick."
        )

        # Pick up the apple on the kitchen table (named "gala").
        # Trying on empty battery should fail.
        robot.battery_level = 0.0
        result = robot.pick_object("apple")
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "Out of battery. Cannot pick."

        robot.battery_level = 100.0
        result = robot.pick_object("apple")
        assert result.status == ExecutionStatus.SUCCESS
        assert robot.manipulated_object == self.test_world.get_entity_by_name("gala")
        assert robot.battery_level == pytest.approx(95.0)

        # Try pick up another object, which should fail since the robot is holding something.
        result = robot.pick_object("banana")
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "Robot is already holding gala."

        # Try place the object at a location that does not permit placing.
        robot.location = "bedroom"
        result = robot.place_object()
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert (
            result.message
            == "Room: bedroom is not an object spawn. Cannot place object."
        )

        # Try place the object at a valid location.
        # Trying on empty battery should fail.
        robot.location = self.test_world.get_entity_by_name("table0_tabletop")
        robot.battery_level = 0.0
        result = robot.place_object()
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "Out of battery. Cannot place."

        robot.battery_level = 100.0
        result = robot.place_object()
        assert result.status == ExecutionStatus.SUCCESS
        assert robot.manipulated_object is None
        assert robot.battery_level == pytest.approx(90.0)

        # Try place an object, which should fail since the robot is holding nothing.
        result = robot.place_object()
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "No manipulated object. Cannot place."

        # Pick an object again, and try place it with an explicit pose.
        robot.pick_object("apple")
        result = robot.place_object(pose=Pose(x=100.0, y=100.0))
        assert result.status == ExecutionStatus.PLANNING_FAILURE
        assert result.message == "Pose in collision or not in location table0_tabletop."

        # Unlock the table to allow opening and closing for this test.
        self.test_world.unlock_location(robot.location.parent)

        # Closing the location should make placing fail.
        self.test_world.close_location(robot.location.parent)
        result = robot.place_object()
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "table0 is not open. Cannot place object."

        # With a closed location, picking should fail too.
        self.test_world.open_location(robot.location.parent)
        robot.place_object()
        self.test_world.close_location(robot.location.parent)
        result = robot.pick_object("apple")
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "table0 is not open. Cannot pick object."

    def test_robot_object_detection(self):
        """Check that the robot can detect objects."""
        # Spawn the robot near the kitchen table
        robot = Robot(
            pose=Pose(x=1.0, y=0.5, yaw=0.0),
            action_execution_options={"detect": ExecutionOptions(battery_usage=1.0)},
        )
        robot.world = self.test_world

        # Detecting objects this way will fail because the robot is not at an object spawn.
        result = robot.detect_objects()
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert (
            result.message == "Robot is not at an object spawn. Cannot detect objects."
        )

        # Moving the robot to a valid location should work.
        robot.location = self.test_world.get_entity_by_name("table0_tabletop")
        result = robot.detect_objects()
        assert result.status == ExecutionStatus.SUCCESS
        assert len(robot.last_detected_objects) == 2
        assert robot.battery_level == pytest.approx(99.0)

        # Trying on empty battery should fail.
        robot.battery_level = 0.0
        result = robot.detect_objects()
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "Out of battery. Cannot detect objects."
        robot.battery_level = 100.0

        # Filtering by category should also affect results.
        result = robot.detect_objects("apple")
        assert result.status == ExecutionStatus.SUCCESS
        len(robot.last_detected_objects) == 1
        assert robot.battery_level == pytest.approx(99.0)

        result = robot.detect_objects("water")
        assert result.status == ExecutionStatus.EXECUTION_FAILURE
        assert (
            result.message == "Failed to detect any objects matching the query 'water'."
        )
        assert len(robot.last_detected_objects) == 0

        # Unlock the table to allow opening and closing for this test.
        self.test_world.unlock_location(robot.location.parent)

        # Closing the location should make detection fail.
        self.test_world.close_location(robot.location.parent)
        result = robot.detect_objects()
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "table0 is not open. Cannot detect objects."

    def test_robot_open_close_hallway(self):
        """Check that the robot can open or close hallways."""
        robot = Robot(
            pose=Pose(x=1.0, y=0.5, yaw=0.0),
            action_execution_options={
                "open": ExecutionOptions(battery_usage=20.0),
                "close": ExecutionOptions(battery_usage=25.0),
            },
        )
        robot.world = self.test_world

        # Try to open and close with an unset location.
        result = robot.open_location()
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "Robot location is not set. Cannot open."

        result = robot.close_location()
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "Robot location is not set. Cannot close."

        # Now set the location to a non-openable location, and get a different warning.
        robot.location = self.test_world.get_room_by_name("kitchen")
        result = robot.open_location()
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "Robot is not at an openable location."

        result = robot.close_location()
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "Robot is not at a closeable location."

        # Set the location to an openable location.
        hallway = self.test_world.get_hallways_from_rooms("kitchen", "bedroom")[0]
        robot.location = hallway

        # Trying to close on empty battery should fail.
        robot.battery_level = 0.0
        result = robot.close_location()
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "Out of battery. Cannot close location."

        # Trying to close with battery should succeed.
        robot.battery_level = 100.0
        assert robot.close_location().is_success()
        assert robot.battery_level == pytest.approx(75.0)

        # Trying to open empty battery should fail.
        robot.battery_level = 0.0
        result = robot.open_location()
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "Out of battery. Cannot open location."

        # Trying to open with battery should succeed.
        robot.battery_level = 100.0
        assert robot.open_location().is_success()
        assert robot.battery_level == pytest.approx(80.0)

        # Set a manipulated object, which will stop open/close from succeeding.
        robot.manipulated_object = self.test_world.get_object_by_name("banana0")
        result = robot.open_location()
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "Robot is holding an object. Cannot open."

        result = robot.close_location()
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "Robot is holding an object. Cannot close."

        # Add a new robot to the world inside the hallway.
        # Now, closing should fail because of this obstructing robot.
        robot.manipulated_object = None
        new_robot = Robot(name="blocker")
        self.test_world.add_robot(new_robot, pose=Pose(x=2.5, y=0.5, yaw=0.0))
        result = robot.close_location()
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert (
            result.message
            == "Robot blocker is in Hallway: hall_bedroom_kitchen. Cannot close."
        )

    def test_execute_action(self):
        """Tests execution of a single action."""
        init_pose = Pose(x=1.0, y=0.5, yaw=0.0)
        robot = Robot(
            name="test_robot",
            pose=init_pose,
            path_planner=WorldGraphPlanner(world=self.test_world),
            path_executor=ConstantVelocityExecutor(linear_velocity=5.0, dt=0.1),
            action_execution_options={"navigate": ExecutionOptions(battery_usage=1.0)},
        )
        robot.location = "kitchen"
        robot.world = self.test_world

        # Navigate to a location and check that the battery level decreased.
        action = TaskAction(
            "navigate",
            source_location="kitchen",
            target_location="my_desk",
        )
        result = robot.execute_action(action)
        assert result.is_success()
        assert robot.battery_level < 100.0

        # Modify the action execution options to deplete the battery extremely quickly.
        robot.action_execution_options["navigate"].battery_usage = 100.0
        action = TaskAction(
            "navigate",
            source_location="my_desk",
            target_location="bathroom",
        )
        result = robot.execute_action(action)
        assert result.status == ExecutionStatus.EXECUTION_FAILURE
        assert result.message == "Battery depleted while navigating."
        assert robot.battery_level == 0.0

    def test_execute_invalid_action(self):
        """Tests execution of an action that is not recognized as a valid type."""
        init_pose = Pose(x=1.0, y=0.5, yaw=0.0)
        robot = Robot(name="test_robot", pose=init_pose)
        robot.world = self.test_world
        action = TaskAction("bad_action")

        result = robot.execute_action(action)
        assert result.status == ExecutionStatus.INVALID_ACTION
        assert result.message == "Invalid action type: bad_action."

    def test_execute_action_simulated_options(self):
        """Test execution of an action that has simulated options."""
        init_pose = Pose(x=1.0, y=0.5, yaw=0.0)
        action_execution_options = {
            "navigate": ExecutionOptions(
                delay=0.1,
                success_probability=0.5,
                rng_seed=1234,
                battery_usage=1.0,
            ),
        }
        robot = Robot(
            pose=init_pose,
            path_planner=WorldGraphPlanner(world=self.test_world),
            path_executor=ConstantVelocityExecutor(linear_velocity=5.0, dt=0.1),
            action_execution_options=action_execution_options,
            initial_battery_level=80.0,
        )
        robot.location = "kitchen"
        robot.world = self.test_world
        my_desk = self.test_world.get_location_by_name("my_desk")
        my_desk.is_charger = True  # Make into a charger to test charging capabilities.
        action = TaskAction(
            "navigate",
            source_location="kitchen",
            target_location="my_desk",
        )

        # The action should fail the first time but succeed the second time.
        result = robot.execute_action(action)
        assert result.status == ExecutionStatus.EXECUTION_FAILURE
        assert result.message == "Simulated navigation failure."
        assert robot.battery_level == pytest.approx(80.0)

        assert robot.execute_action(action).is_success()
        assert robot.battery_level == pytest.approx(100.0)  # Charged!

        # Navigating on empty battery should fail.
        robot.battery_level = 0.0
        result = robot.execute_action(action)
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "Out of battery. Cannot navigate."

    def test_execute_action_cancel(self, caplog):
        """Tests that actions can be canceled during execution."""
        init_pose = Pose(x=1.0, y=0.5, yaw=0.0)
        robot = Robot(
            pose=init_pose,
            path_planner=WorldGraphPlanner(world=self.test_world),
            path_executor=ConstantVelocityExecutor(linear_velocity=3.0, dt=0.1),
        )
        robot.location = "kitchen"
        robot.world = self.test_world
        action = TaskAction(
            "navigate",
            source_location="kitchen",
            target_location="my_desk",
        )

        # Start a thread that cancels the actions mid execution.
        threading.Timer(1.0, robot.cancel_actions).start()

        # Run action and check that it failed due to being canceled.
        result = robot.execute_action(action)
        assert result.status == ExecutionStatus.CANCELED

        # Retry the action, which should now succeed.
        assert robot.execute_action(action).is_success()

        # Try to cancel actions again, which should warn.
        robot.cancel_actions()
        assert "There is no running action or plan to cancel." in caplog.text

    def test_execute_plan(self):
        """Tests execution of a plan consisting of multiple actions."""
        init_pose = Pose(x=1.0, y=0.5, yaw=0.0)
        robot = Robot(
            pose=init_pose,
            path_planner=WorldGraphPlanner(world=self.test_world),
            path_executor=ConstantVelocityExecutor(linear_velocity=5.0, dt=0.1),
        )
        robot.location = "kitchen"
        robot.world = self.test_world

        result, num_completed = robot.execute_plan(self.create_test_plan())

        assert result.is_success()
        assert num_completed == 7

    def test_execute_blank_plan(self):
        """Tests a trivial case of executing a plan that is None."""
        init_pose = Pose(x=1.0, y=0.5, yaw=0.0)
        robot = Robot(name="test_robot", pose=init_pose)
        robot.world = self.test_world

        result, num_completed = robot.execute_plan(None)
        assert result.status == ExecutionStatus.INVALID_ACTION
        assert num_completed == 0
        assert result.message == "Plan is None. Returning."

    def test_execute_plan_cancel(self):
        """Tests that task plans can be canceled during execution."""
        init_pose = Pose(x=1.0, y=0.5, yaw=0.0)
        robot = Robot(
            pose=init_pose,
            path_planner=WorldGraphPlanner(world=self.test_world),
            path_executor=ConstantVelocityExecutor(linear_velocity=5.0, dt=0.1),
        )
        robot.location = "kitchen"
        robot.world = self.test_world

        # Start a thread that cancels the plan mid execution.
        threading.Timer(3.0, robot.cancel_actions).start()

        # Run plan and check that it failed due to being canceled.
        plan = self.create_test_plan()
        result, num_completed = robot.execute_plan(plan)
        assert result.status == ExecutionStatus.CANCELED
        assert result.message == "Canceled plan execution."
        assert num_completed < plan.size()

        # Resume the plan from where it left off, which should now succeed.
        remaining_plan = TaskPlan(actions=plan.actions[num_completed:])
        result, remaining_num_completed = robot.execute_plan(remaining_plan)
        assert result.is_success()
        assert num_completed + remaining_num_completed == plan.size()

    def test_at_object_spawn(self):
        """Tests check for robot being at an object spawn."""
        robot = Robot()
        robot.world = self.test_world
        assert not robot.at_object_spawn()

        robot.location = self.test_world.get_entity_by_name("kitchen")
        assert not robot.at_object_spawn()

        robot.location = self.test_world.get_entity_by_name("table0_tabletop")
        assert robot.at_object_spawn()

    def test_partial_observability(self):
        """Tests partial observability capabilities."""
        robot = Robot(partial_observability=True)

        # If no world is assigned, there should be no known objects
        assert len(robot.get_known_objects()) == 0

        # Even after assigning a world, nothing is yet detected
        robot.world = self.test_world
        assert len(robot.get_known_objects()) == 0

        # If the robot is at a location, and we run the detect action,
        # there should now be detected objects.
        robot.location = self.test_world.get_entity_by_name("table0_tabletop")
        robot.detect_objects()
        assert len(robot.get_known_objects()) == 2
