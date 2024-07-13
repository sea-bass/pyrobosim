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
from pyrobosim.navigation import ConstantVelocityExecutor, PathPlanner
from pyrobosim.planning.actions import ExecutionOptions, TaskAction, TaskPlan
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.motion import Path


class TestRobot:
    @pytest.fixture(autouse=True)
    def create_test_world(self):
        self.test_world = WorldYamlLoader().from_yaml(
            os.path.join(get_data_folder(), "test_world.yaml")
        )

    def test_create_robot_default_args(self):
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

    def test_create_robot_nondefault_args(self):
        """Check that a robot can be created with nondefault arguments."""
        robot = Robot(
            name="test_robot",
            pose=Pose(x=1.0, y=2.0, yaw=np.pi / 2.0),
            radius=0.1,
            height=0.15,
            color=(0.5, 0.5, 0.5),
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

    def test_robot_bad_name(self):
        """Check that a robot with an invalid name raises an exception."""
        with pytest.raises(ValueError) as exc_info:
            Robot(name="world")
        assert exc_info.value.args[0] == "Robots cannot be named 'world'."

    def test_robot_path_planner(self):
        """Check that path planners can be used from a robot."""
        init_pose = Pose(x=1.0, y=0.5, yaw=0.0)
        goal_pose = Pose(x=2.5, y=3.0, yaw=np.pi / 2.0)

        robot = Robot(
            pose=init_pose,
            path_planner=PathPlanner("world_graph", world=self.test_world),
        )
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
        with pytest.warns(UserWarning) as warn_info:
            path = robot.plan_path()
        assert path is None
        assert warn_info[0].message.args[0] == "Did not specify a goal. Returning None."

    def test_robot_path_executor(self):
        """Check that path executors can be used from a robot."""
        init_pose = Pose(x=1.0, y=0.5, yaw=0.0)
        goal_pose = Pose(x=2.5, y=3.0, yaw=np.pi / 2.0)

        robot = Robot(
            pose=init_pose,
            path_planner=PathPlanner("world_graph", world=self.test_world),
            path_executor=ConstantVelocityExecutor(linear_velocity=5.0, dt=0.1),
        )
        robot.world = self.test_world
        robot.location = "kitchen"

        # Non-threaded option -- blocks
        robot.set_pose(init_pose)
        path = robot.plan_path(goal=goal_pose)
        result = robot.follow_path(path, use_thread=False)
        assert result

        # Threaded option with blocking
        robot.set_pose(init_pose)
        path = robot.plan_path(goal=goal_pose)
        result = robot.follow_path(path, use_thread=True, blocking=True)

        # Threaded option without blocking -- must check result
        robot.set_pose(init_pose)
        path = robot.plan_path(goal=goal_pose)
        robot.follow_path(path, use_thread=True, blocking=False)
        assert robot.executing_nav
        while robot.executing_nav:
            time.sleep(0.1)
        assert not robot.executing_nav
        assert robot.last_nav_successful
        pose = robot.get_pose()
        assert pose.x == pytest.approx(goal_pose.x)
        assert pose.y == pytest.approx(goal_pose.y)
        assert pose.q == pytest.approx(goal_pose.q)

    def test_robot_nav_validation(self):
        """Check that the robot can abort execution with runtime collision validation."""
        init_pose = Pose(x=1.0, y=0.75, yaw=0.0)
        goal_pose = Pose(x=2.5, y=3.0, yaw=np.pi / 2.0)

        robot = Robot(
            pose=init_pose,
            path_planner=PathPlanner("world_graph", world=self.test_world),
            path_executor=ConstantVelocityExecutor(
                linear_velocity=3.0,
                dt=0.1,
                validate_during_execution=True,
            ),
        )
        robot.world = self.test_world
        robot.location = "kitchen"

        # Plan a path.
        robot.set_pose(init_pose)
        path = robot.plan_path(goal=goal_pose)

        # Schedule a thread that closes all hallways after a slight delay.
        def close_all_hallways():
            time.sleep(0.5)
            for hallway in self.test_world.hallways:
                self.test_world.close_hallway(hallway)

        threading.Thread(target=close_all_hallways).start()

        # Follow the path and check that it fails.
        with pytest.warns(UserWarning) as warn_info:
            result = robot.follow_path(path, use_thread=False)
        assert (
            warn_info[0].message.args[0]
            == "Remaining path is in collision. Aborting execution."
        )
        assert warn_info[1].message.args[0] == "Trajectory execution aborted."
        assert not result

        # Now open the hallways and try again, which should succeed.
        for hallway in self.test_world.hallways:
            self.test_world.open_hallway(hallway)
        path = robot.plan_path(goal=goal_pose)
        assert robot.follow_path(path, use_thread=False)

    def test_robot_manipulation(self):
        """Check that the robot can manipulate objects."""
        # Spawn the robot near the kitchen table
        robot = Robot(
            pose=Pose(x=1.0, y=0.5, yaw=0.0),
        )
        robot.world = self.test_world
        robot.location = self.test_world.get_entity_by_name("table0_tabletop")

        # Try pick with a object query that cannot not be found.
        with pytest.warns(UserWarning) as warn_info:
            result = robot.pick_object("nonexistent")
        assert not result
        assert "Could not resolve object query" in warn_info[0].message.args[0]

        # Try pick with a object query that can be found, but is not in the robot's location.
        with pytest.warns(UserWarning) as warn_info:
            result = robot.pick_object("apple0")
        assert not result
        assert (
            warn_info[0].message.args[0]
            == "apple0 is at my_desk_desktop and robot is at table0_tabletop. Cannot pick."
        )

        # Pick up the apple on the kitchen table (named "gala")
        result = robot.pick_object("apple")
        assert result
        assert robot.manipulated_object == self.test_world.get_entity_by_name("gala")

        # Try pick up another object, which should fail since the robot is holding something.
        with pytest.warns(UserWarning):
            result = robot.pick_object("banana")
            assert not result

        # Try place the object at a location that does not permit placing.
        robot.location = "bedroom"
        with pytest.warns(UserWarning) as warn_info:
            result = robot.place_object()
        assert not result
        assert (
            warn_info[0].message.args[0]
            == "Room: bedroom is not an object spawn. Cannot place object."
        )

        # Try place the object at a valid location.
        robot.location = "table0_tabletop"
        result = robot.place_object()
        assert result
        assert robot.manipulated_object is None

        # Try place an object, which should fail since the robot is holding nothing.
        with pytest.warns(UserWarning) as warn_info:
            result = robot.place_object()
        assert not result
        assert warn_info[0].message.args[0] == "No manipulated object. Cannot place."

        # Pick an object again, and try place it with an explicit pose.
        robot.pick_object("apple")
        with pytest.warns(UserWarning) as warn_info:
            result = robot.place_object(pose=Pose(x=100.0, y=100.0))
        assert not result
        assert (
            warn_info[0].message.args[0]
            == "Pose in collision or not in location table0_tabletop."
        )

    def test_robot_object_detection(self):
        """Check that the robot can detect objects."""
        # Spawn the robot near the kitchen table
        robot = Robot(
            pose=Pose(x=1.0, y=0.5, yaw=0.0),
        )
        robot.world = self.test_world

        # Detecting objects this way will fail because the robot is not at an object spawn.
        with pytest.warns(UserWarning) as warn_info:
            assert not robot.detect_objects()
        assert (
            warn_info[0].message.args[0]
            == "Robot is not at an object spawn. Cannot detect objects."
        )

        # Moving the robot to a valid location should work.
        robot.location = self.test_world.get_entity_by_name("table0_tabletop")
        assert robot.detect_objects()
        assert len(robot.last_detected_objects) == 2

        # Filtering by category should also affect results.
        assert robot.detect_objects("apple")
        assert len(robot.last_detected_objects) == 1
        assert not robot.detect_objects("water")
        assert len(robot.last_detected_objects) == 0

    def test_robot_open_close_hallway(self):
        """Check that the robot can open or close hallways."""
        robot = Robot(
            pose=Pose(x=1.0, y=0.5, yaw=0.0),
        )
        robot.world = self.test_world

        # Try to open and close with an unset location.
        with pytest.warns(UserWarning) as warn_info:
            result = robot.open_location()
        assert not result
        assert warn_info[0].message.args[0] == "Robot location is not set. Cannot open."

        with pytest.warns(UserWarning) as warn_info:
            result = robot.close_location()
        assert not result
        assert (
            warn_info[0].message.args[0] == "Robot location is not set. Cannot close."
        )

        # Now set the location to a non-openable location, and get a different warning.
        robot.location = self.test_world.get_room_by_name("kitchen")
        with pytest.warns(UserWarning) as warn_info:
            result = robot.open_location()
        assert not result
        assert warn_info[0].message.args[0] == "Robot is not at an openable location."

        with pytest.warns(UserWarning) as warn_info:
            result = robot.close_location()
        assert not result
        assert warn_info[0].message.args[0] == "Robot is not at a closeable location."

        # Set the location to an openable location, which should work.
        robot.location = self.test_world.get_hallways_from_rooms("kitchen", "bedroom")[
            0
        ]
        assert robot.close_location()
        assert robot.open_location()

        # Set a manipulated object, which will stop open/close from succeeding.
        robot.manipulated_object = self.test_world.get_object_by_name("banana0")
        with pytest.warns(UserWarning) as warn_info:
            result = robot.open_location()
        assert not result
        assert (
            warn_info[0].message.args[0] == "Robot is holding an object. Cannot open."
        )

        with pytest.warns(UserWarning) as warn_info:
            result = robot.close_location()
        assert not result
        assert (
            warn_info[0].message.args[0] == "Robot is holding an object. Cannot close."
        )

        # Add a new robot to the world inside the hallway.
        # Now, closing should fail because of this obstructing robot.
        robot.manipulated_object = None
        new_robot = Robot(name="blocker")
        self.test_world.add_robot(new_robot, pose=Pose(x=2.5, y=0.5, yaw=0.0))
        with pytest.warns(UserWarning) as warn_info:
            result = robot.close_location()
        assert not result
        assert (
            warn_info[0].message.args[0]
            == "Robot blocker is in Hallway: hall_kitchen_bedroom. Cannot close."
        )

    def test_execute_action(self):
        """Tests execution of a single action."""
        init_pose = Pose(x=1.0, y=0.5, yaw=0.0)
        robot = Robot(
            pose=init_pose,
            path_planner=PathPlanner("world_graph", world=self.test_world),
            path_executor=ConstantVelocityExecutor(linear_velocity=5.0, dt=0.1),
        )
        robot.location = "kitchen"
        robot.world = self.test_world
        action = TaskAction(
            "navigate",
            source_location="kitchen",
            target_location="my_desk",
        )

        # Blocking action
        result = robot.execute_action(action, blocking=True)
        assert result

        # Non-blocking action
        robot.set_pose(init_pose)
        result = robot.execute_action(action, blocking=False)
        assert result
        assert robot.executing_action
        assert robot.current_action == action
        while robot.executing_action:
            time.sleep(0.1)
        assert not robot.executing_action
        assert robot.current_action is None

    def test_execute_invalid_action(self):
        """Tests execution of an action that is not recognized as a valid type."""
        init_pose = Pose(x=1.0, y=0.5, yaw=0.0)
        robot = Robot(name="test_robot", pose=init_pose)
        robot.world = self.test_world
        action = TaskAction("bad_action")

        with pytest.warns(UserWarning) as warn_info:
            result = robot.execute_action(action)
        assert not result
        assert (
            warn_info[0].message.args[0]
            == "[test_robot] Invalid action type: bad_action."
        )

    def test_execute_action_simulated_options(self):
        """Test execution of an action that has simulated options."""
        init_pose = Pose(x=1.0, y=0.5, yaw=0.0)
        robot = Robot(
            pose=init_pose,
            path_planner=PathPlanner("world_graph", world=self.test_world),
            path_executor=ConstantVelocityExecutor(linear_velocity=5.0, dt=0.1),
        )
        robot.location = "kitchen"
        robot.world = self.test_world
        action = TaskAction(
            "navigate",
            source_location="kitchen",
            target_location="my_desk",
            execution_options=ExecutionOptions(
                delay=0.1,
                success_probability=0.5,
                rng_seed=1234,
            ),
        )

        # The action should fail the first time but succeed the second time.
        assert not robot.execute_action(action, blocking=True)
        assert robot.execute_action(action, blocking=True)

    def test_execute_plan(self):
        """Tests execution of a plan consisting of multiple actions."""
        init_pose = Pose(x=1.0, y=0.5, yaw=0.0)
        robot = Robot(
            pose=init_pose,
            path_planner=PathPlanner("world_graph", world=self.test_world),
            path_executor=ConstantVelocityExecutor(linear_velocity=5.0, dt=0.1),
        )
        robot.location = "kitchen"
        robot.world = self.test_world

        actions = [
            TaskAction(
                "navigate",
                source_location="kitchen",
                target_location="my_desk",
            ),
            TaskAction("detect", object="apple"),
            TaskAction("pick", object="apple"),
            TaskAction("place", "object", "apple"),
            TaskAction(
                "navigate",
                source_location="my_desk",
                target_location="hall_kitchen_bedroom",
            ),
            TaskAction("close", target_location="hall_kitchen_bedroom"),
            TaskAction("open", target_location="hall_kitchen_bedroom"),
        ]
        plan = TaskPlan(actions=actions)

        result, num_completed = robot.execute_plan(plan)

        assert result
        assert num_completed == 7

    def test_execute_blank_plan(self):
        """Tests a trivial case of executing a plan that is None."""
        init_pose = Pose(x=1.0, y=0.5, yaw=0.0)
        robot = Robot(name="test_robot", pose=init_pose)
        robot.world = self.test_world

        with pytest.warns(UserWarning) as warn_info:
            result = robot.execute_plan(None)
        assert not result
        assert warn_info[0].message.args[0] == "[test_robot] Plan is None. Returning."

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
