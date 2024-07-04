#!/usr/bin/env python3

"""
Unit tests for robot class.
"""
import os
import numpy as np
import pytest
import time

from pyrobosim.core import Pose, Robot, WorldYamlLoader
from pyrobosim.navigation import ConstantVelocityExecutor, PathPlanner
from pyrobosim.planning.actions import TaskAction, TaskPlan
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

    def test_robot_path_executor(self):
        """Check that path executors can be used from a robot."""
        init_pose = Pose(x=1.0, y=0.5, yaw=0.0)
        goal_pose = Pose(x=2.5, y=3.0, yaw=np.pi / 2.0)
        target_location = self.test_world.get_entity_by_name("bedroom")

        robot = Robot(
            pose=init_pose,
            path_planner=PathPlanner("world_graph", world=self.test_world),
            path_executor=ConstantVelocityExecutor(linear_velocity=5.0, dt=0.1),
        )
        robot.world = self.test_world
        robot.location = self.test_world.get_entity_by_name("kitchen")

        # Non-threaded option -- blocks
        robot.set_pose(init_pose)
        path = robot.plan_path(goal=goal_pose)
        result = robot.follow_path(
            path, target_location=target_location, use_thread=False
        )
        assert result

        # Threaded option with blocking
        robot.set_pose(init_pose)
        path = robot.plan_path(goal=goal_pose)
        result = robot.follow_path(
            path, target_location=target_location, use_thread=True, blocking=True
        )

        # Threaded option without blocking -- must check result
        robot.set_pose(init_pose)
        path = robot.plan_path(goal=goal_pose)
        robot.follow_path(
            path, target_location=target_location, use_thread=True, blocking=False
        )
        assert robot.executing_nav
        while robot.executing_nav:
            time.sleep(0.1)
        assert not robot.executing_nav
        pose = robot.get_pose()
        assert pose.x == pytest.approx(goal_pose.x)
        assert pose.y == pytest.approx(goal_pose.y)
        assert pose.q == pytest.approx(goal_pose.q)

    def test_robot_manipulation(self):
        """Check that the robot can manipulate objects."""
        # Spawn the robot near the kitchen table
        robot = Robot(
            pose=Pose(x=1.0, y=0.5, yaw=0.0),
        )
        robot.world = self.test_world
        robot.location = self.test_world.get_entity_by_name("table0_tabletop")

        # Pick up the apple on the kitchen table (named "gala")
        result = robot.pick_object("apple")
        assert result
        assert robot.manipulated_object == self.test_world.get_entity_by_name("gala")

        # Try pick up another object, which should fail since the robot is holding something.
        with pytest.warns(UserWarning):
            result = robot.pick_object("banana")
            assert not result

        # Try place the object
        result = robot.place_object()
        assert result
        assert robot.manipulated_object is None

        # Try place an object, which should fail since the robot is holding nothing.
        with pytest.warns(UserWarning):
            result = robot.place_object()
            assert not result

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
        ]
        plan = TaskPlan(actions=actions)

        result, num_completed = robot.execute_plan(plan)

        assert result
        assert num_completed == 4

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
