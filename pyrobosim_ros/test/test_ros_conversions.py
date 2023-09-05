# Tests for pyrobosim ROS conversions functionality

import pytest

import pyrobosim.planning.actions as acts
from pyrobosim.utils.motion import Path
from pyrobosim.utils.pose import Pose
from pyrobosim_ros.ros_conversions import (
    path_from_ros,
    path_to_ros,
    pose_from_ros,
    pose_to_ros,
    task_action_from_ros,
    task_action_to_ros,
    task_plan_from_ros,
    task_plan_to_ros,
)


def test_pose_conversion():
    """Tests round-trip conversion of pose objects."""
    # Create a pyrobosim pose
    orig_pose = Pose(x=1.0, y=2.0, z=3.0, q=[0.707, 0.0, 0.707, 0])  # wxyz

    # Convert to a ROS Message
    ros_pose = pose_to_ros(orig_pose)
    assert ros_pose.position.x == pytest.approx(orig_pose.x)
    assert ros_pose.position.y == pytest.approx(orig_pose.y)
    assert ros_pose.position.z == pytest.approx(orig_pose.z)
    assert ros_pose.orientation.w == pytest.approx(orig_pose.q[0])
    assert ros_pose.orientation.x == pytest.approx(orig_pose.q[1])
    assert ros_pose.orientation.y == pytest.approx(orig_pose.q[2])
    assert ros_pose.orientation.z == pytest.approx(orig_pose.q[3])

    # Convert back to a pyrobosim Pose
    new_pose = pose_from_ros(ros_pose)
    assert new_pose.is_approx(orig_pose)


def test_path_conversion():
    """Tests round-trip conversion of path objects."""

    # Create a pyrobosim path
    poses = [
        Pose(x=0.0, y=0.0, z=0.0, q=[1.0, 0.0, 0.0, 0.0]),
        Pose(x=1.0, y=0.0, z=0.0, q=[0.707, 0.0, 0.0, 0.707]),
        Pose(x=1.0, y=1.0, z=0.0, q=[0.0, 0.0, 0.0, 1.0]),
    ]
    orig_path = Path(poses)

    # Convert to a ROS Message
    ros_path = path_to_ros(orig_path)
    assert len(ros_path.poses) == orig_path.num_poses
    for orig_pose, ros_pose in zip(orig_path.poses, ros_path.poses):
        assert ros_pose.position.x == pytest.approx(orig_pose.x)
        assert ros_pose.position.y == pytest.approx(orig_pose.y)
        assert ros_pose.position.z == pytest.approx(orig_pose.z)
        assert ros_pose.orientation.w == pytest.approx(orig_pose.q[0])
        assert ros_pose.orientation.x == pytest.approx(orig_pose.q[1])
        assert ros_pose.orientation.y == pytest.approx(orig_pose.q[2])
        assert ros_pose.orientation.z == pytest.approx(orig_pose.q[3])

    # Convert back to a pyrobosim path
    new_path = path_from_ros(ros_path)
    assert new_path.num_poses == orig_path.num_poses
    for orig_pose, new_pose in zip(orig_path.poses, new_path.poses):
        assert orig_pose.is_approx(new_pose)


def test_task_action_conversion():
    """Tests round-trip conversion of task action objects."""

    # Create a pyrobosim task action
    orig_action = acts.TaskAction(
        "pick",
        robot="robot0",
        object="apple",
        room="kitchen",
        source_location="table",
        target_location=None,
        pose=Pose(x=1.0, y=2.0, z=3.0, q=[1.0, 0.0, 0.0, 0.0]),
        path=Path(),
        cost=42.0,
    )

    # Convert to a ROS message
    ros_action = task_action_to_ros(orig_action)
    assert ros_action.robot == orig_action.robot
    assert ros_action.type == orig_action.type
    assert ros_action.object == orig_action.object
    assert ros_action.room == orig_action.room
    assert ros_action.source_location == orig_action.source_location
    assert ros_action.target_location == ""  # No None in ROS messages
    assert ros_action.cost == pytest.approx(orig_action.cost)
    assert ros_action.has_pose
    assert ros_action.pose.position.x == pytest.approx(orig_action.pose.x)
    assert ros_action.pose.position.y == pytest.approx(orig_action.pose.y)
    assert ros_action.pose.position.z == pytest.approx(orig_action.pose.z)
    assert ros_action.pose.orientation.w == pytest.approx(orig_action.pose.q[0])
    assert ros_action.pose.orientation.x == pytest.approx(orig_action.pose.q[1])
    assert ros_action.pose.orientation.y == pytest.approx(orig_action.pose.q[2])
    assert ros_action.pose.orientation.z == pytest.approx(orig_action.pose.q[3])
    assert len(ros_action.path.poses) == orig_action.path.num_poses

    # Convert back to a pyrobosim task action
    new_action = task_action_from_ros(ros_action)
    assert new_action.robot == orig_action.robot
    assert new_action.type == orig_action.type
    assert new_action.object == orig_action.object
    assert new_action.room == orig_action.room
    assert new_action.source_location == orig_action.source_location
    assert new_action.target_location == orig_action.target_location
    assert new_action.cost == pytest.approx(orig_action.cost)
    assert new_action.pose is not None
    assert new_action.pose.x == pytest.approx(orig_action.pose.x)
    assert new_action.pose.y == pytest.approx(orig_action.pose.y)
    assert new_action.pose.z == pytest.approx(orig_action.pose.z)
    assert new_action.pose.q == pytest.approx(orig_action.pose.q)
    assert new_action.path.num_poses == orig_action.path.num_poses


def test_task_plan_conversion():
    """Tests round-trip conversion of task plan objects."""

    # Create a pyrobosim task plan
    nav_path = Path(
        [
            Pose(x=0.0, y=0.0, z=0.0, q=[1.0, 0.0, 0.0, 0.0]),
            Pose(x=1.0, y=0.0, z=0.0, q=[0.707, 0.0, 0.0, 0.707]),
            Pose(x=1.0, y=1.0, z=0.0, q=[0.0, 0.0, 0.0, 1.0]),
        ]
    )
    place_pose = Pose(x=0.8, y=1.0, z=0.5, q=[1.0, 0.0, 0.0, 0.0])
    actions = [
        acts.TaskAction("pick", object="apple", source_location="table0", cost=0.5),
        acts.TaskAction(
            "move",
            source_location="table0",
            target_location="desk0",
            path=nav_path,
            cost=0.75,
        ),
        acts.TaskAction(
            "place", object="apple", target_location="desk0", pose=place_pose, cost=0.5
        ),
    ]
    orig_plan = acts.TaskPlan(robot="robot0", actions=actions)

    # Convert to a ROS message
    ros_plan = task_plan_to_ros(orig_plan)
    assert ros_plan.robot == orig_plan.robot
    assert len(ros_plan.actions) == len(orig_plan.actions)
    for orig_action, ros_action in zip(orig_plan.actions, ros_plan.actions):
        assert orig_action.type == ros_action.type
    assert ros_plan.cost == pytest.approx(orig_plan.total_cost)

    # Convert back to a pyrobosim task action
    new_plan = task_plan_from_ros(ros_plan)
    assert new_plan.robot == orig_plan.robot
    assert len(new_plan.actions) == len(orig_plan.actions)
    for orig_action, new_action in zip(orig_plan.actions, new_plan.actions):
        assert orig_action.type == new_action.type
    assert new_plan.total_cost == pytest.approx(orig_plan.total_cost)
