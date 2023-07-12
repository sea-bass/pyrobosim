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
    assert new_pose.x == pytest.approx(orig_pose.x)
    assert new_pose.y == pytest.approx(orig_pose.y)
    assert new_pose.z == pytest.approx(orig_pose.z)
    assert new_pose.q == pytest.approx(orig_pose.q)


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
        assert orig_pose.x == pytest.approx(new_pose.x)
        assert orig_pose.y == pytest.approx(new_pose.y)
        assert orig_pose.z == pytest.approx(new_pose.z)
        assert orig_pose.q == pytest.approx(new_pose.q)
