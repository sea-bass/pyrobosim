#!/usr/bin/env python3

"""
Unit tests for pose utilities class.
"""

import numpy as np
import pytest

from pyrobosim.core import Pose
from pyrobosim.utils.pose import (
    get_angle,
    get_distance,
    get_bearing_range,
    rot2d,
    wrap_angle,
)


########################
# Tests for Pose class #
########################
def test_pose_default_args():
    """Test creating a pose with default arguments."""
    pose = Pose()
    assert pose.x == pytest.approx(0.0)
    assert pose.y == pytest.approx(0.0)
    assert pose.z == pytest.approx(0.0)
    assert pose.eul == pytest.approx([0.0, 0.0, 0.0])
    assert pose.q == pytest.approx([1.0, 0.0, 0.0, 0.0])


def test_pose_from_position():
    """Test creating a pose with nondefault position arguments."""
    pose = Pose(x=1.0, y=2.0, z=3.0)
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(2.0)
    assert pose.z == pytest.approx(3.0)
    assert pose.eul == pytest.approx([0.0, 0.0, 0.0])
    assert pose.q == pytest.approx([1.0, 0.0, 0.0, 0.0])


def test_pose_from_euler_angles():
    """Test creating a pose using Euler angles."""
    pose = Pose(roll=np.pi / 2, pitch=0.0, yaw=-np.pi / 2)
    assert pose.x == pytest.approx(0.0)
    assert pose.y == pytest.approx(0.0)
    assert pose.z == pytest.approx(0.0)
    assert pose.eul == pytest.approx([np.pi / 2, 0.0, -np.pi / 2])
    assert pose.q == pytest.approx([0.5, 0.5, 0.5, -0.5])


def test_pose_from_quaternion():
    """Test creating a pose using quaternions."""
    pose = Pose(q=[1.0, 1.0, 1.0, -1.0])  # Unnormalized
    assert pose.x == pytest.approx(0.0)
    assert pose.y == pytest.approx(0.0)
    assert pose.z == pytest.approx(0.0)
    assert pose.eul == pytest.approx([np.pi / 2, 0.0, -np.pi / 2])
    assert pose.q == pytest.approx([0.5, 0.5, 0.5, -0.5])


def test_pose_from_lists():
    """Test creating a pose using lists."""
    # 2-element lists should be [x, y]
    pose = Pose.from_list([1.0, 2.0])
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(2.0)

    # 3-element lists should be [x, y, z]
    pose = Pose.from_list([1.0, 2.0, 3.0])
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(2.0)
    assert pose.z == pytest.approx(3.0)

    # 4-element lists should be [x, y, z, yaw]
    pose = Pose.from_list([1.0, 2.0, 3.0, np.pi / 2])
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(2.0)
    assert pose.z == pytest.approx(3.0)
    assert pose.eul == pytest.approx([0.0, 0.0, np.pi / 2])
    assert pose.q == pytest.approx([np.sqrt(2) / 2, 0.0, 0.0, np.sqrt(2) / 2])

    # 6-element lists should be [x, y, z, roll, pitch, yaw]
    pose = Pose.from_list([1.0, 2.0, 3.0, np.pi / 2, 0.0, -np.pi / 2])
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(2.0)
    assert pose.z == pytest.approx(3.0)
    assert pose.eul == pytest.approx([np.pi / 2, 0.0, -np.pi / 2])
    assert pose.q == pytest.approx([0.5, 0.5, 0.5, -0.5])

    # 7-element lists should be [x, y, z, qw, qx, qy, qz]
    pose = Pose.from_list([1.0, 2.0, 3.0, 0.5, 0.5, 0.5, -0.5])
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(2.0)
    assert pose.z == pytest.approx(3.0)
    assert pose.eul == pytest.approx([np.pi / 2, 0.0, -np.pi / 2])
    assert pose.q == pytest.approx([0.5, 0.5, 0.5, -0.5])


def test_pose_from_transform():
    """Test creating a pose using a transform."""
    tform = np.array(
        [
            [0.0, 1.0, 0.0, 1.0],
            [0.0, 0.0, -1.0, 2.0],
            [-1.0, 0.0, 0.0, 3.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    pose = Pose.from_transform(tform)

    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(2.0)
    assert pose.z == pytest.approx(3.0)
    assert pose.eul == pytest.approx([np.pi / 2, 0.0, -np.pi / 2])
    assert pose.q == pytest.approx([0.5, 0.5, 0.5, -0.5])


def test_get_linear_distance():
    """Test linear distance calculation function."""
    pose1 = Pose(x=1.0, y=2.0, z=3.0)
    pose2 = Pose(x=4.0, y=-2.0, z=5.0)

    assert pose1.get_linear_distance(pose1, pose1) == pytest.approx(0.0)
    assert pose1.get_linear_distance(pose2, ignore_z=True) == pytest.approx(5.0)
    assert pose1.get_linear_distance(pose2, ignore_z=False) == pytest.approx(5.3851648)


def test_get_angular_distance():
    """Test angular distance calculation function."""
    pose1 = Pose(x=1.0, y=2.0)

    # Straight ahead in the X direction
    pose2 = Pose(x=3.0, y=2.0)
    assert pose1.get_angular_distance(pose2) == pytest.approx(0.0)

    # Straight ahead in the Y direction
    pose2 = Pose(x=1.0, y=3.0)
    assert pose1.get_angular_distance(pose2) == pytest.approx(np.pi / 2)

    # Some other angle
    pose2 = Pose(x=0.0, y=1.0)
    assert pose1.get_angular_distance(pose2) == pytest.approx(-3 * np.pi / 4)


def test_get_matrices():
    """Tests getting matrices from a pose."""
    pose = Pose(x=1.0, y=2.0, z=3.0, roll=np.pi / 2, pitch=0.0, yaw=-np.pi / 2)

    expected_tform = np.array(
        [
            [0.0, 1.0, 0.0, 1.0],
            [0.0, 0.0, -1.0, 2.0],
            [-1.0, 0.0, 0.0, 3.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    expected_translation_matrix = np.eye(4)
    expected_translation_matrix[:3, 3] = expected_tform[:3, 3]

    assert pose.get_transform_matrix() == pytest.approx(expected_tform)
    assert pose.get_rotation_matrix() == pytest.approx(expected_tform[:3, :3])
    assert pose.get_translation_matrix() == pytest.approx(expected_translation_matrix)


def test_is_approx():
    """Test approximate equivalence functionality"""
    pose1 = Pose(x=1.0, y=2.0, z=3.0, roll=np.pi / 2, pitch=0.0, yaw=-np.pi / 2)
    pose2 = Pose(x=1.0 + 1e-4, y=2.0, z=3.0, roll=np.pi / 2, pitch=1e-4, yaw=-np.pi / 2)

    # Default values are too tight for the poses above
    assert not pose1.is_approx(pose2)

    # Can relax tolerances
    assert pose1.is_approx(pose2, rel_tol=1e-3, abs_tol=1e-3)


def test_equality():
    """Test pose equality operator."""
    # Original pose
    pose1 = Pose(x=1.0, y=2.0, z=3.0, roll=np.pi / 2, pitch=0.0, yaw=-np.pi / 2)
    # The same values, but different object
    pose2 = Pose(x=1.0, y=2.0, z=3.0, roll=np.pi / 2, pitch=0.0, yaw=-np.pi / 2)
    # Completely different values
    pose3 = Pose(x=0.0, y=0.0, z=0.0)

    assert pose1 == pose1
    assert pose1 == pose2
    assert not pose1 == pose3


###############################
# Tests for utility functions #
###############################
def test_get_angle():
    """Test the utility function for getting 2D angle between points."""
    # Straight ahead in the X direction
    assert get_angle([1.0, 2.0], [3.0, 2.0]) == pytest.approx(0.0)

    # Straight ahead in the Y direction
    assert get_angle([1.0, 2.0], [1.0, 3.0]) == pytest.approx(np.pi / 2)

    # Some other angle
    assert get_angle([1.0, 2.0], [0.0, 1.0]) == pytest.approx(-3 * np.pi / 4)


def test_get_distance():
    """Test the utility function for getting 2D distance between points."""
    assert get_distance([1.0, 2.0], [1.0, 2.0]) == pytest.approx(0.0)
    assert get_distance([1.0, 2.0], [4.0, -2.0]) == pytest.approx(5.0)


def test_get_bearing_range():
    """Test the utility function for getting bearing and range between two points."""
    bear, rng = get_bearing_range([1.0, 2.0], [4.0, -2.0])
    assert bear == pytest.approx(-np.arctan2(4.0, 3.0))
    assert rng == pytest.approx(5.0)


def test_rot2d():
    """Test the utility to rotate a 2-element position vector by an angle."""
    orig_vec = [1.0, 0.0]

    assert rot2d(orig_vec, 0.0) == pytest.approx(orig_vec)
    assert rot2d(orig_vec, np.pi / 2) == pytest.approx([0.0, 1.0])
    assert rot2d(orig_vec, np.pi) == pytest.approx([-1.0, 0.0])
    assert rot2d(orig_vec, -3 * np.pi / 4) == pytest.approx(
        [-np.sqrt(2) / 2, -np.sqrt(2) / 2]
    )


def test_wrap_angle():
    """Test the angle wrapping utility function."""
    # Angles that do not wrap
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(-np.pi) == pytest.approx(-np.pi)
    assert wrap_angle(np.pi) == pytest.approx(np.pi)
    assert not wrap_angle(None)

    # Angles that wrap
    assert wrap_angle(2 * np.pi) == pytest.approx(0.0)
    assert wrap_angle(2 * np.pi + 0.5) == pytest.approx(0.5)
    assert wrap_angle(-4 * np.pi - 0.5) == pytest.approx(-0.5)
    assert wrap_angle(1200 * np.pi + np.pi / 4) == pytest.approx(np.pi / 4)
