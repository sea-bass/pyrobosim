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


@pytest.mark.parametrize(
    "angle_units, roll, pitch, yaw",
    [
        ("radians", np.pi / 2, 0.0, -np.pi / 2),
        ("degrees", 90.0, 0.0, -90.0),
    ],
)
def test_pose_from_euler_angles(angle_units, roll, pitch, yaw):
    """Test creating a pose using Euler angles specified in different units."""
    pose = Pose(roll=roll, pitch=pitch, yaw=yaw, angle_units=angle_units)

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

    # Invalid list lengths should raise an exception.
    with pytest.raises(ValueError) as exc_info:
        Pose.from_list([1.0, 2.0, 3.0, 4.0, 5.0])
    assert exc_info.value.args[0] == "List must contain 2, 3, 4, 6, or 7 elements."

    # Invalid angle_units raise an exception
    with pytest.raises(ValueError) as exc_info:
        Pose(angle_units="notavalidangle")
    assert exc_info.value.args[0] == (
        "Invalid angle units provided. It should be either 'radians' or 'degrees'."
        "Additionally, if 'q' is provided, angle units are ignored."
    )


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


@pytest.mark.parametrize(
    "angle_units, angle_value", [("radians", np.pi / 2), ("degrees", 90.0)]
)
def test_pose_to_from_dict(angle_units, angle_value):
    """Test creating poses using a dictionary and saving them back out."""
    pose_dict = {}
    pose = Pose.from_dict(pose_dict)
    assert pose.x == pytest.approx(0.0)
    assert pose.y == pytest.approx(0.0)
    assert pose.z == pytest.approx(0.0)
    assert pose.eul == pytest.approx([0.0, 0.0, 0.0])
    assert pose.q == pytest.approx([1.0, 0.0, 0.0, 0.0])

    pose_dict = {
        "position": {"x": 1.0, "y": 2.0, "z": 3.0},
        "rotation_eul": {
            "yaw": angle_value,
            "pitch": angle_value / 2,
            "roll": -angle_value,
            "angle_units": angle_units,
        },
    }
    pose = Pose.from_dict(pose_dict)
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(2.0)
    assert pose.z == pytest.approx(3.0)
    assert pose.eul == pytest.approx([-np.pi / 2, np.pi / 4, np.pi / 2])

    pose_dict["rotation_quat"] = {"w": 0.707107, "x": -0.707107, "y": 0.0, "z": 0.0}
    pose = Pose.from_dict(pose_dict)
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(2.0)
    assert pose.z == pytest.approx(3.0)
    assert pose.eul == pytest.approx([-np.pi / 2, 0.0, 0.0])
    assert pose.q == pytest.approx([0.707107, -0.707107, 0.0, 0.0])

    out_dict = pose.to_dict()
    assert "position" in out_dict
    pos = out_dict["position"]
    assert "x" in pos
    assert pos["x"] == pytest.approx(1.0)
    assert "y" in pos
    assert pos["y"] == pytest.approx(2.0)
    assert "z" in pos
    assert pos["z"] == pytest.approx(3.0)
    assert "rotation_quat" in out_dict
    quat = out_dict["rotation_quat"]
    assert "w" in quat
    assert quat["w"] == pytest.approx(0.707107)
    assert "x" in quat
    assert quat["x"] == pytest.approx(-0.707107)
    assert "y" in quat
    assert quat["y"] == pytest.approx(0.0)
    assert "z" in quat
    assert quat["z"] == pytest.approx(0.0)


@pytest.mark.parametrize(
    "angle_units, angle_value", [("radians", np.pi / 2), ("degrees", 90.0)]
)
def test_construct_pose(angle_units, angle_value):
    """Test pose construct function that accepts various types"""
    pose_from_list = Pose.construct([1.0, 2.0, 3.0, np.pi / 2])
    pose_from_dict = Pose.construct(
        {
            "position": {"x": 1.0, "y": 2.0, "z": 3.0},
            "rotation_eul": {"yaw": angle_value, "angle_units": angle_units},
        }
    )

    expected_tform = np.array(
        [
            [0.0, -1.0, 0.0, 1.0],
            [1.0, 0.0, 0.0, 2.0],
            [0.0, 0.0, 1.0, 3.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    pose_from_tform = Pose.construct(expected_tform)

    assert pose_from_list.is_approx(pose_from_dict)
    assert pose_from_dict.is_approx(pose_from_tform)

    with pytest.raises(ValueError) as exc_info:
        Pose.construct(42)
    assert exc_info.value.args[0] == "Cannot construct pose from object of type int."


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


@pytest.mark.parametrize(
    "x, y, z, roll, pitch, yaw, angle_units",
    [
        (1.0, 2.0, 3.0, np.pi / 2, 0.0, -np.pi / 2, "radians"),
        (1.0, 2.0, 3.0, 90.0, 0.0, -90.0, "degrees"),
    ],
)
def test_get_matrices(x, y, z, roll, pitch, yaw, angle_units):
    """Test getting matrices from a pose with different angle units"""
    pose = Pose(
        x=x,
        y=y,
        z=z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        angle_units=angle_units,
    )

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


def test_is_approx_radians():
    """Test approximate equivalence functionality for pose in radians"""
    pose1 = Pose(
        x=1.0,
        y=2.0,
        z=3.0,
        roll=np.pi / 2,
        pitch=0.0,
        yaw=-np.pi / 2,
        angle_units="radians",
    )
    pose2 = Pose(
        x=1.0 + 1e-4,
        y=2.0,
        z=3.0,
        roll=np.pi / 2,
        pitch=1e-4,
        yaw=-np.pi / 2,
        angle_units="radians",
    )

    # Default values are too tight for the poses above
    assert not pose1.is_approx(pose2)

    # Can relax tolerances
    assert pose1.is_approx(pose2, rel_tol=1e-3, abs_tol=1e-3)

    # Check datatype exception.
    with pytest.raises(TypeError) as exc_info:
        pose1.is_approx(42.0)
    assert exc_info.value.args[0] == "Expected a Pose object."


def test_is_approx_degrees():
    """Test approximate equivalence functionality for pose in degrees"""
    pose1 = Pose(
        x=1.0, y=2.0, z=3.0, roll=90.0, pitch=0.0, yaw=-90.0, angle_units="degrees"
    )
    pose2 = Pose(
        x=1.0 + 1e-4,
        y=2.0,
        z=3.0,
        roll=90.0,
        pitch=0.0057,
        yaw=-90.0,
        angle_units="degrees",
    )

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

    # Check datatype exception.
    with pytest.raises(TypeError) as exc_info:
        pose1 == 42.0
    assert exc_info.value.args[0] == "Expected a Pose object."


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
