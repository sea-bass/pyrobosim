#!/usr/bin/env python3

"""Unit tests for the trajectory utilities"""

import numpy as np
import pytest

from pyrobosim.core import Pose
from pyrobosim.utils.motion import Path
from pyrobosim.utils.trajectory import (
    Trajectory,
    get_constant_speed_trajectory,
    interpolate_trajectory,
)


def test_create_empty_trajectory():
    traj = Trajectory()
    assert traj.num_points() == 0
    assert traj.is_empty()


def test_create_trajectory():
    traj = Trajectory(
        np.array([0.0, 1.0, 2.0]),  # Time points
        np.array([0.1, 0.2, 0.3]),  # X points
        np.array([1.1, 1.2, 1.3]),  # Y points
        np.array([0.0, np.pi / 4, np.pi / 2]),  # Yaw points
    )
    assert traj.num_points() == 3
    assert not traj.is_empty()

    assert traj.pose_at(0).x == 0.1
    assert traj.pose_at(0).y == 1.1
    assert traj.pose_at(0).get_yaw() == 0.0

    assert traj.pose_at(1).x == 0.2
    assert traj.pose_at(1).y == 1.2
    assert traj.pose_at(1).get_yaw() == np.pi / 4

    assert traj.pose_at(2).x == 0.3
    assert traj.pose_at(2).y == 1.3
    assert traj.pose_at(2).get_yaw() == np.pi / 2


def test_pose_at_empty_trajectory():
    traj = Trajectory()
    with pytest.warns(UserWarning):
        assert traj.pose_at(0) is None


def test_pose_at_invalid_indices():
    traj = Trajectory(
        np.array(
            [
                0.0,
                1.0,
            ]
        ),  # Time points
        np.array([0.1, 0.2]),  # X points
        np.array([1.1, 1.2]),  # Y points
        np.array([0.0, np.pi / 4]),  # Yaw points
    )

    with pytest.warns(UserWarning):
        assert traj.pose_at(-1) is None
        assert traj.pose_at(5) is None


def test_delete_empty_trajectory():
    traj = Trajectory()
    with pytest.warns(UserWarning):
        traj.delete(0)


def test_delete_at_invalid_indices():
    traj = Trajectory(
        np.array(
            [
                0.0,
                1.0,
            ]
        ),  # Time points
        np.array([0.1, 0.2]),  # X points
        np.array([1.1, 1.2]),  # Y points
        np.array([0.0, np.pi / 4]),  # Yaw points
    )

    with pytest.warns(UserWarning):
        traj.delete(-1)
        traj.delete(5)

    assert traj.num_points() == 2


def test_delete():
    traj = Trajectory(
        np.array([0.0, 1.0, 2.0]),  # Time points
        np.array([0.1, 0.2, 0.3]),  # X points
        np.array([1.1, 1.2, 1.3]),  # Y points
        np.array([0.0, np.pi / 4, np.pi / 2]),  # Yaw points
    )

    traj.delete(1)

    assert traj.num_points() == 2

    assert traj.pose_at(0).x == 0.1
    assert traj.pose_at(0).y == 1.1
    assert traj.pose_at(0).get_yaw() == 0.0

    assert traj.pose_at(1).x == 0.3
    assert traj.pose_at(1).y == 1.3
    assert traj.pose_at(1).get_yaw() == np.pi / 2


def test_create_invalid_trajectory():
    with pytest.raises(ValueError):
        Trajectory(
            np.array([0.0, 1.0]),  # Time points
            np.array([0.1, 0.2, 0.3]),  # X points
            np.array([1.1, 1.2, 1.3, 1.4]),  # Y points
            np.array([0.0, np.pi / 4, np.pi / 2]),  # Yaw points
        )


def test_get_constant_speed_trajectory_empty_path():
    path = Path(poses=[])
    with pytest.warns(UserWarning):
        assert get_constant_speed_trajectory(path) is None


def test_get_constant_speed_trajectory_insufficient_points():
    path = Path(poses=[Pose(x=1.0, y=1.0)])
    with pytest.warns(UserWarning):
        assert get_constant_speed_trajectory(path) is None


def test_get_constant_speed_trajectory_unlimited_ang_vel():
    path = Path(
        poses=[
            Pose(x=0.0, y=0.0),
            Pose(x=1.0, y=0.0),
            Pose(x=1.0, y=1.0, yaw=np.pi / 2.0),
            Pose(x=0.0, y=1.0, yaw=-3.0 * np.pi / 4.0),
        ]
    )
    traj = get_constant_speed_trajectory(path, linear_velocity=0.5)

    assert traj.num_points() == 4
    assert traj.t_pts == pytest.approx([0.0, 2.0, 4.0, 6.0], rel=1e-4)
    assert traj.x_pts == pytest.approx([0.0, 1.0, 1.0, 0.0])
    assert traj.y_pts == pytest.approx([0.0, 0.0, 1.0, 1.0])
    assert traj.yaw_pts == pytest.approx([0.0, 0.0, np.pi / 2.0, -3.0 * np.pi / 4.0])


def test_get_constant_speed_trajectory_limited_ang_vel():
    path = Path(
        poses=[
            Pose(x=0.0, y=0.0),
            Pose(x=1.0, y=0.0),
            Pose(x=1.0, y=1.0, yaw=np.pi / 2.0),
            Pose(x=0.0, y=1.0, yaw=-3.0 * np.pi / 4.0),
        ]
    )
    traj = get_constant_speed_trajectory(
        path, linear_velocity=0.5, max_angular_velocity=np.pi / 8
    )

    assert traj.num_points() == 4
    assert traj.t_pts == pytest.approx([0.0, 2.0, 6.0, 12.0], rel=1e-4)
    assert traj.x_pts == pytest.approx([0.0, 1.0, 1.0, 0.0])
    assert traj.y_pts == pytest.approx([0.0, 0.0, 1.0, 1.0])
    assert traj.yaw_pts == pytest.approx([0.0, 0.0, np.pi / 2.0, -3.0 * np.pi / 4.0])


def test_interpolate_trajectory():
    path = Path(
        poses=[
            Pose(x=0.0, y=0.0),
            Pose(x=1.0, y=0.0),
            Pose(x=1.0, y=1.0, yaw=np.pi / 2.0),
            Pose(x=0.0, y=1.0, yaw=-3.0 * np.pi / 4.0),
        ]
    )
    traj = get_constant_speed_trajectory(path, linear_velocity=1.0)
    interpolated_traj = interpolate_trajectory(traj, dt=0.1)

    assert interpolated_traj.num_points() == 31


def test_interpolate_trajectory_duplicate_points():
    path = Path(
        poses=[
            Pose(x=0.0, y=0.0),
            Pose(x=0.0, y=0.0),  # Duplicate
            Pose(x=1.0, y=0.0),
            Pose(x=1.0, y=1.0),  # Duplicate
            Pose(x=1.0, y=1.0, yaw=np.pi / 2.0),
            Pose(x=0.0, y=1.0, yaw=-3.0 * np.pi / 4.0),
            Pose(x=0.0, y=1.0, yaw=-3.0 * np.pi / 4.0),  # Duplicate
        ]
    )
    traj = get_constant_speed_trajectory(path, linear_velocity=1.0)
    with pytest.warns(UserWarning):
        interpolated_traj = interpolate_trajectory(traj, dt=0.1)

    assert interpolated_traj.num_points() == 31


def test_interpolate_trajectory_insufficient_points():
    traj = Trajectory(
        (1.0,),  # Time
        (1.0,),  # x points
        (1.0,),  # y points
        (np.pi,),  # yaw points
    )
    with pytest.warns(UserWarning):
        interpolated_traj = interpolate_trajectory(traj, dt=0.1)
    assert interpolated_traj is None
