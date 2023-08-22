#!/usr/bin/env python3

"""Unit tests for the trajectory utilities"""

import numpy as np
import pytest

from pyrobosim.core import Pose
from pyrobosim.utils.motion import Path
from pyrobosim.navigation.trajectory import (
    get_constant_speed_trajectory,
    interpolate_trajectory,
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

    assert isinstance(traj, tuple)
    t_pts, x_pts, y_pts, yaw_pts = traj
    assert len(t_pts) == 4
    assert len(x_pts) == 4
    assert len(y_pts) == 4
    assert len(yaw_pts) == 4
    assert t_pts == pytest.approx([0.0, 2.0, 4.0, 6.0], rel=1e-4)
    assert x_pts == pytest.approx([0.0, 1.0, 1.0, 0.0])
    assert y_pts == pytest.approx([0.0, 0.0, 1.0, 1.0])
    assert yaw_pts == pytest.approx([0.0, 0.0, np.pi / 2.0, -3.0 * np.pi / 4.0])


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

    assert isinstance(traj, tuple)
    t_pts, x_pts, y_pts, yaw_pts = traj
    assert len(t_pts) == 4
    assert len(x_pts) == 4
    assert len(y_pts) == 4
    assert len(yaw_pts) == 4
    assert t_pts == pytest.approx([0.0, 2.0, 6.0, 12.0], rel=1e-4)
    assert x_pts == pytest.approx([0.0, 1.0, 1.0, 0.0])
    assert y_pts == pytest.approx([0.0, 0.0, 1.0, 1.0])
    assert yaw_pts == pytest.approx([0.0, 0.0, np.pi / 2.0, -3.0 * np.pi / 4.0])


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

    assert isinstance(interpolated_traj, tuple)
    t_pts, x_pts, y_pts, yaw_pts = interpolated_traj
    assert len(t_pts) == 31
    assert len(x_pts) == 31
    assert len(y_pts) == 31
    assert len(yaw_pts) == 31


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

    assert isinstance(interpolated_traj, tuple)
    t_pts, x_pts, y_pts, yaw_pts = interpolated_traj
    assert len(t_pts) == 31
    assert len(x_pts) == 31
    assert len(y_pts) == 31
    assert len(yaw_pts) == 31


def test_interpolate_trajectory_insufficient_points():
    traj = (
        (1.0,),  # Time
        (1.0,),  # x points
        (1.0,),  # y points
        (np.pi,),  # yaw points
    )
    with pytest.warns(UserWarning):
        interpolated_traj = interpolate_trajectory(traj, dt=0.1)
    assert interpolated_traj is None
