#!/usr/bin/env python3

"""Unit tests for the trajectory utilities"""

import numpy as np
import pytest
from pytest import LogCaptureFixture

from pyrobosim.core import Pose
from pyrobosim.utils.path import Path
from pyrobosim.utils.trajectory import (
    Trajectory,
    get_constant_speed_trajectory,
    interpolate_trajectory,
)


def test_create_empty_trajectory() -> None:
    traj = Trajectory()
    assert traj.num_points() == 0
    assert traj.is_empty()


def test_create_trajectory() -> None:
    poses = [
        Pose(x=0.1, y=1.1, yaw=0.0),
        Pose(x=0.2, y=1.2, yaw=np.pi / 4),
        Pose(x=0.3, y=1.3, yaw=np.pi / 2),
    ]
    traj = Trajectory([0.0, 1.0, 2.0], poses)
    assert traj.num_points() == 3
    assert not traj.is_empty()

    assert traj.poses[0].x == 0.1
    assert traj.poses[0].y == 1.1
    assert traj.poses[0].get_yaw() == 0.0

    assert traj.poses[1].x == 0.2
    assert traj.poses[1].y == 1.2
    assert traj.poses[1].get_yaw() == np.pi / 4

    assert traj.poses[2].x == 0.3
    assert traj.poses[2].y == 1.3
    assert traj.poses[2].get_yaw() == np.pi / 2


def test_delete_empty_trajectory(caplog: LogCaptureFixture) -> None:
    traj = Trajectory()
    assert not traj.delete(0)
    assert "Trajectory is empty. Cannot delete point." in caplog.text


def test_delete_at_invalid_indices(caplog: LogCaptureFixture) -> None:
    poses = [
        Pose(x=0.1, y=1.1, yaw=0.0),
        Pose(x=0.2, y=1.2, yaw=np.pi / 4),
        Pose(x=0.3, y=1.3, yaw=np.pi / 2),
    ]
    traj = Trajectory([0.0, 1.0, 2.0], poses)

    assert not traj.delete(-1)
    assert "Invalid index -1 for trajectory length 3" in caplog.text
    caplog.clear()

    assert not traj.delete(5)
    assert "Invalid index 5 for trajectory length 3" in caplog.text

    assert traj.num_points() == 3


def test_delete() -> None:
    poses = [
        Pose(x=0.1, y=1.1, yaw=0.0),
        Pose(x=0.2, y=1.2, yaw=np.pi / 4),
        Pose(x=0.3, y=1.3, yaw=np.pi / 2),
    ]
    traj = Trajectory([0.0, 1.0, 2.0], poses)

    traj.delete(1)
    assert traj.num_points() == 2

    assert traj.poses[0].x == 0.1
    assert traj.poses[0].y == 1.1
    assert traj.poses[0].get_yaw() == 0.0

    assert traj.poses[1].x == 0.3
    assert traj.poses[1].y == 1.3
    assert traj.poses[1].get_yaw() == np.pi / 2


def test_create_invalid_trajectory() -> None:
    poses = [
        Pose(x=0.1, y=1.1, yaw=0.0),
        Pose(x=0.2, y=1.2, yaw=np.pi / 4),
        Pose(x=0.3, y=1.3, yaw=np.pi / 2),
    ]
    with pytest.raises(ValueError):
        Trajectory([0.0, 10.0], poses)


def test_get_constant_speed_trajectory_empty_path(caplog: LogCaptureFixture) -> None:
    path = Path(poses=[])
    assert get_constant_speed_trajectory(path) is None
    assert "Insufficient points to generate trajectory." in caplog.text


def test_get_constant_speed_trajectory_insufficient_points(
    caplog: LogCaptureFixture,
) -> None:
    path = Path(poses=[Pose(x=1.0, y=1.0)])
    assert get_constant_speed_trajectory(path) is None
    assert "Insufficient points to generate trajectory." in caplog.text


def test_get_constant_speed_trajectory_unlimited_ang_vel() -> None:
    path = Path(
        poses=[
            Pose(x=0.0, y=0.0),
            Pose(x=1.0, y=0.0),
            Pose(x=1.0, y=1.0, yaw=np.pi / 2.0),
            Pose(x=0.0, y=1.0, yaw=-3.0 * np.pi / 4.0),
        ],
    )
    traj = get_constant_speed_trajectory(path, linear_velocity=0.5)

    assert traj.num_points() == 4
    assert traj.t_pts == pytest.approx([0.0, 2.0, 4.0, 6.0], rel=1e-4)
    assert np.all(traj.poses == path.poses)


@pytest.mark.parametrize(
    "start_yaw, end_yaw",
    [
        (0.0, np.pi / 2),
        (0.0, -np.pi / 2),
        (3 * np.pi / 4, -3 * np.pi / 4),
        (-3 * np.pi / 4, 3 * np.pi / 4),
    ],
)
@pytest.mark.parametrize("distance, duration", [(0.0, 4.0), (0.1, 4.0), (3.0, 6.0)])
def test_get_constant_speed_trajectory_angular_limit_both_directions(
    start_yaw: float, end_yaw: float, distance: float, duration: float
) -> None:
    path = Path(poses=[Pose(yaw=start_yaw), Pose(x=distance, yaw=end_yaw)])
    traj = get_constant_speed_trajectory(
        path, linear_velocity=0.5, max_angular_velocity=np.pi / 8
    )

    assert traj is not None
    assert traj.t_pts == pytest.approx([0.0, duration])
    assert np.all(traj.poses == path.poses)


def test_get_constant_speed_trajectory_limited_ang_vel() -> None:
    path = Path(
        poses=[
            Pose(x=0.0, y=0.0),
            Pose(x=1.0, y=0.0),
            Pose(x=1.0, y=1.0, yaw=np.pi / 2.0),
            Pose(x=0.0, y=1.0, yaw=-3.0 * np.pi / 4.0),
        ],
    )
    traj = get_constant_speed_trajectory(
        path,
        linear_velocity=0.5,
        max_angular_velocity=np.pi / 8,
    )

    assert traj.num_points() == 4
    assert traj.t_pts == pytest.approx([0.0, 2.0, 6.0, 12.0], rel=1e-4)
    assert np.all(traj.poses == path.poses)


def test_interpolate_trajectory() -> None:
    path = Path(
        poses=[
            Pose(x=0.0, y=0.0),
            Pose(x=1.0, y=0.0),
            Pose(x=1.0, y=1.0, yaw=np.pi / 2.0),
            Pose(x=0.0, y=1.0, yaw=-3.0 * np.pi / 4.0),
        ],
    )
    traj = get_constant_speed_trajectory(path, linear_velocity=1.0)
    interpolated_traj = interpolate_trajectory(traj, dt=0.1)

    assert interpolated_traj.num_points() == 31


def test_interpolate_trajectory_duplicate_points(caplog: LogCaptureFixture) -> None:
    path = Path(
        poses=[
            Pose(x=0.0, y=0.0),
            Pose(x=0.0, y=0.0),  # Duplicate
            Pose(x=1.0, y=0.0),
            Pose(x=1.0, y=1.0),  # Duplicate
            Pose(x=1.0, y=1.0, yaw=np.pi / 2.0),
            Pose(x=0.0, y=1.0, yaw=-3.0 * np.pi / 4.0),
            Pose(x=0.0, y=1.0, yaw=-3.0 * np.pi / 4.0),  # Duplicate
        ],
    )
    traj = get_constant_speed_trajectory(path, linear_velocity=1.0)
    interpolated_traj = interpolate_trajectory(traj, dt=0.1)
    assert "De-duplicated trajectory points at the same time." in caplog.text
    assert interpolated_traj.num_points() == 31


def test_interpolate_trajectory_insufficient_points(caplog: LogCaptureFixture) -> None:
    traj = Trajectory([1.0], [Pose()])
    interpolated_traj = interpolate_trajectory(traj, dt=0.1)
    assert interpolated_traj is None
    assert "Insufficient trajectory points for interpolation." in caplog.text


@pytest.mark.parametrize("start_time", [0.0, 1.0, -2.0])
@pytest.mark.parametrize("dt", [0.25, 0.3])
def test_interpolate_trajectory_time_range(start_time: float, dt: float) -> None:
    traj = Trajectory(
        [start_time, start_time + 1.0],
        [Pose(), Pose(x=1.0, yaw=np.pi / 2)],
    )
    interpolated_traj = interpolate_trajectory(traj, dt)

    assert interpolated_traj is not None
    expected_times = np.append(
        np.arange(start_time, start_time + 1.0, dt), start_time + 1.0
    )
    assert interpolated_traj.t_pts == pytest.approx(expected_times)
    assert interpolated_traj.num_points() == len(expected_times)
    for t, pose in zip(interpolated_traj.t_pts, interpolated_traj.poses):
        fraction = t - start_time
        assert pose.x == pytest.approx(fraction)
        assert pose.get_yaw() == pytest.approx(fraction * np.pi / 2)
    assert traj.t_pts == pytest.approx([start_time, start_time + 1.0])


def test_interpolate_trajectory_after_deleting_first_point() -> None:
    traj = Trajectory([0.0, 1.0, 2.0], [Pose(), Pose(x=1.0), Pose(x=2.0)])
    assert traj.delete(0)

    interpolated_traj = interpolate_trajectory(traj, dt=0.5)

    assert interpolated_traj is not None
    assert interpolated_traj.t_pts == pytest.approx([1.0, 1.5, 2.0])
    assert [pose.x for pose in interpolated_traj.poses] == pytest.approx(
        [1.0, 1.5, 2.0]
    )


def test_interpolate_trajectory_single_time_after_deduplication() -> None:
    pose = Pose(x=1.0, yaw=np.pi / 2)
    traj = Trajectory([1.0, 1.0, 1.0], [pose, pose, pose])

    interpolated_traj = interpolate_trajectory(traj, dt=0.1)

    assert interpolated_traj is not None
    assert interpolated_traj.t_pts == pytest.approx([1.0])
    assert interpolated_traj.num_points() == 1
    assert interpolated_traj.poses[0].is_approx(pose)
    assert traj.num_points() == 3
