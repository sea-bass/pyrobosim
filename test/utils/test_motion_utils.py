#!/usr/bin/env python3

"""
Unit tests for motion planning utilities.
"""

import numpy as np
import pytest

from pyrobosim.core import Pose
from pyrobosim.utils.motion import Path, reduce_waypoints_grid, reduce_waypoints_polygon


########################
# Tests for Path class #
########################
def test_path_default_args():
    """Test creating a path with default arguments."""
    path = Path()
    assert len(path.poses) == 0
    assert path.num_poses == 0
    assert path.length == 0.0


def test_path_pose_list():
    """Test creating a path with a list of poses."""
    path = Path(
        poses=[
            Pose(x=0.0, y=0.0),
            Pose(x=1.0, y=0.0, yaw=np.pi / 2.0),
            Pose(x=1.0, y=3.0, q=[0.707, 0.0, 0.707, 0.0]),
        ]
    )
    assert len(path.poses) == 3
    assert path.num_poses == 3
    assert path.length == pytest.approx(4.0)


def test_path_fill_yaws():
    """Test the utility to fill yaws to point along a path."""
    path = Path(
        poses=[
            Pose(x=0.0, y=0.0),  # Yaw should remain at its default value
            Pose(x=1.0, y=1.0),  # Yaw should be pi/4
            Pose(x=2.0, y=2.0),  # Yaw should be pi/4
            Pose(x=2.0, y=5.0),  # Yaw should be pi/2
            Pose(x=0.0, y=5.0),  # Yaw should be pi
            Pose(x=0.0, y=4.0),  # Yaw should remain at its final value
        ]
    )
    expected_yaws = [0.0, np.pi / 4.0, np.pi / 4.0, np.pi / 2.0, np.pi, 0.0]

    path.fill_yaws()
    for pose, expected_yaw in zip(path.poses, expected_yaws):
        assert pose.get_yaw() == pytest.approx(expected_yaw)


def test_path_equality():
    """Tests for equality of Path objects."""
    test_poses = [
        Pose(x=0.0, y=0.0),
        Pose(x=1.0, y=0.0, yaw=np.pi / 2.0),
        Pose(x=1.0, y=3.0, q=[0.707, 0.0, 0.707, 0.0]),
    ]

    path1 = Path(poses=test_poses)
    path2 = Path(poses=test_poses)  # Same poses, but different object
    path3 = Path(poses=list(reversed(test_poses)))  # Different poses

    assert path1 == path1
    assert path1 == path2
    assert not path1 == path3


##########################################
# Tests for waypoint reduction utilities #
##########################################
def test_reduce_waypoints_occupancy_grid():
    """Test utility to reduce waypoints using an occupancy grid."""
    from pyrobosim.navigation.occupancy_grid import OccupancyGrid

    grid_data = np.zeros((10, 10))
    grid_data[5, :8] = 1  # Creates a wall
    grid = OccupancyGrid(grid_data, 1.0, [0.0, 0.0])

    points = [
        [0, 0],  # Not optimized out since it's the start point
        [1, 1],  # Should get optimized out
        [3, 3],  # Should get optimized out
        [3, 9],  # Not optimized out because of the wall
        [7, 9],  # Should get optimized out
        [9, 9],  # Not optimized out since it's the goal point
    ]

    reduced_points = reduce_waypoints_grid(grid, points)
    assert len(reduced_points) == 3
    assert reduced_points == [[0, 0], [3, 9], [9, 9]]


def test_reduce_waypoints_polygon():
    """Test utility to reduce waypoints using polygons from a world model."""
    from pyrobosim.core import World

    world = World()
    world.add_room(name="room1", footprint=[(0, 0), (1, 0), (1, 1), (0, 1)])
    world.add_room(name="room2", footprint=[(2, 0), (3, 0), (3, 1), (2, 1)])
    world.add_hallway(room_start="room1", room_end="room2", width=0.25)

    poses = [
        Pose(x=0.1, y=0.1),  # Not optimized out since it's the start point
        Pose(x=0.3, y=0.3),  # Should get optimized out
        Pose(x=0.8, y=0.3),  # Should get optimized out
        Pose(
            x=0.8, y=0.5
        ),  # Not optimized out since it's needed to traverse the hallway
        Pose(x=1.8, y=0.5),  # Should get optimized out
        Pose(x=2.2, y=0.5),  # Not optimized out since it's needed to reach goal
        Pose(x=2.3, y=0.1),  # Not optimized out since it's the goal point
    ]

    reduced_poses = reduce_waypoints_polygon(world, poses, step_dist=0.01)
    assert len(reduced_poses) == 4
    assert reduced_poses == [poses[0], poses[3], poses[5], poses[6]]
