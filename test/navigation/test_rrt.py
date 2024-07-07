#!/usr/bin/env python3

"""Unit tests for the RRT planner"""

import os
import numpy as np
import pytest

from pyrobosim.core import WorldYamlLoader
from pyrobosim.navigation import OccupancyGrid, PathPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


def test_rrt_long_distance():
    """Tests planning with default world graph planner settings."""
    world = WorldYamlLoader().from_yaml(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "world": world,
        "bidirectional": False,
        "rrt_connect": False,
        "rrt_star": False,
    }
    rrt = PathPlanner("rrt", **planner_config)
    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=2.5, y=3.0)

    path = rrt.plan(start, goal)
    assert len(path.poses) >= 2
    assert path.poses[0] == start
    assert path.poses[-1] == goal


def test_rrt_short_distance_connect():
    """Tests if direct connection works if goal is within max_connection_distance."""
    world = WorldYamlLoader().from_yaml(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "world": world,
        "bidirectional": False,
        "rrt_connect": False,
        "rrt_star": False,
    }
    rrt = PathPlanner("rrt", **planner_config)
    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=-1.6, y=3.0)

    path = rrt.plan(start, goal)
    assert len(path.poses) == 2
    assert path.poses[0] == start
    assert path.poses[1] == goal


def test_rrt_no_path():
    """Test that RRT gracefully returns when there is no feasible path."""
    world = WorldYamlLoader().from_yaml(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "world": world,
        "max_time": 0.5,  # To make the test fail more quickly.
    }

    rrt = PathPlanner("rrt", **planner_config)
    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=12.5, y=3.0)

    with pytest.warns(UserWarning) as warn_info:
        path = rrt.plan(start, goal)
        assert len(path.poses) == 0
    assert warn_info[0].message.args[0] == "Could not find a path from start to goal."


def test_rrt_bidirectional():
    """Tests bidirectional RRT planning."""
    world = WorldYamlLoader().from_yaml(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "world": world,
        "bidirectional": True,
        "rrt_connect": False,
        "rrt_star": False,
    }
    rrt = PathPlanner("rrt", **planner_config)
    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=2.5, y=3.0)

    path = rrt.plan(start, goal)
    assert len(path.poses) >= 2
    assert path.poses[0] == start
    assert path.poses[-1] == goal


def test_rrt_connect():
    """Tests RRTConnect planning."""
    world = WorldYamlLoader().from_yaml(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "world": world,
        "bidirectional": False,
        "rrt_connect": True,
        "rrt_star": False,
    }
    rrt = PathPlanner("rrt", **planner_config)
    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=2.5, y=3.0)

    path = rrt.plan(start, goal)
    assert len(path.poses) >= 2
    assert path.poses[0] == start
    assert path.poses[-1] == goal


def test_rrt_star():
    """Tests RRT* planning."""
    world = WorldYamlLoader().from_yaml(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "world": world,
        "bidirectional": False,
        "rrt_connect": False,
        "rrt_star": True,
    }
    rrt = PathPlanner("rrt", **planner_config)
    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=2.5, y=3.0)

    path = rrt.plan(start, goal)
    assert len(path.poses) >= 2
    assert path.poses[0] == start
    assert path.poses[-1] == goal


def test_rrt_grid_not_supported():
    """Test that RRT is not (yet) supported with occupancy grid maps."""
    world = WorldYamlLoader().from_yaml(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    robot = world.robots[0]
    planner_config = {
        "grid": OccupancyGrid.from_world(
            world, resolution=0.05, inflation_radius=1.5 * robot.radius
        ),
    }

    with pytest.raises(NotImplementedError) as exc_info:
        PathPlanner("rrt", **planner_config)
    assert exc_info.value.args[0] == "RRT planner does not support grid based search."


def test_rrt_compress_path():
    """Tests planning with path compression option."""
    world = WorldYamlLoader().from_yaml(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "world": world,
        "bidirectional": False,
        "rrt_connect": False,
        "rrt_star": False,
        "compress_path": False,
    }
    rrt = PathPlanner("rrt", **planner_config)
    start = Pose(x=-0.3, y=0.6)
    goal = Pose(x=2.5, y=3.0)

    np.random.seed(1234)  # Use same seed for reproducibility
    orig_path = rrt.plan(start, goal)
    assert len(orig_path.poses) >= 2

    np.random.seed(1234)  # Use same seed for reproducibility
    rrt.planner.impl.compress_path = True
    new_path = rrt.plan(start, goal)
    assert len(new_path.poses) >= 2
    assert new_path.length < orig_path.length
