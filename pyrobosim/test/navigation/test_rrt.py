#!/usr/bin/env python3

"""Unit tests for the RRT planner"""

import os
import numpy as np
from pytest import LogCaptureFixture

from pyrobosim.core import WorldYamlLoader
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


def test_rrt_long_distance() -> None:
    """Tests planning with default world graph planner settings."""
    world = WorldYamlLoader().from_file(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "bidirectional": False,
        "rrt_connect": False,
        "rrt_star": False,
    }
    rrt = RRTPlanner(**planner_config)
    world.robots[0].set_path_planner(rrt)

    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=2.5, y=3.0)
    path = rrt.plan(start, goal)

    assert len(path.poses) >= 2
    assert path.poses[0] == start
    assert path.poses[-1] == goal


def test_rrt_short_distance_connect() -> None:
    """Tests if direct connection works if goal is within max_connection_distance."""
    world = WorldYamlLoader().from_file(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "bidirectional": False,
        "rrt_connect": False,
        "rrt_star": False,
    }
    rrt = RRTPlanner(**planner_config)
    world.robots[0].set_path_planner(rrt)

    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=-1.6, y=3.0)
    path = rrt.plan(start, goal)

    assert len(path.poses) == 2
    assert path.poses[0] == start
    assert path.poses[1] == goal


def test_rrt_no_path(caplog: LogCaptureFixture) -> None:
    """Test that RRT gracefully returns when there is no feasible path."""
    world = WorldYamlLoader().from_file(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "max_time": 0.5,  # To make the test fail more quickly.
    }

    rrt = RRTPlanner(**planner_config)
    world.robots[0].set_path_planner(rrt)

    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=12.5, y=3.0)
    path = rrt.plan(start, goal)

    assert len(path.poses) == 0
    assert "Could not find a path from start to goal." in caplog.text


def test_rrt_bidirectional() -> None:
    """Tests bidirectional RRT planning."""
    world = WorldYamlLoader().from_file(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "bidirectional": True,
        "rrt_connect": False,
        "rrt_star": False,
    }
    rrt = RRTPlanner(**planner_config)
    world.robots[0].set_path_planner(rrt)

    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=2.5, y=3.0)
    path = rrt.plan(start, goal)

    assert len(path.poses) >= 2
    assert path.poses[0] == start
    assert path.poses[-1] == goal


def test_rrt_connect() -> None:
    """Tests RRTConnect planning."""
    world = WorldYamlLoader().from_file(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "bidirectional": False,
        "rrt_connect": True,
        "rrt_star": False,
    }
    rrt = RRTPlanner(**planner_config)
    world.robots[0].set_path_planner(rrt)

    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=2.5, y=3.0)
    path = rrt.plan(start, goal)

    assert len(path.poses) >= 2
    assert path.poses[0] == start
    assert path.poses[-1] == goal


def test_rrt_star() -> None:
    """Tests RRT* planning."""
    world = WorldYamlLoader().from_file(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "bidirectional": False,
        "rrt_connect": False,
        "rrt_star": True,
    }
    rrt = RRTPlanner(**planner_config)
    world.robots[0].set_path_planner(rrt)

    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=2.5, y=3.0)
    path = rrt.plan(start, goal)

    assert len(path.poses) >= 2
    assert path.poses[0] == start
    assert path.poses[-1] == goal


def test_rrt_compress_path() -> None:
    """Tests planning with path compression option."""
    world = WorldYamlLoader().from_file(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {
        "bidirectional": False,
        "rrt_connect": False,
        "rrt_star": False,
        "compress_path": False,
    }
    rrt = RRTPlanner(**planner_config)
    world.robots[0].set_path_planner(rrt)

    start = Pose(x=-0.3, y=0.6)
    goal = Pose(x=2.5, y=3.0)

    np.random.seed(1234)  # Use same seed for reproducibility
    orig_path = rrt.plan(start, goal)
    assert len(orig_path.poses) >= 2

    np.random.seed(1234)  # Use same seed for reproducibility
    rrt.compress_path = True
    new_path = rrt.plan(start, goal)
    assert len(new_path.poses) >= 2
    assert new_path.length < orig_path.length
