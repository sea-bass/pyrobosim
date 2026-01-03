#!/usr/bin/env python3

"""Unit tests for the RRT planner"""

import numpy as np
from pytest import LogCaptureFixture

from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.utils.pose import Pose
from test.conftest import WorldFactoryProtocol


def test_rrt_long_distance(world: WorldFactoryProtocol) -> None:
    """Tests planning with default world graph planner settings."""

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


def test_rrt_short_distance_connect(world: WorldFactoryProtocol) -> None:
    """Tests if direct connection works if goal is within max_connection_distance."""
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


def test_rrt_no_path(caplog: LogCaptureFixture, world: WorldFactoryProtocol) -> None:
    """Test that RRT gracefully returns when there is no feasible path."""

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


def test_rrt_bidirectional(world: WorldFactoryProtocol) -> None:
    """Tests bidirectional RRT planning."""

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


def test_rrt_connect(world: WorldFactoryProtocol) -> None:
    """Tests RRTConnect planning."""

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


def test_rrt_star(world: WorldFactoryProtocol) -> None:
    """Tests RRT* planning."""
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


def test_rrt_compress_path(world: WorldFactoryProtocol) -> None:
    """Tests planning with path compression option."""
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
