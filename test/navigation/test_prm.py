#!/usr/bin/env python3

"""Unit tests for the PRM planner"""

import os
import numpy as np
import pytest

from pyrobosim.core import WorldYamlLoader
from pyrobosim.navigation import OccupancyGrid, PathPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


def test_prm_default():
    """Tests planning with default world graph planner settings."""
    world = WorldYamlLoader().from_yaml(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {"world": world}

    np.random.seed(1234)  # Fix seed for reproducibility
    prm = PathPlanner("prm", **planner_config)
    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=2.5, y=3.0)

    path = prm.plan(start, goal)
    assert len(path.poses) >= 2
    assert path.poses[0] == start
    assert path.poses[-1] == goal


def test_prm_default():
    """Tests planning with default world graph planner settings."""
    world = WorldYamlLoader().from_yaml(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {"world": world}

    np.random.seed(1234)  # Fix seed for reproducibility
    prm = PathPlanner("prm", **planner_config)
    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=2.5, y=3.0)

    path = prm.plan(start, goal)
    assert len(path.poses) >= 2
    assert path.poses[0] == start
    assert path.poses[-1] == goal


def test_prm_no_path():
    """Test that PRM gracefully returns when there is no feasible path."""
    world = WorldYamlLoader().from_yaml(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {"world": world}

    prm = PathPlanner("prm", **planner_config)
    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=12.5, y=3.0)

    with pytest.warns(UserWarning) as warn_info:
        path = prm.plan(start, goal)
        assert len(path.poses) == 0
    assert warn_info[0].message.args[0] == "Could not find a path from start to goal."


def test_rrt_grid_not_supported():
    """Test that PRM is not (yet) supported with occupancy grid maps."""
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
        PathPlanner("prm", **planner_config)
    assert exc_info.value.args[0] == "PRM planner does not support grid based search."
