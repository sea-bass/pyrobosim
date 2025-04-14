#!/usr/bin/env python3

"""Unit tests for the PRM planner"""

import os
import numpy as np
from pytest import LogCaptureFixture

from pyrobosim.core import WorldYamlLoader
from pyrobosim.navigation.prm import PRMPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


def test_prm_default() -> None:
    """Tests planning with default world graph planner settings."""
    world = WorldYamlLoader().from_file(
        os.path.join(get_data_folder(), "test_world.yaml")
    )

    np.random.seed(1234)  # Fix seed for reproducibility
    prm = PRMPlanner()
    world.robots[0].set_path_planner(prm)

    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=2.5, y=3.0)
    path = prm.plan(start, goal)

    assert len(path.poses) >= 2
    assert path.poses[0] == start
    assert path.poses[-1] == goal


def test_prm_no_path(caplog: LogCaptureFixture) -> None:
    """Test that PRM gracefully returns when there is no feasible path."""
    world = WorldYamlLoader().from_file(
        os.path.join(get_data_folder(), "test_world.yaml")
    )

    prm = PRMPlanner()
    world.robots[0].set_path_planner(prm)

    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=12.5, y=3.0)
    path = prm.plan(start, goal)

    assert len(path.poses) == 0
    assert "Could not find a path from start to goal." in caplog.text


def test_prm_compress_path() -> None:
    """Tests planning with path compression option."""
    world = WorldYamlLoader().from_file(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    planner_config = {"compress_path": False}
    prm = PRMPlanner(**planner_config)
    world.robots[0].set_path_planner(prm)

    np.random.seed(1234)  # Use same seed for reproducibility
    start = Pose(x=-0.3, y=0.6)
    goal = Pose(x=2.5, y=3.0)
    orig_path = prm.plan(start, goal)
    assert len(orig_path.poses) >= 2

    np.random.seed(1234)  # Use same seed for reproducibility
    prm.compress_path = True
    new_path = prm.plan(start, goal)
    assert len(new_path.poses) >= 2
    assert new_path.length < orig_path.length
