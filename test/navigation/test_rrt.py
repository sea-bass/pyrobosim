#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Unit tests for the planners"""

import os
import pytest

from pyrobosim.utils.pose import Pose
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.core.yaml import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder

data_folder = get_data_folder()
loader = WorldYamlLoader()


def test_rrt_short_distance_connect():
    """Tests if direct connection works if goal is within max_connection_distance"""

    w = loader.from_yaml(os.path.join(data_folder, "test_world.yaml"))
    rrt = RRTPlanner(w, bidirectional=False, rrt_connect=False, rrt_star=False)
    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=-1.6, y=3.0)

    path = rrt.plan(start, goal)
    assert len(path.poses) == 2
    assert path.poses[0] == start
    assert path.poses[1] == goal
