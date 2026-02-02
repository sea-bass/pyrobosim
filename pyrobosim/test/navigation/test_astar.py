#!/usr/bin/env python3

"""Unit tests for A* planner"""

from pyrobosim.navigation.a_star import AStarPlanner
from pyrobosim.core.world import World
from pyrobosim.utils.pose import Pose


def test_astar(test_world: World) -> None:
    """Test A* planner with and without path compression"""

    start = Pose(x=-1.6, y=2.8)
    goal = Pose(x=2.5, y=3.0)

    robot = test_world.robots[0]
    planner_config = {
        "grid_resolution": 0.05,
        "grid_inflation_radius": 1.5 * robot.radius,
        "diagonal_motion": True,
        "heuristic": "euclidean",
        "compress_path": False,
    }
    astar_planner = AStarPlanner(**planner_config)
    robot.set_path_planner(astar_planner)
    full_path = astar_planner.plan(start, goal).poses

    assert len(full_path) >= 2
    assert full_path[0] == start
    assert full_path[-1] == goal

    # Plan for same start and goal with path compression enabled
    planner_config = {
        "grid_resolution": 0.05,
        "grid_inflation_radius": 1.5 * robot.radius,
        "diagonal_motion": True,
        "heuristic": "euclidean",
        "compress_path": True,
    }
    astar_planner = AStarPlanner(**planner_config)

    robot.set_path_planner(astar_planner)
    astar_planner.reset()
    compressed_path = astar_planner.plan(start, goal).poses

    assert len(compressed_path) >= 2
    assert len(compressed_path) <= len(full_path)
    assert compressed_path[0] == start
    assert compressed_path[-1] == goal
