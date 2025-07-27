"""Collision and occupancy checking utilities that require a world and a robot instance."""

from __future__ import annotations

import numpy as np
from typing import Sequence, TYPE_CHECKING

import shapely

from .path import Path
from .pose import Pose

if TYPE_CHECKING:
    from ..core.robot import Robot
    from ..core.world import World


def is_connectable(
    start: Pose,
    goal: Pose,
    world: World,
    robot: Robot | None = None,
    step_dist: float = 0.01,
    max_dist: float | None = None,
) -> bool:
    """
    Checks connectivity between two poses `start` and `goal` in the world
    by sampling points spaced by the `collision_check_dist` parameter
    and verifying that every point is in the free configuration space.

    :param start: Start pose
    :param goal: Goal pose
    :param world: The world in which the poses are located.
    :param robot: The robot instance used for checking occupancy.
    :param step_dist: The step size for discretizing a straight line to check collisions.
    :param max_dist: The maximum allowed connection distance.
    :return: True if poses can be connected, else False.
    """
    # Trivial case where nodes are identical.
    if start == goal:
        return True

    # Check against the max edge distance.
    dist = start.get_linear_distance(goal, ignore_z=True)
    angle = start.get_angular_distance(goal)
    if max_dist and (dist > max_dist):
        return False

    # Build up the array of test X and Y coordinates for sampling between
    # the start and goal points.
    dist_array = np.arange(0, dist, step_dist)
    # If the nodes are coincident, connect them by default.
    if dist_array.size == 0:
        return True
    if dist_array[-1] != dist:
        np.append(dist_array, dist)
    x_pts = start.x + dist_array * np.cos(angle)
    y_pts = start.y + dist_array * np.sin(angle)

    # Check the occupancy of all the test points.
    for x_check, y_check in zip(x_pts[1:], y_pts[1:]):
        if check_occupancy(
            Pose(x=x_check, y=y_check),
            world,
            robot,
        ):
            return False

    # If the loop was traversed for all points without returning, we can
    # connect the points.
    return True


def check_occupancy(
    pose: Pose | Sequence[float],
    world: World,
    robot: Robot | None = None,
) -> bool:
    """
    Check if a pose in the world is occupied.
    If robot instance is provided, interpretation would be based on robot's perception of the world.

    :param pose: The pose to check.
    :param world: The world in which the pose is located.
    :param robot: The robot instance used for checking collision.
    :return: True if the pose is occupied, else False.
    """
    if isinstance(pose, Pose):
        x = pose.x
        y = pose.y
    else:
        x, y = pose

    if (robot is None) or (robot.total_internal_polygon.is_empty):
        polygon = world.total_internal_polygon
    else:
        polygon = robot.total_internal_polygon
    return not bool(shapely.intersects_xy(polygon, x, y))


def is_path_collision_free(
    path: Path,
    robot: Robot,
    step_dist: float = 0.01,
) -> bool:
    """
    Check whether a path is collision free in this world.

    :param path: The path to use for collision checking.
    :param robot: The robot instance used for collision checking.
    :param step_dist: The step size for discretizing a straight line to check collisions.
    :return: True if the path is collision free, else False.
    """
    if robot.world is None:
        robot.logger.error("Robot is not attached to World.")
        raise RuntimeError("Robot is not attached to World.")

    for idx in range(len(path.poses) - 1):
        if not is_connectable(
            path.poses[idx],
            path.poses[idx + 1],
            robot.world,
            robot,
            step_dist,
        ):
            return False
    return True
