"""
Motion planning utilities.
"""

from ..core import World
from ..navigation.occupancy_grid import OccupancyGrid


def reduce_waypoints_grid(
    grid: OccupancyGrid, positions: list[tuple[int, int]]
) -> list[tuple[int, int]]:
    """
    Reduces the number of waypoints in a generated path from a grid-based planner.

    :param grid: The occupancy grid associated with the generated path.
    :type grid: :class:`pyrobosim.navigation.occupancy_grid.OccupancyGrid`
    :param poses: The list of positions that make up the path.
    :return: The optimized list of waypoints.
    """

    waypoints = []
    start = positions[0]
    waypoints.append(start)
    positions = positions[1:]
    i = len(positions) - 1
    while positions and i >= 0:
        current = positions[i]
        if grid.has_straight_line_connection(start, current)[0]:
            waypoints.append(current)
            start = current
            positions = positions[i + 1 :]
            i = len(positions) - 1
        else:
            i -= 1
    return waypoints


def reduce_waypoints_polygon(world, poses, step_dist=0.01):
    """
    Reduces the number of waypoints in a path generated from a polygon based planner.

    :param world: The world object in which the path is generated.
    :type world: :class:`pyrobosim.core.world.World`
    :param poses: The list of poses that make up the path.
    :type poses: list[:class: `pyrobosim.utils.pose.Pose`]
    :param step_dist: The step size for discretizing a straight line to check collisions.
    :type step_dist: float
    :param max_dist: The maximum allowed connection distance.
    :type max_dist: float, optional
    """
    waypoints = []

    start = poses[0]
    waypoints.append(start)
    poses = poses[1:]
    i = len(poses) - 1
    while poses and i >= 0:
        current = poses[i]
        if world.is_connectable(start, current, step_dist):
            waypoints.append(current)
            start = current
            poses = poses[i + 1 :]
            i = len(poses) - 1
        else:
            i -= 1
    return waypoints
