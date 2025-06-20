"""
Motion planning utilities that require a world instance.

This is mostly to avoid circular imports.
"""

from ..core.hallway import Hallway
from ..core.world import World
from ..utils.pose import Pose


def reduce_waypoints_polygon(
    world: World,
    poses: list[Pose],
    step_dist: float = 0.01,
    fog_hallways: bool = False,
    recorded_closed_hallways: set[Hallway] | None = None,
) -> list[Pose]:
    """
    Reduces the number of waypoints in a path generated from a polygon based planner.

    :param world: The world object in which the path is generated.
    :param poses: The list of poses that make up the path.
    :param step_dist: The step size for discretizing a straight line to check collisions.
    :param fog_hallways: If True, connectability is checked based on recorded knowledge, instead of ground truth.
    :param recorded_closed_hallways: Recorded knowledge of hallway states.
    """
    waypoints = []
    start = poses[0]
    waypoints.append(start)
    poses = poses[1:]
    i = len(poses) - 1
    while poses and i >= 0:
        current = poses[i]
        if world.is_connectable(
            start=start,
            goal=current,
            step_dist=step_dist,
            fog_hallways=fog_hallways,
            recorded_closed_hallways=recorded_closed_hallways,
        ):
            waypoints.append(current)
            start = current
            poses = poses[i + 1 :]
            i = len(poses) - 1
        else:
            i -= 1
    return waypoints
