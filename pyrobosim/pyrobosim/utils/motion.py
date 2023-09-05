"""
Motion planning utilities.
"""


class Path:
    """Representation of a path for motion planning."""

    def __init__(self, poses=[]):
        """
        Creates a Path object instance.

        :param poses: List of poses representing a path.
        :type poses: list[:class:`pyrobosim.utils.pose.Pose`], optional
        """
        self.set_poses(poses)

    def set_poses(self, poses):
        """
        Sets the list of poses and computes derived quantities.
        Use this method to change the poses of an existing path,
        rather than directly assigning the `poses` attribute.

        :param poses: List of poses representing a path.
        :type poses: list[:class:`pyrobosim.utils.pose.Pose`], optional
        """
        self.poses = poses
        self.num_poses = len(self.poses)
        self.length = 0.0
        for i in range(self.num_poses - 1):
            self.length += self.poses[i].get_linear_distance(self.poses[i + 1])

    def fill_yaws(self):
        """
        Fills in any yaw angles along a path to point at the next waypoint.
        """
        if self.num_poses < 1:
            return

        for idx in range(1, self.num_poses - 1):
            cur_pose = self.poses[idx]
            prev_pose = self.poses[idx - 1]
            yaw = prev_pose.get_angular_distance(cur_pose)
            cur_pose.set_euler_angles(yaw=yaw)

    def __eq__(self, other):
        """
        Check if two paths are exactly equal.

        :param other: Path with which to check equality.
        :type other: :class:`pyrobosim.utils.motion.Path`
        :return: True if the paths are equal, else False
        :rtype: bool
        """
        if not (isinstance(other, Path)):
            raise TypeError("Expected a Path object")

        return self.poses == other.poses

    def __repr__(self):
        """Return brief description of the path."""
        print_str = f"Path with {self.num_poses} points, Length {self.length:.3f}"
        return print_str

    def print_details(self):
        """Print detailed description of the path."""
        print_str = f"Path with {self.num_poses} points."
        for i, p in enumerate(self.poses):
            print_str += f"\n  {i + 1}. {p}"
        print_str += f"\nTotal Length: {self.length:.3f}"
        print(print_str)


def reduce_waypoints_grid(grid, positions):
    """
    Reduces the number of waypoints in a generated path from a grid-based planner.

    :param grid: The occupancy grid associated with the generated path.
    :type grid: :class:`pyrobosim.navigation.occupancy_grid.OccupancyGrid`
    :param poses: The list of positions that make up the path.
    :type poses: list[(int, int)]
    :return: The optimized list of waypoints.
    :rtype: list[(int, int)]
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
