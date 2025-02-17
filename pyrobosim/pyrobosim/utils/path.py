"""
Path representation for motion planning.
"""

from typing_extensions import Self  # For compatibility with Python <= 3.10

from .pose import Pose


class Path:
    """Representation of a path for motion planning."""

    def __init__(
        self, poses: list[Pose] = [], planning_time: float | None = None
    ) -> None:
        """
        Creates a Path object instance.

        :param poses: List of poses representing a path.
        :param planning_time: The time taken to generate this path.
        """
        self.set_poses(poses)
        self.planning_time = planning_time

    def set_poses(self, poses: list[Pose]) -> None:
        """
        Sets the list of poses and computes derived quantities.
        Use this method to change the poses of an existing path,
        rather than directly assigning the `poses` attribute.

        :param poses: List of poses representing a path.
        """
        self.poses = poses
        self.num_poses = len(self.poses)
        self.length = 0.0
        for i in range(self.num_poses - 1):
            self.length += self.poses[i].get_linear_distance(self.poses[i + 1])

    def fill_yaws(self) -> None:
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

    def __eq__(self, other: object) -> bool:
        """
        Check if two paths are exactly equal.

        :param other: Path with which to check equality.
        :return: True if the paths are equal, else False
        """
        if not (isinstance(other, Path)):
            raise TypeError("Expected a Path object.")

        return (self.poses == other.poses) and (self.length == other.length)

    def __repr__(self) -> str:
        """Return brief description of the path."""
        print_str = f"Path with {self.num_poses} points, Length {self.length:.3f}"
        return print_str

    def print_details(self) -> None:
        """Print detailed description of the path."""
        print_str = f"Path with {self.num_poses} points."
        for i, p in enumerate(self.poses):
            print_str += f"\n  {i + 1}. {p}"
        print_str += f"\nTotal Length: {self.length:.3f}"
        if self.planning_time:
            if self.planning_time > 0.01:
                print_str += f"\nPlanning time: {self.planning_time:3f} seconds"
            else:
                print_str += (
                    f"\nPlanning time: {self.planning_time * 1000.0:3f} milliseconds"
                )
        print(print_str)
