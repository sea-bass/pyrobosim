""" Trajectory generation and interpolation utilities. """

import copy
import numpy as np
from scipy.spatial.transform import Slerp, Rotation
import warnings

from .pose import Pose, wrap_angle


class Trajectory:
    """Representation of a trajectory for motion planning."""

    def __init__(
        self,
        t_pts=[],
        poses=[],
    ):
        """
        Creates a Trajectory object instance.

        :param t_pts: List of time trajectory points, in seconds.
        :type t_pts: list[float]
        :param poses: List of poses that make up the trajectory.
        :type poses: list[:class:`pyrobosim.utils.pose.Pose`]
        """
        if len(t_pts) != len(poses):
            raise ValueError(
                "Time points and poses must have the same number of elements."
            )

        self.t_pts = np.array(t_pts)
        self.poses = np.array(poses)

    def num_points(self):
        """
        Returns the number of points in a trajectory.

        :return: The number of trajectory points.
        :rtype: int
        """
        return len(self.t_pts)

    def is_empty(self):
        """
        Checks whether a trajectory is empty.

        :return: True if the trajectory is empty, otherwise False.
        :rtype: bool
        """
        return self.num_points() == 0

    def delete(self, idx):
        """
        Deletes a trajectory point at the specified index.

        :param idx: The index
        :type idx: int
        :return: True if the point was successfully deleted, otherwise False.
        :rtype: bool
        """
        if self.is_empty():
            warnings.warn("Trajectory is empty. Cannot delete point.")
            return False
        elif idx < 0 or idx >= self.num_points():
            warnings.warn(
                f"Invalid index {idx} for trajectory length {self.num_points()}"
            )
            return False
        else:
            self.t_pts = np.delete(self.t_pts, idx)
            self.poses = np.delete(self.poses, idx)
            return True


def get_constant_speed_trajectory(path, linear_velocity=0.2, max_angular_velocity=None):
    """
    Gets a trajectory from a path (list of Pose objects) by calculating
    time points based on constant velocity and maximum angular velocity.

    :param path: Path object from which to compute a trajectory.
    :type path: :class:`pyrobosim.utils.motion.Path`
    :param linear_velocity: Constant linear velocity in m/s, defaults to 0.2.
    :type linear_velocity: float
    :param max_angular_velocity: Maximum angular velocity in rad/s,
        defaults to None.
    :type max_angular_velocity: float, optional
    :return: Constant speed trajectory.
    :rtype: :class:`pyrobosim.utils.trajectory.Trajectory`
    """
    if path.num_poses < 2:
        warnings.warn("Insufficient points to generate trajectory.")
        return None

    # Calculate the time points for the path at constant velocity, also
    # accounting for the maximum angular velocity if specified.
    t_pts = np.zeros_like(path.poses, dtype=np.float64)
    for idx in range(path.num_poses - 1):
        start_pose = path.poses[idx]
        end_pose = path.poses[idx + 1]
        lin_time = start_pose.get_linear_distance(end_pose) / linear_velocity
        if max_angular_velocity is None:
            ang_time = 0
        else:
            ang_distance = wrap_angle(end_pose.get_yaw() - start_pose.get_yaw())
            ang_time = ang_distance / max_angular_velocity
        t_pts[idx + 1] = t_pts[idx] + max(lin_time, ang_time)

    return Trajectory(t_pts, path.poses)


def interpolate_trajectory(traj: Trajectory, dt: float):
    """
    Interpolates a trajectory given a time step `dt`.
    Positions are interpolated linearly and the angle is interpolated
    using the Spherical Linear Interpolation (Slerp) method.

    :param traj: Input trajectory.
    :type traj: :class:`pyrobosim.utils.trajectory.Trajectory`
    :param dt: Trajectory sample time, in seconds.
    :type dt: float
    :return: Interpolated trajectory
    :rtype: :class:`pyrobosim.utils.trajectory.Trajectory`
    """
    if traj.num_points() < 2:
        warnings.warn("Insufficient trajectory points for interpolation.")
        return None

    if traj.num_points() == 2 and traj.poses[0] == traj.poses[1]:
        return traj

    # De-duplicate time points ensure that Slerp doesn't throw an error.
    # Right now, we're just keeping the later point.
    i = 1
    modified_traj = copy.deepcopy(traj)
    while i < modified_traj.num_points():
        if modified_traj.t_pts[i] <= modified_traj.t_pts[i - 1]:
            warnings.warn("De-duplicated trajectory points at the same time.")
            modified_traj.delete(i - 1)
        else:
            i += 1

    t_final = modified_traj.t_pts[-1]
    t_interp = np.arange(0, t_final, dt)

    # Interpolate the translation elements linearly
    if t_final not in t_interp:
        t_interp = np.append(t_interp, t_final)
    x_interp = np.interp(
        t_interp, modified_traj.t_pts, [pose.x for pose in modified_traj.poses]
    )
    y_interp = np.interp(
        t_interp, modified_traj.t_pts, [pose.y for pose in modified_traj.poses]
    )
    z_interp = np.interp(
        t_interp, modified_traj.t_pts, [pose.z for pose in modified_traj.poses]
    )

    # Set up Slerp interpolation for the angle.
    if t_final > 0:
        euler_angs = [pose.eul for pose in modified_traj.poses]
        slerp = Slerp(modified_traj.t_pts, Rotation.from_euler("xyz", euler_angs))
        eul_interp = [slerp(t).as_euler("xyz", degrees=False) for t in t_interp]
    else:
        eul_interp = [modified_traj.poses[-1].eul]

    # Package up the interpolated trajectory
    poses_interp = [
        Pose(x=x, y=y, z=z, roll=eul[0], pitch=eul[1], yaw=eul[2])
        for x, y, z, eul in zip(x_interp, y_interp, z_interp, eul_interp)
    ]
    return Trajectory(t_interp, poses_interp)
