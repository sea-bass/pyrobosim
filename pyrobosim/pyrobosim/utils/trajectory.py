""" Trajectory generation and interpolation utilities. """

import copy
import numpy as np
from scipy.spatial.transform import Slerp, Rotation
import warnings

from .pose import wrap_angle


class Trajectory:
    """Representation of a trajectory for motion planning."""

    def __init__(
        self,
        t_pts=np.array([]),
        x_pts=np.array([]),
        y_pts=np.array([]),
        yaw_pts=np.array([]),
    ):
        """
        Creates a Trajectory object instance

        :param t_pts: Array of time trajectory points, in seconds.
        :type t_pts: :class:`numpy.array`
        :param x_pts: Array of X translation trajectory points, in meters.
        :type x_pts: :class:`numpy.array`
        :param y_pts: Array of Y translation trajectory points, in meters.
        :type y_pts: :class:`numpy.array`
        :param yaw_pts: Array of yaw trajectory points, in radians.
        :type yaw_pts: :class:`numpy.array`
        """
        num_pts = len(t_pts)
        if len(x_pts) != num_pts or len(y_pts) != num_pts or len(yaw_pts) != num_pts:
            raise ValueError("All point arrays must have the same number of elements.")

        self.t_pts = t_pts
        self.x_pts = x_pts
        self.y_pts = y_pts
        self.yaw_pts = yaw_pts

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
        """
        if len(self.t_pts) == 0:
            warnings.warn("Trajectory is empty. Cannot delete point.")
            return
        elif idx < 0 or idx >= self.num_points():
            warnings.warn(
                f"Invalid index {idx} for trajectory length {self.num_points()}"
            )
            return
        else:
            self.t_pts = np.delete(self.t_pts, idx)
            self.x_pts = np.delete(self.x_pts, idx)
            self.y_pts = np.delete(self.y_pts, idx)
            self.yaw_pts = np.delete(self.yaw_pts, idx)


def get_constant_speed_trajectory(path, linear_velocity=0.2, max_angular_velocity=None):
    """
    Gets a trajectory from a path (list of Pose objects) by calculating
    time points based on constant velocity and maximum angular velocity.

    The trajectory is returned as a tuple of numpy arrays
    (t_pts, x_pts, y_pts, theta_pts).

    :param path: Path object from which to compute a trajectory.
    :type path: :class:`pyrobosim.utils.motion.Path`
    :param linear_velocity: Constant linear velocity in m/s, defaults to 0.2.
    :type linear_velocity: float
    :param max_angular_velocity: Maximum angular velocity in rad/s,
        defaults to None.
    :type max_angular_velocity: float, optional
    :return: Constant speed trajectory.
    :rtype: tuple(:class:`pyrobosim.utils.trajectory.Trajectory`)
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

    # Package up the trajectory
    x_pts = np.array([p.x for p in path.poses])
    y_pts = np.array([p.y for p in path.poses])
    yaw_pts = np.array([p.get_yaw() for p in path.poses])
    return Trajectory(t_pts, x_pts, y_pts, yaw_pts)


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

    # De-duplicate time points ensure that Slerp doesn't throw an error.
    # Right now, we're just keeping the later point.
    i = 0
    modified_traj = copy.deepcopy(traj)
    while i < modified_traj.num_points():
        if (i > 0) and (modified_traj.t_pts[i] <= modified_traj.t_pts[i - 1]):
            warnings.warn("De-duplicated trajectory points at the same time.")
            modified_traj.delete(i - 1)
        else:
            i += 1

    # Set up Slerp interpolation for the angle.
    t_final = modified_traj.t_pts[-1]
    if t_final > 0:
        euler_angs = [[0, 0, th] for th in modified_traj.yaw_pts]
        slerp = Slerp(modified_traj.t_pts, Rotation.from_euler("xyz", euler_angs))

    # Package up the interpolated trajectory
    t_interp = np.arange(0, t_final, dt)
    if t_final not in t_interp:
        t_interp = np.append(t_interp, t_final)
    x_interp = np.interp(t_interp, modified_traj.t_pts, modified_traj.x_pts)
    y_interp = np.interp(t_interp, modified_traj.t_pts, modified_traj.y_pts)
    if t_final > 0:
        yaw_interp = np.array(
            [slerp(t).as_euler("xyz", degrees=False)[2] for t in t_interp]
        )
    else:
        yaw_interp = np.array([modified_traj.yaw_pts[-1]])

    return Trajectory(t_interp, x_interp, y_interp, yaw_interp)
