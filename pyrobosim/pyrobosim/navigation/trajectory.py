""" Trajectory generation and interpolation utilities. """

import numpy as np
from scipy.spatial.transform import Slerp, Rotation
import warnings

from ..utils.pose import wrap_angle


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
    :return: Trajectory type of the form (t_pts, x_pts, y_pts, theta_pts).
    :rtype: tuple(:class:`numpy.array`)
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
    traj = (t_pts, x_pts, y_pts, yaw_pts)
    return traj


def interpolate_trajectory(traj, dt):
    """
    Interpolates a trajectory given a time step `dt`.
    Positions are interpolated linearly and the angle is interpolated
    using the Spherical Linear Interpolation (Slerp) method.

    :param traj: Trajectory type of the form (t_pts, x_pts, y_pts, theta_pts).
    :type traj: tuple(:class:`numpy.array`)
    :param dt: Trajectory sample time, in seconds.
    :type dt: float
    :return: Trajectory type of the form (t_pts, x_pts, y_pts, theta_pts).
    :rtype: tuple(:class:`numpy.array`)
    """
    # Unpack the trajectory
    (t_pts, x_pts, y_pts, yaw_pts) = traj
    if len(t_pts) < 2:
        warnings.warn("Insufficient trajectory points for interpolation.")
        return None

    # De-duplicate time points ensure that Slerp doesn't throw an error.
    # Right now, we're just keeping the later point.
    i = 0
    while i < len(t_pts):
        if (i > 0) and (t_pts[i] <= t_pts[i - 1]):
            warnings.warn("De-duplicated trajectory points at the same time.")
            t_pts = np.delete(t_pts, i - 1)
            x_pts = np.delete(x_pts, i - 1)
            y_pts = np.delete(y_pts, i - 1)
            yaw_pts = np.delete(yaw_pts, i - 1)
        else:
            i += 1

    # Set up Slerp interpolation for the angle.
    t_final = t_pts[-1]
    if t_final > 0:
        euler_angs = [[0, 0, th] for th in yaw_pts]
        slerp = Slerp(t_pts, Rotation.from_euler("xyz", euler_angs))

    # Package up the interpolated trajectory
    t_interp = np.arange(0, t_final, dt)
    if t_final not in t_interp:
        t_interp = np.append(t_interp, t_final)
    x_interp = np.interp(t_interp, t_pts, x_pts)
    y_interp = np.interp(t_interp, t_pts, y_pts)
    if t_final > 0:
        yaw_interp = np.array(
            [slerp(t).as_euler("xyz", degrees=False)[2] for t in t_interp]
        )
    else:
        yaw_interp = np.array([yaw_pts[-1]])
    return (t_interp, x_interp, y_interp, yaw_interp)
