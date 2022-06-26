""" Trajectory generation and interpolation utilities. """

import numpy as np
from scipy.spatial.transform import Slerp, Rotation

from ..utils.pose import wrap_angle


def fill_path_yaws(path):
    """ 
    Fills in any yaw angles along a path to point at the next waypoint.
    
    :param path: List of poses representing a path.
    :type path: list[:class:`pyrobosim.utils.pose.Pose`]
    :return: Path with filled-in yaw angle values.
    :rtype: list[:class:`pyrobosim.utils.pose.Pose`]
    """
    if path is None:
        return path

    for idx in range(1, len(path)-1):
        path[idx].pose.yaw = np.arctan2(path[idx].pose.y - path[idx-1].pose.y,
                                        path[idx].pose.x - path[idx-1].pose.x)
    return path


def get_constant_speed_trajectory(path, linear_velocity=0.2, max_angular_velocity=None):
    """
    Gets a trajectory from a path (list of Pose objects) by
    calculating time points based on constant velocity and maximum angular velocity.

    The trajectory is returned as a tuple of numpy arrays
    (t_pts, x_pts, y_pts, theta_pts).

    :param path: List of poses representing a path.
    :type path: list[:class:`pyrobosim.utils.pose.Pose`]
    :param linear_velocity: Constant linear velocity in m/s, defaults to 0.2.
    :type linear_velocity: float
    :param max_angular_velocity: Maximum angular velocity in rad/s, defaults to None.
    :type max_angular_velocity: float, optional
    :return: Trajectory type of the form (t_pts, x_pts, y_pts, theta_pts).
    :rtype: tuple(:class:`numpy.array`)
    """
    if len(path) == 0:
        return None

    # Calculate the time points for the path at constant velocity, also accounting for
    # the maximum angular velocity if one is specified
    t_pts = np.zeros_like(path, dtype=np.float)
    for idx in range(len(path)-1):
        start_pose = path[idx].pose
        end_pose = path[idx+1].pose
        lin_time = start_pose.get_linear_distance(end_pose) / linear_velocity
        if max_angular_velocity is None:
            ang_time = 0
        else:
            ang_time = wrap_angle(start_pose.get_angular_distance(
                end_pose)) / max_angular_velocity
        t_pts[idx+1] = t_pts[idx] + max(lin_time, ang_time)

    # Package up the trajectory
    x_pts = np.array([p.pose.x for p in path])
    y_pts = np.array([p.pose.y for p in path])
    yaw_pts = np.array([p.pose.yaw for p in path])
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
    t_final = t_pts[-1]

    # De-duplicate time points ensure that Slerp doesn't throw an error.
    # Right now, we're just keeping the later point.
    i = 0
    while i < len(t_pts):
        if (i > 0) and (t_pts[i] <= t_pts[i-1]):
            print("Warning: De-duplicated trajectory points at the same time.")
            t_pts = np.delete(t_pts, i-1)
            x_pts = np.delete(x_pts, i-1)
            y_pts = np.delete(y_pts, i-1)
            yaw_pts = np.delete(yaw_pts, i-1)
        else:
            i += 1

    # Set up Slerp interpolation for the angle.
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
            [slerp(t).as_euler("xyz", degrees=False)[2] for t in t_interp])
    else:
        yaw_interp = np.array([yaw_pts[-1]])
    return (t_interp, x_interp, y_interp, yaw_interp)
