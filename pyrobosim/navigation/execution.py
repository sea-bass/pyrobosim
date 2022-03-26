import time

from ..utils.pose import Pose
from ..utils.trajectory import get_constant_speed_trajectory, interpolate_trajectory

""" Path execution utilities """


def execute_with_linear_trajectory(robot, path, dt, realtime_factor=1.0, 
    linear_velocity=0.2, max_angular_velocity=None):
    """ 
    Executes a path with a linear trajectory assuming constant 
    linear and angular velocity, and that the robot can perfectly
    go to the next pose.
    """
    robot.executing_action = True
    
    # Convert the path to an interpolated trajectory
    traj = get_constant_speed_trajectory(
        path, linear_velocity=linear_velocity, max_angular_velocity=max_angular_velocity)
    (traj_t, traj_x, traj_y, traj_yaw) = interpolate_trajectory(traj, dt)

    # Execute the trajectory
    sleep_time = dt / realtime_factor
    is_holding_object = robot.manipulated_object is not None
    for i in range(len(traj_t)):
        start_time = time.time()
        cur_pose = Pose(x=traj_x[i], y=traj_y[i], yaw=traj_yaw[i])
        robot.set_pose(cur_pose)
        if is_holding_object:
            robot.manipulated_object.pose = cur_pose
        time.sleep(max(0, sleep_time - (time.time()-start_time)))
    robot.executing_action = False
    return True
