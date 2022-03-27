import time
import warnings

from ..utils.pose import Pose
from .trajectory import get_constant_speed_trajectory, interpolate_trajectory

""" Path execution utilities """


class ConstantVelocityExecutor:
    """ 
    Executes a path with a linear trajectory assuming constant 
    linear and angular velocity, and that the robot can perfectly
    go to the next pose.
    """
    def __init__(self, linear_velocity=1.0, dt=0.1, max_angular_velocity=None):
        self.linear_velocity = linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.dt = dt

        self.robot = None


    def execute(self, path, realtime_factor=1.0):
        if self.robot is None:
            warnings.warn("No robot attached to execute the trajectory.")
            return
        self.robot.executing_action = True
        
        # Convert the path to an interpolated trajectory
        traj = get_constant_speed_trajectory(
            path, linear_velocity=self.linear_velocity, 
            max_angular_velocity=self.max_angular_velocity)
        (traj_t, traj_x, traj_y, traj_yaw) = interpolate_trajectory(traj, self.dt)

        # Execute the trajectory
        sleep_time = self.dt / realtime_factor
        is_holding_object = self.robot.manipulated_object is not None
        for i in range(len(traj_t)):
            start_time = time.time()
            cur_pose = Pose(x=traj_x[i], y=traj_y[i], yaw=traj_yaw[i])
            self.robot.set_pose(cur_pose)
            if is_holding_object:
                self.robot.manipulated_object.pose = cur_pose
            time.sleep(max(0, sleep_time - (time.time()-start_time)))

        # Finalize path execution
        self.robot.location = path[-1].parent
        self.robot.executing_action = False
        return True
