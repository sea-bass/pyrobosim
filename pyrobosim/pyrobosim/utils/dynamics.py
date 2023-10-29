"""
Robot dynamics utilities.
"""

import copy
import numpy as np
import warnings

from .pose import Pose


class RobotDynamics2D:
    """Simple 2D dynamics for robots."""

    def __init__(
        self,
        robot,
        init_pose=Pose(),
        init_vel=np.array([0.0, 0.0, 0.0]),
        max_linear_velocity=0.5,
        max_angular_velocity=1.0,
        max_linear_acceleration=1.0,
        max_angular_acceleration=3.0,
    ):
        self.robot = robot

        # Initial state
        self.pose = init_pose
        self.velocity = init_vel
        self.collision = False

        # Velocity and acceleration limits
        self.vel_limits = np.array(
            [max_linear_velocity, max_linear_velocity, max_angular_velocity]
        )
        self.accel_limits = np.array(
            [max_linear_acceleration, max_linear_acceleration, max_angular_acceleration]
        )

    def step(self, cmd_vel, dt, world=None, check_collisions=False):
        """
        Perform a single dynamics step.

        :param cmd_vel: Velocity command of the form [vx, vy, vtheta].
        :type cmd_vel: np.array[float]
        :param dt: Time step, in seconds.
        :type dt: float
        """
        # Trivial case of zero or None command velocities.
        if np.count_nonzero(cmd_vel) == 0 or cmd_vel is None:
            return

        self.velocity = self.saturate_velocity_command(cmd_vel, dt)

        # Dynamics
        roll, pitch, yaw = self.pose.eul
        sin_yaw = np.sin(yaw)
        cos_yaw = np.cos(yaw)

        vx = self.velocity[0] * cos_yaw - self.velocity[1] * sin_yaw
        vy = self.velocity[0] * sin_yaw + self.velocity[1] * cos_yaw

        target_pose = copy.copy(self.pose)
        target_pose.x += vx * dt
        target_pose.y += vy * dt
        target_pose.set_euler_angles(roll, pitch, yaw + self.velocity[2] * dt)

        # Check collisions
        if check_collisions:
            if not world:
                warnings.warn("Cannot check collisions without a world.")
                return

            if world.collides_with_robots(
                target_pose, robot=self.robot
            ) or world.check_occupancy(target_pose):
                self.collision = True
                return

        # If we made it, we succeeded
        self.collision = False
        self.pose = target_pose

    def saturate_velocity_command(self, cmd_vel, dt):
        """Saturate a velocity command given limits"""
        # First saturate to velocity limits
        cmd_vel = np.clip(cmd_vel, -self.vel_limits, self.vel_limits)

        # Then saturate to acceleration limits
        cmd_vel = np.clip(
            cmd_vel,
            self.velocity - self.accel_limits * dt,
            self.velocity + self.accel_limits * dt,
        )

        return cmd_vel

    def reset(self, pose=None):
        """Reset all the dynamics of the robot."""
        if pose is not None:
            self.pose = pose
        self.commanded_velocity = 0.0
        self.velocity = 0.0
