"""
Robot dynamics utilities.
"""

import copy
import numpy as np
import warnings

from .pose import Pose


class RobotDynamics2D:
    """Simple 2D dynamics for robots."""

    def __init__(self, robot, pose=Pose()):
        # State
        self.robot = robot
        self.pose = pose
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.collision = False

        # Limits
        self.max_linear_velocity = 0.5
        self.max_angular_velocity = 1.0
        self.max_linear_acceleration = 1.0
        self.max_angular_acceleration = 3.0

    def step(self, cmd_vel, dt, world=None, check_collisions=False):
        """
        Perform a single dynamics step.

        :param cmd_vel: Velocity command of the form [vx, vy, vtheta].
        :type cmd_vel: np.array[float]
        :param dt: Time step, in seconds.
        :type dt: float
        """
        # TODO: Saturate to accel and velocity limits
        self.velocity = self.saturate_velocity_command(cmd_vel)

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

    def saturate_velocity_command(self, cmd_vel):
        """Saturate a velocity command given limits"""
        # First saturate to max velocity

        # Then saturate to acceleration (uses self.velocity)

        return cmd_vel

    def reset(self):
        """Reset all the dynamics of the robot."""
        self.commanded_velocity = 0.0
        self.velocity = 0.0
