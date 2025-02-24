"""
Robot dynamics utilities.
"""

import copy
import numpy as np
from typing import Sequence

from ..utils.pose import Pose


class RobotDynamics2D:
    """Simple 2D dynamics for robots."""

    def __init__(
        self,
        init_pose: Pose = Pose(),
        init_vel: np.ndarray = np.array([0.0, 0.0, 0.0]),
        max_linear_velocity: float = np.inf,
        max_angular_velocity: float = np.inf,
        max_linear_acceleration: float = np.inf,
        max_angular_acceleration: float = np.inf,
    ) -> None:
        """
        Creates an instance of a 2D robot dynamics object.

        :param init_pose: The initial pose of the robot.
        :param init_vel: The initial velocity of the robot, as a [vx, vy, vtheta] array.
        :param max_linear_velocity: The maximum linear velocity magnitude, in m/s.
        :param max_angular_velocity: The maximum angular velocity magnitude, in rad/s.
        :param max_linear_acceleration: The maximum linear acceleration magnitude, in m/s^2.
        :param max_angular_acceleration: The maximum angular acceleration magnitude, in rad/s^2.
        """
        # Initial state
        self.pose = init_pose
        self.velocity = np.array(init_vel)

        # Velocity and acceleration limits
        self.vel_limits = np.array(
            [max_linear_velocity, max_linear_velocity, max_angular_velocity]
        )
        self.accel_limits = np.array(
            [max_linear_acceleration, max_linear_acceleration, max_angular_acceleration]
        )

    def step(self, cmd_vel: Sequence[float], dt: float) -> Pose:
        """
        Perform a single dynamics step.

        :param cmd_vel: Velocity command array, in the form [vx, vy, vtheta].
        :param dt: Time step, in seconds.
        :return: The new pose after applying the dynamics.
        """
        # Trivial case of zero or None command velocities.
        if (np.count_nonzero(cmd_vel) == 0) or (cmd_vel is None):
            return self.pose

        self.velocity = self.enforce_dynamics_limits(cmd_vel, dt)

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
        return target_pose

    def enforce_dynamics_limits(
        self, cmd_vel: Sequence[float], dt: float
    ) -> np.ndarray:
        """
        Enforces velocity and acceleration limits by saturating a velocity command.

        :param cmd_vel: Velocity command array, in the form [vx, vy, vtheta].
        :param dt: Time step, in seconds.
        :return: The saturated velocity command array.
        """
        # First saturate to velocity limits
        cmd_vel = np.clip(cmd_vel, -self.vel_limits, self.vel_limits)

        # Then saturate to acceleration limits
        cmd_vel = np.clip(
            cmd_vel,
            self.velocity - self.accel_limits * dt,
            self.velocity + self.accel_limits * dt,
        )

        return cmd_vel

    def reset(
        self, pose: Pose | None = None, velocity: np.ndarray = np.array([0.0, 0.0, 0.0])
    ) -> None:
        """
        Reset all the dynamics of the robot to provided values.

        :param pose: The pose to reset to. If None, will keep the current pose.
        :param velocity: The velocity to reset to. Defaults to zero velocities.
        """
        if pose is not None:
            self.pose = pose
        self.velocity = velocity
