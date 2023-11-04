"""
Robot dynamics utilities.
"""

import copy
import numpy as np
import warnings

from ..utils.pose import Pose


class RobotDynamics2D:
    """Simple 2D dynamics for robots."""

    def __init__(
        self,
        robot=None,
        init_pose=Pose(),
        init_vel=np.array([0.0, 0.0, 0.0]),
        max_linear_velocity=np.inf,
        max_angular_velocity=np.inf,
        max_linear_acceleration=np.inf,
        max_angular_acceleration=np.inf,
    ):
        """
        Creates an instance of a 2D robot dynamics object.

        :param robot: The robot corresponding to this dynamics object.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        :param init_pose: The initial pose of the robot.
        :type init_pose: :class:`pyrobosim.utils.pose.Pose`
        :param init_vel: The initial velocity of the robot, as a [vx, vy, vtheta] array.
        :type init_vel: :class:`numpy.array`
        :param max_linear_velocity: The maximum linear velocity magnitude, in m/s.
        :type max_linear_velocity: float
        :param max_angular_velocity: The maximum angular velocity magnitude, in rad/s.
        :type max_angular_velocity: float
        :param max_linear_acceleration: The maximum linear acceleration magnitude, in m/s^2.
        :type max_linear_acceleration: float
        :param max_angular_acceleration: The maximum angular acceleration magnitude, in rad/s^2.
        :type max_linear_acceleration: float
        """
        self.robot = robot

        # Initial state
        self.pose = init_pose
        self.velocity = np.array(init_vel)
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

        :param cmd_vel: Velocity command array, in the form [vx, vy, vtheta].
        :type cmd_vel: :class:`numpy.array`
        :param dt: Time step, in seconds.
        :type dt: float
        """
        # Trivial case of zero or None command velocities.
        if np.count_nonzero(cmd_vel) == 0 or cmd_vel is None:
            return

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

        # Check collisions
        if check_collisions:
            if world is None:
                warnings.warn("Cannot check collisions without a world.")
                return

            if world.collides_with_robots(
                target_pose, robot=self.robot
            ) or world.check_occupancy(target_pose):
                self.velocity = np.array([0.0, 0.0, 0.0])
                self.collision = True
                return

        # If we made it, we succeeded
        self.collision = False
        self.pose = target_pose

    def enforce_dynamics_limits(self, cmd_vel, dt):
        """
        Enforces velocity and acceleration limits by saturating a velocity command.

        :param cmd_vel: Velocity command array, in the form [vx, vy, vtheta].
        :type cmd_vel: :class:`numpy.array`
        :param dt: Time step, in seconds.
        :type dt: float
        :return: The saturated velocity command array.
        :rtype: :class:`numpy.array`
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

    def reset(self, pose=None, velocity=np.array([0.0, 0.0, 0.0])):
        """
        Reset all the dynamics of the robot to provided values.

        :param pose: The pose to reset to. If None, will keep the current pose.
        :type pose: :class:`pyrobosim.utils.pose.Pose`, optional
        :param velocity: The velocity to reset to. Defaults to zero velocities.
        :type velocity: :class:`numpy.array`, optional
        """
        if pose is not None:
            self.pose = pose
        self.velocity = velocity
