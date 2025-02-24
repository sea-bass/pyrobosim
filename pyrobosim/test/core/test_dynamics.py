#!/usr/bin/env python3

"""
Unit tests for robot dynamics capabilities.
"""

import numpy as np
import pytest

from pyrobosim.core import Pose, RobotDynamics2D


def test_create_robot_dynamics_2d_default() -> None:
    """Checks creation of RobotDynamics2D object with default parameters."""
    dynamics = RobotDynamics2D()

    assert dynamics.pose == Pose()
    assert np.all(dynamics.velocity == np.array([0.0, 0.0, 0.0]))
    assert np.all(dynamics.vel_limits == np.array([np.inf, np.inf, np.inf]))
    assert np.all(dynamics.accel_limits == np.array([np.inf, np.inf, np.inf]))


def test_create_robot_dynamics_2d_nondefault() -> None:
    """Checks creation of RobotDynamics2D object with nondefault parameters."""
    dynamics = RobotDynamics2D(
        init_pose=Pose(x=1.0, y=2.0, yaw=np.pi / 2.0),
        init_vel=[-0.1, 0.0, 0.1],
        max_linear_velocity=1.0,
        max_angular_velocity=3.0,
        max_linear_acceleration=2.0,
        max_angular_acceleration=6.0,
    )

    assert dynamics.pose.x == 1.0
    assert dynamics.pose.y == 2.0
    assert dynamics.pose.eul[2] == pytest.approx(np.pi / 2.0)
    assert np.all(dynamics.velocity == np.array([-0.1, 0.0, 0.1]))
    assert np.all(dynamics.vel_limits == np.array([1.0, 1.0, 3.0]))
    assert np.all(dynamics.accel_limits == np.array([2.0, 2.0, 6.0]))


def test_enforce_dynamics_limits() -> None:
    """Tests functionality to saturate velocity commands given velocity and acceleration limits."""
    dynamics = RobotDynamics2D(
        max_linear_velocity=1.0,
        max_angular_velocity=3.0,
        max_linear_acceleration=2.0,
        max_angular_acceleration=6.0,
    )
    dt = 0.1

    # Should not saturate
    cmd_vel = np.array([0.1, 0.0, -0.1])
    sat_cmd_vel = dynamics.enforce_dynamics_limits(cmd_vel, dt)
    assert np.all(sat_cmd_vel == cmd_vel)

    # Should saturate due to velocity limits
    dynamics.velocity = np.array([0.95, 0.0, -2.95])
    cmd_vel = np.array([1.25, 0.0, -3.25])
    sat_cmd_vel = dynamics.enforce_dynamics_limits(cmd_vel, dt)
    assert np.allclose(sat_cmd_vel, np.array([1.0, 0.0, -3.0]))

    # Should saturate due to acceleration limits
    dynamics.velocity = np.array([0.0, 0.0, 0.0])
    cmd_vel = np.array([0.5, 0.0, -2.5])
    sat_cmd_vel = dynamics.enforce_dynamics_limits(cmd_vel, dt)
    assert np.allclose(sat_cmd_vel, np.array([0.2, 0.0, -0.6]))


def test_reset_default_args() -> None:
    """Test resetting the dynamics with default arguments."""
    start_pose = Pose(x=1.0, y=2.0, yaw=3.0)
    start_vel = np.array([0.1, 0.0, -0.1])
    dynamics = RobotDynamics2D(init_pose=start_pose, init_vel=start_vel)

    dynamics.reset()
    assert dynamics.pose == start_pose
    assert np.all(dynamics.velocity == np.array([0.0, 0.0, 0.0]))


def test_reset_nondefault_args() -> None:
    """Test resetting the dynamics with nondefault arguments."""
    start_pose = Pose(x=1.0, y=2.0, yaw=3.0)
    start_vel = np.array([0.1, 0.0, -0.1])
    dynamics = RobotDynamics2D(init_pose=start_pose, init_vel=start_vel)

    target_pose = Pose(x=42.0, y=0.0, yaw=np.pi)
    target_velocity = np.array([0.2, 0.0, 0.0])

    dynamics.reset(pose=target_pose, velocity=target_velocity)
    assert dynamics.pose == target_pose
    assert np.all(dynamics.velocity == target_velocity)


def test_step() -> None:
    """Test stepping dynamics without collision checks."""
    dynamics = RobotDynamics2D()
    dt = 0.1

    # Linear velocity only
    cmd_vel = np.array([1.0, 0.0, 0.0])
    new_pose = dynamics.step(cmd_vel, dt)
    assert new_pose.x == pytest.approx(0.1)
    assert new_pose.y == pytest.approx(0.0)
    assert new_pose.eul[2] == pytest.approx(0.0)
    dynamics.reset(new_pose)

    # Angular velocity only
    cmd_vel = np.array([0.0, 0.0, np.pi / 2.0])
    new_pose = dynamics.step(cmd_vel, dt)
    assert new_pose.x == pytest.approx(0.1)
    assert new_pose.y == pytest.approx(0.0)
    assert new_pose.eul[2] == pytest.approx(np.pi / 20.0)
    dynamics.reset(new_pose)

    # Linear and angular velocity
    cmd_vel = np.array([1.0, 0.0, np.pi / 2.0])
    new_pose = dynamics.step(cmd_vel, dt)
    assert new_pose.x == pytest.approx(0.1 * (1 + np.cos(np.pi / 20.0)))
    assert new_pose.y == pytest.approx(0.1 * np.sin(np.pi / 20.0))
    assert new_pose.eul[2] == pytest.approx(np.pi / 10.0)


def test_step_zero_cmd() -> None:
    """Test that stepping dynamics with zero velocities does nothing."""
    dynamics = RobotDynamics2D()
    dt = 0.1
    cmd_vel = np.array([0.0, 0.0, 0.0])
    new_pose = dynamics.step(cmd_vel, dt)
    assert new_pose == Pose()


def test_step_none_cmd() -> None:
    """Test that stepping dynamics with a None command does nothing."""
    dynamics = RobotDynamics2D()
    dt = 0.1
    cmd_vel = None
    new_pose = dynamics.step(cmd_vel, dt)
    assert new_pose == Pose()
