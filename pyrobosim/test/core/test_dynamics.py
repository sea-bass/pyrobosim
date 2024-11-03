#!/usr/bin/env python3

"""
Unit tests for robot dynamics capabilities.
"""

import numpy as np
import pytest

from pyrobosim.core import Pose, Robot, RobotDynamics2D, World


def test_create_robot_dynamics_2d_default():
    """Checks creation of RobotDynamics2D object with default parameters."""
    dynamics = RobotDynamics2D()

    assert dynamics.robot is None
    assert dynamics.pose == Pose()
    assert not dynamics.collision
    assert np.all(dynamics.velocity == np.array([0.0, 0.0, 0.0]))
    assert np.all(dynamics.vel_limits == np.array([np.inf, np.inf, np.inf]))
    assert np.all(dynamics.accel_limits == np.array([np.inf, np.inf, np.inf]))


def test_create_robot_dynamics_2d_nondefault():
    """Checks creation of RobotDynamics2D object with nondefault parameters."""
    robot = Robot()
    dynamics = RobotDynamics2D(
        robot=robot,
        init_pose=Pose(x=1.0, y=2.0, yaw=np.pi / 2.0),
        init_vel=[-0.1, 0.0, 0.1],
        max_linear_velocity=1.0,
        max_angular_velocity=3.0,
        max_linear_acceleration=2.0,
        max_angular_acceleration=6.0,
    )

    assert dynamics.robot == robot
    assert dynamics.pose.x == 1.0
    assert dynamics.pose.y == 2.0
    assert dynamics.pose.eul[2] == pytest.approx(np.pi / 2.0)
    assert not dynamics.collision
    assert np.all(dynamics.velocity == np.array([-0.1, 0.0, 0.1]))
    assert np.all(dynamics.vel_limits == np.array([1.0, 1.0, 3.0]))
    assert np.all(dynamics.accel_limits == np.array([2.0, 2.0, 6.0]))


def test_enforce_dynamics_limits():
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


def test_reset_default_args():
    """Test resetting the dynamics with default arguments."""
    start_pose = Pose(x=1.0, y=2.0, yaw=3.0)
    start_vel = np.array([0.1, 0.0, -0.1])
    dynamics = RobotDynamics2D(init_pose=start_pose, init_vel=start_vel)

    dynamics.reset()
    assert dynamics.pose == start_pose
    assert np.all(dynamics.velocity == np.array([0.0, 0.0, 0.0]))


def test_reset_nondefault_args():
    """Test resetting the dynamics with nondefault arguments."""
    start_pose = Pose(x=1.0, y=2.0, yaw=3.0)
    start_vel = np.array([0.1, 0.0, -0.1])
    dynamics = RobotDynamics2D(init_pose=start_pose, init_vel=start_vel)

    target_pose = Pose(x=42.0, y=0.0, yaw=np.pi)
    target_velocity = np.array([0.2, 0.0, 0.0])

    dynamics.reset(pose=target_pose, velocity=target_velocity)
    assert dynamics.pose == target_pose
    assert np.all(dynamics.velocity == target_velocity)


def test_step_no_collision():
    """Test stepping dynamics without collision checks."""
    dynamics = RobotDynamics2D()
    dt = 0.1

    # Linear velocity only
    cmd_vel = np.array([1.0, 0.0, 0.0])
    dynamics.step(cmd_vel, dt)
    assert dynamics.pose.x == pytest.approx(0.1)
    assert dynamics.pose.y == pytest.approx(0.0)
    assert dynamics.pose.eul[2] == pytest.approx(0.0)

    # Angular velocity only
    cmd_vel = np.array([0.0, 0.0, np.pi / 2.0])
    dynamics.step(cmd_vel, dt)
    assert dynamics.pose.x == pytest.approx(0.1)
    assert dynamics.pose.y == pytest.approx(0.0)
    assert dynamics.pose.eul[2] == pytest.approx(np.pi / 20.0)

    # Linear and angular velocity
    cmd_vel = np.array([1.0, 0.0, np.pi / 2.0])
    dynamics.step(cmd_vel, dt)
    assert dynamics.pose.x == pytest.approx(0.1 * (1 + np.cos(np.pi / 20.0)))
    assert dynamics.pose.y == pytest.approx(0.1 * np.sin(np.pi / 20.0))
    assert dynamics.pose.eul[2] == pytest.approx(np.pi / 10.0)


def test_step_collision_zero_cmd():
    """Test that stepping dynamics with zero velocities does nothing."""
    dynamics = RobotDynamics2D()
    dt = 0.1
    cmd_vel = np.array([0.0, 0.0, 0.0])
    dynamics.step(cmd_vel, dt)
    assert dynamics.pose == Pose()


def test_step_collision_none_cmd():
    """Test that stepping dynamics with a None command does nothing."""
    dynamics = RobotDynamics2D()
    dt = 0.1
    cmd_vel = None
    dynamics.step(cmd_vel, dt)
    assert dynamics.pose == Pose()


def test_step_collision_no_world(caplog):
    """Test that stepping dynamics with collision but no world returns with a warning."""
    dynamics = RobotDynamics2D()
    dt = 0.1
    cmd_vel = np.array([1.0, 0.0, 0.0])

    dynamics.step(cmd_vel, dt, check_collisions=True)
    assert "Cannot check collisions without a world" in caplog.text
    assert dynamics.pose == Pose()


def test_step_collision_world():
    """Test that stepping dynamics with collision in a world with walls works."""
    world = World()
    world.add_room(
        name="test_room", footprint=[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    )
    robot = Robot(name="test_robot", radius=0.1)
    world.add_robot(robot)

    dt = 0.1
    cmd_vel = np.array([1.0, 0.0, 0.0])

    # Try a safe pose
    start_pose = Pose(x=0.5, y=0.5)
    robot.set_pose(start_pose)
    robot.dynamics.step(cmd_vel, dt, world=world, check_collisions=True)
    assert robot.get_pose().is_approx(Pose(x=0.6, y=0.5))
    assert not robot.is_in_collision()
    assert robot.is_moving()

    # Try an unsafe pose (robot will collide with room wall)
    start_pose = Pose(x=0.89, y=0.5)
    robot.set_pose(start_pose)
    robot.dynamics.step(cmd_vel, dt, world=world, check_collisions=True)
    assert robot.get_pose() == start_pose
    assert robot.is_in_collision()
    assert not robot.is_moving()


def test_step_collision_robot():
    """Test that stepping dynamics with collision between robots works."""
    world = World()
    world.add_room(
        name="test_room", footprint=[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    )
    robot0 = Robot(name="test_robot0", radius=0.1)
    world.add_robot(robot0)
    robot1 = Robot(name="test_robot1", radius=0.1)
    world.add_robot(robot1)

    cmd_vel = np.array([1.0, 0.0, 0.0])
    dt = 0.1

    # Try safe poses
    robot0.set_pose(Pose(x=0.5, y=0.5))
    robot1.set_pose(Pose(x=0.5, y=0.8))
    robot0.dynamics.step(cmd_vel, dt, world=world, check_collisions=True)
    assert robot0.get_pose().is_approx(Pose(x=0.6, y=0.5))
    assert not robot0.is_in_collision()
    assert robot0.is_moving()

    # Try an unsafe pose (robot will collide into the other robot)
    robot0.set_pose(Pose(x=0.5, y=0.5))
    robot1.set_pose(Pose(x=0.55, y=0.5))
    robot0.dynamics.step(cmd_vel, dt, world=world, check_collisions=True)
    assert robot0.get_pose() == Pose(x=0.5, y=0.5)
    assert robot0.is_in_collision()
    assert not robot0.is_moving()
