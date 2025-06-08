#!/usr/bin/env python3

"""Unit tests for the Partial Observability Hallway States feature."""

import os

from pyrobosim.core import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim.sensors.lidar import Lidar2D
from pyrobosim.navigation.execution import ConstantVelocityExecutor


def test_partial_observability_hallway_states_enabled() -> None:
    """Tests planning with default world graph planner settings."""
    world = WorldYamlLoader().from_file(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    # Set up a closed hallway for testing
    world.hallways[0].is_open = False
    world.update_polygons()

    # Enable partial observability hallway states
    robot = world.robots[0]

    # Create a path through closed hallway
    start = Pose(x=-0.2, y=0.7)
    end = Pose(x=-0.8, y=1.3)

    collision_free_without_partial_observability_hallway_states = robot.world.is_connectable(
        start=start,
        goal=end,
        partial_observability_hallway_states=robot.partial_observability_hallway_states,
        recorded_closed_hallways=robot.recorded_closed_hallways,
    )

    # This would test if a robot would regard a hallway as open without prior knowledge
    robot.partial_observability_hallway_states = True
    collision_free_with_partial_observability_hallway_states = robot.world.is_connectable(
        start=start,
        goal=end,
        partial_observability_hallway_states=robot.partial_observability_hallway_states,
        recorded_closed_hallways=robot.recorded_closed_hallways,
    )

    assert collision_free_without_partial_observability_hallway_states == False
    assert collision_free_with_partial_observability_hallway_states == True


def test_detect_closed_hallway() -> None:
    """Tests updating recorded closed hallways."""
    world = WorldYamlLoader().from_file(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    # Set up a closed hallway for testing
    world.hallways[0].is_open = False
    world.update_polygons()
    robot = world.robots[0]
    lidar = Lidar2D(
        update_rate_s=0.1,
        angle_units="degrees",
        min_angle=-120.0,
        max_angle=120.0,
        angular_resolution=5.0,
        max_range_m=2.0,
    )
    robot.set_sensors({"lidar": lidar})

    path_executor = ConstantVelocityExecutor(lidar_sensor_name="lidar")
    robot.set_path_executor(path_executor)

    robot.partial_observability_hallway_states = True

    lidar_sensor = robot.sensors.get(path_executor.lidar_sensor_name)

    # Place robot more than 2m away from closed hallway
    robot.set_pose(Pose(x=2.5, y=0.5, yaw=3.142))
    lidar_sensor.update()
    robot.path_executor.following_path = True
    robot.path_executor.abort_execution = False
    robot.path_executor.detect_closed_hallway()
    robot_knowledge_test1 = robot.recorded_closed_hallways

    # Stop the function
    robot.following_path = False
    robot.path_executor.abort_execution = True

    # Place robot within 2m of closed hallway, but lidar range not detecting closed hallway
    robot.set_pose(Pose(x=0.3, y=0.4, yaw=-0.735))
    lidar_sensor.update()
    robot.path_executor.following_path = True
    robot.path_executor.abort_execution = False
    robot.path_executor.detect_closed_hallway()
    robot_knowledge_test2 = robot.recorded_closed_hallways

    # Stop the function
    robot.following_path = False
    robot.path_executor.abort_execution = True

    # Place robot within 2m of closed hallway, and lidar range detecting closed hallway
    robot.set_pose(Pose(x=0.3, y=0.4, yaw=2.356))
    lidar_sensor.update()
    robot.path_executor.following_path = True
    robot.path_executor.abort_execution = False
    robot.path_executor.detect_closed_hallway()
    robot_knowledge_test3 = robot.recorded_closed_hallways

    # Stop the function
    robot.following_path = False
    robot.path_executor.abort_execution = True

    assert len(robot_knowledge_test1) == 0
    assert len(robot_knowledge_test2) == 0
    assert len(robot_knowledge_test3) == 1
