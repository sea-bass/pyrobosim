#!/usr/bin/env python3

"""Unit tests for the hallway partial observability feature."""

import os
import time

from pyrobosim.core import WorldYamlLoader
from pyrobosim.navigation.execution import ConstantVelocityExecutor
from pyrobosim.sensors.lidar import Lidar2D
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim.utils.world_collision import is_connectable


def test_partial_obs_hallways_enabled() -> None:
    """Tests partial hallway observability feature with slight modifications to default world graph planner settings."""
    world = WorldYamlLoader().from_file(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    # Set up a closed hallway for testing
    world.hallways[0].is_open = False
    world.update_polygons()

    robot = world.robots[0]

    # Create a path through closed hallway
    start = Pose(x=-0.2, y=0.7)
    end = Pose(x=-0.8, y=1.3)

    collision_free_without_partial_obs_hallways = is_connectable(
        start=start,
        goal=end,
        world=world,
        robot=robot,
    )

    # Enable partial_obs_hallways
    # This would test if a robot would regard a hallway as open without prior knowledge
    robot.partial_obs_hallways = True
    robot.update_polygons()
    collision_free_with_partial_obs_hallways = is_connectable(
        start=start,
        goal=end,
        world=world,
        robot=robot,
    )

    assert collision_free_without_partial_obs_hallways == False
    assert collision_free_with_partial_obs_hallways == True


def test_detect_hallway_states() -> None:
    """Tests updating recorded closed hallways."""
    world = WorldYamlLoader().from_file(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    # Set up a closed hallway for testing
    world.hallways[0].is_open = False
    world.update_polygons()
    robot = world.robots[0]
    robot.partial_obs_hallways = True
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

    robot.partial_obs_hallways = True

    lidar_sensor = robot.sensors.get(path_executor.lidar_sensor_name)

    # Place robot more than 2m away from closed hallway
    robot.set_pose(Pose(x=2.5, y=0.5, yaw=3.142))
    lidar_sensor.update()
    time.sleep(1.0)
    assert robot.recorded_closed_hallways == set()

    # Place robot within 2m of closed hallway, but lidar range not detecting closed hallway
    robot.set_pose(Pose(x=0.3, y=0.4, yaw=-0.735))
    lidar_sensor.update()
    time.sleep(1.0)
    assert robot.recorded_closed_hallways == set()

    # Place robot within 2m of closed hallway, and lidar range detecting closed hallway
    robot.set_pose(Pose(x=0.3, y=0.4, yaw=2.356))
    lidar_sensor.update()
    time.sleep(1.0)
    assert robot.recorded_closed_hallways == set(
        [world.get_entity_by_name("hall_bathroom_kitchen")]
    )

    # Now open the hallway
    world.open_location("hall_bathroom_kitchen")
    lidar_sensor.update()
    time.sleep(1.0)
    assert robot.recorded_closed_hallways == set()

    robot.stop_sensor_threads()  # Ensures test can terminate
