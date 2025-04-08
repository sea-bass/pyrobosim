#!/usr/bin/env python3

"""Unit tests for lidar sensor."""

import os
import numpy as np

from pyrobosim.core import WorldYamlLoader
from pyrobosim.sensors.lidar import Lidar2D
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


def test_lidar_2d() -> None:
    # Setup a robot with a lidar sensor.
    world = WorldYamlLoader().from_file(
        os.path.join(get_data_folder(), "test_world.yaml")
    )
    lidar = Lidar2D(
        update_rate_s=0.1,
        angle_units="degrees",
        min_angle=-120.0,
        max_angle=120.0,
        angular_resolution=5.0,
        max_range_m=2.0,
    )
    robot = world.robots[0]
    robot.set_sensors({"lidar": lidar})

    # Verify that the sensor is not active.
    latest_measurement = lidar.get_measurement()
    assert len(latest_measurement) == 0

    # Verify that the sensor returns values after being updated.
    lidar.update()
    latest_measurement = lidar.get_measurement()
    assert len(latest_measurement) == len(lidar.angles)
    assert np.all(latest_measurement <= lidar.max_range_m)
