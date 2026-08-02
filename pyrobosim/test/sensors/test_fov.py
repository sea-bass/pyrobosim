#!/usr/bin/env python3

"""Unit tests for the field-of-view (FOV) sensor."""

import numpy as np
import pathlib

from pyrobosim.core.world import World
from pyrobosim.core.yaml_utils import WorldYamlLoader
from pyrobosim.planning.actions import ExecutionStatus
from pyrobosim.sensors.fov import FOVSensor
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


def create_test_world() -> World:
    return WorldYamlLoader().from_file(
        pathlib.Path(get_data_folder()) / "test_world.yaml"
    )


def create_fov_sensor() -> FOVSensor:
    return FOVSensor(
        update_rate_s=0.1,
        angle_units="degrees",
        min_angle=-45.0,
        max_angle=45.0,
        max_range_m=1.5,
    )


def test_fov_sensor() -> None:
    # Setup a robot with an FOV sensor.
    world = create_test_world()
    fov = create_fov_sensor()
    robot = world.robots[0]
    robot.set_sensors({"object_detector": fov})

    # Verify that the sensor has no measurement before being updated.
    assert fov.get_measurement() == []

    # Face the banana on the kitchen table from nearby.
    banana = world.get_object_by_name("banana0")
    assert banana is not None
    robot.set_pose(Pose(x=banana.pose.x - 0.5, y=banana.pose.y, yaw=0.0))
    fov.update()
    assert banana in fov.get_measurement()

    # The field of view must not extend outside the world's sensing region.
    assert fov.fov_polygon.within(world.total_sensing_polygon.buffer(1.0e-6))

    # Face away from the banana.
    robot.set_pose(Pose(x=banana.pose.x - 0.5, y=banana.pose.y, yaw=np.pi))
    fov.update()
    assert banana not in fov.get_measurement()

    # The display polygons only show when the sensor is active.
    assert fov.get_display_polygons() == []
    fov.is_active = True
    assert len(fov.get_display_polygons()) > 0


def test_fov_sensor_closed_location() -> None:
    # Objects inside a closed location are not visible until it is opened.
    world = create_test_world()
    fov = create_fov_sensor()
    robot = world.robots[0]
    robot.set_sensors({"object_detector": fov})

    fuji = world.get_object_by_name("fuji")  # In the trash can, which is closed.
    assert fuji is not None
    robot.set_pose(Pose(x=fuji.pose.x - 0.5, y=fuji.pose.y, yaw=0.0))
    fov.update()
    assert fuji not in fov.get_measurement()

    result = world.open_location("trash")
    assert result.is_success()
    fov.update()
    assert fuji in fov.get_measurement()


def test_detect_objects_with_fov_sensor() -> None:
    # With an FOV sensor, detection uses the field of view instead of
    # requiring the robot to be at an object spawn.
    world = create_test_world()
    fov = create_fov_sensor()
    robot = world.robots[0]
    robot.set_sensors({"object_detector": fov})

    banana = world.get_object_by_name("banana0")
    assert banana is not None
    robot.set_pose(Pose(x=banana.pose.x - 0.5, y=banana.pose.y, yaw=0.0))

    result = robot.detect_objects()
    assert result.status == ExecutionStatus.SUCCESS
    assert banana in robot.last_detected_objects
    assert banana in robot.known_objects

    result = robot.detect_objects("banana")
    assert result.status == ExecutionStatus.SUCCESS
    assert robot.last_detected_objects == [banana]

    result = robot.detect_objects("nonexistent")
    assert result.status == ExecutionStatus.EXECUTION_FAILURE
    assert robot.last_detected_objects == []

    # Nothing is detected when facing away from the banana.
    robot.set_pose(Pose(x=banana.pose.x - 0.5, y=banana.pose.y, yaw=np.pi))
    result = robot.detect_objects("banana")
    assert result.status == ExecutionStatus.EXECUTION_FAILURE
