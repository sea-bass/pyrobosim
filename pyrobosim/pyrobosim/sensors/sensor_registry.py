"""
Provides a registry of sensor models for instantiation and saving to file.

If adding your own sensors, you should register them in this file.
"""

from typing import Any

from .types import Sensor

# Sensors
from .lidar import Lidar2D

SENSORS_MAP = {
    "lidar": Lidar2D,
}


def get_sensor_class(sensor_type: str) -> Any:
    """
    Helper function that returns a sensor.

    :param sensor_type: The type of sensor.
    :return: The class corresponding to the sensor type specified.
    :raises ValueError: if the sensor type is invalid.
    """
    if sensor_type not in SENSORS_MAP:
        raise ValueError(f"{sensor_type} is not a supported sensor type.")

    return SENSORS_MAP[sensor_type]


def get_sensor_string(sensor: Sensor) -> str:
    """
    Helper function that returns the sensor string from a sensor instance.

    :param sensor: The sensor instance.
    :return: The string corresponding to the entry in `SENSORS_MAP`.
    :raises ValueError: if the sensor has no registered key.
    """
    for sensor_str, sensor_type in SENSORS_MAP.items():
        if isinstance(sensor, sensor_type):
            return sensor_str
    raise ValueError(
        f"Could not find registered key for sensor type: {type(sensor).__name__}"
    )
