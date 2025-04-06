"""Lidar sensor simulation."""

import time
from typing import Any

from matplotlib.artist import Artist
from matplotlib.collections import LineCollection
import numpy as np
from shapely import get_parts, intersects, intersection_all
from shapely.geometry import MultiLineString
from shapely.ops import unary_union

from .types import Sensor

from ..core.robot import Robot
from ..utils.polygon import transform_polygon


class Lidar2D(Sensor):
    """
    Implements a simulated 2D lidar sensor.
    """

    def __init__(
        self,
        *,
        update_rate_s: float,
        min_angle_rad: float,
        max_angle_rad: float,
        angle_resolution_rad: float,
        max_range_m: float,
    ) -> None:
        """
        Constructs a 2D lidar sensor instance.


        """
        super().__init__()
        self.update_rate_s = update_rate_s
        self.min_angle_rad = min_angle_rad
        self.max_angle_rad = max_angle_rad
        self.angle_resolution_rad = angle_resolution_rad
        self.max_range_m = max_range_m

        self.angles = np.arange(
            min_angle_rad, max_angle_rad + angle_resolution_rad, angle_resolution_rad
        )
        self.num_beams = len(self.angles)
        self.orig_lidar_lines = MultiLineString(
            [
                np.array(
                    [
                        [0.0, 0.0],
                        [max_range_m * np.cos(angle), max_range_m * np.sin(angle)],
                    ]
                )
                for angle in self.angles
            ]
        )
        self.lidar_lines = MultiLineString()
        self.lidar_coords = [line.coords for line in self.orig_lidar_lines.geoms]

    def update(self) -> None:
        """Does the lidar calculation."""
        if (self.robot is None) or (self.robot.world is None):
            return

        with self.robot.state_lock:
            if self.robot.world:
                self.lidar_lines = intersection_all(
                    [
                        transform_polygon(self.orig_lidar_lines, self.robot.get_pose()),
                        self.robot.world.total_external_polygon,
                    ]
                ).difference(
                    unary_union(
                        [
                            r.polygon
                            for r in self.robot.world.robots
                            if r is not self.robot
                        ]
                    )
                )
                self.lidar_coords = [
                    part.coords
                    for part in get_parts(self.lidar_lines.geoms)[
                        intersects(self.robot.polygon, self.lidar_lines.geoms)
                    ]
                ]

    def thread_function(self) -> None:
        if self.robot is None:
            return

        while self.robot.sensors_active:
            t_start = time.time()
            self.update()
            t_end = time.time()
            print(f"Lidar calculation took: {(t_end - t_start) * 1000} ms")
            time.sleep(max(0.0, self.update_rate_s - (t_end - t_start)))

    def get_measurement(self) -> tuple[np.ndarray, np.ndarray]:
        """Gets the latest sensor measurement."""
        if self.lidar_lines.is_empty:
            return (self.angles, np.array([]))

        return (self.angles, np.array([line.length for line in self.lidar_lines.geoms]))

    def setup_artists(self) -> list[Artist]:
        """
        Sets up and returns the artists for visualizing the sensor.
        """
        self.artist = LineCollection(
            self.lidar_coords,
            color=self.robot.color if (self.robot is not None) else "b",
            alpha=0.5,
            linewidth=0.5,
            linestyle="-",
            zorder=0,
        )
        return [self.artist]

    def update_artists(self) -> None:
        """
        Updates the artists.

        These should have been originally returned by `setup_artists()`.
        """
        self.artist.set_paths(self.lidar_coords)

    def to_dict(self) -> dict[str, Any]:
        """
        Serializes the sensor to a dictionary.

        :return: A dictionary containing the sensor information.
        """
        from .sensor_registry import get_sensor_string

        return {
            "type": get_sensor_string(self),
            "update_rate_s": self.update_rate_s,
            "min_angle_rad": self.min_angle_rad,
            "max_angle_rad": self.max_angle_rad,
            "angle_resolution_rad": self.angle_resolution_rad,
            "max_range_m": self.max_range_m,
        }
