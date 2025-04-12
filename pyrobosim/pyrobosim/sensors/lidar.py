"""Lidar sensor simulation."""

import time
from typing import Any

from matplotlib.artist import Artist
from matplotlib.collections import LineCollection
import numpy as np
from shapely import get_parts, intersects, intersection_all
from shapely.geometry import LineString, MultiLineString
from shapely.ops import unary_union

from .types import Sensor

from ..utils.polygon import transform_polygon


class Lidar2D(Sensor):
    """
    Implements a simulated 2D lidar sensor.
    """

    plugin_name = "lidar"  # Needed to register plugin.

    def __init__(
        self,
        *,
        update_rate_s: float,
        angle_units: str,
        min_angle: float,
        max_angle: float,
        angular_resolution: float,
        max_range_m: float,
    ) -> None:
        """
        Constructs a 2D lidar sensor instance.

        :param update_rate_s: The sensor thread update rate, in seconds.
        :param angle_units: The units to use for the angle limits and resolutions.
            Must be either "degrees" or "radians".
        :param min_angle: The minimum scan angle, in the specified units.
        :param max_angle: The maximum scan angle, in the specified units.
        :param angular_resolution: The angular resolution, in the specified units.
        :param max_range_m: The maximum range of each lidar beam, in meters.
        """
        super().__init__()
        self.update_rate_s = update_rate_s
        self.angle_units = angle_units
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.angular_resolution = angular_resolution
        self.max_range_m = max_range_m

        if self.angle_units not in ("degrees", "radians"):
            raise ValueError("Must specify angle units of 'degrees' or 'radians'.")
        units_scaling = 1.0 if angle_units == "radians" else np.pi / 180

        self.angles = (
            np.arange(min_angle, max_angle + angular_resolution, angular_resolution)
            * units_scaling
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
        self.lidar_lines: list[LineString] = []
        self.lidar_coords = [line.coords for line in self.orig_lidar_lines.geoms]

    def update(self) -> None:
        """Performs the lidar calculation."""
        if (self.robot is None) or (self.robot.world is None):
            return

        if self.robot.world:
            # Need to extract the robot polygon here to prevent syncing issues.
            robot_polygon = self.robot.polygon
            lines = (
                intersection_all(
                    [
                        transform_polygon(self.orig_lidar_lines, self.robot.get_pose()),
                        self.robot.world.total_external_polygon,
                    ]
                )
                .difference(
                    unary_union(
                        [
                            r.polygon
                            for r in self.robot.world.robots
                            if r is not self.robot
                        ]
                    )
                )
                .geoms
            )
            self.lidar_lines = get_parts(lines)[intersects(robot_polygon, lines)]
            self.lidar_coords = [line.coords for line in self.lidar_lines]

    def thread_function(self) -> None:
        """
        Defines the sensor update function to run in a background thread.
        """
        if self.robot is None:
            return

        while self.is_active:
            t_start = time.time()
            self.update()
            t_end = time.time()
            time.sleep(max(0.0, self.update_rate_s - (t_end - t_start)))

    def get_measurement(self) -> np.ndarray:
        """
        Gets the latest sensor measurement.

        :return: An array of all the lengths, corresponding to the `angles` attribute.
        """
        if len(self.lidar_lines) == 0:
            return np.array([])

        return np.array([line.length for line in self.lidar_lines])

    def setup_artists(self) -> list[Artist]:
        """
        Sets up and returns the artists for visualizing the sensor.

        :return: The list of MatPlotLib artists for the sensor.
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
        return {
            "type": self.plugin_name,
            "update_rate_s": self.update_rate_s,
            "angle_units": self.angle_units,
            "min_angle": self.min_angle,
            "max_angle": self.max_angle,
            "angular_resolution": self.angular_resolution,
            "max_range_m": self.max_range_m,
        }
