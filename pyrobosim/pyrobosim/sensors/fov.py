"""Field-of-view (FOV) sensor simulation."""

import time
from typing import Any, Iterable, TYPE_CHECKING

import numpy as np
from shapely import get_parts, intersection, intersects
from shapely.geometry import MultiPolygon, Polygon

from .types import Sensor

from ..utils.polygon import transform_polygon

if TYPE_CHECKING:
    from ..core.objects import Object


class FOVSensor(Sensor):
    """
    Implements a simulated field-of-view (FOV) object detection sensor.

    The sensor observes a cone (circular sector) attached to the robot,
    clipped so it does not extend through walls or closed hallways.
    Locations do not clip the field of view, so objects on top of them
    can be seen. The measurement is the list of world objects inside the
    field of view, which the robot's detect action uses instead of its
    current location if any such sensor is available.
    """

    plugin_name = "fov"  # Needed to register plugin.

    def __init__(
        self,
        *,
        update_rate_s: float,
        angle_units: str,
        min_angle: float,
        max_angle: float,
        max_range_m: float,
    ) -> None:
        """
        Constructs a field-of-view sensor instance.

        :param update_rate_s: The sensor thread update rate, in seconds.
        :param angle_units: The units to use for the angle limits.
            Must be either "degrees" or "radians".
        :param min_angle: The minimum angle of the cone, in the specified units.
        :param max_angle: The maximum angle of the cone, in the specified units.
        :param max_range_m: The maximum range of the cone, in meters.
        """
        super().__init__()
        self.update_rate_s = update_rate_s
        self.angle_units = angle_units
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.max_range_m = max_range_m

        if self.angle_units not in ("degrees", "radians"):
            raise ValueError("Must specify angle units of 'degrees' or 'radians'.")
        if max_angle <= min_angle:
            raise ValueError("The maximum angle must be greater than the minimum.")
        if max_range_m <= 0.0:
            raise ValueError("The maximum range must be positive.")
        units_scaling = 1.0 if angle_units == "radians" else np.pi / 180

        # Discretize the cone arc, since Shapely polygons have no true arcs.
        # This is purely cosmetic; detection intersects the resulting polygon.
        angles = np.linspace(min_angle, max_angle, 33) * units_scaling
        arc_points = [
            (max_range_m * np.cos(angle), max_range_m * np.sin(angle))
            for angle in angles
        ]
        if (max_angle - min_angle) * units_scaling >= 2.0 * np.pi:
            # A full-circle field of view has no apex point.
            self.orig_fov_polygon = Polygon(arc_points)
        else:
            self.orig_fov_polygon = Polygon([(0.0, 0.0)] + arc_points)

        self.fov_polygon: Polygon | MultiPolygon = Polygon()
        self.fov_coords: list[Iterable[tuple[float, float]]] = []
        self.visible_objects: list["Object"] = []

    def update(self) -> None:
        """Performs the field of view calculation."""
        if (self.robot is None) or (self.robot.world is None):
            return

        # Clip the cone to the world's sensing region, keeping only the parts
        # in view of the robot so the field of view does not cross walls.
        robot_polygon = self.robot.polygon
        clipped = intersection(
            transform_polygon(self.orig_fov_polygon, self.robot.get_pose()),
            self.robot.world.total_sensing_polygon,
        )
        parts = [
            part
            for part in get_parts(clipped)
            if isinstance(part, Polygon) and intersects(robot_polygon, part)
        ]
        fov_polygon = parts[0] if len(parts) == 1 else MultiPolygon(parts)

        self.fov_polygon = fov_polygon
        self.fov_coords = [poly.exterior.coords for poly in parts]
        self.visible_objects = [
            obj
            for obj in self.robot.world.objects
            if ((obj.parent is None) or obj.parent.is_open)
            and obj.polygon.intersects(fov_polygon)
        ]

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

    def get_measurement(self) -> list["Object"]:
        """
        Gets the latest sensor measurement.

        :return: The list of world objects currently inside the field of view.
        """
        return self.visible_objects

    def get_display_polygons(self) -> Iterable[Iterable[tuple[float, float]]]:
        """
        Returns the field of view polygon rings to currently display.

        :return: The list of field of view rings, each a sequence of (x, y) points.
        """
        return self.fov_coords if self.is_active else []

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
            "max_range_m": self.max_range_m,
        }
