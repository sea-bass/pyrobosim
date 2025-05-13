"""Representations for objects that exist in the world."""

from typing import Any, Sequence

import numpy as np
from matplotlib.patches import PathPatch
from matplotlib.text import Text
from shapely.geometry import Polygon
from shapely.plotting import patch_from_polygon
from scipy.spatial import ConvexHull

from .types import Entity, EntityMetadata, InvalidEntityCategoryException
from ..utils.general import parse_color
from ..utils.pose import Pose
from ..utils.polygon import (
    convhull_to_rectangle,
    inflate_polygon,
    polygon_and_height_from_footprint,
    transform_polygon,
)


class Object(Entity):
    """Represents an object in the world."""

    # Default class attributes
    metadata = EntityMetadata()
    """ Metadata for object categories. """
    height = 0.1
    """ Vertical height of object. """
    viz_color: Sequence[float] = (0.0, 0.0, 1.0)
    """ Visualization color (RGB tuple). """

    @classmethod
    def add_metadata(cls, filename: str) -> None:
        """
        Add object metadata from a new file to existing metadata.

        :param filename: Path to object metadata YAML file.
        """
        cls.metadata.add(filename)

    @classmethod
    def clear_metadata(cls) -> None:
        """
        Clear out old object metadata.
        """
        cls.metadata = EntityMetadata()

    def __init__(
        self,
        *,
        name: str,
        category: str,
        parent: Entity | None = None,
        pose: Pose = Pose(),
        inflation_radius: float = 0.0,
        color: Sequence[float] | str | None = None,
    ) -> None:
        """
        Creates an object instance.

        :param name: Name of the object.
        :param category: Object category (e.g., ``"apple"``).
        :param parent: Parent of the object (typically a :class:`pyrobosim.core.locations.ObjectSpawn`)
        :param pose: Pose of the object.
        :param inflation_radius: Inflation radius for polygon collision checks.
        :param color: Visualization color.
         Input can be:

         - an (R, G, B) tuple or list in the range (0.0, 1.0).
         - a string (e.g., "red").
         - a hexadecimal string (e.g., "#FF0000").
         If using a category with a defined color, this parameter overrides the category color.
        """
        super().__init__(name=name)
        self.parent = parent
        self.category = category

        self.inflation_radius = inflation_radius
        self.collision_polygon = Polygon()
        self.viz_patch: PathPatch | None = None
        self.viz_text: Text | None = None

        category_metadata = Object.metadata.get(category)
        if category_metadata is None:
            raise InvalidEntityCategoryException(f"Invalid object category: {category}")
        self.category_metadata = category_metadata

        if color is not None:
            self.viz_color = parse_color(color)
        elif "color" in self.category_metadata:
            self.viz_color = self.category_metadata["color"]

        self.set_pose(pose)
        self.create_polygons()
        self.create_grasp_cuboid()

    def set_pose(self, pose: Pose) -> None:
        """
        Sets the pose of an object, accounting for any height offsets in the target location,
        and update the corresponding object polygons.
        Use this instead of directly assigning the ``pose`` attribute.

        :param pose: New pose for the object.
        """
        self.pose = pose
        if self.parent is not None:
            self.pose.z += self.parent.height

    def create_polygons(self, inflation_radius: float | None = None) -> None:
        """
        Creates collision and visualization polygons for the object.
        If no inflation radius is specified, uses the inflation radius attribute
        set at construction time.

        :param inflation_radius: Inflation radius, in meters.
        """
        self.raw_polygon, height = polygon_and_height_from_footprint(
            self.category_metadata["footprint"]
        )
        self.polygon = transform_polygon(self.raw_polygon, self.pose)
        if height is not None:
            self.height = height
        self.centroid = list(self.polygon.centroid.coords)[0]
        self.update_collision_polygon(inflation_radius)
        self.update_visualization_polygon()

    def update_collision_polygon(self, inflation_radius: float | None = None) -> None:
        """
        Updates the collision polygon using the specified inflation radius.
        If no inflation radius is specified, uses the inflation radius attribute
        set at construction time.

        :param inflation_radius: Inflation radius, in meters.
        """
        radius = inflation_radius or self.inflation_radius
        self.raw_collision_polygon = inflate_polygon(self.raw_polygon, radius)
        self.collision_polygon = inflate_polygon(self.polygon, radius)

    def update_visualization_polygon(self) -> None:
        """Updates the visualization polygon for the object."""
        self.viz_patch = patch_from_polygon(
            self.polygon,
            facecolor=None,
            edgecolor=self.viz_color,
            linewidth=2,
            fill=None,
            alpha=0.75,
            zorder=4,
        )

    def get_footprint(self) -> np.ndarray:
        """
        Returns the object footprint coordinates.

        :return: N-by-2 array of object footprint XY coordinates
        """
        return np.array(list(self.raw_polygon.exterior.coords))

    def create_grasp_cuboid(self) -> None:
        """Fits a grasp cuboid from the object footprint and height."""
        # Fit a cuboid to the irregular object footprint
        footprint = self.get_footprint()
        hull = ConvexHull(footprint)
        hull_pts = footprint[hull.vertices, :]
        rect_pose, rect_dims, _ = convhull_to_rectangle(hull_pts)

        # Define the object dimension and pose
        self.cuboid_pose = rect_pose
        self.cuboid_dims = [rect_dims[0], rect_dims[1], self.height]

    def get_grasp_cuboid_pose(self) -> Pose:
        """
        Gets the cuboid pose with respect to the reference world frame.
        This is done by multiplying the object pose and the grasp cuboid relative pose.

        :return: A Pose object representing the pose of the grasp cuboid.
        """
        return Pose.from_transform(
            np.matmul(
                self.cuboid_pose.get_transform_matrix(),
                self.pose.get_transform_matrix(),
            )
        )

    def to_dict(self) -> dict[str, Any]:
        """
        Serializes the object to a dictionary.

        :return: A dictionary containing the object information.
        """
        obj_dict = {
            "name": self.name,
            "category": self.category,
            "pose": self.pose.to_dict(),
            "color": self.viz_color,
        }
        if self.parent is not None:
            obj_dict["parent"] = self.parent.name
        return obj_dict

    def __repr__(self) -> str:
        """Returns printable string."""
        return f"Object: {self.name}"

    def print_details(self) -> None:
        """Prints string with details."""
        obj_str = f"Object: {self.name}"
        if self.parent is not None:
            obj_str += f" in {self.parent.name}"
        obj_str += f"\n\t{self.pose}"
        print(obj_str)
