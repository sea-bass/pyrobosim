""" Representations for objects that exist in the world. """

import numpy as np
from shapely.plotting import patch_from_polygon
from scipy.spatial import ConvexHull

from ..utils.general import EntityMetadata, InvalidEntityCategoryException
from ..utils.pose import Pose
from ..utils.polygon import (
    convhull_to_rectangle,
    inflate_polygon,
    polygon_and_height_from_footprint,
    transform_polygon,
)


class Object:
    """Represents an object in the world."""

    # Default class attributes
    height = 0.1
    """ Vertical height of object. """
    viz_color = (0, 0, 1)
    """ Visualization color (RGB tuple). """

    @classmethod
    def set_metadata(cls, filename):
        """
        Assign a metadata file to the :class:`pyrobosim.core.objects.Object` class.

        :param filename: Path to object metadata YAML file.
        :type filename: str
        """
        cls.metadata = EntityMetadata(filename)

    def __init__(
        self,
        name=None,
        category=None,
        parent=None,
        pose=None,
        inflation_radius=0.0,
        color=None,
    ):
        """
        Creates an object instance.

        :param name: Name of the object.
        :type name: str, optional
        :param category: Object category (e.g., ``"apple"``).
        :type category: str
        :param parent: Parent of the object (typically a :class:`pyrobosim.core.locations.ObjectSpawn`)
        :type parent: Entity
        :param pose: Pose of the object.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :param inflation_radius: Inflation radius for polygon collision checks.
        :type inflation_radius: float, optional
        :param color: Visualization color as an (R, G, B) tuple in the range (0.0, 1.0).
            If using a category with a defined color, this parameter overrides the category color.
        :type color: (float, float, float), optional
        """
        self.category = category
        self.name = name
        self.parent = parent

        self.inflation_radius = inflation_radius
        self.collision_polygon = None
        self.viz_patch = None
        self.viz_text = None

        self.metadata = Object.metadata.get(self.category)
        if not self.metadata:
            raise InvalidEntityCategoryException(
                f"Invalid object category: {self.category}"
            )

        if color is not None:
            self.viz_color = color
        elif "color" in self.metadata:
            self.viz_color = self.metadata["color"]

        self.set_pose(pose)
        self.create_polygons()
        self.create_grasp_cuboid()

    def set_pose(self, pose):
        """
        Sets the pose of an object, accounting for any height offsets in the target location,
        and update the corresponding object polygons.
        Use this instead of directly assigning the ``pose`` attribute.

        :param pose: New pose for the object.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        """
        self.pose = pose
        if self.pose is not None and self.parent is not None:
            self.pose.z += self.parent.height

    def get_room_name(self):
        """
        Returns the name of the room containing the object.

        :return: Room name.
        :rtype: str
        """
        return self.parent.get_room_name()

    def create_polygons(self, inflation_radius=None):
        """
        Creates collision and visualization polygons for the object.
        If no inflation radius is specified, uses the inflation radius attribute
        set at construction time.

        :param inflation_radius: Inflation radius, in meters.
        :type inflation_radius: float, optional
        """
        self.raw_polygon, height = polygon_and_height_from_footprint(
            self.metadata["footprint"]
        )
        self.polygon = transform_polygon(self.raw_polygon, self.pose)
        if height is not None:
            self.height = height
        self.centroid = list(self.polygon.centroid.coords)[0]
        self.update_collision_polygon(inflation_radius)
        self.update_visualization_polygon()

    def update_collision_polygon(self, inflation_radius=None):
        """
        Updates the collision polygon using the specified inflation radius.
        If no inflation radius is specified, uses the inflation radius attribute
        set at construction time.

        :param inflation_radius: Inflation radius, in meters.
        :type inflation_radius: float, optional
        """
        if inflation_radius is not None:
            radius = inflation_radius
        else:
            radius = self.inflation_radius
        self.raw_collision_polygon = inflate_polygon(self.raw_polygon, radius)
        self.collision_polygon = inflate_polygon(self.polygon, radius)

    def update_visualization_polygon(self):
        """Updates the visualization polygon for the object."""
        self.viz_patch = patch_from_polygon(
            self.polygon,
            facecolor=None,
            edgecolor=self.viz_color,
            linewidth=2,
            fill=None,
            alpha=0.75,
            zorder=3,
        )

    def get_footprint(self):
        """
        Returns the object footprint coordinates.

        :return: N-by-2 array of object footprint XY coordinates
        :rtype: :class:`numpy.ndarray`
        """
        return np.array(list(self.raw_polygon.exterior.coords))

    def create_grasp_cuboid(self):
        """Fits a grasp cuboid from the object footprint and height."""
        # Fit a cuboid to the irregular object footprint
        footprint = self.get_footprint()
        hull = ConvexHull(footprint)
        hull_pts = footprint[hull.vertices, :]
        (rect_pose, rect_dims, _) = convhull_to_rectangle(hull_pts)

        # Define the object dimension and pose
        self.cuboid_pose = rect_pose
        self.cuboid_dims = [rect_dims[0], rect_dims[1], self.height]

    def get_grasp_cuboid_pose(self):
        """
        Gets the cuboid pose with respect to the reference world frame.
        This is done by multiplying the object pose and the grasp cuboid relative pose.

        :return: A Pose object representing the pose of the grasp cuboid.
        :rtype: :class:`pyrobosim.utils.pose.Pose`
        """
        return Pose.from_transform(
            np.matmul(
                self.cuboid_pose.get_transform_matrix(),
                self.pose.get_transform_matrix(),
            )
        )

    def __repr__(self):
        """Returns printable string."""
        return f"Object: {self.name}"

    def print_details(self):
        """Prints string with details."""
        print(f"Object: {self.name} in {self.parent.name}\n\t{self.pose}")
