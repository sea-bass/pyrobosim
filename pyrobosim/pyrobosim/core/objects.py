""" Representations for objects that exist in the world. """

import numpy as np
import warnings
from descartes.patch import PolygonPatch
from scipy.spatial import ConvexHull

from ..utils.general import EntityMetadata
from ..utils.polygon import (
    convhull_to_rectangle, inflate_polygon,
    polygon_and_height_from_footprint, transform_polygon
)


class Object:
    """ Represents an object in the world. """

    # Default class attributes
    height = 0.1
    """ Vertical height of location. """
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

    def __init__(self, category=None, name=None, parent=None, pose=None,
                 inflation_radius=0.0):
        """
        Creates an object instance.

        :param category: Object category (e.g., ``"apple"``).
        :type category: str
        :param name: Name of the location.
        :type name: str, optional
        :param parent: Parent of the location (typically a :class:`pyrobosim.core.room.Room`)
        :type parent: Entity
        :param pose: Pose of the location.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :param inflation_radius: Inflation radius for polygon collision checks.
        :type inflation_radius: float, optional
        """
        self.category = category
        self.name = name
        self.parent = parent

        self.inflation_radius = inflation_radius
        self.collision_polygon = None
        self.viz_patch = None
        self.viz_text = None

        self.metadata = Object.metadata.get(self.category)
        if "color" in self.metadata:
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
            self.metadata["footprint"])
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
        self.collision_polygon = inflate_polygon(self.polygon, radius)


    def update_visualization_polygon(self):
        """ Updates the visualization polygon for the object. """
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.viz_patch = PolygonPatch(
                self.polygon,
                fill=None, ec=self.viz_color,
                lw=2, alpha=0.75, zorder=3)


    def create_grasp_cuboid(self):
        """ Fits a grasp cuboid from the object footprint and height. """
        # Fit a cuboid to the irregular object footprint
        obj_footprint = np.array(list(self.raw_polygon.exterior.coords))
        hull = ConvexHull(obj_footprint)
        hull_pts = obj_footprint[hull.vertices, :]
        (rect_pose, rect_dims, _) = convhull_to_rectangle(hull_pts)

        # Define the object dimension and pose
        self.cuboid_pose = rect_pose
        self.cuboid_dims = [rect_dims[0], rect_dims[1], self.height]


    def __repr__(self):
        """ Returns printable string. """
        return f"Object: {self.name}"


    def print_details(self):
        """ Prints string with details. """
        print(f"Object: {self.name} in {self.parent.name}\n\t{self.pose}")
