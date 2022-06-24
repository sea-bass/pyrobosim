""" Representations for objects that exist in the world. """

import warnings
from descartes.patch import PolygonPatch

from ..utils.general import EntityMetadata
from ..utils.polygon import inflate_polygon, polygon_and_height_from_footprint, transform_polygon


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

    def __init__(self, category=None, name=None, parent=None, pose=None):
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
        """
        self.category = category
        self.name = name
        self.parent = parent

        self.collision_polygon = None
        self.viz_patch = None
        self.viz_text = None

        self.metadata = Object.metadata.get(self.category)
        if "color" in self.metadata:
            self.viz_color = self.metadata["color"]
        self.set_pose(pose)
        self.create_polygons()

    def set_pose(self, pose):
        """
        Sets the pose of an object, accounting for any height offsets in the target location.
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


    def create_polygons(self, inflation_radius=0.0):
        """ 
        Creates collision and visualization polygons for the object. 

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
        

    def update_collision_polygon(self, inflation_radius=0.0):
        """
        Updates the collision polygon using the specified inflation radius.
        
        :param inflation_radius: Inflation radius, in meters.
        :type inflation_radius: float, optional
        """
        self.collision_polygon = inflate_polygon(
            self.polygon, inflation_radius)

    def update_visualization_polygon(self):
        """ Updates the visualization polygon for the object. """
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.viz_patch = PolygonPatch(
                self.polygon,
                fill=None, ec=self.viz_color,
                lw=2, alpha=0.75, zorder=3)

    def __repr__(self):
        """ Returns printable string. """
        return f"Object: {self.name}"

    def print_details(self):
        """ Prints string with details. """
        print(f"Object: {self.name} in {self.parent.name}\n\t{self.pose}")
