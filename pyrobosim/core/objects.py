"""
Representations for objects that exist in the world
"""

import yaml
import warnings
from descartes.patch import PolygonPatch

from ..utils.polygon import inflate_polygon, polygon_and_height_from_footprint


class ObjectMetadata:
    """ Represents metadata about objects """

    def __init__(self, filename):
        self.filename = filename
        with open(self.filename) as file:
            self.data = yaml.load(file, Loader=yaml.FullLoader)

    def has_category(self, category):
        return category in self.data

    def get(self, category):
        return self.data[category]


class Object:
    """ Represents an object in the world """
    height = 0.1
    viz_color = (0, 0, 1)

    @classmethod
    def set_metadata(cls, filename):
        cls.metadata = ObjectMetadata(filename)

    def __init__(self, category=None, name=None, parent=None, pose=None):
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
        """ Sets the pose of an object """
        self.pose = pose
        if self.pose is not None and self.parent is not None:
            self.pose.z += self.parent.height

    def get_room_name(self):
        """ Returns the name of the containing room """
        return self.parent.get_room_name()

    def get_raw_polygon(self):
        """ Gets the raw polygon (without any pose offset) """
        return polygon_and_height_from_footprint(self.metadata["footprint"])[0]

    def create_polygons(self):
        self.polygon, height = polygon_and_height_from_footprint(
            self.metadata["footprint"], pose=self.pose)
        if height is not None:
            self.height = height
        self.centroid = list(self.polygon.centroid.coords)[0]
        self.update_collision_polygon()
        self.update_visualization_polygon()

    def update_collision_polygon(self, inflation_radius=0):
        """ Updates the collision polygon using the specified inflation radius """
        self.collision_polygon = inflate_polygon(
            self.polygon, inflation_radius)

    def update_visualization_polygon(self):
        """ Updates the visualization polygon for the furniture """
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.viz_patch = PolygonPatch(
                self.polygon,
                fill=None, ec=self.viz_color,
                lw=2, alpha=0.75, zorder=3)

    def __repr__(self):
        return f"Object: {self.name} in {self.parent.name}\n\t{self.pose}"
