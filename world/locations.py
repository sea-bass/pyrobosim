"""
Representations for locations and their corresponding object spawns
"""

import yaml
import warnings
from shapely.affinity import rotate, translate
from shapely.geometry import Polygon, Point
from descartes.patch import PolygonPatch

from .utils import box_to_coords, get_polygon_centroid, inflate_polygon, Pose


class LocationMetadata:
    """ Represents metadata about locations """
    def __init__(self, filename):
        self.filename = filename
        with open(self.filename) as file:
            self.data = yaml.load(file, Loader=yaml.FullLoader)

    def get(self, category):
        return self.data[category]


class Location:
    """ Representation of a location in the world """
    viz_color = (0, 0, 0)

    @classmethod
    def set_metadata(cls, filename):
        cls.metadata = LocationMetadata(filename)

    def __init__(self, category, pose, name=None, parent=None):
        # Extract the model information from the model list
        self.name = name
        self.category = category
        self.pose = pose
        self.children = []
        self.parent = parent
        self.metadata = Location.metadata.get(self.category)

        # Calculate the object position and dimensions given the model properties
        if self.metadata["footprint_type"] == "polygon":
            coords = self.metadata["footprint"]
        elif self.metadata["footprint_type"] == "box":
            coords = box_to_coords(self.metadata["footprint"])
        self.polygon = Polygon(coords)
        self.polygon = rotate(self.polygon, self.pose.yaw, use_radians=True)
        self.polygon = translate(self.polygon, 
                                 xoff=self.pose.x, yoff=self.pose.y)
        self.centroid = get_polygon_centroid(self.polygon)

        # Update the collision and visualization polygons
        self.update_collision_polygon()
        self.update_visualization_polygon()


    def get_room_name(self):
        """ Returns the name of the containing room """
        if self.parent is None:
            return None
        else:
            return self.parent.get_room_name()


    def is_inside(self, pose):
        """ Checks if a pose is inside the location footprint """
        if isinstance(pose, Pose):
            p = Point(pose.x, pose.y)
        else:
            p = Point(pose[0], pose[1])
        return self.polygon.intersects(p)


    def update_collision_polygon(self, inflation_radius=0):
        """ Updates the collision polygon using the specified inflation radius """
        self.collision_polygon = inflate_polygon(self.polygon, inflation_radius)


    def update_visualization_polygon(self):
        """ Updates the visualization polygon for the furniture """
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.viz_patch = PolygonPatch(
                self.polygon,
                fill=None, ec=self.viz_color,
                lw=2, alpha=0.75, zorder=2)


    def __repr__(self):
        return f"Location: {self.name} in {self.parent}"
