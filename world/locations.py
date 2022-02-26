"""
Representations for locations and their corresponding object spawns
"""

import yaml
import warnings
from shapely.geometry import Point
from descartes.patch import PolygonPatch

from .utils import inflate_polygon, polygon_from_footprint, Pose


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
        if "color" in self.metadata:
            self.viz_color = self.metadata["color"]

        self.polygon = polygon_from_footprint(
            self.metadata["footprint"], pose=self.pose,
            parent_polygon=self.parent.polygon if self.parent is not None else None)
        
        # Add the spawn locations
        if "locations" in self.metadata:
            for loc_data in self.metadata["locations"]:
                if "name" in loc_data:
                    name = f"{self.name}_{loc_data['name']}"
                else:
                    name = f"{self.name}_loc{len(self.children)}"
                os = ObjectSpawn(name, loc_data, self)
                self.children.append(os)

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


class ObjectSpawn:
    """ Representation of an object spawn in the world """

    def __init__(self, name, metadata, parent=None):
        self.name = name
        self.category = parent.category
        self.parent = parent
        self.children = []

        self.metadata = metadata
        if "color" in self.metadata:
            self.viz_color = self.metadata["color"]
        else:
            self.viz_color = self.parent.viz_color

        if "footprint" not in self.metadata:
            self.metadata["footprint"] = {"type": "parent"}
        self.polygon = polygon_from_footprint(
            self.metadata["footprint"], pose=self.parent.pose,
            parent_polygon=self.parent.polygon if self.parent is not None else None)
        self.update_visualization_polygon()

    
    def get_room_name(self):
        """ Returns the name of the containing room """
        return self.parent.get_room_name()


    def update_visualization_polygon(self):
        """ Adds a visualization polygon for the object spawn """
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.viz_patch = PolygonPatch(
            self.polygon, fill=None, ec=self.parent.viz_color, 
            lw=1, ls="--", zorder=2)


    def __repr__(self):
        return f"Object spawn location: {self.name} in {self.parent.name}"
