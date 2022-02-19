"""
Representations for locations and their corresponding object spawns
"""

import yaml
import warnings
from shapely.geometry import Polygon, Point
from descartes.patch import PolygonPatch

from .utils import (
    box_to_coords, get_polygon_centroid, inflate_polygon, 
    transform_polygon, Pose)


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

        if "footprint_offset" in self.metadata:
            offset_vec = self.metadata["footprint_offset"]
            if len(offset_vec) == 2:
                offset_pose = Pose(x=offset_vec[0], y=offset_vec[1])
            elif len(offset_vec) == 3:
                offset_pose = Pose(x=offset_vec[0], y=offset_vec[1], yaw=offset_vec[2])
            self.polygon = transform_polygon(self.polygon, offset_pose)

        self.polygon = transform_polygon(self.polygon, self.pose)
        
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

        if "footprint" not in self.metadata:
            footprint_type = "parent"
        else:
            footprint_type = self.metadata["footprint_type"]

        if footprint_type == "parent":
            self.polygon = self.parent.polygon
            if "footprint_padding" in self.metadata:
                self.polygon = inflate_polygon(
                    self.polygon, -self.metadata["footprint_padding"])
        else:
            if footprint_type == "polygon":
                coords = self.metadata["footprint"]
            elif footprint_type == "box":
                coords = box_to_coords(self.metadata["footprint"])
            self.polygon = Polygon(coords)
            if "footprint_offset" in self.metadata:
                offset_vec = self.metadata["footprint_offset"]
                if len(offset_vec) == 2:
                    offset_pose = Pose(x=offset_vec[0], y=offset_vec[1])
                elif len(offset_vec) == 3:
                    offset_pose = Pose(x=offset_vec[0], y=offset_vec[1], yaw=offset_vec[2])
                self.polygon = transform_polygon(self.polygon, offset_pose)
            self.polygon = transform_polygon(self.polygon, self.parent.pose)

        self.update_visualization_polygon()

    
    def get_room_name(self):
        """ Returns the name of the containing room """
        return self.parent.get_room_name()


    def update_visualization_polygon(self):
        """ Adds a visualization polygon for the object spawn """
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.viz_patch = PolygonPatch(
            self.polygon, fill=None, ec="k", 
            lw=1, ls="--", zorder=2)


    def __repr__(self):
        return f"Object spawn location: {self.name} in {self.parent}"
