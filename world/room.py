"""
Room Representation for World Modeling
"""

import warnings
from shapely.geometry import Polygon, Point
from descartes.patch import PolygonPatch

from .utils import inflate_polygon, Pose


class Room:
    def __init__(self, coords, name=None, color=[0.4, 0.4, 0.4], wall_width=0.2):
        self.name = name
        self.wall_width = wall_width
        self.color = color

        # Entities associated with the room
        self.hallways = []
        self.locations = []

        # Create the room polygon
        self.polygon = Polygon(coords)
        self.centroid = list(self.polygon.centroid.coords)[0]
        self.update_collision_polygons()
        self.update_visualization_polygon()

    def update_collision_polygons(self, inflation_radius=0):
        """ Updates collision polygons using the specified inflation radius """
        # Internal collision polygon:
        # Deflate the room polygon with the inflation radius and add each location's collision polygon.
        self.internal_collision_polygon = inflate_polygon(
            self.polygon, -inflation_radius)
        for loc in self.locations:
            self.internal_collision_polygon = self.internal_collision_polygon.difference(
                loc.collision_polygon)

        # External collision polygon:
        # Inflate the room polygon with the wall width
        self.external_collision_polygon = inflate_polygon(self.polygon, self.wall_width)

    def update_visualization_polygon(self):
        """ Updates visualization polygon for world plotting """
        self.buffered_polygon = inflate_polygon(self.polygon, self.wall_width)
        self.viz_polygon = self.buffered_polygon.difference(self.polygon)
        for h in self.hallways:
            self.viz_polygon = self.viz_polygon.difference(h.polygon)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.viz_patch = PolygonPatch(
                self.viz_polygon,
                fc=self.color, ec=self.color,
                lw=2, alpha=0.75, zorder=2)

    def is_collision_free(self, pose):
        """ Checks whether a pose in the room is collision-free """
        if isinstance(pose, Pose):
            p = Point(pose.x, pose.y)
        else:
            p = Point(pose[0], pose[1])
        return self.internal_collision_polygon.intersects(p)

    def __repr__(self):
        return f"Room: {self.name}"
