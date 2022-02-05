"""
Room Representation for World Modeling
"""

from shapely.geometry import Polygon, Point
from descartes.patch import PolygonPatch

from .utils import inflate_polygon, Pose

class Room:
    def __init__(self, coords, name=None, color=[0.4, 0.4, 0.4], wall_width=0.2):
        self.name = name
        self.wall_width = wall_width
        self.color = color

        # Create the room polygon
        self.polygon = Polygon(coords)
        self.centroid = list(self.polygon.centroid.coords)[0]
        self.update_visualization_polygon()

    def update_collision_polygon(self, inflation_radius=0):
        """ Updates collision polygon using the specified inflation radius """
        # Deflate the room polygon with the inflation radius
        self.collision_polygon = inflate_polygon(self.polygon, -inflation_radius)

        # Subtract floor spawns and furniture collision polygons
        # for fs in self.floor_spawns:
        #     self.collision_polygon = self.collision_polygon.difference(
        #         fs.collision_polygon)
        # for frn in self.furniture:
        #     self.collision_polygon = self.collision_polygon.difference(
        #         frn.collision_polygon)

    def update_visualization_polygon(self):
        """ Updates visualization polygon for world plotting """
        self.buffered_polygon = inflate_polygon(self.polygon, self.wall_width)
        self.viz_polygon = self.buffered_polygon.difference(self.polygon)
        # for h in self.halls:
        #     self.viz_polygon = self.viz_polygon.difference(h.polygon)
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
        return self.collision_polygon.intersects(p)
