"""
Room representation for world modeling.
"""

import warnings
from shapely.geometry import Polygon, Point
from descartes.patch import PolygonPatch

from ..navigation.search_graph import Node
from ..utils.pose import Pose
from ..utils.polygon import inflate_polygon, polygon_and_height_from_footprint


class Room:
    """ Representation of a room in a world. """
    def __init__(self, footprint, name=None, color=[0.4, 0.4, 0.4], wall_width=0.2, nav_poses=None, height=0.0):
        """ 
        Creates a Room instance.

        :param footprint: Point list or Shapely polygon describing the room 2D footprint.
        :type footprint: :class:`shapely.geometry.Polygon`/list[:class:`pyrobosim.utils.pose.Pose`]
        :param name: Room name
        :type name: str, optional
        :param color: Visualization color as an (R, G, B) tuple in the range (0.0, 1.0)
        :type color: (float, float, float), optional
        :param wall_width: Width of room walls, in meters.
        :type wall_width: float, optional
        :param nav_poses: List of navigation poses in the room. If not specified, defaults to the centroid.
        :type nav_poses: list[:class:`pyrobosim.utils.pose.Pose`]
        :param height: Height of room.
        :type height: float, optional
        """
        self.name = name
        self.wall_width = wall_width
        self.viz_color = color

        # Entities associated with the room
        self.hallways = []
        self.locations = []
        self.graph_nodes = []

        # Create the room polygon
        if isinstance(footprint, list):
            self.polygon = Polygon(footprint)
        else:
            self.polygon, _ = polygon_and_height_from_footprint(footprint)
        self.centroid = list(self.polygon.centroid.coords)[0]
        self.update_collision_polygons()
        self.update_visualization_polygon()

        # Create a navigation pose list -- if none specified, use the room centroid
        if nav_poses is not None:
            self.nav_poses = nav_poses
        else:
            self.nav_poses = [Pose.from_list(self.centroid)]
        self.height = height

    def update_collision_polygons(self, inflation_radius=0):
        """
        Updates the collision polygons using the specified inflation radius.
        
        :param inflation_radius: Inflation radius, in meters.
        :type inflation_radius: float, optional
        """
        # Internal collision polygon:
        # Deflate the room polygon with the inflation radius and add each location's collision polygon.
        self.internal_collision_polygon = inflate_polygon(
            self.polygon, -inflation_radius)
        for loc in self.locations:
            self.internal_collision_polygon = self.internal_collision_polygon.difference(
                loc.collision_polygon)

        # External collision polygon:
        # Inflate the room polygon with the wall width
        self.external_collision_polygon = inflate_polygon(
            self.polygon, self.wall_width)

    def update_visualization_polygon(self):
        """ Updates visualization polygon of the room walls. """
        self.buffered_polygon = inflate_polygon(self.polygon, self.wall_width)
        self.viz_polygon = self.buffered_polygon.difference(self.polygon)
        for h in self.hallways:
            self.viz_polygon = self.viz_polygon.difference(h.polygon)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.viz_patch = PolygonPatch(
                self.viz_polygon,
                fc=self.viz_color, ec=self.viz_color,
                lw=2, alpha=0.75, zorder=2)

    def get_collision_patch(self):
        """ 
        Returns a PolygonPatch of the collision polygon for debug visualization.
        
        :return: Polygon patch of the collision polygon.
        :rtype: :class:`descartes.patch.PolygonPatch`
        """
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            return PolygonPatch(
                self.internal_collision_polygon,
                fc=[1, 0, 1], ec=[1, 0, 1],
                lw=2, alpha=0.5, zorder=2)

    def is_collision_free(self, pose):
        """
        Checks whether a pose in the hallway is collision-free.
        
        :param pose: Pose to test.
        :type pose: :class:`pyrobosim.utils.pose.Pose`/(float, float)
        :return: True if collision-free, else False.
        :rtype: bool
        """
        if isinstance(pose, Pose):
            p = Point(pose.x, pose.y)
        else:
            p = Point(pose[0], pose[1])
        return self.internal_collision_polygon.intersects(p)

    def add_graph_nodes(self):
        """ Creates graph nodes for searching. """
        self.graph_nodes = [Node(p, parent=self) for p in self.nav_poses]

    def __repr__(self):
        """ Returns printable string. """
        return f"Room: {self.name}"
