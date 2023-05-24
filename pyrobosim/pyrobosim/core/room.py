"""
Room representation for world modeling.
"""

from shapely import intersects_xy
from shapely.geometry import Polygon
from shapely.plotting import patch_from_polygon

from ..utils.pose import Pose
from ..utils.polygon import inflate_polygon, polygon_and_height_from_footprint
from ..utils.search_graph import Node


class Room:
    """Representation of a room in a world."""

    def __init__(
        self,
        name=None,
        footprint=[],
        color=[0.4, 0.4, 0.4],
        wall_width=0.2,
        nav_poses=None,
        height=0.0,
    ):
        """
        Creates a Room instance.

        :param name: Room name.
        :type name: str, optional
        :param footprint: Point list or Shapely polygon describing the room 2D footprint (required).
        :type footprint: :class:`shapely.geometry.Polygon`/list[:class:`pyrobosim.utils.pose.Pose`]
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
        self.height = height
        if isinstance(footprint, list):
            self.polygon = Polygon(footprint)
        else:
            self.polygon, _ = polygon_and_height_from_footprint(footprint)
        if self.polygon.is_empty:
            raise Exception("Room footprint cannot be empty.")

        self.centroid = list(self.polygon.centroid.coords)[0]
        self.update_collision_polygons()
        self.update_visualization_polygon()

        # Create a navigation pose list -- if none specified, use the room centroid
        if nav_poses is not None:
            self.nav_poses = nav_poses
        else:
            self.nav_poses = [Pose.from_list(self.centroid)]

    def update_collision_polygons(self, inflation_radius=0):
        """
        Updates the collision polygons using the specified inflation radius.

        :param inflation_radius: Inflation radius, in meters.
        :type inflation_radius: float, optional
        """
        # Internal collision polygon:
        # Deflate the room polygon with the inflation radius and add each location's collision polygon.
        self.internal_collision_polygon = inflate_polygon(
            self.polygon, -inflation_radius
        )
        for loc in self.locations:
            self.internal_collision_polygon = (
                self.internal_collision_polygon.difference(loc.collision_polygon)
            )

        # External collision polygon:
        # Inflate the room polygon with the wall width
        self.external_collision_polygon = inflate_polygon(self.polygon, self.wall_width)

    def update_visualization_polygon(self):
        """Updates visualization polygon of the room walls."""
        self.buffered_polygon = inflate_polygon(self.polygon, self.wall_width)
        self.viz_polygon = self.buffered_polygon.difference(self.polygon)
        for h in self.hallways:
            self.viz_polygon = self.viz_polygon.difference(h.polygon)
        self.viz_patch = patch_from_polygon(
            self.viz_polygon,
            facecolor=self.viz_color,
            edgecolor=self.viz_color,
            linewidth=2,
            alpha=0.75,
            zorder=2,
        )

    def get_collision_patch(self):
        """
        Returns a patch of the collision polygon for debug visualization.

        :return: Polygon patch of the collision polygon.
        :rtype: :class:`matplotlib.patches.PathPatch`
        """
        return patch_from_polygon(
            self.internal_collision_polygon,
            facecolor=(1, 0, 1),
            edgecolor=(1, 0, 1),
            linewidth=2,
            alpha=0.5,
            zorder=2,
        )

    def is_collision_free(self, pose):
        """
        Checks whether a pose in the room is collision-free.

        :param pose: Pose to test.
        :type pose: :class:`pyrobosim.utils.pose.Pose`/(float, float)
        :return: True if collision-free, else False.
        :rtype: bool
        """
        if isinstance(pose, Pose):
            x, y = pose.x, pose.y
        else:
            x, y = pose[0], pose[1]
        return intersects_xy(self.internal_collision_polygon, x, y)

    def add_graph_nodes(self):
        """Creates graph nodes for searching."""
        self.graph_nodes = [Node(p, parent=self) for p in self.nav_poses]

    def __repr__(self):
        """Returns printable string."""
        return f"Room: {self.name}"
