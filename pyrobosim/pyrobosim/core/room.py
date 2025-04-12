"""
Room representation for world modeling.
"""

from typing import Any, Sequence

from matplotlib.patches import PathPatch
from shapely import intersects_xy
from shapely.geometry import Polygon
from shapely.plotting import patch_from_polygon

from .locations import Location
from .types import Entity
from ..utils.graph_types import Node
from ..utils.pose import Pose
from ..utils.polygon import (
    inflate_polygon,
    polygon_and_height_from_footprint,
    transform_polygon,
)
from ..utils.search_graph import Node
from ..utils.general import parse_color


class Room(Entity):
    """Representation of a room in a world."""

    def __init__(
        self,
        *,
        name: str,
        footprint: Sequence[Sequence[float]] | dict[str, Any] = [],
        color: str | Sequence[float] = [0.4, 0.4, 0.4],
        wall_width: float = 0.2,
        pose: Pose | None = None,
        nav_poses: list[Pose] | None = None,
        height: float = 0.0,
    ) -> None:
        """
        Creates a Room instance.

        :param name: Room name.
        :param footprint: Point list or Shapely polygon describing the room 2D footprint (required).
        :param color: Visualization color.
         Input can be:

         - an (R, G, B) tuple or list in the range (0.0, 1.0).
         - a string (e.g., "red").
         - a hexadecimal string (e.g., "#FF0000").
        :param wall_width: Width of room walls, in meters.
        :param pose: Pose of the room. This transforms the specified footprint.
            If set to None, the pose will be the centroid of the room polygon.
        :param nav_poses: List of navigation poses in the room. If not specified, defaults to the centroid.
        :param height: Height of room.
        """
        from .hallway import Hallway  # Avoids circular import

        super().__init__(name=name)
        self.wall_width = wall_width
        self.viz_color = parse_color(color)

        # Entities associated with the room
        self.hallways: list[Hallway] = []
        self.locations: list[Location] = []
        self.graph_nodes: list[Node] = []

        # Create the room polygon
        self.height = height
        if isinstance(footprint, list):
            self.polygon = Polygon(footprint)
            self.footprint = {"type": "polygon", "coords": footprint}
        elif isinstance(footprint, dict):
            self.polygon, out_height = polygon_and_height_from_footprint(footprint)
            self.footprint = footprint
            if out_height is not None:
                self.height = out_height

        if self.polygon.is_empty:
            raise RuntimeError("Room footprint cannot be empty.")

        self.original_pose = pose  # Needed to serialize the world properly.
        if pose is not None:
            self.pose = pose
            self.polygon = transform_polygon(self.polygon, self.pose)

        self.centroid = list(self.polygon.centroid.coords)[0]
        centroid_pose = Pose.from_list(self.centroid)
        if pose is None:
            self.pose = centroid_pose

        self.update_collision_polygons()
        self.update_visualization_polygon()

        # Create a navigation pose list -- if none specified, use the room centroid.
        if nav_poses is not None:
            self.nav_poses = nav_poses
        else:
            self.nav_poses = [centroid_pose]

    def update_collision_polygons(self, inflation_radius: float = 0.0) -> None:
        """
        Updates the collision polygons using the specified inflation radius.

        :param inflation_radius: Inflation radius, in meters.
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

    def update_visualization_polygon(self) -> None:
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

    def get_collision_patch(self) -> PathPatch:
        """
        Returns a patch of the collision polygon for debug visualization.

        :return: Polygon patch of the collision polygon.
        """
        return patch_from_polygon(
            self.internal_collision_polygon,
            facecolor=(1, 0, 1),
            edgecolor=(1, 0, 1),
            linewidth=2,
            alpha=0.5,
            zorder=2,
        )

    def get_room_name(self) -> str:
        """
        Overrides the entity implementation by getting the name of this room.

        :return: The name of this room.
        """
        return self.name

    def is_collision_free(self, pose: Pose | Sequence[float]) -> bool:
        """
        Checks whether a pose in the room is collision-free.

        :param pose: Pose to test.
        :return: True if collision-free, else False.
        """
        if isinstance(pose, Pose):
            x, y = pose.x, pose.y
        else:
            x, y = pose[0], pose[1]
        return bool(intersects_xy(self.internal_collision_polygon, x, y))

    def add_graph_nodes(self) -> None:
        """Creates graph nodes for searching."""
        self.graph_nodes = [Node(p, parent=self) for p in self.nav_poses]

    def to_dict(self) -> dict[str, Any]:
        """
        Serializes the room to a dictionary.

        :return: A dictionary containing the room information.
        """
        room_dict = {
            "name": self.name,
            "color": self.viz_color,
            "wall_width": self.wall_width,
            "footprint": self.footprint,
            "height": self.height,
            "nav_poses": [pose.to_dict() for pose in self.nav_poses],
        }
        if self.original_pose is not None:
            room_dict["pose"] = self.original_pose.to_dict()
        return room_dict

    def __repr__(self) -> str:
        """Returns printable string."""
        return f"Room: {self.name}"
