""" Hallway representation for world modeling. """

import numpy as np
from shapely import intersects_xy
from shapely.geometry import LineString, MultiLineString
from shapely.plotting import patch_from_polygon

from ..utils.pose import Pose, get_angle, get_bearing_range
from ..utils.polygon import inflate_polygon
from ..utils.search_graph import Node


class Hallway:
    """Representation of a hallway connecting two rooms in a world."""

    def __init__(
        self,
        room_start=None,
        room_end=None,
        width=0.0,
        conn_method="auto",
        offset=0,
        conn_angle=0,
        conn_points=[],
        color=[0.4, 0.4, 0.4],
        wall_width=0.2,
        is_open=True,
        is_locked=False,
    ):
        """
        Creates a Hallway instance between two rooms.

        Hallways can be connected with a few different connection methods. These are:
            - ``"auto"`` : Draws straight line between room centroids (default)
            - ``"angle"`` : Directly specifies an angle leaving the centroid of the start room.
            - ``"points"`` : Specify an explicit list of points defining the hallway.

        :param room_start: Room object for the start of the hallway (required).
        :type room_start: :class:`pyrobosim.core.room.Room`
        :param room_end: Room object for the end of the hallway (required).
        :type room_end: :class:`pyrobosim.core.room.Room`
        :param width: Width of the hallway, in meters (required).
        :type width: float
        :param conn_method: Connection method (see description above).
        :type conn_method: str, optional
        :param offset: Perpendicular offset from centroid of start point
            (valid if using ``"auto"`` or ``"angle"`` connection methods)
        :type offset: float, optional
        :param conn_angle: If using ``"angle"`` connection method, specifies
            the angle of the hallway in radians (0 points to the right).
        :type conn_angle: float, optional
        :param conn_points: If using "points" connection method, specifies the hallway points.
        :type conn_points: list[(float, float)], optional
        :param color: Visualization color as an (R, G, B) tuple in the range (0.0, 1.0)
        :type color: (float, float, float), optional
        :param wall_width: Width of hallway walls, in meters.
        :type wall_width: float, optional
        :param is_open: If True, the hallway is open, otherwise it is closed.
        :type is_open: bool, optional
        :param is_locked: If True, the hallway is locked, meaning it cannot be opened or closed.
        :type is_locked: bool, optional
        """
        # Validate input
        if room_start is None:
            raise ValueError("room_start must be a valid Room object.")
        if room_end is None:
            raise ValueError("room_end must be a valid Room object.")
        if width <= 0.0:
            raise ValueError("width must be a positive value.")

        # Unpack input
        self.room_start = room_start
        self.room_end = room_end
        self.name = f"hall_{room_start.name}_{room_end.name}"
        self.width = width
        self.wall_width = wall_width
        self.offset = offset
        self.viz_color = color
        self.graph_nodes = []
        self.nav_poses = []
        self.is_open = is_open
        self.is_locked = is_locked
        self.height = 0.0  # For compatibility with PDDLStream costs

        # Parse the connection method
        # If the connection is "auto" or "angle", the hallway is a simple rectangle
        if conn_method == "auto" or conn_method == "angle":
            theta, length = get_bearing_range(room_start.centroid, room_end.centroid)
            if conn_method == "angle":
                length = length / np.cos(theta - conn_angle)
                theta = conn_angle

            # Calculate start and end points for the hallway
            c = np.cos(theta)
            s = np.sin(theta)
            x, y = room_start.centroid
            pt_start = [x - offset * s, y + offset * c]
            pt_end = [pt_start[0] + length * c, pt_start[1] + length * s]
            self.points = [pt_start, pt_end]

        # If the connection is "points", the hallway is more complex
        elif conn_method == "points":
            self.points = conn_points

        else:
            raise ValueError(f"No valid connection method: {conn_method}.")

        # Create the hallway polygon
        self.polygon = LineString(self.points)
        self.polygon = inflate_polygon(self.polygon, width / 2.0)

        # Get the collision and visualization polygons
        self.update_collision_polygons()
        self.update_visualization_polygon()

    def update_collision_polygons(self, inflation_radius=0.0):
        """
        Updates the collision polygons using the specified inflation radius.

        :param inflation_radius: Inflation radius, in meters.
        :type inflation_radius: float, optional
        """
        # Internal collision polygon:
        # Deflate the resulting difference polygon
        self.internal_collision_polygon = inflate_polygon(
            self.polygon, -inflation_radius
        )
        # Subtract deflated room polygons from the hallway polygon
        self.internal_collision_polygon = self.internal_collision_polygon.difference(
            inflate_polygon(self.room_start.polygon, -inflation_radius)
        )
        self.internal_collision_polygon = self.internal_collision_polygon.difference(
            inflate_polygon(self.room_end.polygon, -inflation_radius)
        )

        # External collision polygon:
        # Inflate the difference polygon by the wall width
        self.external_collision_polygon = inflate_polygon(self.polygon, self.wall_width)
        self.external_collision_polygon = self.external_collision_polygon.difference(
            self.room_start.external_collision_polygon
        )
        self.external_collision_polygon = self.external_collision_polygon.difference(
            self.room_end.external_collision_polygon
        )

        # Closed polygon:
        # Subtract the outer polygons of the adjacent rooms from the regular polygon
        self.closed_polygon = self.polygon.difference(
            self.room_start.external_collision_polygon
        )
        self.closed_polygon = self.closed_polygon.difference(
            self.room_end.external_collision_polygon
        )

    def update_visualization_polygon(self):
        """Updates the visualization polygon for the hallway walls."""
        self.buffered_polygon = inflate_polygon(self.polygon, self.wall_width)
        self.viz_polygon = self.buffered_polygon.difference(self.polygon)
        self.viz_polygon = self.viz_polygon.difference(self.room_start.buffered_polygon)
        self.viz_polygon = self.viz_polygon.difference(self.room_end.buffered_polygon)
        self.viz_patch = patch_from_polygon(
            self.viz_polygon,
            facecolor=self.viz_color,
            edgecolor=self.viz_color,
            linewidth=2,
            alpha=0.75,
            zorder=2,
        )

    def get_closed_patch(self):
        """
        Returns a patch of the hallway polygon to display when it is closed.

        :return: Polygon patch of the closed polygon.
        :rtype: :class:`matplotlib.patches.PathPatch`
        """
        return patch_from_polygon(
            self.closed_polygon,
            facecolor=self.viz_color,
            edgecolor=self.viz_color,
            linewidth=2,
            alpha=0.5,
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
        Checks whether a pose in the hallway is collision-free.

        :param pose: Pose to test.
        :type pose: :class:`pyrobosim.utils.pose.Pose`/(float, float)
        :return: True if collision-free, else False.
        :rtype: bool
        """
        if isinstance(pose, Pose):
            x, y = pose.x, pose.y
        else:
            x, y = pose[0], pose[1]

        is_free = intersects_xy(self.internal_collision_polygon, x, y)
        if not self.is_open:
            is_free = is_free and not intersects_xy(self.closed_polygon, x, y)
        return is_free

    def add_graph_nodes(self):
        """Creates graph nodes for searching."""
        intersect_line = LineString(self.points)
        intersect_line = intersect_line.difference(
            self.room_start.internal_collision_polygon
        )
        intersect_line = intersect_line.difference(
            self.room_end.internal_collision_polygon
        )

        if isinstance(intersect_line, LineString):
            self.graph_nodes = [
                Node(Pose(x=p[0], y=p[1]), parent=self) for p in intersect_line.coords
            ]
        elif isinstance(intersect_line, MultiLineString):
            self.graph_nodes = []
            for line in intersect_line.geoms:
                self.graph_nodes.extend(
                    [Node(Pose(x=p[0], y=p[1]), parent=self) for p in line.coords]
                )

        # Modify the yaw angles for the endpoint poses at the doors.
        door_pose_start_yaw = get_angle(
            (self.graph_nodes[0].pose.x, self.graph_nodes[0].pose.y),
            (self.graph_nodes[1].pose.x, self.graph_nodes[1].pose.y),
        )
        self.graph_nodes[0].pose.set_euler_angles(yaw=door_pose_start_yaw)
        door_pose_end_yaw = get_angle(
            (self.graph_nodes[-1].pose.x, self.graph_nodes[-1].pose.y),
            (self.graph_nodes[-2].pose.x, self.graph_nodes[-2].pose.y),
        )
        self.graph_nodes[-1].pose.set_euler_angles(yaw=door_pose_end_yaw)

        self.nav_poses = [self.graph_nodes[0].pose, self.graph_nodes[-1].pose]

    def __repr__(self):
        """Returns printable string."""
        return f"Hallway: {self.name}"

    def print_details(self):
        """Prints string with details."""
        open_str = "open" if self.is_open else "closed"
        locked_str = "locked" if self.is_locked else "unlocked"
        print(
            f"Hallway: Connecting {self.room_start.name} and {self.room_end.name} "
            + f"({open_str}, {locked_str})"
        )
