"""
Hallway Representation for World Modeling
"""

import warnings
import numpy as np
from shapely.geometry import Point, LineString
from descartes.patch import PolygonPatch

from .utils import get_bearing_range, inflate_polygon, Pose


class Hallway:
    """ Representation of a hallway connecting two rooms in a world """

    def __init__(self, room_start, room_end, hall_width,
                 conn_method="auto", offset=0,
                 conn_angle=0, conn_points=[],
                 color=[0.4, 0.4, 0.4], wall_width=0.2):
        """ Creates a Hallway object

        Arguments:
            room_start : Room object for the start of the hallway
            room_end : Room object for the end of the hallway
            hall_width : Width of the hallway
            conn_method : Connection method
                - auto : Draws straight line between centroids
                - angle : Directly specifies an angle
                - points : Specify a list of points
            offset : Perpendicular offset from centroid of start point
                     (valid if using "auto" or "angle" methods)
            conn_angle : If using "angle" connection method, specifies the angle of the hallway 
                         (0 is horizontal to right)
            conn_points: If using "points" connection method, specifies the hallway points
            color : Visualization color
            wall_width : Width of hallway walls
        """
        self.room_start = room_start
        self.room_end = room_end
        self.name = f"hall_{room_start.name}_{room_end.name}"
        self.hall_width = hall_width
        self.wall_width = wall_width
        self.offset = offset
        self.color = color

        # Parse the connection method
        # If the connection is "auto" or "angle", the hallway is a simple rectangle
        if conn_method == "auto" or conn_method == "angle":
            theta, length = get_bearing_range(
                room_start.centroid, room_end.centroid)
            if conn_method == "angle":
                length = length / np.cos(theta - conn_angle)
                theta = conn_angle

            # Calculate start and end points for the hallway
            c = np.cos(theta)
            s = np.sin(theta)
            x, y = room_start.centroid
            pt_start = [x - offset*s, y + offset*c]
            pt_end = [pt_start[0] + length*c,
                      pt_start[1] + length*s]
            points = [pt_start, pt_end]

        # If the connection is "points", the hallway is more complex
        elif conn_method == "points":
            points = conn_points

        else:
            raise Exception(f"No valid connection method: {conn_method}")

        # Create the hallway polygon
        self.polygon = LineString(points)
        self.polygon = inflate_polygon(self.polygon, hall_width/2.0)

        # Get the collision and visualization polygons
        self.update_collision_polygon()
        self.update_visualization_polygon()

    def update_collision_polygon(self, inflation_radius=0):
        """ Updates the collision polygon using the specified inflation radius """
        # Deflate the resulting difference polygon
        self.collision_polygon = inflate_polygon(
            self.polygon, -inflation_radius)
        # Subtract deflated room polygons from the hallway polygon
        self.collision_polygon = self.collision_polygon.difference(
            inflate_polygon(self.room_start.polygon, -inflation_radius))
        self.collision_polygon = self.collision_polygon.difference(
            inflate_polygon(self.room_end.polygon, -inflation_radius))

    def update_visualization_polygon(self):
        """ Updates the visualization polygon for the hallway walls """
        self.buffered_polygon = inflate_polygon(self.polygon, self.wall_width)
        self.viz_polygon = self.buffered_polygon.difference(self.polygon)
        self.viz_polygon = self.viz_polygon.difference(
            self.room_start.buffered_polygon)
        self.viz_polygon = self.viz_polygon.difference(
            self.room_end.buffered_polygon)
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
        return self.collision_polygon.intersects(p)

    def __repr__(self):
        return f"Hallway: Connecting {self.room_start.name} and {self.room_end.name}"
