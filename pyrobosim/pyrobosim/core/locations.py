""" Representations for locations and their corresponding object spawns. """

import warnings
from shapely.geometry import Point
from descartes.patch import PolygonPatch

from ..navigation.search_graph import Node
from ..utils.general import EntityMetadata
from ..utils.pose import Pose, rot2d
from ..utils.polygon import inflate_polygon, polygon_and_height_from_footprint, transform_polygon


class Location:
    """ Representation of a location in the world. """

    # Default class attributes
    height = 1.0
    """ Vertical height of location. """
    viz_color = (0, 0, 0)
    """ Visualization color (RGB tuple). """

    @classmethod
    def set_metadata(cls, filename):
        """ 
        Assign a metadata file to the :class:`pyrobosim.core.locations.Location` class.
        
        :param filename: Path to location metadata YAML file.
        :type filename: str
        """
        cls.metadata = EntityMetadata(filename)

    def __init__(self, category, pose, name=None, parent=None):
        """
        Creates a location instance.

        :param category: Location category (e.g., ``"table"``).
        :type category: str
        :param pose: Pose of the location.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :param name: Name of the location.
        :type name: str, optional
        :param parent: Parent of the location (typically a :class:`pyrobosim.core.room.Room`)
        :type parent: Entity
        """
        # Extract the model information from the model list
        self.name = name
        self.category = category
        self.parent = parent

        self.metadata = Location.metadata.get(self.category)
        if "color" in self.metadata:
            self.viz_color = self.metadata["color"]

        self.set_pose(pose)
        self.create_polygons()
        self.create_spawn_locations()
        
    def get_room_name(self):
        """ 
        Returns the name of the room containing the location.
        
        :return: Room name.
        :rtype: str
        """
        if self.parent is None:
            return None
        else:
            return self.parent.name

    def is_inside(self, pose):
        """ 
        Checks if a pose is inside the location polygon.
        
        :param pose: Pose to check.
        :type pose: :class:`pyrobosim.utils.pose.Pose`/(float, float)
        :return: True if pose is inside the polygon, else False.
        :rtype: bool
        """
        if isinstance(pose, Pose):
            p = Point(pose.x, pose.y)
        else:
            p = Point(pose[0], pose[1])
        return self.polygon.intersects(p)

    def set_pose(self, pose):
        """
        Sets the pose of a location, accounting for its navigation poses and object spawns.
        Use this instead of directly assigning the ``pose`` attribute.
        
        :param pose: New pose for the object.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        """
        # Update the actual pose
        self.pose = pose

        # If navigation poses were specified, add them if they are collision free.
        self.nav_poses = []
        if "nav_poses" in self.metadata:
            if "offset" in self.metadata["footprint"]:
                p_off = self.metadata["footprint"]["offset"]
            else:
                p_off = (0, 0)
            for p in self.metadata["nav_poses"]:
                rot_p = rot2d((p[0] + p_off[0], p[1] + p_off[1]),
                              self.pose.yaw)
                nav_pose = Pose(x=rot_p[0] + self.pose.x,
                                y=rot_p[1] + self.pose.y,
                                yaw=p[2] + self.pose.yaw)
                if self.parent.is_collision_free(nav_pose):
                    self.nav_poses.append(nav_pose)

    def create_polygons(self, inflation_radius=0.0):
        """ 
        Creates collision and visualization polygons for the location. 

        :param inflation_radius: Inflation radius, in meters.
        :type inflation_radius: float, optional
        """
        self.raw_polygon, self.height = polygon_and_height_from_footprint(
            self.metadata["footprint"],
            parent_polygon=self.parent.polygon if self.parent is not None else None)
        self.polygon = transform_polygon(self.raw_polygon, self.pose)
        self.update_collision_polygon(inflation_radius=inflation_radius)
        self.update_visualization_polygon()

    def update_collision_polygon(self, inflation_radius=0.0):
        """
        Updates the collision polygon using the specified inflation radius.
        
        :param inflation_radius: Inflation radius, in meters.
        :type inflation_radius: float, optional
        """
        self.collision_polygon = inflate_polygon(
            self.polygon, inflation_radius)

    def update_visualization_polygon(self):
        """ Updates the visualization polygon for the location. """
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.viz_patch = PolygonPatch(
                self.polygon,
                fill=None, ec=self.viz_color,
                lw=2, alpha=0.75, zorder=2)

    def create_spawn_locations(self):
        """ Creates the object spawn locations at this location. """
        self.children = []
        if "locations" in self.metadata:
            for loc_data in self.metadata["locations"]:
                if "name" in loc_data:
                    name = f"{self.name}_{loc_data['name']}"
                else:
                    name = f"{self.name}_loc{len(self.children)}"
                os = ObjectSpawn(name, loc_data, self)
                self.children.append(os)

    def add_graph_nodes(self):
        """ Creates graph nodes for searching. """
        for spawn in self.children:
            spawn.add_graph_nodes()

    def __repr__(self):
        """ Returns printable string. """
        return f"Location: {self.name}"

    def print_details(self):
        """ Prints string with details. """
        print(f"Location: {self.name} in {self.parent}\n\t{self.pose}")


class ObjectSpawn:
    """ Representation of an object spawn in the world. """

    def __init__(self, name, metadata, parent=None):
        """
        Creates an object spawn instance.

        :param name: Name of the location.
        :type name: str, optional
        :param metadata: Metadata dictionary for the object spawn
        :type metadata: dict
        :param parent: Parent of the location (typically a :class:`pyrobosim.core.locations.Location`)
        :type parent: Entity
        """
        self.name = name
        self.category = parent.category
        self.parent = parent
        self.children = []
        self.graph_nodes = []

        self.metadata = metadata
        if "color" in self.metadata:
            self.viz_color = self.metadata["color"]
        else:
            self.viz_color = self.parent.viz_color

        self.set_pose_from_parent()

    def set_pose_from_parent(self):
        """ Updates the object spawn's pose from its parent's pose. """
        # Get the footprint and height data
        if "footprint" not in self.metadata:
            self.metadata["footprint"] = {"type": "parent"}
        self.polygon, self.height = polygon_and_height_from_footprint(
            self.metadata["footprint"], pose=self.parent.pose,
            parent_polygon=self.parent.polygon if self.parent is not None else None)
        if self.height is None:
            self.height = self.parent.height

        self.update_visualization_polygon()
        self.centroid = list(self.polygon.centroid.coords)[0]
        self.pose = Pose(
            x=self.centroid[0], y=self.centroid[1], yaw=self.parent.pose.yaw)

        # If navigation poses were specified, add them. Else, use the parent poses.
        # Of course, only add these if they are collision-free.
        if "nav_poses" in self.metadata:
            self.nav_poses = []
            if "offset" in self.metadata["footprint"]:
                p_off = self.metadata["footprint"]["offset"]
            else:
                p_off = (0, 0)
            for p in self.metadata["nav_poses"]:
                rot_p = rot2d((p[0] + p_off[0], p[1] + p_off[1]),
                              self.parent.pose.yaw)
                nav_pose = Pose(x=rot_p[0] + self.parent.pose.x,
                                y=rot_p[1] + self.parent.pose.y,
                                yaw=p[2] + self.parent.pose.yaw)
                if self.parent.parent.is_collision_free(nav_pose):
                    self.nav_poses.append(nav_pose)
        else:
            self.nav_poses = self.parent.nav_poses

    def get_room_name(self):
        """
        Returns the name of the room containing the object spawn.
        
        :return: Room name.
        :rtype: str
        """
        return self.parent.get_room_name()

    def is_inside(self, pose):
        """
        Checks if a pose is inside the object spawn polygon.
        
        :param pose: Pose to check.
        :type pose: :class:`pyrobosim.utils.pose.Pose`/(float, float)
        :return: True if pose is inside the polygon, else False.
        :rtype: bool
        """
        if isinstance(pose, Pose):
            p = Point(pose.x, pose.y)
        else:
            p = Point(pose[0], pose[1])
        return self.polygon.intersects(p)

    def update_visualization_polygon(self):
        """ Updates the visualization polygon for the object spawn. """
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.viz_patch = PolygonPatch(
                self.polygon, fill=None, ec=self.parent.viz_color,
                lw=1, ls="--", zorder=2)

    def add_graph_nodes(self):
        """ Creates graph nodes for searching. """
        self.graph_nodes = [Node(p, parent=self) for p in self.nav_poses]

    def __repr__(self):
        """ Returns printable string. """
        return f"Object spawn: {self.name}"

    def print_details(self):
        """ Prints string with details. """
        print(f"Object spawn: {self.name} in {self.parent.name}\n\t{self.pose}")

