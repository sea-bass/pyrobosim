"""Representations for locations and their corresponding object spawns."""

from typing import Any, Sequence

from shapely.plotting import patch_from_polygon

from .types import Entity, EntityMetadata, InvalidEntityCategoryException
from ..utils.general import parse_color
from ..utils.graph_types import Node
from ..utils.pose import Pose, rot2d
from ..utils.polygon import (
    get_polygon_centroid,
    inflate_polygon,
    polygon_and_height_from_footprint,
    transform_polygon,
)


class Location(Entity):
    """Representation of a location in the world."""

    # Default class attributes
    metadata = EntityMetadata()
    """ Metadata for location categories. """
    height = 1.0
    """ Vertical height of location. """
    viz_color: Sequence[float] = (0.0, 0.0, 0.0)
    """ Visualization color (RGB tuple). """

    @classmethod
    def add_metadata(cls, filename: str) -> None:
        """
        Add location metadata from a new file to existing metadata.

        :param filename: Path to location metadata YAML file.
        """
        cls.metadata.add(filename)

    @classmethod
    def clear_metadata(cls) -> None:
        """
        Clear out old location metadata.
        """
        cls.metadata = EntityMetadata()

    def __init__(
        self,
        *,
        name: str,
        category: str,
        parent: Entity,
        pose: Pose = Pose(),
        color: Sequence[float] | str | None = None,
        is_open: bool = True,
        is_locked: bool = False,
        is_charger: bool = False,
    ) -> None:
        """
        Creates a location instance.

        :param name: Name of the location.
        :param category: Location category (e.g., ``"table"``).
        :param parent: Parent of the location (typically a :class:`pyrobosim.core.room.Room`)
        :param pose: Pose of the location (required).
        :param color: Visualization color.
         Input can be:

         - an (R, G, B) tuple or list in the range (0.0, 1.0).
         - a string (e.g., "red").
         - a hexadecimal string (e.g., "#FF0000").
         If using a category with a defined color, this parameter overrides the category color.
        :param is_open: If True, the location is open, otherwise it is closed.
        :param is_locked: If True, the location is locked, meaning it cannot be opened or closed.
        :param is_charger: If True, the robot charges its battery at this location.
        """
        # Extract the model information from the model list
        super().__init__(name=name)
        self.parent = parent
        self.category = category
        self.is_open = is_open

        category_metadata = Location.metadata.get(category)
        if not category_metadata:
            raise InvalidEntityCategoryException(
                f"Invalid location category: {category}"
            )
        self.category_metadata = category_metadata

        if color is not None:
            self.viz_color = parse_color(color)
        elif "color" in self.category_metadata:
            self.viz_color = self.category_metadata["color"]

        self.set_pose(pose)
        self.create_polygons()
        self.create_spawn_locations()
        self.set_open(is_open)
        self.is_locked = is_locked
        self.is_charger = is_charger

    def set_pose(self, pose: Pose) -> None:
        """
        Sets the pose of a location, accounting for its navigation poses and object spawns.
        Use this instead of directly assigning the ``pose`` attribute.

        :param pose: New pose for the location.
        """
        # Update the actual pose
        self.pose = pose

        # If navigation poses were specified, add them if they are collision free.
        self.nav_poses = []
        if "nav_poses" in self.category_metadata:
            if "offset" in self.category_metadata["footprint"]:
                p_off = self.category_metadata["footprint"]["offset"]
            else:
                p_off = (0.0, 0.0)
            for p in self.category_metadata["nav_poses"]:
                p = Pose.construct(p)
                rot_p = rot2d((p.x + p_off[0], p.y + p_off[1]), self.pose.get_yaw())
                nav_pose = Pose(
                    x=rot_p[0] + self.pose.x,
                    y=rot_p[1] + self.pose.y,
                    z=self.pose.z,
                    yaw=p.get_yaw() + self.pose.get_yaw(),
                )
                if (self.parent is not None) and (
                    self.parent.is_collision_free(nav_pose)
                ):
                    self.nav_poses.append(nav_pose)

    def create_polygons(self, inflation_radius: float = 0.0) -> None:
        """
        Creates collision and visualization polygons for the location.

        :param inflation_radius: Inflation radius, in meters.
        """
        self.raw_polygon, height = polygon_and_height_from_footprint(
            self.category_metadata["footprint"],
            parent_polygon=self.parent.polygon if self.parent is not None else None,
        )
        if height is not None:
            self.height = height
        self.polygon = transform_polygon(self.raw_polygon, self.pose)
        self.update_collision_polygon(inflation_radius=inflation_radius)
        self.update_visualization_polygon()

    def update_collision_polygon(self, inflation_radius: float = 0.0) -> None:
        """
        Updates the collision polygon using the specified inflation radius.

        :param inflation_radius: Inflation radius, in meters.
        """
        self.collision_polygon = inflate_polygon(self.polygon, inflation_radius)

    def update_visualization_polygon(self) -> None:
        """Updates the visualization polygon for the location."""
        self.viz_patch = patch_from_polygon(
            self.polygon,
            facecolor=None if self.is_open else self.viz_color,
            edgecolor=self.viz_color,
            linewidth=2,
            fill=not self.is_open,
            alpha=0.5,
            zorder=2,
        )

    def create_spawn_locations(self) -> None:
        """Creates the object spawn locations at this location."""
        self.children = []
        if "locations" in self.category_metadata:
            for loc_data in self.category_metadata["locations"]:
                if "name" in loc_data:
                    name = f"{self.name}_{loc_data['name']}"
                else:
                    name = f"{self.name}_loc{len(self.children)}"
                os = ObjectSpawn(name, loc_data, self)
                self.children.append(os)

    def add_graph_nodes(self) -> None:
        """Creates graph nodes for searching."""
        for spawn in self.children:
            if isinstance(spawn, ObjectSpawn):
                spawn.add_graph_nodes()

    def to_dict(self) -> dict[str, Any]:
        """
        Serializes the location to a dictionary.

        :return: A dictionary containing the location information.
        """
        loc_dict = {
            "name": self.name,
            "category": self.category,
            "pose": self.pose.to_dict(),
            "color": self.viz_color,
            "is_open": self.is_open,
            "is_locked": self.is_locked,
            "is_charger": self.is_charger,
        }
        if self.parent is not None:
            loc_dict["parent"] = self.parent.name
        return loc_dict

    def __repr__(self) -> str:
        """Returns printable string."""
        return f"Location: {self.name}"

    def print_details(self) -> None:
        """Prints string with details."""
        print(f"Location: {self.name} in {self.parent}\n\t{self.pose}")


class ObjectSpawn(Entity):
    """Representation of an object spawn in the world."""

    def __init__(
        self, name: str, category_metadata: dict[str, Any], parent: Entity | None = None
    ):
        """
        Creates an object spawn instance.

        :param name: Name of the location.
        :param category_metadata: Metadata dictionary for the parent category.
        :param parent: Parent of the location (typically a :class:`pyrobosim.core.locations.Location`)
        """
        super().__init__(name=name)
        assert parent is not None
        self.parent = parent
        self.category = parent.category
        self.set_open(parent.is_open, recursive=True)

        self.category_metadata = category_metadata
        if "color" in self.category_metadata:
            self.viz_color = self.category_metadata["color"]
        else:
            self.viz_color = self.parent.viz_color

        self.set_pose_from_parent()

    def set_pose_from_parent(self) -> None:
        """Updates the object spawn's pose from its parent's pose."""
        assert self.parent is not None

        # Get the footprint and height data
        if "footprint" not in self.category_metadata:
            self.category_metadata["footprint"] = {"type": "parent"}
        self.polygon, height = polygon_and_height_from_footprint(
            self.category_metadata["footprint"],
            pose=self.parent.pose,
            parent_polygon=self.parent.polygon,
        )
        self.height = height or self.parent.height

        self.update_visualization_polygon()
        self.centroid = get_polygon_centroid(self.polygon)
        self.pose = Pose(
            x=self.centroid[0], y=self.centroid[1], z=0.0, q=self.parent.pose.q
        )

        # If navigation poses were specified, add them. Else, use the parent poses.
        # Of course, only add these if they are collision-free.
        if "nav_poses" in self.category_metadata:
            self.nav_poses = []
            if "offset" in self.category_metadata["footprint"]:
                p_off = self.category_metadata["footprint"]["offset"]
            else:
                p_off = (0.0, 0.0)
            for p in self.category_metadata["nav_poses"]:
                p = Pose.construct(p)
                rot_p = rot2d(
                    (p.x + p_off[0], p.y + p_off[1]), self.parent.pose.get_yaw()
                )
                nav_pose = Pose(
                    x=rot_p[0] + self.parent.pose.x,
                    y=rot_p[1] + self.parent.pose.y,
                    z=self.parent.pose.z,
                    yaw=p.get_yaw() + self.parent.pose.get_yaw(),
                )
                room = self.parent.parent
                if (room is not None) and (room.is_collision_free(nav_pose)):
                    self.nav_poses.append(nav_pose)
        else:
            self.nav_poses = self.parent.nav_poses

    def update_visualization_polygon(self) -> None:
        """Updates the visualization polygon for the object spawn."""
        assert self.parent is not None
        self.viz_patch = patch_from_polygon(
            self.polygon,
            facecolor=None,
            edgecolor=self.parent.viz_color,
            linewidth=1,
            fill=None,
            ls="--",
            zorder=2,
        )

    def add_graph_nodes(self) -> None:
        """Creates graph nodes for searching."""
        self.graph_nodes = [Node(p, parent=self) for p in self.nav_poses]

    def __repr__(self) -> str:
        """Returns printable string."""
        return f"Object spawn: {self.name}"

    def print_details(self) -> None:
        """Prints string with details."""
        spawn_str = f"Object spawn: {self.name}"
        if self.parent is not None:
            spawn_str += f" in {self.parent.name}"
        spawn_str += f"\n\t{self.pose}"
        print(spawn_str)
