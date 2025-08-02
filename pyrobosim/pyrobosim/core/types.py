"""
Core types for PyRoboSim.
"""

import os
from typing import Any, Sequence
from typing_extensions import Self  # For compatibility with Python <= 3.10

from matplotlib.patches import PathPatch
from shapely import intersects_xy, Polygon
import yaml

from ..utils.pose import Pose
from ..utils.graph_types import Node


class Entity:
    """
    Generic Entity class that helps with type hinting.

    Several classes in PyRoboSim (such as robots, rooms, etc.) subclass from this.
    """

    name = ""
    """The name of the entity."""
    category: str | None = None
    """The category of the entity."""
    pose = Pose()
    """The pose of the entity."""
    height = 0.0
    """The height (in the Z direction) of the entity."""
    polygon = Polygon()
    """The main polygon representing the entity."""
    collision_polygon = Polygon()
    """The collision polygon representing the entity."""
    internal_collision_polygon = Polygon()
    """The internal collision polygon representing the entity."""

    nav_poses: list[Pose] = []
    """The (optional) list of navigation poses for this entity."""
    graph_nodes: list[Node] = []
    """The (optional) list of search graph nodes for this entity."""

    is_open = True
    """Whether the entity is open or not."""
    is_locked = False
    """Whether the entity is locked or not."""
    is_charger = False
    """Whether the entity is a charger or not."""

    viz_patch: PathPatch | None = None
    """The visualization polygon patch for the entity."""
    viz_color: Sequence[float]
    """The color of the visualization polygon patch and text."""

    def __init__(self, *, name: str) -> None:
        """
        Construct a new Entity instance.

        :param name: A required entity name.
        """
        self.name = name
        self.parent: Entity | None = None
        self.children: list[Entity] = []

    def add_graph_nodes(self) -> None:
        """Creates graph nodes for searching."""
        self.graph_nodes = []

    def get_room_name(self) -> str | None:
        """
        Returns the name of the room containing the object.

        :return: The room name, if one exists, else None.
        """
        if self.parent is None:
            return None
        return self.parent.get_room_name()

    def is_collision_free(self, pose: Pose | Sequence[float]) -> bool:
        """
        Checks whether a pose is collision free in this entity.

        :return: True if collision free, else False.
        """
        return True

    def is_inside(self, pose: Pose | Sequence[float]) -> bool:
        """
        Checks if a pose is inside the entity polygon.

        :param pose: Pose to check.
        :return: True if pose is inside the polygon, else False.
        """
        if isinstance(pose, Pose):
            x, y = pose.x, pose.y
        else:
            x, y = pose[0], pose[1]
        return bool(intersects_xy(self.polygon, x, y))

    def set_open(self, state: bool, recursive: bool = True) -> None:
        """
        Helper function that sets the entity open or closed, including children.

        :param state: True if the location should be open, else false.
        :param recursive: If True (default), sets the state of all children recursively.
        """
        self.is_open = state
        if recursive:
            for child in self.children:
                child.set_open(state)


class EntityMetadata:
    """Represents metadata about entities, such as locations or objects."""

    def __init__(self, filename: str | None = None) -> None:
        """
        Creates metadata from a YAML file.

        :param filename: Path to metadata YAML file.
        """

        # Holds metadata in the form of a dictionary.
        self.data = self._load_metadata(filename)

        # List of file paths from which metadata has been loaded.
        self.sources: set[str] = set([filename]) if filename is not None else set()

    def _load_metadata(self, filename: str | None) -> dict[str, Any]:
        """
        Loads metadata from a YAML file

        :param filename: Path to metadata YAML file.
        """
        if filename is None:
            return {}

        if not os.path.isfile(filename):
            raise FileNotFoundError(f"Metadata filename not found: {filename}")

        with open(filename) as file:
            metadata = yaml.load(file, Loader=yaml.FullLoader)
            assert isinstance(metadata, dict)
            return metadata

    def has_category(self, category: str) -> bool:
        """
        Check whether a category name is in the metadata.

        :param category: Query category name.
        :return: True if the category is in the metadata, else False.
        """
        return category in self.data

    def get(self, category: str) -> dict[str, Any] | None:
        """
        Get metadata about a specific category.

        :param category: Query category name.
        :return: Category metadata dictionary if it exists, else None.
        """
        data = self.data.get(category)
        assert data is None or isinstance(data, dict)
        return data

    def add(self, filename: str) -> None:
        """
        Add metadata from a new YAML file to existing data.

        :param filename: Path to metadata YAML file.
        """
        new_data = self._load_metadata(filename)
        for key, value in new_data.items():
            if key in self.data and self.data[key] != value:
                raise MetadataConflictException(key, self.data[key], value, filename)
            self.data[key] = value

        self.sources.add(filename)

    def get_categories(self) -> list[str]:
        """
        Gets the list of categories in this metadata.

        :return: A list of category names for this metadata.
        """
        return list(self.data.keys())


class InvalidEntityCategoryException(Exception):
    """Raised when an invalid entity metadata category is used."""


class MetadataConflictException(Exception):
    """
    Raised when a conflict occurs while adding metadata.

    This exception is raised when a key in the metadata already exists
    with a value that conflicts with the new value being added
    """

    def __init__(
        self, key: str, old_value: Any, new_value: Any, source: str | None = None
    ) -> None:
        """
        Creates an MetadataConflictException instance.

        :param key: Conflicting metadata key.
        :param old_value: Existing value for key.
        :param new_value: New value for key.
        :param source: Source of new metadata.
        """
        message = f"Conflict for key '{key}': existing value '{old_value}' conflicts with new value '{new_value}'"
        if source:
            message += f" from source '{source}'"
        super().__init__(message)


def set_parent(this: Entity, other: Entity) -> None:
    """
    Helper function to cleanly set the parent of an entity.

    :param this: The entity whose parent to modify.
    :param other: The new parent of the entity.
    """
    if (this.parent is not None) and (this in this.parent.children):
        this.parent.children.remove(this)

    this.parent = other
    other.children.append(this)
