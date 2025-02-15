"""
Core types for PyRoboSim.
"""

import os
from typing import Any

from matplotlib.patches import PathPatch
import yaml

from ..utils.pose import Pose
from ..utils.graph_types import Node


class Entity:
    """
    Generic Entity class that helps with type hinting.

    Several classes in PyRoboSim (such as robots, rooms, etc.) subclass from this.
    """

    def __init__(self) -> None:
        self.name = ""
        self.category: str | None = None
        self.pose = Pose()

        self.parent: Entity | None = None
        self.children: list[Entity] = []
        self.graph_nodes: list[Node] = []

        self.viz_patch: PathPatch | None = None


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
        self.sources: list[str] = [filename] if filename is not None else []

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
            return yaml.load(file, Loader=yaml.FullLoader)

    def has_category(self, category: str) -> bool:
        """
        Check whether a category name is in the metadata.

        :param category: Query category name.
        :return: True if the category is in the metadata, else False.
        """
        return category in self.data

    def get(self, category: str) -> dict[str, Any]:
        """
        Get metadata about a specific category.

        :param category: Query category name.
        :return: Category metadata dictionary if it exists, else None.
        """
        return self.data.get(category)

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

        self.sources.append(filename)


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
        super().__init(message)
