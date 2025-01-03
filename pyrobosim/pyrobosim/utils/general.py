""" General package utilities. """

import os
import yaml
import re
from matplotlib.colors import CSS4_COLORS, to_rgb


def get_data_folder():
    """
    Get a path to the folder containing data.

    If running using ``ros2 run``, this looks at the installed data in the
    ``pyrobosim`` colcon package's share directory.

    If running standalone, this looks at the data folder in the actual source.

    :return: Path to data folder.
    :rtype: str
    """
    try:
        # If running as a ROS 2 node, get the data folder from the package share directory.
        from ament_index_python.packages import get_package_share_directory

        data_folder = os.path.join(get_package_share_directory("pyrobosim"), "data")
    except:
        # Else, assume it's relative to the file's current directory.
        data_folder = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "data"
        )

    return data_folder


class EntityMetadata:
    """Represents metadata about entities, such as locations or objects."""

    def __init__(self, filename=None):
        """
        Creates metadata from a YAML file.

        :param filename: Path to metadata YAML file.
        :type filename: str, optional
        """
        self.data = {}  # Holds metadata in the form of a dictionary.
        self.sources = []  # List of file paths from which metadata has been loaded.

        if filename:
            self.data = self._load_metadata(filename)
            self.sources = [filename]

    def _load_metadata(self, filename):
        """
        Loads metadata from a YAML file

        :param filename: Path to metadata YAML file.
        :type filename: str
        """
        if not os.path.isfile(filename):
            raise FileNotFoundError(f"Metadata filename not found: {filename}")

        with open(filename) as file:
            return yaml.load(file, Loader=yaml.FullLoader)

    def has_category(self, category):
        """
        Check whether a category name is in the metadata.

        :param category: Query category name.
        :type category: str
        :return: True if the category is in the metadata, else False.
        :rtype: bool
        """
        return category in self.data

    def get(self, category):
        """
        Get metadata about a specific category.

        :param category: Query category name.
        :type category: str
        :return: Category metadata dictionary if it exists, else None.
        :rtype: dict
        """
        return self.data.get(category, None)

    def add(self, filename):
        """
        Add metadata from a new YAML file to existing data.

        :param filename: Path to metadata YAML file.
        :type filename: str
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

    def __init__(self, key, old_value, new_value, source=None):
        """
        Creates an MetadataConflictException instance.

        :param key: Conflicting metadata key.
        :type key: str
        :param old_value: Existing value for key.
        :type old_value: Any
        :param new_value: New value for key.
        :type new_value: Any
        :param source: Source of new metadata.
        :type source: str, Optional
        """
        message = f"Conflict for key '{key}': existing value '{old_value}' conflicts with new value '{new_value}'"
        if source:
            message += f" from source '{source}'"
        super().__init(message)


def replace_special_yaml_tokens(in_text, root_dir=None):
    """
    Replaces special tokens permitted in our YAML specification.
    If you want to add any other special tokens, you should do so in the process_text helper function.

    :param in_text: Input YAML text or a list of YAML texts.
    :type in_text: str or list[str]
    :param root_dir: Root directory for basing some tokens, uses the current directory if not specified.
    :type root_dir: str, optional
    :return: YAML text(s) with all special tokens substituted.
    :rtype: str or list[str]
    """

    if root_dir is None:
        root_dir = os.getcwd()

    def process_text(text):
        """Helper function to replace tokens in a single metadata string."""
        text = text.replace("$HOME", os.environ["HOME"])
        text = text.replace("$DATA", get_data_folder())
        text = text.replace("$PWD", root_dir)
        return text

    if isinstance(in_text, list):
        out_text = [process_text(text) for text in in_text]
        return out_text
    if isinstance(in_text, str):
        out_text = process_text(in_text)
        return out_text


def parse_color(color):
    """
    Parses a color input and returns an RGB tuple.

    :param color: Input color as a list, tuple, string, or hexadecimal.
    :type color: list[float] | tuple[float, float, float] | str
    :return: RGB tuple in range (0.0, 1.0).
    :rtype: tuple[float, float, float]
    """
    if isinstance(color, (list, tuple)):
        if len(color) == 3:
            return tuple(color)
        raise ValueError(
            "Incorrect number of elements. RGB color must have exactly 3 elements."
        )

    if isinstance(color, str):
        if color in CSS4_COLORS:
            return to_rgb(CSS4_COLORS[color])

        hex_pattern = r"^#(?:[0-9a-fA-F]{3}){1,2}$"
        if re.match(hex_pattern, color):
            return to_rgb(color)

        raise ValueError(f"Invalid color name or hexadecimal value: {color}.")

    raise ValueError(
        "Unsupported input type. Expected a list, tuple, or string representing a color."
    )
