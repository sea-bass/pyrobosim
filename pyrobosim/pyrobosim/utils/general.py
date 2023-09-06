""" General package utilities. """

import os
import yaml


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

    def __init__(self, filename):
        """
        Creates metadata from a YAML file.

        :param filename: Path to metadata YAML file.
        :type filename: str
        """
        if filename:
            if not os.path.isfile(filename):
                raise FileNotFoundError(f"Metadata filename not found: {filename}")

            self.filename = filename
            with open(self.filename) as file:
                self.data = yaml.load(file, Loader=yaml.FullLoader)
        else:
            self.filename = None
            self.data = {}

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


class InvalidEntityCategoryException(Exception):
    """Raised when an invalid entity metadata category is used."""

    pass


def replace_special_yaml_tokens(in_text, root_dir=None):
    """
    Replaces special tokens permitted in our YAML specification.
    If you want to add any other special tokens, you should do that here.

    :param in_text: Input YAML text.
    :type in_text: str
    :param root_dir: Root directory for basing some tokens, uses the current directory if not specified.
    :type root_dir: str, optional
    :return: YAML text with all special tokens substituted.
    :rtype: str
    """
    out_text = in_text
    out_text = out_text.replace("$HOME", os.environ["HOME"])
    out_text = out_text.replace("$DATA", get_data_folder())

    if root_dir is None:
        root_dir = os.getcwd()
    out_text = out_text.replace("$PWD", root_dir)

    return out_text
