"""General package utilities."""

import re
import pathlib
from typing import Sequence

from matplotlib.colors import CSS4_COLORS, to_rgb


def get_data_folder() -> pathlib.Path:
    """
    Get a path to the folder containing data.

    If running using ``ros2 run``, this looks at the installed data in the
    ``pyrobosim`` colcon package's share directory.

    If running standalone, this looks at the data folder in the actual source.

    :return: Path to data folder.
    """
    try:
        # If running as a ROS 2 node, get the data folder from the package share directory.
        from ament_index_python.packages import get_package_share_directory

        data_folder = pathlib.Path(get_package_share_directory("pyrobosim")) / "data"
    except:
        # Else, assume it's relative to the file's current directory.
        data_folder = pathlib.Path(__file__).parent.parent / "data"

    return data_folder


def replace_special_yaml_tokens(
    in_text: str | pathlib.Path | list[str | pathlib.Path],
    root_dir: pathlib.Path | str | None = None,
) -> pathlib.Path | list[pathlib.Path]:
    """
    Replaces special tokens permitted in our YAML specification.
    If you want to add any other special tokens, you should do so in the process_text helper function.

    :param in_text: Input YAML path or a list of YAML paths.
    :param root_dir: Root directory for basing some tokens, uses the current directory if not specified.
    :return: YAML text(s) with all special tokens substituted.
    """
    root_dir = pathlib.Path(root_dir) if root_dir is not None else pathlib.Path.cwd()

    def process_text(text: str | pathlib.Path) -> pathlib.Path:
        """Helper function to replace tokens in a single metadata string."""
        if isinstance(text, pathlib.Path):
            text = text.as_posix()
        text = text.replace("$HOME", pathlib.Path.home().as_posix())
        text = text.replace("$DATA", get_data_folder().as_posix())
        text = text.replace("$PWD", root_dir.as_posix())
        return pathlib.Path(text)

    if isinstance(in_text, list):
        return [process_text(text) for text in in_text]
    elif isinstance(in_text, (str, pathlib.Path)):
        return process_text(in_text)
    else:
        raise TypeError(f"Could not replace text for input type: {type(in_text)}.")


def parse_color(color: Sequence[float] | str) -> Sequence[float]:
    """
    Parses a color input and returns an RGB tuple.

    :param color: Input color as a list, tuple, string, or hexadecimal.
    :return: RGB tuple in range (0.0, 1.0).
    """
    if isinstance(color, (list, tuple)):
        if len(color) == 3:
            return tuple(color)
        raise ValueError(
            "Incorrect number of elements. RGB color must have exactly 3 elements."
        )

    if isinstance(color, str):
        if color in CSS4_COLORS:
            color = to_rgb(CSS4_COLORS[color])
            assert isinstance(color, tuple) and len(color) == 3
            return color

        hex_pattern = r"^#(?:[0-9a-fA-F]{3}){1,2}$"
        if re.match(hex_pattern, color):
            color = to_rgb(color)
            assert isinstance(color, tuple) and len(color) == 3
            return color

        raise ValueError(f"Invalid color name or hexadecimal value: {color}.")

    raise ValueError(
        "Unsupported input type. Expected a list, tuple, or string representing a color."
    )
