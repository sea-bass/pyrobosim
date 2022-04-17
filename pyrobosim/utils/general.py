""" General package utilities. """

import os


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
        # If running as a ROS2 node, get the data folder from the package share directory.
        from ament_index_python.packages import get_package_share_directory
        data_folder = os.path.join(
            get_package_share_directory("pyrobosim"), "data")
    except:
        # Else, assume it's relative to the file's current directory.
        data_folder = os.path.join(os.path.dirname(
            os.path.abspath(__file__)), "..", "pyrobosim", "data")

    return data_folder


def replace_special_yaml_tokens(in_text):
    """ 
    Replaces special tokens permitted in our YAML specification.
    If you want to add any other special tokens, you should do that here.
    
    :param in_text: Input YAML text.
    :type in_text: str
    :return: YAML text with all special tokens substituted.
    :rtype: str
    """
    out_text = in_text
    out_text = out_text.replace("$HOME", os.environ["HOME"])
    out_text = out_text.replace("$DATA", get_data_folder())
    return out_text
