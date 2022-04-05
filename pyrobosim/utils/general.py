import os

""" General package utilities """

def get_data_folder():
    """ Get a path to the folder containing data """
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
