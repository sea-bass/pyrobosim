import os
from os.path import join
from setuptools import setup

# This is for standalone (non-ROS) use.

def get_files_in_folder(directory):
    """ Helper function to get all files in a specific directory. """
    file_list = []
    for (path, _, fnames) in os.walk(directory):
        for filename in fnames:
            file_list.append(join("..", path, filename))
    return file_list


install_requires = [
            'numpy',
            'shapely',
            'PyYAML',
            'pycollada',
            'astar',
            'descartes',
            'trimesh',
            'scipy',
            'PyQt5',
            'adjustText']


project_name = "pyrobosim"
setup(
    name=project_name,
    version="0.0.0",
    url="https://github.com/sea-bass/pyrobosim",
    author="Sebastian Castro",
    author_email="sebas.a.castro@gmail.com",
    description="ROS2 enabled 2D mobile robot simulator for behavior prototyping.",
    license="BSD",
    install_requires=install_requires,
    packages=[
        project_name,
        join(project_name, "core"),
        join(project_name, "gui"),
        join(project_name, "navigation"),
        join(project_name, "planning"),
        join(project_name, "utils")
    ],
    package_data={
        project_name: get_files_in_folder(join(project_name, "data"))
    },
    zip_safe=True
)
