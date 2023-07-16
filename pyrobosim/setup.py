import os
from os.path import join
from setuptools import setup, find_packages

# This is for standalone (non-ROS) use.


def get_files_in_folder(directory):
    """Helper function to get all files in a specific directory."""
    file_list = []
    for path, _, fnames in os.walk(directory):
        for filename in fnames:
            file_list.append(join("..", path, filename))
    return file_list


install_requires = [
    "adjustText",
    "astar",
    "matplotlib",
    "numpy",
    "pycollada",
    "PyQt5",
    "PyYAML",
    "shapely>=2.0.1",
    "scipy",
    "transforms3d",
    "trimesh",
]

project_name = "pyrobosim"
setup(
    name=project_name,
    version="1.0.0",
    url="https://github.com/sea-bass/pyrobosim",
    author="Sebastian Castro",
    author_email="sebas.a.castro@gmail.com",
    description="ROS 2 enabled 2D mobile robot simulator for behavior prototyping.",
    license="MIT",
    install_requires=install_requires,
    packages=find_packages(),
    package_data={project_name: get_files_in_folder(join(project_name, "data"))},
    zip_safe=True,
)
