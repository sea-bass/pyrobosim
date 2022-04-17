from os.path import join
from setuptools import setup

# This is for standalone (non-ROS) use.
project_name = "pyrobosim"
setup(
    name=project_name,
    version="0.0.0",
    url="https://github.com/sea-bass/pyrobosim",
    author="Sebastian Castro",
    author_email="sebas.a.castro@gmail.com",
    description="ROS2 enabled 2D mobile robot simulator for behavior prototyping.",
    license="BSD",
    packages=[
        project_name,
        join(project_name, "core"),
        join(project_name, "gui"),
        join(project_name, "navigation"),
        join(project_name, "planning"),
        join(project_name, "utils")
    ],
    zip_safe=True
)
