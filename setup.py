from os import filesep
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
        project_name + filesep + "core",
        project_name + filesep + "gui",
        project_name + filesep + "navigation",
        project_name + filesep + "planning",
        project_name + filesep + "utils"
    ],
    zip_safe=True
)
