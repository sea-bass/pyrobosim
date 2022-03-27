from setuptools import setup, find_packages

# This is for standalone (non-ROS) use.
project_name = "pyrobosim"
setup(
    name=project_name,
    version="0.0.0",
    url="https://github.com/sea-bass/pyrobosim",
    author="Sebastian Castro",
    author_email="sebas.a.castro@gmail.com",
    description="ROS2 enabled 2D mobile robot simulator for behavior prototyping.",
    license="MIT",
    packages=[
        project_name,
        project_name + "/gui",
        project_name + "/navigation",
        project_name + "/planning",
        project_name + "/utils",
        project_name + "/world",
    ],
    zip_safe=True
)
