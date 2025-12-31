from setuptools import setup, find_packages


project_name = "pyrobosim_ros"

install_requires = [
    "pyrobosim",
]

setup(
    name=project_name,
    version="4.3.3",
    url="https://github.com/sea-bass/pyrobosim",
    author="Sebastian Castro",
    author_email="sebas.a.castro@gmail.com",
    description="ROS 2 interface to PyRoboSim.",
    license="MIT",
    install_requires=install_requires,
    packages=find_packages(),
    zip_safe=True,
)
