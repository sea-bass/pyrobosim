import os
from pathlib import Path
from setuptools import setup, find_packages


def get_files_in_folder(directory):
    """Helper function to get all files in a specific directory."""
    file_list = []
    for path, _, fnames in os.walk(directory):
        for filename in fnames:
            file_list.append(os.path.join("..", path, filename))
    return file_list


project_name = "pyrobosim"

data_dir = os.path.join(project_name, "data")
install_requires = [
    "adjustText",
    "astar",
    "matplotlib",
    "numpy<2.1.0",
    "pycollada",
    "PySide6>=6.4.0",
    "PyYAML",
    "shapely>=2.0.1",
    "scipy",
    "transforms3d",
    "trimesh",
]

# This will gracefully fall back to an empty string if the README.md cannot be read.
# This can happen if building this package with `colcon build --symlink-install`.
readme_path = Path(__file__).parent / "README.md"
readme_text = readme_path.read_text() if readme_path.exists() else ""

setup(
    name=project_name,
    version="3.0.0",
    url="https://github.com/sea-bass/pyrobosim",
    author="Sebastian Castro",
    author_email="sebas.a.castro@gmail.com",
    description="ROS 2 enabled 2D mobile robot simulator for behavior prototyping.",
    long_description=readme_text,
    long_description_content_type="text/markdown",
    license="MIT",
    install_requires=install_requires,
    packages=find_packages(),
    package_data={project_name: get_files_in_folder(data_dir)},
    tests_require=["pytest"],
    zip_safe=True,
)
