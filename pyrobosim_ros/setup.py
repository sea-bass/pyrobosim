from setuptools import setup, find_packages


install_requires = [
    "pyrobosim",
]

project_name = "pyrobosim_ros"
setup(
    name=project_name,
    version="1.0.0",
    url="https://github.com/sea-bass/pyrobosim",
    author="Sebastian Castro",
    author_email="sebas.a.castro@gmail.com",
    description="ROS 2 interface to pyrobosim.",
    license="MIT",
    install_requires=install_requires,
    packages=find_packages(),
    zip_safe=True,
)
