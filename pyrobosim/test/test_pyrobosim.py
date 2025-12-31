"""
Basic sanity checks for PyRoboSim module.
"""

from importlib.metadata import version
from importlib.util import find_spec


def test_import() -> None:
    assert find_spec("pyrobosim")


def test_version() -> None:
    ver = version("pyrobosim")
    assert ver == "4.3.3", "Incorrect pyrobosim version"
