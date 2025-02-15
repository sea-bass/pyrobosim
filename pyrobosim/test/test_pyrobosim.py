"""
Basic sanity checks for pyrobosim module.
"""

import importlib
from importlib.metadata import version


def test_import() -> None:
    assert importlib.util.find_spec("pyrobosim")


def test_version() -> None:
    ver = version("pyrobosim")
    assert ver == "3.3.0", "Incorrect pyrobosim version"
