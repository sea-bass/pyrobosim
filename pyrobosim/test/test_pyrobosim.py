"""
Basic sanity checks for pyrobosim module.
"""

import importlib
from importlib.metadata import version


def test_import():
    assert importlib.util.find_spec("pyrobosim")


def test_version():
    ver = version("pyrobosim")
    assert ver == "3.0.0", "Incorrect pyrobosim version"
