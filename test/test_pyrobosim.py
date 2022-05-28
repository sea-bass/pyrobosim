"""
Basic sanity checks for pyrobosim module.
"""

import pytest
from importlib.metadata import version

def test_import():
    import pyrobosim
    assert("__init__.py" in pyrobosim.__file__)

def test_version():
    ver = version("pyrobosim")
    assert(ver == "0.0.0"), "Incorrect pyrobosim version"
