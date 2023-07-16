"""
Basic sanity checks for pyrobosim module.
"""

import sys
from importlib.metadata import version


def test_import():
    import pyrobosim

    assert "pyrobosim" in sys.modules


def test_version():
    ver = version("pyrobosim")
    assert ver == "1.0.0", "Incorrect pyrobosim version"
