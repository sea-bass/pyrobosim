""" GUI utilities.

This module contains the main pyrobosim UI, which is based on PySide6,
as well as the tools to embed the world model as a matplotlib canvas.

The GUI allows users to view the state of the robot(s) in the world
and interact with the world model using actions such as navigating,
picking, and placing objects.
"""

from .main import *
from .world_canvas import *
