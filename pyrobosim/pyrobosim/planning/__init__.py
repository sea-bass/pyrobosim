""" Task and motion planning utilities.

This module contains tools associated with task and motion planning
(TAMP). This includes representations for parametric actions that comprise
a plan, as well as infrastructure to come up with such a plan given the
current and desired state of the world.
"""

import importlib

# Only import PDDLStream planner if the module is available.
if importlib.util.find_spec("pddlstream") is not None:
    from .pddlstream.planner import *
