"""
Core types for PyRoboSim.
"""

from matplotlib.patches import PathPatch

from ..utils.pose import Pose
from ..utils.search_graph import Node


class Entity:
    """
    Generic Entity class that helps with type hinting.

    Several classes in PyRoboSim (such as robots, rooms, etc.) subclass from this.
    """

    def __init__(self) -> None:
        self.name = ""
        self.category: str | None = None
        self.pose = Pose()

        self.parent: Entity | None = None
        self.children: list[Entity] = []
        self.graph_nodes: list[Node] = []

        self.viz_patch: PathPatch | None = None
