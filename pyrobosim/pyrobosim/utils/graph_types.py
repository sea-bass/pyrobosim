"""Basic types for graphs."""

from typing import Any
from typing_extensions import Self  # For compatibility with Python <= 3.10

from .pose import Pose


class Node:
    """Graph node representation."""

    def __init__(self, pose: Pose, parent: Any = None, cost: float = 0.0) -> None:
        """
        Creates a graph node.

        :param pose: Pose of the node.
        :param parent: Parent node, if any.
        :param cost: Cost of the node, defaults to zero.
        """
        self.pose = pose
        self.parent = parent
        self.cost = cost
        self.neighbors: set[Self] = set()  # used in graph based planners


class Edge:
    """Graph edge representation."""

    def __init__(self, nodeA: Node, nodeB: Node) -> None:
        """
        Creates a graph edge.

        :param nodeA: First node
        :param nodeB: Second node
        """
        self.nodeA = nodeA
        self.nodeB = nodeB
        self.cost = nodeA.pose.get_linear_distance(nodeB.pose, ignore_z=True)
