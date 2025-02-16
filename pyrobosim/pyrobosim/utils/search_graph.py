"""Graph search utilities."""

from astar import AStar
import numpy as np

from .graph_types import Edge, Node
from .path import Path
from .pose import Pose
from ..utils.logging import get_global_logger


class SearchGraph:
    """Graph representation class."""

    def __init__(
        self,
        color: list[float] = [0, 0, 0],
        color_alpha: float = 0.5,
        use_planner: bool = False,
    ) -> None:
        """
        Creates an instance of SearchGraph.

        :param color: The display color for the graph.
        :param color_alpha: The intensity of the color.
        :param use_planner: If true, the graph will create a planner for path finding.
        """

        self.nodes: set[Node] = set()
        self.edges: set[Edge] = set()
        self.color = color
        self.color_alpha = color_alpha
        self.path_finder: SearchGraphPlanner | None = (
            SearchGraphPlanner() if use_planner else None
        )

    def add_node(self, node: Node) -> None:
        """
        Adds a node to the graph.

        :param node: The node to be added into the graph.
        """
        self.nodes.add(node)

    def remove_node(self, node: Node) -> None:
        """
        Removes a node from the graph.

        :param node: The node to be removed.
        """
        for other in self.nodes:
            other.neighbors.discard(node)
        self.nodes.discard(node)

        edges_to_remove = []
        for edge in self.edges:
            if edge.nodeA == node or edge.nodeB == node:
                edges_to_remove.append(edge)
        for edge in edges_to_remove:
            self.edges.discard(edge)

    def add_edge(self, nodeA: Node, nodeB: Node) -> Edge:
        """
        Adds an edge between 2 nodes.

        :param nodeA: The first node.
        :param nodeB: The second node.
        :return: The edge that was created.
        """
        edge = Edge(nodeA, nodeB)
        self.edges.add(edge)
        nodeA.neighbors.add(nodeB)
        nodeB.neighbors.add(nodeA)
        return edge

    def remove_edge(self, nodeA: Node, nodeB: Node) -> None:
        """
        Removes an edge between 2 nodes.

        :param nodeA: The first node.
        :param nodeB: The second node.
        """
        nodeA.neighbors.discard(nodeB)
        nodeB.neighbors.discard(nodeA)

        edges_to_remove = []
        for edge in self.edges:
            if edge.nodeA == nodeA and edge.nodeB == nodeB:
                edges_to_remove.append(edge)
        for edge in edges_to_remove:
            self.edges.discard(edge)

    def nearest(self, pose: Pose) -> Node | None:
        """
        Get the nearest node in the graph to a specified pose.

        :param pose: Query pose
        :return: The nearest node to the query pose, or None if the graph is empty.
        """
        if len(self.nodes) == 0:
            return None

        # Find the nearest node
        min_dist = np.inf
        for n in self.nodes:
            dist = pose.get_linear_distance(n.pose, ignore_z=True)
            if dist < min_dist:
                min_dist = dist
                n_nearest = n
        return n_nearest

    def find_path(self, nodeA: Node, nodeB: Node) -> Path:
        """
        Finds a path from nodeA to nodeB.

        :param nodeA: The start node.
        :param nodeB: The end node.
        :return: The path from nodeA to nodeB, if one exists.
        """
        path = Path()

        if self.path_finder is None:
            get_global_logger().warning(
                "Graph should be created with `use_planner = True` to use planner."
            )
        elif nodeA not in self.nodes:
            get_global_logger().warning("Node `nodeA` is not in the search graph.")
        elif nodeB not in self.nodes:
            get_global_logger().warning("Node `nodeB` is not in the search graph.")
        else:
            path_nodes = self.path_finder.plan(nodeA, nodeB)
            if path_nodes is None:
                get_global_logger().warning("Could not find a path from start to goal.")
                return path

            path_poses = [node.pose for node in path_nodes]
            if len(path_poses) > 1:
                path.set_poses(path_poses)

        return path


class SearchGraphPlanner(AStar):  # type: ignore[misc]
    """
    Graph based implementation of A*.
    """

    def __init__(self) -> None:
        super().__init__()

    def heuristic_cost_estimate(self, n0: Node, n1: Node) -> float:
        """
        Compute heuristic cost estimate using linear distance.

        :param n0: First node
        :param n1: Second node
        :return: Heuristic cost estimate
        """
        return n0.pose.get_linear_distance(n1.pose, ignore_z=True)

    def distance_between(self, n0: Node, n1: Node) -> float:
        """
        Compute distance between two nodes.

        :param n0: First node
        :param n1: Second node
        :return: Heuristic cost estimate
        """
        return n0.pose.get_linear_distance(n1.pose, ignore_z=True)

    def neighbors(self, n: Node) -> list[Node]:
        """
        Get neighbors of a graph node.

        :param n: Node
        :return: List of node neighbors
        """
        return list(n.neighbors)

    def plan(self, start: Node, goal: Node) -> list[Node] | None:
        """
        Plan path from start to goal.

        :param start: The start node.
        :param goal: The goal node.
        :return: A list of nodes describing the path, or None if planning failed.
        """
        try:
            astar_result = self.astar(start, goal)
            if astar_result is None:
                return None
            return [node for node in astar_result]
        except IndexError as e:
            get_global_logger().warning(f"Error calling astar: {e}")
            return None
