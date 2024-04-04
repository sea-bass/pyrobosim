""" Graph search utilities. """

import warnings

from astar import AStar
import numpy as np

from .motion import Path


class Node:
    """Graph node representation."""

    def __init__(self, pose, parent=None, cost=0.0):
        """
        Creates a graph node.

        :param pose: Pose of the node.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :param parent: Parent node, if any.
        :type parent: :class:`Node`, optional
        :param cost: Cost of the node, defaults to zero.
        :type cost: float, optional
        """
        self.pose = pose
        self.parent = parent
        self.cost = cost
        self.neighbors = set()  # used in graph based planners


class Edge:
    """Graph edge representation."""

    def __init__(self, nodeA, nodeB):
        """
        Creates a graph edge.
        :param nodeA: First node
        :type nodeA: :class:`Node`
        :param nodeB: Second node
        :type nodeB: :class:`Node`
        """
        self.nodeA = nodeA
        self.nodeB = nodeB
        self.cost = nodeA.pose.get_linear_distance(nodeB.pose, ignore_z=True)


class SearchGraph:
    """Graph representation class."""

    def __init__(self, color=[0, 0, 0], color_alpha=0.5, use_planner=False):
        """
        Creates an instance of SearchGraph.
        :param color: The display color for the graph.
        :type param: Integer List [R, G, B]
        :param color_alpha: The intensity of the color.
        :type color_alpha: float
        :param use_planner: If true, the graph will create a planner for path finding.
        :type use_planner: bool
        """

        self.nodes = set()
        self.edges = set()
        self.color = color
        self.color_alpha = color_alpha
        self.use_planner = use_planner
        if self.use_planner:
            self.path_finder = SearchGraphPlanner()

    def add_node(self, node):
        """
        Adds a node to the graph.

        :param node: The node to be added into the graph.
        :type node: :class:`pyrobosim.utils.search_graph.Node`
        """

        self.nodes.add(node)

    def remove_node(self, node):
        """
        Removes a node from the graph.

        :param node: The node to be removed.
        :type node: :class:`pyrobosim.utils.search_graph.Node`
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

    def add_edge(self, nodeA, nodeB):
        """
        Adds an edge between 2 nodes.

        :param nodeA: The first node.
        :type nodeA: :class:`pyrobosim.utils.search_graph.Node`
        :param nodeB: The second node.
        :type nodeB: :class:`pyrobosim.utils.search_graph.Node`
        :return: The edge that was created.
        :rtype: :class:`pyrobosim.utils.search_graph.Edge`
        """
        edge = Edge(nodeA, nodeB)
        self.edges.add(edge)
        nodeA.neighbors.add(nodeB)
        nodeB.neighbors.add(nodeA)
        return edge

    def remove_edge(self, nodeA, nodeB):
        """
        Removes an edge between 2 nodes.
        :param nodeA: The first node.
        :type nodeA: :class:`pyrobosim.utils.search_graph.Node`
        :param nodeB: The second node.
        :type nodeB: :class:`pyrobosim.utils.search_graph.Node`
        """
        nodeA.neighbors.discard(nodeB)
        nodeB.neighbors.discard(nodeA)

        edges_to_remove = []
        for edge in self.edges:
            if edge.nodeA == nodeA and edge.nodeB == nodeB:
                edges_to_remove.append(edge)
        for edge in edges_to_remove:
            self.edges.discard(edge)

    def nearest(self, pose):
        """
        Get the nearest node in the graph to a specified pose.
        :param pose: Query pose
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :return: The nearest node to the query pose, or None if the graph is empty.
        :rtype: :class:`pyrobosim.utils.search_graph.Node`
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

    def find_path(self, nodeA, nodeB):
        """
        Finds a path from nodeA to nodeB.

        :param nodeA: The start node.
        :type nodeA: :class: `pyrobosim.utils.search_graph.Node`
        :param nodeB: The end node.
        :type nodeB: :class: `pyrobosim.utils.search_graph.Node`
        :return: The path from nodeA to nodeB, if one exists.
        :rtype: :class: `pyrobosim.utils.motion.Path`
        """
        path = Path()

        if not self.use_planner:
            warnings.warn(
                "Graph should be created with `use_planner = True` to use planner."
            )
        elif nodeA not in self.nodes:
            warnings.warn("Node `nodeA` is not in the search graph.")
        elif nodeB not in self.nodes:
            warnings.warn("Node `nodeB` is not in the search graph.")
        else:
            path_nodes = self.path_finder.plan(nodeA, nodeB)
            if path_nodes is None:
                warnings.warn("Could not find a path from start to goal.")
                return path

            path_poses = [node.pose for node in path_nodes]
            if len(path_poses) > 1:
                path.set_poses(path_poses)

        return path


class SearchGraphPlanner(AStar):
    """
    Graph based implementation of A*.
    """

    def __init__(self):
        super().__init__()

    def heuristic_cost_estimate(self, n0, n1):
        """
        Compute heuristic cost estimate using linear distance.
        :param n0: First node
        :type n0: :class:`pyrobosim.utils.search_graph.Node`
        :param n1: Second node
        :type n1: :class:`pyrobosim.utils.search_graph.Node`
        :return: Heuristic cost estimate
        :rtype: float
        """
        return n0.pose.get_linear_distance(n1.pose, ignore_z=True)

    def distance_between(self, n0, n1):
        """
        Compute distance between two nodes.
        :param n0: First node
        :type n0: :class:`pyrobosim.utils.search_graph.Node`
        :param n1: Second node
        :type n1: :class:`pyrobosim.utils.search_graph.Node`
        :return: Heuristic cost estimate
        :rtype: float
        """
        return n0.pose.get_linear_distance(n1.pose, ignore_z=True)

    def neighbors(self, n):
        """
        Get neighbors of a graph node.
        :param n: Node
        :type n: :class:`pyrobosim.utils.search_graph.Node`
        :return: List of node neighbors
        :rtype: list[:class:`pyrobosim.utils.search_graph.Node`]
        """
        return list(n.neighbors)

    def plan(self, start, goal):
        """
        Plan path from start to goal.

        :param start: Node
        :type start: :class:`pyrobosim.utils.search_graph.Node`
        :param goal: Node
        :type goal: :class:`pyrobosim.utils.search_graph.Node`
        """
        try:
            self.latest_path = self.astar(start, goal)
        except IndexError as e:
            warnings.warn(f"Error calling astar: {e}")
            self.latest_path = None
        return self.latest_path
