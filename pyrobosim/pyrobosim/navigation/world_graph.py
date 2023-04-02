""" Graph search utilities. """

import numpy as np
from collections import defaultdict


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


class WorldGraph:

    """Graph representation of the world."""

    def __init__(self):
        """Creates an instance of WorldGraph."""

        self.nodes = set()
        self.edges = defaultdict(lambda: set())

    def add_node(self, node):
        """
        Adds a node to the graph.

        :param node: The node to be added into the graph.
        :type node: :class: `pyrobosim.navigation.search_graph.Node`
        """

        self.nodes.add(node)

    def remove_node(self, node):
        """
        Removes a node from the graph.

        :param node: The node to be removed.
        :type node: :class: `pyrobosim.navigation.search_graph.Node`
        """

        self.nodes.discard(node)

    def add_edge(self, nodeA, nodeB):
        """
        Adds an edge between 2 node.

        :param nodeA: The first node.
        :type nodeA: :class:`pyrobosim.navigation.search_graph.Node`
        :param nodeB: The second node.
        :type nodeB: :class:`pyrobosim.navigation.search_graph.Node`
        """

        self.edges[nodeA].add(nodeB)
        self.edges[nodeB].add(nodeA)
        self.update_neighbors()

    def remove_edge(self, nodeA, nodeB):
        """
        Removes an edge between 2 node.

        :param nodeA: The first node.
        :type nodeA: :class:`pyrobosim.navigation.search_graph.Node`
        :param nodeB: The second node.
        :type nodeB: :class:`pyrobosim.navigation.search_graph.Node`
        """

        self.edges[nodeA].discard(nodeB)
        self.edges[nodeB].discard(nodeA)
        self.update_neighbors()

    def update_neighbors(self):
        """Updates the neighbors of the nodes. For use after an update to the graph."""

        for node in self.nodes:
            node.neighbors = self.edges[node]

    def nearest(self, pose):
        """
        Get the nearest node in the graph to a specified pose.
        :param pose: Query pose
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :return: The nearest node to the query pose, or None if the graph is empty
        :rtype: :class:`Node`
        """
        if len(self.nodes) == 0:
            return None

        # Find the nearest node
        min_dist = np.inf
        for n in self.nodes:
            dist = pose.get_linear_distance(n.pose)
            if dist < min_dist:
                min_dist = dist
                n_nearest = n
        return n_nearest
