#!/usr/bin/env python3

"""
Unit tests for search graph utilities.
"""

import numpy as np
import pytest

from pyrobosim.core import Pose
from pyrobosim.utils.search_graph import Node, SearchGraph, SearchGraphPlanner


####################
# Helper functions #
####################


def create_test_graph(use_planner=False):
    """Creates a test graph with multiple nodes and edges."""
    graph = SearchGraph(use_planner=use_planner)

    nodes = [
        Node(Pose(x=0.0, y=0.0)),  # Lower left
        Node(Pose(x=1.0, y=0.0)),  # Lower right
        Node(Pose(x=1.0, y=1.0)),  # Upper right
        Node(Pose(x=0.0, y=1.0)),  # Upper left
    ]

    for node in nodes:
        graph.add_node(node)

    for node in nodes:
        for other_node in [n for n in nodes if n != node]:
            graph.add_edge(node, other_node)

    return graph, nodes


##############
# Unit tests #
##############


def test_search_graph_default_args():
    graph = SearchGraph()

    assert len(graph.nodes) == 0
    assert len(graph.edges) == 0
    assert graph.color == [0, 0, 0]
    assert graph.color_alpha == 0.5
    assert not graph.use_planner


def test_search_graph_nondefault_args():
    graph = SearchGraph(color=[0.6, 0.7, 0.8], color_alpha=0.9, use_planner=True)

    assert len(graph.nodes) == 0
    assert len(graph.edges) == 0
    assert graph.color == [0.6, 0.7, 0.8]
    assert graph.color_alpha == 0.9
    assert graph.use_planner
    assert isinstance(graph.path_finder, SearchGraphPlanner)


def test_search_graph_add_remove_nodes():
    graph = SearchGraph()

    # Add a node
    node_0 = Node(Pose(x=0.0, y=0.0, yaw=0.0))
    graph.add_node(node_0)
    assert len(graph.nodes) == 1
    assert len(graph.edges) == 0

    # Add a second node and an edge
    node_1 = Node(Pose(x=3.0, y=4.0, yaw=np.pi / 2.0))
    graph.add_node(node_1)
    edge = graph.add_edge(node_0, node_1)

    assert len(graph.nodes) == 2
    assert len(graph.edges) == 1
    assert edge.nodeA == node_0
    assert edge.nodeB == node_1
    assert edge.cost == pytest.approx(5.0)
    assert node_1 in node_0.neighbors
    assert node_0 in node_1.neighbors

    # Remove the edge between nodes
    graph.remove_edge(node_0, node_1)
    assert len(graph.nodes) == 2
    assert len(graph.edges) == 0
    assert node_1 not in node_0.neighbors
    assert node_0 not in node_1.neighbors

    # Removing the edge again should do nothing, as the edge does not exist
    graph.remove_edge(node_0, node_1)
    assert len(graph.nodes) == 2
    assert len(graph.edges) == 0
    assert node_1 not in node_0.neighbors
    assert node_0 not in node_1.neighbors

    # Re-add the edge, and remove the node which should get rid of the edge as well.
    graph.add_edge(node_0, node_1)
    graph.remove_node(node_1)
    assert len(graph.nodes) == 1
    assert len(graph.edges) == 0


def test_search_graph_get_nearest_node():
    # If the graph is empty, there is no nearest node.
    graph = SearchGraph()
    query_pose = Pose(x=0.0, y=0.0)
    assert graph.nearest(query_pose) is None

    # Now create a filled out graph.
    graph, nodes = create_test_graph()

    # This pose is exactly at the lower left node and should return it.
    query_pose = Pose(x=0.0, y=0.0)
    assert graph.nearest(query_pose) == nodes[0]

    # This pose is nearest to the upper right node and should return it.
    query_pose = Pose(x=0.8, y=1.1)
    assert graph.nearest(query_pose) == nodes[2]


def test_search_graph_find_path_no_planner():
    """Checks that path finding fails if no planner is specified in the graph."""
    graph, nodes = create_test_graph()
    with pytest.warns(UserWarning):
        path = graph.find_path(nodes[0], nodes[2])
        assert path.num_poses == 0


def test_search_graph_find_path_no_nodes():
    """Checks that path finding fails with missing nodes."""
    graph, nodes = create_test_graph(use_planner=True)

    dummy_node = Node(Pose(x=2.0, y=2.0))
    with pytest.warns(UserWarning):
        path = graph.find_path(nodes[0], dummy_node)
        assert path.num_poses == 0
    with pytest.warns(UserWarning):
        path = graph.find_path(dummy_node, nodes[2])
        assert path.num_poses == 0


def test_search_graph_find_path():
    """Checks successful path planning."""
    graph, nodes = create_test_graph(use_planner=True)

    # Planning to the same node as the start node should yield an empty path, but no warnings.
    path = graph.find_path(nodes[0], nodes[0])
    assert path.num_poses == 0

    # Planning to the upper right corner should yield a diagonal path
    path = graph.find_path(nodes[0], nodes[2])
    assert path.num_poses == 2
    assert path.poses[0] == nodes[0].pose
    assert path.poses[1] == nodes[2].pose

    # Adding a new node connected to the upper right should yield a longer path
    new_node = Node(Pose(x=2.0, y=2.0))
    graph.add_node(new_node)
    graph.add_edge(nodes[2], new_node)

    path = graph.find_path(nodes[0], new_node)
    assert path.num_poses == 3
    assert path.poses[0] == nodes[0].pose
    assert path.poses[1] == nodes[2].pose
    assert path.poses[2] == new_node.pose

    # If we remove the edge to the goal node, we should find no path.
    graph.remove_edge(nodes[2], new_node)
    with pytest.warns(UserWarning):
        path = graph.find_path(nodes[0], new_node)
        assert path.num_poses == 0
