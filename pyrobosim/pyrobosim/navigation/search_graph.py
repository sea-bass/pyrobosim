""" Graph search utilities. """

from astar import AStar
import numpy as np
import warnings

from ..utils.pose import Pose


class SearchGraph:
    """ Graph for searching using A* """

    def __init__(self, world=None, max_edge_dist=np.inf, collision_check_dist=0.1):
        """
        Creates a new search graph.
        
        :param world: World object from which to create the search graph.
        :type world: :class:`pyrobosim.core.world.World`
        :param max_edge_dist: Maximum distance to automatically connect two edges, defaults to infinity.
        :type max_edge_dist: float, optional
        :param collision_check_dist: Distance sampled along an edge to check for collisions, defaults to 0.1.
        :type collision_check_dist: float, optional
        """
        self.nodes = set()
        self.edges = set()
        self.world = world

        self.max_edge_dist = max_edge_dist
        self.collision_check_dist = collision_check_dist
        self.distance_dict = {}

        self.solver = GraphSolver()

    def add(self, n, autoconnect=False):
        """ 
        Adds a new node or list of nodes to the graph.
        
        :param n: Node to add to the graph
        :type n: :class:`Node`
        :param autoconnect: Whether to autoconnect the node to existing nodes in the graph, defaults to False.
        :type autoconnect: bool, optional
        """
        if isinstance(n, list):
            for i in n:
                self.add(i, autoconnect=autoconnect)
        else:
            self.nodes.add(n)
            if autoconnect:
                for nconn in self.nodes:
                    self.connect(n, nconn)

    def connect(self, n0, n1):
        """
        Connects two nodes in a graph, if collision free.
        
        :param n0: First node to connect
        :type n0: :class:`Node`
        :param n1: Second node to connect
        :type n1: :class:`Node`
        :return: True if nodes were, else False.
        :rtype: bool
        """
        if (n0 != n1) and self.check_connectivity(n0, n1):
            n0.neighbors.add(n1)
            n1.neighbors.add(n0)
            self.edges.add(Edge(n0, n1))
            return True
        return False

    def remove(self, ndel):
        """ 
        Removes a node or list of nodes and all its connections from the graph.
        
        :param ndel: Node or list of nodes to remove
        :type ndel: :class:`Node`/list[:class:`Node`]
        """
        if isinstance(ndel, list):
            for i in ndel:
                self.remove(i)
        else:
            # Remove node from the node set in the graph
            if ndel in self.nodes:
                self.nodes.remove(ndel)
            # Remove node from every other node's neighbor set
            for n in self.nodes.copy():
                if ndel in n.neighbors:
                    n.neighbors.remove(ndel)
            # Remove any edges involving this node from the edge set
            for e in self.edges.copy():
                if e.n0 == ndel or e.n1 == ndel:
                    self.edges.remove(e)

    def check_connectivity(self, start, goal):
        """
        Checks connectivity between two nodes `start` and `goal` in the world
        by sampling points spaced by the `self.collision_check_dist` parameter and verifying
        that every point is in the free configuration space.

        :param start: Start node
        :type start: :class:`Node`
        :param goal: Goal node
        :type goal: :class:`Node`
        :return: True if nodes can be connected, else False.
        :rtype: bool
        """
        # Trivial case where nodes are identical or there is no world.
        if (self.world is None) or (start == goal):
            return True

        # Build up the array of test X and Y coordinates for sampling between
        # the start and goal points.
        dist = start.pose.get_linear_distance(goal.pose, ignore_z=True)
        angle = start.pose.get_angular_distance(goal.pose)
        dist_array = np.arange(0, dist, self.collision_check_dist)
        # If the nodes are coincident, connect them by default.
        if dist_array.size == 0:
            return True
        if dist_array[-1] != dist:
            np.append(dist_array, dist)
        x_pts = start.pose.x + dist_array * np.cos(angle)
        y_pts = start.pose.y + dist_array * np.sin(angle)

        # Check the occupancy of all the test points.
        # Since we know the nodes already were sampled in free space, use a reduced inflation radius.
        for x_check, y_check in zip(x_pts[1:-1], y_pts[1:-1]):
            if self.world.check_occupancy(Pose(x=x_check, y=y_check)):
                return False

        # If the loop was traversed for all points without returning, we can connect.
        return True

    def find_path(self, start, goal):
        """
        Gets a path from start to goal poses by searching the graph
        The path consists of a tuple of Pose objects
        
        :param start: Start node
        :type start: :class:`Node`
        :param goal: Goal node
        :type goal: :class:`Node`
        :return: List of graph Node objects describing the path
        :rtype: list[:class:`Node`]
        """
        path = self.solver.astar(start, goal)
        if path is None:
            warnings.warn("Did not find a path from start to goal.")
            return []
        else:
            return list(path)


class Node:
    """ Graph node representation. """

    def __init__(self, pose, parent=None, cost=0.0):
        """
        Creates a graph node. 
        
        :param pose: Pose of the node.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :param parent: Parent node, if any.
        :type parent: :class:`Node`, optional
        :param cost: Cose of the node, defaults to zero
        :type cost: float, optional
        """
        self.pose = pose
        self.parent = parent
        self.cost = cost
        self.neighbors = set()


class Edge:
    """ Graph edge representation. """

    def __init__(self, n0, n1):
        """
        Creates a graph edge.
        
        :param n0: First node
        :type n0: :class:`Node`
        :param n1: Second node
        :type n1: :class:`Node`
        """
        self.n0 = n0
        self.n1 = n1
        self.cost = n0.pose.get_linear_distance(n1.pose)


class GraphSolver(AStar):
    """ 
    Implementation of the AStar class from python-astar to solve
    the A* shortest path algorithm for our graph representation.
    """

    def heuristic_cost_estimate(self, n0, n1):
        """ 
        Compute heuristic cost estimate using linear distance.

        :param n0: First node
        :type n0: :class:`Node`
        :param n1: Second node
        :type n1: :class:`Node`
        :return: Heuristic cost estimate
        :rtype: float
        """
        return n0.pose.get_linear_distance(n1.pose, ignore_z=True)

    def distance_between(self, n0, n1):
        """
        Compute distance between two nodes
        
        :param n0: First node
        :type n0: :class:`Node`
        :param n1: Second node
        :type n1: :class:`Node`
        :return: Heuristic cost estimate
        :rtype: float
        """
        return n0.pose.get_linear_distance(n1.pose, ignore_z=True)

    def neighbors(self, n):
        """
        Get neighbors of a graph node.
        
        :param n: Node
        :type n: :class:`Node`
        :return: List of node neighbors
        :rtype: list[:class:`Node`]
        """
        return list(n.neighbors)
