""" Graph search utilities. """

from astar import AStar
import numpy as np
import warnings
import time

from ..utils.motion import Path
from ..utils.pose import Pose


class SearchGraphPlanner:
    """Lightweight path planner that wraps around SearchGraph."""

    def __init__(self, graph):
        self.graph = graph
        self.latest_path = None

    def plan(self, start, goal):
        """Plan a path from a start to a goal node."""
        if self.graph is None:
            warnings.warn("No search graph defined for this world.")
            return None

        self.latest_path = self.graph.find_path(start, goal)
        self.latest_path.fill_yaws()
        return self.latest_path

    def plot(self, axes, path_color="m", show_graph=True, show_path=True):
        """
        Plots the search graph and/or path on a specified set of axes.

        :param axes: The axes on which to draw.
        :type axes: :class:`matplotlib.axes.Axes`
        :param path_color: Color of the path, as an RGB tuple or string.
        :type path_color: tuple[float] / str, optional
        :param show_graph: If True, shows the RRTs used for planning.
        :type show_graph: bool
        :param show_path: If True, shows the last planned path.
        :type show_path: bool
        :return: List of Matplotlib artists containing what was drawn,
            used for bookkeeping.
        :rtype: list[:class:`matplotlib.artist.Artist`]
        """
        if show_graph:
            artists = self.graph.plot(axes)
        else:
            artists = []

        if show_path and self.latest_path and self.latest_path.num_poses > 0:
            x = [p.x for p in self.latest_path.poses]
            y = [p.y for p in self.latest_path.poses]
            (path,) = axes.plot(x, y, linestyle="-", color=path_color,
                                linewidth=3, alpha=0.5, zorder=1)
            (start,) = axes.plot(x[0], y[0], "go", zorder=2)
            (goal,) = axes.plot(x[-1], y[-1], "rx", zorder=2)
            artists.extend((path, start, goal))

        return artists


class SearchGraph:
    """Graph for searching using A*"""

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
        self.lock = False  # To prevent adding/removing nodes while plotting.

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
            while self.lock:
                time.sleep(0.001)
            self.lock = True

            self.nodes.add(n)
            if autoconnect:
                for nconn in self.nodes:
                    self.connect(n, nconn)

            self.lock = False

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
            while self.lock:
                time.sleep(0.001)
            self.lock = True

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

            self.lock = False

    def check_connectivity(self, start, goal, ignore_max_dist=False):
        """
        Checks connectivity between two nodes `start` and `goal` in the world
        by sampling points spaced by the `self.collision_check_dist` parameter
        and verifying that every point is in the free configuration space.

        :param start: Start node
        :type start: :class:`Node`
        :param goal: Goal node
        :type goal: :class:`Node`
        :param ignore_max_dist: If True, ignores maximum connection distance.
        :type ignore_max_dist: bool, optional
        :return: True if nodes can be connected, else False.
        :rtype: bool
        """
        # Trivial case where nodes are identical or there is no world.
        if (self.world is None) or (start == goal):
            return True

        # Check against the max edge distance.
        dist = start.pose.get_linear_distance(goal.pose, ignore_z=True)
        angle = start.pose.get_angular_distance(goal.pose)
        if (not ignore_max_dist) and (dist > self.max_edge_dist):
            return False

        # Build up the array of test X and Y coordinates for sampling between
        # the start and goal points.
        dist_array = np.arange(0, dist, self.collision_check_dist)
        # If the nodes are coincident, connect them by default.
        if dist_array.size == 0:
            return True
        if dist_array[-1] != dist:
            np.append(dist_array, dist)
        x_pts = start.pose.x + dist_array * np.cos(angle)
        y_pts = start.pose.y + dist_array * np.sin(angle)

        # Check the occupancy of all the test points.
        for x_check, y_check in zip(x_pts[1:], y_pts[1:]):
            if self.world.check_occupancy(Pose(x=x_check, y=y_check)):
                return False

        # If the loop was traversed for all points without returning, we can
        # connect the points.
        return True

    def find_path(self, start, goal):
        """
        Gets a path from start to goal poses by searching the graph
        The path consists of a tuple of Pose objects

        :param start: Start node
        :type start: :class:`Node`
        :param goal: Goal node
        :type goal: :class:`Node`
        :return: Path from start to goal.
        :rtype: :class:`pyrobosim.utils.motion.Path`
        """
        path = self.solver.astar(start, goal)
        if path is None:
            warnings.warn("Did not find a path from start to goal.")
            return Path()
        else:
            return Path(poses=[p.pose for p in path])

    def nearest_node(self, pose):
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

    def plot(self, axes):
        """
        Plots the search graph on a specified set of axes.
        """
        while self.lock:
            time.sleep(0.001)
        self.lock = True

        artists = []
        x = [n.pose.x for n in self.nodes]
        y = [n.pose.y for n in self.nodes]
        (nodes,) = axes.plot(x, y, "k.", linestyle="None", markersize=10)
        artists.append(nodes)

        for e in self.edges:
            x = (e.n0.pose.x, e.n1.pose.x)
            y = (e.n0.pose.y, e.n1.pose.y)
            (edge,) = axes.plot(x, y, "k:", linewidth=1)
            artists.append(edge)

        self.lock = False
        return artists


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
        self.neighbors = set()


class Edge:
    """Graph edge representation."""

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
