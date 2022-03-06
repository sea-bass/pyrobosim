from astar import AStar
import numpy as np

from .utils import Pose


class SearchGraph:
    """ Graph for searching using A* """

    def __init__(self, world=None, max_edge_dist=np.inf, collision_check_dist=0.1):
        """
        Args:
            max_edge_dist (float): Maximum distance to automatically connect two edges
            collision_check_dist (float): Distance sampled along an edge to check for collisions
        """
        self.nodes = set()
        self.edges = set()
        self.world = world

        self.max_edge_dist = max_edge_dist
        self.collision_check_dist = collision_check_dist
        self.distance_dict = {}

        self.solver = GraphSolver()

    def search(self, start, goal):
        """ 
        Returns a path between start and goal nodes
        """
        if start not in self.nodes:
            raise Exception("Start node not in nodes")
            return None
        if goal not in self.nodes:
            raise Exception("Goal node not in nodes")

        path = self.solver.astar(start, goal)
        if path is None:
            print("No valid path found")
            return path
        else:
            path = list(path)
            return path

    def add(self, n, autoconnect=False):
        """ Adds a new node or list of nodes to the graph """
        if isinstance(n, list):
            for i in n:
                self.add(i, autoconnect=autoconnect)
        else:
            self.nodes.add(n)
            if autoconnect:
                for nconn in self.nodes:
                    self.connect(n, nconn)

    def connect(self, n0, n1):
        """ Connects two nodes in a graph, if collision free """
        if (n0 != n1) and self.check_connectivity(n0, n1):
            n0.neighbors.add(n1)
            n1.neighbors.add(n0)
            self.edges.add(Edge(n0, n1))

    def remove(self, ndel):
        """ Removes a node pr list of nodes and all its connections from the graph """
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

    def check_connectivity(self, start, end):
        """
        Checks connectivity between two nodes `start` and `end` in the world
        by sampling points spaced by the `self.collision_check_dist` parameter and verifying
        that every point is in the free configuration space.
        """

        # Trivial case where nodes are identical or there is no world
        if (self.world is None) or (start == end):
            return True

        # Build up the array of test X and Y coordinates for sampling between
        # the start and end points
        dist = start.pose.get_linear_distance(end.pose, ignore_z=True)
        angle = start.pose.get_angular_distance(end.pose)
        dist_array = np.arange(0, dist, self.collision_check_dist)
        if dist_array[-1] != dist:
            np.append(dist_array, dist)
        x_pts = start.pose.x + dist_array * np.cos(angle)
        y_pts = start.pose.y + dist_array * np.sin(angle)

        # Check the occupancy of all the test points.
        # Since we know the nodes already were sampled in free space, use a reduced inflation radius
        for x_check, y_check in zip(x_pts, y_pts):
            if self.world.check_occupancy(Pose(x=x_check, y=y_check)):
                return False

        # If the loop was traversed for all points without returning, we can connect
        return True


class Node:
    """ Graph node representation """

    def __init__(self, pose):
        self.pose = pose
        self.neighbors = set()


class Edge:
    """ Edge node representation """

    def __init__(self, n0, n1):
        self.n0 = n0
        self.n1 = n1
        self.cost = n0.pose.get_linear_distance(n1.pose)


class GraphSolver(AStar):
    """ 
    Implementation of the AStar class from python-astar to solve
    the A* shortest path algorithm for our graph representation.
    """

    def heuristic_cost_estimate(self, n0, n1):
        """ Compute distance """
        return n0.pose.get_linear_distance(n1.pose, ignore_z=True)

    def distance_between(self, n0, n1):
        """ Compute distance """
        return n0.pose.get_linear_distance(n1.pose, ignore_z=True)

    def neighbors(self, n):
        """ Get neighbors """
        return list(n.neighbors)
