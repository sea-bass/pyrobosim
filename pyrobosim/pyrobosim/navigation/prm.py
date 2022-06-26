""" Probabilistic Roadmap (PRM) implementation. """

import time
import warnings

from .search_graph import SearchGraph, Node
from ..utils.pose import Pose


class PRMPlanner:
    """
    Implementation of Probabilistic Roadmaps (PRM) for motion planning.
    """

    def __init__(self, world, max_nodes=100, max_connection_dist=2.0):
        """
        Creates an instance of a PRM planner.

        :param world: World object to use in the planner.
        :type world: :class:`pyrobosim.core.world.World`
        :param max_nodes: Maximum nodes sampled to build the PRM.
        :type max_nodes: int
        :param max_connection_dist: Maximum connection distance between nodes.
        :type max_connection_dist: float
        """
        # Parameters
        self.max_connection_dist = max_connection_dist
        self.max_nodes = max_nodes

        # Visualization
        self.color = [0, 0.4, 0.8]
        self.color_alpha = 0.25

        self.world = world
        self.reset()

    def reset(self):
        """Resamples the PRM and resets planning metrics."""
        self.planning_time = self.sampling_time = 0.0
        self.latest_path = None

        # Create a search graph and sample nodes.
        self.graph = SearchGraph(
            world=self.world, max_edge_dist=self.max_connection_dist
        )
        t_start = time.time()
        for i in range(self.max_nodes):
            n_sample = self.sample_configuration()
            if not n_sample:
                warnings.warn(f"Could not sample more than {i} nodes")
                break
            self.graph.add(Node(n_sample), autoconnect=True)
        self.sampling_time = time.time() - t_start

    def plan(self, start, goal):
        """
        Plans a path from start to goal.

        :param start: Start pose or graph node.
        :type start: :class:`pyrobosim.utils.pose.Pose` /
            :class:`pyrobosim.navigation.search_graph.Node`
        :param goal: Goal pose or graph node.
        :type goal: :class:`pyrobosim.utils.pose.Pose` /
            :class:`pyrobosim.navigation.search_graph.Node`
        :return: Path from start to goal.
        :rtype: :class:`pyrobosim.utils.motion.Path`
        """
        # Create the start and goal nodes
        if isinstance(start, Pose):
            start = Node(start, parent=None)
        self.graph.add(start, autoconnect=True)
        if isinstance(goal, Pose):
            goal = Node(goal, parent=None)
        self.graph.add(goal, autoconnect=True)

        # Find a path from start to goal nodes
        t_start = time.time()
        self.latest_path = self.graph.find_path(start, goal)
        self.latest_path.fill_yaws()
        self.planning_time = time.time() - t_start
        return self.latest_path

    def sample_configuration(self):
        """
        Samples a random configuration from the world.

        :return: Collision-free pose if found, else ``None``.
        :rtype: :class:`pyrobosim.utils.pose.Pose`
        """
        return self.world.sample_free_robot_pose_uniform()

    def print_metrics(self):
        """
        Print metrics about the latest path computed.
        """
        if self.latest_path is None:
            print("No path.")
            return

        print("Latest path from PRM:")
        self.latest_path.print_details()
        print("")
        print(f"Time to sample nodes: {self.sampling_time} seconds")
        print(f"Time to plan: {self.planning_time} seconds")

    def plot(self, axes, show_graph=True, show_path=True):
        """
        Plots the PRM and the planned path on a specified set of axes.

        :param axes: The axes on which to draw.
        :type axes: :class:`matplotlib.axes.Axes`
        :param show_graph: If True, shows the RRTs used for planning.
        :type show_graph: bool
        :param show_path: If True, shows the last planned path.
        :type show_path: bool
        :return: List of Matplotlib artists containing what was drawn,
            used for bookkeeping.
        :rtype: list[:class:`matplotlib.artist.Artist`]
        """
        artists = []
        if show_graph:
            for e in self.graph.edges:
                x = (e.n0.pose.x, e.n1.pose.x)
                y = (e.n0.pose.y, e.n1.pose.y)
                (edge,) = axes.plot(
                    x,
                    y,
                    color=self.color,
                    alpha=self.color_alpha,
                    linewidth=0.5,
                    marker="o",
                    markerfacecolor=self.color,
                    markeredgecolor=self.color,
                    markersize=3,
                    zorder=1,
                )
                artists.append(edge)

        if show_path and self.latest_path.num_poses > 0:
            x = [p.x for p in self.latest_path.poses]
            y = [p.y for p in self.latest_path.poses]
            (path,) = axes.plot(x, y, "m-", linewidth=3, zorder=1)
            (start,) = axes.plot(x[0], y[0], "go", zorder=2)
            (goal,) = axes.plot(x[-1], y[-1], "rx", zorder=2)
            artists.extend((path, start, goal))

        return artists

    def show(self, show_graph=True, show_path=True):
        """
        Shows the PRM and the planned path in a new figure.

        :param show_graph: If True, shows the RRTs used for planning.
        :type show_graph: bool
        :param show_path: If True, shows the last planned path.
        :type show_path: bool
        """
        import matplotlib.pyplot as plt

        f = plt.figure()
        ax = f.add_subplot(111)
        self.plot(ax, show_graph=show_graph, show_path=show_path)
        plt.title("PRM")
        plt.axis("equal")
        plt.show()
