"""
Implementation of Probabilistic Roadmaps (PRM) for motion planning.
"""

import time
import warnings

from .search_graph import SearchGraph, Node, Edge
from .trajectory import fill_path_yaws
from ..utils.pose import Pose

class PRMPlanner:
    def __init__(self, world, max_nodes=100, max_connection_dist=2.0):
        # Parameters
        self.max_connection_dist = max_connection_dist
        self.max_nodes = max_nodes

        # Visualization
        self.color = [0, 0.4, 0.8]
        self.color_alpha = 0.25

        self.world = world
        self.reset()


    def reset(self):
        """ Resets the search trees and planning metrics. """
        self.planning_time = self.sampling_time = 0.0
        self.latest_path = None

        # Create a search graph and sample nodes.
        self.graph = SearchGraph(
            world=self.world, max_edge_dist=self.max_connection_dist)
        t_start = time.time()
        for i in range(self.max_nodes):
            n_sample = self.sample_configuration()
            if not n_sample:
                warnings.warn(f"Could not sample more than {i} nodes")
                break
            self.graph.add(Node(n_sample), autoconnect=True)
        self.sampling_time = time.time() - t_start
        

    def plan(self, start, goal):
        """ Plan a path from start to goal. """
        # Create the start and goal nodes
        if isinstance(start, Pose):
            start = Node(start, parent=None)
        self.graph.add(start, autoconnect=True)
        if isinstance(goal, Pose):  
            goal = Node(goal, parent=None)
        self.graph.add(goal, autoconnect=True)

        # Find a path from start to goal nodes
        t_start = time.time()
        path = self.graph.find_path(start, goal)
        path = fill_path_yaws(path)
        self.planning_time = time.time() - t_start
        self.latest_path = path
        return self.latest_path       


    def sample_configuration(self):
        """ Sample a random configuration from the world. """
        return self.world.sample_free_robot_pose_uniform()


    def print_metrics(self):
        """
        Print metrics about the latest path.
        """
        if self.latest_path is None:
            print("No path.")
            return

        print("Latest path from PRM:")
        for n in self.latest_path:
            print(n.pose)
        print("")
        print(f"Time to sample nodes: {self.sampling_time} seconds")
        print(f"Time to plan: {self.planning_time} seconds")

    def plot(self, axes, show_graph=True, show_path=True):
        """
        Plots the PRM graph on a specified set of axes
        """
        artists = []
        if show_graph:
            for e in self.graph.edges:
                x = (e.n0.pose.x, e.n1.pose.x)
                y = (e.n0.pose.y, e.n1.pose.y)
                edge, = axes.plot(x, y, color=self.color, alpha=self.color_alpha,
                                  linewidth=0.5, marker="o", markerfacecolor=self.color,
                                  markeredgecolor=self.color, markersize=3, zorder=1)
                artists.append(edge)
        
        if show_path and self.latest_path is not None:
            x = [p.pose.x for p in self.latest_path]
            y = [p.pose.y for p in self.latest_path]
            path, = axes.plot(x, y, "m-", linewidth=3, zorder=1)
            start, = axes.plot(x[0], y[0], "go", zorder=2)
            goal, = axes.plot(x[-1], y[-1], "rx", zorder=2)
            artists.extend((path, start, goal))

        return artists

    def show(self, show_graph=True, show_path=True):
        """
        Shows the PRM in a new figure.
        """
        import matplotlib.pyplot as plt
        f = plt.figure()
        ax = f.add_subplot(111)
        self.plot(ax, show_graph=show_graph, show_path=show_path)
        plt.title("PRM")
        plt.axis("equal")
        plt.show()
