""" Specification of the interface that all planners must implement. """

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

from pyrobosim.utils.motion import Path


class PathPlannerBase:
    """The base class for path planners."""

    def __init__(self):
        """
        Creates an instance of PathPlannerBase.
        """
        self.goal = None
        self.start = None
        self.impl = None
        self.planning_time = 0.0
        self.graphs = []
        self.graphs_updated = False
        self.latest_path = Path()

    def reset(self):
        """
        Resets the state of the planner.
        Sub-classes should add their own relevant mechanisms
        """
        self.goal = None
        self.start = None
        self.planner = None
        self.planning_time = 0.0
        self.graphs = []
        self.latest_path = Path()
        self.graphs_updated = False

    def plan(self, start, goal):
        """
        Plans a path from start to goal.

        :param start: Start pose or graph node.
        :type start: :class:`pyrobosim.utils.pose.Pose` /
            :class:`pyrobosim.utils.search_graph.Node`
        :param goal: Goal pose or graph node.
        :type goal: :class:`pyrobosim.utils.pose.Pose` /
            :class:`pyrobosim.utils.search_graph.Node`
        :return: Path from start to goal.
        :rtype: :class:`pyrobosim.utils.motion.Path`
        """

        raise NotImplementedError(
            f"Subclasses must implement {self.__class__.__name__}.plan()"
        )

    def info(self):
        """Prints information about the last generated plan."""
        print(f"Planner : {self.impl.__class__.__name__}")
        print(f"Planning time : {self.planning_time}")
        print(f"Number of waypoints : {self.latest_path.num_poses}")

    def plot(self, axes, show_graph=True, path=None, path_color="m"):
        """
        Plots the planned path on a specified set of axes.

        :param axes: The axes on which to draw.
        :type axes: :class:`matplotlib.axes.Axes`
        :param path: Path to display, defaults to None. If none, uses the `latest_path` attribute.
        :type path: :class:`pyrobosim.utils.motion.Path`, optional
        :param path_color: Color of the path, as an RGB tuple or string.
        :type path_color: tuple[float] / str, optional
        :return: List of Matplotlib artists containing what was drawn,
            used for bookkeeping.
        :rtype: list[:class:`matplotlib.artist.Artist`]
        """

        graph_artists = []
        path_artists = []
        artists = {}

        if not path:
            path = self.latest_path

        if show_graph:
            for graph in self.graphs:
                # Plot the markers
                (markers,) = axes.plot(
                    [n.pose.x for n in graph.nodes],
                    [n.pose.y for n in graph.nodes],
                    color=graph.color,
                    alpha=graph.color_alpha,
                    linestyle="",
                    marker="o",
                    markerfacecolor=graph.color,
                    markeredgecolor=graph.color,
                    markersize=3,
                    zorder=1,
                )
                graph_artists.append(markers)

                # Plot the edges as a LineCollection
                edge_coords = [
                    [[e.nodeA.pose.x, e.nodeA.pose.y], [e.nodeB.pose.x, e.nodeB.pose.y]]
                    for e in graph.edges
                ]
                line_segments = LineCollection(
                    edge_coords,
                    color=graph.color,
                    alpha=graph.color_alpha,
                    linewidth=0.5,
                    linestyle="--",
                    zorder=1,
                )
                axes.add_collection(line_segments)
                graph_artists.append(line_segments)

        if path and path.num_poses > 0:
            x = [p.x for p in path.poses]
            y = [p.y for p in path.poses]
            (path,) = axes.plot(
                x, y, linestyle="-", color=path_color, linewidth=3, alpha=0.5, zorder=1
            )
            (start,) = axes.plot(x[0], y[0], "go", zorder=2)
            (goal,) = axes.plot(x[-1], y[-1], "rx", zorder=2)
            path_artists.extend((path, start, goal))

        if graph_artists:
            artists["graph"] = graph_artists
        if path_artists:
            artists["path"] = path_artists
        return artists

    def show(self):
        """
        Shows the planned path in a new figure.

        :param show_path: If True, shows the last planned path.
        :type show_path: bool
        """

        f = plt.figure()
        ax = f.add_subplot(111)
        self.plot(ax)
        plt.title(self.impl.__class__.__name__)
        plt.axis("equal")
        plt.show()
