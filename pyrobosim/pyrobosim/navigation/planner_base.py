""" Specification of the interface that all planners must implement. """

from pyrobosim.utils.motion import Path


class PathPlannerBase:
    """The base class for path planners."""

    def __init__(self):
        self.goal = None
        self.start = None
        self.planning_time = 0
        self.latest_path = Path()

    def reset(self):
        """
        Resets the state of the planner.
        Sub-classes should add their own relevant mechanims
        """
        self.goal = None
        self.start = None
        self.planning_time = 0
        self.latest_path = Path()

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
        raise NotImplementedError(
            f"Subclasses must implement {self.__class__.__name__}.plan()"
        )

    def plot(self, axes, path_color="m"):
        """
        Plots the planned path on a specified set of axes.

        :param axes: The axes on which to draw.
        :type axes: :class:`matplotlib.axes.Axes`
        :param path_color: Color of the path, as an RGB tuple or string.
        :type path_color: tuple[float] / str, optional
        :return: List of Matplotlib artists containing what was drawn,
            used for bookkeeping.
        :rtype: list[:class:`matplotlib.artist.Artist`]
        """
        artists = []
        if self.latest_path.num_poses > 0:
            x = [p.x for p in self.latest_path.poses]
            y = [p.y for p in self.latest_path.poses]
            (path,) = axes.plot(
                x, y, linestyle="-", color=path_color, linewidth=3, alpha=0.5, zorder=1
            )
            (start,) = axes.plot(x[0], y[0], "go", zorder=2)
            (goal,) = axes.plot(x[-1], y[-1], "rx", zorder=2)
            artists.extend((path, start, goal))

        return artists

    def show(self):
        """
        Shows the A* the planned path in a new figure.

        :param show_path: If True, shows the last planned path.
        :type show_path: bool
        """
        import matplotlib.pyplot as plt

        f = plt.figure()
        ax = f.add_subplot(111)
        self.plot(ax)
        plt.title("A*")
        plt.axis("equal")
        plt.show()
