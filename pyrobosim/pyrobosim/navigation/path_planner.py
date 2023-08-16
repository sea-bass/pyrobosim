""" Implementation of the generic path planner. """
import warnings
from pyrobosim.navigation.a_star import AstarPlanner
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.navigation.prm import PRMPlanner
from pyrobosim.navigation.world_graph import WorldGraphPlanner


class PathPlanner:
    """
    Creates a path planner.
    """

    def __init__(self, planner_type, **planner_config):
        """
        Creates a PathPlanner instance of given type and configuration

        :param planner_type: The type of planner to be used (astar, prm, rrt, world_graph)
        :type planner_type: str
        :param planner_config: The configuration to be used with the specified planner type.
        :type planner_config: dict
        """

        self.planners = {
            "astar": AstarPlanner,
            "rrt": RRTPlanner,
            "prm": PRMPlanner,
            "world_graph": WorldGraphPlanner,
        }

        if planner_type not in self.planners:
            warnings.warn(
                f"{planner_type} is not a supported planner type.", UserWarning
            )
            return None
        if not planner_config:
            warnings.warn(
                f"No planner configuration provided. Must provide either a World or OccupancyGrid object.",
                UserWarning,
            )
            return None

        self.planner_type = planner_type
        self.planner_config = planner_config
        self.planner = self.planners[self.planner_type](**self.planner_config)

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

        self.latest_path = self.planner.plan(start, goal)
        return self.latest_path

    def plot(self, axes, path=None, path_color="m"):
        """
        Plots the planned path on a specified set of axes.

        :param axes: The axes on which to draw.
        :type axes: :class:`matplotlib.axes.Axes`
        :param path: Path to display, defaults to None.
        :type path: :class:`pyrobosim.utils.motion.Path`, optional
        :param path_color: Color of the path, as an RGB tuple or string.
        :type path_color: tuple[float] / str, optional
        :return: List of Matplotlib artists containing what was drawn,
            used for bookkeeping.
        :rtype: list[:class:`matplotlib.artist.Artist`]
        """

        return self.planner.plot(axes, path=path, path_color=path_color)

    def show(self):
        """Displays the planned path on the GUI."""

        self.planner.show()

    def info(self):
        """Display information about planning process."""

        self.planner.info()
