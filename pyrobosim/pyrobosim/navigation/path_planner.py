""" Implementation of the generic path planner. """
import warnings
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.navigation.prm import PRMPlanner
from pyrobosim.navigation.a_star import AstarPlanner


class PathPlanner:
    """
    Creates a path planner.
    """

    def __init__(self, planner_type, planner_config):
        """
        Creates a PathPlanner instance of given type and configuration

        :param planner_type: The type of planner to be used (AStar, RRT, PRM)
        :type planner_type: str
        :param planner_config: The configuration to be used with the specified planner type.
        :type planner_config: dict

        The planner_config can have a many options within it.

        1. grid - None by default, but can be assigned to an OccupancyGrid object.
                  If set to an OccupancyGrid, the planner will use the grid for planning.
                  If None, the planner will use the polygon based methods for planning.

        These parameters apply to both RRT and PRM planners.

        2. max_nodes_sampled (int): Specifies the maximum number of nodes to be generated for sampling-
                  based planners.

        3. max_connection_dist (float) : Specifies the maximum distance allowed between two nodes, in meters.

        These parameters are only applicable to RRT planners

        4. rrt_bidirectional (bool) : If true, will use bidirectional RRT.

        5. rrt_connect (bool) :  If true, will use rrt connect.

        6. rrt_star (bool) : If true, will use use rrt star for optimization.

        7. rrt_rewire_radius (float) : The radius within which a node can find a rewire candidate, in meters.
        """

        self.planners = {"astar": AstarPlanner, "rrt": RRTPlanner, "prm": PRMPlanner}

        if planner_type not in self.planners:
            warnings.warn(
                f"{planner_type} is not a supported planner type.", UserWarning
            )
            return None
        if not planner_config:
            warnings.warn(f"No planner configuration provided", UserWarning)
            return None

        self.planner_type = planner_type
        self.planner_config = planner_config
        self.planner = self.planners[self.planner_type](self.planner_config)

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

        self.latest_path = self.planner.plan(start, goal)
        return self.latest_path

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

        return self.planner.plot(axes, path_color)

    def show(self):
        """Displays the planned path on the GUI."""

        self.planner.show()

    def info(self):
        """Display information about planning process."""

        self.planner.info()
