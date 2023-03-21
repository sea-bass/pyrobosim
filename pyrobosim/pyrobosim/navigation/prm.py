""" Probabilistic Roadmap (PRM) implementation. """

import time
import warnings

from .search_graph import SearchGraph, Node
from ..utils.motion import Path
from ..utils.pose import Pose
from pyrobosim.navigation.planner_base import PathPlannerBase


class PRMPlannerPolygon:
    """
    Implementation of Probabilistic Roadmaps (PRM) for motion planning.
    """

    def __init__(self, planner_config):
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
        self.max_connection_dist = 2.0
        self.max_nodes = 100
        self.world = None

        for key, value in planner_config.items():
            setattr(self, key, value)
        self.reset()

    def reset(self):
        """Resamples the PRM and resets planning metrics."""
        self.planning_time = self.sampling_time = 0.0
        self.latest_path = Path()

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


class PRMPlanner(PathPlannerBase):
    """Factort class for PRM path planner."""

    def __init__(self, planner_config):
        super().__init__()

        self.impl = None

        if planner_config["grid"]:
            raise NotImplementedError("Grid based PRM is not supported. ")
        else:
            self.impl = PRMPlannerPolygon(planner_config)

    def plan(self, start, goal):
        self.latest_path = self.impl.plan(start, goal)
        return self.latest_path
