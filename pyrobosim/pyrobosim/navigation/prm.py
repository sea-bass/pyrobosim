""" Probabilistic Roadmap (PRM) implementation. """

import time
import warnings

from .world_graph import WorldGraph, Node
from ..utils.motion import Path
from ..utils.pose import Pose
from .a_star import AStarGraph
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
        self.path_finder = AStarGraph()
        for key, value in planner_config.items():
            setattr(self, key, value)
        self.reset()

    def reset(self):
        """Resamples the PRM and resets planning metrics."""
        self.planning_time = self.sampling_time = 0.0
        self.latest_path = Path()

        # Create a search graph and sample nodes.
        self.graph = WorldGraph()
        t_start = time.time()
        for i in range(self.max_nodes):
            n_sample = self.sample_configuration()
            if not n_sample:
                warnings.warn(f"Could not sample more than {i} nodes")
                break
            self.graph.add_node(Node(pose=n_sample))
        self.sampling_time = time.time() - t_start

        for node in self.graph.nodes:
            self.connect_neighbors(node)

    def connect_neighbors(self, node):
        # Connect each node to all nodes within connection distance.
        for other in self.graph.nodes:
            if node == other:
                continue
            dist = node.pose.get_linear_distance(other.pose, ignore_z=True)
            if dist <= self.max_connection_dist and self.is_connectable(
                node.pose, other.pose
            ):
                self.graph.add_edge(node, other)

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
        self.graph.add_node(start)
        if isinstance(goal, Pose):
            goal = Node(goal, parent=None)
        self.graph.add_node(goal)

        self.connect_neighbors(start)
        self.connect_neighbors(goal)

        # Find a path from start to goal nodes
        t_start = time.time()
        waypoints = self.path_finder.astar(
            start, goal
        )  # self.graph.find_path(start, goal)
        self.latest_path = Path(poses=[waypoint.pose for waypoint in waypoints])
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

    def is_connectable(self, poseA, poseB):
        return self.world.is_connectable(poseA, poseB)


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
        start_time = time.time()
        self.latest_path = self.impl.plan(start, goal)
        self.planning_time = time.time() - start_time
        return self.latest_path
