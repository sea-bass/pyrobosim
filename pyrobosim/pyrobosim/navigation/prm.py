""" Probabilistic Roadmap (PRM) implementation. """

import time
import warnings

from .world_graph import WorldGraph, Node
from ..utils.motion import Path
from ..utils.pose import Pose
from .a_star import AStarGraph
from ..utils.motion import reduce_waypoints_polygon
from pyrobosim.navigation.planner_base import PathPlannerBase


class PRMPlannerPolygon:
    """
    Polygon representation based implementation of PRM.
    """

    def __init__(
        self, max_connection_dist=2.0, max_nodes=50, compress_path=False, world=None
    ):
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
        self.world = world
        self.compress_path = compress_path

        self.path_finder = AStarGraph()
        # for key, value in planner_config.items():
        #     setattr(self, key, value)
        self.reset()

    def reset(self):
        """Resamples the PRM and resets planning metrics."""
        self.planning_time = self.sampling_time = 0.0
        self.latest_path = Path()

        # Create a search graph and sample nodes.
        self.graph = WorldGraph(color=[0, 0.4, 0.8], color_alpha=0.25)
        self.graph.was_updated = True
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
            if self.is_connectable(node.pose, other.pose):
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
        # Reset the path and time
        self.latest_path = Path()
        self.planning_time = 0.0
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
        waypoints = self.path_finder.plan(start, goal)
        # Return empty path if no path was found.
        if not waypoints:
            return self.latest_path

        path_poses = [waypoint.pose for waypoint in waypoints]
        if self.compress_path:
            path_poses = reduce_waypoints_polygon(self.world, path_poses)
        self.latest_path = Path(poses=path_poses)
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
        return self.world.is_connectable(poseA, poseB, self.max_connection_dist)

    def get_graphs(self):
        """Returns the graphs generated by the planner if any."""
        return [self.graph]


class PRMPlanner(PathPlannerBase):
    """Factory class for Probabilistic RoadMap path planner."""

    def __init__(self, **planner_config):
        """
        Creates an instance of PRM planner.
        """
        super().__init__()

        self.impl = None

        if planner_config.get("grid", None):
            raise NotImplementedError("Grid based PRM is not supported. ")
        else:
            self.impl = PRMPlannerPolygon(**planner_config)

    def plan(self, start, goal):
        """
        Plans a path from start to goal.

        :param start: Start pose.
        :type start: :class:`pyrobosim.utils.pose.Pose`
        :param goal: Goal pose.
        :type goal: :class:`pyrobosim.utils.pose.Pose`
        :return: Path from start to goal.
        :rtype: :class:`pyrobosim.utils.motion.Path`
        """
        start_time = time.time()
        self.latest_path = self.impl.plan(start, goal)
        self.planning_time = time.time() - start_time
        self.graphs = self.impl.get_graphs()
        return self.latest_path
