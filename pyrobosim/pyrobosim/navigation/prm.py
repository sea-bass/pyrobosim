"""Probabilistic Roadmap (PRM) implementation."""

import time
from typing import Any

from .types import PathPlanner
from ..core.world import World
from ..utils.path import Path
from ..utils.pose import Pose
from ..utils.search_graph import SearchGraph, Node
from ..utils.world_motion_planning import reduce_waypoints_polygon


class PRMPlanner(PathPlanner):
    """
    Implements a Probabilistic Roadmap (PRM) path planner.
    """

    def __init__(
        self,
        *,
        world: World,
        compress_path: bool = False,
        collision_check_step_dist: float = 0.025,
        max_connection_dist: float = 2.0,
        max_nodes: int = 50,
    ) -> None:
        """
        Creates an instance of a PRM planner.

        :param world: World object to use in the planner.
        :param compress_path: If true, tries to shorten the path with polygon-based collision checks.
        :param collision_check_step_dist: Step size for discretizing collision checking.
        :param max_connection_dist: Maximum connection distance between nodes.
        :param max_nodes: Maximum nodes sampled to build the PRM.
        """
        # Parameters
        self.collision_check_step_dist = collision_check_step_dist
        self.max_connection_dist = max_connection_dist
        self.max_nodes = max_nodes
        self.world = world
        self.compress_path = compress_path

        self.reset()

    def reset(self) -> None:
        """Resamples the PRM and resets planning metrics."""
        self.latest_path = Path()

        # Create a search graph and sample nodes.
        self.graph = SearchGraph(
            color=[0, 0.4, 0.8], color_alpha=0.25, use_planner=True
        )
        for i in range(self.max_nodes):
            n_sample = self.sample_configuration()
            if n_sample is None:
                self.world.logger.warning(f"Could not sample more than {i} nodes.")
                break
            node = Node(pose=n_sample)
            self.graph.add_node(node)
            self.connect_neighbors(node)

    def connect_neighbors(self, node: Node) -> None:
        """
        Connect a node to all nodes within connection distance.

        :param node: Node to try add to the graph.
        """
        for other in self.graph.nodes:
            if node == other:
                continue
            if self.world.is_connectable(
                node.pose,
                other.pose,
                self.collision_check_step_dist,
                self.max_connection_dist,
            ):
                self.graph.add_edge(node, other)

    def plan(self, start: Pose | Node, goal: Pose | Node) -> Path:
        """
        Plans a path from start to goal.

        :param start: Start pose or graph node.
        :param goal: Goal pose or graph node.
        :return: Path from start to goal.
        """
        # Reset the path and time
        self.latest_path = Path()
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
        self.latest_path = self.graph.find_path(start, goal)
        if self.compress_path:
            compressed_poses = reduce_waypoints_polygon(
                self.world, self.latest_path.poses, self.collision_check_step_dist
            )
            self.latest_path.set_poses(compressed_poses)
        self.latest_path.fill_yaws()
        self.latest_path.planning_time = time.time() - t_start
        self.graph.remove_node(start)
        self.graph.remove_node(goal)
        return self.latest_path

    def sample_configuration(self) -> Pose | None:
        """
        Samples a random configuration from the world.

        :return: Collision-free pose if found, else ``None``.
        """
        return self.world.sample_free_robot_pose_uniform()

    def get_graphs(self) -> list[SearchGraph]:
        """
        Returns the graphs generated by the planner, if any.

        :return: List of graphs.
        """
        return [self.graph]

    def get_latest_path(self) -> Path | None:
        """
        Returns the latest path generated by the planner, if any.

        :return: List of graphs.
        """
        return self.latest_path

    def to_dict(self) -> dict[str, Any]:
        """
        Serializes the planner to a dictionary.

        :return: A dictionary containing the planner information.
        """
        from .planner_registry import get_planner_string

        return {
            "type": get_planner_string(self),
            "collision_check_step_dist": self.collision_check_step_dist,
            "compress_path": self.compress_path,
            "max_connection_dist": self.max_connection_dist,
            "max_nodes": self.max_nodes,
        }
