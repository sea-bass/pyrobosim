"""World graph planner implementation."""

import time
import itertools

from .types import PathPlanner
from ..core.locations import Location
from ..utils.path import Path
from ..utils.pose import Pose
from ..utils.search_graph import SearchGraph, Node
from ..utils.world_motion_planning import reduce_waypoints_polygon


class WorldGraphPlanner(PathPlanner):
    """
    Implementation of a path planner which creates a visibility-based roadmap using the polygons in the world.
    """

    def __init__(
        self,
        *,
        world,
        collision_check_step_dist=0.025,
        max_connection_dist=None,
        compress_path=False,
    ):
        """
        Creates an instance of a world graph planner.

        :param world: World object to use in the planner.
        :type world: :class:`pyrobosim.core.world.World`
        :param collision_check_step_dist: Step size for discretizing collision checking.
        :type collision_check_step_dist: float
        :param max_connection_dist: Maximum connection distance between nodes.
        :type max_connection_dist: float
        :param compress_path: If true, tries to shorten the path with polygon-based collision checks.
        :type compress_path: bool
        """
        # Parameters
        self.collision_check_step_dist = collision_check_step_dist
        self.max_connection_dist = max_connection_dist
        self.world = world
        self.compress_path = compress_path

        self.planning_time = 0.0
        self.latest_path = Path()

        self.reset()

    def reset(self):
        """
        Initializes the graph from the entity nodes in the world linked to this planner.
        """
        # Create a search graph from the nodes in the world.
        self.graph = SearchGraph(color=[0, 0.4, 0.8], color_alpha=0.5, use_planner=True)
        for entity in itertools.chain(
            self.world.rooms, self.world.hallways, self.world.locations
        ):
            entity.add_graph_nodes()
            if isinstance(entity, Location):
                for spawn in entity.children:
                    for node in spawn.graph_nodes:
                        self.graph.add_node(node)
                        self.connect_neighbors(node)
            else:
                for node in entity.graph_nodes:
                    self.graph.add_node(node)
                    self.connect_neighbors(node)

    def connect_neighbors(self, node):
        """
        Connect a node to all nodes within connection distance.

        :param node: Node to try add to the graph.
        :type node: :class:`pyrobosim.utils.search_graph.Node`
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
        :rtype: :class:`pyrobosim.utils.path.Path`
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
        self.latest_path = self.graph.find_path(start, goal)
        if self.compress_path:
            compressed_poses = reduce_waypoints_polygon(
                self.world, self.latest_path.poses, self.collision_check_step_dist
            )
            self.latest_path.set_poses(compressed_poses)
        self.graph.remove_node(start)
        self.graph.remove_node(goal)

        self.latest_path.fill_yaws()
        self.latest_path.planning_time = time.time() - t_start
        return self.latest_path

    def get_graphs(self):
        """Returns the graphs generated by the planner if any."""
        return [self.graph]

    def get_latest_path(self):
        """
        Returns the latest path generated by the planner, if any.

        :return: List of graphs.
        :rtype: list[:class:`pyrobosim.utils.path.Path`]
        """
        return self.latest_path

    def to_dict(self):
        """
        Serializes the planner to a dictionary.

        :return: A dictionary containing the planner information.
        :rtype: dict[str, Any]
        """
        from .planner_registry import get_planner_string

        return {
            "type": get_planner_string(self),
            "collision_check_step_dist": self.collision_check_step_dist,
            "max_connection_dist": self.max_connection_dist,
            "compress_path": self.compress_path,
        }
