""" Grid based Probablistic Roadmap implementation. """

import time
import warnings
import numpy as np

from ..utils.pose import Pose
from ..utils.motion import Path
from ..navigation.search_graph import Node
from ..navigation.search_graph import GraphSolver
from ..navigation.occupancy_grid import occupancy_grid_from_world


class PRMGridPlanner:
    def __init__(
        self,
        world,
        resolution=0.05,
        inflation_radius=0.15,
        max_nodes=100,
        max_connection_dist=2.0,
    ):
        self.world = world
        self.resolution = resolution
        self.inflation_radius = inflation_radius
        self.max_nodes = max_nodes
        # Since the connection distance is specified in meters, we have to convert it into grid distance
        # eg : 2.0 m --> (2.0 / 0.05) = 40 cells
        self.max_connection_dist = max_connection_dist / self.resolution

        self.grid = occupancy_grid_from_world(
            world=self.world,
            resolution=self.resolution,
            inflation_radius=self.inflation_radius,
        )
        self.latest_path = Path()
        self.nodes = []
        self.solver = GraphSolver()
        self._generate_graph()

    def _generate_graph(self):
        
        node_count = 0
        max_dim = max(self.grid.width, self.grid.height) - 1
        np_nodes = np.empty((self.max_nodes, 2), dtype=np.int64)
        while node_count < self.max_nodes:
            x, y = np.random.randint(0, max_dim, (2, ))
            if not self.grid.is_occupied((x, y)):
                self.nodes.append(Node(Pose(x, y)))
                np_nodes[node_count, :] = [x, y]
                node_count += 1

        # Find nodes within max_connection_dist
        for i in range(len(self.nodes)):
            distances = np.linalg.norm(np_nodes[i] - np_nodes, axis=1)
            potential_neighbors = np.where( (distances < self.max_connection_dist) & (distances > 0) )

            # If the current node can be connected to any of the potential neighbors, add them as a neighbour
            xa, ya = np_nodes[i]
            for idx in potential_neighbors[0]:
                xb, yb = np_nodes[idx]
                if self.grid.connectable((xa, ya), (xb, yb)):
                    self.nodes[i].neighbors.add(self.nodes[idx])
                    self.nodes[idx].neighbors.add(self.nodes[i])

        self.np_nodes = np_nodes

    def _is_valid_start_goal(self):
        """
        Validate the start and goal locations provided to the planner

        :return: True if the start and goal given to the planner are not occupied, else False
        :rtype: bool
        """
        valid = True
        if self.grid.is_occupied(self.start):
            warnings.warn(f"Start position {self.start} is occupied")
            valid = False
        if self.grid.is_occupied(self.goal):
            valid = False
            warnings.warn(f"Goal position {self.goal} is occupied")
        return valid

    def _find_path(self, start, goal):
        """
        Gets a path from start to goal poses by searching the graph
        The path consists of a tuple of Pose objects

        :param start: Start node
        :type start: :class:`Node`
        :param goal: Goal node
        :type goal: :class:`Node`
        :return: Path from start to goal.
        :rtype: :class:`pyrobosim.utils.motion.Path`
        """
        path = self.solver.astar(start, goal)
        if path is None:
            warnings.warn("Did not find a path from start to goal.")
            return Path()
        else:
            return Path(poses=[p.pose for p in path])

    def plan(self, start, goal):
        if isinstance(start, Node):
            start = start.pose
        if isinstance(goal, Node):
            goal = goal.pose
        self.start = self.grid.world_to_grid((start.x, start.y))
        self.goal = self.grid.world_to_grid((goal.x, goal.y))
        # If start or goal position is occupied, return empty path with warning.
        if not self._is_valid_start_goal():
            return self.latest_path
        
        start = Node(start)
        goal = Node(goal)
        # TODO : Add start and goal to graph 
        return self._find_path(start, goal)