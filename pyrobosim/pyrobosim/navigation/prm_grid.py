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
    """
    Implementation of grid based Probabilistic Roadmaps (PRM) for motion planning.
    """

    def __init__(
        self,
        world,
        resolution=0.05,
        inflation_radius=0.15,
        max_nodes=100,
        max_connection_dist=2.0,
        compress_path=True,
        max_time=5.0
    ):
        """
        Creates an instance of a PRM planner.

        :param world: World object to use in the planner.
        :type world: :class:`pyrobosim.core.world.World`
        :param resolution: The resolution to be used in the occupancy grid, in meters.
        :type resolution: float
        :param inflation_radius: The inflation radius to be used for the planner's occupancy grid, in meters.
        :type inflation_radius: float
        :param max_nodes: Maximum nodes sampled to build the PRM.
        :type max_nodes: int
        :param max_connection_dist: Maximum connection distance between nodes.
        :type max_connection_dist: float
        :param compress_path: If true, waypoint reduction will be applied to generated path, else full path is returned.
        :type compress_path: bool
        :param max_time: Maximum time allowed for planning, in seconds.
        :type max_time: float
        """
        self.world = world
        self.max_time = max_time
        self.max_nodes = max_nodes 
        self.resolution = resolution      
        self.compress_path = compress_path
        self.inflation_radius = inflation_radius
        # Since the connection distance is specified in meters, we have to convert it into grid distance
        # eg : 2.0 m --> (2.0 / 0.05) = 40 cells
        self.max_connection_dist = max_connection_dist / self.resolution

        self.grid = occupancy_grid_from_world(
            world=self.world,
            resolution=self.resolution,
            inflation_radius=self.inflation_radius,
        )

        self.nodes = []
        self.planning_time = 0
        self.latest_path = Path()
        self.solver = GraphSolver()
        self.np_nodes = np.empty((self.max_nodes, 2), dtype=np.int64)
        self._generate_graph()

    def _generate_graph(self):
        """
        Generates the PRM graph.
        """
        node_count = 0
        max_dim = max(self.grid.width, self.grid.height) - 1
        while node_count < self.max_nodes:
            x, y = np.random.randint(0, max_dim, (2, ))
            if not self.grid.is_occupied((x, y)):
                self.nodes.append(Node(Pose(x, y)))
                self.np_nodes[node_count, :] = [x, y]
                node_count += 1

        # Find nodes within max_connection_dist
        for i in range(len(self.nodes)):
            distances = np.linalg.norm(self.np_nodes[i] - self.np_nodes, axis=1)
            potential_neighbors = np.where( (distances < self.max_connection_dist) & (distances > 0) )

            # If the current node can be connected to any of the potential neighbors, add them as a neighbour
            xa, ya = self.np_nodes[i]
            for idx in potential_neighbors[0]:
                xb, yb = self.np_nodes[idx]
                if self.grid.connectable((xa, ya), (xb, yb))[0]:
                    self.nodes[i].neighbors.add(self.nodes[idx])
                    self.nodes[idx].neighbors.add(self.nodes[i])

    def _add_start_goal(self,start, goal):
        """
        Add the start and goal to the graph for planning.

        :param start: The start node
        :type start: :class: `pyrobosim.navigation.search_graph.Node`
        :param goal: The goal node
        :type goal: :class: `pyrobosim.navigation.search_graph.Node`        
        """

        self.nodes.append(start)
        self.nodes.append(goal)

        # Add neighbors of start 
        distances = np.linalg.norm([start.pose.x, start.pose.y] - self.np_nodes, axis=1)
        potential_neighbors = np.where( (distances < self.max_connection_dist) & (distances > 0) )
        # If the current node can be connected to any of the potential neighbors, add them as a neighbour
        xa, ya = [start.pose.x, start.pose.y]
        for idx in potential_neighbors[0]:
            xb, yb = self.np_nodes[idx]
            if self.grid.connectable((xa, ya), (xb, yb))[0]:
                self.nodes[-2].neighbors.add(self.nodes[idx])
                self.nodes[idx].neighbors.add(self.nodes[-2])

        # Add neighbors of goal 
        distances = np.linalg.norm([goal.pose.x, goal.pose.y] - self.np_nodes, axis=1)
        potential_neighbors = np.where( (distances < self.max_connection_dist) & (distances > 0) )
        # If the current node can be connected to any of the potential neighbors, add them as a neighbour
        xa, ya = [goal.pose.x, goal.pose.y]
        for idx in potential_neighbors[0]:
            xb, yb = self.np_nodes[idx]
            if self.grid.connectable((xa, ya), (xb, yb))[0]:
                self.nodes[-1].neighbors.add(self.nodes[idx])
                self.nodes[idx].neighbors.add(self.nodes[-1])

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
            poses = []
            for node in path:
                world_x, world_y = self.grid.grid_to_world((node.pose.x, node.pose.y))
                poses.append(Pose(world_x, world_y))

            return Path(poses=poses)

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
        
        start = Node(Pose(x = self.start[0], y = self.start[1]))
        goal = Node(Pose(x = self.goal[0], y = self.goal[1]))
        # TODO : Add start and goal to graph 
        self._add_start_goal(start, goal)
        self.latest_path = self._find_path(start, goal)
        self.latest_path.fill_yaws()
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

    def show(self, show_path=True):
        """
        Shows the A* the planned path in a new figure.

        :param show_path: If True, shows the last planned path.
        :type show_path: bool
        """
        import matplotlib.pyplot as plt

        f = plt.figure()
        ax = f.add_subplot(111)
        self.plot(ax, show_path=show_path)
        plt.title("A*")
        plt.axis("equal")
        plt.show()
