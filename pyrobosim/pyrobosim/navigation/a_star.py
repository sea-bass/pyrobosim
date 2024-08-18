""" Implementation of the A* path planner. """

import math
import time
import warnings
from astar import AStar

from .occupancy_grid import OccupancyGrid
from ..utils.pose import Pose
from ..utils.motion import Path, reduce_waypoints_grid


class AStarPlanner(AStar):
    """Occupancy grid based implementation of the A* path planning algorithm."""

    def __init__(
        self,
        *,
        world,
        grid_resolution,
        grid_inflation_radius,
        heuristic="euclidean",
        diagonal_motion=True,
        compress_path=False,
    ):
        """
        Creates an instance of grid based A* planner.

        :param world: World object to use in the planner.
        :type world: :class:`pyrobosim.core.world.World`
        :param grid_resolution: The resolution of the occupancy grid, in meters.
        :type grid_resolution: float
        :param grid_inflation_radius: The inflation radius of the occupancy grid, in meters.
        :type grid_inflation_radius: float
        :param heuristic: The metric to be used as heuristic ('manhattan', 'euclidean', 'none').
        :type heuristic: string
        :param diagonal_motion: If true, expand nodes using diagonal motion.
        :type diagonal_motion: bool
        :param compress_path: If true, waypoint reduction will be applied to generated path, else full path is returned.
        :type compress_path: bool
        """
        super().__init__()
        self.world = world
        self.grid_resolution = grid_resolution
        self.grid_inflation_radius = grid_inflation_radius
        self.heuristic = heuristic
        self.diagonal_motion = diagonal_motion
        self.compress_path = compress_path
        self._set_actions()
        self._set_heuristic()
        self.reset()

    def _set_actions(self):
        """
        Generates the actions available
        """
        orthogonal_distance = 1
        diagonal_distance = math.sqrt(2)
        self.actions = {
            "left": {"cost": orthogonal_distance, "action": (-1, 0)},
            "right": {"cost": orthogonal_distance, "action": (1, 0)},
            "up": {"cost": orthogonal_distance, "action": (0, 1)},
            "down": {"cost": orthogonal_distance, "action": (0, -1)},
            "left_up": {"cost": diagonal_distance, "action": (-1, 1)},
            "left_down": {"cost": diagonal_distance, "action": (-1, -1)},
            "right_up": {"cost": diagonal_distance, "action": (1, 1)},
            "right_down": {"cost": diagonal_distance, "action": (1, -1)},
        }
        keys = list(self.actions.keys())
        self.selected_actions = keys if self.diagonal_motion else keys[:4]

    def _set_heuristic(self):
        """
        Sets the heuristic
        """
        euclidean = lambda p1, p2: math.sqrt(
            (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2
        )
        manhattan = lambda p1, p2: abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

        if self.heuristic == "euclidean":
            self._heuristic = euclidean
        elif self.heuristic == "manhattan":
            if not self.diagonal_motion:
                warnings.warn("Manhattan over estimates without diagonal motion")
            self._heuristic = manhattan
        elif self.heuristic == "none":
            self._heuristic = lambda p1, p2: 0
        else:
            warnings.warn(f"Unknown heuristic : {self.heuristic}")
            warnings.warn(f"Defaulting to heuristic : 'none' ")
            self._heuristic = lambda p1, p2: 0

    def heuristic_cost_estimate(self, cell1, cell2):
        """
        Compute heuristic cost estimate using selected heuristic.

        :param cell1: The source cell
        :type cell1: tuple(int, int)
        :param cell2: The destination cell
        :type cell2: tuple(int, int)
        """
        return self._heuristic(cell1, cell2)

    def distance_between(self, cell1, cell2):
        """
        Computes the distance between 2 cells in the grid.

        :param cell1: The source cell
        :type cell1: tuple(int, int)
        :param cell2: The destination cell
        :type cell2: tuple(int, int)
        """
        x1, y1 = cell1
        x2, y2 = cell2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def neighbors(self, cell):
        """
        Get the neighbors of a cell in the grid.

        :param cell: The cell for which the neighbors need to be computed.
        :type cell: tuple(int, int)
        """
        neighbors_list = []
        for action in self.selected_actions:
            dx, dy = self.actions[action]["action"]
            x, y = cell[0] + dx, cell[1] + dy
            if not self.grid.is_occupied((x, y)):
                neighbors_list.append((x, y))
        return neighbors_list

    def reset(self):
        """Resets the occupancy grid."""
        self.grid = OccupancyGrid.from_world(
            self.world,
            resolution=self.grid_resolution,
            inflation_radius=self.grid_inflation_radius,
        )

    def plan(self, start, goal):
        """
        Plans a path from start to goal.

        :param start: Start pose.
        :type start: :class:`pyrobosim.utils.pose.Pose`
        :param goal: Goal pose.
        :type goal: :class:`pyrobosim.utils.pose.Pose`
        """
        t_start = time.time()
        start_grid = self.grid.world_to_grid((start.x, start.y))
        goal_grid = self.grid.world_to_grid((goal.x, goal.y))
        path = self.astar(start_grid, goal_grid)

        # Apply waypoint reduction if enabled.
        if self.compress_path:
            path = reduce_waypoints_grid(self.grid, list(path))

        world_path = []
        if path is not None:
            for waypoint in path:
                world_x, world_y = self.grid.grid_to_world(waypoint)
                world_path.append(Pose(world_x, world_y))
            # Ensure the start and end poses have the full poses (with yaw) set.
            world_path[0] = start
            world_path[-1] = goal

        self.latest_path = Path(poses=world_path)
        self.latest_path.fill_yaws()
        self.latest_path.planning_time = time.time() - t_start
        return self.latest_path

    def get_graphs(self):
        """
        Returns the graphs generated by the planner, if any.

        :return: List of graphs.
        :rtype: list[:class:`pyrobosim.utils.search_graph.SearchGraph`]
        """
        return []

    def get_latest_path(self):
        """
        Returns the latest path generated by the planner, if any.

        :return: List of graphs.
        :rtype: list[:class:`pyrobosim.utils.motion.Path`]
        """
        return self.latest_path
