""" Implementation of the A* path planner. """

import math
import warnings
from astar import AStar
from pyrobosim.utils.pose import Pose
from pyrobosim.utils.motion import Path
from pyrobosim.navigation.planner_base import PathPlannerBase


class GridSearch(AStar):
    def __init__(self, config) -> None:
        super().__init__()

        self.grid = config["grid"]
        self.heuristic = config["heuristic"]
        self.diagonal_motion = config["diagonal_motion"]
        self._set_actions()
        self._set_heuristic()

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
        :type cell1: Tuple (int, int)
        :param cell2: The destination cell
        :type cell2: Tuple (int, int)
        """
        return self._heuristic(cell1, cell2)

    def distance_between(self, cell1, cell2):
        """
        Computes the distance between 2 cells in the grid.

        :param cell1: The source cell
        :type cell1: Tuple (int, int)
        :param cell2: The destination cell
        :type cell2: Tuple (int, int)
        """
        x1, y1 = cell1
        x2, y2 = cell2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def neighbors(self, cell):
        """
        Get the neighbors of a cell in the grid.

        :param cell: The cell for which the neighbors need to be computed.
        :type cell: Tuple (int, int)
        """
        neighbours_list = []
        for action in self.selected_actions:
            dx, dy = self.actions[action]["action"]
            x, y = cell[0] + dx, cell[1] + dy
            if not self.grid.is_occupied((x, y)):
                neighbours_list.append((x, y))
        return neighbours_list

    def plan(self, start, goal):
        """
        Plans a path from start to goal.

        :param start: The start position in grid.
        :type start: Tuple (int, int)
        :param goal: The goal position in grid.
        :type goal: Tuple (int, int)
        """
        start = self.grid.world_to_grid((start.x, start.y))
        goal = self.grid.world_to_grid((goal.x, goal.y))
        path = self.astar(start, goal)
        world_path = []
        for waypoint in path:
            world_x, world_y = self.grid.grid_to_world(waypoint)
            world_path.append(Pose(world_x, world_y))
        self.latest_path = Path(poses=world_path)
        return self.latest_path


class AstarPlanner(PathPlannerBase):
    """The A* path planner."""

    def __init__(self, planner_config):
        """
        Creates an instance of A* planner.

        :param planner_config: The configuration to be used.
        :type planner_config: dict
        """
        self.impl = None  # Holds the implementation.

        # Depending on if grid is provided, select the implementation.
        if planner_config["grid"]:
            self.impl = GridSearch(planner_config)
        else:
            # Will be implemented after restructuring of the World class
            raise NotImplementedError("A* does not support graph based search")

    def plan(self, start, goal):
        self.latest_path = self.impl.plan(start, goal)
        return self.latest_path
