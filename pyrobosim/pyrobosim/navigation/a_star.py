"""Implementation of the A* path planner."""

from dataclasses import dataclass
import math
import time
from typing import Any

from astar import AStar

from .types import PathPlanner
from ..utils.pose import Pose
from ..utils.path import Path
from ..utils.search_graph import SearchGraph


@dataclass
class AStarAction:
    """Stores information about A* actions."""

    cost: float
    action: tuple[int, int]


ORTHOGONAL_DISTANCE = 1.0
DIAGONAL_DISTANCE = math.sqrt(2.0)
ASTAR_ACTIONS = {
    "left": AStarAction(cost=ORTHOGONAL_DISTANCE, action=(-1, 0)),
    "right": AStarAction(cost=ORTHOGONAL_DISTANCE, action=(1, 0)),
    "up": AStarAction(cost=ORTHOGONAL_DISTANCE, action=(0, 1)),
    "down": AStarAction(cost=ORTHOGONAL_DISTANCE, action=(0, -1)),
    "left_up": AStarAction(cost=DIAGONAL_DISTANCE, action=(-1, 1)),
    "left_down": AStarAction(cost=DIAGONAL_DISTANCE, action=(-1, -1)),
    "right_up": AStarAction(cost=DIAGONAL_DISTANCE, action=(1, 1)),
    "right_down": AStarAction(cost=DIAGONAL_DISTANCE, action=(1, -1)),
}


class AStarPlanner(PathPlanner, AStar):  # type: ignore[misc]
    """Occupancy grid based implementation of the A* path planning algorithm."""

    plugin_name = "astar"  # Needed to register plugin.

    def __init__(
        self,
        *,
        grid_resolution: float,
        grid_inflation_radius: float,
        heuristic: str = "euclidean",
        diagonal_motion: bool = True,
        compress_path: bool = False,
    ) -> None:
        """
        Creates an instance of grid based A* planner.

        :param grid_resolution: The resolution of the occupancy grid, in meters.
        :param grid_inflation_radius: The inflation radius of the occupancy grid, in meters.
        :param heuristic: The metric to be used as heuristic ('manhattan', 'euclidean', 'none').
        :param diagonal_motion: If true, expand nodes using diagonal motion.
        :param compress_path: If true, waypoint reduction will be applied to generated path, else full path is returned.
        """
        super().__init__()
        self.grid_resolution = grid_resolution
        self.grid_inflation_radius = grid_inflation_radius
        self.heuristic = heuristic
        self.diagonal_motion = diagonal_motion
        self.compress_path = compress_path

    def _set_actions(self) -> None:
        """Generates the actions available."""
        keys = list(ASTAR_ACTIONS.keys())
        self.allowable_actions = keys if self.diagonal_motion else keys[:4]

    def _set_heuristic(self) -> None:
        """Sets the A* heuristic."""
        assert self.world is not None
        if self.heuristic == "euclidean":
            self._heuristic = lambda p1, p2: math.sqrt(
                (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2
            )
        elif self.heuristic == "manhattan":
            if not self.diagonal_motion:
                self.world.logger.warning(
                    "Manhattan distance overestimates costs without diagonal motion."
                )
            self._heuristic = lambda p1, p2: abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
        elif self.heuristic == "none":
            self._heuristic = lambda p1, p2: 0.0
        else:
            self.world.logger.warning(f"Unknown heuristic : {self.heuristic}")
            self.world.logger.warning(f"Defaulting to heuristic : 'none' ")
            self._heuristic = lambda p1, p2: 0.0

    def heuristic_cost_estimate(
        self, cell1: tuple[int, int], cell2: tuple[int, int]
    ) -> float:
        """
        Compute heuristic cost estimate using selected heuristic.

        :param cell1: The source cell.
        :param cell2: The destination cell.
        :return: The heuristic cost estimate.
        """
        return self._heuristic(cell1, cell2)

    def distance_between(self, cell1: tuple[int, int], cell2: tuple[int, int]) -> float:
        """
        Computes the distance between 2 cells in the grid.

        :param cell1: The source cell.
        :param cell2: The destination cell.
        :return: The distance between two cells.
        """
        x1, y1 = cell1
        x2, y2 = cell2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def neighbors(self, cell: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Get the neighbors of a cell in the grid.

        :param cell: The cell for which the neighbors need to be computed.
        :return: The list of neighbors of that cell.
        """
        neighbors_list = []
        for action in self.allowable_actions:
            dx, dy = ASTAR_ACTIONS[action].action
            x, y = cell[0] + dx, cell[1] + dy
            if not self.grid.is_occupied((x, y)):
                neighbors_list.append((x, y))
        return neighbors_list

    def reset(self) -> None:
        """Resets the occupancy grid."""
        from .occupancy_grid import OccupancyGrid

        super().reset()
        if (self.world is None) or (self.robot is None):
            return

        self._set_actions()
        self._set_heuristic()
        self.grid = OccupancyGrid.from_world(
            self.world,
            resolution=self.grid_resolution,
            inflation_radius=self.grid_inflation_radius,
            robot=self.robot,
        )

    def plan(self, start: Pose, goal: Pose) -> Path:
        """
        Plans a path from start to goal.

        :param start: Start pose.
        :param goal: Goal pose.
        :return: Path from start to goal.
        """
        t_start = time.time()
        start_grid = self.grid.world_to_grid((start.x, start.y))
        goal_grid = self.grid.world_to_grid((goal.x, goal.y))
        path = self.astar(start_grid, goal_grid)

        # Apply waypoint reduction if enabled.
        if self.compress_path:
            from .occupancy_grid import reduce_waypoints_grid

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

    def get_graphs(self) -> list[SearchGraph]:
        """
        Returns the graphs generated by the planner, if any.

        :return: List of graphs.
        """
        return []

    def get_latest_path(self) -> Path | None:
        """
        Returns the latest path generated by the planner, if any.

        :return: The latest path if one exists, else None.
        """
        return self.latest_path

    def to_dict(self) -> dict[str, Any]:
        """
        Serializes the planner to a dictionary.

        :return: A dictionary containing the planner information.
        """
        return {
            "type": self.plugin_name,
            "grid_resolution": self.grid_resolution,
            "grid_inflation_radius": self.grid_inflation_radius,
            "heuristic": self.heuristic,
            "diagonal_motion": self.diagonal_motion,
            "compress_path": self.compress_path,
        }
