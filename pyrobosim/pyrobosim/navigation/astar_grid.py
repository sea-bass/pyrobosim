""" Grid based A* implementation. """

import time
import math
import warnings
from queue import PriorityQueue
from pyrobosim.utils.pose import Pose
from pyrobosim.utils.motion import Path
from pyrobosim.navigation.occupancy_grid import occupancy_grid_from_world


class AStarGridPlanner:
    """
    Occupancy grid based A* planner
    """

    def __init__(
        self,
        world,
        resolution=0.05,
        inflation_radius=0.0,
        distance_metric="euclidean",
        diagonal_motion=True,
        max_time=5.0,
    ):
        """
        Creates a grid based A* planner

        :param world: World object to use in the planner.
        :type world: :class:`pyrobosim.core.world.World`
        :param resolution: The resolution to be used in the occupancy grid, in meters.
        :type resolution: float
        :param inflation_radius: The inflation radius to be used for the planner's occupancy grid, in meters.
        :type inflation_radius: float
        :param distance_metric: The metric to be used as heuristic ('manhattan' or 'euclidean').
        :type distance_metric: string
        :param diagonal_motion: If true, expand nodes using diagonal motion.
        :type diagonal_motion: bool
        :param max_time: Maximum time allowed for planning, in seconds.
        :type max_time: float
        """

        self.goal = None
        self.start = None
        self.world = world
        self.planning_time = 0
        self.max_time = max_time
        self.num_nodes_expanded = 0
        self.resolution = resolution
        self.distance_metric = distance_metric
        self.diagonal_motion = diagonal_motion
        self.inflation_radius = inflation_radius
        self.candidates = PriorityQueue()
        self.latest_path = Path()

        self._set_actions()
        self._set_heuristic()
        self._set_occupancy_grid()

    def _set_occupancy_grid(self):
        """
        Generates occupancy grid of specified configuration
        """
        ts = time.time()
        self.grid = occupancy_grid_from_world(
            self.world,
            resolution=self.resolution,
            inflation_radius=self.inflation_radius,
        )
        self.grid_generation_time = time.time() - ts

    def _set_actions(self):
        """
        Generates the actions available
        """
        D1 = 1
        D2 = math.sqrt(2)
        self.actions = {
            "left": {"cost": D1, "action": (-1, 0)},
            "right": {"cost": D1, "action": (1, 0)},
            "up": {"cost": D1, "action": (0, 1)},
            "down": {"cost": D1, "action": (0, -1)},
            "left_up": {"cost": D2, "action": (-1, 1)},
            "left_down": {"cost": D2, "action": (-1, -1)},
            "right_up": {"cost": D2, "action": (1, 1)},
            "right_down": {"cost": D2, "action": (1, -1)},
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

        if self.distance_metric == "euclidean":
            self._heuristic = euclidean
        elif self.distance_metric == "manhattan":
            if not self.diagonal_motion:
                warnings.warn("Manhattan over estimates without diagonal motion")
            self._heuristic = manhattan

    def _is_valid_start_goal(self, start, goal):
        """
        Validate the start and goal locations provided to the planner

        :return: True if the start and goal given to the planner are not occupied, else False
        :rtype: bool
        """
        valid = True
        if self.grid.is_occupied(start):
            warnings.warn(f"Start position {start} is occupied")
            valid = False
        if self.grid.is_occupied(goal):
            valid = False
            warnings.warn(f"Goal position {goal} is occupied")
        return valid

    def plan(self, start, goal):
        """
        Plans a path from start to goal.

        :param start: The start pose in world coordinates.
        :type start: :class:`pyrobosim.utils.pose.Pose`
        :param goal: The goal pose in world coordinates.
        :type goal: :class:`pyrobosim.utils.pose.Pose`
        :return: Path from start to goal.
        :rtype: :class:`pyrobosim.utils.motion.Path`
        """
        self.latest_path = Path()
        self.num_nodes_expanded = 0
        self.planning_time = 0
        start = self.grid.world_to_grid((start.x, start.y))
        goal = self.grid.world_to_grid((goal.x, goal.y))
        # If start or goal position is occupied, return empty path with warning.
        if not self._is_valid_start_goal(start, goal):
            return self.latest_path

        candidates = PriorityQueue()
        parent_of = dict()
        cost_till = dict()

        path_found = False
        timed_out = False
        start_time = time.time()
        
        parent_of[start] = None
        cost_till[start] = 0.0
        candidates.put(((cost_till[start] + self._heuristic(start, goal)), start))
        while not path_found and not candidates.empty() and not timed_out:

            current = candidates.get()[1]
            if current == goal:
                path_found = True
                break
            
            # Expand the current node
            x, y = current
            for a in self.selected_actions:
                action  = self.actions[a]["action"]
                next  = (x + action[0], y + action[1])
                cost_new = cost_till[current] + self.actions[a]["cost"]
                if not self.grid.is_occupied(next) and (next not in cost_till or cost_new < cost_till[next]):
                    expected_toal_cost = cost_new + self._heuristic(next, goal)
                    candidates.put((expected_toal_cost,next))
                    parent_of[next] = current
                    cost_till[next] = cost_new
                    self.num_nodes_expanded += 1

            self.planning_time = (time.time() - start_time)
            timed_out = self.planning_time >= self.max_time
        
        # Generate the path if a path was found
        if path_found:
            waypoints = []
            while current is not None:
                x, y = self.grid.grid_to_world((current[0], current[1]))
                waypoints.append(Pose(x, y))
                current = parent_of[current]
            waypoints.reverse()
            print(waypoints)
            self.latest_path = Path(poses=waypoints)
            self.latest_path.fill_yaws()
            print("Path found")
            print(self.latest_path)
            return self.latest_path
        
        else:
            print("No path found")
            return self.latest_path
        
    def print_metrics(self):
        """Print metrics about the latest path computed."""
        if self.latest_path.num_poses == 0:
            print("No path.")
        else:
            print("Latest path from A*:")
            self.latest_path.print_details()
        print("\n")
        print(f"Selected actions : {self.selected_actions}")
        print(f"Selected heuristic: {self.distance_metric}")
        print(f"Occupancy grid generated in : {self.grid_generation_time} seconds")
        print(f"Time to plan: {self.planning_time} seconds")
        print(f"Number of nodes expanded : {self.num_nodes_expanded}")
        # print(f"Number of waypoints in path : {self.latest_path.num_poses}")

    def plot(self, axes, path_color="m", show_path=True):
        """
        Plots the planned path on a specified set of axes.

        :param axes: The axes on which to draw.
        :type axes: :class:`matplotlib.axes.Axes`
        :param path_color: Color of the path, as an RGB tuple or string.
        :type path_color: tuple[float] / str, optional
        :param show_graph: If True, shows the RRTs used for planning.
        :type show_graph: bool
        :param show_path: If True, shows the last planned path.
        :type show_path: bool
        :return: List of Matplotlib artists containing what was drawn,
            used for bookkeeping.
        :rtype: list[:class:`matplotlib.artist.Artist`]
        """
        artists = []
        x = [p.x for p in self.latest_path.poses]
        y = [p.y for p in self.latest_path.poses]
        (path,) = axes.plot(
            x, y, linestyle="-", color=path_color, linewidth=3, alpha=0.5, zorder=1
        )
        (start,) = axes.plot(x[0], y[0], "go", zorder=2)
        (goal,) = axes.plot(x[-1], y[-1], "rx", zorder=2)
        artists.extend((path, start, goal))

        return artists

    def show(self, show_graph=True, show_path=True):
        """
        Shows the RRTs and the planned path in a new figure.

        :param show_graph: If True, shows the RRTs used for planning.
        :type show_graph: bool
        :param show_path: If True, shows the last planned path.
        :type show_path: bool
        """
        import matplotlib.pyplot as plt

        f = plt.figure()
        ax = f.add_subplot(111)
        self.plot(ax, show_graph=show_graph, show_path=show_path)
        plt.title("A*")
        plt.axis("equal")
        plt.show()