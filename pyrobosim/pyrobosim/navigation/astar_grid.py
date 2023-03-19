""" Grid based A* implementation. """

import time
import math
import warnings
from queue import PriorityQueue
from pyrobosim.utils.pose import Pose
from pyrobosim.utils.motion import Path, reduce_waypoints
from pyrobosim.navigation import occupancy_grid_from_world
from pyrobosim.navigation.search_graph import Node


class AStarGridPlanner:
    """
    Occupancy grid based A* planner
    """

    def __init__(
        self,
        world,
        resolution=0.05,
        inflation_radius=0.0,
        heuristic="euclidean",
        diagonal_motion=True,
        compress_path=True,
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
        :param heuristic: The metric to be used as heuristic ('manhattan', 'euclidean', 'none').
        :type heuristic: string
        :param diagonal_motion: If true, expand nodes using diagonal motion.
        :type diagonal_motion: bool
        :param compress_path: If true, waypoint reduction will be applied to generated path, else full path is returned.
        :type compress_path: bool
        :param max_time: Maximum time allowed for planning, in seconds.
        :type max_time: float
        """

        self.world = world
        self.max_time = max_time
        self.resolution = resolution
        self.heuristic = heuristic
        self.compress_path = compress_path
        self.diagonal_motion = diagonal_motion
        self.inflation_radius = inflation_radius

        self.goal = None
        self.start = None
        self.planning_time = 0
        self.num_nodes_expanded = 0
        self.candidates = PriorityQueue()
        self.latest_path = Path()
        # This dictionary keeps track of the parent node for each expanded node
        # eg : self.parent_of[(10,10)] = (9,10)
        self.parent_of = {}
        # This dictionary keeps track of the cost till the particular node
        # eg : self.cost_till((10, 10)) = 20
        self.cost_till = {}

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

    def _reset(self):
        """
        Resets the data used by the planner
        """
        self.num_nodes_expanded = 0
        self.planning_time = 0
        self.latest_path = Path()
        self.candidates = PriorityQueue()
        self.parent_of.clear()
        self.cost_till.clear()

    def _expand(self, current):
        """
        Expands the given node

        :param current: The node to expand
        :type current: (int, int)
        """
        x, y = current
        for action in self.selected_actions:
            delta = self.actions[action]["action"]
            next = (x + delta[0], y + delta[1])
            cost_new = self.cost_till[current] + self.actions[action]["cost"]
            update_candidates = (
                next not in self.cost_till or cost_new < self.cost_till[next]
            )
            if not self.grid.is_occupied(next) and update_candidates:
                expected_toal_cost = cost_new + self._heuristic(next, self.goal)
                self.candidates.put((expected_toal_cost, next))
                self.parent_of[next] = current
                self.cost_till[next] = cost_new
                self.num_nodes_expanded += 1

    def _get_best_candidate(self):
        """
        Returns the candidate with best metric
        :return: The candidate with the best metric (least cost)
        :rtype: (int, int)
        """
        return self.candidates.get()[1]

    def _generate_path(self, current):
        """
        Generates the full path if a path was found

        :param current: The node from which to trace back the path
        :type current: (int, int)
        """

        waypoints = []
        while current is not None:
            waypoints.append(current)
            current = self.parent_of[current]
        waypoints.reverse()
        if self.compress_path:
            waypoints = reduce_waypoints(self.grid, waypoints)
        poses = []
        for point in waypoints:
            world_x, world_y = self.grid.grid_to_world(point)
            poses.append(Pose(world_x, world_y))
        self.latest_path = Path(poses=poses)
        self.latest_path.fill_yaws()

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
        self._reset()
        if isinstance(start, Node):
            start = start.pose
        if isinstance(goal, Node):
            goal = goal.pose
        self.start = self.grid.world_to_grid((start.x, start.y))
        self.goal = self.grid.world_to_grid((goal.x, goal.y))
        # If start or goal position is occupied, return empty path with warning.
        if not self._is_valid_start_goal():
            return self.latest_path

        path_found = False
        timed_out = False
        start_time = time.time()
        # Add start node data
        self.parent_of[self.start] = None
        self.cost_till[self.start] = 0.0
        expected_toal_cost = self.cost_till[self.start] + self._heuristic(
            self.start, self.goal
        )
        self.candidates.put((expected_toal_cost, self.start))
        # Search for a path
        while not path_found and not self.candidates.empty() and not timed_out:
            current = self._get_best_candidate()
            if current == self.goal:
                path_found = True
                break
            self._expand(current)
            self.planning_time = time.time() - start_time
            timed_out = self.planning_time >= self.max_time
        # Generate the path if a path was found
        if path_found:
            self._generate_path(current)
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
        print(f"Selected heuristic: {self.heuristic}")
        print(f"Occupancy grid generated in : {self.grid_generation_time} seconds")
        print(f"Time to plan: {self.planning_time} seconds")
        print(f"Number of nodes expanded : {self.num_nodes_expanded}")
        print(f"Number of waypoints in path : {self.latest_path.num_poses}")

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
