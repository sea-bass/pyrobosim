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

        self.world = world
        self.max_time = max_time
        self.resolution = resolution
        self.distance_metric = distance_metric
        self.diagonal_motion = diagonal_motion
        self.inflation_radius = inflation_radius

        self.goal = None
        self.start = None
        self.planning_time = 0
        self.num_nodes_expanded = 0
        self.candidates = PriorityQueue()
        self.latest_path = Path()
        self.parent_of = dict()
        self.cost_till = dict()

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
        for a in self.selected_actions:
            action = self.actions[a]["action"]
            next = (x + action[0], y + action[1])
            cost_new = self.cost_till[current] + self.actions[a]["cost"]
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
        """
        return self.candidates.get()[1]

    def _reduce_waypoints(self, poses):
        """
        Reduce the number of waypoints by removing points on same line

        :param poses: The poses for the full path generated by A*
        :type poses: list[:class:`pyrobosim.utils.pose.Pose`]
        :return: The reduced list of waypoints
        :rtype: list[:class:`pyrobosim.utils.pose.Pose`]
        """
        waypoints = []
        len_poses = len(poses)
        if len_poses <= 2:
            return poses
        slope = lambda p1, p2: ((p2.x - p1.x), (p2.y - p1.y))
        curr_start = poses[0]
        curr_dx, curr_dy = slope(poses[0], poses[1])
        for i in range(2, len_poses):
            new_dx, new_dy = slope(curr_start, poses[i])
            if not math.isclose((curr_dy * new_dx), (new_dy * curr_dx)):
                waypoints.append(curr_start)
                curr_dx, curr_dy = new_dx, new_dy
                curr_start = poses[i - 1]
        waypoints.append(poses[-1])
        return waypoints

    def _generate_path(self, current):
        """
        Generates the full path if a path was found

        :param current: The node from which to trace back the path
        :type current: (int, int)
        """

        waypoints = []
        while current is not None:
            x, y = self.grid.grid_to_world(current)
            waypoints.append(Pose(x, y))
            current = self.parent_of[current]
        waypoints.reverse()
        waypoints = self._reduce_waypoints(waypoints)
        self.latest_path = Path(poses=waypoints)
        self.latest_path.fill_yaws()

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
        self._reset()
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
        print(f"Selected heuristic: {self.distance_metric}")
        print(f"Occupancy grid generated in : {self.grid_generation_time} seconds")
        print(f"Time to plan: {self.planning_time} seconds")
        print(f"Number of nodes expanded : {self.num_nodes_expanded}")
        print(f"Number of waypoints in path : {self.latest_path.num_poses}")

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
