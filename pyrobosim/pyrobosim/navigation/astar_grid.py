""" Grid based A* implementation. """

import time
import math
import warnings
from queue import PriorityQueue
from pyrobosim.utils.pose import Pose
from pyrobosim.utils.motion import Path
from pyrobosim.navigation.occupancy_grid import occupancy_grid_from_world


class Node:
    """
    A node in the A* path
    """

    def __init__(self, x, y, g, h, parent=None) -> None:
        self.x = x
        self.y = y
        self.g = g  # Cost to reach this node from start
        self.h = h  # Approximate cost to goal from this node
        self.parent = parent

    @property
    def f(self):
        """Total cost to goal via this node"""
        return self.g + self.h

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __le__(self, other):
        return self.f <= other.f

    def __ge__(self, other):
        return self.f >= other.f

    def __lt__(self, other):
        return self.f < other.f

    def __gt__(self, other):
        return self.f > other.f

    def __ne__(self, other):
        return self.x != other.x or self.y != other.y

    def __repr__(self) -> str:
        return f"(Node -> {self.x}, {self.y}), Cost : {self.f}\n"


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
        self.visited = set()  # (x, y)
        self.planning_time = 0
        self.max_time = max_time
        self.num_nodes_expanded = 0
        self.resolution = resolution
        self.distance_metric = distance_metric
        self.diagonal_motion = diagonal_motion
        self.inflation_radius = inflation_radius
        self.candidates = PriorityQueue()  # Node

        self._set_actions()
        self._set_heuristic()
        self._set_occupancy_grid()

    def _set_occupancy_grid(self):
        """Generates occupancy grid of specified configuration"""
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
        self.actions = {
            "left": {"cost": 1, "action": (-1, 0)},
            "right": {"cost": 1, "action": (1, 0)},
            "up": {"cost": 1, "action": (0, 1)},
            "down": {"cost": 1, "action": (0, -1)},
            "left_up": {"cost": 1.414, "action": (-1, 1)},
            "left_down": {"cost": 1.414, "action": (-1, -1)},
            "right_up": {"cost": 1.414, "action": (1, 1)},
            "right_down": {"cost": 1.414, "action": (1, -1)},
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

    def _reset(self):
        """
        Resets the state of data used by the algorithm
        """
        self.candidates = PriorityQueue()
        self.visited.clear()
        self.goal = None
        self.start = None
        self.latest_path = Path()
        self.num_nodes_expanded = 0

    def _expand(self, node):
        """
        Generates and adds free neighbours to exploration candidates

        :param node: The node to be expanded from
        :type node: :class: Node
        """
        for action_name in self.selected_actions:
            cost = self.actions[action_name]["cost"]
            action = self.actions[action_name]["action"]
            _x = node.x + action[0]
            _y = node.y + action[1]
            visited = (_x, _y) in self.visited
            occupied = self.grid.is_occupied(_x, _y)
            if not visited and not occupied:
                _g = node.g + cost
                _h = self._heuristic((_x, _y), self.goal)
                new_node = Node(_x, _y, _g, _h, parent=node)
                self.candidates.put(new_node)
                self.num_nodes_expanded += 1

    def _get_best_candidate(self):
        """
        Return the candidate with best metric

        :return: The candidate with the minimum cost
        :rtype: :class:`pyrobosim.navigation.astar_grid.Node`
        """
        return self.candidates.get()

    def _mark_visited(self, node):
        """
        Adds the given node to visited nodes collection

        :param node: The node that has been visited
        :type node: :class: Node
        """
        self.visited.add((node.x, node.y))

    def _reduce_waypoints(self, poses):
        """
        Reduce the number of waypoints by removing points on same line

        :param poses: The poses for the full path generated by A*
        :type poses: [(int, int)]
        :return: The reduced list of waypoints
        :rtype: list[:class:`pyrobosim.utils.pose.Pose`]
        """
        waypoints = []
        len_poses = len(poses)
        if len_poses <= 2:
            return poses
        slope = lambda p1, p2: ((p2[1] - p1[1]), (p2[0] - p1[0]))
        curr_start = poses[0]
        curr_dx, curr_dy = slope(poses[0], poses[1])
        for i in range(2, len_poses):
            new_dx, new_dy = slope(curr_start, poses[i])
            if not math.isclose((curr_dy * new_dx), (new_dy * curr_dx)):
                x, y = self.grid.grid_to_world(curr_start[0], curr_start[1])
                waypoints.append(Pose(x=x, y=y))
                curr_dx, curr_dy = new_dx, new_dy
                curr_start = poses[i - 1]
        return waypoints

    def _generate_path(self, node):
        """
        Backtrack from a node to generate the full path till that node

        :param node: The node from which to retrace the path
        :type node: :class:`pyrobosim.navigation.astar_grid.Node`
        """
        poses = []  # Tuple (x,y)
        while node is not None:
            poses.append((node.x, node.y))
            node = node.parent
        poses.reverse()
        poses = self._reduce_waypoints(poses)
        self.latest_path = Path(poses=poses)
        self.latest_path.fill_yaws()

    def _is_valid_start_goal(self):
        """
        Validate the start and goal locations provided to the planner

        :return: True if the start and goal given to the planner are not occupied, else False
        :rtype: bool
        """
        valid = True
        if self.grid.is_occupied(self.start[0], self.start[1]):
            warnings.warn(f"Start position {self.start} is occupied")
            valid = False
        if self.grid.is_occupied(self.goal[0], self.goal[1]):
            valid = False
            warnings.warn(f"Goal position {self.goal} is occupied")
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
        self._reset()
        self.start = self.grid.world_to_grid(start.x, start.y)
        self.goal = self.grid.world_to_grid(goal.x, goal.y)

        # If start or goal position is occupied, return empty path with warning.
        if not self._is_valid_start_goal():
            return Path()

        start_node = Node(
            self.start[0], self.start[1], g=0, h=self._heuristic(self.start, self.goal)
        )
        goal_node = Node(self.goal[0], self.goal[1], 0, 0)

        timed_out = False
        path_found = False
        t_start = time.time()
        self.candidates.put(start_node)
        while not path_found and self.candidates and not timed_out:
            current = self._get_best_candidate()
            if current == goal_node:
                path_found = True
            else:
                self._expand(current)
            self._mark_visited(current)
            self.planning_time = time.time() - t_start
            timed_out = self.planning_time >= self.max_time

        if path_found:
            self._generate_path(current)
            return self.latest_path
        else:
            return Path()

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
