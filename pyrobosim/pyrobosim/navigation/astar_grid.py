#!/usr/bin/env python3
import time
from queue import PriorityQueue
from pyrobosim.utils.pose import Pose
from pyrobosim.utils.motion import Path
from pyrobosim.navigation.occupancy_grid import occupancy_grid_from_world


class Node:
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
    """Occupancy grid based A-star planner"""

    def __init__(self, world, resolution=0.05, inflation_radius=0.0) -> None:
        self.grid = occupancy_grid_from_world(
            world, resolution=resolution, inflation_radius=inflation_radius
        )
        self.world = world
        self.resolution = resolution
        self.inflation_radius = inflation_radius
        self.candidates = PriorityQueue()  # Node
        self.visited = set()  # (x, y)
        self.start = None
        self.goal = None

    def _reset(self):
        """Resets the state of data used by the algorithm"""
        self.grid = occupancy_grid_from_world(
            self.world,
            resolution=self.resolution,
            inflation_radius=self.inflation_radius,
        )
        self.candidates = PriorityQueue()
        self.visited.clear()
        self.goal = None
        self.start = None
        self.latest_path = Path()

    def _heuristic(self, p1, p2):
        """
        Computes a heuristc between 2 points
        :param p1: Source point
        :type p1: Tuple (x, y)
        :param p2: destination point
        :type p2: Tuple (x, y)
        """
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    def _expand(self, node):
        """Generates and adds free neighbours to exploration candidates"""
        motions = [(-1, 0), (1, 0), (0, 1), (0, -1), (-1, 1), (-1, -1), (1, 1), (1, -1)]
        motion_costs = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]
        for i in range(len(motions)):
            _x = node.x + motions[i][0]
            _y = node.y + motions[i][1]
            if (_x, _y) not in self.visited and not self.grid.is_occupied(_x, _y):
                _g = node.g + motion_costs[i]
                _h = self._heuristic((_x, _y), self.goal)
                self.candidates.put(Node(_x, _y, _g, _h, parent=node))

    def _get_best_candidate(self):
        """Return the candidate with best metric"""
        return self.candidates.get()

    def _mark_visited(self, node):
        """
        Adds the given node to visited nodes collection
        :param node: The node that has been visited
        :type node: :class: Node
        """
        self.visited.add((node.x, node.y))

    def _generate_path(self, node):
        """
        Backtrack from a node to generate the full path till that node
        """
        poses = []
        while node is not None:
            x, y = self.grid.grid_to_world(node.x, node.y)
            poses.append(Pose(x=x, y=y))
            node = node.parent
        poses.reverse()
        self.latest_path = Path(poses=poses)
        self.latest_path.fill_yaws()

    def plan(self, start, goal):
        """Plans a path from start to goal"""
        # TODO : add time based termination
        self._reset()
        start = self.grid.world_to_grid(start.x, start.y)
        goal = self.grid.world_to_grid(goal.x, goal.y)
        self.start, self.goal = start, goal
        start_node = Node(start[0], start[1], g=0, h=self._heuristic(start, goal))
        goal_node = Node(goal[0], goal[1], 0, 0)

        path_found = False
        self.candidates.put(start_node)

        t_start = time.time()
        while not path_found and self.candidates:
            current = self._get_best_candidate()
            if current == goal_node:
                path_found = True
            self._expand(current)
            self._mark_visited(current)
        self.planning_time = time.time() - t_start

        if path_found:
            self._generate_path(current)
            return self.latest_path
        else:
            return Path()

    def print_metrics(self):
        """
        Print metrics about the latest path computed.
        """
        if self.latest_path.num_poses == 0:
            print("No path.")
        else:
            print("Latest path from RRT:")
            self.latest_path.print_details()
        print("")
        print(f"Time to plan: {self.planning_time} seconds")

    def plot(self, axes, path_color="m", show_path=True):
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
