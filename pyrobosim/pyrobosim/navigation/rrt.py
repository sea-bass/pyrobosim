""" Rapidly-exploring Random Tree (RRT) implementation. """

import copy
import time
import numpy as np

from .search_graph import SearchGraph, Node, Edge
from ..utils.motion import Path
from ..utils.pose import Pose


class RRTPlanner:
    """
    Implementation of the Rapidly-exploring Random Tree (RRT)
    algorithm for motion planning.
    """

    def __init__(
        self,
        world,
        bidirectional=False,
        rrt_connect=False,
        rrt_star=False,
        max_connection_dist=0.25,
        max_nodes_sampled=1000,
        max_time=5.0,
        rewire_radius=1.0,
    ):
        """
        Creates an instance of an RRT planner.

        :param world: World object to use in the planner.
        :type world: :class:`pyrobosim.core.world.World`
        :param bidirectional: If True, uses bidirectional RRT to grow trees
            from both start and goal.
        :type bidirectional: bool
        :param rrt_connect: If True, uses RRT-connect to bias tree growth
            towards goals.
        :type rrt_connect: bool
        :param rrt_star: If True, uses RRT* to rewire trees to smooth and
            shorten paths.
        :type rrt_star: bool
        :param max_connection_dist: Maximum connection distance between nodes.
        :type max_connection_dist: float
        :param max_nodes_sampled: Maximum nodes sampled before planning stops.
        :type max_nodes_sampled: int
        :param max_time: Maximum wall clock time before planning stops.
        :type max_time: float
        :param rewire_radius: Radius around a node to rewire the RRT,
            if using the RRT* algorithm.
        :param rewire_radius: float
        """
        # Algorithm options
        self.bidirectional = bidirectional
        self.rrt_connect = rrt_connect
        self.rrt_star = rrt_star

        # Parameters
        self.max_connection_dist = max_connection_dist
        self.max_nodes_sampled = max_nodes_sampled
        self.max_time = max_time
        self.rewire_radius = rewire_radius

        # Visualization
        self.color_start = [0, 0, 0]
        self.color_goal = [0, 0.4, 0.8]
        self.color_alpha = 0.5

        self.world = world
        self.reset()

    def reset(self):
        """Resets the search trees and planning metrics."""
        self.graph = SearchGraph(world=self.world)
        self.graph_goal = SearchGraph(world=self.world)
        self.latest_path = Path()
        self.planning_time = 0.0
        self.nodes_sampled = 0
        self.n_rewires = 0

    def plan(self, start, goal):
        """
        Plans a path from start to goal.

        :param start: Start pose.
        :type start: :class:`pyrobosim.utils.pose.Pose`
        :param goal: Goal pose.
        :type goal: :class:`pyrobosim.utils.pose.Pose`
        :return: Path from start to goal.
        :rtype: :class:`pyrobosim.utils.motion.Path`
        """
        self.reset()

        # Create the start and goal nodes
        n_start = Node(start, parent=None)
        n_goal = Node(goal, parent=None)
        self.graph.nodes = {n_start}
        if self.bidirectional:
            self.graph_goal.nodes = {n_goal}

        t_start = time.time()
        goal_found = False
        while not goal_found:
            # Sample a node
            q_sample = self.sample_configuration()
            self.nodes_sampled += 1

            # Connect a new node to the parent
            n_near = self.graph.nearest_node(q_sample)
            n_new = self.new_node(n_near, q_sample)
            connected_node = self.graph.connect(n_near, n_new)
            if connected_node:
                self.graph.nodes.add(n_new)

            # If bidirectional,
            # also connect a new node to the parent of the goal graph.
            if self.bidirectional:
                n_near_goal = self.graph_goal.nearest_node(q_sample)
                n_new_goal = self.new_node(n_near_goal, q_sample)
                connected_node_goal = self.graph_goal.connect(n_near_goal, n_new_goal)
                if connected_node_goal:
                    self.graph_goal.nodes.add(n_new_goal)
            else:
                connected_node_goal = False

            # Optional rewire, if RRT* is enabled
            if self.rrt_star:
                if connected_node:
                    self.rewire_node(self.graph, n_new)
                if connected_node_goal:
                    self.rewire_node(self.graph_goal, n_new_goal)

            # See if the new nodes can directly connect to the goal.
            # This is done either as a single connection within max distance,
            # or using RRT-Connect.
            if self.bidirectional:
                if connected_node:
                    # If we added a node to the start tree,
                    # try connect to the goal tree.
                    n_goal_goal_tree = self.graph_goal.nearest_node(n_new.pose)
                    goal_found, n_goal_start_tree = self.try_connect_until(
                        self.graph, n_new, n_goal_goal_tree
                    )

                if connected_node_goal and not goal_found:
                    # If we added a node to the goal tree,
                    # try connect to the start tree.
                    n_goal_start_tree = self.graph.nearest_node(n_new_goal.pose)
                    goal_found, n_goal_goal_tree = self.try_connect_until(
                        self.graph_goal, n_new_goal, n_goal_start_tree
                    )

            elif connected_node:
                goal_found, _ = self.try_connect_until(self.graph, n_new, n_goal)

            # Check max nodes samples or max time elapsed
            self.planning_time = time.time() - t_start
            if (
                self.planning_time > self.max_time
                or self.nodes_sampled > self.max_nodes_sampled
            ):
                self.latest_path = Path()
                return

        # Now back out the path
        if self.bidirectional:
            n = n_goal_start_tree
        else:
            n = n_goal
        path_poses = [n.pose]
        path_built = False
        while not path_built:
            if n.parent is None:
                path_built = True
            else:
                n = n.parent
                path_poses.append(n.pose)
        path_poses.reverse()
        if self.bidirectional:
            n = n_goal_goal_tree
            path_built = False
            while not path_built:
                if n.parent is None:
                    path_built = True
                else:
                    n = n.parent
                    path_poses.append(n.pose)

        self.latest_path = Path(poses=path_poses)
        self.latest_path.fill_yaws()
        return self.latest_path

    def sample_configuration(self):
        """
        Samples a random configuration from the world.

        :return: Collision-free pose if found, else ``None``.
        :rtype: :class:`pyrobosim.utils.pose.Pose`
        """
        return self.world.sample_free_robot_pose_uniform()

    def new_node(self, n_start, q_target):
        """
        Grows the RRT from a specific node towards a sampled pose in the world.
        The maximum distance to grow the tree is dictated by the
        ``max_connection_dist`` parameter.
        If the target pose is nearer than this distance, a new node is created
        at exactly that pose.

        :param n_start: Tree node from which to grow the new node.
        :type n_start: :class:`pyrobosim.navigation.search_graph.Node`
        :param q_target: Target pose towards which to grow the new node.
        :type q_target: :class:`pyrobosim.utils.pose.Pose`
        :return: A new node grown from the start node towards the target pose.
        :rtype: :class:`pyrobosim.navigation.search_graph.Node`
        """
        q_start = n_start.pose
        dist = q_start.get_linear_distance(q_target)

        step_dist = self.max_connection_dist
        if dist <= step_dist:
            q_new = q_target
        else:
            theta = q_start.get_angular_distance(q_target)
            q_new = Pose(
                x=q_start.x + step_dist * np.cos(theta),
                y=q_start.y + step_dist * np.sin(theta),
            )

        return Node(q_new, parent=n_start, cost=n_start.cost + dist)

    def rewire_node(self, graph, n_tgt):
        """
        Rewires a node in the RRT by checking if switching the parent node to
        another nearby node will reduce its total cost from the root node.

        This is the key modification in the RRT* algorithm which requires more
        computation, but produces paths that are shorter and smoother than
        plain RRT. The vicinity around the node is defined by the
        ``rewire_radius`` parameter.

        :param graph: The tree to rewire.
        :type graph: :class:`pyrobosim.navigation.search_graph.SearchGraph`
        :param n_tgt: The target tree node to rewire within the tree.
        :type n_tgt: :class:`pyrobosim.navigation.search_graph.Node`
        """
        # First, find the node to rewire, if any
        n_rewire = None
        for n in graph.nodes:
            dist = n.pose.get_linear_distance(n_tgt.pose)
            if (n != n_tgt) and (dist <= self.rewire_radius):
                alt_cost = n.cost + dist
                if (alt_cost < n_tgt.cost) and graph.check_connectivity(
                    n, n_tgt, ignore_max_dist=True
                ):
                    n_rewire = n
                    n_tgt.cost = alt_cost

        # If we found a rewire node, do the rewire
        if n_rewire is not None:
            n_tgt.parent = n_rewire
            for e in graph.edges:
                if e.n0 == n_tgt or e.n1 == n_tgt:
                    e.n0.neighbors.remove(e.n1)
                    e.n1.neighbors.remove(e.n0)
                    graph.edges.remove(e)
                    break
            graph.edges.add(Edge(n_tgt, n_tgt.parent))
            self.n_rewires += 1

    def try_connect_until(self, graph, n_curr, n_tgt):
        """
        Try to connect a node ``n_curr`` to a target node ``n_tgt``.
        This will keep extending the current node towards the target if
        RRT-Connect is enabled, or else will just try once.

        :param graph: The tree object.
        :type graph: :class:`pyrobosim.navigation.search_graph.SearchGraph`
        :param n_curr: The current tree node to try connect to the target node.
        :type n_curr: :class:`pyrobosim.navigation.search_graph.Node`
        :param n_tgt: The target tree node defining the connection goal.
        :type n_tgt: :class:`pyrobosim.navigation.search_graph.Node`
        :return: A tuple containing connection success and final node added.
        :rtype: (bool, :class:`pyrobosim.navigation.search_graph.Node`)
        """
        # Needed for bidirectional RRT so the connection node is in both trees.
        if self.bidirectional:
            n_tgt = copy.deepcopy(n_tgt)

        while True:
            dist = n_curr.pose.get_linear_distance(n_tgt.pose)

            # First, try directly connecting to the goal
            if dist < self.max_connection_dist and self.graph.connect(n_curr, n_tgt):
                n_tgt.parent = n_curr
                graph.nodes.add(n_tgt)
                if self.rrt_star:
                    self.rewire_node(graph, n_tgt)
                return True, n_tgt

            if self.rrt_connect:
                # If using RRT-Connect, keep trying to connect.
                n_new = self.new_node(n_curr, n_tgt.pose)
                if graph.connect(n_curr, n_new):
                    graph.nodes.add(n_new)
                    n_curr = n_new
                else:
                    return False, n_curr
            else:
                # If not using RRT-Connect, we only get one chance to connect.
                return False, n_curr

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
        print(f"Nodes sampled: {self.nodes_sampled}")
        print(f"Time to plan: {self.planning_time} seconds")
        print(f"Number of rewires: {self.n_rewires}")

    def plot(self, axes, path_color="m", show_graph=True, show_path=True):
        """
        Plots the RRTs and the planned path on a specified set of axes.

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
        if show_graph:
            for e in self.graph.edges:
                x = (e.n0.pose.x, e.n1.pose.x)
                y = (e.n0.pose.y, e.n1.pose.y)
                (edge,) = axes.plot(
                    x,
                    y,
                    linestyle=":",
                    linewidth=1,
                    color=self.color_start,
                    alpha=self.color_alpha,
                )
                artists.append(edge)
            if self.bidirectional:
                for e in self.graph_goal.edges:
                    x = (e.n0.pose.x, e.n1.pose.x)
                    y = (e.n0.pose.y, e.n1.pose.y)
                    (edge,) = axes.plot(
                        x,
                        y,
                        linestyle="--",
                        linewidth=1,
                        color=self.color_goal,
                        alpha=self.color_alpha,
                    )
                    artists.append(edge)

        if show_path and self.latest_path.num_poses > 0:
            x = [p.x for p in self.latest_path.poses]
            y = [p.y for p in self.latest_path.poses]
            (path,) = axes.plot(x, y, linestyle="-", color=path_color,
                                linewidth=3, alpha=0.5, zorder=1)
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
        plt.title("RRT")
        plt.axis("equal")
        plt.show()
