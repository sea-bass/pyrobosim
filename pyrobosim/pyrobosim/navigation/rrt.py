"""
Implementation of the Rapidly-exploring Random Tree (RRT) 
algorithm for motion planning.
"""

import time
import numpy as np

from .search_graph import SearchGraph, Node, Edge
from ..utils.pose import Pose

class RRTPlanner:
    def __init__(self, world, bidirectional=False, rrt_star=False):
        self.max_connection_dist = 0.5
        self.max_nodes_sampled = 1000
        self.max_time = 10

        # Algorithm options
        self.bidirectional = bidirectional
        self.rrt_star = rrt_star
        self.rewire_radius = 1.0

        self.world = world
        self.reset()

    def reset(self):
        """ Resets the search trees """
        self.graph = SearchGraph(world=self.world)
        self.graph_goal = SearchGraph(world=self.world)

    def plan(self, start, goal):
        self.reset()

        # Create the start and goal nodes
        n_start = Node(start, parent=None)
        n_goal = Node(goal, parent=None)
        self.graph.nodes = {n_start}
        if self.bidirectional:
            self.graph_goal.nodes = {n_goal}

        t_start = time.time()
        n_sampled = 0
        goal_found = False
        while not goal_found:
            # Sample a node
            q_sample = self.sample_configuration()
            n_sampled += 1

            # Connect aa new node to the parent
            n_near = self.graph.nearest_node(q_sample)
            n_new = self.new_node(n_near, q_sample)
            connected_node = self.graph.connect(n_near, n_new)
            if connected_node:
                self.graph.nodes.add(n_new)

            # If bidirectional, also connect a new node to the parent of the goal graph
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

            # See if the new nodes can directly connect to the goal
            n_connect_start = None
            n_connect_goal = None
            if self.bidirectional:
                min_dist = self.max_connection_dist
                if connected_node:
                    # If we added a node to the start tree, check it against all the goal tree nodes
                    for n_g in self.graph_goal.nodes:
                        dist = n_new.pose.get_linear_distance(n_g.pose)
                        if dist < min_dist and self.graph.check_connectivity(n_new, n_g):
                            min_dist = dist
                            n_connect_start = n_new
                            n_connect_goal = n_g
                            goal_found = True

                elif connected_node_goal:
                    # If we added a node to the goal tree, check it against all the start tree nodes
                    for n_s in self.graph.nodes:
                        dist = n_new_goal.pose.get_linear_distance(n_s.pose)
                        if dist < min_dist and self.graph.check_connectivity(n_new_goal, n_s):
                            min_dist = dist
                            n_connect_start = n_s
                            n_connect_goal = n_new_goal
                            goal_found = True

            elif connected_node:
                # If not bidirectional, check if the nearest can be added to the goal
                dist = n_new.pose.get_linear_distance(n_goal.pose)
                if dist < self.max_connection_dist and self.graph.connect(n_new, n_goal):
                    n_goal.parent = n_new
                    self.graph.add(n_goal)
                    goal_found = True

            # Check max nodes samples or max time elapsed
            t_elapsed = time.time() - t_start
            if t_elapsed > self.max_time or n_sampled > self.max_nodes_sampled:
                break
            
        # Now back out the path
        if self.bidirectional:
            n = n_connect_start
        else:
            n = n_goal
        path = [n]
        path_built = False
        while not path_built:
            if n.parent is None:
                path_built = True
            else:
                n = n.parent
                path.append(n)
        path.reverse()
        if self.bidirectional:
            n = n_connect_goal
            path_built = False
            while not path_built:
                if n.parent is None:
                    path_built = True
                else:
                    n = n.parent
                    path.append(n)
        return path         

    def sample_configuration(self):
        """ Sample a random configuration from the world. """
        return self.world.sample_free_robot_pose_uniform()

    def new_node(self, n_near, q_sample):
        """ Creates a new node based on the nearest """
        q_start = n_near.pose
        dist = q_start.get_linear_distance(q_sample)

        step_dist = self.max_connection_dist
        if dist <= step_dist:
            q_new = q_sample
        else:
            theta = q_start.get_angular_distance(q_sample)
            q_new = Pose(x=q_start.x + step_dist * np.cos(theta),
                         y=q_start.y + step_dist * np.sin(theta))

        return Node(q_new, parent=n_near, cost=n_near.cost + dist)

    def rewire_node(self, graph, n_tgt):
        """ 
        Rewires a node by checking vicinity. 
        This is the key modification for RRT*.
        """
        did_rewire = False
        for n in graph.nodes:
            dist = n.pose.get_linear_distance(n_tgt.pose)
            if (n != n_tgt) and (dist <= self.rewire_radius):
                alt_cost = n.cost + dist
                should_rewire = (alt_cost < n_tgt.cost) and \
                    graph.check_connectivity(n, n_tgt)
                if should_rewire:
                    n_tgt.cost = alt_cost
                    n_tgt.parent = n
                    did_rewire = True

        # Uncomment to show the rewired tree
        # if did_rewire:
        #     for e in graph.edges.copy():
        #         if e.n0 == n_tgt or e.n1 == n_tgt:
        #             graph.edges.remove(e)
        #     graph.edges.add(Edge(n_tgt, n_tgt.parent))
                    