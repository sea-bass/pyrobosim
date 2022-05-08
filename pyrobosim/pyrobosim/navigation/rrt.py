"""
Implementation of the Rapidly-exploring Random Tree (RRT) 
algorithm for motion planning.
"""

import time
import numpy as np

from .search_graph import SearchGraph, Node, Edge
from ..utils.pose import Pose

class RRTPlanner:
    def __init__(self, world):
        self.max_connection_dist = 1.0
        self.max_nodes_sampled = 1000
        self.max_time = 10

        self.world = world
        self.reset()

    def reset(self):
        self.graph = SearchGraph(world=self.world)

    def plan(self, start, goal, plot=False):
        self.reset()

        # Create the start and goal nodes
        n_start = Node(start, parent=None)
        n_goal = Node(goal, parent=None)
        self.graph.nodes = {n_start}

        t_start = time.time()
        n_sampled = 0
        goal_found = False
        while not goal_found:
            # Sample a node
            q_sample = self.sample_configuration()
            n_sampled += 1
            n_near = self.nearest_node(q_sample)
            n_new = self.new_node(n_near, q_sample)

            # Connect the new node to the parent
            if not self.graph.connect(n_near, n_new):
                continue
            self.graph.nodes.add(n_new)

            # See if the new node can directly connect to the goal
            goal_dist = n_near.pose.get_linear_distance(n_goal.pose)
            if goal_dist < self.max_connection_dist and self.graph.connect(n_near, n_goal):
                n_goal.parent = n_near
                self.graph.add(n_goal)
                goal_found = True

            # Check max nodes samples or max time elapsed
            t_elapsed = time.time() - t_start
            if t_elapsed > self.max_time or n_sampled > self.max_nodes_sampled:
                break
            
        # Now back out the path
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
        return path        
        

    def sample_configuration(self):
        """ Sample a random configuration from the world. """
        return self.world.sample_free_robot_pose_uniform()

    def nearest_node(self, pose):
        """ Get the nearest node in the graph to a specified pose. """
        if len(self.graph.nodes) == 0:
            return None
        
        # Find the nearest node
        min_dist = np.inf
        for n in self.graph.nodes:
            dist = pose.get_linear_distance(n.pose)
            if dist < min_dist:
                min_dist = dist
                n_nearest = n
        return n_nearest

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

        return Node(q_new, parent=n_near)
