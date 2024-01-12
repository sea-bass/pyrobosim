from pyrobosim.navigation.planner_base import PathPlannerBase
from pyrobosim.utils.motion import Path
from pyrobosim.utils.pose import Pose

from math import sqrt, inf
from queue import PriorityQueue

class Planner(PathPlannerBase):
    
    def __init__(self, grid):
        super().__init__()

        self.grid = grid

    def euclidean(self, from_node, to_tuple):

        return sqrt((to_tuple[0] - from_node.x)**2 + (to_tuple[1] - from_node.y)**2)

    def neighbors_of(self, target_node):

        neighbors = []

        directions = [
            (1,0),
            (1,1),
            (0,1),
            (-1,1),
            (-1,0),
            (-1,-1),
            (0,-1),
            (1,-1)
        ]

        for direction in directions:

            new_x = target_node.x + direction[0]
            new_y = target_node.y + direction[1]

            if (self.grid.is_in_bounds(pos=(new_x, new_y))):

                neighbors.append((new_x, new_y))

        return neighbors
    
    def plan(self, start, goal):

        # f: total cost of node
        # g: the distance between the current node and the start node
        # h: the heuristic - estimated distance from the current node to the end node
        # f = g + h

        self.unexplored = PriorityQueue()

        goal_found = False

        # convert to grid context
        start_grid = self.grid.world_to_grid((start.x, start.y))        
        goal_grid = self.grid.world_to_grid((goal.x, goal.y))

        # Put start node in pqueue/unexplored
        start_node = Node(x=start_grid[0], y=start_grid[1])
        start_node.g = 0
        start_node.set_h(self.euclidean(start_node, goal_grid))
        self.unexplored.put(start_node)

        # while (unexplored is not empty)
        while (not goal_found):

            # current_node = pqueue.pop(), move current_node to explored
            current_node = self.unexplored.get()

            # if current_node is the goal node, hooray!:
            if (current_node.make_tuple() == goal_grid):

                # TODO return path traced backwards, just tuples of coordinates
                path = reversed(current_node.retrace())

                goal_found = True
                
            # for neighbor tuple in current_node.neighbors():
            for neighbor in self.neighbors_of(current_node):

                # print(self.unexplored.queue)
                
                # neighbor_node = self.unexplored.queued_node(neighbor)
                # if neighbor already in unexplored
                unexplored_tuples = [n.make_tuple() for n in self.unexplored.queue]
                if (neighbor in unexplored_tuples):

                    # if current_node.g + 1 < neighbor.g
                    if ((current_node.g + 1) < neighbor_node.g):
                        neighbor_node.set_parent_node(current_node)

                    else:
                        pass

                elif (not self.grid.is_occupied(neighbor)):
                    neighbor_node = Node(neighbor[0], neighbor[1])
                    neighbor_node.set_parent_node(current_node)
                    neighbor_node.set_h(self.euclidean(neighbor_node, goal_grid))
                    self.unexplored.put(neighbor_node)

                # else IF neighbor is traversable:
                    # put neighbor in pqueue/unexplored

        # self.unexplored.clear()

        # convert to tuples for final path
        tuples = []

        for waypoint in path:

            # convert back to world
            tuple_x, tuple_y = self.grid.grid_to_world(waypoint.make_tuple())
            tuples.append(Pose(tuple_x, tuple_y))


        return Path(poses=tuples)

class Node(object):

    def __init__(self, x, y):

        # all coordinates are in grid space

        self.x = x
        self.y = y
        self.parent = None
        self.h = inf
        self.in_unexplored = False

    def __lt__(self, other):
        return self.f < other.f

    def set_parent_node(self, node):

        self.g = node.g + 1
        self.parent = node
        self.f = self.g + self.h

    def make_tuple(self):

        return (self.x, self.y)

    def set_h(self, h):

        self.h = h
        self.f = self.g + self.h

    def retrace(self):

        return [self] + (self.parent.retrace() if self.parent else [])
