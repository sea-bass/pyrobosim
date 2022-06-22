"""
Helper primitives for PDDLStream based planning.
"""

import numpy as np

def get_pick_place_cost(l, o):
    """
    Estimates a dummy pick / place cost for a specific location / object combination,
    which a constant value plus the height of the location and half height of the object.
    
    :param l: Location where pick / place action occurs.
    :type l: Location
    :param o: Object that is manipulated.
    :type o: Object
    """
    return 0.5 + l.height + (0.5 * o.height)


def get_straight_line_distance(l1, l2):
    """ 
    Optimistically estimate the distance between two locations by getting the minimum 
    straight-line distance between any two navigation poses.

    :param l1: First location.
    :type l1: Location
    :param l2: Second location.
    :type l2: Location
    :return: Straight-line distance between locations.
    :rtype: float
    """
    min_dist = np.inf
    for p1 in l1.nav_poses:
        for p2 in l2.nav_poses:
            dist = p1.get_linear_distance(p2)
            if dist < min_dist:
                min_dist = dist
    return min_dist


def get_nav_poses(l):
    """
    Gets a finite list of navigation poses for a specific location.

    :param l: Location from which get navigation poses.
    :type l: Location
    :return: List of tuples containing navigation poses.
    :rtype: list[tuple]
    """
    return [(p,) for p in l.nav_poses]


def get_path_length(path):
    """ 
    Simple wrapper to get the length of a path.

    :param path: Path from start to goal.
    :type path: :class:`pyrobosim.utils.motion.Path`
    :return: Length of the path.
    :rtype: float
    """
    return path.length


def sample_motion(planner, p1, p2):
    """
    Samples a feasible motion plan from a start to a goal pose.

    :param planner: Motion planner object.
    :type planner: Planner
    :param start: Start pose.
    :type start: :class:`pyrobosim.utils.pose.Pose`
    :param goal: Goal pose.
    :type goal: :class:`pyrobosim.utils.pose.Pose`
    :return: Generator yielding tuple containing a path from start to goal
    :rtype: generator[tuple[:class:`pyrobosim.utils.motion.Path`]]
    """
    while True:
        path = planner.plan(p1, p2)
        if path.num_poses == 0:
            break
        yield (path,)
