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
