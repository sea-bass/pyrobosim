"""
Helper primitives for PDDLStream
"""

import numpy as np

def get_distance(l1, l2):
    """ 
    Optimistically estimate the distance between two locations by getting the minimum 
    straight-line distance between any two navigation poses.

    :param l1: First location
    :type l1: Entity
    :param l2: Second location
    :type l2: Entity
    :return: Distance estimate
    :rtype: float
    """
    min_dist = np.inf
    for p1 in l1.nav_poses:
        for p2 in l2.nav_poses:
            dist = p1.get_linear_distance(p2)
            if dist < min_dist:
                min_dist = dist
    return min_dist
