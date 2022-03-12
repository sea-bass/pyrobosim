import numpy as np

"""
Pose Utilities
"""

class Pose:
    """ Represents a 2.5D (X, Y, Z, yaw) pose """

    def __init__(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = wrap_angle(yaw)

    def get_linear_distance(self, other, ignore_z=False):
        """ Gets the straight-line distance between two poses """
        sum_squares = (other.x - self.x)**2 + (other.y - self.y)**2
        if not ignore_z:
            sum_squares += (other.z - self.z)**2
        return np.sqrt(sum_squares)

    def get_angular_distance(self, other):
        """ Gets the angular distance between two poses """
        return np.arctan2(other.y - self.y, other.x - self.x)

    def __repr__(self):
        return f"Pose: [x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, yaw={self.yaw:.2f}]"


def get_angle(p1, p2):
    """ 
    Basic utility for getting angle between 2D points.
    The convention is p2 relative to p1.
    """
    return wrap_angle(np.arctan2(p2[1]-p1[1], p2[0]-p1[0]))


def get_distance(p1, p2):
    """ Basic utility for getting distance between points. """
    sqrs = [(i-j)**2 for i, j in zip(p1, p2)]
    return np.sqrt(sum(sqrs))


def get_bearing_range(p1, p2):
    """ 
    Gets bearing and range between 2 points p1 and p2.
    The convention is p2 relative to p1.
    """
    rng = get_distance(p1, p2)
    bear = get_angle(p1, p2)
    return (bear, rng)

def rot2d(vec, ang):
    """ Rotates a 2-element vector `vec` by an angle `ang` """
    v = np.array([[vec[0]],
                  [vec[1]]])
    M = np.array([[np.cos(ang), -np.sin(ang)],
                  [np.sin(ang),  np.cos(ang)]])
    v_tf = np.matmul(M, v)
    return v_tf.flatten().tolist()

def wrap_angle(ang):
    """ Wraps an angle in the range [-pi, pi]. """
    if ang is None:
        return ang

    while ang < -np.pi:
        ang += 2*np.pi
    while ang > np.pi:
        ang -= 2*np.pi
    return ang
