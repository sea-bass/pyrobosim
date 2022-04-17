"""
Pose representation utilities.
"""

import numpy as np


class Pose:
    """ Represents a 2.5D (X, Y, Z, yaw) pose. """

    def __init__(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """ Creates a new Pose object. """
        self.x = x
        self.y = y
        self.z = z
        self.yaw = wrap_angle(yaw)

    @classmethod
    def from_list(cls, plist):
        """ 
        Creates a pose from a list. The assumptions are:
        
        * 2-element lists: ``[x, y]``
        * 3-element lists: ``[x, y, yaw]``
        * 4-element lists: ``[x, y, z, yaw]``

        :param plist: List containing the input pose (see format above).
        :type plist: list[float]
        :return: Pose object
        :rtype: :class:`pyrobosim.utils.pose.Pose`
        """
        num_elems = len(plist)
        plist = [float(elem) for elem in plist]
        if num_elems == 2:
            return cls(x=plist[0], y=plist[1])
        elif num_elems == 3:
            return cls(x=plist[0], y=plist[1], yaw=plist[2])
        elif num_elems == 4:
            return cls(x=plist[0], y=plist[1], z=plist[2], yaw=plist[3])
        else:
            raise Exception("List must contain 2, 3, or 4 elements.")


    def get_linear_distance(self, other, ignore_z=False):
        """
        Gets the straight-line distance between two poses.

        :param other: Pose from which to get the linear distance.
        :type other: :class:`pyrobosim.utils.pose.Pose`
        :param ignore_z: If True, ignores the Z component of the distance.
        :type ignore_z: bool
        :return: Linear distance between this and the other pose.
        :rtype: float
        """
        sum_squares = (other.x - self.x)**2 + (other.y - self.y)**2
        if not ignore_z:
            sum_squares += (other.z - self.z)**2
        return np.sqrt(sum_squares)

    def get_angular_distance(self, other):
        """
        Gets the angular distance between two poses, wrapped in the range [-pi, pi].

        :param other: Pose from which to get the linear distance.
        :type other: :class:`pyrobosim.utils.pose.Pose`
        :return: Angular distance between this and the other pose.
        :rtype: float        
        """
        return np.arctan2(other.y - self.y, other.x - self.x)

    def __repr__(self):
        """ 
        Representation for printing a Pose object.
        
        :return: Printable string.
        :rtype: str
        """
        return f"Pose: [x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, yaw={self.yaw:.2f}]"


def get_angle(p1, p2):
    """ 
    Basic utility for getting angle between 2D points.
    The convention is the angle ``p2`` with reference to ``p1``.

    :param p1: Reference point
    :type p1: list[float]
    :param p2: Target point
    :type p2: list[float]
    :return: Angle of target pose with respect to reference pose.
    :rtype: float  
    """
    return wrap_angle(np.arctan2(p2[1]-p1[1], p2[0]-p1[0]))


def get_distance(p1, p2):
    """ 
    Basic utility for getting distance between points. 

    :param p1: Reference point
    :type p1: list[float]
    :param p2: Target point
    :type p2: list[float]
    :return: Distance between target pose and reference pose.
    :rtype: float      
    """
    sqrs = [(i-j)**2 for i, j in zip(p1, p2)]
    return np.sqrt(sum(sqrs))


def get_bearing_range(p1, p2):
    """ 
    Gets bearing and range between 2 points ``p1`` and ``p2``.
    The convention is the bearing of ``p2`` with reference to ``p1``.

    :param p1: Reference point
    :type p1: list[float]
    :param p2: Target point
    :type p2: list[float]
    :return: Bearing and range of target pose with respect to reference pose.
    :rtype: (float, float)  
    """
    rng = get_distance(p1, p2)
    bear = get_angle(p1, p2)
    return (bear, rng)


def rot2d(vec, ang):
    """ 
    Rotates a 2-element vector by an angle.

    :param vec: Vector to rotate.
    :type vec: (float, float)
    :param ang: Rotation angle, in radians.
    :type ang: float
    :return: Rotated vector.
    :type: (float, float)
    """
    v = np.array([[vec[0]],
                  [vec[1]]])
    M = np.array([[np.cos(ang), -np.sin(ang)],
                  [np.sin(ang),  np.cos(ang)]])
    v_tf = np.matmul(M, v)
    return v_tf.flatten().tolist()


def wrap_angle(ang):
    """ 
    Wraps an angle in the range [-pi, pi].
    
    :param ang: Original angle.
    :type ang: float
    :return: Wrapped angle.
    :rtype: float
    """
    if ang is None:
        return ang

    while ang < -np.pi:
        ang += 2*np.pi
    while ang > np.pi:
        ang -= 2*np.pi
    return ang
