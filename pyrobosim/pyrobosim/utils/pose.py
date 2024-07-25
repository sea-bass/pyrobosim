"""
Pose representation utilities.
"""

import numpy as np
from transforms3d.euler import euler2quat, quat2euler
from transforms3d.quaternions import mat2quat, nearly_equivalent, qnorm, quat2mat


class Pose:
    """Represents a 3D pose."""

    def __init__(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, q=None):
        """
        Creates a new Pose object.

        :param x: X position
        :type x: float
        :param y: Y position
        :type y: float
        :param z: Z position
        :type z: float
        :param roll: Roll angle (about X axis), in radians
        :type roll: float
        :param pitch: Pitch angle (about Y axis), in radians
        :type pitch: float
        :param yaw: Yaw angle (about Z axis), in radians
        :type yaw: float
        :param q: Quaternion, specified at [qw, qx, qy, qz]
            If specified, will override roll/pitch/yaw values.
        :type q: list[float]
        """
        self.x = x
        self.y = y
        self.z = z

        if q is not None:
            self.set_quaternion(q)
        else:
            self.set_euler_angles(roll, pitch, yaw)

    @classmethod
    def from_list(cls, plist):
        """
        Creates a pose from a list. The assumptions are:

        * 2-element lists: ``[x, y]``
        * 3-element lists: ``[x, y, z]``
        * 4-element lists: ``[x, y, z, yaw]``
        * 6-element lists: ``[x, y, z, roll, pitch, yaw]``
        * 7-element lists: ``[x, y, z, qw, qx, qy, qz]``
        * other lengths: invalid

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
            return cls(x=plist[0], y=plist[1], z=plist[2])
        elif num_elems == 4:
            return cls(x=plist[0], y=plist[1], z=plist[2], yaw=plist[3])
        elif num_elems == 6:
            return cls(
                x=plist[0],
                y=plist[1],
                z=plist[2],
                roll=plist[3],
                pitch=plist[4],
                yaw=plist[5],
            )
        elif num_elems == 7:
            return cls(x=plist[0], y=plist[1], z=plist[2], q=plist[3:])
        else:
            raise ValueError("List must contain 2, 3, 4, 6, or 7 elements.")

    @classmethod
    def from_transform(cls, tform):
        """
        Creates a pose from a transformation matrix.

        :param tform: 4-by-4 transformation matrix
        :type tform: :class:`numpy.ndarray`
        :return: Pose object
        :rtype: :class:`pyrobosim.utils.pose.Pose`
        """
        return cls(
            x=tform[0, 3], y=tform[1, 3], z=tform[2, 3], q=mat2quat(tform[:3, :3])
        )

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
        sum_squares = (other.x - self.x) ** 2 + (other.y - self.y) ** 2
        if not ignore_z:
            sum_squares += (other.z - self.z) ** 2
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

    def get_yaw(self):
        """
        Gets the yaw angle, in radians.
        This is a handy utility for 2D (or 2.5D) calculations.

        :return: Yaw angle (about Z axis), in radians
        :rtype: float
        """
        return self.eul[2]

    def set_euler_angles(self, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Sets the orientation component as Euler angles.

        :param roll: Roll angle (about X axis), in radians
        :type roll: float
        :param pitch: Pitch angle (about Y axis), in radians
        :type pitch: float
        :param yaw: Yaw angle (about Z axis), in radians
        :type yaw: float
        """
        self.eul = [roll, pitch, yaw]
        self.q = euler2quat(roll, pitch, yaw, "rxyz")

    def set_quaternion(self, q):
        """
        Sets the orientation component as a quaternion.

        :param q: Quaternion, specified at [qw, qx, qy, qz]
            If specified, will override roll/pitch/yaw values.
        :type q: list[float]
        """
        self.q = q / qnorm(q)
        self.eul = quat2euler(self.q, "rxyz")

    def get_translation_matrix(self):
        """
        Gets a translation matrix from the pose representation.

        :return: 4-by-4 translation matrix
        :rtype: :class:`numpy.ndarray`
        """
        trans_mat = np.eye(4)
        trans_mat[:3, 3] = [self.x, self.y, self.z]
        return trans_mat

    def get_rotation_matrix(self):
        """
        Gets a rotation matrix from the pose representation.

        :return: 3-by-3 rotation matrix
        :rtype: :class:`numpy.ndarray`
        """
        return quat2mat(self.q)

    def get_transform_matrix(self):
        """
        Gets a homogeneous transformation matrix from the pose representation.

        :return: 4-by-4 transformation matrix
        :rtype: :class:`numpy.ndarray`
        """
        tf_mat = self.get_translation_matrix()
        tf_mat[:3, :3] = self.get_rotation_matrix()
        return tf_mat

    def get_translation(self):
        """
        Gets the pose x y and z of the pose as an array.

        :return: Pose x y and z as an array
        :rtype: :class: numpy.ndarray
        """
        return np.array([self.x, self.y, self.z])

    def is_approx(self, other, rel_tol=1e-06, abs_tol=1e-06):
        """
        Check if two poses are approximately equal with a tolerance.

        :param other: Pose with which to check approximate equality.
        :type other: :class:`pyrobosim.utils.pose.Pose`
        :param rel_tol: Relative tolerance
        :type rel_tol: float
        :param abs_tol: Absolute tolerance
        :type abs_tol: float
        :return: True if the Poses are approximately equal, else False
        :rtype: bool
        """
        if not (isinstance(other, Pose)):
            raise TypeError("Expected a Pose object.")

        return np.allclose(
            self.get_translation(), other.get_translation(), rel_tol, abs_tol
        ) and nearly_equivalent(self.q, other.q, rel_tol, abs_tol)

    def __eq__(self, other):
        """
        Check if two poses are exactly equal.

        :param other: Pose with which to check equality.
        :type other: :class:`pyrobosim.utils.pose.Pose`
        :return: True if the poses are equal, else False
        :rtype: bool
        """
        if not (isinstance(other, Pose)):
            raise TypeError("Expected a Pose object.")

        return np.all(self.get_translation() == other.get_translation()) and np.all(
            self.q == other.q
        )

    def __repr__(self):
        """
        Representation for printing a Pose object.

        :return: Printable string.
        :rtype: str
        """
        pos_str = f"x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}"
        quat_str = (
            f"qw={self.q[0]:.3f}, qx={self.q[1]:.3f}, "
            + f"qy={self.q[2]:.3f}, qz={self.q[3]:.3f}"
        )
        return f"Pose: [{pos_str}, {quat_str}]"


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
    return wrap_angle(np.arctan2(p2[1] - p1[1], p2[0] - p1[0]))


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
    sqrs = [(i - j) ** 2 for i, j in zip(p1, p2)]
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
    v = np.array([[vec[0]], [vec[1]]])
    M = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
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
    if ang is None or (ang >= -np.pi and ang <= np.pi):
        # If the angle is none or within range, return it as is.
        return ang
    else:
        # Otherwise, add pi, wrap it in the range [0..2pi], and then subtract pi.
        divided_ang = (ang + np.pi) / (2 * np.pi)
        remainder = divided_ang - np.floor(divided_ang)
        wrapped_ang = remainder * (2 * np.pi) - np.pi
        return wrapped_ang
