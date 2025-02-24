"""
Pose representation utilities.
"""

import math
import numpy as np
from typing import Any, Sequence
from typing_extensions import Self  # For compatibility with Python <= 3.10

from transforms3d.euler import euler2quat, quat2euler
from transforms3d.quaternions import mat2quat, nearly_equivalent, qnorm, quat2mat


class Pose:
    """Represents a 3D pose."""

    def __init__(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        q: list[float] | None = None,
        angle_units: str = "radians",
    ) -> None:
        """
        Creates a new Pose object.

        :param x: X position
        :param y: Y position
        :param z: Z position
        :param roll: Roll angle (about X axis), in specified angle units
        :param pitch: Pitch angle (about Y axis), in specified angle units
        :param yaw: Yaw angle (about Z axis), in specified angle units
        :param angle_units: Units for angle ('radians' or 'degrees'). Default is 'radians'
        :param q: Quaternion, specified at [qw, qx, qy, qz]
            If specified, will override roll/pitch/yaw values.
        """
        self.x = x
        self.y = y
        self.z = z

        if q is not None:
            self.set_quaternion(q)
            return

        if angle_units not in ("radians", "degrees"):
            raise ValueError(
                "Invalid angle units provided. It should be either 'radians' or 'degrees'."
                "Additionally, if 'q' is provided, angle units are ignored."
            )

        if angle_units == "degrees":
            roll, pitch, yaw = map(math.radians, [roll, pitch, yaw])

        self.set_euler_angles(roll, pitch, yaw)

    @classmethod
    def from_list(cls, plist: list[float]) -> Self:
        """
        Creates a pose from a list. The assumptions are:

        * 2-element lists: ``[x, y]``
        * 3-element lists: ``[x, y, z]``
        * 4-element lists: ``[x, y, z, yaw]``
        * 6-element lists: ``[x, y, z, roll, pitch, yaw]``
        * 7-element lists: ``[x, y, z, qw, qx, qy, qz]``
        * other lengths: invalid
        * angle units are always "radians"

        :param plist: List containing the input pose (see format above).
        :return: Pose object
        """
        num_elems = len(plist)
        plist = [float(elem) for elem in plist]
        if num_elems == 2:
            return cls(x=plist[0], y=plist[1])
        elif num_elems == 3:
            return cls(x=plist[0], y=plist[1], z=plist[2])
        elif num_elems == 4:
            return cls(
                x=plist[0],
                y=plist[1],
                z=plist[2],
                yaw=plist[3],
            )
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
    def from_transform(cls, tform: np.array) -> Self:
        """
        Creates a pose from a transformation matrix.

        :param tform: 4-by-4 transformation matrix
        :return: Pose object
        """
        return cls(
            x=tform[0, 3], y=tform[1, 3], z=tform[2, 3], q=mat2quat(tform[:3, :3])
        )

    @classmethod
    def from_dict(cls, pose_dict: dict[str, Any]) -> Self:
        """
        Creates a pose from a dictionary.

        The keys of this dictionary can be as follows:

        .. code-block:: yaml

           position:
             x: 1.0
             y: 2.0
             z: 3.0
           rotation_eul:
             yaw: 0.5
             pitch: 0.6
             roll: 0.7
             angle_units: "radians"
           rotation_quat:
             w: 0.7071
             x: 0.0
             y: -0.7071
             z: 0.0

        Note that the ``rotation_quat`` key overrides the `rotation_eul` key.

        :param pose_dict: A pose dictionary.
        :return: Pose object
        """

        pos = pose_dict.get("position", {})
        args = {
            "x": pos.get("x", 0.0),
            "y": pos.get("y", 0.0),
            "z": pos.get("z", 0.0),
        }

        quat = pose_dict.get("rotation_quat")
        eul = pose_dict.get("rotation_eul")
        if quat is not None:
            args.update(
                {
                    "q": [
                        quat.get("w", 0.0),
                        quat.get("x", 0.0),
                        quat.get("y", 0.0),
                        quat.get("z", 0.0),
                    ]
                }
            )
        elif eul is not None:
            args.update(
                {
                    "yaw": eul.get("yaw", 0.0),
                    "pitch": eul.get("pitch", 0.0),
                    "roll": eul.get("roll", 0.0),
                    "angle_units": eul.get("angle_units", "radians"),
                }
            )
        return cls(**args)

    @classmethod
    def construct(self, data: dict[str, Any]) -> Self:
        """
        Constructs a pose object from any of the allowable input types.

        :param data: The input data describing the pose.
        :return: Pose object
        raises ValueError: if the input data type is unsupported.
        """
        if isinstance(data, list):
            return self.from_list(data)
        elif isinstance(data, dict):
            return self.from_dict(data)
        elif isinstance(data, np.ndarray):
            return self.from_transform(data)
        else:
            raise ValueError(
                f"Cannot construct pose from object of type {type(data).__name__}."
            )

    def to_dict(self) -> dict[str, Any]:
        """
        Converts the pose instance to a dictionary compatible with YAML.

        :return: The output dictionary.
        """
        return {
            "position": {"x": float(self.x), "y": float(self.y), "z": float(self.z)},
            "rotation_quat": {
                "w": float(self.q[0]),
                "x": float(self.q[1]),
                "y": float(self.q[2]),
                "z": float(self.q[3]),
            },
        }

    def get_linear_distance(self, other: Self, ignore_z: bool = False) -> float:
        """
        Gets the straight-line distance between two poses.

        :param other: Pose from which to get the linear distance.
        :param ignore_z: If True, ignores the Z component of the distance.
        :return: Linear distance between this and the other pose.
        """
        sum_squares = (other.x - self.x) ** 2 + (other.y - self.y) ** 2
        if not ignore_z:
            sum_squares += (other.z - self.z) ** 2
        return math.sqrt(sum_squares)

    def get_angular_distance(self, other: Self) -> float:
        """
        Gets the angular distance between two poses, wrapped in the range [-pi, pi].

        :param other: Pose from which to get the linear distance.
        :return: Angular distance between this and the other pose.
        """
        return float(np.arctan2(other.y - self.y, other.x - self.x))

    def get_yaw(self) -> float:
        """
        Gets the yaw angle, in radians.
        This is a handy utility for 2D (or 2.5D) calculations.

        :return: Yaw angle (about Z axis), in radians
        """
        return self.eul[2]

    def set_euler_angles(
        self, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0
    ) -> None:
        """
        Sets the orientation component as Euler angles.

        :param roll: Roll angle (about X axis), in radians
        :param pitch: Pitch angle (about Y axis), in radians
        :param yaw: Yaw angle (about Z axis), in radians
        """
        self.eul = [roll, pitch, yaw]
        self.q = euler2quat(roll, pitch, yaw, "rxyz")

    def set_quaternion(self, q: list[float]) -> None:
        """
        Sets the orientation component as a quaternion.

        :param q: Quaternion, specified at [qw, qx, qy, qz]
            If specified, will override roll/pitch/yaw values.
        """
        self.q = q / qnorm(q)
        self.eul = quat2euler(self.q, "rxyz")

    def get_translation_matrix(self) -> np.ndarray:
        """
        Gets a translation matrix from the pose representation.

        :return: 4-by-4 translation matrix
        """
        trans_mat = np.eye(4)
        trans_mat[:3, 3] = [self.x, self.y, self.z]
        return trans_mat

    def get_rotation_matrix(self) -> np.ndarray:
        """
        Gets a rotation matrix from the pose representation.

        :return: 3-by-3 rotation matrix
        """
        return quat2mat(self.q)

    def get_transform_matrix(self) -> np.ndarray:
        """
        Gets a homogeneous transformation matrix from the pose representation.

        :return: 4-by-4 transformation matrix
        """
        tf_mat = self.get_translation_matrix()
        tf_mat[:3, :3] = self.get_rotation_matrix()
        return tf_mat

    def get_translation(self) -> np.ndarray:
        """
        Gets the pose x y and z of the pose as an array.

        :return: Pose x y and z as an array
        """
        return np.array([self.x, self.y, self.z])

    def is_approx(
        self, other: Self, rel_tol: float = 1e-06, abs_tol: float = 1e-06
    ) -> bool:
        """
        Check if two poses are approximately equal with a tolerance.

        :param other: Pose with which to check approximate equality.
        :param rel_tol: Relative tolerance
        :param abs_tol: Absolute tolerance
        :return: True if the Poses are approximately equal, else False
        """
        if not (isinstance(other, Pose)):
            raise TypeError("Expected a Pose object.")

        return bool(
            np.allclose(
                self.get_translation(), other.get_translation(), rel_tol, abs_tol
            )
            and nearly_equivalent(self.q, other.q, rel_tol, abs_tol)
        )

    def __eq__(self, other: object) -> bool:
        """
        Check if two poses are exactly equal.

        :param other: Pose with which to check equality.
        :return: True if the poses are equal, else False
        """
        if not (isinstance(other, Pose)):
            raise TypeError("Expected a Pose object.")

        return bool(
            np.all(self.get_translation() == other.get_translation())
            and np.all(self.q == other.q)
        )

    def __repr__(self) -> str:
        """
        Representation for printing a Pose object.

        :return: Printable string.
        """
        pos_str = f"x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}"
        quat_str = (
            f"qw={self.q[0]:.3f}, qx={self.q[1]:.3f}, "
            + f"qy={self.q[2]:.3f}, qz={self.q[3]:.3f}"
        )
        return f"Pose: [{pos_str}, {quat_str}]"


def get_angle(p1: Sequence[float], p2: Sequence[float]) -> float:
    """
    Basic utility for getting angle between 2D points.
    The convention is the angle ``p2`` with reference to ``p1``.

    :param p1: Reference point
    :param p2: Target point
    :return: Angle of target pose with respect to reference pose.
    """
    return wrap_angle(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))


def get_distance(p1: Sequence[float], p2: Sequence[float]) -> float:
    """
    Basic utility for getting distance between points.

    :param p1: Reference point
    :param p2: Target point
    :return: Distance between target pose and reference pose.
    """
    sqrs = [(i - j) ** 2 for i, j in zip(p1, p2)]
    return math.sqrt(sum(sqrs))


def get_bearing_range(p1: Sequence[float], p2: Sequence[float]) -> tuple[float, float]:
    """
    Gets bearing and range between 2 points ``p1`` and ``p2``.
    The convention is the bearing of ``p2`` with reference to ``p1``.

    :param p1: Reference point
    :param p2: Target point
    :return: Bearing and range of target pose with respect to reference pose.
    """
    rng = get_distance(p1, p2)
    bear = get_angle(p1, p2)
    return (bear, rng)


def rot2d(vec: Sequence[float], ang: float) -> Sequence[float]:
    """
    Rotates a 2-element vector by an angle.

    :param vec: Vector to rotate.
    :param ang: Rotation angle, in radians.
    :return: Rotated vector.
    """
    v = np.array([[vec[0]], [vec[1]]])
    M = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
    v_tf = np.matmul(M, v)
    return list(v_tf.flatten())


def wrap_angle(ang: float) -> float:
    """
    Wraps an angle in the range [-pi, pi].

    :param ang: Original angle.
    :return: Wrapped angle.
    """
    if (ang is None) or (ang >= -math.pi and ang <= math.pi):
        # If the angle is none or within range, return it as is.
        return ang
    else:
        # Otherwise, add pi, wrap it in the range [0..2pi], and then subtract pi.
        divided_ang = (ang + math.pi) / (2 * math.pi)
        remainder = divided_ang - math.floor(divided_ang)
        wrapped_ang = remainder * (2 * math.pi) - math.pi
        return wrapped_ang
