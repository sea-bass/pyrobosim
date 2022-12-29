""" Grasping utilities. """

from ..utils.pose import Pose

from enum import Enum
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from transforms3d.quaternions import rotate_vector, qinverse


class GraspDirection(Enum):
    """ Enumerates grasp direction types. """
    UNKNOWN = 0
    FRONT = 1
    BACK = 2
    TOP = 3
    BOTTOM = 4
    LEFT = 5
    RIGHT = 6

normal_from_direction = {
    GraspDirection.UNKNOWN: None,
    GraspDirection.FRONT:  [-1.0, 0.0, 0.0],
    GraspDirection.BACK:   [1.0, 0.0, 0.0],
    GraspDirection.TOP:    [0.0, 0.0, 1.0],
    GraspDirection.BOTTOM: [0.0, 0.0, -1.0],
    GraspDirection.LEFT:   [0.0, 1.0, 0.0],
    GraspDirection.RIGHT:  [0.0, -1.0, 0.0],
}


class Grasp:
    """
    Representation of an object grasp.
    """
    def __init__(self, origin, properties, direction=GraspDirection.UNKNOWN):
        """
        Creates a grasp object instance.
        
        :param origin: Grasp origin pose
        :type origin: :class:`pyrobosim.utils.pose.Pose`
        :param properties: Grasping properties object
        :type properties: :class:`pyrobosim.manipulation.grasping.ParallelGraspProperties`
        :param direction: Enumeration denoting grasp direction relative to object.
        :type direction: :class:`pyrobosim.manipulation.grasping.GraspDirection`, optional
        """
        self.origin = origin
        self.direction = direction
        self.properties = properties

    def translate_origin(self, vec):
        """ 
        Adds the origin position to a specified position vector.
        
        :param vec: Original position vector
        :type vec: list[float]
        :return: Translated position vector
        :rype: list[float]
        """
        return [self.origin.x + vec[0],
                self.origin.y + vec[1],
                self.origin.z + vec[2]]

    def plot(self, ax, color, alpha=0.8):
        """ Displays the grasp on an existing set of axes. """
        d = self.properties.depth
        h = self.properties.height / 2
        w = self.properties.max_width / 2

        qinv = qinverse(self.origin.q)
        left_bottom_base = self.translate_origin(rotate_vector([w, -h, -d], qinv))
        left_top_base = self.translate_origin(rotate_vector([w, h, -d], qinv))
        left_bottom_tip = self.translate_origin(rotate_vector([w, -h, 0], qinv))
        left_top_tip = self.translate_origin(rotate_vector([w, h, 0], qinv))
        right_bottom_base = self.translate_origin(rotate_vector([-w, -h, -d], qinv))
        right_top_base = self.translate_origin(rotate_vector([-w, h, -d], qinv))
        right_bottom_tip = self.translate_origin(rotate_vector([-w, -h, 0], qinv))
        right_top_tip = self.translate_origin(rotate_vector([-w, h, 0], qinv))

        gripper_verts = [
            # Right gripper
            [right_bottom_base, right_bottom_tip,
                right_top_tip, right_top_base],
            # Left gripper
            [left_bottom_base, left_bottom_tip,
                left_top_tip, left_top_base],
            # Gripper base
            [right_bottom_base, left_bottom_base,
                left_top_base, right_top_base]
        ]
        ax.add_collection3d(
            Poly3DCollection(gripper_verts, color=color, alpha=alpha)
        )

    def __repr__(self):
        """ Printable string representation """
        display_str = f"Grasp:\n\t{self.origin}\n\tDirection: {self.direction}\n"
        if self.properties is not None:
            display_str += "\t{self.properties}\n"
        return display_str


class ParallelGraspProperties:
    """
    Representation of a parallel-jaw gripper.

        SIDE VIEW               TOP VIEW
                  |            _________
        _____     v           |              ^
    ===|     |  height     ===|          max_width
        -----     ^           |_________     v
                  |
                              |<-depth->|
    """
    def __init__(self, max_width, depth, height):
        """
        Creates a parallel gripper grasp properties instance.

        :param max_width: Maximum gripper opening width 
        :type max_width: float
        :param depth: Distance from end effector base to finger tips (or equivalent)
        :type depth: float
        :param height: Height of end effector finger tips (or equivalent)
        :type height: float
        """
        self.max_width = max_width
        self.depth = depth
        self.height = height

    def __repr__(self):
        """ Printable string representation """
        display_str = "Parallel jaw gripper properties:\n"
        display_str += f"\tMax width: {self.max_width}, depth: {self.depth}, height: {self.height}"
        return display_str


class GraspGenerator:
    """
    Generates grasps given object dimensions and pose relative to a robot.
    """
    def __init__(self, properties):
        """
        Creates a grasp generator instance given grasping properties.

        :param properties: Grasping properties object
        :type properties: :class:`pyrobosim.manipulation.grasping.ParallelGraspProperties`
        """
        self.properties = properties

    def generate(self, object_dims, p_robot_rt_object=None):
        """
        Generates a set of axis-aligned grasps for a cuboid object.

        :param object_dims: List containing the object [x, y, z] dimensions
        :type object_dims: list[float]
        :param p_robot_rt_object: The pose of the robot with respect to the object center
        :type p_robot_rt_object: :class:`pyrobosim.utils.pose.Pose`, optional
        :return: A list of generated grasps
        :rtype: list[:class:`pyrobosim.manipulation.grasping.Grasp`]      
        """
        grasps = []

        # Front grasp
        grasp_center = Pose(x=0.0, y=0.0, z=0.0 - 0.01,
                            pitch=-np.pi/2, yaw=np.pi/2)
        grasps.append(
            Grasp(origin=grasp_center, properties=self.properties,
                  direction=GraspDirection.FRONT)
        )

        # Top grasp
        grasp_center = Pose(x=0.0, y=0.0, z=0.0 + 0.05,
                            pitch=np.pi, yaw=np.pi/2)
        grasps.append(
            Grasp(origin=grasp_center, properties=self.properties,
                  direction=GraspDirection.TOP)
        )

        # Side grasp
        grasp_center = Pose(x=0.0, y=0.0, z=0.0 + 0.03,
                            roll=np.pi/2)
        grasps.append(
            Grasp(origin=grasp_center, properties=self.properties,
            direction=GraspDirection.RIGHT)
        )

        return grasps


    def show_grasps(self, object_dims, grasps):
        """
        Display the grasps on top of an object.

        :param object_dims: List containing the object [x, y, z] dimensions
        :type object_dims: list[float]
        :param grasps: A list of grasps
        :type grasps: list[:class:`pyrobosim.manipulation.grasping.Grasp`]
        """
        fig = plt.figure()
        ax = Axes3D(fig)
        fig.add_axes(ax)
        
        # Show the object
        x, y, z = [d/2 for d in object_dims]
        verts = [
            [(-x,-y,-z), ( x,-y,-z), ( x, y,-z), (-x, y,-z)],
            [(-x,-y, z), ( x,-y, z), ( x, y, z), (-x, y, z)],
            [(-x,-y,-z), (-x, y,-z), (-x, y, z), (-x,-y, z)],
            [( x,-y,-z), ( x, y,-z), ( x, y, z), ( x,-y, z)],
            [(-x,-y,-z), ( x,-y,-z), ( x,-y, z), (-x,-y, z)],
            [(-x, y,-z), ( x, y,-z), ( x, y, z), (-x, y, z)],
        ]
        ax.add_collection3d(Poly3DCollection(verts, color=[0.3, 0.3, 0.3, 0.3]))

        # Show the grasps
        color_idx = 0
        colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
        for grasp in grasps:
            xo = grasp.origin.x
            yo = grasp.origin.y
            zo = grasp.origin.z
            xd, yd, zd = normal_from_direction[grasp.direction]
            depth = grasp.properties.depth

            # Plot the grasp point
            color = colors[color_idx]
            ax.plot3D(xo, yo, zo, "o", color=color)
            ax.plot3D([xo, xo + depth * xd],
                      [yo, yo + depth * yd],
                      [zo, zo + depth * zd],
                      ":", color=color)

            # Plot the grasp itself
            grasp.plot(ax, color)
            
            color_idx += 1

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        max_dim = max(object_dims)
        ax.axes.set_xlim3d(-max_dim, max_dim)
        ax.axes.set_ylim3d(-max_dim, max_dim)
        ax.axes.set_zlim3d(-max_dim, max_dim)
        plt.show()
