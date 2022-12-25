""" Grasping utilities. """

from ..utils.pose import Pose

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from transforms3d.quaternions import rotate_vector, qinverse


class Grasp:
    """
    Representation of an object grasp.
    """
    def __init__(self, origin, direction, properties=None):
        self.origin = origin
        self.direction = direction
        self.properties = properties

    def add_origin(self, vec):
        """ Adds an offset vector to the origin pose. """
        return [self.origin.x + vec[0],
                self.origin.y + vec[1],
                self.origin.z + vec[2]]

    def plot(self, ax, color):
        """ Displays the grasp on an existing set of axes. """
        d = self.properties.depth
        h = self.properties.height / 2
        w = self.properties.max_width / 2

        qinv = qinverse(self.origin.q)
        left_bottom_base = self.add_origin(rotate_vector([w, -h, -d], qinv))
        left_top_base = self.add_origin(rotate_vector([w, h, -d], qinv))
        left_bottom_tip = self.add_origin(rotate_vector([w, -h, 0], qinv))
        left_top_tip = self.add_origin(rotate_vector([w, h, 0], qinv))
        right_bottom_base = self.add_origin(rotate_vector([-w, -h, -d], qinv))
        right_top_base = self.add_origin(rotate_vector([-w, h, -d], qinv))
        right_bottom_tip = self.add_origin(rotate_vector([-w, -h, 0], qinv))
        right_top_tip = self.add_origin(rotate_vector([-w, h, 0], qinv))

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
            Poly3DCollection(gripper_verts, color=color, alpha=0.8)
        )

    def __repr__(self):
        display_str = f"Grasp:\n\t{self.origin}\n\tDirection: {self.direction}\n"
        if self.properties is not None:
            display_str += "\t{self.properties}\n"
        return display_str


class ParallelGraspProperties:
    """
    Representation of a parallel-jaw gripper.

                  |            _________
        _____     v           |              ^
    ===|     |  height     ===|          max_width
        -----     ^           |_________     v
                  |
                              |<-depth->|

        SIDE                      TOP
    """
    def __init__(self, max_width, depth, height):
        self.max_width = max_width
        self.depth = depth
        self.height = height

    def __repr__(self):
        display_str = "Parallel jaw gripper properties:\n"
        display_str += f"\tMax width: {self.max_width}, depth: {self.depth}, height: {self.height}"
        return display_str


class GraspGenerator:
    """
    Generates grasps given object dimensions and pose relative to a robot.
    """
    def __init__(self, properties):
        self.properties = properties

    def generate(self, obj_dims):
        """
        TODO: Generate grasps
        """
        grasps = []

        # Front grasp
        grasp_center = Pose(x=0.0, y=0.0, z=0.0 - 0.01,
                            pitch=-np.pi/2, yaw=np.pi/2)
        grasp_direction = np.array([-1.0, 0.0, 0.0])
        grasps.append(
            Grasp(origin=grasp_center, direction=grasp_direction,
                  properties=self.properties)
        )

        # Top grasp
        grasp_center = Pose(x=0.0, y=0.0, z=0.0 + 0.05,
                            pitch=np.pi, yaw=np.pi/2)
        grasp_direction = np.array([0.0, 0.0, 1.0])
        grasps.append(
            Grasp(origin=grasp_center, direction=grasp_direction,
                  properties=self.properties)
        )

        # Side grasp
        grasp_center = Pose(x=0.0, y=0.0, z=0.0 + 0.03,
                            roll=np.pi/2)
        grasp_direction = np.array([0.0, -1.0, 0.0])
        grasps.append(
            Grasp(origin=grasp_center, direction=grasp_direction,
                  properties=self.properties)
        )

        return grasps


    def show_grasps(self, obj_dims, grasps):
        """
        Display the grasps on top of an object
        """
        fig = plt.figure()
        ax = Axes3D(fig)
        fig.add_axes(ax)
        
        # Show the object
        x, y, z = [d/2 for d in obj_dims]
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
        colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
        for grasp in grasps:
            xo = grasp.origin.x
            yo = grasp.origin.y
            zo = grasp.origin.z
            xd, yd, zd = grasp.direction
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
        dim = max(obj_dims)
        ax.axes.set_xlim3d(-dim, dim)
        ax.axes.set_ylim3d(-dim, dim)
        ax.axes.set_zlim3d(-dim, dim)
        plt.show()
