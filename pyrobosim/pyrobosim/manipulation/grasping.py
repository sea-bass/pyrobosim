""" Grasping utilities. """

from ..utils.pose import Pose

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class Grasp:
    """
    Representation of an object grasp.
    """
    def __init__(self, origin, direction):
        self.origin = origin
        self.direction = direction

    def __repr__(self):
        return f"Grasp:\n\t{self.origin}\n\tDirection: {self.direction}\n"


class GraspGenerator:
    """
    Generates grasps given object dimensions and pose relative to a robot.
    """
    def generate(self, obj_dims):
        """
        TODO: Generate grasps
        """
        grasps = []

        # Front grasp
        grasp_center = Pose(x=obj_dims[0]/2 - 0.3, y=obj_dims[1]/2, z=obj_dims[2]/2)
        grasp_direction = np.array([-1.0, 0.0, 0.0])
        grasps.append(
            Grasp(origin=grasp_center, direction=grasp_direction)
        )

        # Top grasp
        grasp_center = Pose(x=obj_dims[0]/2, y=obj_dims[1]/2, z=obj_dims[2]/2 + 0.1)
        grasp_direction = np.array([0.0, 0.0, 1.0])
        grasps.append(
            Grasp(origin=grasp_center, direction=grasp_direction)
        )

        # Side grasp
        grasp_center = Pose(x=obj_dims[0]/2, y=obj_dims[1]/2 - 0.2, z=obj_dims[2]/2)
        grasp_direction = np.array([0.0, -1.0, 0.0])
        grasps.append(
            Grasp(origin=grasp_center, direction=grasp_direction)
        )

        return grasps


    def show_grasps(self, obj_dims, grasps):
        """
        TODO: Display the grasps on top of an object
        """

        fig = plt.figure()
        ax = Axes3D(fig)
        fig.add_axes(ax)
        
        # Show the object
        x, y, z = obj_dims
        verts = [
            [(0,0,0), (x,0,0), (x,y,0), (0,y,0)],
            [(0,0,z), (x,0,z), (x,y,z), (0,y,z)],
            [(0,0,0), (0,y,0), (0,y,z), (0,0,z)],
            [(x,0,0), (x,y,0), (x,y,z), (x,0,z)],
            [(0,0,0), (x,0,0), (x,0,z), (0,0,z)],
            [(0,y,0), (x,y,0), (x,y,z), (0,y,z)],
        ]
        ax.add_collection3d(Poly3DCollection(verts, color=[0, 0, 1, 0.3]))

        # Show the grasps
        color_idx = 0
        colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
        for grasp in grasps:
            xo = grasp.origin.x
            yo = grasp.origin.y
            zo = grasp.origin.z
            xd, yd, zd = grasp.direction

            color = colors[color_idx]
            ax.plot3D(xo, yo, zo, "o", color=color)
            ax.plot3D([xo, xo + xd], [yo, yo + yd], [zo, zo + zd],
                      "-", color=color)
            color_idx += 1

        plt.axis("equal")
        plt.show()
