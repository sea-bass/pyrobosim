""" Grasping utilities. """

from ..utils.pose import Pose

from enum import Enum
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from transforms3d.quaternions import rotate_vector, qinverse


class GraspFace(Enum):
    """ Enumerates grasp face types. """
    UNKNOWN = 0
    FRONT = 1
    BACK = 2
    TOP = 3
    BOTTOM = 4
    LEFT = 5
    RIGHT = 6

normal_from_face = {
    GraspFace.UNKNOWN: None,
    GraspFace.FRONT:   np.array([-1.0, 0.0, 0.0]),
    GraspFace.BACK:    np.array([1.0, 0.0, 0.0]),
    GraspFace.TOP:     np.array([0.0, 0.0, 1.0]),
    GraspFace.BOTTOM:  np.array([0.0, 0.0, -1.0]),
    GraspFace.LEFT:    np.array([0.0, 1.0, 0.0]),
    GraspFace.RIGHT:   np.array([0.0, -1.0, 0.0]),
}

class GraspDirection(Enum):
    """ Enumerates grasp direction types. """
    UNKNOWN = 0
    X_POS = 1
    X_NEG = 2
    Y_POS = 3
    Y_NEG = 4
    Z_POS = 5
    Z_NEG = 6

vec_from_direction = {
    GraspDirection.UNKNOWN: None,
    GraspDirection.X_POS:   np.array([1.0, 0.0, 0.0]),
    GraspDirection.X_NEG:   np.array([-1.0, 0.0, 0.0]),
    GraspDirection.Y_POS:   np.array([0.0, 1.0, 0.0]),
    GraspDirection.Y_NEG:   np.array([0.0, -1.0, 0.0]),
    GraspDirection.Z_POS:   np.array([0.0, 0.0, 1.0]),
    GraspDirection.Z_NEG:   np.array([0.0, 0.0, -1.0]),
}

class Grasp:
    """
    Representation of an object grasp.
    """
    def __init__(self, origin, properties,
                 face=GraspFace.UNKNOWN, direction=GraspDirection.UNKNOWN):
        """
        Creates a grasp object instance.
        
        :param origin: Grasp origin pose
        :type origin: :class:`pyrobosim.utils.pose.Pose`
        :param properties: Grasping properties object
        :type properties: :class:`pyrobosim.manipulation.grasping.ParallelGraspProperties`
        :param face: Enumeration denoting grasp face relative to object.
        :type face: :class:`pyrobosim.manipulation.grasping.GraspFace`, optional
        :param direction: Enumeration denoting grasp direction relative to object.
        :type direction: :class:`pyrobosim.manipulation.grasping.GraspDirection`, optional
        """
        self.origin = origin
        self.face = face
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
        display_str = f"Grasp:\n\t{self.origin}\n"
        display_str += f"\tFace: {self.face}\n\tDirection: {self.direction}\n"
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
           depth_clearance -->||<--
    """
    def __init__(self, max_width, depth, height, width_clearance=0.0, depth_clearance=0.0):
        """
        Creates a parallel gripper grasp properties instance.

        :param max_width: Maximum gripper opening width 
        :type max_width: float
        :param depth: Distance from end effector base to finger tips (or equivalent)
        :type depth: float
        :param height: Height of end effector finger tips (or equivalent)
        :type height: float
        :param width_clearance: Width clearance so grasps are not flush with gripper fingers
        :type width_clearance: float, optional
        :param depth_clearance: Depth clearance so grasps are not flush with gripper base
        :type depth_clearance: float, optional
        """
        self.max_width = max_width
        self.depth = depth
        self.height = height
        self.width_clearance = width_clearance
        self.depth_clearance = depth_clearance

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

    def compute_robot_facing_rot(self, object_pose=Pose(), robot_pose=None):
        """
        Computes the rotation matrix to convert from nominal cuboid orientation to robot-facing orientation.
        
        :param object_pose: The pose of the object center, defaults to identity transform
        :type object_pose: :class:`pyrobosim.utils.pose.Pose`, optional
        :param robot_pose: The pose of the robot. If none specified, it is not used in calculations.
        :type robot_pose: :class:`pyrobosim.utils.pose.Pose`, optional
        :return: Rotation matrix transforming the nominal cuboid orientation to a robot-facing orientation
        :rtype: :class:`numpy.ndarray`
        """
        rot_matrix = np.eye(3)
        if robot_pose is None:
            return rot_matrix

        tform_robot_to_object = np.matmul(
            np.linalg.inv(robot_pose.get_transform_matrix()),
            object_pose.get_transform_matrix())
        v_robot_to_object = np.dot(
            tform_robot_to_object[:3, :3].T, tform_robot_to_object[:3, 3])

        # Figure out what the front face is
        v_robot_rt_object_proj = np.array([-v_robot_to_object[0],
                                           -v_robot_to_object[1],
                                           0.0])
        v_robot_rt_object_proj /= np.linalg.norm(v_robot_rt_object_proj)
        all_grasp_faces = [GraspFace.FRONT, GraspFace.BACK,
                           GraspFace.TOP, GraspFace.BOTTOM,
                           GraspFace.LEFT, GraspFace.RIGHT]
        max_dot_prod = -10  # Unrealistic value for dot product
        for face in all_grasp_faces:
            dot_prod = np.dot(v_robot_rt_object_proj, normal_from_face[face])
            if dot_prod > max_dot_prod:
                max_dot_prod = dot_prod
                front_face_dir = face

        # Figure out what the top face is
        unit_z = np.dot(
            tform_robot_to_object[:3, :3].T,
            np.array([[0.0], [0.0], [1.0]])).T
        max_dot_prod = -10  # Unrealistic value for dot product
        for face in all_grasp_faces:
            dot_prod = np.dot(unit_z, normal_from_face[face])
            if dot_prod > max_dot_prod:
                max_dot_prod = dot_prod
                top_face_dir = face

        # Compute the transform from nominal coordinates to robot-facing
        x_vec = -1.0 * normal_from_face[front_face_dir]
        z_vec = normal_from_face[top_face_dir]
        y_vec = np.cross(-1.0 * x_vec, z_vec)
        rot_matrix[:,0] = x_vec
        rot_matrix[:,1] = y_vec
        rot_matrix[:,2] = z_vec
        return rot_matrix

    def should_try_grasp(self, faces_enabled, face_normals, face_vec):
        """
        Helper function to validate whether to compute grasps on a specific face.
        
        :param faces_enabled: Faces enabled, in the form (front, top, side)
        :type faces_enabled: list[bool]
        :param face_normals: Face normals in the canonical directions, in the form (front, top, left, right)
        :type face_normals: list[:class:`numpy.ndarray`]
        :param face_vec: Face vector to check against
        :type face_vec: :class:`numpy.ndarray`
        :return: A tuple determining whether the grasp should be attempted, and what face that corresponds to
        :type face_vec: (bool, :class:`pyrobosim.manipulation.grasping.GraspFace`)
        """
        try_grasp = False
        grasp_face = GraspFace.UNKNOWN

        front_grasps, top_grasps, side_grasps = faces_enabled
        front_face_vec, top_face_vec, left_face_vec, right_face_vec = face_normals

        if front_grasps and np.allclose(front_face_vec, face_vec):
            try_grasp = True
            grasp_face = GraspFace.FRONT
        elif top_grasps and np.allclose(top_face_vec, face_vec):
            try_grasp = True
            grasp_face = GraspFace.TOP
        elif side_grasps:
            if np.allclose(left_face_vec, face_vec):
                try_grasp = True
                grasp_face = GraspFace.LEFT
            elif np.allclose(right_face_vec, face_vec):
                try_grasp = True
                grasp_face = GraspFace.RIGHT

        return (try_grasp, grasp_face)

    def generate(self, object_dims, object_pose=Pose(), robot_pose=None,
                 top_grasps=True, front_grasps=True, side_grasps=True):
        """
        Generates a set of axis-aligned grasps for a cuboid object.

        :param object_dims: List containing the object [x, y, z] dimensions
        :type object_dims: list[float]
        :param object_pose: The pose of the object center, defaults to identity transform
        :type object_pose: :class:`pyrobosim.utils.pose.Pose`, optional
        :param robot_pose: The pose of the robot. If none specified, it is not used in calculations.
        :type robot_pose: :class:`pyrobosim.utils.pose.Pose`, optional
        :param top_grasps: Enable top grasp generation, defaults to True
        :type top_grasps: bool, optional
        :param front_grasps: Enable front grasp generation, defaults to True
        :type front_grasps: bool, optional
        :param side_grasps: Enable side grasp generation, defaults to True
        :type side_grasps: bool, optional
        :return: A list of generated grasps
        :rtype: list[:class:`pyrobosim.manipulation.grasping.Grasp`]      
        """
        grasps = []
        rot_matrix = self.compute_robot_facing_rot(object_pose, robot_pose)
        front_face_vec = np.dot(rot_matrix, normal_from_face[GraspFace.FRONT])
        top_face_vec = np.dot(rot_matrix, normal_from_face[GraspFace.TOP])
        left_face_vec = np.dot(rot_matrix, normal_from_face[GraspFace.LEFT])
        right_face_vec = np.dot(rot_matrix, normal_from_face[GraspFace.RIGHT])

        # Unpack useful variables
        object_x, object_y, object_z = object_dims
        effective_max_width = self.properties.max_width - self.properties.width_clearance
        effective_depth = self.properties.depth - self.properties.depth_clearance

        # Compute all feasible grasps
        face_normals = (front_face_vec, top_face_vec, left_face_vec, right_face_vec)
        directions_enabled = (front_grasps, top_grasps, side_grasps)

        #################
        # -X face grasp #
        #################
        grasp_dir = GraspDirection.X_NEG
        (try_grasp, grasp_face) = self.should_try_grasp(
            directions_enabled, face_normals, vec_from_direction[grasp_dir])
        if try_grasp:
            x = min(0.0, effective_depth - object_x / 2)
            # Grasp with horizontal gripper, uses Y dimension
            if effective_max_width >= object_y:
                grasp_center = Pose(x=x, y=0.0, z=0.0,
                                    pitch=-np.pi/2, yaw=np.pi/2)
                grasps.append(
                    Grasp(origin=grasp_center, properties=self.properties,
                          face=grasp_face, direction=grasp_dir)
                )
            # Grasp with vertical gripper, uses Z dimension
            if effective_max_width >= object_z:
                grasp_center = Pose(x=x, y=0.0, z=0.0,
                                    pitch=-np.pi/2, yaw=0.0)
                grasps.append(
                    Grasp(origin=grasp_center, properties=self.properties,
                          face=grasp_face, direction=grasp_dir)
                )

        #################
        # +X face grasp #
        #################
        grasp_dir = GraspDirection.X_POS
        (try_grasp, grasp_face) = self.should_try_grasp(
            directions_enabled, face_normals, vec_from_direction[grasp_dir])
        if try_grasp:
            x = max(0.0, object_x / 2 - effective_depth)
            # Grasp with horizontal gripper, uses Y dimension
            if effective_max_width >= object_y:
                grasp_center = Pose(x=x, y=0.0, z=0.0,
                                    pitch=np.pi/2, yaw=np.pi/2)
                grasps.append(
                    Grasp(origin=grasp_center, properties=self.properties,
                          face=grasp_face, direction=grasp_dir)
                )
            # Grasp with vertical gripper, uses Z dimension
            if effective_max_width >= object_z:
                grasp_center = Pose(x=x, y=0.0, z=0.0,
                                    pitch=np.pi/2, yaw=0.0)
                grasps.append(
                    Grasp(origin=grasp_center, properties=self.properties,
                          face=grasp_face, direction=grasp_dir)
                )

        #################
        # -Z face grasp #
        #################
        grasp_dir = GraspDirection.Z_NEG
        (try_grasp, grasp_face) = self.should_try_grasp(
            directions_enabled, face_normals, vec_from_direction[grasp_dir])
        if try_grasp:
            z = min(0.0, effective_depth - object_z / 2)
            # Top grasp with horizontal gripper, uses Y dimension
            if effective_max_width >= object_y:
                grasp_center = Pose(x=0.0, y=0.0, z=z,
                                    pitch=0.0, yaw=np.pi/2)
                grasps.append(
                    Grasp(origin=grasp_center, properties=self.properties,
                          face=grasp_face, direction=grasp_dir)
                )
            # Top grasp with vertical gripper, uses X dimension
            if effective_max_width >= object_x:
                grasp_center = Pose(x=0.0, y=0.0, z=z,
                                    pitch=0.0, yaw=0.0)
                grasps.append(
                    Grasp(origin=grasp_center, properties=self.properties,
                          face=grasp_face, direction=grasp_dir)
            )

        #################
        # +Z face grasp #
        #################
        grasp_dir = GraspDirection.Z_POS
        (try_grasp, grasp_face) = self.should_try_grasp(
            directions_enabled, face_normals, vec_from_direction[grasp_dir])
        if try_grasp:
            z = max(0.0, object_z / 2 - effective_depth)
            # Top grasp with horizontal gripper, uses Y dimension
            if effective_max_width >= object_y:
                grasp_center = Pose(x=0.0, y=0.0, z=z,
                                    pitch=np.pi, yaw=np.pi/2)
                grasps.append(
                    Grasp(origin=grasp_center, properties=self.properties,
                          face=grasp_face, direction=grasp_dir)
                )
            # Top grasp with vertical gripper, uses X dimension
            if effective_max_width >= object_x:
                grasp_center = Pose(x=0.0, y=0.0, z=z,
                                    pitch=np.pi, yaw=0.0)
                grasps.append(
                    Grasp(origin=grasp_center, properties=self.properties,
                          face=grasp_face, direction=grasp_dir)
            )

        #################
        # -Y face grasp #
        #################
        grasp_dir = GraspDirection.Y_NEG
        (try_grasp, grasp_face) = self.should_try_grasp(
            directions_enabled, face_normals, vec_from_direction[grasp_dir])
        if try_grasp:
            y = min(0.0, effective_depth - object_y / 2)
            # Grasp with horizontal gripper, uses X dimension
            if effective_max_width >= object_x:
                grasp_center = Pose(x=0.0, y=y, z=0.0,
                                    roll=np.pi/2)
                grasps.append(
                    Grasp(origin=grasp_center, properties=self.properties,
                          face=grasp_face, direction=grasp_dir)
                )
            # Grasp with vertical gripper, uses Z dimension
            if effective_max_width >= object_z:
                grasp_center = Pose(x=0.0, y=y, z=0.0,
                                    roll=np.pi/2, yaw=np.pi/2)
                grasps.append(
                    Grasp(origin=grasp_center, properties=self.properties,
                    face=grasp_face, direction=grasp_dir)
                )

        #################
        # +Y face grasp #
        #################
        grasp_dir = GraspDirection.Y_POS
        (try_grasp, grasp_face) = self.should_try_grasp(
            directions_enabled, face_normals, vec_from_direction[grasp_dir])
        if try_grasp:
            y = max(0.0, object_y / 2 - effective_depth)
            # Left grasp with horizontal gripper, uses X dimension
            if effective_max_width >= object_x:
                grasp_center = Pose(x=0.0, y=y, z=0.0,
                                    roll=-np.pi/2)
                grasps.append(
                    Grasp(origin=grasp_center, properties=self.properties,
                          face=grasp_face, direction=grasp_dir)
                )
            # Left grasp with vertical gripper, uses Z dimension
            if effective_max_width >= object_z:
                grasp_center = Pose(x=0.0, y=y, z=0.0,
                                    roll=-np.pi/2, yaw=-np.pi/2)
                grasps.append(
                    Grasp(origin=grasp_center, properties=self.properties,
                          face=grasp_face, direction=grasp_dir)
                )

        return grasps


    def show_grasps(self, object_dims, grasps, object_pose=Pose(), robot_pose=None):
        """
        Display the grasps on top of an object.

        :param object_dims: List containing the object [x, y, z] dimensions
        :type object_dims: list[float]
        :param grasps: A list of grasps
        :type grasps: list[:class:`pyrobosim.manipulation.grasping.Grasp`]
        :param object_pose: The pose of the object center, defaults to identity transform
        :type object_pose: :class:`pyrobosim.utils.pose.Pose`, optional
        :param robot_pose: The pose of the robot. If none specified, it is not used in calculations.
        :type robot_pose: :class:`pyrobosim.utils.pose.Pose`, optional
        """
        fig = plt.figure()
        ax = Axes3D(fig)
        fig.add_axes(ax)
        
        # Show the object
        max_dim = max(object_dims)
        min_x = min_y = min_z = -max_dim
        max_x = max_y = max_z = max_dim
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

        # Show the robot pose, if present
        if robot_pose is not None:
            p_robot_rt_object = Pose.from_transform(
                np.matmul(
                    np.linalg.inv(object_pose.get_transform_matrix()),
                    robot_pose.get_transform_matrix())
            )
            xr = p_robot_rt_object.x
            yr = p_robot_rt_object.y
            zr = p_robot_rt_object.z
            ax.plot3D(xr, yr, zr, "ko", markersize=10)
            ax.plot3D([0, xr], [0, yr], [0, zr], "k--", linewidth=1)

        # Show the grasps
        color_idx = 0
        colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
        for grasp in grasps:
            xo = grasp.origin.x
            yo = grasp.origin.y
            zo = grasp.origin.z
            xd, yd, zd = normal_from_face[grasp.face]
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
        ax.axes.set_xlim3d(min_x, max_x)
        ax.axes.set_ylim3d(min_y, max_y)
        ax.axes.set_zlim3d(min_z, max_z)
        plt.show()
