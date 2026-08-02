"""Grasping utilities."""

from enum import Enum
from typing import Sequence

import numpy as np
import plotly.graph_objects as go
from plotly.colors import qualitative
from transforms3d.quaternions import rotate_vector, qinverse

from ..utils.pose import Pose

# A quadrilateral face for 3D visualization, as a list of four XYZ vertices.
Quad = list[list[float]]


def _mesh_from_quads(quads: list[Quad], color: str, opacity: float) -> go.Mesh3d:
    """
    Builds a Plotly 3D mesh trace from a list of quadrilateral faces.

    :param quads: The quadrilateral faces, each a list of four XYZ vertices.
    :param color: The CSS color of the mesh.
    :param opacity: The opacity of the mesh, in the range (0.0, 1.0).
    :return: The mesh trace.
    """
    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []
    i: list[int] = []
    j: list[int] = []
    k: list[int] = []
    for quad in quads:
        base = len(xs)
        for vx, vy, vz in quad:
            xs.append(vx)
            ys.append(vy)
            zs.append(vz)
        # Split the quad into two triangles.
        i += [base, base]
        j += [base + 1, base + 2]
        k += [base + 2, base + 3]
    return go.Mesh3d(
        x=xs,
        y=ys,
        z=zs,
        i=i,
        j=j,
        k=k,
        color=color,
        opacity=opacity,
        flatshading=True,
        hoverinfo="skip",
    )


class GraspFace(Enum):
    """Enumerates grasp face types."""

    UNKNOWN = 0
    FRONT = 1
    BACK = 2
    TOP = 3
    BOTTOM = 4
    LEFT = 5
    RIGHT = 6


normal_from_face = {
    GraspFace.UNKNOWN: None,
    GraspFace.FRONT: np.array([-1.0, 0.0, 0.0]),
    GraspFace.BACK: np.array([1.0, 0.0, 0.0]),
    GraspFace.TOP: np.array([0.0, 0.0, 1.0]),
    GraspFace.BOTTOM: np.array([0.0, 0.0, -1.0]),
    GraspFace.LEFT: np.array([0.0, 1.0, 0.0]),
    GraspFace.RIGHT: np.array([0.0, -1.0, 0.0]),
}


class GraspDirection(Enum):
    """Enumerates grasp direction types."""

    UNKNOWN = 0
    X_POS = 1
    X_NEG = 2
    Y_POS = 3
    Y_NEG = 4
    Z_POS = 5
    Z_NEG = 6


vec_from_direction = {
    GraspDirection.UNKNOWN: None,
    GraspDirection.X_POS: np.array([1.0, 0.0, 0.0]),
    GraspDirection.X_NEG: np.array([-1.0, 0.0, 0.0]),
    GraspDirection.Y_POS: np.array([0.0, 1.0, 0.0]),
    GraspDirection.Y_NEG: np.array([0.0, -1.0, 0.0]),
    GraspDirection.Z_POS: np.array([0.0, 0.0, 1.0]),
    GraspDirection.Z_NEG: np.array([0.0, 0.0, -1.0]),
}


class ParallelGraspProperties:
    """
    Representation of a parallel-jaw gripper.

    .. code-block::

            SIDE VIEW               TOP VIEW
                      |            _________
            _____     v           |              ^
        ===|_____|  height     ===|          max_width
                      ^           |_________     v
                      |
                                  |<-depth->|
               depth_clearance -->||<--
    """

    def __init__(
        self,
        max_width: float,
        depth: float,
        height: float,
        width_clearance: float = 0.0,
        depth_clearance: float = 0.0,
    ) -> None:
        """
        Creates a parallel gripper grasp properties instance.

        :param max_width: Maximum gripper opening width
        :param depth: Distance from end effector base to finger tips (or equivalent)
        :param height: Height of end effector finger tips (or equivalent)
        :param width_clearance: Width clearance so grasps are not flush with gripper fingers
        :param depth_clearance: Depth clearance so grasps are not flush with gripper base
        """
        self.max_width = max_width
        self.depth = depth
        self.height = height
        self.width_clearance = width_clearance
        self.depth_clearance = depth_clearance

    def __repr__(self) -> str:
        """Printable string representation"""
        display_str = "Parallel jaw gripper properties:\n"
        display_str += (
            f"\tMax width: {self.max_width}, depth: {self.depth}, height: {self.height}"
        )
        return display_str


class Grasp:
    """
    Representation of an object grasp.
    """

    def __init__(
        self,
        properties: ParallelGraspProperties,
        origin_wrt_object: Pose,
        origin_wrt_world: Pose | None = None,
        face: GraspFace = GraspFace.UNKNOWN,
        direction: GraspDirection = GraspDirection.UNKNOWN,
    ) -> None:
        """
        Creates a grasp object instance.

        :param properties: The parallel grasp properties to use.
        :param origin_wrt_object: Grasp origin pose, expressed with respect to the object
        :param origin_wrt_world: Grasp origin pose, expressed with respect to the world
        :param face: Enumeration denoting grasp face relative to object.
        :param direction: Enumeration denoting grasp direction relative to object.
        """
        self.properties = properties
        self.origin_wrt_object = origin_wrt_object
        self.origin_wrt_world = origin_wrt_world
        self.face = face
        self.direction = direction

    def translate_origin(self, vec: list[float]) -> list[float]:
        """
        Adds the origin position to a specified position vector.

        :param vec: Original position vector
        :return: Translated position vector
        """
        return [
            self.origin_wrt_object.x + vec[0],
            self.origin_wrt_object.y + vec[1],
            self.origin_wrt_object.z + vec[2],
        ]

    def get_gripper_quads(self) -> list[Quad]:
        """
        Computes the gripper faces for visualizing the grasp.

        :return: The gripper faces (right finger, left finger, and base).
        """
        d = self.properties.depth
        h = self.properties.height / 2
        w = self.properties.max_width / 2

        qinv = qinverse(self.origin_wrt_object.q)
        left_bottom_base = self.translate_origin(rotate_vector([w, -h, -d], qinv))
        left_top_base = self.translate_origin(rotate_vector([w, h, -d], qinv))
        left_bottom_tip = self.translate_origin(rotate_vector([w, -h, 0], qinv))
        left_top_tip = self.translate_origin(rotate_vector([w, h, 0], qinv))
        right_bottom_base = self.translate_origin(rotate_vector([-w, -h, -d], qinv))
        right_top_base = self.translate_origin(rotate_vector([-w, h, -d], qinv))
        right_bottom_tip = self.translate_origin(rotate_vector([-w, -h, 0], qinv))
        right_top_tip = self.translate_origin(rotate_vector([-w, h, 0], qinv))

        return [
            # Right gripper
            [right_bottom_base, right_bottom_tip, right_top_tip, right_top_base],
            # Left gripper
            [left_bottom_base, left_bottom_tip, left_top_tip, left_top_base],
            # Gripper base
            [right_bottom_base, left_bottom_base, left_top_base, right_top_base],
        ]

    def __repr__(self) -> str:
        """Printable string representation"""
        display_str = f"Grasp:\n"
        display_str += f"\tOrigin w.r.t. object:{self.origin_wrt_object}\n"
        if self.origin_wrt_world is not None:
            display_str += f"\tOrigin w.r.t. world:{self.origin_wrt_world}\n"
        display_str += f"\tFace: {self.face}\n\tDirection: {self.direction}\n"
        if self.properties is not None:
            display_str += f"\t{self.properties}\n"
        return display_str


class GraspGenerator:
    """
    Generates grasps given object dimensions and pose relative to a robot.
    """

    def __init__(self, properties: ParallelGraspProperties) -> None:
        """
        Creates a grasp generator instance given grasping properties.

        :param properties: Grasping properties object
        """
        self.properties = properties

    def compute_robot_facing_rot(
        self, object_pose: Pose = Pose(), robot_pose: Pose | None = None
    ) -> np.ndarray:
        """
        Computes the rotation matrix to convert from nominal cuboid orientation to robot-facing orientation.

        The nominal orientation is such that the front face of the cuboid is -X, the sides are -Y and +Y,
        and the top is +Z, as shown below:

        .. code-block::

                                      Z
                        +----+        ^
            robot -->   |    |        |
                        +----+        +--> X

        If the robot is facing from another direction, for example the left side, then the front face in the
        robot reference frame is the +Y and the side faces are -X and +X.
        This function therefore returns the rotation matrix to perform this alignment to correspond which of
        the 6 cuboid faces corresponds to directions such as "front", "right", "top", etc. from the perspective
        of the robot.

        :param object_pose: The pose of the object center, defaults to identity transform
        :param robot_pose: The pose of the robot. If none specified, it is not used in calculations.
        :return: Rotation matrix transforming the nominal cuboid orientation to a robot-facing orientation
        """
        rot_matrix = np.eye(3)
        if robot_pose is None:
            return rot_matrix

        tform_robot_to_object = np.matmul(
            np.linalg.inv(robot_pose.get_transform_matrix()),
            object_pose.get_transform_matrix(),
        )
        v_robot_to_object = np.dot(
            tform_robot_to_object[:3, :3].T, tform_robot_to_object[:3, 3]
        )

        # Figure out what the front face is
        v_robot_rt_object_proj = np.array(
            [-v_robot_to_object[0], -v_robot_to_object[1], 0.0]
        )
        v_robot_rt_object_proj /= np.linalg.norm(v_robot_rt_object_proj)
        all_grasp_faces = [
            GraspFace.FRONT,
            GraspFace.BACK,
            GraspFace.TOP,
            GraspFace.BOTTOM,
            GraspFace.LEFT,
            GraspFace.RIGHT,
        ]
        max_dot_prod = -10  # Unrealistic value for dot product
        for face in all_grasp_faces:
            dot_prod = np.dot(v_robot_rt_object_proj, normal_from_face[face])
            if dot_prod > max_dot_prod:
                max_dot_prod = dot_prod
                front_face_dir = face

        # Figure out what the top face is
        unit_z = np.dot(
            tform_robot_to_object[:3, :3].T, np.array([[0.0], [0.0], [1.0]])
        ).T
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
        rot_matrix[:, 0] = x_vec
        rot_matrix[:, 1] = y_vec
        rot_matrix[:, 2] = z_vec
        return rot_matrix

    def should_try_grasp(
        self,
        faces_enabled: Sequence[bool],
        face_normals: Sequence[np.ndarray],
        face_vec: np.ndarray,
    ) -> tuple[bool, GraspFace]:
        """
        Helper function to validate whether to compute grasps on a specific face.

        Given the set of enabled grasp types, and the normal vectors of the cuboid faces, this function
        determines whether grasping along a specific normal vector is permitted.
        For example, if

            - Only top and front grasps are enabled
            - The top face normal vector is [0, 0, 1]
            - The front face normal vector is [-1, 0, 0]

        Then this function will

            - Allow grasps along the vectors [0, 0, 1] and [-1, 0, 0] because these are the top and front faces
            - Disallow a grasp along the vectors [0, 1, 0]  and [0, -1, 0] because side grasps are disabled
            - Disallow a grasp along the vector [1, 0, 0] because this would be a back grasp which is not supported

        :param faces_enabled: Faces for which grasp generation is enabled, in the form (front, top, side)
        :param face_normals: Face normals in the canonical directions, in the form (front, top, left, right)
        :param face_vec: Normal vector of the cuboid face to check.
        :return: A tuple determining whether the grasp should be attempted, and what face that corresponds to
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

    def _create_grasp(
        self,
        grasp_center: Pose,
        grasp_face: GraspFace,
        grasp_dir: GraspDirection,
        object_pose: Pose,
    ) -> Grasp:
        """
        Helper function to create a grasp object.

        :param grasp_center: The grasp origin pose with respect to the object.
        :param grasp_face: The grasp face
        :param grasp_dir: The grasp direction
        :param object_pose: The object pose in world coordinates.
        :return: Grasp object
        """
        grasp_wrt_world = Pose.from_transform(
            np.matmul(
                grasp_center.get_transform_matrix(),
                object_pose.get_transform_matrix(),
            )
        )
        return Grasp(
            properties=self.properties,
            origin_wrt_object=grasp_center,
            origin_wrt_world=grasp_wrt_world,
            face=grasp_face,
            direction=grasp_dir,
        )

    def generate(
        self,
        object_dims: list[float],
        object_pose: Pose = Pose(),
        robot_pose: Pose | None = None,
        top_grasps: bool = True,
        front_grasps: bool = True,
        side_grasps: bool = True,
    ) -> list[Grasp]:
        """
        Generates a set of axis-aligned grasps for a cuboid object.

        :param object_dims: List containing the object [x, y, z] dimensions
        :param object_pose: The pose of the object center, defaults to identity transform
        :param robot_pose: The pose of the robot. If none specified, it is not used in calculations.
        :param top_grasps: Enable top grasp generation, defaults to True
        :param front_grasps: Enable front grasp generation, defaults to True
        :param side_grasps: Enable side grasp generation, defaults to True
        :return: A list of generated grasps
        """
        grasps = []
        rot_matrix = self.compute_robot_facing_rot(object_pose, robot_pose)
        front_face_vec = np.dot(rot_matrix, normal_from_face[GraspFace.FRONT])
        top_face_vec = np.dot(rot_matrix, normal_from_face[GraspFace.TOP])
        left_face_vec = np.dot(rot_matrix, normal_from_face[GraspFace.LEFT])
        right_face_vec = np.dot(rot_matrix, normal_from_face[GraspFace.RIGHT])

        # Unpack useful variables
        object_x, object_y, object_z = object_dims
        effective_max_width = (
            self.properties.max_width - self.properties.width_clearance
        )
        effective_depth = self.properties.depth - self.properties.depth_clearance

        # Compute all feasible grasps
        face_normals = (front_face_vec, top_face_vec, left_face_vec, right_face_vec)
        directions_enabled = (front_grasps, top_grasps, side_grasps)

        #################
        # -X face grasp #
        #################
        grasp_dir = GraspDirection.X_NEG
        try_grasp, grasp_face = self.should_try_grasp(
            directions_enabled, face_normals, vec_from_direction[grasp_dir]
        )
        if try_grasp:
            x = min(0.0, effective_depth - object_x / 2)
            # Grasp with horizontal gripper, uses Y dimension
            if effective_max_width >= object_y:
                grasp_center = Pose(x=x, y=0.0, z=0.0, pitch=-np.pi / 2, yaw=np.pi / 2)
                grasps.append(
                    self._create_grasp(grasp_center, grasp_face, grasp_dir, object_pose)
                )
            # Grasp with vertical gripper, uses Z dimension
            if effective_max_width >= object_z:
                grasp_center = Pose(x=x, y=0.0, z=0.0, pitch=-np.pi / 2, yaw=0.0)
                grasps.append(
                    self._create_grasp(grasp_center, grasp_face, grasp_dir, object_pose)
                )

        #################
        # +X face grasp #
        #################
        grasp_dir = GraspDirection.X_POS
        try_grasp, grasp_face = self.should_try_grasp(
            directions_enabled, face_normals, vec_from_direction[grasp_dir]
        )
        if try_grasp:
            x = max(0.0, object_x / 2 - effective_depth)
            # Grasp with horizontal gripper, uses Y dimension
            if effective_max_width >= object_y:
                grasp_center = Pose(x=x, y=0.0, z=0.0, pitch=np.pi / 2, yaw=np.pi / 2)
                grasps.append(
                    self._create_grasp(grasp_center, grasp_face, grasp_dir, object_pose)
                )
            # Grasp with vertical gripper, uses Z dimension
            if effective_max_width >= object_z:
                grasp_center = Pose(x=x, y=0.0, z=0.0, pitch=np.pi / 2, yaw=0.0)
                grasps.append(
                    self._create_grasp(grasp_center, grasp_face, grasp_dir, object_pose)
                )

        #################
        # -Z face grasp #
        #################
        grasp_dir = GraspDirection.Z_NEG
        try_grasp, grasp_face = self.should_try_grasp(
            directions_enabled, face_normals, vec_from_direction[grasp_dir]
        )
        if try_grasp:
            z = min(0.0, effective_depth - object_z / 2)
            # Top grasp with horizontal gripper, uses Y dimension
            if effective_max_width >= object_y:
                grasp_center = Pose(x=0.0, y=0.0, z=z, pitch=0.0, yaw=np.pi / 2)
                grasps.append(
                    self._create_grasp(grasp_center, grasp_face, grasp_dir, object_pose)
                )
            # Top grasp with vertical gripper, uses X dimension
            if effective_max_width >= object_x:
                grasp_center = Pose(x=0.0, y=0.0, z=z, pitch=0.0, yaw=0.0)
                grasps.append(
                    self._create_grasp(grasp_center, grasp_face, grasp_dir, object_pose)
                )

        #################
        # +Z face grasp #
        #################
        grasp_dir = GraspDirection.Z_POS
        try_grasp, grasp_face = self.should_try_grasp(
            directions_enabled, face_normals, vec_from_direction[grasp_dir]
        )
        if try_grasp:
            z = max(0.0, object_z / 2 - effective_depth)
            # Top grasp with horizontal gripper, uses Y dimension
            if effective_max_width >= object_y:
                grasp_center = Pose(x=0.0, y=0.0, z=z, pitch=np.pi, yaw=np.pi / 2)
                grasps.append(
                    self._create_grasp(grasp_center, grasp_face, grasp_dir, object_pose)
                )
            # Top grasp with vertical gripper, uses X dimension
            if effective_max_width >= object_x:
                grasp_center = Pose(x=0.0, y=0.0, z=z, pitch=np.pi, yaw=0.0)
                grasps.append(
                    self._create_grasp(grasp_center, grasp_face, grasp_dir, object_pose)
                )

        #################
        # -Y face grasp #
        #################
        grasp_dir = GraspDirection.Y_NEG
        try_grasp, grasp_face = self.should_try_grasp(
            directions_enabled, face_normals, vec_from_direction[grasp_dir]
        )
        if try_grasp:
            y = min(0.0, effective_depth - object_y / 2)
            # Grasp with horizontal gripper, uses X dimension
            if effective_max_width >= object_x:
                grasp_center = Pose(x=0.0, y=y, z=0.0, roll=np.pi / 2)
                grasps.append(
                    self._create_grasp(grasp_center, grasp_face, grasp_dir, object_pose)
                )
            # Grasp with vertical gripper, uses Z dimension
            if effective_max_width >= object_z:
                grasp_center = Pose(x=0.0, y=y, z=0.0, roll=np.pi / 2, yaw=np.pi / 2)
                grasps.append(
                    self._create_grasp(grasp_center, grasp_face, grasp_dir, object_pose)
                )

        #################
        # +Y face grasp #
        #################
        grasp_dir = GraspDirection.Y_POS
        try_grasp, grasp_face = self.should_try_grasp(
            directions_enabled, face_normals, vec_from_direction[grasp_dir]
        )
        if try_grasp:
            y = max(0.0, object_y / 2 - effective_depth)
            # Left grasp with horizontal gripper, uses X dimension
            if effective_max_width >= object_x:
                grasp_center = Pose(x=0.0, y=y, z=0.0, roll=-np.pi / 2)
                grasps.append(
                    self._create_grasp(grasp_center, grasp_face, grasp_dir, object_pose)
                )
            # Left grasp with vertical gripper, uses Z dimension
            if effective_max_width >= object_z:
                grasp_center = Pose(x=0.0, y=y, z=0.0, roll=-np.pi / 2, yaw=-np.pi / 2)
                grasps.append(
                    self._create_grasp(grasp_center, grasp_face, grasp_dir, object_pose)
                )

        return grasps

    def show_grasps(
        self,
        object_dims: list[float],
        grasps: list[Grasp],
        object_pose: Pose = Pose(),
        robot_pose: Pose | None = None,
        object_footprint: np.ndarray | None = None,
    ) -> None:
        """
        Display the grasps on top of an object in the browser.

        :param object_dims: List containing the object [x, y, z] dimensions
        :param grasps: A list of grasps
        :param object_pose: The pose of the object center, defaults to identity transform
        :param robot_pose: The pose of the robot. If none specified, it is not used in calculations.
        :param object_footprint: Optional N-by-2 array of the object footprint points to overlay
        """
        fig = go.Figure()

        # Show the object cuboid
        x, y, z = [d / 2 for d in object_dims]
        cuboid_quads: list[Quad] = [
            [[-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z]],
            [[-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z]],
            [[-x, -y, -z], [-x, y, -z], [-x, y, z], [-x, -y, z]],
            [[x, -y, -z], [x, y, -z], [x, y, z], [x, -y, z]],
            [[-x, -y, -z], [x, -y, -z], [x, -y, z], [-x, -y, z]],
            [[-x, y, -z], [x, y, -z], [x, y, z], [-x, y, z]],
        ]
        fig.add_trace(_mesh_from_quads(cuboid_quads, "gray", opacity=0.3))

        # Show the object footprint points, if specified
        if object_footprint is not None:
            fig.add_trace(
                go.Scatter3d(
                    x=object_footprint[:, 0],
                    y=object_footprint[:, 1],
                    z=[-z] * len(object_footprint),
                    mode="lines",
                    line={"color": "gray", "width": 4},
                    showlegend=False,
                )
            )

        # Show the robot pose, if present
        if robot_pose is not None:
            p_robot_rt_object = Pose.from_transform(
                np.matmul(
                    np.linalg.inv(object_pose.get_transform_matrix()),
                    robot_pose.get_transform_matrix(),
                )
            )
            xr = p_robot_rt_object.x
            yr = p_robot_rt_object.y
            zr = p_robot_rt_object.z
            fig.add_trace(
                go.Scatter3d(
                    x=[0, xr],
                    y=[0, yr],
                    z=[0, zr],
                    mode="lines+markers",
                    line={"color": "black", "dash": "dash", "width": 2},
                    marker={"color": "black", "size": [0, 8]},
                    name="robot",
                )
            )

        # Show the grasps
        colors = qualitative.Plotly
        for color_idx, grasp in enumerate(grasps):
            xo = grasp.origin_wrt_object.x
            yo = grasp.origin_wrt_object.y
            zo = grasp.origin_wrt_object.z
            xd, yd, zd = normal_from_face[grasp.face]
            depth = grasp.properties.depth

            # Plot the grasp point and approach direction
            color = colors[color_idx % len(colors)]
            fig.add_trace(
                go.Scatter3d(
                    x=[xo, xo + depth * xd],
                    y=[yo, yo + depth * yd],
                    z=[zo, zo + depth * zd],
                    mode="lines+markers",
                    line={"color": color, "dash": "dot", "width": 4},
                    marker={"color": color, "size": [5, 0]},
                    name=f"grasp {color_idx}",
                )
            )

            # Plot the grasp itself
            fig.add_trace(
                _mesh_from_quads(grasp.get_gripper_quads(), color, opacity=0.8)
            )

        # Set an equal-aspect view spanning the object dimensions.
        max_dim = max(object_dims)
        axis_settings = {"range": [-max_dim, max_dim]}
        fig.update_layout(
            scene={
                "xaxis": {"title": "X", **axis_settings},
                "yaxis": {"title": "Y", **axis_settings},
                "zaxis": {"title": "Z", **axis_settings},
                "aspectmode": "cube",
            },
            title="Grasps",
        )
        fig.show()

    def to_dict(self) -> dict[str, str | float]:
        """
        Serializes the grasp generator to a dictionary.

        :return: A dictionary containing the grasp generator information.
        """
        return {
            "generator": "parallel_grasp",
            "max_width": self.properties.max_width,
            "depth": self.properties.depth,
            "height": self.properties.height,
            "width_clearance": self.properties.width_clearance,
            "depth_clearance": self.properties.depth_clearance,
        }
