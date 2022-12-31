#!/usr/bin/env python3

import numpy as np
from scipy.spatial import ConvexHull

from pyrobosim.manipulation.grasping import GraspGenerator, ParallelGraspProperties
from pyrobosim.utils.polygon import convhull_to_rectangle
from pyrobosim.utils.pose import Pose


if __name__ == "__main__":
    # Define the inputs
    robot_pose = Pose(x=0.0, y=0.0, z=0.0)
    object_pose = Pose(x=0.5, y=0.1, z=0.2)
    obj_footprint = np.array([
        [0.05, 0.0],
        [0.025, 0.025],
        [-0.05, 0.025],
        [-0.025, 0.0],
        [-0.05, -0.025],
        [0.025, -0.025],
        [0.05, 0.0]
    ])
    obj_height = 0.25

    # Fit a cuboid to the irregular object footprint
    hull = ConvexHull(obj_footprint)
    hull_pts = obj_footprint[hull.vertices, :]
    (rect_pose, rect_dims, _) = convhull_to_rectangle(hull_pts)

    # Define the object dimension and pose
    cuboid_dims = [rect_dims[0], rect_dims[1], obj_height]
    cuboid_pose = Pose.from_transform(
        np.matmul(rect_pose.get_transform_matrix(),
                  object_pose.get_transform_matrix())
    )

    # Create a grasp generator
    properties = ParallelGraspProperties(
        max_width=0.15, depth=0.1, height=0.04,
        width_clearance=0.01, depth_clearance=0.01
    )
    gen = GraspGenerator(properties)

    # Generate grasps
    grasps = gen.generate(cuboid_dims, cuboid_pose, robot_pose,
        front_grasps=True, top_grasps=True, side_grasps=False)
    print(grasps)
    gen.show_grasps(cuboid_dims, grasps, cuboid_pose, robot_pose, obj_footprint)
