#!/usr/bin/env python3

from pyrobosim.manipulation.grasping import GraspGenerator, ParallelGraspProperties
from pyrobosim.utils.pose import Pose

if __name__ == "__main__":
    # Define inputs
    object_dims = [0.1, 0.05, 0.25]
    object_pose = Pose(x=0.5, y=-0.1, z=0.25)
    robot_pose = Pose(x=0.0, y=0.0, z=0.0)

    # Create a grasp generator
    properties = ParallelGraspProperties(
        max_width=0.15, depth=0.1, height=0.04,
        width_clearance=0.01, depth_clearance=0.01
    )
    gen = GraspGenerator(properties)

    # Generate grasps
    grasps = gen.generate(object_dims, object_pose, robot_pose,
        front_grasps=True, top_grasps=True, side_grasps=False)
    print(grasps)
    gen.show_grasps(object_dims, grasps, object_pose, robot_pose)
