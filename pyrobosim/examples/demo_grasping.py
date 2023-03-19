#!/usr/bin/env python3

from pyrobosim.core import Object
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties
from pyrobosim.utils.pose import Pose


def create_test_object(object_pose):
    """Helper function to create a test object."""
    obj_footprint_coords = [
        [0.05, 0.0],
        [0.025, 0.025],
        [-0.05, 0.025],
        [-0.025, 0.0],
        [-0.05, -0.025],
        [0.025, -0.025],
        [0.05, 0.0],
    ]
    obj_data = {}
    obj_data["test_object"] = {
        "footprint": {
            "type": "polygon",
            "coords": obj_footprint_coords,
            "height": 0.25,
        },
    }
    Object.metadata = obj_data

    return Object(category="test_object", pose=object_pose)


if __name__ == "__main__":
    # Define the inputs
    robot_pose = Pose(x=0.0, y=0.0, z=0.0)
    object_pose = Pose(x=0.5, y=0.1, z=0.2)
    obj = create_test_object(object_pose)
    cuboid_pose = obj.get_grasp_cuboid_pose()

    # Create a grasp generator
    properties = ParallelGraspProperties(
        max_width=0.15,
        depth=0.1,
        height=0.04,
        width_clearance=0.01,
        depth_clearance=0.01,
    )
    gen = GraspGenerator(properties)

    # Generate and display grasps
    grasps = gen.generate(
        obj.cuboid_dims,
        cuboid_pose,
        robot_pose,
        front_grasps=True,
        top_grasps=True,
        side_grasps=False,
    )
    print(grasps)
    gen.show_grasps(
        obj.cuboid_dims, grasps, cuboid_pose, robot_pose, obj.get_footprint()
    )
