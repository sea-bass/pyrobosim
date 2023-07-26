#!/usr/bin/env python3

"""
Unit tests for grasp generation.
"""

from pyrobosim.manipulation.grasping import (
    GraspDirection,
    GraspFace,
    GraspGenerator,
    ParallelGraspProperties,
)
from pyrobosim.utils.pose import Pose

# Create a grasp generator
properties = ParallelGraspProperties(
    max_width=0.15, depth=0.1, height=0.04, width_clearance=0.01, depth_clearance=0.01
)
gen = GraspGenerator(properties)


def test_cube_no_grasps():
    """
    If no grasps are specified, we should return no grasps.
    """
    object_dims = [0.1, 0.1, 0.1]
    grasps = gen.generate(
        object_dims, front_grasps=False, top_grasps=False, side_grasps=False
    )
    assert len(grasps) == 0


def test_cube_front_grasps():
    """
    Front grasps only for a cube object.
    Should return two perpendicular front grasps in the -X face
    """
    object_dims = [0.1, 0.1, 0.1]
    grasps = gen.generate(
        object_dims, front_grasps=True, top_grasps=False, side_grasps=False
    )
    assert len(grasps) == 2
    for grasp in grasps:
        assert grasp.face == GraspFace.FRONT
        assert grasp.direction == GraspDirection.X_NEG


def test_cube_top_grasps():
    """
    Top grasps only for a cube object.
    Should return two perpendicular front grasps in the -+Z face
    """
    object_dims = [0.1, 0.1, 0.1]
    grasps = gen.generate(
        object_dims, front_grasps=False, top_grasps=True, side_grasps=False
    )
    assert len(grasps) == 2
    for grasp in grasps:
        assert grasp.face == GraspFace.TOP
        assert grasp.direction == GraspDirection.Z_POS


def test_cube_side_grasps():
    """
    Side grasps only for a cube object.
    Should return two perpendicular front grasps on each side
    """
    object_dims = [0.1, 0.1, 0.1]
    grasps = gen.generate(
        object_dims, front_grasps=False, top_grasps=False, side_grasps=True
    )
    assert len(grasps) == 4
    assert grasps[0].face == GraspFace.RIGHT
    assert grasps[0].direction == GraspDirection.Y_NEG
    assert grasps[1].face == GraspFace.RIGHT
    assert grasps[1].direction == GraspDirection.Y_NEG
    assert grasps[2].face == GraspFace.LEFT
    assert grasps[2].direction == GraspDirection.Y_POS
    assert grasps[3].face == GraspFace.LEFT
    assert grasps[3].direction == GraspDirection.Y_POS


def test_cube_all_grasps():
    """
    All grasps for a cube object.
    Should return 8 total grasps
    """
    object_dims = [0.1, 0.1, 0.1]
    grasps = gen.generate(
        object_dims, front_grasps=True, top_grasps=True, side_grasps=True
    )
    assert len(grasps) == 8
    assert grasps[0].face == GraspFace.FRONT
    assert grasps[0].direction == GraspDirection.X_NEG
    assert grasps[1].face == GraspFace.FRONT
    assert grasps[1].direction == GraspDirection.X_NEG
    assert grasps[2].face == GraspFace.TOP
    assert grasps[2].direction == GraspDirection.Z_POS
    assert grasps[3].face == GraspFace.TOP
    assert grasps[3].direction == GraspDirection.Z_POS
    assert grasps[4].face == GraspFace.RIGHT
    assert grasps[4].direction == GraspDirection.Y_NEG
    assert grasps[5].face == GraspFace.RIGHT
    assert grasps[5].direction == GraspDirection.Y_NEG
    assert grasps[6].face == GraspFace.LEFT
    assert grasps[6].direction == GraspDirection.Y_POS
    assert grasps[7].face == GraspFace.LEFT
    assert grasps[7].direction == GraspDirection.Y_POS


def test_deep_cube_all_grasps():
    """
    All grasps for a deep object (large X dimension)
    Should return 5 grasps: 2 front, 1 top, 1 left, 1 right
    """
    object_dims = [0.5, 0.1, 0.1]
    grasps = gen.generate(
        object_dims, front_grasps=True, top_grasps=True, side_grasps=True
    )
    assert len(grasps) == 5
    assert grasps[0].face == GraspFace.FRONT
    assert grasps[0].direction == GraspDirection.X_NEG
    assert grasps[1].face == GraspFace.FRONT
    assert grasps[1].direction == GraspDirection.X_NEG
    assert grasps[2].face == GraspFace.TOP
    assert grasps[2].direction == GraspDirection.Z_POS
    assert grasps[3].face == GraspFace.RIGHT
    assert grasps[3].direction == GraspDirection.Y_NEG
    assert grasps[4].face == GraspFace.LEFT
    assert grasps[4].direction == GraspDirection.Y_POS


def test_wide_cube_all_grasps():
    """
    All grasps for a wide object (large Z dimension)
    Should return 5 grasps: 1 front, 1 top, 2 left, 2 right
    """
    object_dims = [0.1, 0.5, 0.1]
    grasps = gen.generate(
        object_dims, front_grasps=True, top_grasps=True, side_grasps=True
    )
    assert len(grasps) == 6
    assert grasps[0].face == GraspFace.FRONT
    assert grasps[0].direction == GraspDirection.X_NEG
    assert grasps[1].face == GraspFace.TOP
    assert grasps[1].direction == GraspDirection.Z_POS
    assert grasps[2].face == GraspFace.RIGHT
    assert grasps[2].direction == GraspDirection.Y_NEG
    assert grasps[3].face == GraspFace.RIGHT
    assert grasps[3].direction == GraspDirection.Y_NEG
    assert grasps[4].face == GraspFace.LEFT
    assert grasps[4].direction == GraspDirection.Y_POS
    assert grasps[5].face == GraspFace.LEFT
    assert grasps[5].direction == GraspDirection.Y_POS


def test_tall_cube_all_grasps():
    """
    All grasps for a tall object (large Z dimension)
    Should return 5 grasps: 1 front, 2 top, 1 left, 1 right
    """
    object_dims = [0.1, 0.1, 0.5]
    grasps = gen.generate(
        object_dims, front_grasps=True, top_grasps=True, side_grasps=True
    )
    assert len(grasps) == 5
    assert grasps[0].face == GraspFace.FRONT
    assert grasps[0].direction == GraspDirection.X_NEG
    assert grasps[1].face == GraspFace.TOP
    assert grasps[1].direction == GraspDirection.Z_POS
    assert grasps[2].face == GraspFace.TOP
    assert grasps[2].direction == GraspDirection.Z_POS
    assert grasps[3].face == GraspFace.RIGHT
    assert grasps[3].direction == GraspDirection.Y_NEG
    assert grasps[4].face == GraspFace.LEFT
    assert grasps[4].direction == GraspDirection.Y_POS


def test_large_cube_all_grasps():
    """
    All grasps enabled, but object is too large on all dimensions.
    Should return no grasps.
    """
    object_dims = [0.5, 0.5, 0.5]
    grasps = gen.generate(
        object_dims, front_grasps=True, top_grasps=True, side_grasps=True
    )
    assert len(grasps) == 0


def test_all_grasps_robot_in_front():
    """
    All grasps enabled and robot facing from the front.
    """
    object_dims = [0.1, 0.1, 0.1]
    object_pose = Pose(x=0.0, y=0.0, z=0.0)
    robot_pose = Pose(x=-1.0, y=0.0, z=0.0)
    grasps = gen.generate(
        object_dims,
        object_pose,
        robot_pose,
        front_grasps=True,
        top_grasps=True,
        side_grasps=True,
    )

    assert len(grasps) == 8
    for grasp in grasps:
        if grasp.face == GraspFace.FRONT:
            assert grasp.direction == GraspDirection.X_NEG
        elif grasp.face == GraspFace.TOP:
            assert grasp.direction == GraspDirection.Z_POS
        elif grasp.face == GraspFace.LEFT:
            assert grasp.direction == GraspDirection.Y_POS
        elif grasp.face == GraspFace.RIGHT:
            assert grasp.direction == GraspDirection.Y_NEG


def test_all_grasps_robot_in_right():
    """
    All grasps enabled and robot facing from the right.
    """
    object_dims = [0.1, 0.1, 0.1]
    object_pose = Pose(x=0.0, y=0.0, z=0.0)
    robot_pose = Pose(x=0.0, y=-1.0, z=0.0)
    grasps = gen.generate(
        object_dims,
        object_pose,
        robot_pose,
        front_grasps=True,
        top_grasps=True,
        side_grasps=True,
    )

    assert len(grasps) == 8
    for grasp in grasps:
        if grasp.face == GraspFace.FRONT:
            assert grasp.direction == GraspDirection.Y_NEG
        elif grasp.face == GraspFace.TOP:
            assert grasp.direction == GraspDirection.Z_POS
        elif grasp.face == GraspFace.LEFT:
            assert grasp.direction == GraspDirection.X_NEG
        elif grasp.face == GraspFace.RIGHT:
            assert grasp.direction == GraspDirection.X_POS


def test_all_grasps_robot_in_left():
    """
    All grasps enabled and robot facing from the left.
    """
    object_dims = [0.1, 0.1, 0.1]
    object_pose = Pose(x=0.0, y=0.0, z=0.0)
    robot_pose = Pose(x=0.0, y=1.0, z=0.0)
    grasps = gen.generate(
        object_dims,
        object_pose,
        robot_pose,
        front_grasps=True,
        top_grasps=True,
        side_grasps=True,
    )

    assert len(grasps) == 8
    for grasp in grasps:
        if grasp.face == GraspFace.FRONT:
            assert grasp.direction == GraspDirection.Y_POS
        elif grasp.face == GraspFace.TOP:
            assert grasp.direction == GraspDirection.Z_POS
        elif grasp.face == GraspFace.LEFT:
            assert grasp.direction == GraspDirection.X_POS
        elif grasp.face == GraspFace.RIGHT:
            assert grasp.direction == GraspDirection.X_NEG


def test_all_grasps_robot_in_back():
    """
    All grasps enabled and robot facing from the back.
    """
    object_dims = [0.1, 0.1, 0.1]
    object_pose = Pose(x=0.0, y=0.0, z=0.0)
    robot_pose = Pose(x=1.0, y=0.0, z=0.0)
    grasps = gen.generate(
        object_dims,
        object_pose,
        robot_pose,
        front_grasps=True,
        top_grasps=True,
        side_grasps=True,
    )

    assert len(grasps) == 8
    for grasp in grasps:
        if grasp.face == GraspFace.FRONT:
            assert grasp.direction == GraspDirection.X_POS
        elif grasp.face == GraspFace.TOP:
            assert grasp.direction == GraspDirection.Z_POS
        elif grasp.face == GraspFace.LEFT:
            assert grasp.direction == GraspDirection.Y_NEG
        elif grasp.face == GraspFace.RIGHT:
            assert grasp.direction == GraspDirection.Y_POS
