"""
Helper primitives for PDDLStream based planning.
"""

import numpy as np
from typing import Generator

from ...core.locations import ObjectSpawn
from ...core.objects import Object
from ...core.types import Entity
from ...manipulation.grasping import Grasp, GraspFace, GraspGenerator
from ...navigation.types import PathPlanner
from ...utils.path import Path
from ...utils.pose import Pose
from ...utils.polygon import sample_from_polygon, transform_polygon


def get_pick_place_cost(loc: ObjectSpawn, obj: Object) -> float:
    """
    Estimates a dummy pick / place cost for a specific location / object
    combination, which a constant value plus the height of the location and
    half height of the object.

    :param loc: Location where pick / place action occurs.
    :param obj: Object that is manipulated.
    :return: Cost of performing action.
    """
    return 0.5 + loc.height + (0.5 * obj.height)


def get_pick_place_at_pose_cost(
    loc: ObjectSpawn, obj: Object, p: Pose, pr: Pose
) -> float:
    """
    Estimates a dummy pick / place cost for a specific location / object
    combination, given the pose of the object and the robot.

    :param loc: Location where pick / place action occurs.
    :param obj: Object that is manipulated.
    :param p: Object pose.
    :param pr: Robot pose.
    :return: Cost of performing action.
    """
    return p.get_linear_distance(pr) + get_pick_place_cost(loc, obj)


def get_grasp_at_pose_cost(g: Grasp, pr: Pose) -> float:
    """
    Estimates the cost of grasping a specific object,
    given the grasp properties and the pose of the robot.

    :param g: Object grasp.
    :param pr: Robot pose.
    :return: Cost of performing action.
    """
    # Define cost for distance between robot and grasp pose
    assert isinstance(g.origin_wrt_world, Pose)
    distance_cost = g.origin_wrt_world.get_linear_distance(pr)

    # Define dummy costs for types of grasps
    if g.face == GraspFace.TOP:
        face_cost = 0.0
    elif g.face == GraspFace.FRONT:
        face_cost = 0.5
    else:
        face_cost = 1.0

    return distance_cost + face_cost


def get_detect_cost(loc: ObjectSpawn) -> float:
    """
    Estimates the cost of detecting objects at a location.

    :param loc: Location where the detect action occurs.
    :return: Cost of performing action.
    """
    return 0.5


def get_open_close_cost(loc: ObjectSpawn) -> float:
    """
    Estimates the detection cost of opening or closing a location.

    :param loc: Location where the open or close action occurs.
    :return: Cost of performing action.
    """
    return 1.0


def get_straight_line_distance(l1: Entity, l2: Entity) -> float:
    """
    Optimistically estimate the distance between two locations by getting the
    minimum straight-line distance between any two navigation poses.

    :param l1: First location.
    :param l2: Second location.
    :return: Straight-line distance between locations.
    """
    min_dist = float(np.inf)
    for p1 in l1.nav_poses:
        for p2 in l2.nav_poses:
            dist = p1.get_linear_distance(p2)
            if dist < min_dist:
                min_dist = dist
    return min_dist


def get_nav_poses(loc: Entity) -> list[tuple[Pose]]:
    """
    Gets a finite list of navigation poses for a specific location.

    :param loc: Location from which get navigation poses.
    :return: List of tuples containing navigation poses.
    """
    return [(p,) for p in loc.nav_poses]


def get_path_length(path: Path) -> float:
    """
    Simple wrapper to get the length of a path.

    :param path: Path from start to goal.
    :return: Length of the path.
    """
    return path.length


def sample_motion(
    planner: PathPlanner, p1: Pose, p2: Pose
) -> Generator[tuple[Path], None, None]:
    """
    Samples a feasible motion plan from a start to a goal pose.

    :param planner: Motion planner object.
    :param start: Start pose.
    :param goal: Goal pose.
    :return: Generator yielding tuple containing a path from start to goal.
    """
    while True:
        path = planner.plan(p1, p2)
        if (path is None) or (path.num_poses == 0):
            break
        yield (path,)


def sample_grasp_pose(
    grasp_gen: GraspGenerator,
    obj: Object,
    p_obj: Pose,
    p_robot: Pose,
    front_grasps: bool = True,
    top_grasps: bool = True,
    side_grasps: bool = False,
) -> Generator[tuple[Grasp], None, None]:
    """
    Samples feasible grasps for an object given its pose and the relative robot pose.

    :param grasp_gen: Grasp generator object
    :param obj: Target object
    :param p_obj: Object pose.
    :param p_robot: Robot pose.
    :param front_grasps: Enable front grasps
    :param top_grasps: Enable top grasps
    :param side_grasps: Enable side grasps
    :return: Generator yielding tuple containing a grasp
    """
    # Get the object cuboid pose assuming the object is at pose p_obj
    cuboid_pose = Pose.from_transform(
        np.matmul(
            obj.cuboid_pose.get_transform_matrix(),
            p_obj.get_transform_matrix(),
        )
    )

    # Generate grasps up front and yield them as requested
    grasps = grasp_gen.generate(
        obj.cuboid_dims,
        cuboid_pose,
        p_robot,
        front_grasps=front_grasps,
        top_grasps=top_grasps,
        side_grasps=side_grasps,
    )
    for g in grasps:
        yield (g,)


def sample_place_pose(
    loc: Entity, obj: Object, max_tries: int = 100
) -> Generator[tuple[Pose], None, None]:
    """
    Samples a feasible placement pose for an object at a specific location.

    :param loc: Location at which to place object.
    :param obj: Object to place.
    :param max_tries: Maximum samples to try before giving up.
    :return: Generator yielding tuple containing a placement pose
    """
    while True:
        is_valid_pose = False
        while not is_valid_pose:
            # Sample a pose
            x_sample, y_sample = sample_from_polygon(loc.polygon, max_tries=max_tries)
            if not x_sample or not y_sample:
                break  # If we can't sample a pose, we should give up.
            yaw_sample = np.random.uniform(-np.pi, np.pi)
            pose_sample = Pose(
                x=x_sample, y=y_sample, yaw=yaw_sample, z=loc.height + obj.height / 2.0
            )

            # Check that the object is inside the polygon.
            poly_sample = transform_polygon(obj.raw_collision_polygon, pose_sample)
            is_valid_pose = poly_sample.within(loc.polygon)
            if not is_valid_pose:
                continue  # If our sample is in collision, simply retry.

        yield (pose_sample,)


def test_collision_free(o1: Object, p1: Pose, o2: Object, p2: Pose) -> bool:
    """
    Test for collisions between two objects at specified poses.

    :param o1: First object
    :param p1: Pose of first object
    :param o2: Second object
    :param p2: Pose of second object
    :return: True if the two objects are collision free.
    """
    o1_poly = transform_polygon(o1.raw_collision_polygon, p1)
    o2_poly = transform_polygon(o2.raw_collision_polygon, p2)
    return not o1_poly.intersects(o2_poly)
