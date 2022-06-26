"""
Helper primitives for PDDLStream based planning.
"""

import numpy as np

from ...utils.pose import Pose
from ...utils.polygon import inflate_polygon, sample_from_polygon, transform_polygon


def get_pick_place_cost(loc, obj):
    """
    Estimates a dummy pick / place cost for a specific location / object
    combination, which a constant value plus the height of the location and
    half height of the object.

    :param loc: Location where pick / place action occurs.
    :type loc: Location
    :param obj: Object that is manipulated.
    :type obj: Object
    :return: Cost of performing action.
    :rtype: float
    """
    return 0.5 + loc.height + (0.5 * obj.height)


def get_pick_place_at_pose_cost(loc, obj, p, pr):
    """
    Estimates a dummy pick / place cost for a specific location / object
    combination, given the pose of the object and the robot.

    :param loc: Location where pick / place action occurs.
    :type loc: Location
    :param obj: Object that is manipulated.
    :type obj: Object
    :param p: Object pose.
    :type p: :class:`pyrobosim.utils.pose.Pose`
    :param pr: Robot pose.
    :type pr: :class:`pyrobosim.utils.pose.Pose`
    :return: Cost of performing action.
    :rtype: float
    """
    return p.get_linear_distance(pr) + get_pick_place_cost(loc, obj)


def get_straight_line_distance(l1, l2):
    """
    Optimistically estimate the distance between two locations by getting the
    minimum straight-line distance between any two navigation poses.

    :param l1: First location.
    :type l1: Location
    :param l2: Second location.
    :type l2: Location
    :return: Straight-line distance between locations.
    :rtype: float
    """
    min_dist = np.inf
    for p1 in l1.nav_poses:
        for p2 in l2.nav_poses:
            dist = p1.get_linear_distance(p2)
            if dist < min_dist:
                min_dist = dist
    return min_dist


def get_nav_poses(loc):
    """
    Gets a finite list of navigation poses for a specific location.

    :param loc: Location from which get navigation poses.
    :type loc: Location
    :return: List of tuples containing navigation poses.
    :rtype: list[tuple]
    """
    return [(p,) for p in loc.nav_poses]


def get_path_length(path):
    """
    Simple wrapper to get the length of a path.

    :param path: Path from start to goal.
    :type path: :class:`pyrobosim.utils.motion.Path`
    :return: Length of the path.
    :rtype: float
    """
    return path.length


def sample_motion(planner, p1, p2):
    """
    Samples a feasible motion plan from a start to a goal pose.

    :param planner: Motion planner object.
    :type planner: Planner
    :param start: Start pose.
    :type start: :class:`pyrobosim.utils.pose.Pose`
    :param goal: Goal pose.
    :type goal: :class:`pyrobosim.utils.pose.Pose`
    :return: Generator yielding tuple containing a path from start to goal
    :rtype: generator[tuple[:class:`pyrobosim.utils.motion.Path`]]
    """
    while True:
        path = planner.plan(p1, p2)
        if path is None or path.num_poses == 0:
            break
        yield (path,)


def sample_place_pose(loc, obj, padding=0.0, max_tries=100):
    """
    Samples a feasible placement pose for an object at a specific location.

    :param loc: Location at which to place object.
    :type loc: Location
    :param obj: Object to place.
    :type obj: Object
    :param padding: Padding around edge of location polygon for collision checking.
    :type padding: float, optional
    :param max_tries: Maximum samples to try before giving up.
    :type max_tries: int, optional
    :return: Generator yielding tuple containing a placement pose
    :rtype: generator[tuple[:class:`pyrobosim.utils.pose.Pose`]]
    """
    obj_poly = inflate_polygon(obj.raw_polygon, padding)
    while True:
        is_valid_pose = False
        while not is_valid_pose:
            # Sample a pose
            x_sample, y_sample = sample_from_polygon(loc.polygon, max_tries=max_tries)
            if not x_sample or not y_sample:
                break  # If we can't sample a pose, we should give up.
            yaw_sample = np.random.uniform(-np.pi, np.pi)
            pose_sample = Pose(
                x=x_sample, y=y_sample, yaw=yaw_sample,
                z=loc.height + obj.height / 2.0
            )

            # Check that the object is inside the polygon.
            poly_sample = transform_polygon(obj_poly, pose_sample)
            is_valid_pose = poly_sample.within(loc.polygon)
            if not is_valid_pose:
                continue  # If our sample is in collision, simply retry.

        yield (pose_sample,)


def test_collision_free(o1, p1, o2, p2, padding=0.0):
    """
    Test for collisions between two objects at specified poses and padding.

    :param o1: First object
    :type o1: :class:`pyrobosim.core.objects.Object`
    :param p1: Pose of first object
    :type p1: :class:`pyrobosim.utils.pose.Pose`
    :param o2: Second object
    :type o2: :class:`pyrobosim.core.objects.Object`
    :param p2: Pose of second object
    :type p2: :class:`pyrobosim.utils.pose.Pose`
    :param padding: Padding for collision checking.
    :type padding: float, optional
    :return: True if the two objects are collision free.
    :rtype: bool
    """
    o1_poly = transform_polygon(inflate_polygon(o1.raw_polygon, padding), p1)
    o2_poly = transform_polygon(o2.raw_polygon, p2)
    return not o1_poly.intersects(o2_poly)
