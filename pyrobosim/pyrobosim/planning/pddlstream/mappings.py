"""
Mappings for PDDLStream functions, streams, and certificate tests.
"""

from pddlstream.language.stream import StreamInfo
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_test

from . import primitives


def get_stream_map(world):
    """
    Returns a dictionary mapping stream names to function implementations.

    :return: The stream map dictionary.
    :rtype: dict(str, function)
    """
    planner = world.robot.path_planner

    return {
        # Functions
        "Dist": primitives.get_straight_line_distance,
        "PickPlaceCost": primitives.get_pick_place_cost,
        "PickPlaceAtPoseCost": primitives.get_pick_place_at_pose_cost,
        "PathLength": primitives.get_path_length,
        # Streams (that sample)
        "s-navpose": from_list_fn(primitives.get_nav_poses),
        "s-motion": from_gen_fn(
            lambda p1, p2: primitives.sample_motion(planner, p1, p2)
        ),
        "s-place": from_gen_fn(
            lambda loc, obj: primitives.sample_place_pose(
                loc,
                obj,
                padding=world.object_radius,
                max_tries=world.max_object_sample_tries,
            )
        ),
        # Streams (no sampling, just testing)
        "t-collision-free": from_test(
            lambda o1, p1, o2, p2: primitives.test_collision_free(
                o1, p1, o2, p2, padding=world.object_radius
            )
        ),
    }


def get_stream_info():
    """
    Returns a dictionary from stream name to StreamInfo altering how
    individual streams are handled.

    :return: The stream information dictionary.
    :rtype: dict(str, FunctionInfo/StreamInfo)
    """
    return {
        # Streams (that sample)
        "s-navpose": StreamInfo(eager=False),
        "s-motion": StreamInfo(eager=False),
        "s-place": StreamInfo(eager=False),
        # Streams (no sampling, just testing)
        "t-collision-free": StreamInfo(eager=False, negate=True),
    }
