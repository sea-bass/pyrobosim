"""
Default mappings for PDDLStream functions, streams, and certificate tests that represent
Task and Motion Planning for pick-and-place applications with a mobile manipulator.
"""

from pddlstream.language.stream import StreamInfo
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_test

from . import primitives


def get_stream_map(world, robot):
    """
    Returns a dictionary mapping stream names to function implementations.

    :param world: World object for planning.
    :type world: :class:`pyrobosim.core.world.World`
    :param robot: Robot object for planning.
    :type robot: :class:`pyrobosim.core.robot.Robot`
    :return: The stream map dictionary.
    :rtype: dict(str, function)
    """
    planner = robot.path_planner
    grasp_gen = robot.grasp_generator

    return {
        # Functions
        "Dist": primitives.get_straight_line_distance,
        "PickPlaceCost": primitives.get_pick_place_cost,
        "PickPlaceAtPoseCost": primitives.get_pick_place_at_pose_cost,
        "GraspAtPoseCost": primitives.get_grasp_at_pose_cost,
        "DetectCost": primitives.get_detect_cost,
        "OpenCloseCost": primitives.get_open_close_cost,
        "PathLength": primitives.get_path_length,
        # Streams (that sample)
        "s-navpose": from_list_fn(primitives.get_nav_poses),
        "s-motion": from_gen_fn(
            lambda p1, p2: primitives.sample_motion(planner, p1, p2)
        ),
        "s-grasp": from_gen_fn(
            lambda obj, p_obj, p_robot: primitives.sample_grasp_pose(
                grasp_gen,
                obj,
                p_obj,
                p_robot,
                front_grasps=True,
                top_grasps=True,
                side_grasps=False,
            )
        ),
        "s-place": from_gen_fn(
            lambda loc, obj: primitives.sample_place_pose(
                loc,
                obj,
                max_tries=world.max_object_sample_tries,
            )
        ),
        # Streams (no sampling, just testing)
        "t-collision-free": from_test(primitives.test_collision_free),
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
        # Nav poses can be eagerly sampled since locations don't move.
        "s-navpose": StreamInfo(eager=True),
        # Other streams cannot be eagerly sampled as they depend on the
        # instantaneous pose of entities in the world during planning.
        "s-motion": StreamInfo(eager=False),
        "s-grasp": StreamInfo(eager=False),
        "s-place": StreamInfo(eager=False),
        # Streams (no sampling, just testing)
        "t-collision-free": StreamInfo(eager=False, negate=True),
    }
