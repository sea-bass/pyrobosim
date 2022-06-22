"""
Mappings for PDDLStream functions, streams, and certificate tests.
"""

from pddlstream.language.function import FunctionInfo
from pddlstream.language.stream import StreamInfo
from pddlstream.language.generator import from_fn, from_gen_fn, from_list_fn

from . import primitives

def get_stream_map(world):
    """
    Returns a dictionary mapping stream names to actual function implementations.
    
    :return: The stream map dictionary.
    :rtype: dict(str, function) 
    """
    planner = world.robot.path_planner

    return {
        # Functions
        "Dist": primitives.get_straight_line_distance,
        "PickPlaceCost": primitives.get_pick_place_cost,
        "PathLength": primitives.get_path_length,
        # Streams
        "s-navpose": from_list_fn(primitives.get_nav_poses),
        "s-motion": from_gen_fn(
            lambda p1, p2: primitives.sample_motion(planner, p1, p2))
    }

def get_stream_info():
    """
    Returns a dictionary from stream name to StreamInfo altering how individual streams are handled.

    :return: The stream information dictionary.
    :rtype: dict(str, FunctionInfo/StreamInfo)
    """
    return {
        # Functions
        "Dist": FunctionInfo(opt_fn=primitives.get_straight_line_distance),
        "PickPlaceCost": FunctionInfo(opt_fn=primitives.get_pick_place_cost),
        "PathLength": FunctionInfo(opt_fn=primitives.get_path_length),
        # Streams
        "s-navpose": StreamInfo(eager=True),
        "s-motion": StreamInfo(eager=True)
    }
