"""
Mappings for PDDLStream functions, streams, and certificate tests.
"""

from pddlstream.language.function import FunctionInfo

from . import primitives

def get_stream_map():
    """
    Returns a dictionary mapping stream names to actual function implementations.
    
    :return: The stream map dictionary.
    :rtype: dict(str, function) 
    """
    return {
        # Functions
        "Dist": primitives.get_straight_line_distance,
        "PickPlaceCost": primitives.get_pick_place_cost
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
        "PickPlaceCost": FunctionInfo(opt_fn=primitives.get_pick_place_cost)
    }
