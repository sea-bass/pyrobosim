import sys
import warnings
import numpy as np

from ..world.locations import Location

"""
Utilities to reason about entities using world knowledge
(that is, metadata about locations and objects)
"""

def apply_resolution_strategy(world, entity_list, resolution_strategy):
    """
    Applies a resolution strategy on a list of entities based on one of the 
    following criteria:
      "first"   : Return the first entity that meets this query
      "random"  : Return a random entity from all possible options 
      "nearest" : Return the nearest entity based on robot pose
                    (So, a robot must exist in the world)
    """
    if entity_list is None or len(entity_list) == 0:
        return None

    if resolution_strategy == "first":
        return entity_list[0]
    elif resolution_strategy == "random":
        return np.random.choice(entity_list)
    elif resolution_strategy == "nearest":
        if not world.has_robot:
            warnings.warn("Cannot apply nearest resolution strategy without a robot!")
            return None
        nearest_dist = sys.float_info.max
        nearest_entity = None
        robot_pose = world.robot.pose
        for entity in entity_list:
            dist = robot_pose.get_linear_distance(entity.pose)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_entity = entity
        return nearest_entity
    else:
        warnings.warn(f"Invalid resolution strategy: {resolution_strategy}")
        return None


def resolve_to_location(world, category=None, room=None,
                        resolution_strategy="first", expand_locations=False):
    """ 
    Resolves a query to a location 
      expand_locations will expand locations to individual object spawns
    """
    if room is None:
        if category is None:
            possible_locations = world.get_locations()
        else:
            possible_locations = world.get_locations(category_list=[category])
    else:
        if isinstance(room, str):
            room_name = room
            room = world.get_room_by_name(room)
        else:
            room_name = room.name
        possible_locations = [
            loc for loc in room.locations if loc.category is category]

    # Optionally expand locations to their individual object spawns
    if expand_locations:
        expanded_locations = []
        for loc in possible_locations:
            if isinstance(loc, Location):
                expanded_locations.extend(loc.children)
            else:
                expanded_locations.append(loc)
    else:
        expanded_locations = possible_locations

    loc = apply_resolution_strategy(world, expanded_locations, resolution_strategy)
    if not loc:
        warnings.warn(f"Could not resolve location query with category: {category}, room: {room_name}.")
        return None
    return loc


def resolve_to_object(world, category=None, location=None, room=None, 
                      resolution_strategy="first", ignore_grasped=True):
    """ 
    Resolves a query to an object 
      ignore_grasped will ignore any currently manipulated object
    """
    if category is None:
        possible_objects = world.get_objects()
    else:
        possible_objects = world.get_objects(category_list=[category])

    if ignore_grasped:
        if world.robot is not None:
            if world.robot.manipulated_object in possible_objects:
                possible_objects.remove(world.robot.manipulated_object)

    obj = apply_resolution_strategy(world, possible_objects, resolution_strategy)
    if not obj:
        warnings.warn(f"Could not resolve object query with category: {category}, location: {location}, room: {room}.")   
        return None
    return obj
