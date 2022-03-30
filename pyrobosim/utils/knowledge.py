import sys
import warnings
import numpy as np

from ..world.locations import Location, ObjectSpawn
from ..world.objects import Object

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


def query_to_entity(world, query_list, mode, resolution_strategy="first"):
    """ 
    Resolves a generic query list of strings to an entity 
    mode can be "location" or "object"
    """
    room = None
    named_location = None
    loc_category = None
    obj_category = None

    # Direct name search
    entity_list = []
    for elem in query_list:
        # First, directly search for location/object spawn names
        for loc in world.locations:
            if elem == loc.name:
                named_location = loc
            for spawn in loc.children:
                if elem == spawn.name:
                    named_location = spawn
        # Then, directly search for object names and get the location
        for obj in world.objects:
            if elem == obj.name:
                if mode == "location":
                    return obj.parent
                elif mode == "object":
                    return obj

    # Resolution search: Build a list of possible query terms
    for elem in query_list:
        if elem in world.get_room_names() and not room:
            room = elem
        if Location.metadata.has_category(elem) and not loc_category:
            loc_category = elem
        if Object.metadata.has_category(elem) and not obj_category:
            obj_category = elem

    # Special case: A room is selected purely by name
    if room and not named_location and not loc_category and not obj_category:
        return world.get_room_by_name(room)

    # If a named location is given, check that have an object category and filter by that.
    # Otherwise, just use the named location itself.
    if named_location is not None:
        if obj_category is None:
            return named_location
        else:
            if isinstance(named_location, ObjectSpawn):
                entity_list = named_location.children
            elif isinstance(named_location, Location):
                entity_list = []
                for spawn in named_location.children:
                    entity_list.extend(spawn.children)

        entity_list = [o for o in entity_list if o.category == obj_category]
        obj_candidate = apply_resolution_strategy(world, entity_list,
                                                  resolution_strategy=resolution_strategy)
        if not obj_candidate:
            warnings.warn(f"Could not resolve query {query_list}")
        else:
            if mode == "object":
                return obj_candidate
            elif mode == "location":
                return obj_candidate.parent

    # Resolve a location from any other query
    if obj_category or mode=="object":
        obj_candidate = resolve_to_object(world, category=obj_category, 
                                          location=loc_category, room=room,
                                          resolution_strategy=resolution_strategy)
        if not obj_candidate:
            warnings.warn(f"Could not resolve query {query_list}")
        else:
            if mode == "object":
                return obj_candidate
            elif mode == "location":
                return obj_candidate.parent
    else:
        loc_candidate = resolve_to_location(world, category=loc_category, room=room,
                                            resolution_strategy=resolution_strategy)
        if not loc_candidate:
            warnings.warn(f"Could not resolve query {query_list}")
        else:
            return loc_candidate
    

def resolve_to_location(world, category=None, room=None,
                        resolution_strategy="first", expand_locations=False):
    """ 
    Resolves a category/room combination to a location. 
      expand_locations will expand locations to individual object spawns
    """
    if room is None:
        room_name = None
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

        if category is None:
            possible_locations = [loc for loc in room.locations]
        else:
            possible_locations = [
                loc for loc in room.locations if loc.category == category]

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
    # Filter by category
    if category is None:
        possible_objects = world.get_objects()
    else:
        possible_objects = world.get_objects(category_list=[category])

    # Filter by room and/or location
    if room is not None:
        if isinstance(room, str):
            room_name = room
        else:
            room_name = room.name
        possible_objects = [
            o for o in possible_objects if o.parent.parent.parent.name == room_name]

    if location is not None:
        possible_objects = [o for o in possible_objects if 
            (o.parent == location or o.parent.category == location or o.parent.parent.name == location)]

    if ignore_grasped:
        if world.robot is not None:
            if world.robot.manipulated_object in possible_objects:
                possible_objects.remove(world.robot.manipulated_object)

    obj = apply_resolution_strategy(world, possible_objects, resolution_strategy)
    if not obj:
        warnings.warn(f"Could not resolve object query with category: {category}, location: {location}, room: {room}.")   
        return None
    return obj
