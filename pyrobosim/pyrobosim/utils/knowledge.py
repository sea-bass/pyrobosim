"""
Utilities to reason about entities using world knowledge
(that is, metadata about locations and objects).
"""

import sys
import numpy as np

from ..utils.logging import get_global_logger


def apply_resolution_strategy(entity_list, resolution_strategy, robot=None):
    """
    Accepts a list of entities in the world (e.g. rooms, objects, etc.) and
    applies a resolution strategy to get a single entity from that list that best
    meets one of the following criteria:

    - ``"first"`` : Return the first entity that meets this query
    - ``"random"`` : Return a random entity from all possible options
    - ``"nearest"`` : Return the nearest entity based on robot pose (So, a robot must exist in the world)

    :param world: World model.
    :type world: :class:`pyrobosim.core.world.World`
    :param entity_list: List of entities (e.g., rooms or objects)
    :type entity_list: list[Entity]
    :param resolution_strategy: Resolution strategy to apply
    :type resolution_strategy: str
    :param robot: If set to a Robot instance, uses that robot for resolution strategy.
    :type robot: :class:`pyrobosim.core.robot.Robot`, optional
    :return: The entity that meets the resolution strategy, or None.
    :rtype: Entity
    """
    if entity_list is None or len(entity_list) == 0:
        return None

    if resolution_strategy == "first":
        return entity_list[0]
    elif resolution_strategy == "random":
        return np.random.choice(entity_list)
    elif resolution_strategy == "nearest":
        if not robot:
            get_global_logger().warning(
                "Cannot apply nearest resolution strategy without a robot!"
            )
            return None
        nearest_dist = sys.float_info.max
        nearest_entity = None
        robot_pose = robot.get_pose()
        for entity in entity_list:
            dist = robot_pose.get_linear_distance(entity.pose)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_entity = entity
        return nearest_entity
    else:
        get_global_logger().warning(
            f"Invalid resolution strategy: {resolution_strategy}"
        )
        return None


def query_to_entity(world, query_list, mode, resolution_strategy="first", robot=None):
    """
    Resolves a generic query list of strings to an entity
    mode can be "location" or "object"

    :param world: World model.
    :type world: :class:`pyrobosim.core.world.World`
    :param query_list: List of query terms (e.g., "kitchen table apple").
        These can be specified as a list of strings, or as a single space-separated string.
    :type query_list: str or list[str]
    :param mode: Can be either "location" or "object".
    :type mode: str
    :param resolution_strategy: Resolution strategy to apply (see :func:`apply_resolution_strategy`)
    :type resolution_strategy: str
    :param robot: If set to a Robot instance, uses that robot for resolution strategy.
    :type robot: :class:`pyrobosim.core.robot.Robot`, optional
    :return: The entity that meets the mode and resolution strategy, or None.
    :rtype: Entity
    """
    from ..core.locations import Location, ObjectSpawn
    from ..core.objects import Object

    room = None
    named_location = None
    loc_category = None
    obj_category = None

    # Process the input and convert it to a list.
    if query_list is None:
        query_list = []
    elif isinstance(query_list, str):
        query_list = [elem for elem in query_list.split(" ") if elem]

    if robot is None:
        possible_objects = world.get_objects()
    else:
        possible_objects = robot.get_known_objects()

    # Direct name search
    entity_list = []
    resolved_queries = set()
    for elem in query_list:
        # First, directly search for location/object spawn names
        for loc in world.locations:
            if elem == loc.name:
                named_location = loc
                resolved_queries.add(elem)
            for spawn in loc.children:
                if elem == spawn.name:
                    named_location = spawn
                    resolved_queries.add(elem)
        # Also search for hallway names
        for hall in world.hallways:
            if (elem == hall.name) or (elem == hall.reversed_name):
                named_location = hall
                resolved_queries.add(elem)
        # Then, directly search for object names and get the location
        for obj in possible_objects:
            if elem == obj.name:
                if mode == "location":
                    return obj.parent
                elif mode == "object":
                    return obj

    # Resolution search: Build a list of possible query terms
    for elem in query_list:
        if elem in world.get_room_names() and not room:
            room = elem
            resolved_queries.add(elem)
        if Location.metadata.has_category(elem) and not loc_category:
            loc_category = elem
            resolved_queries.add(elem)
        if Object.metadata.has_category(elem) and not obj_category:
            obj_category = elem
            resolved_queries.add(elem)

    # If any query elements are unaccounted for (for example, in the case of a nonexistent object or garbage value),
    # then entity resolution should fail.
    for elem in query_list:
        if elem not in resolved_queries:
            get_global_logger().warning(
                f"Did not resolve query element {elem}. Returning None."
            )
            return None

    # Special case: A room is selected purely by name
    if (
        mode == "location"
        and room
        and not named_location
        and not loc_category
        and not obj_category
    ):
        return world.get_room_by_name(room)

    # If a named location is given, check that have an object category and filter by that.
    # Otherwise, just use the named location itself.
    if named_location is not None:
        if obj_category is None and mode == "location":
            return named_location
        else:
            if isinstance(named_location, ObjectSpawn):
                entity_list = named_location.children
            elif isinstance(named_location, Location):
                entity_list = []
                for spawn in named_location.children:
                    entity_list.extend(spawn.children)

        entity_list = [obj for obj in entity_list if obj.category == obj_category]
        obj_candidate = apply_resolution_strategy(
            entity_list, resolution_strategy=resolution_strategy, robot=robot
        )
        if obj_candidate is not None:
            if mode == "object":
                return obj_candidate
            elif mode == "location":
                return obj_candidate.parent

    if mode == "object" and loc_category is None and robot:
        loc_category = robot.location.name

    # Resolve a location from any other query
    if obj_category or mode == "object":
        obj_candidate = resolve_to_object(
            world,
            category=obj_category,
            location=loc_category,
            room=room,
            resolution_strategy=resolution_strategy,
            robot=robot,
        )
        if obj_candidate is not None:
            if mode == "object":
                return obj_candidate
            elif mode == "location":
                return obj_candidate.parent
    else:
        loc_candidate = resolve_to_location(
            world,
            category=loc_category,
            room=room,
            resolution_strategy=resolution_strategy,
            robot=robot,
        )
        if loc_candidate is not None:
            return loc_candidate

    get_global_logger().warning(f"Could not resolve query {query_list}")
    return None


def resolve_to_location(
    world,
    category=None,
    room=None,
    resolution_strategy="first",
    robot=None,
    expand_locations=False,
):
    """
    Resolves a category/room query combination to a single specific location.

    :param world: World model.
    :type world: :class:`pyrobosim.core.world.World`
    :param category: Location category (e.g. "table")
    :type category: str, optional
    :param room: Room name to search in (e.g. "kitchen")
    :type room: str, optional
    :param resolution_strategy: Resolution strategy to apply (see :func:`apply_resolution_strategy`)
    :type resolution_strategy: str
    :param robot: If set to a Robot instance, uses that robot for resolution strategy.
    :type robot: :class:`pyrobosim.core.robot.Robot`, optional
    :param expand_locations: If True, expands location to individual object spawns.
    :type expand_locations: bool
    :return: The location or object spawn that meets the category and/or room filters, or None.
    :rtype: :class:`pyrobosim.core.locations.Location`/:class:`pyrobosim.core.locations.ObjectSpawn`
    """
    from ..core.locations import Location

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
                loc for loc in room.locations if loc.category == category
            ]

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

    loc = apply_resolution_strategy(
        expanded_locations, resolution_strategy, robot=robot
    )
    if not loc:
        get_global_logger().warning(
            f"Could not resolve location query with category: {category}, room: {room_name}."
        )
        return None
    return loc


def resolve_to_object(
    world,
    category=None,
    location=None,
    room=None,
    resolution_strategy="first",
    robot=None,
    ignore_grasped=True,
):
    """
    Resolves a category/location/room query to an object.

    :param world: World model.
    :type world: :class:`pyrobosim.core.world.World`
    :param category: Object category (e.g. "apple")
    :type category: str, optional
    :param location: Location category search in (e.g. "table")
    :type location: str, optional
    :param room: Room name to search in (e.g. "kitchen")
    :type room: str, optional
    :param resolution_strategy: Resolution strategy to apply (see :func:`apply_resolution_strategy`)
    :type resolution_strategy: str
    :param robot: If set to a Robot instance, uses that robot for resolution strategy.
    :type robot: :class:`pyrobosim.core.robot.Robot`, optional
    :param ignore_grasped: If True, ignores the current manipulated object.
    :type ignore_grasped: bool
    :return: The object that meets the category, location, and/or room filters, or None.
    :rtype: :class:`pyrobosim.core.objects.Object`
    """
    # If a robot is not specified, start with the full list of objects.
    # Otherwise, remove any objects manipulated or unobserved by the robot.
    if robot is None:
        possible_objects = world.get_objects()
    else:
        possible_objects = robot.get_known_objects()

        if ignore_grasped and robot.manipulated_object in possible_objects:
            possible_objects.remove(robot.manipulated_object)

    # Filter by category
    if category is not None:
        possible_objects = [obj for obj in possible_objects if obj.category in category]

    # Filter by room and/or location
    if room is not None:
        if isinstance(room, str):
            room_name = room
        else:
            room_name = room.name
        possible_objects = [
            obj
            for obj in possible_objects
            if obj.parent.parent.parent.name == room_name
        ]

    if location is not None:
        possible_objects = [
            obj
            for obj in possible_objects
            if (
                # Check whether the parent is a robot performing further checks
                hasattr(obj.parent, "parent")
                and (
                    obj.parent == location
                    or obj.parent.name == location
                    or obj.parent.parent == location
                    or obj.parent.parent.name == location
                    or obj.parent.category == location
                    or obj.parent.parent.category == location
                )
            )
        ]

    obj = apply_resolution_strategy(possible_objects, resolution_strategy, robot=robot)
    if not obj:
        get_global_logger().warning(
            f"Could not resolve object query with category: {category}, location: {location}, room: {room}."
        )
        return None
    return obj
