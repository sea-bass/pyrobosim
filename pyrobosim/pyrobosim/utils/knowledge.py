"""
Utilities to reason about entities using world knowledge
(that is, metadata about locations and objects).
"""

import random
import sys

from ..core.hallway import Hallway
from ..core.locations import Location, ObjectSpawn
from ..core.robot import Robot
from ..core.room import Room
from ..core.types import Entity
from ..core.objects import Object
from ..core.world import World
from ..utils.graph_types import Node
from ..utils.logging import get_global_logger


def apply_resolution_strategy(
    entity_list: list[Entity | Node],
    resolution_strategy: str,
    robot: Robot | None = None,
) -> Entity | Node | None:
    """
    Accepts a list of entities in the world (e.g. rooms, objects, etc.) and
    applies a resolution strategy to get a single entity from that list that best
    meets one of the following criteria:

    - ``"first"`` : Return the first entity that meets this query
    - ``"random"`` : Return a random entity from all possible options
    - ``"nearest"`` : Return the nearest entity based on robot pose (So, a robot must exist in the world)

    :param entity_list: List of entities (e.g., rooms or objects)
    :param resolution_strategy: Resolution strategy to apply
    :param robot: If set to a Robot instance, uses that robot for resolution strategy.
    :return: The entity that meets the resolution strategy, or None.
    """
    if (entity_list is None) or len(entity_list) == 0:
        return None

    if resolution_strategy == "first":
        return entity_list[0]
    elif resolution_strategy == "random":
        return random.choice(entity_list)
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


def query_to_entity(
    world: World,
    query_list: str | list[str],
    mode: str,
    resolution_strategy: str = "first",
    robot: Robot | None = None,
) -> Entity | None:
    """
    Resolves a generic query list of strings to an entity
    mode can be "location" or "object"

    :param world: World model.
    :param query_list: List of query terms (e.g., "kitchen table apple").
        These can be specified as a list of strings, or as a single space-separated string.
    :param mode: Can be either "location" or "object".
    :param resolution_strategy: Resolution strategy to apply (see :func:`apply_resolution_strategy`)
    :param robot: If set to a Robot instance, uses that robot for resolution strategy.
    :return: The entity that meets the mode and resolution strategy, or None.
    """
    named_room: Room | None = None
    named_location: Entity | None = None
    loc_category: str | None = None
    obj_category: str | None = None

    # Process the input and convert it to a list.
    if query_list is None:
        query_list = []
    elif isinstance(query_list, str):
        query_list = [elem for elem in query_list.split(" ") if elem]

    possible_objects: list[Object] = []
    if robot is None:
        possible_objects = world.get_objects()
    else:
        possible_objects = robot.get_known_objects()

    # Direct name search
    entity_list: list[Entity | Node] = []
    resolved_queries = set()
    for elem in query_list:
        # Directly search for entity names
        for room in world.rooms:
            if elem == room.name:
                named_room = room
                resolved_queries.add(elem)
        for loc in world.locations:
            if elem == loc.name:
                named_location = loc
                resolved_queries.add(elem)
            for spawn in loc.children:
                if elem == spawn.name:
                    named_location = spawn
                    resolved_queries.add(elem)
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

    # Build a list of possible query terms
    for elem in query_list:
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
        and named_room
        and not named_location
        and not loc_category
        and not obj_category
    ):
        return named_room

    # If a named location is given, check that have an object category and filter by that.
    # Otherwise, just use the named location itself.
    if named_location is not None:
        if obj_category is None and mode == "location":
            return named_location
        else:
            if isinstance(named_location, ObjectSpawn):
                entity_list = [child for child in named_location.children]
            elif isinstance(named_location, Location):
                entity_list = []
                for spawn in named_location.children:
                    entity_list.extend(spawn.children)

        entity_list = [
            obj
            for obj in entity_list
            if (isinstance(obj, Entity) and (obj.category == obj_category))
        ]
        obj_candidate = apply_resolution_strategy(
            entity_list, resolution_strategy=resolution_strategy, robot=robot
        )
        assert not isinstance(obj_candidate, Node)
        if obj_candidate is not None:
            if mode == "object":
                return obj_candidate
            elif mode == "location":
                return obj_candidate.parent

    if (
        (mode == "object")
        and (loc_category is None)
        and (robot is not None)
        and (robot.location is not None)
    ):
        loc_category = robot.location.name

    # Resolve a location from any other query
    if (obj_category is not None) or (mode == "object"):
        obj_candidate = resolve_to_object(
            world,
            category=obj_category,
            location=loc_category,
            room=named_room,
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
            room=named_room,
            resolution_strategy=resolution_strategy,
            robot=robot,
        )
        if loc_candidate is not None:
            return loc_candidate

    get_global_logger().warning(f"Could not resolve query {query_list}")
    return None


def resolve_to_location(
    world: World,
    category: str | None = None,
    room: Room | str | None = None,
    resolution_strategy: str = "first",
    robot: Robot | None = None,
    expand_locations: bool = False,
) -> Location | ObjectSpawn | None:
    """
    Resolves a category/room query combination to a single specific location.

    :param world: World model.
    :param category: Location category (e.g. "table")
    :param room: Room or room name to search in (e.g. "kitchen")
    :param resolution_strategy: Resolution strategy to apply (see :func:`apply_resolution_strategy`)
    :param robot: If set to a Robot instance, uses that robot for resolution strategy.
    :param expand_locations: If True, expands location to individual object spawns.
    :return: The location or object spawn that meets the category and/or room filters, or None.
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

        if not isinstance(room, Room):
            return None

        if category is None:
            possible_locations = [loc for loc in room.locations]
        else:
            possible_locations = [
                loc for loc in room.locations if loc.category == category
            ]

    # Optionally expand locations to their individual object spawns
    if expand_locations:
        expanded_locations: list[Entity | Node] = []
        for loc in possible_locations:
            if isinstance(loc, Location):
                expanded_locations.extend(loc.children)
            else:
                expanded_locations.append(loc)
    else:
        expanded_locations = [loc for loc in possible_locations]

    resolved_loc = apply_resolution_strategy(
        expanded_locations, resolution_strategy, robot=robot
    )
    if resolved_loc is None:
        get_global_logger().warning(
            f"Could not resolve location query with category: {category}, room: {room_name}."
        )
        return None
    assert isinstance(resolved_loc, (Location, ObjectSpawn))
    return resolved_loc


def resolve_to_object(
    world: World,
    category: str | None = None,
    location: Location | ObjectSpawn | str | None = None,
    room: Room | str | None = None,
    resolution_strategy: str = "first",
    robot: Robot | None = None,
    ignore_grasped: bool = True,
) -> Object | None:
    """
    Resolves a category/location/room query to an object.

    :param world: World model.
    :param category: Object category (e.g. "apple")
    :param location: Location category search in (e.g. "table")
    :param room: Room or room name to search in (e.g. "kitchen")
    :param resolution_strategy: Resolution strategy to apply (see :func:`apply_resolution_strategy`)
    :param robot: If set to a Robot instance, uses that robot for resolution strategy.
    :param ignore_grasped: If True, ignores the current manipulated object.
    :return: The object that meets the category, location, and/or room filters, or None.
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
        possible_objects = [obj for obj in possible_objects if obj.category == category]

    # Filter by room and/or location
    if room is not None:
        if isinstance(room, str):
            room_name = room
        else:
            room_name = room.name
        possible_objects = [
            obj for obj in possible_objects if obj.get_room_name() == room_name
        ]

    if location is not None:
        possible_objects = [
            obj
            for obj in possible_objects
            if (
                # Verify the object's parent is not a robot before performing further checks
                obj.parent is not None
                and (
                    obj.parent == location
                    or obj.parent.name == location
                    or obj.parent.parent == location
                    or obj.parent.category == location
                    or (
                        (obj.parent.parent is not None)
                        and (
                            (obj.parent.parent.name == location)
                            or (obj.parent.parent.category == location)
                        )
                    )
                )
            )
        ]

    resolved_obj = apply_resolution_strategy(
        [obj for obj in possible_objects], resolution_strategy, robot=robot
    )
    if resolved_obj is None:
        get_global_logger().warning(
            f"Could not resolve object query with category: {category}, location: {location}, room: {room}."
        )
        return None
    assert isinstance(resolved_obj, Object)
    return resolved_obj


def graph_node_from_entity(
    world: World,
    entity_query: Entity | Node | str,
    resolution_strategy: str = "nearest",
    robot: Robot | None = None,
) -> Node | None:
    """
    Gets a graph node from an entity query, which could be any combination of
    room, hallway, location, object spawn, or object in the world, as well as
    their respective categories.
    For more information on the inputs, refer to the :func:`pyrobosim.utils.knowledge.query_to_entity` function.

    :param entity_query: The entity from which to get a graph node.
    :param resolution_strategy: Resolution strategy to apply
    :param robot: If set to a Robot instance, uses that robot for resolution strategy.
    :return: A graph node for the entity that meets the resolution strategy, or None.
    """
    graph_nodes: list[Entity | Node] = []

    if isinstance(entity_query, Node):
        return entity_query
    elif isinstance(entity_query, str):
        # Try resolve an entity based on its name. If that fails, we assume it must be a category,
        # so try resolve it to a location or to an object by category.
        entity = world.get_entity_by_name(entity_query)
        if entity is None:
            entity = resolve_to_location(
                world,
                category=entity_query,
                expand_locations=True,
                resolution_strategy=resolution_strategy,
                robot=robot,
            )
        if entity is None:
            entity = resolve_to_object(
                world,
                category=entity_query,
                resolution_strategy=resolution_strategy,
                robot=robot,
                ignore_grasped=True,
            )
    else:
        entity = entity_query

    if isinstance(entity, (ObjectSpawn, Room)):
        graph_nodes = [node for node in entity.graph_nodes]
    elif isinstance(entity, Hallway):
        graph_nodes = [entity.graph_nodes[0], entity.graph_nodes[-1]]

        # Special rule: If all the hallways connected to the room are closed, and the robot is not in the room or at the hallway,
        # remove the graph node from consideration.
        if (robot is not None) and (robot.location != entity):
            robot_in_start_room = entity.room_start.is_collision_free(robot.get_pose())
            if not robot_in_start_room:
                room_accessible = False
                for hall in world.get_hallways_attached_to_room(entity.room_start):
                    if hall.is_open:
                        room_accessible = True
                        break
                if not room_accessible:
                    graph_nodes.remove(entity.graph_nodes[0])

            robot_in_end_room = entity.room_end.is_collision_free(robot.get_pose())
            if not robot_in_end_room:
                room_accessible = False
                for hall in world.get_hallways_attached_to_room(entity.room_end):
                    if hall.is_open:
                        room_accessible = True
                        break
                if not room_accessible:
                    graph_nodes.remove(entity.graph_nodes[-1])

    elif isinstance(entity, Object) and (entity.parent is not None):
        graph_nodes = [node for node in entity.parent.graph_nodes]
    elif isinstance(entity, Location):
        graph_nodes = [node for node in entity.children[0].graph_nodes]
    else:
        world.logger.warning(f"Cannot get graph node from {entity}")
        return None

    # Select a graph node using the same resolution strategy.
    graph_node = apply_resolution_strategy(
        graph_nodes, resolution_strategy, robot=robot
    )
    if graph_node is None:
        return None
    assert isinstance(graph_node, Node)
    return graph_node
