""" Main file containing the core world modeling tools. """

import itertools
import numpy as np

from .hallway import Hallway
from .locations import Location, ObjectSpawn
from .objects import Object
from .room import Room
from .robot import Robot
from ..planning.actions import ExecutionResult, ExecutionStatus
from ..utils.general import InvalidEntityCategoryException
from ..utils.logging import create_logger
from ..utils.pose import Pose
from ..utils.knowledge import (
    apply_resolution_strategy,
    resolve_to_location,
    resolve_to_object,
)
from ..utils.polygon import sample_from_polygon, transform_polygon
from ..utils.search_graph import Node


class World:
    """Core world modeling class."""

    def __init__(
        self, name="world", inflation_radius=0.0, object_radius=0.0375, wall_height=2.0
    ):
        """
        Creates a new world model instance.

        :param name: Name of world model.
        :type name: str
        :param inflation_radius: Inflation radius around entities (locations, walls, etc.), in meters.
        :type inflation_radius: float, optional
        :param object_radius: Buffer radius around objects for collision checking, in meters.
        :type object_radius: float, optional
        :param wall_height: Height of walls, in meters, for 3D model generation.
        :type wall_height: float, optional
        """
        self.name = name
        self.wall_height = wall_height
        self.source_yaml = None
        self.source_yaml_file = None
        self.logger = create_logger(self.name)

        # Connected apps
        self.has_gui = False
        self.gui = None
        self.has_ros_node = False
        self.ros_node = False

        # World entities (robots, locations, objects, etc.)
        self.robots = []
        self.name_to_entity = {}
        self.rooms = []
        self.hallways = []
        self.locations = []
        self.objects = []
        self.set_metadata()

        # Counters
        self.num_rooms = 0
        self.num_hallways = 0
        self.num_locations = 0
        self.num_objects = 0
        self.hallway_instance_counts = {}
        self.location_instance_counts = {}
        self.object_instance_counts = {}

        # World bounds, will be set by update_bounds()
        self.x_bounds = None
        self.y_bounds = None

        # Other parameters
        self.max_object_sample_tries = (
            1000  # Max number of tries to sample object locations
        )

        # Distances for collision-aware navigation and sampling
        self.object_radius = object_radius
        self.set_inflation_radius(inflation_radius)

        self.logger.info("Created world.")

    ############
    # Metadata #
    ############
    def set_metadata(self, locations=None, objects=None):
        """
        Sets location and object metadata from the specified files.

        :param locations: Path to location metadata YAML file.
        :type locations: str, optional
        :param objects: Path to object metadata YAML file.
        :type objects: str, optional
        """
        if locations is not None:
            Location.set_metadata(locations)
        if objects is not None:
            Object.set_metadata(objects)

    def get_location_metadata(self):
        """
        Returns the location metadata associated with this world.

        :return: The location metadata.
        :rtype: :class:`pyrobosim.utils.general.EntityMetadata`
        """
        return Location.metadata

    def get_object_metadata(self):
        """
        Returns the object metadata associated with this world.

        :return: The object metadata.
        :rtype: :class:`pyrobosim.utils.general.EntityMetadata`
        """
        return Object.metadata

    ##########################
    # World Building Methods #
    ##########################
    def add_room(self, **room_config):
        r"""
        Adds a room to the world.

        If the room does not have a specified name, it will be given an automatic
        name of the form ``"room0"``, ``"room1"``, etc.

        If the room has an empty footprint or would cause a collision with another entity in the world,
        it will not be added to the world model.

        :param \*\*room_config: Keyword arguments describing the room.

            You can use ``room=Room(...)`` to directly pass in a :class:`pyrobosim.core.room.Room` object,
            or alternatively use the same keyword arguments you would use to create a Room object.
        :return: room object if successfully created, else None.
        :rtype: :class:`pyrobosim.core.room.room`
        """

        # If it's a room object, get it from the "room" named argument.
        # Else, create a room directly from the specified arguments.
        if "room" in room_config:
            room = room_config["room"]
        else:
            room = Room(**room_config)

        # Check for duplicate names.
        if room.name in self.get_room_names():
            self.logger.warning(
                f"Room {room.name} already exists in the world. Cannot add."
            )
            return None

        # If the room name is empty, automatically name it.
        if room.name is None:
            room.name = f"room{self.num_rooms}"

        # If the room geometry is empty, do not allow it
        if room.polygon.is_empty:
            self.logger.warning(
                f"Room {room.name} has empty geometry. Cannot add to world."
            )
            return None

        # Check if the room collides with any other rooms or hallways
        is_valid_pose = True
        for other_loc in self.rooms + self.hallways:
            if room.polygon.intersects(other_loc.external_collision_polygon):
                is_valid_pose = False
                break
        if not is_valid_pose:
            self.logger.warning(f"Room {room.name} in collision. Cannot add to world.")
            return None

        self.rooms.append(room)
        self.name_to_entity[room.name] = room
        self.num_rooms += 1
        self.update_bounds(entity=room)

        # Update the room collision polygon based on the world inflation radius
        room.update_collision_polygons(self.inflation_radius)

        room.add_graph_nodes()
        return room

    def remove_room(self, room_name):
        """
        Removes a room from the world by name.

        :param room_name: Name of room to remove.
        :type room_name: str
        :return: True if the room was successfully removed, else False.
        :rtype: bool
        """
        room = self.get_room_by_name(room_name)
        if room is None:
            self.logger.warning(f"No room {room_name} found for removal.")
            return False

        # Remove hallways associated with the room
        while len(room.hallways) > 0:
            self.remove_hallway(room.hallways[-1])

        # Remove locations in the room
        while len(room.locations) > 0:
            self.remove_location(room.locations[-1])

        # Remove the room itself
        self.rooms.remove(room)
        self.name_to_entity.pop(room_name)
        self.num_rooms -= 1
        self.update_bounds(entity=room, remove=True)

        return True

    def add_hallway(self, **hallway_config):
        r"""
        Adds a hallway from with specified parameters related to the :class:`pyrobosim.core.hallway.Hallway` class.

        :param \*\*hallway_config: Keyword arguments describing the hallway.

            You can use ``hallway=Hallway(...)`` to directly pass in a :class:`pyrobosim.core.hallway.Hallway`
            object, or alternatively use the same keyword arguments you would use to create a Hallway object.

            You can also pass in the room names as the ``room_start`` and ``room_end`` arguments,
            and they will be resolved to actual room objects, if they exist in the world.

        :return: Hallway object if successfully created, else None.
        :rtype: :class:`pyrobosim.core.hallway.Hallway`
        """
        # If it's a hallway object, get it from the "hallway" named argument.
        # Else, create a hallway directly from the specified arguments.
        if "hallway" in hallway_config:
            hallway = hallway_config["hallway"]
        else:
            if isinstance(hallway_config["room_start"], str):
                hallway_config["room_start"] = self.get_room_by_name(
                    hallway_config["room_start"]
                )
            if isinstance(hallway_config["room_end"], str):
                hallway_config["room_end"] = self.get_room_by_name(
                    hallway_config["room_end"]
                )

            hallway = Hallway(**hallway_config)

        # Check for duplicate names.
        if hallway.name in self.get_hallway_names():
            self.logger.warning(
                f"Hallway {hallway.name} already exists in the world. Cannot add."
            )
            return None

        # If the hallway name is empty, automatically name it.
        ordered_rooms = tuple(sorted([hallway.room_start.name, hallway.room_end.name]))
        if ordered_rooms not in self.hallway_instance_counts:
            self.hallway_instance_counts[ordered_rooms] = 0
            suffix = ""
        else:
            suffix = f"_{self.hallway_instance_counts[ordered_rooms]}"

        if hallway.name is None:
            hallway.name = f"hall_{ordered_rooms[0]}_{ordered_rooms[1]}{suffix}"
            hallway.reversed_name = (
                f"hall_{ordered_rooms[1]}_{ordered_rooms[0]}{suffix}"
            )

        # Check if the hallway collides with any other rooms or hallways
        is_valid_pose = True
        for other_loc in self.rooms + self.hallways:
            if (other_loc == hallway.room_start) or (other_loc == hallway.room_end):
                continue
            if hallway.polygon.intersects(other_loc.external_collision_polygon):
                is_valid_pose = False
                break
        if not is_valid_pose:
            self.logger.warning(
                f"Hallway {hallway.name} in collision. Cannot add to world."
            )
            return None

        # Do all the necessary bookkeeping
        self.hallways.append(hallway)
        self.name_to_entity[hallway.name] = hallway
        self.name_to_entity[hallway.reversed_name] = hallway
        hallway.room_start.hallways.append(hallway)
        hallway.room_start.update_visualization_polygon()
        hallway.room_end.hallways.append(hallway)
        hallway.room_end.update_visualization_polygon()
        self.hallway_instance_counts[ordered_rooms] += 1
        self.num_hallways += 1
        hallway.update_collision_polygons(self.inflation_radius)
        self.update_bounds(entity=hallway)

        hallway.add_graph_nodes()
        # Finally, return the Hallway object
        return hallway

    def remove_hallway(self, hallway):
        """
        Removes a hallway between two rooms.

        :param hallway: Hallway object to remove.
        :type hallway: :class:`pyrobosim.core.hallway.Hallway`
        :return: True if the hallway was successfully removed, else False.
        :rtype: bool
        """
        # Validate the input
        if not hallway in self.hallways:
            self.logger.warning("Invalid hallway specified.")
            return False

        # Remove the hallways from the world and relevant rooms.
        self.hallways.remove(hallway)
        self.num_hallways -= 1
        ordered_rooms = tuple(sorted([hallway.room_start.name, hallway.room_end.name]))
        self.hallway_instance_counts[ordered_rooms] -= 1
        self.name_to_entity.pop(hallway.name)
        self.name_to_entity.pop(hallway.reversed_name)
        for room in [hallway.room_start, hallway.room_end]:
            room.hallways.remove(hallway)
            room.update_collision_polygons()
            room.update_visualization_polygon()
            self.update_bounds(entity=hallway, remove=True)
        return True

    def add_location(self, **location_config):
        r"""
        Adds a location at the specified parent entity, usually a room.

        If the location does not have a specified name, it will be given an
        automatic name using its category, e.g., ``"table0"``.

        :param \*\*location_config: Keyword arguments describing the location.

            You can use ``location=Location(...)`` to directly pass in a :class:`pyrobosim.core.location.Location`
            object, or alternatively use the same keyword arguments you would use to create a Location object.

            You can also pass in the room name as the ``parent`` argument, and it will be resolved to an actual room object, if it exists in the world.

        :return: Location object if successfully created, else None.
        :rtype: :class:`pyrobosim.core.locations.Location`
        """
        # If it's a location object, get it from the "location" named argument.
        # Else, create a location directly from the specified arguments.
        if "location" in location_config:
            loc = location_config["location"]
        elif "parent" in location_config:
            if isinstance(location_config["parent"], str):
                location_config["parent"] = self.get_room_by_name(
                    location_config["parent"]
                )

            try:
                loc = Location(**location_config)
            except InvalidEntityCategoryException as exception:
                self.logger.warning(str(exception))
                return None
        else:
            self.logger.warning("Location instance or parent must be specified.")
            return None

        # Check for duplicate names.
        if loc.name in self.get_location_names():
            self.logger.warning(
                f"Location {loc.name} already exists in the world. Cannot add."
            )
            return None

        # If the category name is empty, use "location" as the base name.
        category = loc.category
        if category is None:
            category = "location"
        # If the location name is empty, automatically name it.
        if category not in self.location_instance_counts:
            self.location_instance_counts[category] = 0
        if loc.name is None:
            loc.name = f"{category}{self.location_instance_counts[category]}"
            loc.create_spawn_locations()

        # Check that the location fits within the room and is not in collision with
        # other locations already in the room. Else, warn and do not add it.
        is_valid_pose = loc.polygon.within(loc.parent.polygon)
        if is_valid_pose:
            for other_loc in loc.parent.locations:
                if loc.polygon.intersects(other_loc.polygon):
                    is_valid_pose = False
                    break
        if not is_valid_pose:
            self.logger.warning(
                f"Location {loc.name} in collision. Cannot add to world."
            )
            return None

        # Do all the necessary bookkeeping
        loc.update_collision_polygon(self.inflation_radius)
        loc.parent.locations.append(loc)
        loc.parent.update_collision_polygons(self.inflation_radius)
        self.locations.append(loc)
        self.location_instance_counts[loc.category] += 1
        self.num_locations += 1
        self.name_to_entity[loc.name] = loc
        for spawn in loc.children:
            self.name_to_entity[spawn.name] = spawn

        loc.add_graph_nodes()
        if self.has_gui:
            self.gui.canvas.show_locations_signal.emit()
            self.gui.canvas.show_objects_signal.emit()
            self.gui.canvas.draw_signal.emit()
        return loc

    def update_location(self, loc, pose, room=None, is_open=None, is_locked=None):
        """
        Updates an existing location in the world.

        :param loc: Location instance or name to update.
        :type loc: :class:`pyrobosim.core.locations.Location`/str
        :param pose: Pose of the location.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :param room: Room instance or name. If none, uses the previous room.
        :type room: :class:`pyrobosim.core.room.Room`/str, optional
        :param is_open: Whether the location should be open. If None, keeps the current state.
        :type is_open: bool, optional
        :param is_locked: Whether the location should be locked. If None, keeps the current state.
        :type is_locked: bool, optional
        :return: True if the update was successful, else False.
        :rtype: bool
        """
        if isinstance(loc, str):
            loc = self.get_location_by_name(loc)
        if not isinstance(loc, Location):
            self.logger.warning("Could not find location. Not updating.")
            return False

        if room is not None:
            if isinstance(room, str):
                room = self.get_room_by_name(room)

            if not isinstance(room, Room):
                self.logger.warning(
                    f"Room {loc} did not resolve to a valid room for a location."
                )
                return False

        if is_open is not None:
            loc.is_open = is_open
        if is_locked is not None:
            loc.is_locked = is_locked

        # Check that the location fits within the room and is not in collision with
        # other locations already in the room. Else, warn and do not add it.
        new_polygon = transform_polygon(loc.raw_polygon, pose)
        is_valid_pose = new_polygon.within(room.polygon)
        for other_loc in room.locations:
            if loc != other_loc:
                is_valid_pose = is_valid_pose and not new_polygon.intersects(
                    other_loc.polygon
                )
        if not is_valid_pose:
            self.logger.warning(
                f"Location {loc.name} in collision. Cannot add to world."
            )
            return False

        # If we passed all checks, update the polygon.
        loc.parent.locations.remove(loc)
        loc.parent = room
        room.locations.append(loc)
        loc.set_pose(pose)
        loc.create_polygons(self.inflation_radius)
        for spawn in loc.children:
            spawn.set_pose_from_parent()
        if self.has_gui:
            self.gui.canvas.show_locations_signal.emit()
            self.gui.canvas.show_objects_signal.emit()
            self.gui.canvas.draw_signal.emit()
        return True

    def remove_location(self, loc):
        """
        Cleanly removes a location from the world.

        :param loc: Location instance of name to remove.
        :type loc: :class:`pyrobosim.core.locations.Location`/str
        :return: True if the location was successfully removed, else False.
        :rtype: bool
        """
        # Parse inputs
        if isinstance(loc, str):
            loc = self.get_location_by_name(loc)

        if loc not in self.locations:
            self.logger.warning(f"{loc} not found in world. Cannot remove.")
            return False

        # Remove objects at the location before removing the location.
        for spawn in loc.children:
            while len(spawn.children) > 0:
                self.remove_object(spawn.children[-1])

        # Remove the location.
        self.locations.remove(loc)
        self.num_locations -= 1
        self.location_instance_counts[loc.category] -= 1
        room = loc.parent
        room.locations.remove(loc)
        room.update_collision_polygons(self.inflation_radius)
        self.name_to_entity.pop(loc.name)
        for spawn in loc.children:
            self.name_to_entity.pop(spawn.name)
        if self.has_gui:
            self.gui.canvas.show_locations_signal.emit()
            self.gui.canvas.show_objects_signal.emit()
            self.gui.canvas.draw_signal.emit()
        return True

    def open_location(self, location):
        """
        Opens a storage location or hallway between two rooms..

        :param location: Location or Hallway object to open, or its name.
        :type location: :class:`pyrobosim.core.locations.Location`, :class:`pyrobosim.core.hallway.Hallway`, or str
        :return: An object describing the execution result.
        :rtype: :class:`pyrobosim.planning.actions.ExecutionResult`
        """
        # Validate the input
        if isinstance(location, str):
            location = self.get_entity_by_name(location)
        if not (isinstance(location, Location) or isinstance(location, Hallway)):
            message = message = (
                f"Cannot open {location} since it is of type {type(location).__name__}."
            )
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.INVALID_ACTION, message=message
            )

        if not (location in self.locations or location in self.hallways):
            message = "Invalid location specified."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if location.is_open:
            message = f"{location} is already open."
            self.logger.warning(message)
            return ExecutionResult(status=ExecutionStatus.SUCCESS, message=message)

        if location.is_locked:
            message = f"{location} is locked."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        location.is_open = True
        location.update_visualization_polygon()
        if self.has_gui:
            if isinstance(location, Hallway):
                self.gui.canvas.show_hallways_signal.emit()
            else:
                self.gui.canvas.show_locations_signal.emit()
            self.gui.update_buttons_signal.emit()
        return ExecutionResult(status=ExecutionStatus.SUCCESS)

    def close_location(self, location, ignore_robots=[]):
        """
        Close a storage location or hallway.

        :param location: Location or Hallway object to close, or its name.
        :type location: :class:`pyrobosim.core.locations.Location`, :class:`pyrobosim.core.hallway.Hallway`, or str
        :param ignore_robots: List of robots to ignore, for example the robot closing the hallway.
        :type ignore_robots: list[:class:`pyrobosim.core.robot.Robot`]
        :return: An object describing the execution result.
        :rtype: :class:`pyrobosim.planning.actions.ExecutionResult`
        """
        # Validate the input
        if isinstance(location, str):
            location = self.get_entity_by_name(location)
        if not (isinstance(location, Location) or isinstance(location, Hallway)):
            message = message = (
                f"Cannot close {location} since it is of type {type(location).__name__}."
            )
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.INVALID_ACTION, message=message
            )

        if not (location in self.locations or location in self.hallways):
            message = "Invalid location specified."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if not location.is_open:
            message = f"{location} is already closed."
            self.logger.warning(message)
            return ExecutionResult(status=ExecutionStatus.SUCCESS, message=message)

        if location.is_locked:
            message = f"{location} is locked."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        is_hallway = isinstance(location, Hallway)
        if is_hallway:
            for robot in [r for r in self.robots if r not in ignore_robots]:
                if location.is_collision_free(robot.get_pose()):
                    message = f"Robot {robot.name} is in {location}. Cannot close."
                    self.logger.warning(message)
                    return ExecutionResult(
                        status=ExecutionStatus.PRECONDITION_FAILURE, message=message
                    )

        location.is_open = False
        location.update_visualization_polygon()
        if self.has_gui:
            if is_hallway:
                self.gui.canvas.show_hallways_signal.emit()
            else:
                self.gui.canvas.show_locations_signal.emit()
            self.gui.update_buttons_signal.emit()
        return ExecutionResult(status=ExecutionStatus.SUCCESS)

    def lock_location(self, location):
        """
        Locks a storage location or hallway.

        :param location: Location or Hallway object to lock, or its name.
        :type location: :class:`pyrobosim.core.locations.Location`, :class:`pyrobosim.core.hallway.Hallway`, or str
        :return: An object describing the execution result.
        :rtype: :class:`pyrobosim.planning.actions.ExecutionResult`
        """
        # Validate the input
        if isinstance(location, str):
            location = self.get_entity_by_name(location)
        if not (isinstance(location, Location) or isinstance(location, Hallway)):
            message = message = (
                f"Cannot lock {location} since it is of type {type(location).__name__}."
            )
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.INVALID_ACTION, message=message
            )

        if not (location in self.locations or location in self.hallways):
            message = "Invalid location specified."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if location.is_locked:
            message = f"{location} is already locked."
            self.logger.warning(message)
            return ExecutionResult(status=ExecutionStatus.SUCCESS, message=message)

        location.is_locked = True
        return ExecutionResult(status=ExecutionStatus.SUCCESS)

    def unlock_location(self, location):
        """
        Unlocks a storage location or hallway.

        :param location: Location or Hallway object to unlock, or its name.
        :type location: :class:`pyrobosim.core.locations.Location`, :class:`pyrobosim.core.hallway.Hallway`, or str
        :return: An object describing the execution result.
        :rtype: :class:`pyrobosim.planning.actions.ExecutionResult`
        """
        # Validate the input
        if isinstance(location, str):
            location = self.get_entity_by_name(location)
        if not (isinstance(location, Location) or isinstance(location, Hallway)):
            message = message = (
                f"Cannot unlock {location} since it is of type {type(location).__name__}."
            )
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.INVALID_ACTION, message=message
            )

        if not (location in self.locations or location in self.hallways):
            message = "Invalid location specified."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if not location.is_locked:
            message = f"{location} is already unlocked."
            self.logger.warning(message)
            return ExecutionResult(status=ExecutionStatus.SUCCESS, message=message)

        location.is_locked = False
        return ExecutionResult(status=ExecutionStatus.SUCCESS)

    def sample_object_spawn_pose(self, obj, obj_spawn, max_tries):
        """
        Samples an object pose in a specific object spawn.

        :param obj: Object instance.
        :type obj: :class:`pyrobosim.core.objects.Object`
        :param obj_spawn: Object spawn instance.
        :type obj_spawn: :class:`pyrobosim.core.locations.ObjectSpawn`
        :param max_tries: The maximum number of tries to sample.
        :type max_tries: int
        :return: A sampled spawn pose, if one was found. Otherwise returns None.
        :rtype: :class:`pyrobosim.utils.pose.Pose` / None
        """
        for _ in range(max_tries):
            x_sample, y_sample = sample_from_polygon(obj_spawn.polygon)
            yaw_sample = np.random.uniform(-np.pi, np.pi)
            pose_sample = Pose(x=x_sample, y=y_sample, z=0.0, yaw=yaw_sample)
            poly = transform_polygon(obj.raw_collision_polygon, pose_sample)

            is_valid_pose = True
            if not poly.within(obj_spawn.polygon):
                continue
            for other_obj in obj_spawn.children:
                if other_obj == obj:
                    continue
                if poly.intersects(other_obj.collision_polygon):
                    is_valid_pose = False
                    break
            if is_valid_pose:
                return pose_sample
        return None

    def add_object(self, **object_config):
        r"""
        Adds an object to a specific location.

        If the object does not have a specified name, it will be given an
        automatic name using its category, e.g., ``"apple0"``.

        If the location contains multiple object spawns, one will be selected at random.

        :param \*\*object_config: Keyword arguments describing the object.

            You can use ``object=Object(...)`` to directly pass in a :class:`pyrobosim.core.objects.Object`
            object, or alternatively use the same keyword arguments you would use to create an Object instance.

            You can also pass in the parent entity name as the ``parent`` argument, and it will be resolved to an actual entity, if it exists in the world.

        :return: Object instance if successfully created, else None.
        :rtype: :class:`pyrobosim.core.objects.Object`
        """
        # If it's an Object instance, get it from the "object" named argument.
        # Else, create an object directly from the specified arguments.
        if "object" in object_config:
            obj = object_config["object"]
        elif "parent" in object_config:
            parent = object_config.get("parent", None)
            if isinstance(parent, str):
                parent = self.get_entity_by_name(parent)

            # If it's a location object, pick an object spawn at random.
            # Otherwise, if it's an object spawn, use that entity as is.
            if isinstance(parent, Location):
                parent = np.random.choice(parent.children)

            if not isinstance(parent, ObjectSpawn):
                parent_arg = object_config.get("parent", None)
                self.logger.warning(
                    f"Parent location {parent_arg} did not resolve to a valid location for an object."
                )
                return None

            object_config["parent"] = parent
            object_config["inflation_radius"] = self.object_radius
            try:
                obj = Object(**object_config)
            except InvalidEntityCategoryException as exception:
                self.logger.warning(str(exception))
                return None
        else:
            self.logger.warning("Object instance or parent must be specified.")
            return None

        # Check for duplicate names.
        if obj.name in self.get_object_names():
            self.logger.warning(
                f"Object {obj.name} already exists in the world. Cannot add."
            )
            return None

        # If the category name is empty, use "object" as the base name.
        category = obj.category
        if category is None:
            category = "object"
        # If no name is specified, create one automatically.
        if category not in self.object_instance_counts:
            self.object_instance_counts[category] = 0
        if obj.name is None:
            obj.name = f"{category}{self.object_instance_counts[category]}"
        self.object_instance_counts[category] += 1

        # If no pose is specified, sample a valid one.
        obj_spawn = obj.parent
        if obj.pose is None:
            pose_sample = self.sample_object_spawn_pose(
                obj, obj_spawn, self.max_object_sample_tries
            )

            if pose_sample:
                obj.parent = obj_spawn
                obj.set_pose(pose_sample)
                obj.create_polygons()
            else:
                self.logger.warning(
                    f"Could not sample valid pose to add object {obj.name}."
                )
                return None

        # If a pose was specified, collision check it
        else:
            poly = obj.collision_polygon
            is_valid_pose = poly.within(obj_spawn.polygon)
            for other_obj in obj_spawn.children:
                is_valid_pose = is_valid_pose and not poly.intersects(
                    other_obj.collision_polygon
                )
            if not is_valid_pose:
                self.logger.warning(
                    f"Object {obj.name} in collision or not in location {obj_spawn.name}. Cannot add to world."
                )
                return None

        # Do the necessary bookkeeping
        obj_spawn.children.append(obj)
        self.objects.append(obj)
        self.name_to_entity[obj.name] = obj
        self.num_objects += 1
        if self.has_gui:
            self.gui.canvas.show_objects_signal.emit()
        return obj

    def update_object(self, obj, loc=None, pose=None):
        """
        Updates an existing object in the world.

        :param obj: Object instance or name to update.
        :type obj: :class:`pyrobosim.core.objects.Object`/str
        :param loc: Location or object spawn instance or name. If none, uses the previous location.
        :type loc: :class:`pyrobosim.core.locations.Location`/:class:`pyrobosim.core.locations.ObjectSpawn`/str, optional
        :param pose: Pose of the object. If none is specified, it will be sampled.
        :type pose: :class:`pyrobosim.utils.pose.Pose`, optional
        :return: True if the update was successful, else False.
        :rtype: bool
        """
        if isinstance(obj, str):
            obj = self.get_object_by_name(obj)
        if not isinstance(obj, Object):
            self.logger.warning("Could not find object. Not updating.")
            return False

        if loc is not None:
            # Find an object spawn that matches the specified location.
            # If it's a string, get the location name.
            if isinstance(loc, str):
                loc = self.get_entity_by_name(loc)
            # If it's a location object, pick an object spawn at random.
            # Otherwise, if it's an object spawn, use that entity as is.
            if isinstance(loc, Location):
                obj_spawn = np.random.choice(loc.children)
            elif isinstance(loc, ObjectSpawn):
                obj_spawn = loc
            else:
                self.logger.warning(
                    f"Location {loc} did not resolve to a valid location for an object."
                )
                return False

            # Next, sample a pose within the new object spawn, if one was not specified.
            if pose is None:
                pose = self.sample_object_spawn_pose(
                    obj, obj_spawn, self.max_object_sample_tries
                )
            if pose is None:
                self.logger.warning("Cannot sample a valid spawn pose.")
                return False

            obj.parent.children.remove(obj)
            obj.parent = obj_spawn
            obj_spawn.children.append(obj)

        if pose is not None:
            obj.set_pose(pose)
            obj.create_polygons()

        return True

    def remove_object(self, obj):
        """
        Cleanly removes an object from the world.

        :param loc: Object instance of name to remove.
        :type loc: :class:`pyrobosim.core.objects.Object`/str
        :return: True if the object was successfully removed, else False.
        :rtype: bool
        """
        if isinstance(obj, str):
            obj = self.get_object_by_name(obj)

        if obj not in self.objects:
            return False

        self.objects.remove(obj)
        self.name_to_entity.pop(obj.name)
        self.num_objects -= 1
        obj.parent.children.remove(obj)
        if self.has_gui:
            self.gui.canvas.show_objects_signal.emit()
        return True

    def remove_all_objects(self, restart_numbering=True):
        """
        Cleanly removes all objects from the world.

        :param restart_numbering: If True, restarts numbering of all
            categories to zero, defaults to True.
        :type restart_numbering: bool, optional
        """
        for obj in reversed(self.objects):
            self.remove_object(obj)
        self.num_objects = 0
        if restart_numbering:
            self.object_instance_counts = {}

    def add_robot(self, robot, loc=None, pose=None):
        """
        Adds a robot to the world given either a world entity and/or pose.

        :param robot: Robot instance to add to the world.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        :param loc: World entity instance or name to place the robot.
        :type loc: Entity, optional
        :param pose: Pose at which to add the robot. If not specified, will be sampled.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        """
        # Check that the robot name doesn't already exist.
        if robot.name in self.get_robot_names():
            self.logger.warning(f"Robot name {robot.name} already exists in world.")
            return

        # If the new robot has a bigger inflation radius than previously,
        # use this new one. Otherwise, we can leave it as is.
        old_inflation_radius = self.inflation_radius
        new_inflation_radius = max(
            [other_robot.radius for other_robot in self.robots] + [robot.radius]
        )
        if new_inflation_radius > old_inflation_radius:
            self.set_inflation_radius(new_inflation_radius)

        valid_pose = True
        if loc is None:
            if pose is None:
                # If nothing is specified, sample any valid location in the world
                robot_pose = self.sample_free_robot_pose_uniform(
                    robot, ignore_robots=False
                )
                if robot_pose is None:
                    self.logger.warning("Unable to sample free pose.")
                    valid_pose = False
            else:
                # Validate that the pose is unoccupied
                if self.check_occupancy((pose.x, pose.y)):
                    self.logger.warning(f"{pose} is occupied.")
                    valid_pose = False
                robot_pose = pose
            # If we have a valid pose, extract its location
            loc = self.get_location_from_pose(robot_pose)

        else:
            # First, validate that the location is valid for a robot
            if isinstance(loc, str):
                loc = self.get_entity_by_name(loc)

            if isinstance(loc, Room) or isinstance(loc, Hallway):
                if pose is None:
                    # Sample a pose in the location
                    x_sample, y_sample = sample_from_polygon(
                        loc.internal_collision_polygon,
                        max_tries=self.max_object_sample_tries,
                    )
                    if x_sample is None:
                        self.logger.warning(f"Could not sample pose in {loc.name}.")
                        valid_pose = False
                    yaw_sample = np.random.uniform(-np.pi, np.pi)
                    robot_pose = Pose(x=x_sample, y=y_sample, z=0.0, yaw=yaw_sample)
                else:
                    # Validate that the pose is unoccupied and in the right location
                    if not loc.is_collision_free(pose):
                        self.logger.warning(f"{pose} is occupied")
                        valid_pose = False
                    robot_pose = pose
            elif isinstance(loc, Location) or isinstance(loc, ObjectSpawn):
                if isinstance(loc, Location):
                    # NOTE: If you don't want a random object spawn, use the object spawn as the input location.
                    loc = np.random.choice(loc.children)

                if pose in loc.nav_poses:  # Slim chance of this happening lol
                    robot_pose = pose
                else:
                    robot_pose = np.random.choice(loc.nav_poses)
            else:
                self.logger.warning("Invalid location specified.")
                valid_pose = False

        # If we got a valid location / pose combination, add the robot
        if valid_pose:
            robot.location = loc
            robot.set_pose(robot_pose)
            robot.world = self
            self.robots.append(robot)
            self.name_to_entity[robot.name] = robot
        else:
            self.logger.warning("Could not add robot.")
            self.set_inflation_radius(old_inflation_radius)

        if self.has_gui:
            self.gui.canvas.show_robots_signal.emit()
        if self.has_ros_node:
            self.ros_node.add_robot_ros_interfaces()

    def remove_robot(self, robot_name):
        """
        Removes a robot from the world.

        :return: True if the robot was successfully removed, else False.
        :rtype: bool
        """
        robot = self.get_robot_by_name(robot_name)
        if robot:
            self.robots.remove(robot)
            self.name_to_entity.pop(robot_name)
            if self.has_gui:
                self.gui.canvas.show_robots_signal.emit()
            if self.has_ros_node:
                self.ros_node.remove_robot_ros_interfaces(robot)

            # Find the new max inflation radius and revert it.
            new_inflation_radius = max(
                [other_robot.radius for other_robot in self.robots] + [robot.radius]
            )
            if new_inflation_radius != self.inflation_radius:
                self.set_inflation_radius(new_inflation_radius)
            return True
        else:
            self.logger.warning(f"Could not find robot {robot_name} to remove.")
            return False

    def set_inflation_radius(self, inflation_radius=0.0):
        """
        Sets world inflation radius.

        :param inflation_radius: Inflation radius, in meters.
        :type inflation_radius: float, optional
        """
        self.inflation_radius = inflation_radius
        for loc in self.locations:
            loc.update_collision_polygon(self.inflation_radius)
        for entity in itertools.chain(self.rooms, self.hallways):
            entity.update_collision_polygons(self.inflation_radius)

    def update_bounds(self, entity, remove=False):
        """
        Updates the X and Y bounds of the world.

        :param entity: The entity that is being added or removed
        :type entity: :class:`pyrobosim.core.room.Room`/:class:`pyrobosim.core.hallway.Hallway`
        :param remove: Specifies if the update is due to removal of an entity.
        :type remove: bool
        """
        if isinstance(entity, (Room, Hallway)):
            (xmin, ymin, xmax, ymax) = entity.polygon.bounds

            if not self.x_bounds:
                # When adding the first room
                self.x_bounds = [xmin, xmax]
                self.y_bounds = [ymin, ymax]
                return

            if remove:
                sets_x_bounds = (self.x_bounds[0] == xmin) or (self.x_bounds[1] == xmax)
                sets_y_bounds = (self.y_bounds[0] == ymin) or (self.y_bounds[1] == ymin)
                is_last_room = len(self.rooms) == 0 and isinstance(entity, Room)
                # Only update if the Room/Hallway being removed affects the bounds
                if sets_x_bounds or sets_y_bounds:
                    for other_entity in itertools.chain(self.rooms, self.hallways):
                        (xmin, ymin, xmax, ymax) = other_entity.polygon.bounds
                        self.x_bounds[0] = min(self.x_bounds[0], xmin)
                        self.x_bounds[1] = max(self.x_bounds[1], xmax)
                        self.y_bounds[0] = min(self.y_bounds[0], ymin)
                        self.y_bounds[1] = max(self.y_bounds[1], ymax)
                if is_last_room:
                    # When last room has been deleted
                    self.x_bounds = None
                    self.y_bounds = None
            else:
                # Adding a Room/Hallway
                self.x_bounds[0] = min(self.x_bounds[0], xmin)
                self.x_bounds[1] = max(self.x_bounds[1], xmax)
                self.y_bounds[0] = min(self.y_bounds[0], ymin)
                self.y_bounds[1] = max(self.y_bounds[1], ymax)
        else:
            self.logger.warning(
                f"Updating bounds with unsupported entity type {type(entity)}"
            )

    ################################
    # Lookup Functionality Methods #
    ################################
    def get_room_names(self):
        """
        Gets all room names in the world.

        :return: List of all room names.
        :rtype: list[str]
        """
        return [room.name for room in self.rooms]

    def get_room_by_name(self, name):
        """
        Gets a room object by its name.

        :param name: Name of room.
        :type name: str
        :return: Room instance matching the input name, or ``None`` if not valid.
        :rtype: :class:`pyrobosim.core.room.Room`
        """
        if name not in self.name_to_entity:
            self.logger.warning(f"Room not found: {name}")
            return None

        entity = self.name_to_entity[name]
        if not isinstance(entity, Room):
            self.logger.warning(f"Entity {name} found but it is not a Room.")
            return None

        return entity

    def get_hallway_names(self):
        """
        Gets all hallway names.

        :return: List of all hallway names.
        :rtype: list[str]
        """
        return [hall.name for hall in self.hallways]

    def get_hallway_by_name(self, name):
        """
        Gets a hallway object by its name.

        :param name: Name of hallway.
        :type name: str
        :return: Hallway instance matching the input name, or ``None`` if not valid.
        :rtype: :class:`pyrobosim.core.hallway.Hallway`
        """
        if name not in self.name_to_entity:
            self.logger.warning(f"Hallway not found: {name}")
            return None

        entity = self.name_to_entity[name]
        if not isinstance(entity, Hallway):
            self.logger.warning(f"Entity {name} found but it is not a Hallway.")
            return None

        return entity

    def get_hallways_from_rooms(self, room1, room2):
        """
        Returns a list of hallways between two rooms.

        :param room1: Instance or name of first room.
        :type room1: :class:`pyrobosim.core.room.Room`/str
        :param room2: Instance or name of second room.
        :type room2: :class:`pyrobosim.core.room.Room`/str
        :return: List of hallways.
        :rtype: list[:class:`pyrobosim.core.hallway.Hallway`]
        """
        # Validate room input
        if isinstance(room1, str):
            room1 = self.get_room_by_name(room1)
        if not isinstance(room1, Room):
            self.logger.warning("Invalid room1 specified.")
            return []
        if isinstance(room2, str):
            room2 = self.get_room_by_name(room2)
        if not isinstance(room2, Room):
            self.logger.warning("Invalid room2 specified.")
            return []

        # Now search through the hallways and add any valid ones to the list
        hallways = []
        for hall in room1.hallways:
            is_valid_hallway = (
                (hall.room_start == room1) and (hall.room_end == room2)
            ) or ((hall.room_start == room2) and (hall.room_end == room1))
            if is_valid_hallway:
                hallways.append(hall)
        return hallways

    def get_hallways_attached_to_room(self, room):
        """
        Returns a list of hallways attached to a specific room.

        :param room: Instance or name of room.
        :type room: :class:`pyrobosim.core.room.Room`/str
        :return: List of hallways.
        :rtype: list[:class:`pyrobosim.core.hallway.Hallway`]
        """
        # Validate room input
        if isinstance(room, str):
            room = self.get_room_by_name(room)
        if not isinstance(room, Room):
            self.logger.warning("Invalid room specified.")
            return []

        # Now search through the hallways and add any valid ones to the list
        hallways = []
        for hall in room.hallways:
            is_valid_hallway = (hall.room_start == room) or (hall.room_end == room)
            if is_valid_hallway:
                hallways.append(hall)
        return hallways

    def get_locations(self, category_list=None):
        """
        Gets all locations, optionally filtered by category.

        :param category_list: Optional list of categories to filter by.
        :type category_list: list[str]
        :return: List of locations that match the input category list, if specified.
        :rtype: list[:class:`pyrobosim.core.locations.Location`]
        """
        if not category_list:
            return [loc for loc in self.locations]
        else:
            return [loc for loc in self.locations if loc.category in category_list]

    def get_location_names(self, category_list=None):
        """
        Gets all location names, optionally filtered by category.

        :param category_list: Optional list of categories to filter by.
        :type category_list: list[str]
        :return: List of location names that match the input category list, if specified.
        :rtype: list[str]
        """
        if not category_list:
            return [loc.name for loc in self.locations]
        else:
            return [loc.name for loc in self.locations if loc.category in category_list]

    def get_location_by_name(self, name):
        """
        Gets a location object by its name.

        :param name: Name of location.
        :type name: str
        :return: Location instance matching the input name, or ``None`` if not valid.
        :rtype: :class:`pyrobosim.core.locations.Location`
        """
        if name not in self.name_to_entity:
            self.logger.warning(f"Location not found: {name}")
            return None

        entity = self.name_to_entity[name]
        if not isinstance(entity, Location):
            self.logger.warning(f"Entity {name} found but it is not a Location.")
            return None

        return entity

    def get_location_in_room(self, pose, room):
        """
        Gets a location in a room given an input pose.

        :param pose: Input pose.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :param room: The reference room object.
        :type room: :class:`pyrobosim.core.room.Room`
        :return: Entity matching the input pose and room, or ``None`` if not valid.
            This could either be the Room itself, an object spawn, or ``None``.
        :rtype: :class:`pyrobosim.core.room.Room` / :class:`pyrobosim.core.locations.ObjectSpawn` / None
        """
        if room.is_collision_free(pose):
            for location in room.locations:
                for spawn in location.children:
                    for nav_pose in spawn.nav_poses:
                        if pose.is_approx(nav_pose):
                            return spawn
                for nav_pose in location.nav_poses:
                    if pose.is_approx(nav_pose):
                        return location
            return room
        return None

    def get_location_in_hallway(self, pose, hallway):
        """
        Gets a location in a hallway given an input pose.

        :param pose: Input pose.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :param hallway: The reference hallway object.
        :type hallway: :class:`pyrobosim.core.hallway.Hallway`
        :return: Entity matching the input pose and hallway, or ``None`` if not valid.
            This could either be the Hallway itself, or ``None``.
        :rtype: :class:`pyrobosim.core.hallway.Hallway` / None
        """
        for nav_pose in hallway.nav_poses:
            if pose.is_approx(nav_pose):
                return hallway
        if hallway.is_collision_free(pose):
            return hallway
        return None

    def get_location_from_pose(self, pose, prev_location=None):
        """
        Gets the name of a location given a pose.

        :param pose: Input pose.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :param prev_location: The robot's previous location.
            This helps optimize this function by first checking the robot's previous location.
        :type prev_location: :class:`pyrobosim.core.room.Room`/:class:`pyrobosim.core.hallway.Hallway`/:class:`pyrobosim.core.locations.ObjectSpawn`
        :return: Entity matching the input pose, or ``None`` if not valid.
            This could be a room, hallway, or object spawn.
        :rtype: :class:`pyrobosim.core.room.Room`/:class:`pyrobosim.core.hallway.Hallway`/:class:`pyrobosim.core.locations.ObjectSpawn`
        """
        # Check the previous location.
        prev_room = None
        prev_hallways = []
        if prev_location is not None:
            if isinstance(prev_location, ObjectSpawn):
                prev_location = prev_location.get_room_name()

            if isinstance(prev_location, Room):
                loc = self.get_location_in_room(pose, prev_location)
                if loc is not None:
                    attached_hallways = self.get_hallways_attached_to_room(
                        prev_location
                    )
                    for hallway in attached_hallways:
                        hall_loc = self.get_location_in_hallway(pose, hallway)
                        if hall_loc:
                            return hall_loc
                        prev_hallways = attached_hallways
                    return loc
                prev_room = loc
            elif isinstance(prev_location, Hallway):
                loc = self.get_location_in_hallway(pose, prev_location)
                if loc is not None:
                    return loc
                prev_hallways.append(loc)

        # Check remaining hallways and their nav poses.
        for hallway in self.hallways:
            if hallway in prev_hallways:
                continue
            loc = self.get_location_in_hallway(pose, hallway)
            if loc is not None:
                return loc
        # Check remaining rooms and the locations / object spawns inside them.
        for room in self.rooms:
            if room == prev_room:
                continue
            loc = self.get_location_in_room(pose, room)
            if loc is not None:
                return loc

        # No valid location was found.
        return None

    def get_object_spawns(self, category_list=None):
        """
        Gets all object spawn locations, optionally filtered by category.

        :param category_list: Optional list of categories to filter by.
        :type category_list: list[str]
        :return: List of object spawn locations that match the input category
            list, if specified.
        :rtype: list[:class:`pyrobosim.core.locations.ObjectSpawn`]
        """
        spawn_list = []
        for loc in self.locations:
            if not category_list or loc.category in category_list:
                spawn_list.extend(loc.children)
        return spawn_list

    def get_object_spawn_names(self, category_list=None):
        """
        Gets all object spawn location names, optionally filtered by category.

        :param category_list: Optional list of categories to filter by.
        :type category_list: list[str]
        :return: List of object spawn location names that match the input
            category list, if specified.
        :rtype: list[str]
        """
        spawn_name_list = []
        for loc in self.locations:
            if not category_list or loc.category in category_list:
                spawn_name_list.extend([spawn.name for spawn in loc.children])
        return spawn_name_list

    def get_objects(self, category_list=None):
        """
        Gets all objects in the world, optionally filtered by category.

        :param name: Name of location.
        :type name: str
        :return: List of object that match the input category list, if specified.
        :rtype: list[:class:`pyrobosim.core.objects.Object`]
        """
        if not category_list:
            return self.objects
        else:
            return [o for o in self.objects if o.category in category_list]

    def get_object_names(self, category_list=None):
        """
        Gets all object names in the world, optionally filtered by category.

        :param category_list: Optional list of categories to filter by.
        :type category_list: list[str]
        :return: List of object names that match the input category list, if specified.
        :rtype: list[str]
        """
        if not category_list:
            return [o.name for o in self.objects]
        else:
            return [o.name for o in self.objects if o.category in category_list]

    def get_object_by_name(self, name):
        """
        Gets an object by its name.

        :param name: Name of object.
        :type name: str
        :return: Object instance matching the input name, or ``None`` if not valid.
        :rtype: :class:`pyrobosim.core.object.Object`
        """
        if name not in self.name_to_entity:
            return None

        entity = self.name_to_entity[name]
        if not isinstance(entity, Object):
            self.logger.warning(f"Entity {name} found but it is not an Object.")
            return None

        return entity

    def get_robot_names(self):
        """
        Gets all robot names in the world.

        :return: List of robot names.
        :rtype: list[str]
        """
        return [robot.name for robot in self.robots]

    def get_robot_by_name(self, name):
        """
        Gets a robot by its name.

        :param name: Name of robot.
        :type name: str
        :return: Robot instance matching the input name, or ``None`` if not valid.
        :rtype: :class:`pyrobosim.core.robot.Robot`
        """
        if name not in self.name_to_entity:
            return None

        entity = self.name_to_entity[name]
        if not isinstance(entity, Robot):
            self.logger.warning(f"Entity {name} found but it is not a Robot.")
            return None

        return entity

    def get_entity_by_name(self, name):
        """
        Gets any world entity by its name.

        :param name: Name of entity.
        :type name: str
        :return: Entity object instance matching the input name, or ``None`` if not valid.
        """
        if name in self.name_to_entity:
            return self.name_to_entity[name]
        else:
            return None

    #######################
    # Occupancy utilities #
    #######################

    def is_connectable(self, start, goal, step_dist=0.01, max_dist=None):
        """
        Checks connectivity between two poses `start` and `goal` in the world
        by sampling points spaced by the `self.collision_check_dist` parameter
        and verifying that every point is in the free configuration space.
        :param start: Start node
        :type start: :class:`Pose`
        :param goal: Goal node
        :type goal: :class:`Pose`
        :param step_dist: The step size for discretizing a straight line to check collisions.
        :type step_dist: float
        :param max_dist: The maximum allowed connection distance.
        :type max_dist: float, optional
        :return: True if nodes can be connected, else False.
        :rtype: bool
        """
        # Trivial case where nodes are identical.
        if start == goal:
            return True

        # Check against the max edge distance.
        dist = start.get_linear_distance(goal, ignore_z=True)
        angle = start.get_angular_distance(goal)
        if max_dist and (dist > max_dist):
            return False

        # Build up the array of test X and Y coordinates for sampling between
        # the start and goal points.
        dist_array = np.arange(0, dist, step_dist)
        # If the nodes are coincident, connect them by default.
        if dist_array.size == 0:
            return True
        if dist_array[-1] != dist:
            np.append(dist_array, dist)
        x_pts = start.x + dist_array * np.cos(angle)
        y_pts = start.y + dist_array * np.sin(angle)

        # Check the occupancy of all the test points.
        for x_check, y_check in zip(x_pts[1:], y_pts[1:]):
            if self.check_occupancy(Pose(x=x_check, y=y_check)):
                return False

        # If the loop was traversed for all points without returning, we can
        # connect the points.
        return True

    def check_occupancy(self, pose):
        """
        Check if a pose in the world is occupied.
        :param pose: Pose for checking occupancy.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :return: True if the pose is occupied, else False if free.
        :rtype: bool
        """
        # Loop through all the rooms and hallways and check if the pose
        # is deemed collision-free in any of them.
        for entity in itertools.chain(self.rooms, self.hallways):
            if entity.is_collision_free(pose):
                return False
        # If we made it through, the pose is occupied.
        return True

    def collides_with_robots(self, pose, robot=None):
        """
        Checks if a pose collides with robots in the world.
        Currently assumes that robots are circles, so we can do simple checks.
        If this changes, should account for polygon collisions.
        :param pose: Candidate pose to check.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :param robot: Robot instance, if specified.
        :type robot: :class:`pyrobosim.core.robot.Robot`, optional
        :return: True if the pose collides with a robot besides the input.
        :rtype: bool
        """
        radius = self.inflation_radius if robot is None else robot.radius
        robot_list = [
            other_robot for other_robot in self.robots if other_robot is not robot
        ]
        for other_robot in robot_list:
            min_distance = radius + other_robot.radius
            if pose.get_linear_distance(other_robot.get_pose()) < min_distance:
                return True
        return False

    def sample_free_robot_pose_uniform(self, robot=None, ignore_robots=True):
        """
        Sample an unoccupied robot pose in the world.
        This is done using uniform sampling within the world X-Y bounds and rejecting
        any samples that are in collision with entities in the world.
        If no valid samples could be found within the `max_object_sample_tries` instance
        attribute, this will return ``None``.
        :param robot: Robot instance, if specified.
        :type robot: :class:`pyrobosim.core.robot.Robot`, optional
        :param ignore_robots: If True, ignore collisions with other robots.
        :type ignore_robots: bool
        :return: Collision-free pose if found, else ``None``.
        :rtype: :class:`pyrobosim.utils.pose.Pose`
        """
        xmin, xmax = self.x_bounds
        ymin, ymax = self.y_bounds
        r = self.inflation_radius if robot is None else robot.radius

        for _ in range(self.max_object_sample_tries):
            x = (xmax - xmin - 2 * r) * np.random.random() + xmin + r
            y = (ymax - ymin - 2 * r) * np.random.random() + ymin + r
            yaw = 2.0 * np.pi * np.random.random()
            pose = Pose(x=x, y=y, z=0.0, yaw=yaw)
            if not self.check_occupancy(pose) and (
                ignore_robots or not self.collides_with_robots(pose, robot)
            ):
                return pose
        self.logger.warning("Could not sample pose.")
        return None

    ###################
    # Graph Utilities #
    ###################

    def graph_node_from_entity(
        self, entity_query, resolution_strategy="nearest", robot=None
    ):
        """
        Gets a graph node from an entity query, which could be any combination of
        room, hallway, location, object spawn, or object in the world, as well as
        their respective categories.
        For more information on the inputs, refer to the :func:`pyrobosim.utils.knowledge.query_to_entity` function.
        :param entity_query: List of entities (e.g., rooms or objects) or names/categories.
        :type entity_query: list[Entity/str]
        :param resolution_strategy: Resolution strategy to apply
        :type resolution_strategy: str
        :param robot: If set to a Robot instance, uses that robot for resolution strategy.
        :type robot: :class:`pyrobosim.core.robot.Robot`, optional
        :return: A graph node for the entity that meets the resolution strategy, or None.
        :rtype: :class:`pyrobosim.utils.search_graph.Node`
        """
        if isinstance(entity_query, Node):
            return entity_query
        elif isinstance(entity_query, str):
            # Try resolve an entity based on its name. If that fails, we assume it must be a category,
            # so try resolve it to a location or to an object by category.
            entity = self.get_entity_by_name(entity_query)
            if entity is None:
                entity = resolve_to_location(
                    self,
                    category=entity_query,
                    expand_locations=True,
                    resolution_strategy=resolution_strategy,
                    robot=robot,
                )
            if entity is None:
                entity = resolve_to_object(
                    self,
                    category=entity_query,
                    resolution_strategy=resolution_strategy,
                    robot=robot,
                    ignore_grasped=True,
                )
        else:
            entity = entity_query

        if isinstance(entity, ObjectSpawn) or isinstance(entity, Room):
            graph_nodes = entity.graph_nodes
        elif isinstance(entity, Hallway):
            graph_nodes = [entity.graph_nodes[0], entity.graph_nodes[-1]]

            # Special rule: If all the hallways connected to the room are closed, and the robot is not in the room or at the hallway,
            # remove the graph node from consideration.
            if (robot is not None) and (robot.location != entity):
                robot_in_start_room = entity.room_start.is_collision_free(
                    robot.get_pose()
                )
                if not robot_in_start_room:
                    room_accessible = False
                    for hall in self.get_hallways_attached_to_room(entity.room_start):
                        if hall.is_open:
                            room_accessible = True
                            break
                    if not room_accessible:
                        graph_nodes.remove(entity.graph_nodes[0])

                robot_in_end_room = entity.room_end.is_collision_free(robot.get_pose())
                if not robot_in_end_room:
                    room_accessible = False
                    for hall in self.get_hallways_attached_to_room(entity.room_end):
                        if hall.is_open:
                            room_accessible = True
                            break
                    if not room_accessible:
                        graph_nodes.remove(entity.graph_nodes[-1])

        elif isinstance(entity, Object):
            graph_nodes = entity.parent.graph_nodes
        elif isinstance(entity, Location):
            graph_nodes = entity.children[0].graph_nodes
        else:
            self.logger.warning(f"Cannot get graph node from {entity}")
            return None

        # Select a graph node using the same resolution strategy.
        graph_node = apply_resolution_strategy(
            graph_nodes, resolution_strategy, robot=robot
        )
        return graph_node
