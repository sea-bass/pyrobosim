"""Main file containing the core world modeling tools."""

import itertools
import numpy as np
from typing import Any

import shapely
from shapely.geometry import Polygon
from shapely.ops import unary_union

from .hallway import Hallway
from .locations import Location, ObjectSpawn
from .objects import Object
from .room import Room
from .robot import Robot
from .types import Entity, EntityMetadata, InvalidEntityCategoryException, set_parent
from ..planning.actions import ExecutionResult, ExecutionStatus
from ..utils.logging import create_logger
from ..utils.pose import Pose
from ..utils.polygon import sample_from_polygon, transform_polygon
from ..utils.world_collision import check_occupancy


class World:
    """Core world modeling class."""

    def __init__(
        self,
        name: str = "world",
        inflation_radius: float = 0.0,
        object_radius: float = 0.0375,
        wall_height: float = 2.0,
    ) -> None:
        """
        Creates a new world model instance.

        :param name: Name of world model.
        :param inflation_radius: Inflation radius around entities (locations, walls, etc.), in meters.
        :param object_radius: Buffer radius around objects for collision checking, in meters.
        :param wall_height: Height of walls, in meters, for 3D model generation.
        """
        from ..gui.main import PyRoboSimMainWindow

        self.name = name
        self.wall_height = wall_height
        self.source_yaml: dict[str, Any] | None = None
        self.source_yaml_file: str | None = None
        self.logger = create_logger(self.name)

        # Connected apps
        self.gui: PyRoboSimMainWindow | None = None
        self.ros_node = None

        # World entities (robots, locations, objects, etc.)
        self.name_to_entity: dict[str, Entity] = {}
        self.robots: list[Robot] = []
        self.rooms: list[Room] = []
        self.hallways: list[Hallway] = []
        self.locations: list[Location] = []
        self.object_spawns: list[ObjectSpawn] = []
        self.objects: list[Object] = []
        self.set_metadata()

        # Counters
        self.num_rooms = 0
        self.num_hallways = 0
        self.num_locations = 0
        self.num_objects = 0
        self.hallway_instance_counts: dict[tuple[str, ...], int] = {}
        self.location_instance_counts: dict[str, int] = {}
        self.object_instance_counts: dict[str, int] = {}

        # World bounds, will be set by update_bounds()
        self.x_bounds = None
        self.y_bounds = None

        # Polygons for collision checking
        self.total_internal_polygon = Polygon()
        self.total_external_polygon = Polygon()

        # Other parameters
        # Max number of tries to sample object locations
        self.max_object_sample_tries = 1000

        # Distances for collision-aware navigation and sampling
        self.object_radius = object_radius
        self.set_inflation_radius(inflation_radius)

        self.logger.info("Created world.")

    def reset(self, deterministic: bool = False, seed: int = -1) -> bool:
        """
        Resets the world to its initial state.

        :param deterministic: If True, resets the world completely deterministically.
            Otherwise, sampled poses may be resampled randomly.
        :param seed: The seed to use for random number generation.
            This is useful for applications such as machine learning where you want to control randomness.
            If -1 (default), does not use a fixed seed.
        :return: True if the reset was successful, else False.
        """
        from ..core.yaml_utils import WorldYamlLoader, WorldYamlWriter

        if seed < 0:
            self.logger.info("Resetting world...")
        else:
            np.random.seed(seed)
            self.logger.info(f"Resetting world with seed {seed}...")

        if (not deterministic) and (self.source_yaml_file is not None):
            WorldYamlLoader().from_file(self.source_yaml_file, world=self)
        else:
            if self.source_yaml is None:
                self.source_yaml = WorldYamlWriter().to_dict(self)
            WorldYamlLoader().from_yaml(self.source_yaml, world=self)

        if self.gui is not None:
            self.gui.canvas.show()
            self.gui.update_buttons_signal.emit()
            self.gui.canvas.draw_signal.emit()

        self.logger.info("Reset world successfully.")
        return True  # No error handling yet

    def shutdown(self) -> None:
        """Cleanly shuts down the world."""
        for robot in self.robots:
            robot.stop_sensor_threads()

    ############
    # Metadata #
    ############
    def set_metadata(
        self,
        locations: str | list[str] | None = None,
        objects: str | list[str] | None = None,
    ) -> None:
        """
        Sets location and object metadata from the specified files.

        :param locations: Path(s) to location metadata YAML file(s).
        :param objects: Path(s) to object metadata YAML file(s).
        """
        # Clear out old metadata
        Location.clear_metadata()
        Object.clear_metadata()

        self.add_metadata(locations, objects)

    def add_metadata(
        self,
        locations: str | list[str] | None = None,
        objects: str | list[str] | None = None,
    ) -> None:
        """
        Add location and object metadata from the specified files, allowing
        additional metadata to be incrementally merged with the existing data.

        :param locations: Path(s) to location metadata YAML file(s).
        :param objects: Path(s) to object metadata YAML file(s).
        """
        if isinstance(locations, list):
            for location in locations:
                Location.add_metadata(location)
        elif isinstance(locations, str):
            Location.add_metadata(locations)

        if isinstance(objects, list):
            for object in objects:
                Object.add_metadata(object)
        elif isinstance(objects, str):
            Object.add_metadata(objects)

    def get_location_metadata(self) -> EntityMetadata:
        """
        Returns the location metadata associated with this world.

        :return: The location metadata.
        """
        return Location.metadata

    def get_object_metadata(self) -> EntityMetadata:
        """
        Returns the object metadata associated with this world.

        :return: The object metadata.
        """
        return Object.metadata

    ##########################
    # World Building Methods #
    ##########################
    def add_room(self, **room_config: Any) -> Room | None:
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
        """
        # If it's a room object, get it from the "room" named argument.
        # Else, create a room directly from the specified arguments.
        if "room" in room_config:
            room: Room = room_config["room"]
        else:
            # If the room name is empty, automatically name it.
            if "name" not in room_config:
                room_config["name"] = f"room{self.num_rooms}"
            room = Room(**room_config)

        # Check for duplicate names.
        if room.name in self.get_room_names():
            self.logger.warning(
                f"Room {room.name} already exists in the world. Cannot add."
            )
            return None

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
        self.update_polygons()

        room.add_graph_nodes()
        return room

    def remove_room(self, room: Room | str) -> bool:
        """
        Removes a room from the world.

        :param room: Room entity or name to remove.
        :return: True if the room was successfully removed, else False.
        """
        # Validate the input
        if isinstance(room, Room):
            resolved_room: Room | None = room
        elif isinstance(room, str):
            resolved_room = self.get_room_by_name(room)
        if resolved_room is None:
            self.logger.warning(f"No room {room} found for removal.")
            return False

        # Remove hallways associated with the room
        while len(resolved_room.hallways) > 0:
            self.remove_hallway(resolved_room.hallways[-1])

        # Remove locations in the room
        while len(resolved_room.locations) > 0:
            self.remove_location(resolved_room.locations[-1])

        # Remove the room itself
        self.rooms.remove(resolved_room)
        self.name_to_entity.pop(resolved_room.name)
        self.num_rooms -= 1
        self.update_bounds(entity=resolved_room, remove=True)

        return True

    def remove_all_rooms(self) -> None:
        """Cleanly removes all rooms from the world."""
        for room in reversed(self.rooms):
            self.remove_room(room)

    def add_hallway(self, **hallway_config: Any) -> Hallway | None:
        r"""
        Adds a hallway from with specified parameters related to the :class:`pyrobosim.core.hallway.Hallway` class.

        :param \*\*hallway_config: Keyword arguments describing the hallway.

            You can use ``hallway=Hallway(...)`` to directly pass in a :class:`pyrobosim.core.hallway.Hallway`
            object, or alternatively use the same keyword arguments you would use to create a Hallway object.

            You can also pass in the room names as the ``room_start`` and ``room_end`` arguments,
            and they will be resolved to actual room objects, if they exist in the world.

        :return: Hallway object if successfully created, else None.
        """
        # If it's a hallway object, get it from the "hallway" named argument.
        # Else, create a hallway directly from the specified arguments.
        if "hallway" in hallway_config:
            hallway: Hallway = hallway_config["hallway"]
        else:
            if isinstance(hallway_config["room_start"], str):
                room_start = self.get_room_by_name(hallway_config["room_start"])
                if room_start is None:
                    raise ValueError(
                        f"{hallway_config['room_start']} must be a valid start Room object."
                    )
                hallway_config["room_start"] = room_start

            if isinstance(hallway_config["room_end"], str):
                room_end = self.get_room_by_name(hallway_config["room_end"])
                if room_end is None:
                    raise ValueError(
                        f"{hallway_config['room_end']} must be a valid end Room object."
                    )
                hallway_config["room_end"] = room_end

            # If the hallway name is empty, automatically name it.
            reversed_name: str | None = None
            if "name" not in hallway_config:
                ordered_rooms = tuple(
                    sorted(
                        [
                            hallway_config["room_start"].name,
                            hallway_config["room_end"].name,
                        ]
                    )
                )
                if ordered_rooms not in self.hallway_instance_counts:
                    self.hallway_instance_counts[ordered_rooms] = 0
                    suffix = ""
                else:
                    suffix = f"_{self.hallway_instance_counts[ordered_rooms]}"

                hallway_config["name"] = (
                    f"hall_{ordered_rooms[0]}_{ordered_rooms[1]}{suffix}"
                )
                reversed_name = f"hall_{ordered_rooms[1]}_{ordered_rooms[0]}{suffix}"

            hallway = Hallway(**hallway_config)
            if reversed_name is not None:
                hallway.reversed_name = reversed_name

        # Check for duplicate names.
        if hallway.name in self.get_hallway_names():
            self.logger.warning(
                f"Hallway {hallway.name} already exists in the world. Cannot add."
            )
            return None

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
        ordered_rooms = tuple(sorted([hallway.room_start.name, hallway.room_end.name]))
        if not ordered_rooms in self.hallway_instance_counts:
            self.hallway_instance_counts[ordered_rooms] = 0
        self.hallway_instance_counts[ordered_rooms] += 1
        self.num_hallways += 1
        hallway.update_collision_polygons(self.inflation_radius)
        self.update_bounds(entity=hallway)
        self.update_polygons()

        hallway.add_graph_nodes()
        # Finally, return the Hallway object
        return hallway

    def remove_hallway(self, hallway: Hallway | str) -> bool:
        """
        Removes a hallway between two rooms.

        :param hallway: Hallway object or name to remove.
        :return: True if the hallway was successfully removed, else False.
        """
        # Validate the input
        if isinstance(hallway, Hallway):
            resolved_hallway: Hallway | None = hallway
        elif isinstance(hallway, str):
            resolved_hallway = self.get_hallway_by_name(hallway)
        if resolved_hallway is None:
            self.logger.warning(f"No hallway {hallway} found for removal.")
            return False

        # Remove the hallways from the world and relevant rooms.
        self.hallways.remove(resolved_hallway)
        self.num_hallways -= 1
        ordered_rooms = sorted(
            [resolved_hallway.room_start.name, resolved_hallway.room_end.name]
        )
        self.hallway_instance_counts[tuple(ordered_rooms)] -= 1
        self.name_to_entity.pop(resolved_hallway.name)
        if resolved_hallway.reversed_name in self.name_to_entity:
            self.name_to_entity.pop(resolved_hallway.reversed_name)
        for room in [resolved_hallway.room_start, resolved_hallway.room_end]:
            room.hallways.remove(resolved_hallway)
            room.update_collision_polygons()
            room.update_visualization_polygon()
            self.update_bounds(entity=resolved_hallway, remove=True)
        return True

    def remove_all_hallways(self, restart_numbering: bool = True) -> None:
        """
        Cleanly removes all hallways from the world.

        :param restart_numbering: If True, restarts numbering of all categories to zero.
        """
        for hallway in reversed(self.hallways):
            self.remove_hallway(hallway)
        if restart_numbering:
            self.hallway_instance_counts = {}

    def add_location(
        self, show: bool = True, **location_config: Any
    ) -> Location | None:
        r"""
        Adds a location at the specified parent entity, usually a room.

        If the location does not have a specified name, it will be given an
        automatic name using its category, e.g., ``"table0"``.

        :param show: If True (default), causes the GUI to be updated.
            This is mostly for internal usage to speed up reloading.
        :param \*\*location_config: Keyword arguments describing the location.

            You can use ``location=Location(...)`` to directly pass in a :class:`pyrobosim.core.location.Location`
            object, or alternatively use the same keyword arguments you would use to create a Location object.

            You can also pass in the room name as the ``parent`` argument, and it will be resolved to an actual room object, if it exists in the world.

        :return: Location object if successfully created, else None.
        """
        # If the category name is empty, use "location" as the base name.
        category = location_config.get("category", "location")
        if category not in self.location_instance_counts:
            self.location_instance_counts[category] = 0

        # If it's a location object, get it from the "location" named argument.
        # Else, create a location directly from the specified arguments.
        if "location" in location_config:
            loc: Location = location_config["location"]
        elif "parent" in location_config:
            if isinstance(location_config["parent"], str):
                location_config["parent"] = self.get_room_by_name(
                    location_config["parent"]
                )

            # If no name is specified, create one automatically.
            if "name" not in location_config:
                location_config["name"] = (
                    f"{category}{self.location_instance_counts[category]}"
                )

            try:
                loc = Location(**location_config)
            except InvalidEntityCategoryException as exception:
                self.logger.warning(str(exception))
                return None
        else:
            self.logger.warning("Location instance or parent must be specified.")
            return None

        assert isinstance(loc.parent, Room)

        # Check for duplicate names.
        if loc.name in self.get_location_names():
            self.logger.warning(
                f"Location {loc.name} already exists in the world. Cannot add."
            )
            return None

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
        loc.create_spawn_locations()
        loc.add_graph_nodes()
        loc.parent.locations.append(loc)
        loc.parent.update_collision_polygons(self.inflation_radius)
        self.locations.append(loc)
        self.object_spawns.extend(
            [spawn for spawn in loc.children if isinstance(spawn, ObjectSpawn)]
        )
        self.location_instance_counts[category] += 1
        self.num_locations += 1
        self.name_to_entity[loc.name] = loc
        for spawn in loc.children:
            self.name_to_entity[spawn.name] = spawn
        self.update_polygons()

        if show and self.gui is not None:
            self.gui.canvas.show_locations_signal.emit()
            self.gui.canvas.show_objects_signal.emit()
            self.gui.canvas.draw_signal.emit()
        return loc

    def update_location(
        self,
        location: Location | str,
        pose: Pose,
        room: Room | str | None = None,
        is_open: bool | None = None,
        is_locked: bool | None = None,
    ) -> bool:
        """
        Updates an existing location in the world.

        :param location: Location instance or name to update.
        :param pose: Pose of the location.
        :param room: Room instance or name. If None, uses the previous room.
        :param is_open: Whether the location should be open. If None, keeps the current state.
        :param is_locked: Whether the location should be locked. If None, keeps the current state.
        :return: True if the update was successful, else False.
        """
        if isinstance(location, str):
            resolved_location = self.get_location_by_name(location)
        elif isinstance(location, Location):
            resolved_location = location

        if resolved_location is None:
            self.logger.warning("Could not find location. Not updating.")
            return False

        assert isinstance(resolved_location.parent, Room)
        if room is None:
            room = resolved_location.parent
        else:
            if isinstance(room, str):
                room = self.get_room_by_name(room)

            if not isinstance(room, Room):
                self.logger.warning(
                    f"{room} did not resolve to a valid room for a location."
                )
                return False

        if is_open is not None:
            resolved_location.set_open(is_open, recursive=True)
        if is_locked is not None:
            resolved_location.is_locked = is_locked

        # Check that the location fits within the room and is not in collision with
        # other locations already in the room. Else, warn and do not add it.
        new_polygon = transform_polygon(resolved_location.raw_polygon, pose)
        is_valid_pose = new_polygon.within(room.polygon)
        for other_loc in room.locations:
            if resolved_location != other_loc:
                is_valid_pose = is_valid_pose and not new_polygon.intersects(
                    other_loc.polygon
                )
        if not is_valid_pose:
            self.logger.warning(
                f"Location {resolved_location.name} in collision. Cannot add to world."
            )
            return False

        # If we passed all checks, update the polygon.
        set_parent(resolved_location, room)
        resolved_location.set_pose(pose)
        resolved_location.create_polygons(self.inflation_radius)
        for spawn in resolved_location.children:
            assert isinstance(spawn, ObjectSpawn)
            spawn.set_pose_from_parent()
        if self.gui is not None:
            self.gui.canvas.show_locations_signal.emit()
            self.gui.canvas.show_objects_signal.emit()
            self.gui.canvas.draw_signal.emit()
        return True

    def remove_location(self, location: Location | str) -> bool:
        """
        Cleanly removes a location from the world.

        :param location: Location instance of name to remove.
        :return: True if the location was successfully removed, else False.
        """
        # Parse inputs
        if isinstance(location, str):
            resolved_location = self.get_location_by_name(location)
        if isinstance(location, Location):
            resolved_location = location

        if resolved_location is None:
            return False

        if resolved_location not in self.locations:
            self.logger.warning(
                f"{resolved_location} not found in world. Cannot remove."
            )
            return False

        # Remove objects at the location before removing the location.
        for spawn in resolved_location.children:
            while len(spawn.children) > 0:
                object_to_remove = spawn.children[-1]
                assert isinstance(object_to_remove, Object)
                self.remove_object(object_to_remove)

        # Remove the location.
        self.locations.remove(resolved_location)
        for spawn in resolved_location.children:
            assert isinstance(spawn, ObjectSpawn)
            self.object_spawns.remove(spawn)
        self.num_locations -= 1
        self.location_instance_counts[resolved_location.category or "location"] -= 1
        room = resolved_location.parent
        if isinstance(room, Room):
            room.locations.remove(resolved_location)
            room.update_collision_polygons(self.inflation_radius)
        self.name_to_entity.pop(resolved_location.name)
        for spawn in resolved_location.children:
            self.name_to_entity.pop(spawn.name)
        return True

    def remove_all_locations(self, restart_numbering: bool = True) -> None:
        """
        Cleanly removes all locations from the world.

        :param restart_numbering: If True, restarts numbering of all categories to zero.
        """
        for loc in reversed(self.locations):
            self.remove_location(loc)
        if restart_numbering:
            self.location_instance_counts = {}

    def open_location(self, location: Entity | str | None) -> ExecutionResult:
        """
        Opens a storage location or hallway between two rooms..

        :param location: Location or Hallway object to open, or its name.
        :return: An object describing the execution result.
        """
        # Validate the input
        if isinstance(location, str):
            location = self.get_entity_by_name(location)
        if not isinstance(location, (Location, Hallway)):
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

        location.set_open(True, recursive=True)
        location.update_visualization_polygon()
        self.update_polygons()
        if self.gui is not None:
            if isinstance(location, Hallway):
                self.gui.canvas.show_hallways_signal.emit()
            else:
                self.gui.canvas.show_locations_signal.emit()
            self.gui.update_buttons_signal.emit()
        return ExecutionResult(status=ExecutionStatus.SUCCESS)

    def close_location(
        self, location: Entity | str | None, ignore_robots: list[Robot] = []
    ) -> ExecutionResult:
        """
        Close a storage location or hallway.

        :param location: Location or Hallway object to close, or its name.
        :param ignore_robots: List of robots to ignore, for example the robot closing the hallway.
        :return: An object describing the execution result.
        """
        # Validate the input
        if isinstance(location, str):
            location = self.get_entity_by_name(location)
        if not isinstance(location, (Location, Hallway)):
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

        location.set_open(False, recursive=True)
        location.update_visualization_polygon()
        self.update_polygons()
        if self.gui is not None:
            if is_hallway:
                self.gui.canvas.show_hallways_signal.emit()
            else:
                self.gui.canvas.show_locations_signal.emit()
            self.gui.update_buttons_signal.emit()
        return ExecutionResult(status=ExecutionStatus.SUCCESS)

    def lock_location(self, location: Entity | str | None) -> ExecutionResult:
        """
        Locks a storage location or hallway.

        :param location: Location or Hallway object to lock, or its name.
        :return: An object describing the execution result.
        """
        # Validate the input
        if isinstance(location, str):
            location = self.get_entity_by_name(location)
        if not isinstance(location, (Location, Hallway)):
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

    def unlock_location(self, location: Entity | str | None) -> ExecutionResult:
        """
        Unlocks a storage location or hallway.

        :param location: Location or Hallway object to unlock, or its name.
        :return: An object describing the execution result.
        """
        # Validate the input
        if isinstance(location, str):
            location = self.get_entity_by_name(location)
        if not isinstance(location, (Location, Hallway)):
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

    def sample_object_spawn_pose(
        self, obj: Object, obj_spawn: ObjectSpawn, max_tries: int
    ) -> Pose | None:
        """
        Samples an object pose in a specific object spawn.

        :param obj: Object instance.
        :param obj_spawn: Object spawn instance.
        :param max_tries: The maximum number of tries to sample.
        :return: A sampled spawn pose, if one was found. Otherwise returns None.
        """
        for _ in range(max_tries):
            x_sample, y_sample = sample_from_polygon(obj_spawn.polygon)
            if (x_sample is None) or (y_sample is None):
                continue
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

    def add_object(self, show: bool = True, **object_config: Any) -> Object | None:
        r"""
        Adds an object to a specific location.

        If the object does not have a specified name, it will be given an
        automatic name using its category, e.g., ``"apple0"``.

        If the location contains multiple object spawns, one will be selected at random.

        :param show: If True (default), causes the GUI to be updated.
            This is mostly for internal usage to speed up reloading.
        :param \*\*object_config: Keyword arguments describing the object.

            You can use ``object=Object(...)`` to directly pass in a :class:`pyrobosim.core.objects.Object`
            object, or alternatively use the same keyword arguments you would use to create an Object instance.

            You can also pass in the parent entity name as the ``parent`` argument, and it will be resolved to an actual entity, if it exists in the world.

            If you pass in a list of parent entities, a random one will be selected.

        :return: Object instance if successfully created, else None.
        """
        # If the category name is empty, use "object" as the base name.
        category = object_config.get("category", "object")
        if category not in self.object_instance_counts:
            self.object_instance_counts[category] = 0

        # If it's an Object instance, get it from the "object" named argument.
        # Else, create an object directly from the specified arguments.
        if "object" in object_config:
            obj: Object = object_config["object"]
        else:
            parent = object_config.get("parent")

            if isinstance(parent, list):
                parent = np.random.choice(parent)

            if isinstance(parent, str):
                from ..utils.knowledge import query_to_entity

                parent = query_to_entity(
                    self, parent, mode="location", resolution_strategy="random"
                )
            elif (parent is None) and (len(self.locations) > 0):
                # If no parent was specified, spawn at a random location.
                parent = np.random.choice(self.object_spawns)

            # If the parent is a location object, pick an object spawn at random.
            # Otherwise, if it's an object spawn, use that entity as is.
            if isinstance(parent, Location):
                parent = np.random.choice(parent.children)

            if not isinstance(parent, ObjectSpawn):
                parent_arg = object_config.get("parent", None)
                self.logger.warning(
                    f"Parent location {parent_arg} did not resolve to a valid location for an object."
                )
                return None

            # If no name is specified, create one automatically.
            if "name" not in object_config:
                object_config["name"] = (
                    f"{category}{self.object_instance_counts[category]}"
                )

            object_config["parent"] = parent
            object_config["inflation_radius"] = self.object_radius
            try:
                obj = Object(**object_config)
            except InvalidEntityCategoryException as exception:
                self.logger.warning(str(exception))
                return None

        # Check for duplicate names.
        if obj.name in self.get_object_names():
            self.logger.warning(
                f"Object {obj.name} already exists in the world. Cannot add."
            )
            return None

        # If no pose is specified, sample a valid one.
        obj_spawn = obj.parent
        assert isinstance(obj_spawn, ObjectSpawn)
        if "pose" not in object_config:
            pose_sample = self.sample_object_spawn_pose(
                obj, obj_spawn, self.max_object_sample_tries
            )

            if pose_sample is not None:
                obj.set_pose(pose_sample)
                obj.create_polygons()
            else:
                self.logger.warning(
                    f"Could not sample valid pose to add object {obj.name}."
                )
                return None

        # If a pose was specified, collision check it.
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
        self.object_instance_counts[category] += 1
        if show and self.gui is not None:
            self.gui.canvas.show_objects_signal.emit()
        return obj

    def update_object(
        self,
        obj: Object | str,
        loc: Location | ObjectSpawn | str | None = None,
        pose: Pose | None = None,
    ) -> bool:
        """
        Updates an existing object in the world.

        :param obj: Object instance or name to update.
        :param loc: Location or object spawn instance or name. If none, uses the previous location.
        :param pose: Pose of the object. If none is specified, it will be sampled.
        :return: True if the update was successful, else False.
        """
        if isinstance(obj, str):
            resolved_object = self.get_object_by_name(obj)
        elif isinstance(obj, Object):
            resolved_object = obj

        if resolved_object is None:
            self.logger.warning("Could not find object. Not updating.")
            return False

        if loc is not None:
            # Find an object spawn that matches the specified location.
            # If it's a string, get the location name.
            if isinstance(loc, str):
                entity = self.get_entity_by_name(loc)
                if isinstance(entity, (Location, ObjectSpawn)):
                    loc = entity

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
                    resolved_object, obj_spawn, self.max_object_sample_tries
                )
            if pose is None:
                self.logger.warning("Cannot sample a valid spawn pose.")
                return False

            set_parent(resolved_object, obj_spawn)

        if pose is not None:
            resolved_object.set_pose(pose)
            resolved_object.create_polygons()

        return True

    def remove_object(self, obj: Object | str, show: bool = True) -> bool:
        """
        Cleanly removes an object from the world.

        :param loc: Object instance or name to remove.
        :param show: If True (default), causes the GUI to be updated.
            This is mostly for internal usage to speed up reloading.
        :return: True if the object was successfully removed, else False.
        """
        if isinstance(obj, str):
            resolved_object = self.get_object_by_name(obj)
        elif isinstance(obj, Object):
            resolved_object = obj

        if resolved_object not in self.objects:
            self.logger.warning("Could not find object. Not removing.")
            return False

        self.objects.remove(resolved_object)
        self.name_to_entity.pop(resolved_object.name)
        self.num_objects -= 1
        if resolved_object.parent is not None:
            resolved_object.parent.children.remove(resolved_object)
        if show and self.gui is not None:
            self.gui.canvas.show_objects_signal.emit()
        return True

    def remove_all_objects(self, restart_numbering: bool = True) -> None:
        """
        Cleanly removes all objects from the world.

        :param restart_numbering: If True, restarts numbering of all categories to zero.
        """
        for obj in reversed(self.objects):
            # Only update the UI on the final object to be removed.
            self.remove_object(obj, show=(len(self.objects) == 1))
        if restart_numbering:
            self.object_instance_counts = {}

    def add_robot(
        self,
        robot: Robot,
        loc: Entity | str | list[Entity | str] | None = None,
        pose: Pose | None = None,
        show: bool = True,
    ) -> None:
        """
        Adds a robot to the world given either a world entity and/or pose.

        :param robot: Robot instance to add to the world.
        :param loc: World entity instance or name to place the robot.
            You can also pass in a list, in which case a random entity will be selected.
        :param pose: Pose at which to add the robot. If not specified, will be sampled.
        :param show: If True (default), causes the GUI to be updated.
            This is mostly for internal usage to speed up reloading.
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

        robot_pose: Pose | None = None
        if isinstance(loc, list):
            resolved_loc: Entity | str | None = np.random.choice(loc)
        else:
            resolved_loc = loc

        if resolved_loc is None:
            if pose is None:
                # If nothing is specified, sample any valid location in the world
                robot_pose = self.sample_free_robot_pose_uniform(
                    robot, ignore_robots=False
                )
                if robot_pose is None:
                    self.logger.warning("Unable to sample free pose.")
            else:
                # Validate that the pose is unoccupied
                if check_occupancy((pose.x, pose.y), self):
                    self.logger.warning(f"{pose} is occupied.")
                robot_pose = pose
            # If we have a valid pose, extract its location
            if robot_pose is not None:
                resolved_loc = self.get_location_from_pose(robot_pose)

        else:
            # First, validate that the location is valid for a robot
            if isinstance(resolved_loc, str):
                from ..utils.knowledge import query_to_entity

                resolved_loc = query_to_entity(
                    self, resolved_loc, mode="location", resolution_strategy="random"
                )

            if isinstance(resolved_loc, (Room, Hallway)):
                if pose is None:
                    # Sample a pose in the location
                    x_sample, y_sample = sample_from_polygon(
                        resolved_loc.internal_collision_polygon,
                        max_tries=self.max_object_sample_tries,
                    )
                    if (x_sample is None) or (y_sample is None):
                        self.logger.warning(
                            f"Could not sample pose in {resolved_loc.name}."
                        )
                    else:
                        yaw_sample = np.random.uniform(-np.pi, np.pi)
                        robot_pose = Pose(x=x_sample, y=y_sample, z=0.0, yaw=yaw_sample)
                else:
                    # Validate that the pose is unoccupied and in the right location
                    if not resolved_loc.is_collision_free(pose):
                        self.logger.warning(f"{pose} is occupied")
                    robot_pose = pose

            elif isinstance(resolved_loc, (Location, ObjectSpawn)):
                if isinstance(resolved_loc, Location):
                    # NOTE: If you don't want a random object spawn, use the object spawn as the input location.
                    resolved_loc = np.random.choice(resolved_loc.children)

                assert isinstance(resolved_loc, ObjectSpawn)
                if (pose is not None) and (pose in resolved_loc.nav_poses):
                    # Slim chance of this happening lol
                    robot_pose = pose
                else:
                    robot_pose = np.random.choice(resolved_loc.nav_poses)
            else:
                self.logger.warning(f"Invalid location specified: {loc}.")

        # If we got a valid location / pose combination, add the robot
        if robot_pose is not None:
            robot.location = resolved_loc
            robot.set_pose(robot_pose)
            robot.world = self
            if robot.path_planner is not None:
                robot.path_planner.reset()
            self.robots.append(robot)
            self.name_to_entity[robot.name] = robot
        else:
            self.logger.warning("Could not add robot.")
            self.set_inflation_radius(old_inflation_radius)

        # Update robot's total_internal_polygon and initialize planner.
        robot.update_polygons()
        if robot.path_planner is not None:
            robot.path_planner.reset()

        if show and self.gui is not None:
            self.gui.canvas.show_robots_signal.emit()
        if self.ros_node is not None:
            self.ros_node.add_robot_ros_interfaces(robot)

    def remove_robot(
        self, robot: Robot | str, show: bool = True, remove_ros_interfaces: bool = True
    ) -> bool:
        """
        Removes a robot from the world.

        :param robot: Robot instance or name to remove.
        :param show: If True (default), causes the GUI to be updated.
            This is mostly for internal usage to speed up reloading.
        :param remove_ros_interfaces: If True (default), and the world has a ROS interface,
            it removes them. This is configurable since resetting the world is prone to a rclpy bug.
            See https://github.com/ros2/rclpy/issues/1206 for more details.
        :return: True if the robot was successfully removed, else False.
        """
        if isinstance(robot, Robot):
            resolved_robot: Robot | None = robot
        elif isinstance(robot, str):
            resolved_robot = self.get_robot_by_name(robot)
        if resolved_robot is None:
            self.logger.warning(f"Could not find robot {robot} to remove.")
            return False

        self.robots.remove(resolved_robot)
        self.name_to_entity.pop(resolved_robot.name)
        if show and self.gui is not None:
            self.gui.canvas.show_robots_signal.emit()
        if (self.ros_node is not None) and remove_ros_interfaces:
            self.ros_node.remove_robot_ros_interfaces(resolved_robot)

        # Find the new max inflation radius and revert it.
        new_inflation_radius = max(
            [other_robot.radius for other_robot in self.robots]
            + [self.inflation_radius]
        )
        if new_inflation_radius != self.inflation_radius:
            self.set_inflation_radius(new_inflation_radius)
        return True

    def remove_all_robots(self, remove_ros_interfaces: bool = True) -> None:
        """
        Cleanly removes all robots from the world.

        :param remove_ros_interfaces: If True (default), and the world has a ROS interface,
            it removes them. This is configurable since resetting the world is prone to a rclpy bug.
            See https://github.com/ros2/rclpy/issues/1206 for more details.
        """
        for robot in reversed(self.robots):
            # Only update the UI on the last robot to remove.
            self.remove_robot(
                robot,
                show=(len(self.robots) == 1),
                remove_ros_interfaces=remove_ros_interfaces,
            )

    def set_inflation_radius(self, inflation_radius: float = 0.0) -> None:
        """
        Sets world inflation radius.

        :param inflation_radius: Inflation radius, in meters.
        """
        self.inflation_radius = inflation_radius
        for loc in self.locations:
            loc.update_collision_polygon(self.inflation_radius)
        for room in self.rooms:
            room.update_collision_polygons(self.inflation_radius)
        for hallway in self.hallways:
            hallway.update_collision_polygons(self.inflation_radius)
        self.update_polygons()

    def update_bounds(self, entity: Entity, remove: bool = False) -> None:
        """
        Updates the X and Y bounds of the world.

        :param entity: The entity that is being added or removed
        :param remove: Specifies if the update is due to removal of an entity.
        """
        if isinstance(entity, (Room, Hallway)):
            (xmin, ymin, xmax, ymax) = entity.polygon.bounds

            # Simply assign the bounds when adding the first entity.
            if (self.x_bounds is None) or (self.y_bounds is None):
                self.x_bounds = [xmin, xmax]  # type: ignore[assignment]
                self.y_bounds = [ymin, ymax]  # type: ignore[assignment]
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
    def get_room_names(self) -> list[str]:
        """
        Gets all room names in the world.

        :return: List of all room names.
        """
        return [room.name for room in self.rooms]

    def get_room_by_name(self, name: str) -> Room | None:
        """
        Gets a room object by its name.

        :param name: Name of room.
        :return: Room instance matching the input name, or ``None`` if not valid.
        """
        if name not in self.name_to_entity:
            self.logger.warning(f"Room not found: {name}")
            return None

        entity = self.name_to_entity[name]
        if not isinstance(entity, Room):
            self.logger.warning(f"Entity {name} found but it is not a Room.")
            return None

        return entity

    def get_hallway_names(self) -> list[str]:
        """
        Gets all hallway names.

        :return: List of all hallway names.
        """
        return [hall.name for hall in self.hallways]

    def get_hallway_by_name(self, name: str) -> Hallway | None:
        """
        Gets a hallway object by its name.

        :param name: Name of hallway.
        :return: Hallway instance matching the input name, or ``None`` if not valid.
        """
        if name not in self.name_to_entity:
            self.logger.warning(f"Hallway not found: {name}")
            return None

        entity = self.name_to_entity[name]
        if not isinstance(entity, Hallway):
            self.logger.warning(f"Entity {name} found but it is not a Hallway.")
            return None

        return entity

    def get_hallways_from_rooms(
        self, room1: Room | str, room2: Room | str
    ) -> list[Hallway]:
        """
        Returns a list of hallways between two rooms.

        :param room1: Instance or name of first room.
        :param room2: Instance or name of second room.
        :return: List of hallways.
        """
        # Validate room input
        resolved_room1: Room | None = None
        if isinstance(room1, str):
            resolved_room1 = self.get_room_by_name(room1)
        else:
            resolved_room1 = room1
        if resolved_room1 is None:
            self.logger.warning(f"Invalid room1 specified: {room1}.")
            return []

        resolved_room2: Room | None = None
        if isinstance(room2, str):
            resolved_room2 = self.get_room_by_name(room2)
        else:
            resolved_room2 = room2
        if resolved_room2 is None:
            self.logger.warning(f"Invalid room2 specified: {room2}.")
            return []

        # Now search through the hallways and add any valid ones to the list
        hallways = []
        for hall in resolved_room1.hallways:
            is_valid_hallway = (
                (hall.room_start == resolved_room1)
                and (hall.room_end == resolved_room2)
            ) or (
                (hall.room_start == resolved_room2)
                and (hall.room_end == resolved_room1)
            )
            if is_valid_hallway:
                hallways.append(hall)
        return hallways

    def get_hallways_attached_to_room(self, room: Room | str) -> list[Hallway]:
        """
        Returns a list of hallways attached to a specific room.

        :param room: Instance or name of room.
        :return: List of hallways.
        """
        # Validate room input
        resolved_room: Room | None = None
        if isinstance(room, str):
            resolved_room = self.get_room_by_name(room)
        else:
            resolved_room = room
        if resolved_room is None:
            self.logger.warning(f"Invalid room specified: {room}.")
            return []

        # Now search through the hallways and add any valid ones to the list
        hallways = []
        for hall in resolved_room.hallways:
            is_valid_hallway = (hall.room_start == resolved_room) or (
                hall.room_end == resolved_room
            )
            if is_valid_hallway:
                hallways.append(hall)
        return hallways

    def get_locations(self, category_list: list[str] | None = None) -> list[Location]:
        """
        Gets all locations, optionally filtered by category.

        :param category_list: Optional list of categories to filter by.
        :return: List of locations that match the input category list, if specified.
        """
        if not category_list:
            return [loc for loc in self.locations]
        else:
            return [loc for loc in self.locations if loc.category in category_list]

    def get_location_names(self, category_list: list[str] | None = None) -> list[str]:
        """
        Gets all location names, optionally filtered by category.

        :param category_list: Optional list of categories to filter by.
        :return: List of location names that match the input category list, if specified.
        """
        if not category_list:
            return [loc.name for loc in self.locations]
        else:
            return [loc.name for loc in self.locations if loc.category in category_list]

    def get_location_by_name(self, name: str) -> Location | None:
        """
        Gets a location object by its name.

        :param name: Name of location.
        :return: Location instance matching the input name, or ``None`` if not valid.
        """
        if name not in self.name_to_entity:
            self.logger.warning(f"Location not found: {name}")
            return None

        entity = self.name_to_entity[name]
        if not isinstance(entity, Location):
            self.logger.warning(f"Entity {name} found but it is not a Location.")
            return None

        return entity

    def get_location_in_room(self, pose: Pose, room: Room) -> Entity | None:
        """
        Gets a location in a room given an input pose.

        :param pose: Input pose.
        :param room: The reference room object.
        :return: Entity matching the input pose and room, or ``None`` if not valid.
            If valid, this could be the Room itself, a location, or an object spawn.
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

    def get_location_in_hallway(self, pose: Pose, hallway: Hallway) -> Hallway | None:
        """
        Gets a location in a hallway given an input pose.

        :param pose: Input pose.
        :param hallway: The reference hallway object.
        :return: Entity matching the input pose and hallway, or ``None`` if not valid.
            This could either be the Hallway itself, or ``None``.
        """
        for nav_pose in hallway.nav_poses:
            if pose.is_approx(nav_pose):
                return hallway
        if hallway.is_collision_free(pose):
            return hallway
        return None

    def get_location_from_pose(
        self, pose: Pose, prev_location: Entity | None = None
    ) -> Entity | None:
        """
        Gets the name of a location given a pose.

        :param pose: Input pose.
        :param prev_location: The robot's previous location.
            This helps optimize this function by first checking the robot's previous location.
        :return: Entity matching the input pose, or ``None`` if not valid.
            If valid, this could be a room, hallway, or object spawn.
        """
        # Check the previous location.
        prev_room: Room | None = None
        prev_hallways: list[Hallway] = []
        if prev_location is not None:
            if isinstance(prev_location, ObjectSpawn) and (
                prev_location.parent is not None
            ):
                prev_location = prev_location.parent.parent

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
                hall_loc = self.get_location_in_hallway(pose, prev_location)
                if hall_loc is not None:
                    return hall_loc

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

    def get_object_spawns(
        self, category_list: list[str] | None = None
    ) -> list[ObjectSpawn]:
        """
        Gets all object spawn locations, optionally filtered by category.

        :param category_list: Optional list of categories to filter by.
        :return: List of object spawn locations that match the input category
            list, if specified.
        """
        spawn_list = []
        for loc in self.locations:
            if (category_list is None) or (loc.category in category_list):
                spawn_list.extend(
                    [child for child in loc.children if isinstance(child, ObjectSpawn)]
                )
        return spawn_list

    def get_object_spawn_names(
        self, category_list: list[str] | None = None
    ) -> list[str]:
        """
        Gets all object spawn location names, optionally filtered by category.

        :param category_list: Optional list of categories to filter by.
        :return: List of object spawn location names that match the input
            category list, if specified.
        """
        spawn_name_list = []
        for loc in self.locations:
            if not category_list or loc.category in category_list:
                spawn_name_list.extend([spawn.name for spawn in loc.children])
        return spawn_name_list

    def get_objects(self, category_list: list[str] | None = None) -> list[Object]:
        """
        Gets all objects in the world, optionally filtered by category.

        :param name: Name of location.
        :param category_list: Optional list of category names to filter by.
        :return: List of object that match the input category list, if specified.
        """
        if not category_list:
            return self.objects
        else:
            return [o for o in self.objects if o.category in category_list]

    def get_object_names(self, category_list: list[str] | None = None) -> list[str]:
        """
        Gets all object names in the world, optionally filtered by category.

        :param category_list: Optional list of categories to filter by.
        :return: List of object names that match the input category list, if specified.
        """
        if not category_list:
            return [o.name for o in self.objects]
        else:
            return [o.name for o in self.objects if o.category in category_list]

    def get_object_by_name(self, name: str) -> Object | None:
        """
        Gets an object by its name.

        :param name: Name of object.
        :return: Object instance matching the input name, or ``None`` if not valid.
        """
        if name not in self.name_to_entity:
            return None

        entity = self.name_to_entity[name]
        if not isinstance(entity, Object):
            self.logger.warning(f"Entity {name} found but it is not an Object.")
            return None

        return entity

    def get_robot_names(self) -> list[str]:
        """
        Gets all robot names in the world.

        :return: List of robot names.
        """
        return [robot.name for robot in self.robots]

    def get_robot_by_name(self, name: str) -> Robot | None:
        """
        Gets a robot by its name.

        :param name: Name of robot.
        :return: Robot instance matching the input name, or ``None`` if not valid.
        """
        if name not in self.name_to_entity:
            return None

        entity = self.name_to_entity[name]
        if not isinstance(entity, Robot):
            self.logger.warning(f"Entity {name} found but it is not a Robot.")
            return None

        return entity

    def get_entity_by_name(self, name: str) -> Entity | None:
        """
        Gets any world entity by its name.

        :param name: Name of entity.
        :return: Entity object instance matching the input name, or ``None`` if not valid.
        """
        return self.name_to_entity.get(name)

    def get_pose_relative_to(self, pose: Pose, entity: Entity | str) -> Pose:
        """
        Given a relative pose to an entity, and the entity itself, gets the absolute pose.

        :param pose: The pose specified with respect to the entity.
        :param entity: The entity, or entity name:
        :return: Absolute pose computed by transforming the input pose to the entity frame.
        """
        # Look for the target entity if specified by name.
        if isinstance(entity, str):
            grounded_entity = self.get_entity_by_name(entity)
            if grounded_entity is None:
                raise ValueError(f"Could not find entity by name: {entity}")

            entity = grounded_entity

        # Validate that the entity is of one of the supported types.
        if not isinstance(entity, (Room, Location, ObjectSpawn, Object, Robot)):
            raise TypeError(f"Invalid entity type: {type(entity)}")

        # Extract the pose from the entity and use it to transform the given pose
        # to the entity frame.
        # Note that the rotation is applied separately from the translation.
        new_transform = (
            pose.get_translation_matrix() @ entity.pose.get_transform_matrix()
        )
        new_transform[:3, :3] = pose.get_rotation_matrix() @ new_transform[:3, :3]
        return Pose.from_transform(new_transform)

    #######################
    # Occupancy utilities #
    #######################
    def update_polygons(self) -> None:
        """
        Updates the world's collision polygons when an entity is added or removed.
        """
        self.total_internal_polygon = unary_union(
            [
                entity.internal_collision_polygon
                for entity in itertools.chain(self.rooms, self.hallways)
            ]
        ).difference(
            unary_union(
                [
                    hall.inflated_closed_polygon
                    for hall in self.hallways
                    if not hall.is_open
                ]
            )
        )
        shapely.prepare(self.total_internal_polygon)

        self.total_external_polygon = unary_union(
            [entity.polygon for entity in itertools.chain(self.rooms, self.hallways)]
        ).difference(
            unary_union(
                [loc.polygon for loc in self.locations]
                + [hall.closed_polygon for hall in self.hallways if not hall.is_open]
            )
        )
        shapely.prepare(self.total_external_polygon)

        for robot in self.robots:
            robot.update_polygons()

    def collides_with_robots(self, pose: Pose, robot: Robot | None = None) -> bool:
        """
        Checks if a pose collides with robots in the world.
        Currently assumes that robots are circles, so we can do simple checks.
        If this changes, should account for polygon collisions.

        :param pose: Candidate pose to check.
        :param robot: Robot instance, if specified.
        :return: True if the pose collides with a robot besides the input.
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

    def sample_free_robot_pose_uniform(
        self,
        robot: Robot | None = None,
        ignore_robots: bool = True,
    ) -> Pose | None:
        """
        Sample an unoccupied robot pose in the world.
        This is done using uniform sampling within the world X-Y bounds and rejecting
        any samples that are in collision with entities in the world.
        If no valid samples could be found within the `max_object_sample_tries` instance
        attribute, this will return ``None``.

        :param robot: Robot instance, if specified.
        :param ignore_robots: If True, ignore collisions with other robots.
        :return: Collision-free pose if found, else ``None``.
        """
        if (self.x_bounds is None) or (self.y_bounds is None):
            self.logger.error("Cannot sample poses if world bounds are not set.")
            return None

        xmin, xmax = self.x_bounds
        ymin, ymax = self.y_bounds
        r = self.inflation_radius if robot is None else robot.radius

        for _ in range(self.max_object_sample_tries):
            x = (xmax - xmin - 2 * r) * np.random.random() + xmin + r
            y = (ymax - ymin - 2 * r) * np.random.random() + ymin + r
            yaw = 2.0 * np.pi * np.random.random()
            pose = Pose(x=x, y=y, z=0.0, yaw=yaw)
            if not check_occupancy(pose, self, robot) and (
                ignore_robots or not self.collides_with_robots(pose, robot)
            ):
                return pose
        self.logger.warning("Could not sample pose.")
        return None
