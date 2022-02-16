from tkinter import NONE
import warnings

from .robot import Robot
from .hallway import Hallway
from .locations import Location

class World:
    def __init__(self, robot=Robot(), inflation_radius=None):
        # Define a robot and all its sensors
        self.robot = robot

        if inflation_radius is None:
            self.inflation_radius = self.robot.radius
        else:
            self.inflation_radius = inflation_radius

        # Define the world (rooms, locations, etc.)
        self.rooms = []
        self.hallways = []
        self.locations = []
        self.objects = []

        # Counters
        self.num_rooms = 0
        self.num_hallways = 0
        self.num_locations = 0
        self.location_instance_counts = {}

        # World bounds
        self.x_bounds = [0, 0]
        self.y_bounds = [0, 0]

    ############
    # Metadata #
    ############
    def set_metadata(self, locations=None, objects=None):
        """ Sets location and object metadata from the specified file """
        if locations is not None:
            Location.set_metadata(locations)
        # TODO Objects

    ##########################
    # World Building Methods #
    ##########################
    def add_room(self, room):
        """ Adds a room to the world """
        if room.name is None:
            room.name = f"room_{self.num_rooms}"
        self.rooms.append(room)
        self.num_rooms += 1
        self.update_bounds()

        # Update the room collision polygon based on the world inflation radius
        room.update_collision_polygon(self.inflation_radius)

    def remove_room(self, room_name):
        """ Removes a room from the world by name """
        for i, r in enumerate(self.rooms):
            if r.name == room_name:
                self.rooms.pop(i)
                self.num_rooms -= 1
                self.update_bounds()
                return
        warnings.warn(f"No room {room_name} found for removal")

    def add_hallway(self, room_start, room_end, width,
                    conn_method="auto", offset=0,
                    conn_angle=0, conn_points=[], color=None):
        """
        Adds a hallway from room_start to room_end, with a specified 
        width and options related to the Hallway class
        """
        # Parse inputs
        if isinstance(room_start, str):
            room_start = self.get_room_by_name(room_start)
        if isinstance(room_end, str):
            room_end = self.get_room_by_name(room_end)

        # Create the hallway
        h = Hallway(room_start, room_end, width,
                    conn_method=conn_method, offset=offset,
                    conn_angle=conn_angle, conn_points=conn_points,
                    color=color)

        # Do all the necessary bookkeeping
        self.hallways.append(h)
        room_start.hallways.append(h)
        room_start.update_visualization_polygon()
        room_end.hallways.append(h)
        room_end.update_visualization_polygon()
        self.num_hallways += 1
        h.update_collision_polygon(self.inflation_radius)

        # Finally, return the Hallway object
        return h

    def remove_hallway(self, room1, room2):
        """ TODO removes a hallway between two rooms. """
        raise NotImplementedError("Hallway removal not implemented.")

    def add_location(self, category, room, pose, name=None):
        """ Adds a location at the specified room """
        # Parse inputs
        if isinstance(room, str):
            room = self.get_room_by_name(room)
        if category not in self.location_instance_counts:
            self.location_instance_counts[category] = 0
        if name is None:
            name = f"{category}{self.location_instance_counts[category]}"
        self.location_instance_counts[category] +=1

        # Create the location
        # TODO: Check that it fits within the room, else error out
        loc = Location(category, parent=room, pose=pose, name=name)

        # Do all the necessary bookkeeping
        loc.update_collision_polygon(self.inflation_radius)
        room.locations.append(loc)
        room.update_collision_polygon(self.inflation_radius)
        self.locations.append(loc)
        self.num_locations += 1

        return loc

    def remove_location(self, loc):
        """ Cleanly removes a location from the world """
        # Parse inputs
        if isinstance(loc, str):
            loc = self.get_location_by_name(loc)

        if loc in self.locations:
            self.locations.remove(loc)
            self.num_locations -= 1
            self.location_instance_counts[loc.category] -= 1
            room = loc.parent
            room.locations.remove(loc)
            room.update_collision_polygon(self.inflation_radius)

    def update_bounds(self):
        """ 
        Updates the X and Y bounds of the world 
        TODO: If we're just adding a single room, we only need to check that one
        """
        for r in self.rooms:
            (xmin, ymin, xmax, ymax) = r.polygon.bounds
            self.x_bounds[0] = min(self.x_bounds[0], xmin)
            self.x_bounds[1] = max(self.x_bounds[1], xmax)
            self.y_bounds[0] = min(self.y_bounds[0], ymin)
            self.y_bounds[1] = max(self.y_bounds[1], ymax)

    def check_occupancy(self, pose):
        """
        Check if a pose in the world is occupied
        """
        # Loop through all the rooms
        for room in self.rooms + self.hallways:
            if room.is_collision_free(pose):
                return False

        # If we looped through all rooms, the pose is occupied
        return True

    ################################
    # Lookup Functionality Methods #
    ################################
    def get_room_names(self):
        """ Gets all room names. """
        return [r.name for r in self.rooms]

    def get_room_by_name(self, name):
        """ Gets a room object by its name. """
        names = self.get_room_names()
        if name in names:
            idx = names.index(name)
            return self.rooms[idx]
        else:
            warnings.warn(f"Room not found: {name}")
            return None

    def get_location_names(self):
        """ Gets all location names """
        return [loc.name for loc in self.locations]

    def get_location_by_name(self, name):
        """ Gets a location object by its name """
        names = self.get_location_names()
        if name in names:
            idx = names.index(name)
            return self.locations[idx]
        else:
            return None
