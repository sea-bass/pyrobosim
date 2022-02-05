import warnings

from .robot import Robot

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
        self.locations = []
        self.hallways = []
        self.objects = []

        # Counters
        self.num_rooms = 0

        self.x_bounds = [0, 0]
        self.y_bounds = [0, 0]

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
        for room in self.rooms:
            if room.is_collision_free(pose):
                return False
        
        # If we looped through all rooms, the pose is occupied
        return True