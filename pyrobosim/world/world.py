import itertools
import numpy as np
import threading
import warnings

from .robot import Robot
from .hallway import Hallway
from .locations import Location, ObjectSpawn
from .objects import Object
from .room import Room
from ..navigation.search_graph import SearchGraph, Node
from ..navigation.execution import execute_with_linear_trajectory
from ..utils.knowledge import resolve_to_location, resolve_to_object
from ..utils.pose import Pose
from ..utils.polygon import inflate_polygon, sample_from_polygon, transform_polygon
from ..utils.trajectory import fill_path_yaws

class World:
    def __init__(self, inflation_radius=0.0, object_radius=0.05):
        # Connected apps
        self.has_gui = False
        self.gui = None
        self.has_ros_node = False
        self.ros_node = False

        # Robot
        self.robot = None
        self.has_robot = False

        # World entities (rooms, locations, objects, etc.)
        self.rooms = []
        self.hallways = []
        self.locations = []
        self.objects = []

        # Counters
        self.num_rooms = 0
        self.num_hallways = 0
        self.num_locations = 0
        self.num_objects = 0
        self.location_instance_counts = {}
        self.object_instance_counts = {}

        # World bounds
        self.x_bounds = [0, 0]
        self.y_bounds = [0, 0]

        # Search graph for navigation
        self.search_graph = None
        self.current_path = None

        # Other parameters
        self.max_object_sample_tries = 1000 # Max number of tries to sample object locations

        # Distances for collision-aware navigation and sampling
        self.object_radius = object_radius
        self.set_inflation_radius(inflation_radius)


    ############
    # Metadata #
    ############
    def set_metadata(self, locations=None, objects=None):
        """ Sets location and object metadata from the specified file """
        if locations is not None:
            Location.set_metadata(locations)
        if objects is not None:
            Object.set_metadata(objects)

    def set_inflation_radius(self, inflation_radius=0.0):
        """ Sets inflation radius """
        self.inflation_radius = inflation_radius
        for loc in self.locations:
            loc.update_collision_polygon(self.inflation_radius)
        for entity in itertools.chain(self.rooms, self.hallways):
            entity.update_collision_polygons(self.inflation_radius)
        self.update_search_graph()

    ##########################
    # World Building Methods #
    ##########################
    def add_room(self, room):
        """ Adds a room to the world """
        if room.name is None:
            room.name = f"room_{self.num_rooms}"

        # Check if the room collides with any other rooms or hallways
        is_valid_pose = True
        for other_loc in self.rooms + self.hallways:
            is_valid_pose = is_valid_pose and not \
                room.external_collision_polygon.intersects(
                    other_loc.external_collision_polygon)
        if not is_valid_pose:
            warnings.warn(f"Room {room.name} in collision. Cannot add to world.")
            return None

        self.rooms.append(room)
        self.num_rooms += 1
        self.update_bounds()

        # Update the room collision polygon based on the world inflation radius
        room.update_collision_polygons(self.inflation_radius)

        # Update the search graph, if any
        if self.search_graph is not None:
            room.add_graph_nodes()
            self.search_graph.add(room.graph_nodes, autoconnect=True)

    def remove_room(self, room_name):
        """ Removes a room from the world by name """
        for i, room in enumerate(self.rooms):
            if room.name == room_name:
                self.rooms.pop(i)
                self.num_rooms -= 1
                self.update_bounds()
                
                # Update the search graph, if any
                if self.search_graph is not None:
                    self.search_graph.remove(room.graph_nodes)
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

        # Check if the hallway collides with any other rooms or hallways
        is_valid_pose = True
        for other_loc in self.rooms + self.hallways:
            if (other_loc == room_start) or (other_loc == room_end):
                continue
            is_valid_pose = is_valid_pose and not \
                h.external_collision_polygon.intersects(
                    other_loc.external_collision_polygon)
        if not is_valid_pose:
            warnings.warn(f"Hallway {h.name} in collision. Cannot add to world.")
            return None

        # Do all the necessary bookkeeping
        self.hallways.append(h)
        room_start.hallways.append(h)
        room_start.update_visualization_polygon()
        room_end.hallways.append(h)
        room_end.update_visualization_polygon()
        self.num_hallways += 1
        h.update_collision_polygons(self.inflation_radius)

        # Update the search graph, if any
        if self.search_graph is not None:
            h.add_graph_nodes()
            self.search_graph.add(h.graph_nodes, autoconnect=True)

        # Finally, return the Hallway object
        return h

    def remove_hallway(self, hallway):
        """ Removes a hallway between two rooms. """
        # Validate the input
        if not hallway in self.hallways:
            warnings.warn("Invalid hallway specified.")
            return

        # Remove the hallways from the world and relevant rooms.
        self.hallways.remove(hallway)
        for r in [hallway.room_start, hallway.room_end]:
            r.hallways.remove(hallway)
            r.update_collision_polygons()
            r.update_visualization_polygon()

    def add_location(self, category, room, pose, name=None):
        """ Adds a location at the specified room """
        # Parse inputs
        if isinstance(room, str):
            room = self.get_room_by_name(room)
        if category not in self.location_instance_counts:
            self.location_instance_counts[category] = 0
        if name is None:
            name = f"{category}{self.location_instance_counts[category]}"

        # Create the location
        loc = Location(category, parent=room, pose=pose, name=name)

        # Check that the location fits within the room and is not in collision with
        # other locations already in the room. Else, warn and do not add it.
        is_valid_pose = loc.polygon.within(room.polygon)
        for other_loc in room.locations:
            is_valid_pose = is_valid_pose and not loc.polygon.intersects(other_loc.polygon)
        if not is_valid_pose:
            warnings.warn(f"Location {loc.name} in collision. Cannot add to world.")
            return None

        # Do all the necessary bookkeeping
        loc.update_collision_polygon(self.inflation_radius)
        room.locations.append(loc)
        room.update_collision_polygons(self.inflation_radius)
        self.locations.append(loc)
        self.location_instance_counts[category] +=1
        self.num_locations += 1

        # Update the search graph, if any
        if self.search_graph is not None:
            loc.add_graph_nodes()
            for spawn in loc.children:
                self.search_graph.add(spawn.graph_nodes, autoconnect=True)

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
            room.update_collision_polygons(self.inflation_radius)

    def add_object(self, category, loc, pose=None, name=None):
        """
        Adds an object to a location at the specified pose
        """
        # If no name is specified, create one automatically
        if name is None:
            if category not in self.object_instance_counts:
                self.object_instance_counts[category] = 0
            name = f"{category}{self.object_instance_counts[category]}"
        self.object_instance_counts[category] +=1

        # If it's a string, get the location name
        if isinstance(loc, str):
            loc = self.get_location_by_name(loc)
        # If it's a location object, pick an object spawn at random
        if isinstance(loc, Location):
            obj_spawn = np.random.choice(loc.children)
        else:
            obj_spawn = loc

        # Create the object
        obj = Object(category=category, name=name, parent=obj_spawn, pose=pose)
        
        # If no pose is specified, sample a valid one
        if pose is None:
            obj_added = False
            for _ in range(self.max_object_sample_tries):
                if isinstance(loc, Location):
                    obj_spawn = np.random.choice(loc.children)
                x_sample, y_sample = sample_from_polygon(obj_spawn.polygon)
                yaw_sample = np.random.uniform(-np.pi, np.pi)
                pose_sample = Pose(x=x_sample, y=y_sample, yaw=yaw_sample)
                poly = inflate_polygon(
                    transform_polygon(obj.polygon, pose_sample), self.object_radius)
                
                is_valid_pose = poly.within(obj_spawn.polygon)
                for other_obj in obj_spawn.children:
                    is_valid_pose = is_valid_pose and not poly.intersects(other_obj.polygon)
                if is_valid_pose:
                    obj.pose = pose_sample
                    obj.create_polygons()
                    obj.parent = obj_spawn
                    obj_added = True
                    break
            if not obj_added:
                warnings.warn(f"Could not sample valid pose to add object {obj.name}.")
                return None

        # If a pose was specified, collision check it
        else:
            poly = inflate_polygon(obj.polygon, self.object_radius)
            is_valid_pose = poly.within(obj_spawn.polygon)
            for other_obj in obj_spawn.children:
                is_valid_pose = is_valid_pose and not poly.intersects(other_obj.polygon)
            if not is_valid_pose:
                warnings.warn(f"Object {obj.name} in collision or not in location {loc.name}. Cannot add to world.")
                return None
            
        # Do the necessary bookkeeping
        obj_spawn.children.append(obj)
        self.objects.append(obj)
        self.num_objects += 1
        return obj

    def remove_object(self, obj):
        """ Cleanly removes an object from the world """
        if isinstance(obj, str):
            obj = self.get_object_by_name(obj)
        if obj in self.objects:
            self.objects.remove(obj)
            self.num_objects -= 1
            obj.parent.children.remove(obj)
    
    def remove_all_objects(self, restart_numbering=True):
        """ Cleanly removes all objects from the world """
        for obj in reversed(self.objects):
            self.remove_object(obj)
        self.num_objects = 0
        if restart_numbering:
            self.object_instance_counts = {}

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

    ######################################
    # Search Graph and Occupancy Methods #
    ######################################
    def check_occupancy(self, pose):
        """
        Check if a pose in the world is occupied
        """
        # Loop through all the rooms and hallways and check if the pose
        # is deemed collision-free in any of them.
        for entity in itertools.chain(self.rooms, self.hallways):
            if entity.is_collision_free(pose):
                return False
        # If we made it through, the pose is occupied
        return True

    def create_search_graph(self, max_edge_dist=np.inf, collision_check_dist=0.1):
        """ Creates a search graph for the world """
        self.search_graph = SearchGraph(world=self,
            max_edge_dist=max_edge_dist, collision_check_dist=collision_check_dist)

        # Add nodes to the world
        for entity in itertools.chain(self.rooms, self.hallways, self.locations):
            entity.add_graph_nodes()
            if isinstance(entity, Location):
                for spawn in entity.children:
                    self.search_graph.add(spawn.graph_nodes, autoconnect=True)
            else:
                self.search_graph.add(entity.graph_nodes, autoconnect=True)

    def update_search_graph(self):
        """ Updates a search graph with the same properties as the previous one """
        if self.search_graph is None:
            return
        self.create_search_graph(max_edge_dist=self.search_graph.max_edge_dist,
                                 collision_check_dist=self.search_graph.collision_check_dist)

    def find_path(self, goal, start=None):
        """
        Finds a path from the start to goal
        If no start argument is provided, we assume it is the robot pose.
        """
        if self.search_graph is None:
            warnings.warn("No search graph defined for this world.")
            return None

        if start is None:
            start = self.robot.pose

        created_start_node = False
        if isinstance(start, Pose):
            start_loc = self.robot.location if self.has_robot else None
            if not start_loc:
                start_loc = self.get_location_from_pose(start)
            start_node = Node(start, parent=start_loc)
            self.search_graph.add(start_node, autoconnect=True)
            created_start_node = True
        else:
            start_node = self.graph_node_from_entity(start)
            if start_node is None:
                warnings.warn("Invalid start specified")
                return None

        created_goal_node = False
        if isinstance(goal, Pose):
            goal_node = Node(goal, parent=self.get_location_from_pose(goal))
            self.search_graph.add(goal_node, autoconnect=True)
            created_goal_node = True
        else:
            goal_node = self.graph_node_from_entity(goal)
            if goal_node is None:
                warnings.warn("Invalid goal specified")
                return None
        
        # Do the search
        self.current_path = self.search_graph.find_path(start_node, goal_node)
        self.current_path = fill_path_yaws(self.current_path)

        # If we created temporary nodes for search, remove them
        if created_start_node:
            self.search_graph.remove(start_node)
        if created_goal_node:
            self.search_graph.remove(goal_node)

        return self.current_path

    def graph_node_from_entity(self, entity_query, resolution_strategy="nearest"):
        """
        Gets a graph node from an entity query, which could be any 
        room, hallway, location, object spawn, or object in the world,
        as well as respective categories.
        """
        if isinstance(entity_query, Node):
            return entity_query
        elif isinstance(entity_query, str):
            entity = self.get_entity_by_name(entity_query)
            if entity is None:
                entity = resolve_to_location(self, category=entity_query,
                    expand_locations=True, resolution_strategy=resolution_strategy)
            if entity is None:
                entity = resolve_to_object(self, category=entity_query,
                    resolution_strategy=resolution_strategy, ignore_grasped=True)
        else:
            entity = entity_query

        if (isinstance(entity, ObjectSpawn) or isinstance(entity, Room)
            or isinstance(entity, Hallway)):
            graph_nodes = entity.graph_nodes
        elif isinstance(entity, Object):
            graph_nodes = entity.parent.graph_nodes
        elif isinstance(entity, Location):
            graph_nodes = entity.children[0].graph_nodes
            # TODO: Select a child node
        else:
            warnings.warn(f"Cannot get graph node from {entity}")
            return None

        # TODO: Select a graph node
        graph_node = graph_nodes[0]
        return graph_node

    def sample_free_robot_pose_uniform(self):
        """ 
        Sample an unoccupied robot pose in the world.
        """
        xmin, xmax = self.x_bounds
        ymin, ymax = self.y_bounds
        r = self.inflation_radius

        for _ in range(self.max_object_sample_tries):
            x = (xmax - xmin - 2*r) * np.random.random() + xmin + r
            y = (ymax - ymin - 2*r) * np.random.random() + ymin + r
            yaw = 2.0 * np.pi * np.random.random()
            if not self.check_occupancy((x, y)):
                return Pose(x=x, y=y, yaw=yaw)
        warnings.warn("Could not sample pose.")
        return None

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

    def get_hallways_from_rooms(self, room1, room2):
        """ Returns a list of hallways between two rooms """
        # Validate room input
        if isinstance(room1, str):
            room1 = self.get_room_by_name(room1)
        if not isinstance(room1, Room):
            warnings.warn("Invalid room1 specified")
            return []
        if isinstance(room2, str):
            room2 = self.get_room_by_name(room2)
        if not isinstance(room2, Room):
            warnings.warn("Invalid room2 specified")
            return []

        # Now search through the hallways and add any valid ones to the list
        hallways = []
        for hall in room1.hallways:
            is_valid_hallway = \
                ((hall.room_start == room1) and (hall.room_end == room2)) or \
                ((hall.room_end == room2) and (hall.room_end == room1))
            if is_valid_hallway:
                hallways.append(hall)
        return hallways

    def get_locations(self, category_list=None):
        """ Gets all locations, optionally filtered by category """
        if not category_list:
            return [loc for loc in self.locations]
        else:
            return [loc for loc in self.locations
                    if loc.category in category_list]

    def get_location_names(self, category_list=None):
        """ Gets all location names, optionally filtered by category """
        if not category_list:
            return [loc.name for loc in self.locations]
        else:
            return [loc.name for loc in self.locations
                    if loc.category in category_list]

    def get_location_by_name(self, name):
        """ Gets a location object by its name """
        names = self.get_location_names()
        if name in names:
            idx = names.index(name)
            return self.locations[idx]
        else:
            return None

    def get_location_from_pose(self, pose):
        """ Gets the name of a location given a pose """
        # First, check rooms and hallways.
        for entity in itertools.chain(self.rooms, self.hallways):
            if entity.is_collision_free(pose):
                return entity
        # Then, check object spawns.
        for loc in self.locations:
            for spawn in loc.children:
                if spawn.is_inside(pose):
                    return spawn
        return None

    def get_objects(self, category_list=None):
        """ Gets all objects, optionally filtered by category """
        if not category_list:
            return self.objects
        else:
            return [o for o in self.objects 
                    if o.category in category_list]

    def get_object_names(self, category_list=None):
        """ Gets all object names, optionally filtered by category """
        if not category_list:
            return [o.name for o in self.objects]
        else:
            return [o.name for o in self.objects 
                    if o.category in category_list]

    def get_object_by_name(self, name):
        """ Gets an object by its name """
        names = self.get_object_names()
        if name in names:
            idx = names.index(name)
            return self.objects[idx]
        else:
            return None

    def get_entity_by_name(self, name):
        """ Gets any entity above by its name """
        for entity in itertools.chain(
            self.rooms, self.hallways, self.locations, self.objects):
            if entity.name == name:
                return entity
            elif isinstance(entity, Location):
                for spawn in entity.children:
                    if spawn.name == name:
                        return spawn
        return None


    ###########
    # Actions #
    ###########
    def add_robot(self, robot=Robot(), loc=None, pose=None, use_robot_pose=False):
        """
        Adds a robot to the world given either a world entity and/or pose
        """
        old_inflation_radius = self.inflation_radius
        self.set_inflation_radius(robot.radius)
        valid_pose = True

        if use_robot_pose:
            # If using the robot pose, simply add the robot and we're done!
            robot_pose = robot.pose
            if self.check_occupancy((pose.x, pose.y)):
                warnings.warn(f"{pose} is occupied.")
                valid_pose = False

        elif loc is None:
            if pose is None:
                # If nothing is specified, sample any valid location in the world
                robot_pose = self.sample_free_robot_pose_uniform()
                if robot_pose is None:
                    warnings.warn("Unable to sample free pose.")
                    valid_pose = False
            else:
                # Validate that the pose is unoccupied
                if self.check_occupancy((pose.x, pose.y)):
                    warnings.warn(f"{pose} is occupied.")
                    valid_pose = False
                robot_pose = pose
            # If we have a valid pose, extract its location
            loc = self.get_location_from_pose(robot_pose)
        
        elif loc is not None:
            # First, validate that the location is valid for a robot
            if isinstance(loc, str):    
                loc = self.get_entity_by_name(loc)
            
            if isinstance(loc, Room) or isinstance(loc, Hallway):
                if pose is None:
                    # Sample a pose in the location
                    x_sample, y_sample = sample_from_polygon(
                        loc.internal_collision_polygon, max_tries=self.max_object_sample_tries)
                    if x_sample is None:
                        warnings.warn(f"Could not sample pose in {loc.name}.")
                        valid_pose = False
                    yaw_sample = np.random.uniform(-np.pi, np.pi)
                    robot_pose = Pose(x=x_sample, y=y_sample, yaw=yaw_sample)
                else:
                    # Validate that the pose is unoccupied and in the right location 
                    if not loc.is_collision_free(pose):
                        warnings.warn(f"{pose} is occupied")
                        valid_pose = False
                    robot_pose = pose
            elif isinstance(loc, Location) or isinstance(loc, ObjectSpawn):
                if isinstance(loc, Location):
                    # NOTE: If you don't want a random object spawn, use the object spawn as the input location.
                    loc = np.random.choice(loc.children) 
                
                if pose in loc.nav_poses: # Slim chance of this happening lol
                    robot_pose = pose
                else:
                    robot_pose = np.random.choice(loc.nav_poses)
            else:
                warnings.warn("Invalid location specified.")
                valid_pose = False

        # If we got a valid location / pose combination, add the robot
        if valid_pose:
            self.robot = robot
            self.robot.location = loc
            self.robot.set_pose(robot_pose)
            self.has_robot = True
        else:
            warnings.warn("Could not add robot.")
            self.set_inflation_radius(old_inflation_radius)

    def remove_robot(self):
        """ Removes a robot from the world """
        if self.has_robot:
            self.robot = None
            self.has_robot = False
        else:
            warnings.warn("No robot to remove.")    

    def execute_path(self, path, dt=0.05, realtime_factor=1.0,
                     linear_velocity=0.2, max_angular_velocity=None,
                     blocking=False):
        """
        Executes a specified base path
        TODO: Set nav parameters as properties of the robot
        TODO: Set goal as part of path to simplify
        """
        if path is None:
            return

        # Start a thread with the path execution
        self.nav_thread = threading.Thread(target=execute_with_linear_trajectory,
                                           args=(self.robot, path, dt, realtime_factor,
                                                 linear_velocity, max_angular_velocity))
        self.nav_thread.start()
        if blocking:
            self.nav_thread.join()

    def pick_object(self, obj_query):
        """ 
        Picks up an object in the world given an object and location query. 
        Returns True if successful and False otherwise.
        """
        # Validate input
        if not self.has_robot:
            warnings.warn(f"No robot in the world.")
            return False
        elif self.robot.manipulated_object is not None:
            warnings.warn(f"Robot is already holding {self.robot.manipulated_object.name}.")
            return False

        # Validate the robot location
        loc = self.robot.location
        if not isinstance(loc, ObjectSpawn):
            warnings.warn("Not an object spawn. Cannot pick object.")
            return False

        # Get object
        if not obj_query or isinstance(obj_query, str):
            obj = self.get_object_by_name(obj_query)
            if not obj:
                obj = resolve_to_object(
                    self, category=obj_query, location=loc,
                    resolution_strategy="nearest")
        if not isinstance(obj, Object):
            warnings.warn(f"Invalid object {obj.name}.")
            return False
    
        # Denote the target object as the manipulated object
        self.robot.manipulated_object = obj
        obj.parent.children.remove(obj)
        obj.parent = self.robot
        obj.pose = self.robot.pose
        return True

    def place_object(self, loc, pose=None):
        """
        Places an object in a target location and (optionally) pose.
        Returns True if successful and False otherwise.
        """
        # Validate input
        if not self.has_robot:
            warnings.warn(f"No robot in the world.")
            return False
        elif self.robot.manipulated_object is None:
            warnings.warn("No manipulated object.")
            return False

        # Validate the robot location
        loc = self.robot.location
        if not isinstance(loc, ObjectSpawn):
            warnings.warn("Not an object spawn. Cannot place object.")
            return False

        # Place the object somewhere in the current location
        poly = inflate_polygon(self.robot.manipulated_object.get_raw_polygon(),
                               self.object_radius)
        if pose is None:
            # If no pose was specified, sample one
            is_valid_pose = False
            for _ in range(self.max_object_sample_tries):
                x_sample, y_sample = sample_from_polygon(loc.polygon)
                yaw_sample = np.random.uniform(-np.pi, np.pi)
                pose_sample = Pose(x=x_sample, y=y_sample, yaw=yaw_sample)
                sample_poly = transform_polygon(poly, pose_sample)
                is_valid_pose = sample_poly.within(loc.polygon)
                for other_obj in loc.children:
                    is_valid_pose = is_valid_pose and not sample_poly.intersects(other_obj.polygon)
                if is_valid_pose:
                    self.robot.manipulated_object.pose = pose_sample
                    self.robot.manipulated_object.parent = loc
                    self.robot.manipulated_object.create_polygons()
                    loc.children.append(self.robot.manipulated_object)
                    self.robot.manipulated_object = None
                    return True
            warnings.warn(f"Could not sample a placement position at {loc.name}")
            return False
        else:
            # If a pose was specified, collision check it
            poly = transform_polygon(poly, pose_sample)
            is_valid_pose = poly.within(loc.polygon)
            for other_obj in loc.children:
                is_valid_pose = is_valid_pose and not poly.intersects(other_obj.polygon)
            if is_valid_pose:
                return True
            else:
                warnings.warn(f"Pose in collision or not in location {loc.name}.")
                return False
            