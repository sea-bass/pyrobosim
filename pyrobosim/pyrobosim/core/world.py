""" Main file containing the core world modeling tools. """

import itertools
import numpy as np
import warnings

from .hallway import Hallway
from .locations import Location, ObjectSpawn
from .objects import Object
from .room import Room
from ..navigation.search_graph import Node, SearchGraph, SearchGraphPlanner
from ..utils.knowledge import resolve_to_location, resolve_to_object
from ..utils.pose import Pose
from ..utils.polygon import inflate_polygon, sample_from_polygon, transform_polygon

class World:
    """ Core world modeling class. """
    def __init__(self, name="world", inflation_radius=0.0, object_radius=0.05, wall_height=2.0):
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
        self.current_goal = None
        self.current_path = None
        self.path_planner = None

        # Other parameters
        self.max_object_sample_tries = 1000 # Max number of tries to sample object locations

        # Distances for collision-aware navigation and sampling
        self.object_radius = object_radius
        self.set_inflation_radius(inflation_radius)


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
        self.update_search_graph()

    ##########################
    # World Building Methods #
    ##########################
    def add_room(self, room):
        """ 
        Adds a room to the world.

        If the room does not have a specified name, it will be given an automatic
        name of the form ``"room0"``, ``"room1"``, etc.

        If the room would cause a collision with another entity in the world,
        it will not be added to the world model.
        
        :param room: Room object to add to the world.
        :type room: :class:`pyrobosim.core.room.Room`
        :return: True if the room was successfully added, else False.
        :rtype: bool
        """
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
            return False

        self.rooms.append(room)
        self.num_rooms += 1
        self.update_bounds()

        # Update the room collision polygon based on the world inflation radius
        room.update_collision_polygons(self.inflation_radius)

        # Update the search graph, if any
        if self.search_graph is not None:
            room.add_graph_nodes()
            self.search_graph.add(room.graph_nodes, autoconnect=True)

        return True

    def remove_room(self, room_name):
        """ 
        Removes a room from the world by name.
        
        :param room_name: Name of room to remove.
        :type room_name: str
        :return: True if the room was successfully removed, else False.
        :rtype: bool
        """
        for i, room in enumerate(self.rooms):
            if room.name == room_name:
                self.rooms.pop(i)
                self.num_rooms -= 1
                self.update_bounds()
                
                # Update the search graph, if any
                if self.search_graph is not None:
                    self.search_graph.remove(room.graph_nodes)
                return True

        warnings.warn(f"No room {room_name} found for removal.")
        return False

    def add_hallway(self, room_start, room_end, width,
                    conn_method="auto", offset=0, conn_angle=0,
                    conn_points=[], color=[0.4, 0.4, 0.4], wall_width=0.2):
        """
        Adds a hallway from room_start to room_end, with specified parameters 
        related to the :class:`pyrobosim.core.hallway.Hallway` class.

        :param room_start: Start room instance or name.
        :type room_start: :class:`pyrobosim.core.room.Room`/str
        :param room_end: End room instance or name.
        :type room_end: :class:`pyrobosim.core.room.Room`/str
        :param width: Width of the hallway, in meters.
        :type width: float
        :param conn_method: Connection method (see :class:`pyrobosim.core.hallway.Hallway` documentation).
        :type conn_method: str, optional
        :param offset: Perpendicular offset from centroid of start point 
            (valid if using ``"auto"`` or ``"angle"`` connection methods)
        :type offset: float, optional
        :param conn_angle: If using ``"angle"`` connection method, specifies
            the angle of the hallway in radians (0 points to the right).
        :type conn_angle: float, optional
        :param conn_points: If using "points" connection method, specifies the hallway points.
        :type conn_points: list[(float, float)], optional
        :param color: Visualization color as an (R, G, B) tuple in the range (0.0, 1.0)
        :type color: (float, float, float), optional
        :param wall_width: Width of hallway walls, in meters.
        :type wall_width: float, optional
        :return: Hallway object if successfully created, else None.
        :rtype: :class:`pyrobosim.core.hallway.Hallway`
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
                    color=color, wall_width=wall_width)

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
        self.update_bounds()

        # Update the search graph, if any
        if self.search_graph is not None:
            h.add_graph_nodes()
            self.search_graph.add(h.graph_nodes, autoconnect=True)

        # Finally, return the Hallway object
        return h

    def remove_hallway(self, hallway):
        """ 
        Removes a hallway between two rooms. 
        
        :param hallway: Hallway object remove.
        :type hallway: :class:`pyrobosim.core.hallway.Hallway`
        :return: True if the hallway was successfully removed, else False.
        :rtype: bool
        """
        # Validate the input
        if not hallway in self.hallways:
            warnings.warn("Invalid hallway specified.")
            return False

        # Remove the hallways from the world and relevant rooms.
        self.hallways.remove(hallway)
        for r in [hallway.room_start, hallway.room_end]:
            r.hallways.remove(hallway)
            r.update_collision_polygons()
            r.update_visualization_polygon()
        return True

    def add_location(self, category, room, pose, name=None):
        """ 
        Adds a location at the specified room.

        If the location does not have a specified name, it will be given an
        automatic name using its category, e.g., ``"table0"``.
        
        :param category: Location category (e.g., ``"table"``).
        :type category: str
        :param room: Room instance or name.
        :type room: :class:`pyrobosim.core.room.Room`/str
        :param pose: Pose of the location.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :param name: Name of the location.
        :type name: str, optional
        :return: Location object if successfully created, else None.
        :rtype: :class:`pyrobosim.core.locations.Location`
        """
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


    def update_location(self, loc, pose, room=None):
        """ 
        Updates an existing location in the world.
        
        :param loc: Location instance or name to update.
        :type loc: :class:`pyrobosim.core.locations.Location`/str
        :param pose: Pose of the location.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :param room: Room instance or name. If none, uses the previous room.
        :type room: :class:`pyrobosim.core.room.Room`/str, optional
        :return: True if the update was successful, else False.
        :rtype: bool
        """
        if isinstance(loc, str):
            loc = self.get_location_by_name(loc)
        if not isinstance(loc, Location):
            warnings.warn("Could not find location. Not updating.")
            return False

        if room is not None:
            if isinstance(room, str):
                room = self.get_room_by_name(room)
        
            if not isinstance(room, Room):
                warnings.warn(f"Room {loc} did not resolve to a valid room for a location.")
                return False

        # Check that the location fits within the room and is not in collision with
        # other locations already in the room. Else, warn and do not add it.
        new_polygon = transform_polygon(loc.raw_polygon, pose)
        is_valid_pose = new_polygon.within(room.polygon)
        for other_loc in room.locations:
            if loc != other_loc:
                is_valid_pose = is_valid_pose and not new_polygon.intersects(other_loc.polygon)
        if not is_valid_pose:
            warnings.warn(f"Location {loc.name} in collision. Cannot add to world.")
            return False

        # If we passed all checks, update the polygon.
        loc.parent.locations.remove(loc)
        loc.parent = room
        room.locations.append(loc)
        loc.set_pose(pose)
        loc.create_polygons(self.inflation_radius)
        for spawn in loc.children:
            spawn.set_pose_from_parent()
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

        if loc in self.locations:
            self.locations.remove(loc)
            self.num_locations -= 1
            self.location_instance_counts[loc.category] -= 1
            room = loc.parent
            room.locations.remove(loc)
            room.update_collision_polygons(self.inflation_radius)
            return True
        return False

    def add_object(self, category, loc, pose=None, name=None):
        """
        Adds an object to a specific location.

        If the location does not have a specified name, it will be given an
        automatic name using its category, e.g., ``"apple0"``.

        If the location contains multiple object spawns, one will be selected at random.

        :param category: Object category (e.g., ``"apple"``).
        :type category: str
        :param loc: Location or object spawn instance or name.
        :type loc: :class:`pyrobosim.core.locations.Location`/:class:`pyrobosim.core.locations.ObjectSpawn`/str
        :param pose: Pose of the location. If none is specified, it will be sampled.
        :type pose: :class:`pyrobosim.utils.pose.Pose`, optional
        :param name: Name of the location.
        :type name: str, optional
        :return: Object instance if successfully created, else None.
        :rtype: :class:`pyrobosim.core.objects.Object`
        """
        # If no name is specified, create one automatically
        if category not in self.object_instance_counts:
            self.object_instance_counts[category] = 0
        if name is None:
            name = f"{category}{self.object_instance_counts[category]}"
        self.object_instance_counts[category] +=1

        # If it's a string, get the location name
        if isinstance(loc, str):
            loc = self.get_entity_by_name(loc)
        # If it's a location object, pick an object spawn at random.
        # Otherwise, if it's an object spawn, use that entity as is.
        if isinstance(loc, Location):
            obj_spawn = np.random.choice(loc.children)
        elif isinstance(loc, ObjectSpawn):
            obj_spawn = loc
        else:
            warnings.warn(f"Location {loc} did not resolve to a valid location for an object.")
            return None

        # Create the object
        obj = Object(category=category, name=name, parent=obj_spawn, pose=pose)
        
        # If no pose is specified, sample a valid one
        if pose is None:
            obj_added = False
            for _ in range(self.max_object_sample_tries):
                x_sample, y_sample = sample_from_polygon(obj_spawn.polygon)
                yaw_sample = np.random.uniform(-np.pi, np.pi)
                pose_sample = Pose(x=x_sample, y=y_sample, yaw=yaw_sample)
                poly = inflate_polygon(
                    transform_polygon(obj.polygon, pose_sample), self.object_radius)
                
                is_valid_pose = poly.within(obj_spawn.polygon)
                for other_obj in obj_spawn.children:
                    is_valid_pose = is_valid_pose and not poly.intersects(other_obj.polygon)
                if is_valid_pose:
                    obj.parent = obj_spawn
                    obj.set_pose(pose_sample)
                    obj.create_polygons()
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


    def update_object(self, obj, loc=None, pose=None):
        """
        Updates an existing object in the world.

        :param obj: Object instance or name to update.
        :type obj: :class:`pyrobosim.core.objects.Object`/str
        :param loc: Location or object spawn instance or name. If none, uses the previous location.
        :type loc: :class:`pyrobosim.core.locations.Location`/:class:`pyrobosim.core.locations.ObjectSpawn`/str, optional
        :param pose: Pose of the location. If none is specified, it will be sampled.
        :type pose: :class:`pyrobosim.utils.pose.Pose`, optional
        :return: True if the update was successful, else False.
        :rtype: bool
        """
        if isinstance(obj, str):
            obj = self.get_object_by_name(obj)
        if not isinstance(obj, Object):
            warnings.warn("Could not find object. Not updating.")
            return False

        if loc is not None:
            if pose is None:
                warnings.warn("Cannot specify a location without a pose.")
    
            # If it's a string, get the location name
            if isinstance(loc, str):
                loc = self.get_entity_by_name(loc)
            # If it's a location object, pick an object spawn at random.
            # Otherwise, if it's an object spawn, use that entity as is.
            if isinstance(loc, Location):
                obj_spawn = np.random.choice(loc.children)
            elif isinstance(loc, ObjectSpawn):
                obj_spawn = loc
            else:
                warnings.warn(f"Location {loc} did not resolve to a valid location for an object.")
                return False

            obj.parent.children.remove(obj)
            obj.parent = obj_spawn
            obj_spawn.children.append(obj)

        if pose is not None:
            obj.set_pose(pose)
            obj.create_polygons(self.object_radius)

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
        if obj in self.objects:
            self.objects.remove(obj)
            self.num_objects -= 1
            obj.parent.children.remove(obj)
            return True
        return False
    
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

    def update_bounds(self):
        """ 
        Updates the X and Y bounds of the world.

        TODO: If we're just adding a single room, we only need to check that one
        and there is probably a more efficient way to do this.
        """
        for entity in itertools.chain(self.rooms, self.hallways):
            (xmin, ymin, xmax, ymax) = entity.polygon.bounds
            self.x_bounds[0] = min(self.x_bounds[0], xmin)
            self.x_bounds[1] = max(self.x_bounds[1], xmax)
            self.y_bounds[0] = min(self.y_bounds[0], ymin)
            self.y_bounds[1] = max(self.y_bounds[1], ymax)

    ######################################
    # Search Graph and Occupancy Methods #
    ######################################
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

    def create_search_graph(self, max_edge_dist=np.inf, collision_check_dist=0.1,
                            create_planner=False):
        """ 
        Creates a search graph for the world, as a :class:`pyrobosim.navigation.search_graph.SearchGraph` attribute.
        
        :param max_edge_dist: Maximum distance to automatically connect two edges, defaults to infinity.
        :type max_edge_dist: float, optional
        :param collision_check_dist: Distance sampled along an edge to check for collisions, defaults to 0.1.
        :type collision_check_dist: float, optional
        :param create_planner: If True, creates a :class:`pyrobosim.navigation.search_graph.SearchGraphPlanner`.
        :type create_planner: bool
        """
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
        
        # Optionally, create a global planner from the search graph
        if create_planner:
            self.path_planner = SearchGraphPlanner(self.search_graph)

    def update_search_graph(self):
        """ 
        Updates a search graph with the same properties as the previous one, if there was one. 
        """
        if self.search_graph is None:
            return
        self.create_search_graph(max_edge_dist=self.search_graph.max_edge_dist,
                                 collision_check_dist=self.search_graph.collision_check_dist)

    def find_path(self, goal, start=None):
        """
        Finds a path from a start to goal location.
        If no start argument is provided, we assume it is the current robot pose.

        :param goal: Goal pose, search graph node, or world entity.
        :param start: Optional start pose, search graph node, or world entity.
        :return: List of graph Node objects describing the path, or None if not found.
        :rtype: list[:class:`pyrobosim.navigation.search_graph.Node`]
        """
        # Prepare by adding start and goal nodes to the graph.
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
        self.current_goal = goal_node.parent

        # Do the actual planning.
        if self.robot.path_planner:
            # Plan with the robot's local planner.
            goal = goal_node.pose
            self.current_path = self.robot.path_planner.plan(start, goal)
        elif self.path_planner:
            # Plan with the robot's global planner.
            self.current_path = self.path_planner.plan(start_node, goal_node)
        else:
            warnings.warn("No global or local path planners specified.")
            return None

        # If we created temporary nodes for search, remove them
        if created_start_node:
            self.search_graph.remove(start_node)
        if created_goal_node:
            self.search_graph.remove(goal_node)

        return self.current_path

    def graph_node_from_entity(self, entity_query, resolution_strategy="nearest"):
        """
        Gets a graph node from an entity query, which could be any combination of
        room, hallway, location, object spawn, or object in the world, as well as 
        their respective categories.

        For more information on the inputs, refer to the :func:`pyrobosim.utils.knowledge.query_to_entity` function.

        :param entity_query: List of entities (e.g., rooms or objects) or names/categories.
        :type entity_query: list[Entity/str]
        :param resolution_strategy: Resolution strategy to apply
        :type resolution_strategy: str
        :return: A graph node for the entity that meets the resolution strategy, or None.
        :rtype: :class:`pyrobosim.navigation.search_graph.Node`
        """
        if isinstance(entity_query, Node):
            return entity_query
        elif isinstance(entity_query, str):
            # Try resolve an entity based on its name. If that fails, we assume it must be a category,
            # so try resolve it to a location or to an object by category.
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
        
        This is done using uniform sampling within the world X-Y bounds and rejecting
        any samples that are in collision with entities in the world.
        If no valid samples could be found within the `max_object_sample_tries` instance
        attribute, this will return ``None``.

        :return: Collision-free pose if found, else ``None``.
        :rtype: :class:`pyrobosim.utils.pose.Pose`
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
        """ 
        Gets all room names in the world.
        
        :return: List of all room names.
        :rtype: list[str]
        """
        return [r.name for r in self.rooms]

    def get_room_by_name(self, name):
        """ 
        Gets a room object by its name.
        
        :param name: Name of room.
        :type name: str
        :return: Room instance matching the input name, or ``None`` if not valid.
        :rtype: :class:`pyrobosim.core.room.Room`
        """
        names = self.get_room_names()
        if name in names:
            idx = names.index(name)
            return self.rooms[idx]
        else:
            warnings.warn(f"Room not found: {name}")
            return None

    def get_hallways_from_rooms(self, room1, room2):
        """ 
        Returns a list of hallways between two rooms.
        
        :param room1: Instance or name of first room.
        :type room1: :class:`pyrobosim.core.room.Room`/str
        :param room2: Instance or name of second room.
        :type room2: :class:`pyrobosim.core.room.Room`/str
        :return: Hallway instance matching the inputs, or ``None`` if not valid.
        :rtype: :class:`pyrobosim.core.hallway.Hallway`
        """
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
            return [loc for loc in self.locations
                    if loc.category in category_list]

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
            return [loc.name for loc in self.locations
                    if loc.category in category_list]

    def get_location_by_name(self, name):
        """
        Gets a location object by its name.
        
        :param name: Name of location.
        :type name: str
        :return: Location instance matching the input name, or ``None`` if not valid.
        :rtype: :class:`pyrobosim.core.locations.Location`
        """
        names = self.get_location_names()
        if name in names:
            idx = names.index(name)
            return self.locations[idx]
        else:
            return None

    def get_location_from_pose(self, pose):
        """ 
        Gets the name of a location given a pose.
        
        :param pose: Input pose.
        :type name: :class:`pyrobosim.utils.pose.Pose`
        :return: Entity matching the input pose, or ``None`` if not valid.
            This could be a room, hallway, or object spawn.
        :rtype: :class:`pyrobosim.core.room.Room`/:class:`pyrobosim.core.hallway.Hallway`/:class:`pyrobosim.core.locations.ObjectSpawn`
        """
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
            return [o for o in self.objects 
                    if o.category in category_list]

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
            return [o.name for o in self.objects 
                    if o.category in category_list]

    def get_object_by_name(self, name):
        """ 
        Gets an object by its name.
        
        :param name: Name of object.
        :type name: str
        :return: Object instance matching the input name, or ``None`` if not valid.
        :rtype: :class:`pyrobosim.core.object.Object`
        """
        names = self.get_object_names()
        if name in names:
            idx = names.index(name)
            return self.objects[idx]
        else:
            return None

    def get_entity_by_name(self, name):
        """ 
        Gets any world entity by its name.
        
        :param name: Name of entity.
        :type name: str
        :return: Entity object instance matching the input name, or ``None`` if not valid.
        """
        if self.robot and name == self.robot.name:
            return self.robot # Should be part of the chain below when multiple robots are supported.
        for entity in itertools.chain(
            self.rooms, self.hallways, self.locations, self.objects):
            if entity.name == name:
                return entity
            elif isinstance(entity, Location):
                for spawn in entity.children:
                    if spawn.name == name:
                        return spawn
        return None

    def add_robot(self, robot, loc=None, pose=None, use_robot_pose=False):
        """
        Adds a robot to the world given either a world entity and/or pose.

        :param robot: Robot instance to add to the world.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        :param loc: World entity instance or name to place the robot.
        :type loc: Entity, optional
        :param pose: Pose at which to add the robot. If not specified, will be sampled.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :param use_robot_pose: If True, uses the pose already specified in the robot instance.
        :type use_robot_pose: bool, optional
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
            self.robot.world = self
            self.has_robot = True
        else:
            warnings.warn("Could not add robot.")
            self.set_inflation_radius(old_inflation_radius)

    def remove_robot(self):
        """ 
        Removes a robot from the world.
        
        :return: True if the robot was successfully removed, else False.
        :rtype: bool
        """
        if self.has_robot:
            self.robot = None
            self.has_robot = False
            return True
        else:
            warnings.warn("No robot to remove.")
            return False
            