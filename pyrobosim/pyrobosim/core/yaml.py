""" Utilities to create worlds from YAML files. """

import numpy as np
import warnings
import yaml

from .robot import Robot
from .room import Room
from .world import World
from ..utils.general import replace_special_yaml_tokens
from ..utils.pose import Pose


def get_value_or(data, key, default=None):
    """ 
    Utility function to get a value from a dictionary if the key exists,
    or else get a default value.

    :param data: Dictionary containing the data.
    :type data: dict
    :param key: Dictionary key.
    :type key: str
    :param default: Default value to use if key does not exist, defaults to None.
    :return: Value from the dictionary, or the default value if the key does not exist.
    """
    return data[key] if key in data else default


class WorldYamlLoader:
    """ Creates world models from YAML files. """

    def from_yaml(self, filename):
        """ 
        Load a world from a YAML description. 
        
        :param filename: Path to YAML file describing the world.
        :type filename: str
        :return: World model instance.
        :rtype: :class:`pyrobosim.core.world.World`
        """
        self.filename = filename
        with open(self.filename) as file:
            self.data = yaml.load(file, Loader=yaml.FullLoader)

        # Build and return the world
        self.create_world()
        self.add_rooms()
        self.add_hallways()
        self.add_locations()
        self.add_objects()
        self.add_robot()
        self.add_global_path_planner()
        return self.world


    def create_world(self):
        """ Creates an initial world with the specified global parameters. """
        if "params" in self.data:
            params = self.data["params"]
            name = get_value_or(params, "name", default="world")
            inf_radius = get_value_or(params, "inflation_radius", default=0.0)
            obj_radius = get_value_or(params, "object_radius", default=0.0)
            wall_height = get_value_or(params, "wall_height", default=2.0)
            self.world = World(name=name, inflation_radius=inf_radius,
                               object_radius=obj_radius, wall_height=wall_height)
        else:
            self.world = World()

        # Set the location/object metadata
        metadata = self.data["metadata"]
        if "locations" in metadata:
            loc_data = replace_special_yaml_tokens(metadata["locations"])
        else:
            loc_data = None
        if "objects" in metadata:
            obj_data = replace_special_yaml_tokens(metadata["objects"])
        else:
            obj_data = None
        self.world.set_metadata(locations=loc_data, objects=obj_data)


    def add_rooms(self):
        """ Add rooms to the world. """
        if "rooms" not in self.data:
            return

        for room_data in self.data["rooms"]:
            name = get_value_or(room_data, "name", default=None)
            color = get_value_or(room_data, "color", default=[0.4, 0.4, 0.4])
            wall_width = get_value_or(room_data, "wall_width", default=0.2)
            if "nav_poses" in room_data:
                nav_poses = [Pose.from_list(p) for p in room_data["nav_poses"]]
            else:
                nav_poses = None
        
            room = Room(room_data["footprint"], name=name, color=color, 
                        wall_width=wall_width, nav_poses=nav_poses)
            self.world.add_room(room)
        

    def add_hallways(self):
        """ Add hallways connecting rooms to the world. """
        if "hallways" not in self.data:
            return

        for hall in self.data["hallways"]:
            conn_method = get_value_or(hall, "conn_method", default="auto")
            offset = get_value_or(hall, "offset", default=0.0)
            conn_angle = get_value_or(hall, "conn_angle", default=0.0)
            conn_points = get_value_or(hall, "conn_points", default=[])
            color = get_value_or(hall, "color", default=[0.4, 0.4, 0.4])
            wall_width = get_value_or(hall, "wall_width", default=0.2)

            self.world.add_hallway(
                hall["from"], hall["to"], hall["width"],
                conn_method=conn_method, offset=offset, 
                conn_angle=conn_angle, conn_points=conn_points,
                color=color, wall_width=wall_width)


    def add_locations(self):
        """ Add locations for object spawning to the world. """
        if "locations" not in self.data:
            return
        
        for loc in self.data["locations"]:
            category = loc["type"]
            room = loc["room"]
            pose = Pose.from_list(loc["pose"])
            name = get_value_or(loc, "name", default=None)
            self.world.add_location(category, room, pose, name=name)
        

    def add_objects(self):
        """ Add objects to the world. """
        if "objects" not in self.data:
            return
        
        for obj in self.data["objects"]:
            category = obj["type"]
            loc = obj["location"]
            if "pose" in obj:
                pose = Pose.from_list(obj["pose"])
            else:
                pose = None
            name = get_value_or(obj, "name", default=None)
            self.world.add_object(category, loc, pose=pose, name=name)


    def add_robot(self):
        """ Add a robot to the world. """
        if "robots" not in self.data:
            return

        # We only support a single robot now, so output the appropriate warning.
        if len(self.data["robots"]) > 1:
            warnings.warn("Multiple robots not supported. Only using the first one.")
        robot_data = self.data["robots"][0]

        # Create the robot
        robot = Robot(radius=robot_data["radius"],
                      path_planner=self.get_local_path_planner(robot_data),
                      path_executor=self.get_path_executor(robot_data))
        
        loc = robot_data["location"] if "location" in robot_data else None
        if "pose" in robot_data:
            pose = Pose.from_list(robot_data["pose"])
        else:
            pose = None
        self.world.add_robot(robot, loc=loc, pose=pose)


    def add_global_path_planner(self):
        """ Adds a global path planner to the world. """
        if "global_path_planner" not in self.data or "type" not in self.data["global_path_planner"]:
            return
        planner_data = self.data["global_path_planner"]
        planner_type = planner_data["type"]
        
        if planner_type == "search_graph":
            max_edge_dist = get_value_or(planner_data, "max_edge_dist", default=np.inf)
            collision_check_dist = get_value_or(planner_data, "collision_check_dist", default=0.1)
            self.world.create_search_graph(
                max_edge_dist=max_edge_dist, collision_check_dist=collision_check_dist, create_planner=True)
        else:
            # Always make a search graph as we use it for other things.
            max_edge_dist = get_value_or(planner_data, "max_edge_dist", default=np.inf)
            collision_check_dist = get_value_or(planner_data, "collision_check_dist", default=0.1)
            self.world.create_search_graph(
                max_edge_dist=max_edge_dist, collision_check_dist=collision_check_dist, create_planner=False)

            if planner_type == "prm":
                from pyrobosim.navigation.prm import PRMPlanner
                max_nodes = get_value_or(planner_data, "max_nodes", default=100)
                max_connection_dist = get_value_or(planner_data, "max_connection_dist", default=1.0)
                self.world.path_planner = PRMPlanner(
                    self.world, max_nodes=max_nodes, max_connection_dist=max_connection_dist)
            else:
                warnings.warn(f"Invalid global planner type specified: {planner_type}")


    def get_local_path_planner(self, robot_data):
        """ Gets local planner path planner to a robot. """
        if "path_planner" not in robot_data:
            return None
        planner_data = robot_data["path_planner"]
        planner_type = planner_data["type"]

        if planner_type == "rrt":
            from pyrobosim.navigation.rrt import RRTPlanner
            bidirectional = get_value_or(planner_data, "bidirectional", default=False)
            rrt_star = get_value_or(planner_data, "rrt_star", default=False)
            rrt_connect = get_value_or(planner_data, "rrt_connect", default=False)
            max_connection_dist = get_value_or(planner_data, "max_connection_dist", default=0.5)
            max_nodes_sampled = get_value_or(planner_data, "max_nodes_sampled", default=1000)
            max_time = get_value_or(planner_data, "max_time", default=5.0)
            rewire_radius = get_value_or(planner_data, "rewire_radius", default=1.0)
            return RRTPlanner(self.world, bidirectional=bidirectional, rrt_star=rrt_star, rrt_connect=rrt_connect,
                              max_connection_dist=max_connection_dist, max_nodes_sampled=max_nodes_sampled,
                              max_time=max_time, rewire_radius=rewire_radius)
        else:
            warnings.warn(f"Invalid planner type specified: {planner_type}")
            return None


    def get_path_executor(self, robot_data):
        """ Adds a path executor to a robot. """
        from pyrobosim.navigation.execution import ConstantVelocityExecutor
        return ConstantVelocityExecutor()
