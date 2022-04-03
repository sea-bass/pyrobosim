"""
Utilities to create worlds from YAML files
"""

import numpy as np
import os
import warnings
import yaml

from .robot import Robot
from .room import Room
from .world import World
from ..utils.pose import Pose


def get_value_or(data, key, default=None):
    """ 
    Utility function to get a value from a dictionary if the key exists,
    or else get a default value.
    """
    return data[key] if key in data else default


class WorldYamlLoader:
    world = None

    def from_yaml(self, filename):
        """ Load a world from a YAML description. """
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
        self.add_planner()
        return self.world


    def create_world(self):
        """ Creates an initial world """
        # Create a world given its global parameters
        if "params" in self.data:
            params = self.data["params"]
            name = get_value_or(params, "name", default="world")
            inf_radius = get_value_or(params, "inflation_radius", default=0.0)
            obj_radius = get_value_or(params, "object_radius", default=0.0)
            self.world = World(name=name, inflation_radius=inf_radius, object_radius=obj_radius)
        else:
            self.world = World()

        # Set the location/object metadata
        world_dir = os.path.dirname(self.filename)
        metadata = self.data["metadata"]
        if "locations" in metadata:
            loc_data = os.path.join(world_dir, metadata["locations"])
        else:
            loc_data = None
        if "objects" in metadata:
            obj_data = os.path.join(world_dir, metadata["objects"])
        else:
            obj_data = None
        self.world.set_metadata(locations=loc_data, objects=obj_data)


    def add_rooms(self):
        """ Add rooms to the world """
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
        """ Add hallways connecting rooms to the world """
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
        """ Add locations for object spawning to the world """
        if "locations" not in self.data:
            return
        
        for loc in self.data["locations"]:
            category = loc["type"]
            room = loc["room"]
            pose = Pose.from_list(loc["pose"])
            name = get_value_or(loc, "name", default=None)
            self.world.add_location(category, room, pose, name=name)
        

    def add_objects(self):
        """ Add objects to the world """
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
        """ Add a robot to the world """
        if "robots" not in self.data:
            return

        # We only support a single robot now, so output the appropriate warning.
        if len(self.data["robots"]) > 1:
            warnings.warn("Multiple robots not supported. Only using the first one.")
        robot_data = self.data["robots"][0]

        from pyrobosim.navigation.execution import ConstantVelocityExecutor
        robot = Robot(radius=robot_data["radius"], path_executor=ConstantVelocityExecutor())
        
        loc = robot_data["location"] if "location" in robot_data else None
        if "pose" in robot_data:
            pose = Pose.from_list(robot_data["pose"])
        else:
            pose = None
        self.world.add_robot(robot, loc=loc, pose=pose)


    def add_planner(self):
        """ Adds a global planner to the world """
        if "planning" not in self.data or "planner_type" not in self.data["planning"]:
            return
        planning = self.data["planning"]
        planner_type = planning["planner_type"]
        
        if planner_type == "search_graph":
            max_edge_dist = get_value_or(planning, "max_edge_dist", default=np.inf)
            collision_check_dist = get_value_or(planning, "collision_check_dist", default=0.1)
            self.world.create_search_graph(
                max_edge_dist=max_edge_dist, collision_check_dist=collision_check_dist)
