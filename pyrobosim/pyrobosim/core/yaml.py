# -*- coding: utf-8 -*-
""" Utilities to create worlds from YAML files. """

import numpy as np
import os
import warnings
import yaml

from .robot import Robot
from .room import Room
from .world import World
from ..utils.general import replace_special_yaml_tokens
from ..utils.pose import Pose


class WorldYamlLoader:
    """Creates world models from YAML files."""

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
        self.add_robots()
        self.add_global_path_planner()
        return self.world

    def create_world(self):
        """Creates an initial world with the specified global parameters."""
        if "params" in self.data:
            params = self.data["params"]
            name = params.get("name", "world")
            inf_radius = params.get("inflation_radius", 0.0)
            obj_radius = params.get("object_radius", 0.0)
            wall_height = params.get("wall_height", 2.0)
            self.world = World(
                name=name,
                inflation_radius=inf_radius,
                object_radius=obj_radius,
                wall_height=wall_height,
            )
        else:
            self.world = World()

        # Set the location/object metadata
        (world_dir, _) = os.path.split(self.filename)
        metadata = self.data["metadata"]
        if "locations" in metadata:
            loc_data = replace_special_yaml_tokens(metadata["locations"], world_dir)
        else:
            loc_data = None
        if "objects" in metadata:
            obj_data = replace_special_yaml_tokens(metadata["objects"], world_dir)
        else:
            obj_data = None
        self.world.set_metadata(locations=loc_data, objects=obj_data)

    def add_rooms(self):
        """Add rooms to the world."""
        if "rooms" not in self.data:
            return

        for room_data in self.data["rooms"]:
            name = room_data.get("name", None)
            color = room_data.get("color", [0.4, 0.4, 0.4])
            wall_width = room_data.get("wall_width", 0.2)
            if "nav_poses" in room_data:
                nav_poses = [Pose.from_list(p) for p in room_data["nav_poses"]]
            else:
                nav_poses = None

            room = Room(
                room_data["footprint"],
                name=name,
                color=color,
                wall_width=wall_width,
                nav_poses=nav_poses,
            )
            self.world.add_room(room)

    def add_hallways(self):
        """Add hallways connecting rooms to the world."""
        if "hallways" not in self.data:
            return

        for hall in self.data["hallways"]:
            conn_method = hall.get("conn_method", "auto")
            offset = hall.get("offset", 0.0)
            conn_angle = hall.get("conn_angle", 0.0)
            conn_points = hall.get("conn_points", [])
            color = hall.get("color", [0.4, 0.4, 0.4])
            wall_width = hall.get("wall_width", 0.2)

            self.world.add_hallway(
                hall["from"],
                hall["to"],
                hall["width"],
                conn_method=conn_method,
                offset=offset,
                conn_angle=conn_angle,
                conn_points=conn_points,
                color=color,
                wall_width=wall_width,
            )

    def add_locations(self):
        """Add locations for object spawning to the world."""
        if "locations" not in self.data:
            return

        for loc in self.data["locations"]:
            category = loc["type"]
            room = loc["room"]
            pose = Pose.from_list(loc["pose"])
            name = loc.get("name", None)
            self.world.add_location(category, room, pose, name=name)

    def add_objects(self):
        """Add objects to the world."""
        if "objects" not in self.data:
            return

        for obj in self.data["objects"]:
            category = obj["type"]
            loc = obj["location"]
            if "pose" in obj:
                pose = Pose.from_list(obj["pose"])
            else:
                pose = None
            name = obj.get("name", None)
            self.world.add_object(category, loc, pose=pose, name=name)

    def add_robots(self):
        """Add robots to the world."""
        if "robots" not in self.data:
            return

        for id, robot_data in enumerate(self.data["robots"]):
            # Create the robot
            robot_name = robot_data.get("name", f"robot{id}")
            robot_color = robot_data.get("color", (0.8, 0.0, 0.8))
            robot = Robot(
                name=robot_name,
                radius=robot_data["radius"],
                color=robot_color,
                path_planner=self.get_local_path_planner(robot_data),
                path_executor=self.get_path_executor(robot_data),
                grasp_generator=self.get_grasp_generator(robot_data),
            )

            loc = robot_data["location"] if "location" in robot_data else None
            if "pose" in robot_data:
                pose = Pose.from_list(robot_data["pose"])
            else:
                pose = None
            self.world.add_robot(robot, loc=loc, pose=pose)

    def add_global_path_planner(self):
        """Adds a global path planner to the world."""
        if (
            "global_path_planner" not in self.data
            or "type" not in self.data["global_path_planner"]
        ):
            return
        planner_data = self.data["global_path_planner"]
        planner_type = planner_data["type"]

        if planner_type == "search_graph":
            max_edge_dist = planner_data.get("max_edge_dist", np.inf)
            collision_check_dist = planner_data.get("collision_check_dist", 0.1)
            self.world.create_search_graph(
                max_edge_dist=max_edge_dist,
                collision_check_dist=collision_check_dist,
                create_planner=True,
            )
        else:
            # Always make a search graph as we use it for other things.
            max_edge_dist = planner_data.get("max_edge_dist", np.inf)
            collision_check_dist = planner_data.get("collision_check_dist", 0.1)
            self.world.create_search_graph(
                max_edge_dist=max_edge_dist,
                collision_check_dist=collision_check_dist,
                create_planner=False,
            )

            if planner_type == "prm":
                from pyrobosim.navigation.prm import PRMPlanner

                max_nodes = planner_data.get("max_nodes", 100)
                max_connection_dist = planner_data.get("max_connection_dist", 1.0)
                self.world.path_planner = PRMPlanner(
                    self.world,
                    max_nodes=max_nodes,
                    max_connection_dist=max_connection_dist,
                )
            else:
                warnings.warn(f"Invalid global planner type specified: {planner_type}")

    def get_local_path_planner(self, robot_data):
        """Gets local planner path planner to a robot."""
        if "path_planner" not in robot_data:
            return None
        planner_data = robot_data["path_planner"]
        planner_type = planner_data["type"]

        if planner_type == "rrt":
            from pyrobosim.navigation.rrt import RRTPlanner

            bidirectional = planner_data.get("bidirectional", False)
            rrt_star = planner_data.get("rrt_star", False)
            rrt_connect = planner_data.get("rrt_connect", False)
            max_connection_dist = planner_data.get("max_connection_dist", 0.5)
            max_nodes_sampled = planner_data.get("max_nodes_sampled", 1000)
            max_time = planner_data.get("max_time", 5.0)
            rewire_radius = planner_data.get("rewire_radius", 1.0)
            return RRTPlanner(
                self.world,
                bidirectional=bidirectional,
                rrt_star=rrt_star,
                rrt_connect=rrt_connect,
                max_connection_dist=max_connection_dist,
                max_nodes_sampled=max_nodes_sampled,
                max_time=max_time,
                rewire_radius=rewire_radius,
            )
        else:
            warnings.warn(f"Invalid planner type specified: {planner_type}")
            return None

    def get_path_executor(self, robot_data):
        """Gets a path executor to add to a robot."""
        from pyrobosim.navigation.execution import ConstantVelocityExecutor

        return ConstantVelocityExecutor()

    def get_grasp_generator(self, robot_data):
        """Gets a grasp generator to add to a robot."""
        from pyrobosim.manipulation.grasping import (
            GraspGenerator,
            ParallelGraspProperties,
        )

        if "grasping" not in robot_data:
            return None

        grasp_params = robot_data["grasping"]
        grasp_gen_type = grasp_params["generator"]
        if grasp_gen_type == "parallel_grasp":
            grasp_properties = ParallelGraspProperties(
                max_width=grasp_params.get("max_width", 0.15),
                depth=grasp_params.get("depth", 0.1),
                height=grasp_params.get("height", 0.04),
                width_clearance=grasp_params.get("width_clearance", 0.01),
                depth_clearance=grasp_params.get("depth_clearance", 0.01),
            )
            return GraspGenerator(grasp_properties)
        else:
            warnings.warn(f"Invalid grasp generator type specified: {grasp_gen_type}")
            return None
