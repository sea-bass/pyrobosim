""" Utilities to create worlds from YAML files. """

import os
import warnings
import yaml

from .robot import Robot
from .world import World
from ..utils.general import replace_special_yaml_tokens
from ..utils.pose import Pose
from ..navigation import (
    ConstantVelocityExecutor,
    OccupancyGrid,
    PathPlanner,
)


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
        return self.world

    def create_world(self):
        """Creates an initial world with the specified global parameters."""
        if "params" in self.data:
            params = self.data["params"]
            self.world = World(
                name=params.get("name", "world"),
                inflation_radius=params.get("inflation_radius", 0.0),
                object_radius=params.get("object_radius", 0.0),
                wall_height=params.get("wall_height", 2.0),
            )
        else:
            self.world = World()

        # Set the location/object metadata
        (world_dir, _) = os.path.split(self.filename)
        metadata = self.data.get("metadata")
        if metadata:
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
        for room_data in self.data.get("rooms", []):
            # TODO: Find a way to parse poses as YAML.
            if "nav_poses" in room_data:
                room_data["nav_poses"] = [
                    Pose.from_list(p) for p in room_data["nav_poses"]
                ]

            self.world.add_room(**room_data)

    def add_hallways(self):
        """Add hallways connecting rooms to the world."""
        for hall_data in self.data.get("hallways", []):
            self.world.add_hallway(**hall_data)

    def add_locations(self):
        """Add locations for object spawning to the world."""
        for loc_data in self.data.get("locations", []):
            # TODO: Find a way to parse poses as YAML.
            loc_data["pose"] = Pose.from_list(loc_data["pose"])

            self.world.add_location(**loc_data)

    def add_objects(self):
        """Add objects to the world."""
        if "objects" not in self.data:
            return

        for obj_data in self.data.get("objects", []):
            # TODO: Find a way to parse poses as YAML.
            if "pose" in obj_data:
                obj_data["pose"] = Pose.from_list(obj_data["pose"])
            self.world.add_object(**obj_data)

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
            if loc:
                loc = self.world.get_entity_by_name(loc)
            if "pose" in robot_data:
                pose = Pose.from_list(robot_data["pose"])
            else:
                pose = None
            self.world.add_robot(robot, loc=loc, pose=pose)

    def get_local_path_planner(self, robot_data):
        """Gets local planner path planner to a robot."""
        if "path_planner" not in robot_data:
            return None

        planner_data = robot_data["path_planner"]
        planner_type = planner_data["type"]
        planner_data.pop("type")
        occupancy_grid = planner_data.get("occupancy_grid", None)
        if occupancy_grid:
            resolution = occupancy_grid.get("resolution", 0.05)
            inflation_radius = occupancy_grid.get("inflation_radius", 0.15)
            occupancy_grid = OccupancyGrid.from_world(
                self.world, resolution, inflation_radius
            )
            # Remove the metadata about occupancy grid.
            planner_data.pop("occupancy_grid")
            planner_data["grid"] = occupancy_grid

        # We only need to include a world object if occupancy grid was not specified.
        if not occupancy_grid:
            planner_data["world"] = self.world
        path_planner = PathPlanner(planner_type, **planner_data)

        return path_planner

    def get_path_executor(self, robot_data):
        """Gets a path executor to add to a robot."""
        if "path_executor" not in robot_data:
            return ConstantVelocityExecutor()

        path_executor_data = robot_data["path_executor"]
        path_executor_type = path_executor_data["type"]
        if path_executor_type == "constant_velocity":
            return ConstantVelocityExecutor(
                linear_velocity=path_executor_data.get("linear_velocity", 1.0),
                dt=path_executor_data.get("dt", 0.1),
                max_angular_velocity=path_executor_data.get("max_angular_velocity"),
            )
        else:
            warnings.warn(f"Invalid path executor type specified: {path_executor_type}")
            return None

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
