"""Utilities to create worlds from YAML files."""

import copy
import os
from typing import Any
import yaml

from .robot import Robot
from .world import World
from ..planning.actions import ExecutionOptions
from ..utils.general import replace_special_yaml_tokens
from ..utils.logging import get_global_logger
from ..utils.pose import Pose


def construct_pose_from_args(pose_args: dict[str, Any], world: World) -> Pose:
    """
    Constructs a PyRoboSim pose from a dictionary of YAML arguments.

    This handles poses of different formats, as well as relative pose specification.

    :param pose_args: A dictionary containing the pose arguments.
    :param world: The world to use for looking up entities.
    :return: Pose object
    """
    pose = Pose.construct(pose_args)
    if "relative_to" in pose_args:
        pose = world.get_pose_relative_to(pose, pose_args["relative_to"])
    return pose


class WorldYamlLoader:
    """Creates world models from YAML files."""

    def from_yaml(
        self,
        world_dict: dict[str, Any],
        world_dir: str | None = None,
        world: World | None = None,
    ) -> World:
        """
        Load a world from a YAML description.

        :param world_dict: Dictionary containing all the world information
        :param world_dir: Root directory for basing some tokens, uses the current directory if not specified.
        :param world: If specified, loads the YAML contents to this world.
            Otherwise, creates and returns a new world instance.
        :return: World model instance.
        """
        self.data = world_dict
        self.world_dir = world_dir

        # Build and return the world
        self.create_world(world)
        self.add_rooms()
        self.add_hallways()
        self.add_locations()
        self.add_objects()
        self.add_robots()
        return self.world

    def from_file(self, filename: str, world: World | None = None) -> World:
        """
        Load a world from a YAML file.

        :param filename: Path to YAML file describing the world.
        :param world: If specified, loads the YAML contents to this world.
            Otherwise, creates and returns a new world instance.
        :return: World model instance.
        """
        with open(filename) as file:
            world_dict = yaml.load(file, Loader=yaml.FullLoader)
        (world_dir, _) = os.path.split(filename)
        world = self.from_yaml(world_dict, world_dir, world=world)
        world.source_yaml_file = filename
        return world

    def create_world(self, world: World | None = None) -> None:
        """
        Creates an initial world with the specified global parameters.

        :param world: If specified, loads the YAML contents to this world.
            Otherwise, creates and returns a new world instance.
        """
        params = self.data.get("params", {})

        if world is None:
            self.world = World(**params)
        else:
            world.remove_all_objects()
            world.remove_all_robots(remove_ros_interfaces=False)  # OK for resetting.
            world.remove_all_locations()
            world.remove_all_hallways()
            world.remove_all_rooms()
            self.world = world

        # Set the location/object metadata
        metadata = self.data.get("metadata")
        if metadata:
            if "locations" in metadata:
                loc_data = replace_special_yaml_tokens(
                    metadata["locations"], self.world_dir
                )
            else:
                loc_data = None
            if "objects" in metadata:
                obj_data = replace_special_yaml_tokens(
                    metadata["objects"], self.world_dir
                )
            else:
                obj_data = None

            self.world.add_metadata(locations=loc_data, objects=obj_data)

    def add_rooms(self) -> None:
        """Add rooms to the world."""
        for room_data in self.data.get("rooms", []):
            room_args = copy.deepcopy(room_data)
            if "pose" in room_args:
                room_args["pose"] = construct_pose_from_args(
                    room_args["pose"], self.world
                )
            if "nav_poses" in room_args:
                room_args["nav_poses"] = [
                    construct_pose_from_args(p, self.world)
                    for p in room_args["nav_poses"]
                ]
            self.world.add_room(**room_args)

    def add_hallways(self) -> None:
        """Add hallways connecting rooms to the world."""
        for hall_data in self.data.get("hallways", []):
            self.world.add_hallway(**hall_data)

    def add_locations(self) -> None:
        """Add locations for object spawning to the world."""
        for loc_data in self.data.get("locations", []):
            loc_args = copy.deepcopy(loc_data)
            if "pose" in loc_args:
                loc_args["pose"] = construct_pose_from_args(
                    loc_args["pose"], self.world
                )
            self.world.add_location(**loc_args, show=False)

    def add_objects(self) -> None:
        """Add objects to the world."""
        if "objects" not in self.data:
            return

        for obj_data in self.data.get("objects", []):
            obj_args = copy.deepcopy(obj_data)
            if "pose" in obj_args:
                obj_args["pose"] = construct_pose_from_args(
                    obj_args["pose"], self.world
                )
            self.world.add_object(**obj_args, show=False)

    def add_robots(self) -> None:
        """Add robots to the world."""
        for id, robot_data in enumerate(self.data.get("robots", [])):
            # Create the robot
            robot_args = copy.deepcopy(robot_data)
            if "location" in robot_args:
                del robot_args["location"]
            if "name" not in robot_args:
                robot_args["name"] = f"robot{id}"
            robot_args["path_planner"] = self.get_path_planner(robot_args)
            robot_args["path_executor"] = self.get_path_executor(robot_args)
            robot_args["grasp_generator"] = self.get_grasp_generator(robot_args)
            robot_args["action_execution_options"] = self.get_action_execution_options(
                robot_args
            )
            robot_args["sensors"] = self.get_sensors(robot_args)
            robot = Robot(**robot_args)

            loc = robot_data.get("location")
            if "pose" in robot_args:
                pose = construct_pose_from_args(robot_args["pose"], self.world)
            else:
                pose = None
            self.world.add_robot(robot, loc=loc, pose=pose, show=False)

        # Clean up any unused ROS interface due to resetting the world.
        new_robot_names = self.world.get_robot_names()
        if self.world.ros_node is not None:
            for robot_name in self.world.ros_node.latest_robot_cmds:
                if robot_name not in new_robot_names:
                    self.world.ros_node.remove_robot_interfaces(robot_name)

    def get_path_planner(self, robot_data: dict[str, Any]) -> Any:
        """Gets path planner to add to a robot."""
        from ..navigation.types import PathPlanner

        if "path_planner" not in robot_data:
            return None

        planner_data = robot_data["path_planner"]
        planner_type = planner_data["type"]
        planner_data.pop("type")

        planner_class = PathPlanner.registered_plugins.get(planner_type)
        if planner_class is None:
            raise RuntimeError(
                f"Path planner '{planner_type}' is not available. Make sure you have imported its implementation."
            )
        path_planner = planner_class(**planner_data)
        del robot_data["path_planner"]
        return path_planner

    def get_path_executor(self, robot_data: dict[str, Any]) -> Any:
        """Gets a path executor to add to a robot."""
        from ..navigation.execution import ConstantVelocityExecutor

        if "path_executor" not in robot_data:
            return ConstantVelocityExecutor()

        path_executor_data = robot_data["path_executor"].copy()
        path_executor_type = path_executor_data["type"]
        del robot_data["path_executor"]
        del path_executor_data["type"]
        if path_executor_type == "constant_velocity":
            return ConstantVelocityExecutor(**path_executor_data)
        else:
            get_global_logger().warning(
                f"Invalid path executor type specified: {path_executor_type}"
            )
            return None

    def get_sensors(self, robot_data: dict[str, Any]) -> Any:
        """Gets a dictionary of sensors to add to a robot."""
        from ..sensors.types import Sensor

        if "sensors" not in robot_data:
            return None

        sensor_args: dict[str, Sensor] = {}
        sensor_data = robot_data["sensors"].copy()

        for name, sensor in sensor_data.items():
            sensor_type = sensor["type"]
            sensor.pop("type")
            sensor_class = Sensor.registered_plugins.get(sensor_type)
            if sensor_class is None:
                raise RuntimeError(
                    f"Sensor '{sensor_type}' is not available. Make sure you have imported its implementation."
                )
            sensor_args[name] = sensor_class(**sensor)

        del robot_data["sensors"]
        return sensor_args

    def get_grasp_generator(self, robot_data: dict[str, Any]) -> Any:
        """Gets a grasp generator to add to a robot."""
        from pyrobosim.manipulation.grasping import (
            GraspGenerator,
            ParallelGraspProperties,
        )

        if "grasping" not in robot_data:
            return None

        grasp_params = robot_data["grasping"].copy()
        grasp_gen_type = grasp_params["generator"]
        del robot_data["grasping"]
        del grasp_params["generator"]
        if grasp_gen_type == "parallel_grasp":
            grasp_properties = ParallelGraspProperties(**grasp_params)
            return GraspGenerator(grasp_properties)
        else:
            get_global_logger().warning(
                f"Invalid grasp generator type specified: {grasp_gen_type}"
            )
            return None

    def get_action_execution_options(
        self, robot_data: dict[str, Any]
    ) -> dict[str, ExecutionOptions]:
        """Gets action execution information to add to a robot."""
        action_execution_options = {}
        exec_data = robot_data.get("action_execution_options", {})

        for action_name, kwargs in exec_data.items():
            action_execution_options[action_name] = ExecutionOptions(**kwargs)

        return action_execution_options


class WorldYamlWriter:
    """Creates YAML files from world models."""

    def to_dict(self, world: World) -> dict[str, Any]:
        """
        Serializes a world to a dictionary.

        :param world: The world model to serialize.
        :return: The dictionary containing the world information.
        """
        # Extract the global world parameters to YAML.
        world_dict: dict[str, Any] = {
            "params": {
                "name": world.name,
                "inflation_radius": world.inflation_radius,
                "object_radius": world.object_radius,
                "wall_height": world.wall_height,
            }
        }

        # Extract the location and object metadata.
        loc_metadata_files = list(world.get_location_metadata().sources)
        obj_metadata_files = list(world.get_object_metadata().sources)

        world_dict["metadata"] = {
            "locations": loc_metadata_files,
            "objects": obj_metadata_files,
        }

        # Go through all the entities in the world and similarly add them to the dictionary.
        if len(world.robots) > 0:
            world_dict["robots"] = [robot.to_dict() for robot in world.robots]
        if len(world.rooms) > 0:
            world_dict["rooms"] = [room.to_dict() for room in world.rooms]
        if len(world.hallways) > 0:
            world_dict["hallways"] = [hall.to_dict() for hall in world.hallways]
        if len(world.locations) > 0:
            world_dict["locations"] = [loc.to_dict() for loc in world.locations]
        if len(world.objects) > 0:
            world_dict["objects"] = [obj.to_dict() for obj in world.objects]

        return world_dict

    def to_file(self, world: World, filename: str) -> None:
        """
        Serializes a world to a YAML file.

        :param world: The world model to serialize.
        :param filename: The name of the file to write to.
        """
        world_dict = self.to_dict(world)
        with open(filename, "w") as out_file:
            yaml.dump(
                world_dict,
                out_file,
                default_flow_style=False,
                indent=2,
                sort_keys=False,
            )
