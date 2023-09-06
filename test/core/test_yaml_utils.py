#!/usr/bin/env python3

"""Unit tests for world YAML loading utilities."""

import pytest

from pyrobosim.core.locations import Location
from pyrobosim.core.objects import Object
from pyrobosim.core.world import World
from pyrobosim.core.yaml_utils import WorldYamlLoader
from pyrobosim.utils.polygon import polygon_and_height_from_footprint
from pyrobosim.utils.pose import Pose


class TestWorldYamlLoading:
    """Class to test world YAML loading."""

    @staticmethod
    @pytest.mark.dependency()
    def test_create_world_yaml_loader():
        """Creates a WorldYamlLoader object."""
        TestWorldYamlLoading.yaml_loader = WorldYamlLoader()

        # Clean up metadata for test reproducibility
        if hasattr(Location, "metadata"):
            delattr(Location, "metadata")
        if hasattr(Object, "metadata"):
            delattr(Object, "metadata")

    @staticmethod
    @pytest.mark.dependency(
        depends=["TestWorldYamlLoading::test_create_world_yaml_loader"]
    )
    def test_create_world_from_yaml():
        """Tests creating a world from YAML data."""
        loader = TestWorldYamlLoading.yaml_loader
        loader.filename = "test_world.yaml"

        # If no parameters are provided, loader should load a default world
        loader.data = {}
        loader.create_world()
        assert isinstance(loader.world, World)
        assert not hasattr(Location, "metadata")
        assert not hasattr(Object, "metadata")

        # If parameters are provided, the loader should load a world using the specified parameters
        params_dict = {
            "name": "test_world",
            "inflation_radius": 0.125,
            "object_radius": 0.03,
            "wall_height": 1.6,
        }
        loader.data = {"params": params_dict}
        loader.create_world()
        assert loader.world.name == "test_world"
        assert loader.world.inflation_radius == 0.125
        assert loader.world.object_radius == 0.03
        assert loader.world.wall_height == 1.6
        assert not hasattr(Location, "metadata")
        assert not hasattr(Object, "metadata")

        # Specifying Location and Object metadata should load it into the world.
        metadata_dict = {
            "locations": "$DATA/example_location_data.yaml",
            "objects": "$DATA/example_object_data.yaml",
        }
        loader.data = {
            "params": params_dict,
            "metadata": metadata_dict,
        }
        loader.create_world()
        assert hasattr(Location, "metadata")
        assert hasattr(Object, "metadata")

    @staticmethod
    @pytest.mark.dependency(
        depends=["TestWorldYamlLoading::test_create_world_from_yaml"]
    )
    def test_create_rooms_from_yaml():
        """Tests adding rooms to a world from YAML data."""
        loader = TestWorldYamlLoading.yaml_loader

        # No room data means no rooms should be added.
        loader.data = {}
        loader.add_rooms()
        assert len(loader.world.rooms) == 0

        # Load rooms from a YAML specified dictionary.
        rooms_dict = {
            "rooms": [
                {
                    "name": "kitchen",
                    "footprint": {
                        "type": "polygon",
                        "coords": [[-1.0, -1.0], [1.5, -1.0], [1.5, 1.5], [0.5, 1.5]],
                    },
                    "nav_poses": [[0.75, 0.5, 0.0]],
                    "wall_width": 0.2,
                    "color": [1, 0, 0],
                },
                {
                    "name": "bedroom",
                    "footprint": {
                        "type": "box",
                        "dims": [1.75, 1.5],
                        "offset": [2.625, 3.25],
                    },
                    "wall_width": 0.2,
                    "color": [0, 1, 0],
                },
            ],
        }
        loader.data = rooms_dict
        loader.add_rooms()
        assert len(loader.world.rooms) == 2

        assert loader.world.rooms[0].name == "kitchen"
        poly, _ = polygon_and_height_from_footprint(rooms_dict["rooms"][0]["footprint"])
        assert loader.world.rooms[0].polygon == poly
        assert loader.world.rooms[0].wall_width == 0.2
        assert loader.world.rooms[0].viz_color == [1, 0, 0]
        assert loader.world.rooms[0].nav_poses == [Pose.from_list([0.75, 0.5, 0.0])]

        assert loader.world.rooms[1].name == "bedroom"
        poly, _ = polygon_and_height_from_footprint(rooms_dict["rooms"][1]["footprint"])
        assert loader.world.rooms[1].polygon == poly
        assert loader.world.rooms[1].wall_width == 0.2
        assert loader.world.rooms[1].viz_color == [0, 1, 0]
        assert loader.world.rooms[1].nav_poses == [
            Pose.from_list(loader.world.rooms[1].centroid)
        ]

    @staticmethod
    @pytest.mark.dependency(
        depends=["TestWorldYamlLoading::test_create_rooms_from_yaml"]
    )
    def test_create_hallways_from_yaml():
        """Tests adding hallways to a world from YAML data."""
        loader = TestWorldYamlLoading.yaml_loader

        # No hallway data means no hallways should be added.
        loader.data = {}
        loader.add_hallways()
        assert len(loader.world.hallways) == 0

        # Load hallways from a YAML specified dictionary.
        hallways_dict = {
            "hallways": [
                {
                    "room_start": "kitchen",
                    "room_end": "bedroom",
                    "width": 0.75,
                    "wall_width": 0.18,
                    "conn_method": "auto",
                    "color": [0.5, 0.5, 0.5],
                },
            ]
        }
        loader.data = hallways_dict
        loader.add_hallways()
        assert len(loader.world.hallways) == 1

        assert loader.world.hallways[0].room_start.name == "kitchen"
        assert loader.world.hallways[0].room_end.name == "bedroom"
        assert loader.world.hallways[0].width == 0.75
        assert loader.world.hallways[0].wall_width == 0.18
        assert loader.world.hallways[0].viz_color == [0.5, 0.5, 0.5]

    @staticmethod
    @pytest.mark.dependency(
        depends=["TestWorldYamlLoading::test_create_hallways_from_yaml"]
    )
    def test_create_locations_from_yaml():
        """Tests adding locations to a world from YAML data."""
        loader = TestWorldYamlLoading.yaml_loader

        # No location data means no locations should be added.
        loader.data = {}
        loader.add_locations()
        assert len(loader.world.locations) == 0

        # No parent means the location is not added.
        loader.data = {
            "locations": [
                {
                    "category": "table",
                    "pose": [0.85, -0.5, 0.0, -1.57],
                }
            ]
        }
        with pytest.warns(UserWarning):
            loader.add_locations()
            assert len(loader.world.locations) == 0

        # Invalid location category means the object is not added.
        loader.data = {
            "locations": [
                {
                    "category": "does_not_exist",
                    "parent": "kitchen",
                    "pose": [0.85, -0.5, 0.0, -1.57],
                }
            ]
        }
        with pytest.warns(UserWarning):
            loader.add_locations()
            assert len(loader.world.locations) == 0

        # Load locations from a YAML specified dictionary.
        locations_dict = {
            "locations": [
                {
                    "category": "table",
                    "parent": "kitchen",
                    "pose": [0.85, -0.5, 0.0, -1.57],
                },
                {
                    "name": "test_desk",
                    "category": "desk",
                    "parent": "bedroom",
                    "pose": [3.15, 3.65, 0.0, 0.0],
                },
            ],
        }
        loader.data = locations_dict
        loader.add_locations()
        assert len(loader.world.locations) == 2

        assert loader.world.locations[0].name == "table0"  # Auto-named
        assert loader.world.locations[0].category == "table"
        assert loader.world.locations[0].parent.name == "kitchen"
        assert loader.world.locations[0].pose == Pose.from_list(
            [0.85, -0.5, 0.0, -1.57]
        )

        assert loader.world.locations[1].name == "test_desk"
        assert loader.world.locations[1].category == "desk"
        assert loader.world.locations[1].parent.name == "bedroom"
        assert loader.world.locations[1].pose == Pose.from_list([3.15, 3.65, 0.0, 0.0])

    @staticmethod
    @pytest.mark.dependency(
        depends=["TestWorldYamlLoading::test_create_locations_from_yaml"]
    )
    def test_create_objects_from_yaml():
        """Tests adding objects to a world from YAML data."""
        loader = TestWorldYamlLoading.yaml_loader

        # No object data means no objects should be added.
        loader.data = {}
        loader.add_objects()
        assert len(loader.world.objects) == 0

        # No parent means the object is not added.
        loader.data = {
            "objects": [
                {
                    "category": "banana",
                    "pose": [3.2, 3.5, 0.0, 0.707],
                }
            ]
        }
        with pytest.warns(UserWarning):
            loader.add_objects()
            assert len(loader.world.objects) == 0

        # Invalid object category means the object is not added.
        loader.data = {
            "objects": [
                {
                    "category": "does_not_exist",
                    "parent": "table0",
                    "pose": [3.2, 3.5, 0.0, 0.707],
                }
            ]
        }
        with pytest.warns(UserWarning):
            loader.add_objects()
            assert len(loader.world.objects) == 0

        # Load objects from a YAML specified dictionary.
        objects_dict = {
            "objects": [
                {
                    "category": "banana",
                    "parent": "table0",
                },
                {
                    "name": "test_apple",
                    "category": "apple",
                    "parent": "test_desk",
                    "pose": [3.2, 3.5, 0.0, 0.707],
                },
            ],
        }
        loader.data = objects_dict
        loader.add_objects()
        assert len(loader.world.objects) == 2

        assert loader.world.objects[0].name == "banana0"  # Auto-named
        assert loader.world.objects[0].category == "banana"
        assert loader.world.objects[0].parent.name == "table0_tabletop"

        assert loader.world.objects[1].name == "test_apple"
        assert loader.world.objects[1].category == "apple"
        assert loader.world.objects[1].parent.name == "test_desk_desktop"
        # NOTE: The height of the object gets modified to the height of the object spawn
        height = loader.world.get_entity_by_name("test_desk_desktop").height
        assert loader.world.objects[1].pose == Pose.from_list([3.2, 3.5, height, 0.707])

    @staticmethod
    @pytest.mark.dependency(
        depends=["TestWorldYamlLoading::test_create_objects_from_yaml"]
    )
    def test_create_robots_from_yaml():
        """Tests adding robots to a world from YAML data."""
        from pyrobosim.manipulation.grasping import (
            GraspGenerator,
            ParallelGraspProperties,
        )
        from pyrobosim.navigation import PathPlanner
        from pyrobosim.navigation.execution import ConstantVelocityExecutor

        loader = TestWorldYamlLoading.yaml_loader

        # No robot data means no robots should be added.
        loader.data = {}
        loader.add_robots()
        assert len(loader.world.robots) == 0

        # Load robots from a YAML specified dictionary.
        robots_dict = {
            "robots": [
                {
                    # Minimal specification
                    "radius": 0.12,
                    "color": [0.8, 0.0, 0.8],
                    "location": "kitchen",
                },
                {
                    # Full specification
                    "name": "test_robot",
                    "radius": 0.09,
                    "color": [0.0, 0.8, 0.8],
                    "location": "bedroom",
                    "pose": [2.5, 3.0, 1.57],
                    "path_planner": {
                        "type": "rrt",
                        "collision_check_step_dist": 0.025,
                        "max_connection_dist": 0.5,
                        "bidirectional": True,
                        "rrt_star": True,
                        "rewire_radius": 1.5,
                    },
                    "path_executor": {
                        "type": "constant_velocity",
                        "linear_velocity": 1.0,
                        "max_angular_velocity": 3.14,
                        "dt": 0.1,
                    },
                    "grasping": {
                        "generator": "parallel_grasp",
                        "max_width": 0.175,
                        "depth": 0.1,
                        "height": 0.04,
                        "width_clearance": 0.01,
                        "depth_clearance": 0.01,
                    },
                },
            ],
        }
        loader.data = robots_dict
        loader.add_robots()
        assert len(loader.world.robots) == 2

        assert loader.world.robots[0].name == "robot0"  # Auto-named
        assert loader.world.robots[0].radius == 0.12
        assert loader.world.robots[0].location.name == "kitchen"
        assert loader.world.robots[0].path_planner is None
        assert isinstance(
            loader.world.robots[0].path_executor, ConstantVelocityExecutor
        )
        assert loader.world.robots[0].grasp_generator is None

        assert loader.world.robots[1].name == "test_robot"
        assert loader.world.robots[1].radius == 0.09
        assert loader.world.robots[1].location.name == "bedroom"
        assert loader.world.robots[1].pose == Pose.from_list([2.5, 3.0, 1.57])

        path_planner = loader.world.robots[1].path_planner
        assert isinstance(path_planner, PathPlanner)
        assert path_planner.planner_type == "rrt"
        assert path_planner.planner_config["collision_check_step_dist"] == 0.025
        assert path_planner.planner_config["max_connection_dist"] == 0.5
        assert path_planner.planner_config["bidirectional"] == True
        assert path_planner.planner_config["rrt_star"] == True
        assert path_planner.planner_config["rewire_radius"] == 1.5

        path_executor = loader.world.robots[1].path_executor
        assert isinstance(path_executor, ConstantVelocityExecutor)
        assert path_executor.linear_velocity == 1.0
        assert path_executor.max_angular_velocity == 3.14
        assert path_executor.dt == 0.1

        grasp_generator = loader.world.robots[1].grasp_generator
        assert isinstance(grasp_generator, GraspGenerator)
        assert isinstance(grasp_generator.properties, ParallelGraspProperties)
        assert grasp_generator.properties.max_width == 0.175
        assert grasp_generator.properties.depth == 0.1
        assert grasp_generator.properties.height == 0.04
        assert grasp_generator.properties.width_clearance == 0.01
        assert grasp_generator.properties.depth_clearance == 0.01
