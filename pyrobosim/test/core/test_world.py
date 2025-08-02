"""
Unit tests for core world modeling.
"""

import os
import pytest
from pytest import LogCaptureFixture
import numpy as np

from pyrobosim.core import Hallway, Object, World
from pyrobosim.utils.pose import Pose

from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.world_collision import check_occupancy, is_connectable


class TestWorldModeling:
    """Tests for the world modeling tools"""

    world = World()

    @staticmethod
    @pytest.mark.dependency()  # type: ignore[misc]
    def test_clearing_old_metadata() -> None:
        """Tests the creation of a world and clear out old metadata"""

        data_folder = get_data_folder()
        TestWorldModeling.world.add_metadata(
            locations=[
                os.path.join(data_folder, "example_location_data_furniture.yaml"),
                os.path.join(data_folder, "example_location_data_accessories.yaml"),
            ],
            objects=[
                os.path.join(data_folder, "example_object_data_food.yaml"),
                os.path.join(data_folder, "example_object_data_drink.yaml"),
            ],
        )

        assert isinstance(TestWorldModeling.world, World)
        assert len(TestWorldModeling.world.get_location_metadata().sources) == 2
        assert len(TestWorldModeling.world.get_object_metadata().sources) == 2

        # Clear out previous metadata before setting new one
        TestWorldModeling.world.set_metadata(
            locations=os.path.join(data_folder, "example_location_data_furniture.yaml"),
            objects=os.path.join(data_folder, "example_object_data_food.yaml"),
        )

        assert len(TestWorldModeling.world.get_location_metadata().sources) == 1
        assert len(TestWorldModeling.world.get_object_metadata().sources) == 1

        # Add more metadata
        TestWorldModeling.world.add_metadata(
            locations=[
                os.path.join(data_folder, "example_location_data_accessories.yaml")
            ],
            objects=[os.path.join(data_folder, "example_object_data_drink.yaml")],
        )

        assert len(TestWorldModeling.world.get_location_metadata().sources) == 2
        assert len(TestWorldModeling.world.get_object_metadata().sources) == 2

    @staticmethod
    @pytest.mark.dependency()  # type: ignore[misc]
    def test_create_world_default() -> None:
        """Tests the creation of a world"""

        TestWorldModeling.world = World()

        data_folder = get_data_folder()
        TestWorldModeling.world.set_metadata(
            locations=[
                os.path.join(data_folder, "example_location_data_furniture.yaml"),
                os.path.join(data_folder, "example_location_data_accessories.yaml"),
            ],
            objects=[
                os.path.join(data_folder, "example_object_data_food.yaml"),
                os.path.join(data_folder, "example_object_data_drink.yaml"),
            ],
        )

        assert isinstance(TestWorldModeling.world, World)
        assert len(TestWorldModeling.world.get_location_metadata().sources) == 2
        assert len(TestWorldModeling.world.get_object_metadata().sources) == 2

    ##############################################
    # These tests incrementally build up a world #
    ##############################################

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_create_world_default"])  # type: ignore[misc]
    def test_create_room() -> None:
        """Tests the creation of a room"""

        room1_name = "kitchen"
        room1_coords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
        TestWorldModeling.world.add_room(
            name=room1_name, footprint=room1_coords, color=[1, 0, 0]
        )

        assert len(TestWorldModeling.world.rooms) == 1
        assert TestWorldModeling.world.get_room_names() == ["kitchen"]
        assert TestWorldModeling.world.get_room_by_name(room1_name) is not None

        room2_name = "bedroom"
        room2_coords = [(1.75, 2.5), (3.5, 2.5), (3.5, 4), (1.75, 4)]
        TestWorldModeling.world.add_room(
            name=room2_name, footprint=room2_coords, color=[1, 0, 0]
        )

        assert len(TestWorldModeling.world.rooms) == 2
        assert TestWorldModeling.world.get_room_names() == ["kitchen", "bedroom"]
        assert TestWorldModeling.world.get_room_by_name(room2_name) is not None

    @staticmethod
    @pytest.mark.dependency(  # type: ignore[misc]
        depends=[
            "TestWorldModeling::test_create_world_default",
            "TestWorldModeling::test_create_room",
        ]
    )
    def test_create_hallway() -> None:
        """Tests the creation of a hallway between 2 rooms"""

        hallway = TestWorldModeling.world.add_hallway(
            room_start="kitchen",
            room_end="bedroom",
            offset=0.5,
            conn_method="auto",
            width=0.5,
        )
        assert len(TestWorldModeling.world.hallways) == 1
        assert TestWorldModeling.world.get_hallway_names() == ["hall_bedroom_kitchen"]
        assert (
            TestWorldModeling.world.get_hallway_by_name("hall_kitchen_bedroom")
            == hallway
        )
        assert (
            TestWorldModeling.world.get_hallway_by_name("hall_bedroom_kitchen")
            == hallway
        )
        assert TestWorldModeling.world.get_hallways_from_rooms(
            "kitchen", "bedroom"
        ) == [hallway]
        assert TestWorldModeling.world.get_hallways_from_rooms(
            "bedroom", "kitchen"
        ) == [hallway]
        assert TestWorldModeling.world.get_hallways_attached_to_room("bedroom") == [
            hallway
        ]
        assert TestWorldModeling.world.get_hallways_attached_to_room("kitchen") == [
            hallway
        ]

    @staticmethod
    @pytest.mark.dependency(  # type: ignore[misc]
        depends=[
            "TestWorldModeling::test_create_world_default",
            "TestWorldModeling::test_create_room",
            "TestWorldModeling::test_create_hallway",
        ]
    )
    def test_create_location(caplog: LogCaptureFixture) -> None:
        """Tests the creation of locations"""
        table = TestWorldModeling.world.add_location(
            category="table",
            parent="kitchen",
            pose=Pose(x=0.85, y=-0.5, z=0.0, yaw=-np.pi / 2.0),
        )

        assert len(TestWorldModeling.world.locations) == 1
        assert (
            TestWorldModeling.world.get_location_by_name("table0") == table
        )  # automatic naming check
        assert TestWorldModeling.world.get_locations() == [table]
        assert TestWorldModeling.world.get_locations(category_list=["table"]) == [table]
        assert TestWorldModeling.world.get_locations(category_list=["desk"]) == []
        assert TestWorldModeling.world.get_location_names() == ["table0"]
        assert TestWorldModeling.world.get_location_names(category_list=["table"]) == [
            "table0"
        ]
        assert TestWorldModeling.world.get_location_names(category_list=["desk"]) == []

        desk = TestWorldModeling.world.add_location(
            category="desk",
            parent="bedroom",
            name="study_desk",
            pose=Pose(x=3.15, y=3.65, z=0.0, yaw=0.0),
        )
        assert len(TestWorldModeling.world.locations) == 2
        assert TestWorldModeling.world.get_location_by_name("study_desk") == desk
        assert TestWorldModeling.world.get_locations() == [table, desk]
        assert TestWorldModeling.world.get_locations(category_list=["table"]) == [table]
        assert TestWorldModeling.world.get_locations(category_list=["desk"]) == [desk]
        assert TestWorldModeling.world.get_location_names() == ["table0", "study_desk"]
        assert TestWorldModeling.world.get_location_names(category_list=["table"]) == [
            "table0"
        ]
        assert TestWorldModeling.world.get_location_names(category_list=["desk"]) == [
            "study_desk"
        ]

        # Check object spawns
        assert TestWorldModeling.world.get_object_spawns() == (
            table.children + desk.children
        )
        assert (
            TestWorldModeling.world.get_object_spawns(category_list=["table"])
            == table.children
        )
        assert (
            TestWorldModeling.world.get_object_spawns(category_list=["desk"])
            == desk.children
        )
        assert TestWorldModeling.world.get_object_spawn_names() == [
            "table0_tabletop",
            "study_desk_desktop",
        ]
        assert TestWorldModeling.world.get_object_spawn_names(
            category_list=["table"]
        ) == ["table0_tabletop"]
        assert TestWorldModeling.world.get_object_spawn_names(
            category_list=["desk"]
        ) == ["study_desk_desktop"]

        # Test missing parent
        result = TestWorldModeling.world.add_location(
            category="desk",
            pose=Pose(),
        )
        assert result is None
        assert "Location instance or parent must be specified." in caplog.text
        caplog.clear()

        # Test invalid category
        result = TestWorldModeling.world.add_location(
            category="does_not_exist",
            parent="bedroom",
            pose=Pose(),
        )
        assert result is None
        assert "Invalid location category: does_not_exist" in caplog.text

    @staticmethod
    @pytest.mark.dependency(  # type: ignore[misc]
        depends=[
            "TestWorldModeling::test_create_world_default",
            "TestWorldModeling::test_create_room",
            "TestWorldModeling::test_create_hallway",
            "TestWorldModeling::test_create_location",
        ]
    )
    def test_create_object(caplog: LogCaptureFixture) -> None:
        """Tests adding objects to a location"""

        apple = TestWorldModeling.world.add_object(category="apple", parent="table0")
        banana = TestWorldModeling.world.add_object(
            category="banana", name="ripe_banana", parent="study_desk"
        )
        assert len(TestWorldModeling.world.objects) == 2

        # second apple
        test_obj = TestWorldModeling.world.objects[1]
        assert isinstance(test_obj, Object)
        assert (
            TestWorldModeling.world.get_object_by_name("apple0") == apple
        )  # Automatic naming
        assert (
            TestWorldModeling.world.get_object_by_name("ripe_banana") == banana
        )  # Manual naming
        assert TestWorldModeling.world.get_objects() == [apple, banana]
        assert TestWorldModeling.world.get_objects(category_list=["apple"]) == [apple]
        assert TestWorldModeling.world.get_objects(category_list=["banana"]) == [banana]
        assert TestWorldModeling.world.get_objects(category_list=["water"]) == []
        assert TestWorldModeling.world.get_object_names() == ["apple0", "ripe_banana"]
        assert TestWorldModeling.world.get_object_names(category_list=["apple"]) == [
            "apple0"
        ]
        assert TestWorldModeling.world.get_object_names(category_list=["banana"]) == [
            "ripe_banana"
        ]
        assert TestWorldModeling.world.get_object_names(category_list=["water"]) == []

        # Test invalid category
        result = TestWorldModeling.world.add_object(
            category="does_not_exist",
            parent="study_desk",
            pose=Pose(),
        )
        assert result is None
        assert "Invalid object category: does_not_exist" in caplog.text

    @staticmethod
    @pytest.mark.dependency(  # type: ignore[misc]
        depends=[
            "TestWorldModeling::test_create_world_default",
            "TestWorldModeling::test_create_room",
            "TestWorldModeling::test_create_hallway",
            "TestWorldModeling::test_create_location",
            "TestWorldModeling::test_create_object",
        ]
    )
    def test_add_robot(caplog: LogCaptureFixture) -> None:
        """Tests adding a robot to the world"""
        from pyrobosim.core import Robot
        from pyrobosim.navigation.rrt import RRTPlanner

        world_robots = TestWorldModeling.world.robots
        path_planner = RRTPlanner()

        # Add a robot at a specific location, but let the pose be sampled.
        robot = Robot(name="test_robot", radius=0.05, path_planner=path_planner)
        TestWorldModeling.world.add_robot(robot, loc="kitchen")
        assert len(world_robots) == 1
        assert isinstance(world_robots[0], Robot)
        assert world_robots[0].location == TestWorldModeling.world.get_entity_by_name(
            "kitchen"
        )

        # Add a robot at a specific location, but let it be sampled.
        robot = Robot(name="other_test_robot", radius=0.05, path_planner=path_planner)
        target_pose = Pose(x=2.5, y=3.0, yaw=np.pi / 2.0)
        TestWorldModeling.world.add_robot(robot, loc="bedroom", pose=target_pose)
        assert len(world_robots) == 2
        assert isinstance(world_robots[1], Robot)
        assert world_robots[1].location == TestWorldModeling.world.get_entity_by_name(
            "bedroom"
        )
        assert world_robots[1].get_pose() == target_pose

        # Add a robot with the same name as an existing robot, which should fail gracefully
        caplog.clear()
        robot = Robot(name="test_robot", radius=0.05, path_planner=path_planner)
        TestWorldModeling.world.add_robot(robot, loc="bedroom")
        assert len(world_robots) == 2
        assert "Robot name test_robot already exists in world." in caplog.text

    #############################################
    # These tests utilize the fully built world #
    #############################################

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_add_robot"])  # type: ignore[misc]
    def test_check_occupancy() -> None:
        """Tests occupancy using the entity polygons."""

        # Free pose in a room
        free_room_pose = Pose(x=1.0, y=1.0)
        assert not check_occupancy(free_room_pose, TestWorldModeling.world)

        # Free pose in a hallway
        free_hallway_pose = Pose(x=1.3, y=2.2)
        assert not check_occupancy(free_hallway_pose, TestWorldModeling.world)

        # Occupied pose in a room due to locations
        occupied_pose_in_table = Pose(x=1.0, y=-0.5)
        assert check_occupancy(occupied_pose_in_table, TestWorldModeling.world)

        # Occupied pose outside the walls
        occupied_pose_outside_walls = Pose(x=0.0, y=3.0)
        assert check_occupancy(occupied_pose_outside_walls, TestWorldModeling.world)

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_add_robot"])  # type: ignore[misc]
    def test_collides_with_robots() -> None:
        """Tests if poses are colliding with robots."""

        robot_0 = TestWorldModeling.world.robots[0]
        robot_1 = TestWorldModeling.world.robots[1]

        # Free poses should not return collision
        free_hallway_pose = Pose(x=1.3, y=2.2)
        assert not TestWorldModeling.world.collides_with_robots(free_hallway_pose)

        # Both robot poses should return collision if no robot is specified.
        assert TestWorldModeling.world.collides_with_robots(robot_0.get_pose())
        assert TestWorldModeling.world.collides_with_robots(robot_1.get_pose())

        # Robots should collide with each other
        assert not TestWorldModeling.world.collides_with_robots(
            robot_0.get_pose(), robot=robot_0
        )
        assert TestWorldModeling.world.collides_with_robots(
            robot_1.get_pose(), robot=robot_0
        )
        assert TestWorldModeling.world.collides_with_robots(
            robot_0.get_pose(), robot=robot_1
        )
        assert not TestWorldModeling.world.collides_with_robots(
            robot_1.get_pose(), robot=robot_1
        )

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_add_robot"])  # type: ignore[misc]
    def test_is_connectable() -> None:
        """Tests if poses are connectable in a straight line."""

        ## Test with a simple straight line in free space
        pose_start = Pose(x=1.0, y=0.25)
        pose_goal = Pose(x=0.5, y=1.0)

        # Default arguments should return True.
        assert is_connectable(pose_start, pose_goal, TestWorldModeling.world)

        # If the max connection distance is larger than the actual distance, this should return False instead.
        assert not is_connectable(
            pose_start, pose_goal, TestWorldModeling.world, max_dist=0.5
        )

        ## Test with a straight line right near a wall
        pose_start = Pose(x=1.5, y=2.4)
        pose_goal = Pose(x=3.0, y=2.75)

        # With a small step distance, this should return False
        assert not is_connectable(
            pose_start, pose_goal, TestWorldModeling.world, step_dist=0.01
        )

        # With a large step distance, this should return True as the collision point is skipped
        assert not is_connectable(
            pose_start, pose_goal, TestWorldModeling.world, step_dist=0.5
        )

    @staticmethod
    @pytest.mark.dependency(  # type: ignore[misc]
        depends=[
            "TestWorldModeling::test_check_occupancy",
            "TestWorldModeling::test_collides_with_robots",
            "TestWorldModeling::test_is_connectable",
        ]
    )
    def test_reset_world() -> None:
        """Tests resetting a world."""
        world = TestWorldModeling.world

        world.reset()
        assert world.num_rooms == 2
        assert world.num_locations == 2
        assert world.num_hallways == 1
        assert world.num_objects == 2
        assert len(world.robots) == 2

        # Reset with "deterministic" mode
        original_robot_0_pose = world.robots[0].get_pose()
        original_apple_pose = world.objects[0].pose
        world.reset(deterministic=True)
        assert world.robots[0].get_pose() == original_robot_0_pose
        assert world.objects[0].pose == original_apple_pose

        # Now reset with a fixed seed.
        seed = 1234
        world.reset(seed=seed)

        assert world.num_rooms == 2
        assert world.num_locations == 2
        assert world.num_hallways == 1
        assert world.num_objects == 2
        assert len(world.robots) == 2

        original_robot_0_pose = world.robots[0].get_pose()
        original_apple_pose = world.objects[0].pose
        for i in range(10):
            world.reset(seed=seed)
            assert world.robots[0].get_pose() == original_robot_0_pose
            assert world.objects[0].pose == original_apple_pose

    ##############################################
    # These tests incrementally clean up a world #
    ##############################################

    @staticmethod
    @pytest.mark.dependency(  # type: ignore[misc]
        depends=["TestWorldModeling::test_reset_world"]
    )
    def test_remove_robot(caplog: LogCaptureFixture) -> None:
        """Tests deleting robots from the world"""

        assert TestWorldModeling.world.remove_robot("test_robot") is True
        assert len(TestWorldModeling.world.robots) == 1

        assert TestWorldModeling.world.remove_robot("other_test_robot") is True
        assert len(TestWorldModeling.world.robots) == 0

        assert TestWorldModeling.world.remove_robot("does_not_exist") is False
        assert "Could not find robot does_not_exist to remove." in caplog.text

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_remove_robot"])  # type: ignore[misc]
    def test_remove_object() -> None:
        """Tests deleting objects from the world"""

        assert TestWorldModeling.world.remove_object("ripe_banana") is True
        assert len(TestWorldModeling.world.locations) == 2
        assert TestWorldModeling.world.get_object_by_name("ripe_banana") is None
        assert TestWorldModeling.world.objects[0].name == "apple0"

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_create_location"])  # type: ignore[misc]
    def test_remove_location(caplog: LogCaptureFixture) -> None:
        """Tests removing a location from the world"""

        assert TestWorldModeling.world.remove_location("study_desk") is True
        assert len(TestWorldModeling.world.locations) == 1
        assert TestWorldModeling.world.get_location_by_name("study_desk") is None
        assert "Location not found: study_desk" in caplog.text

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_create_hallway"])  # type: ignore[misc]
    def test_remove_hallway() -> None:
        """Tests removing a hallway"""

        hallways = TestWorldModeling.world.get_hallways_from_rooms("kitchen", "bedroom")
        assert len(hallways) == 1
        assert isinstance(hallways[0], Hallway)

        assert TestWorldModeling.world.remove_hallway(hallways[0]) is True
        hallways = TestWorldModeling.world.get_hallways_from_rooms("kitchen", "bedroom")
        assert len(hallways) == 0

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_create_room"])  # type: ignore[misc]
    def test_remove_room() -> None:
        """Tests deleting rooms"""

        assert TestWorldModeling.world.remove_room("bedroom") is True
        assert len(TestWorldModeling.world.rooms) == 1
        assert TestWorldModeling.world.rooms[0].name == "kitchen"

    @staticmethod
    @pytest.mark.dependency(  # type: ignore[misc]
        depends=[
            "TestWorldModeling::test_remove_room",
            "TestWorldModeling::test_remove_hallway",
            "TestWorldModeling::test_remove_location",
            "TestWorldModeling::test_remove_object",
        ]
    )
    def test_hierarchical_cleanup() -> None:
        """Tests if an entity is automatically deleted on parent deletion"""

        TestWorldModeling.world.add_object(category="apple", parent="table0")
        loc = TestWorldModeling.world.get_location_by_name("table0")
        assert len(loc.children[0].children) == 2
        assert TestWorldModeling.world.remove_room("kitchen") is True
        assert len(TestWorldModeling.world.rooms) == 0
        assert len(TestWorldModeling.world.hallways) == 0
        assert len(TestWorldModeling.world.locations) == 0
        assert len(TestWorldModeling.world.objects) == 0


if __name__ == "__main__":
    from pyrobosim.gui import start_gui

    TestWorldModeling.test_create_world_default()
    TestWorldModeling.test_create_room()
    TestWorldModeling.test_create_hallway()
    TestWorldModeling.test_create_location()
    TestWorldModeling.test_create_object()
    TestWorldModeling.test_add_robot()
    start_gui(TestWorldModeling.world)
