"""
Unit tests for core world modeling
"""
import os
import pytest
import numpy as np

from pyrobosim.core import Hallway, Object, World
from pyrobosim.utils.pose import Pose

from pyrobosim.utils.general import get_data_folder, InvalidEntityCategoryException


class TestWorldModeling:
    """Tests for the world modeling tools"""

    @staticmethod
    @pytest.mark.dependency()
    def test_create_world_default():
        """Tests the creation of a world"""

        TestWorldModeling.world = World()

        data_folder = get_data_folder()
        TestWorldModeling.world.set_metadata(
            locations=os.path.join(data_folder, "example_location_data.yaml"),
            objects=os.path.join(data_folder, "example_object_data.yaml"),
        )
        assert isinstance(TestWorldModeling.world, World)

    ##############################################
    # These tests incrementally build up a world #
    ##############################################

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_create_world_default"])
    def test_create_room():
        """Tests the creation of a room"""

        room1_name = "kitchen"
        room1_coords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
        TestWorldModeling.world.add_room(
            name=room1_name, footprint=room1_coords, color=[1, 0, 0]
        )

        assert len(TestWorldModeling.world.rooms) == 1
        assert TestWorldModeling.world.get_room_by_name(room1_name) is not None

        room2_name = "bedroom"
        room2_coords = [(1.75, 2.5), (3.5, 2.5), (3.5, 4), (1.75, 4)]
        TestWorldModeling.world.add_room(
            name=room2_name, footprint=room2_coords, color=[1, 0, 0]
        )

        assert len(TestWorldModeling.world.rooms) == 2
        assert TestWorldModeling.world.get_room_by_name(room2_name) is not None

    @staticmethod
    @pytest.mark.dependency(
        depends=[
            "TestWorldModeling::test_create_world_default",
            "TestWorldModeling::test_create_room",
        ]
    )
    def test_create_hallway():
        """Tests the creation of a hallway between 2 rooms"""

        TestWorldModeling.world.add_hallway(
            room_start="kitchen",
            room_end="bedroom",
            offset=0.5,
            conn_method="auto",
            width=0.5,
        )
        assert len(TestWorldModeling.world.hallways) == 1

    @staticmethod
    @pytest.mark.dependency(
        depends=[
            "TestWorldModeling::test_create_world_default",
            "TestWorldModeling::test_create_room",
            "TestWorldModeling::test_create_hallway",
        ]
    )
    def test_create_location():
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

        desk = TestWorldModeling.world.add_location(
            category="desk",
            parent="bedroom",
            name="study_desk",
            pose=Pose(x=3.15, y=3.65, z=0.0, yaw=0.0),
        )
        assert len(TestWorldModeling.world.locations) == 2
        assert TestWorldModeling.world.get_location_by_name("study_desk") == desk

        # Test missing parent
        with pytest.warns(UserWarning):
            result = TestWorldModeling.world.add_location(
                category="desk",
                pose=Pose(),
            )
            assert result is None

        # Test invalid category
        with pytest.warns(UserWarning):
            result = TestWorldModeling.world.add_location(
                category="does_not_exist",
                parent="bedroom",
                pose=Pose(),
            )
            assert result is None

    @staticmethod
    @pytest.mark.dependency(
        depends=[
            "TestWorldModeling::test_create_world_default",
            "TestWorldModeling::test_create_room",
            "TestWorldModeling::test_create_hallway",
            "TestWorldModeling::test_create_location",
        ]
    )
    def test_create_object():
        """Tests adding objects to a location"""

        TestWorldModeling.world.add_object(category="apple", parent="table0")
        TestWorldModeling.world.add_object(category="apple", parent="study_desk")
        assert len(TestWorldModeling.world.objects) == 2

        # second apple
        test_obj = TestWorldModeling.world.objects[1]
        assert isinstance(test_obj, Object)
        assert (
            TestWorldModeling.world.get_object_by_name("apple1") == test_obj
        )  # Automatic naming

        # Test missing parent
        with pytest.warns(UserWarning):
            result = TestWorldModeling.world.add_object(
                category="apple",
                pose=Pose(),
            )
            assert result is None

        # Test invalid category
        with pytest.warns(UserWarning):
            result = TestWorldModeling.world.add_object(
                category="does_not_exist",
                parent="study_desk",
                pose=Pose(),
            )
            assert result is None

    @staticmethod
    @pytest.mark.dependency(
        depends=[
            "TestWorldModeling::test_create_world_default",
            "TestWorldModeling::test_create_room",
            "TestWorldModeling::test_create_hallway",
            "TestWorldModeling::test_create_location",
            "TestWorldModeling::test_create_object",
        ]
    )
    def test_add_robot():
        """Tests adding a robot to the world"""
        from pyrobosim.core import Robot
        from pyrobosim.navigation import PathPlanner

        world_robots = TestWorldModeling.world.robots
        path_planner_config = {"world": TestWorldModeling.world}
        path_planner = PathPlanner("rrt", **path_planner_config)

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
        assert world_robots[1].pose == target_pose

        # Add a robot with the same name as an existing robot, which should fail gracefully
        robot = Robot(name="test_robot", radius=0.05, path_planner=path_planner)
        with pytest.warns(UserWarning):
            TestWorldModeling.world.add_robot(robot, loc="bedroom")
            assert len(world_robots) == 2

    #############################################
    # These tests utilize the fully built world #
    #############################################

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_add_robot"])
    def test_check_occupancy():
        """Tests occupancy using the entity polygons."""

        # Free pose in a room
        free_room_pose = Pose(x=1.0, y=1.0)
        assert not TestWorldModeling.world.check_occupancy(free_room_pose)

        # Free pose in a hallway
        free_hallway_pose = Pose(x=1.3, y=2.2)
        assert not TestWorldModeling.world.check_occupancy(free_hallway_pose)

        # Occupied pose in a room due to locations
        occupied_pose_in_table = Pose(x=1.0, y=-0.5)
        assert TestWorldModeling.world.check_occupancy(occupied_pose_in_table)

        # Occupied pose outside the walls
        occupied_pose_outside_walls = Pose(x=0.0, y=3.0)
        assert TestWorldModeling.world.check_occupancy(occupied_pose_outside_walls)

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_add_robot"])
    def test_collides_with_robots():
        """Tests if poses are colliding with robots."""

        robot_0 = TestWorldModeling.world.robots[0]
        robot_1 = TestWorldModeling.world.robots[1]

        # Free poses should not return collision
        free_hallway_pose = Pose(x=1.3, y=2.2)
        assert not TestWorldModeling.world.collides_with_robots(free_hallway_pose)

        # Both robot poses should return collision if no robot is specified.
        assert TestWorldModeling.world.collides_with_robots(robot_0.pose)
        assert TestWorldModeling.world.collides_with_robots(robot_1.pose)

        # Robots should collide with each other
        assert not TestWorldModeling.world.collides_with_robots(
            robot_0.pose, robot=robot_0
        )
        assert TestWorldModeling.world.collides_with_robots(robot_1.pose, robot=robot_0)
        assert TestWorldModeling.world.collides_with_robots(robot_0.pose, robot=robot_1)
        assert not TestWorldModeling.world.collides_with_robots(
            robot_1.pose, robot=robot_1
        )

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_add_robot"])
    def test_is_connectable():
        """Tests if poses are connectable in a straight line."""

        ## Test with a simple straight line in free space
        pose_start = Pose(x=1.0, y=0.25)
        pose_goal = Pose(x=0.5, y=1.0)

        # Default arguments should return True.
        assert TestWorldModeling.world.is_connectable(pose_start, pose_goal)

        # If the max connection distance is larger than the actual distance, this should return False instead.
        assert not TestWorldModeling.world.is_connectable(
            pose_start, pose_goal, max_dist=0.5
        )

        ## Test with a straight line right near a wall
        pose_start = Pose(x=1.5, y=2.4)
        pose_goal = Pose(x=3.0, y=2.75)

        # With a small step distance, this should return False
        assert not TestWorldModeling.world.is_connectable(
            pose_start, pose_goal, step_dist=0.01
        )

        # With a large step distance, this should return True as the collision point is skipped
        assert not TestWorldModeling.world.is_connectable(
            pose_start, pose_goal, step_dist=0.5
        )

    ##############################################
    # These tests incrementally clean up a world #
    ##############################################

    @staticmethod
    @pytest.mark.dependency(
        depends=[
            "TestWorldModeling::test_check_occupancy",
            "TestWorldModeling::test_collides_with_robots",
            "TestWorldModeling::test_is_connectable",
        ]
    )
    def test_remove_robot():
        """Tests deleting robots from the world"""

        assert TestWorldModeling.world.remove_robot("test_robot") is True
        assert len(TestWorldModeling.world.robots) == 1

        assert TestWorldModeling.world.remove_robot("other_test_robot") is True
        assert len(TestWorldModeling.world.robots) == 0

        with pytest.warns(UserWarning):
            assert TestWorldModeling.world.remove_robot("does_not_exist") is False

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_remove_robot"])
    def test_remove_object():
        """Tests deleting objects from the world"""

        assert TestWorldModeling.world.remove_object("apple1") is True
        assert len(TestWorldModeling.world.locations) == 2
        assert TestWorldModeling.world.get_object_by_name("apple1") is None
        assert TestWorldModeling.world.objects[0].name == "apple0"

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_create_location"])
    def test_remove_location():
        """Tests removing a location from the world"""

        assert TestWorldModeling.world.remove_location("study_desk") is True
        assert len(TestWorldModeling.world.locations) == 1
        with pytest.warns(UserWarning):
            assert TestWorldModeling.world.get_location_by_name("study_desk") is None

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_create_hallway"])
    def test_remove_hallway():
        """Tests removing a hallway"""

        hallways = TestWorldModeling.world.get_hallways_from_rooms("kitchen", "bedroom")
        assert len(hallways) == 1
        assert isinstance(hallways[0], Hallway)

        assert TestWorldModeling.world.remove_hallway(hallways[0]) is True
        hallways = TestWorldModeling.world.get_hallways_from_rooms("kitchen", "bedroom")
        assert len(hallways) == 0

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_create_room"])
    def test_remove_room():
        """Tests deleting rooms"""

        assert TestWorldModeling.world.remove_room("bedroom") is True
        assert len(TestWorldModeling.world.rooms) == 1
        assert TestWorldModeling.world.rooms[0].name == "kitchen"

    @staticmethod
    @pytest.mark.dependency(
        depends=[
            "TestWorldModeling::test_remove_room",
            "TestWorldModeling::test_remove_hallway",
            "TestWorldModeling::test_remove_location",
            "TestWorldModeling::test_remove_object",
        ]
    )
    def test_hierarchical_cleanup():
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
