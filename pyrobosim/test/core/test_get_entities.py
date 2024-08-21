#!/usr/bin/env python3

"""
Unit tests for entity getting with world models.
"""
import os
import pytest
import numpy as np

from pyrobosim.core import Robot, Room, Location, ObjectSpawn, Object, World
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


class TestGetEntities:
    @pytest.fixture(autouse=True)
    def create_world(self):
        world = World()

        # Set the location and object metadata
        data_folder = get_data_folder()
        world.set_metadata(
            locations=os.path.join(data_folder, "example_location_data.yaml"),
            objects=os.path.join(data_folder, "example_object_data.yaml"),
        )

        # Add rooms
        r1coords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
        world.add_room(
            name="kitchen",
            footprint=r1coords,
            color=[1, 0, 0],
            nav_poses=[Pose(x=0.75, y=0.75, z=0.0, q=[1.0, 0.0, 0.0, 0.0])],
        )
        r2coords = [(1.75, 2.5), (3.5, 2.5), (3.5, 4), (1.75, 4)]
        world.add_room(name="bedroom", footprint=r2coords, color=[0, 0.6, 0])
        r3coords = [(-1, 1), (-1, 3.5), (-3.0, 3.5), (-2.5, 1)]
        world.add_room(name="bathroom", footprint=r3coords, color=[0, 0, 0.6])

        # Add hallways between the rooms
        world.add_hallway(room_start="kitchen", room_end="bathroom", width=0.7)
        world.add_hallway(
            room_start="bathroom",
            room_end="bedroom",
            width=0.5,
            conn_method="angle",
            conn_angle=0,
            offset=0.8,
        )
        world.add_hallway(
            room_start="kitchen",
            room_end="bedroom",
            width=0.6,
            conn_method="points",
            conn_points=[(1.0, 0.5), (2.5, 0.5), (2.5, 3.0)],
        )

        # Add locations
        table = world.add_location(
            category="table",
            parent="kitchen",
            pose=Pose(x=0.85, y=-0.5, z=0.0, yaw=-np.pi / 2.0),
        )
        desk = world.add_location(
            category="desk", parent="bedroom", pose=Pose(x=3.15, y=3.65, z=0.0)
        )
        counter = world.add_location(
            category="counter",
            parent="bathroom",
            pose=Pose(x=-2.45, y=2.5, z=0.0, yaw=np.pi / 2.0 + np.pi / 16.0),
        )

        # Add objects
        world.add_object(
            category="banana",
            parent=table,
            pose=Pose(x=1.0, y=-0.5, z=0.0, yaw=np.pi / 4.0),
        )
        world.add_object(
            category="apple",
            parent=desk,
            pose=Pose(x=3.2, y=3.5, z=0.0, q=[1.0, 0.0, 0.0, 0.0]),
        )
        world.add_object(category="apple", parent=table)
        world.add_object(category="apple", parent=table)
        world.add_object(category="water", parent=counter)
        world.add_object(category="banana", parent=counter)
        world.add_object(category="water", parent=desk)

        # Add a robot
        robot = Robot(radius=0.1, name="robby")
        world.add_robot(robot, loc="kitchen")
        self.world = world

    #####################
    # ACTUAL UNIT TESTS #
    #####################
    def test_valid_room(self):
        """Checks for existence of valid room."""
        result = self.world.get_room_by_name("kitchen")
        assert isinstance(result, Room) and result.name == "kitchen"

    def test_invalid_room(self):
        """Checks for existence of invalid room."""
        with pytest.warns(UserWarning):
            result = self.world.get_room_by_name("living room")
            assert result is None

    def test_add_remove_room(self):
        """Checks adding a room and removing it cleanly."""
        room_name = "test_room"
        coords = [(9, 9), (11, 9), (11, 11), (9, 11)]
        result = self.world.add_room(name=room_name, footprint=coords, color=[0, 0, 0])
        assert result is not None

        with pytest.warns(UserWarning):
            self.world.remove_room(room_name)
            result = self.world.get_room_by_name(room_name)
            assert result is None

    def test_valid_location(self):
        """Checks for existence of valid location."""
        result = self.world.get_location_by_name("counter0")
        assert isinstance(result, Location) and result.name == "counter0"

    def test_invalid_location(self):
        """Checks for existence of invalid location."""
        with pytest.warns(UserWarning):
            result = self.world.get_location_by_name("table42")
            assert result is None

    def test_valid_spawn(self):
        """Checks for existence of valid object spawn."""
        result = self.world.get_entity_by_name("counter0_left")
        assert isinstance(result, ObjectSpawn) and result.name == "counter0_left"

    def test_invalid_spawn(self):
        """Checks for existence of invalid object spawn."""
        result = self.world.get_entity_by_name("counter0_middle")
        assert result is None

    def test_add_remove_location(self):
        """
        Checks adding a location and removing it cleanly.
        This also includes any object spawns created for the location.
        """
        loc_name = "desk42"
        new_desk = self.world.add_location(
            category="desk",
            parent="kitchen",
            pose=Pose(x=1.0, y=1.1, yaw=-np.pi / 2.0),
            name=loc_name,
        )
        assert isinstance(new_desk, Location)
        self.world.remove_location(loc_name)

        with pytest.warns(UserWarning):
            result = self.world.get_location_by_name(loc_name)
            assert result is None
            result = self.world.get_entity_by_name(loc_name + "_desktop")
            assert result is None

    def test_valid_object(self):
        """Checks for existence of valid object."""
        result = self.world.get_object_by_name("apple1")
        assert isinstance(result, Object) and result.name == "apple1"

    def test_invalid_object(self):
        """Checks for existence of invalid object."""
        result = self.world.get_object_by_name("apple42")
        assert result is None

    def test_invalid_object_valid_name(self):
        """
        Checks for existence of invalid object, but whose name is a valid name
        for another entity type.
        """
        with pytest.warns(UserWarning):
            result = self.world.get_object_by_name("counter0")
            assert result is None

        result = self.world.get_entity_by_name("counter0")
        assert isinstance(result, Location) and result.name == "counter0"

    def test_add_remove_object(self):
        """Checks adding an object and removing it cleanly."""
        obj_name = "banana13"
        table = self.world.get_location_by_name("table0")
        new_obj = self.world.add_object(category="banana", parent=table, name=obj_name)
        assert isinstance(new_obj, Object)
        self.world.remove_object(obj_name)
        result = self.world.get_object_by_name(obj_name)
        assert result is None

    def test_valid_robot(self):
        """Checks for existence of a valid robot."""
        result = self.world.get_robot_by_name("robby")
        assert isinstance(result, Robot) and result.name == "robby"

    def test_invalid_robot(self):
        """Checks for existence of an invalid robot."""
        result = self.world.get_robot_by_name("robot0")
        assert result is None

    def test_get_graph_node_from_entity(self):
        """Checks whether graph nodes can be found from various entities."""
        robot = self.world.robots[0]

        # Querying directly for a graph node returns the node itself
        tabletop = self.world.get_entity_by_name("table0_tabletop")
        tabletop_node = tabletop.graph_nodes[0]
        assert self.world.graph_node_from_entity(tabletop_node) == tabletop_node

        # Querying for an object spawn, room, or hallway returns itself
        entity_names = [
            "table0_tabletop",  # Object spawn
            "kitchen",  # Room
            "hall_kitchen_bedroom",  # Hallway
        ]
        for entity_name in entity_names:
            entity = self.world.get_entity_by_name(entity_name)
            graph_node = self.world.graph_node_from_entity(entity_name, robot=robot)
            assert graph_node in entity.graph_nodes

        # Querying for an object will give a graph node from its parent
        desktop = self.world.get_entity_by_name("desk0_desktop")
        assert (
            self.world.graph_node_from_entity("apple0", robot=robot)
            in desktop.graph_nodes
        )

        # Querying for a location will give a graph node from its child location
        assert (
            self.world.graph_node_from_entity("desk", robot=robot)
            in desktop.graph_nodes
        )

        # Querying for a type of entity that does not have graph nodes will fail
        with pytest.warns(UserWarning):
            assert not self.world.graph_node_from_entity("robot0", robot=robot)
