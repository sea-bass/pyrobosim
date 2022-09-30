#!/usr/bin/env python3

"""
Test script for entity getting with world models.
"""
import os
import pytest
import numpy as np

from pyrobosim.core.robot import Robot
from pyrobosim.core.room import Room
from pyrobosim.core.locations import Location, ObjectSpawn
from pyrobosim.core.objects import Object
from pyrobosim.core.world import World
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


class TestGetEntities:

    @pytest.fixture(autouse=True)
    def create_world(self):
        w = World()

        # Set the location and object metadata
        data_folder = get_data_folder()
        w.set_metadata(
            locations=os.path.join(data_folder, "example_location_data.yaml"),
            objects=os.path.join(data_folder, "example_object_data.yaml"))

        # Add rooms
        r1coords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
        w.add_room(Room(r1coords, name="kitchen", color=[1, 0, 0],
                nav_poses=[Pose(x=0.75, y=0.75, yaw=0)]))
        r2coords = [(1.75, 2.5), (3.5, 2.5), (3.5, 4), (1.75, 4)]
        w.add_room(Room(r2coords, name="bedroom", color=[0, 0.6, 0]))
        r3coords = [(-1, 1), (-1, 3.5), (-3.0, 3.5), (-2.5, 1)]
        w.add_room(Room(r3coords, name="bathroom", color=[0, 0, 0.6]))

        # Add hallways between the rooms
        w.add_hallway("kitchen", "bathroom", width=0.7)
        w.add_hallway("bathroom", "bedroom", width=0.5,
                    conn_method="angle", conn_angle=0, offset=0.8)
        w.add_hallway("kitchen", "bedroom", width=0.6,
                    conn_method="points",
                    conn_points=[(1.0, 0.5), (2.5, 0.5), (2.5, 3.0)])

        # Add locations
        table = w.add_location("table", "kitchen", Pose(
            x=0.85, y=-0.5, yaw=-np.pi/2))
        desk = w.add_location("desk", "bedroom", Pose(x=3.15, y=3.65, yaw=0))
        counter = w.add_location("counter", "bathroom", Pose(
            x=-2.45, y=2.5, yaw=np.pi/2 + np.pi/16))

        # Add objects
        w.add_object("banana", table, pose=Pose(x=1.0, y=-0.5, yaw=np.pi/4))
        w.add_object("apple", desk, pose=Pose(x=3.2, y=3.5, yaw=0))
        w.add_object("apple", table)
        w.add_object("apple", table)
        w.add_object("water", counter)
        w.add_object("banana", counter)
        w.add_object("water", desk)

        # Add a robot
        r = Robot(radius=0.1, name="robby")
        w.add_robot(r, loc="kitchen")
        self.world = w


    #####################
    # ACTUAL UNIT TESTS #
    #####################
    def test_valid_room(self):
        """ Checks for existence of valid room. """
        result = self.world.get_room_by_name("kitchen")
        assert isinstance(result, Room) and result.name == "kitchen"

    def test_invalid_room(self):
        """ Checks for existence of invalid room. """
        result = self.world.get_room_by_name("living room")
        assert result is None

    def test_add_remove_room(self):
        """ Checks adding a room and removing it cleanly. """
        room_name = "test_room"
        coords = [(9, 9), (11, 9), (11, 11), (9, 11)]
        result = self.world.add_room(
            Room(coords, name=room_name, color=[0, 0, 0]))
        assert result == True
        self.world.remove_room(room_name)
        result = self.world.get_room_by_name(room_name)
        assert result is None

    def test_valid_location(self):
        """ Checks for existence of valid location. """
        result = self.world.get_location_by_name("counter0")
        assert isinstance(result, Location) and result.name == "counter0"

    def test_invalid_location(self):
        """ Checks for existence of invalid location. """
        result = self.world.get_location_by_name("table42")
        assert result is None

    def test_valid_spawn(self):
        """ Checks for existence of valid object spawn. """
        result = self.world.get_entity_by_name("counter0_left")
        assert isinstance(result, ObjectSpawn) and result.name == "counter0_left"

    def test_invalid_spawn(self):
        """ Checks for existence of invalid object spawn. """
        result = self.world.get_entity_by_name("counter0_middle")
        assert result is None

    def test_add_remove_location(self):
        """
        Checks adding a location and removing it cleanly.
        This also includes any object spawns created for the location.
        """
        loc_name = "desk42"
        new_desk = self.world.add_location("desk", "kitchen", 
            Pose(x=1.0, y=1.1, yaw=-np.pi/2),
            name = loc_name)
        assert isinstance(new_desk, Location)
        self.world.remove_location(loc_name)
        result = self.world.get_location_by_name(loc_name)
        assert result is None
        result = self.world.get_entity_by_name(loc_name + "_desktop")
        assert result is None

    def test_valid_object(self):
        """ Checks for existence of valid object. """
        result = self.world.get_object_by_name("apple1")
        assert isinstance(result, Object) and result.name == "apple1"

    def test_invalid_object(self):
        """ Checks for existence of invalid object. """
        result = self.world.get_object_by_name("apple42")
        assert result is None

    def test_invalid_object_valid_name(self):
        """ 
        Checks for existence of invalid object, but whose name is a valid name
        for another entity type. 
        """
        result = self.world.get_object_by_name("counter0")
        assert result is None
        result = self.world.get_entity_by_name("counter0")
        assert isinstance(result, Location) and result.name == "counter0"

    def test_add_remove_object(self):
        """ Checks adding an object and removing it cleanly. """
        obj_name = "banana13"
        table = self.world.get_location_by_name("table0")
        new_obj = self.world.add_object("banana", table, name=obj_name)
        assert isinstance(new_obj, Object)
        self.world.remove_object(obj_name)
        result = self.world.get_object_by_name(obj_name)
        assert result is None

    def test_valid_robot(self):
        """ Checks for existence of a valid robot. """
        result = self.world.get_robot_by_name("robby")
        assert isinstance(result, Robot) and result.name == "robby"

    def test_invalid_robot(self):
        """ Checks for existence of an invalid robot. """
        result = self.world.get_robot_by_name("robot0")
        assert result is None
