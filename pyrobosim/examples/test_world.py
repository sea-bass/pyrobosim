"""
Unit tests for core world modeling
"""
import os
import pytest
import numpy as np

from pyrobosim.core.hallway import Hallway
from pyrobosim.core.objects import Object
from pyrobosim.core.room import Room
from pyrobosim.core.world import World
from pyrobosim.utils.pose import Pose

from pyrobosim.utils.general import get_data_folder

class TestWorldModeling():
    """ Tests for the world modelling tools"""

    @staticmethod
    @pytest.mark.dependency()
    def test_create_world_default():
        """ Tests the creation of a world """
        
        TestWorldModeling.world = World()
        TestWorldModeling.world.set_metadata(locations=os.path.join(get_data_folder(), "example_location_data.yaml"),
                   objects=os.path.join(get_data_folder(), "example_object_data.yaml"))
        assert TestWorldModeling.world is not None
    
    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_create_world_default"])
    def test_create_room():
        """ Tests the creation of a room """

        room1_name = "kitchen"
        room1_coords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
        room1 = Room(room1_coords,
                    name=room1_name,
                    color=[1, 0, 0])
        TestWorldModeling.world.add_room(room1)

        assert len(TestWorldModeling.world.rooms) == 1
        assert TestWorldModeling.world.get_room_by_name(room1_name) == room1
        
        room2_name = "bedroom"
        room2_coords = [(1.75, 2.5), (3.5, 2.5), (3.5, 4), (1.75, 4)]
        room2 = Room(room2_coords,
                    name=room2_name,
                    color=[1, 0, 0])
        TestWorldModeling.world.add_room(room2)

        assert len(TestWorldModeling.world.rooms) == 2
        assert TestWorldModeling.world.get_room_by_name(room2_name) == room2

    @staticmethod
    @pytest.mark.dependency(depends=
        ["TestWorldModeling::test_create_world_default","TestWorldModeling::test_create_room"])
    def test_create_hallway():
        """ Tests the creation of a hallway between 2 rooms"""

        TestWorldModeling.world.add_hallway("kitchen", "bedroom", offset=0.5,
            conn_method="auto", width = 0.5)
        assert len(TestWorldModeling.world.hallways) == 1

    @staticmethod
    @pytest.mark.dependency(depends=
        ["TestWorldModeling::test_create_world_default","TestWorldModeling::test_create_room", 
        "TestWorldModeling::test_create_hallway"])
    def test_create_location():
        """ Tests the creation of locations"""
        table = TestWorldModeling.world.add_location("table", "kitchen", Pose(
            x=0.85, y=-0.5, z=0.0, yaw=-np.pi/2))

        assert len(TestWorldModeling.world.locations) == 1
        assert TestWorldModeling.world.get_location_by_name("table0") == table #automatic naming check

        desk = TestWorldModeling.world.add_location("desk", "bedroom", name="study_desk", pose=Pose(x=3.15, y=3.65, z=0.0, yaw=0.0))
        assert len(TestWorldModeling.world.locations) == 2
        assert TestWorldModeling.world.get_location_by_name("study_desk") == desk


    @staticmethod
    @pytest.mark.dependency(depends=
        ["TestWorldModeling::test_create_world_default","TestWorldModeling::test_create_room", 
        "TestWorldModeling::test_create_hallway", "TestWorldModeling::test_create_location"])
    def test_create_object():
        """ Tests adding objects to a location"""

        TestWorldModeling.world.add_object("apple","table0")
        TestWorldModeling.world.add_object("apple","study_desk")
        assert len(TestWorldModeling.world.objects) == 2

        # second apple
        test_obj = TestWorldModeling.world.objects[1]
        assert isinstance(test_obj, Object)
        assert test_obj.name == "apple1" # Automatic naming


    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_create_object"])
    def test_remove_object():
        """ Tests deleting objects from the world """

        assert TestWorldModeling.world.remove_object("apple1") is True
        assert len(TestWorldModeling.world.locations) == 2
        assert TestWorldModeling.world.get_object_by_name("apple1") is None
        assert TestWorldModeling.world.objects[0].name == "apple0"

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_create_location"])
    def test_remove_location():
        """ Tests removing a location from the world """

        assert TestWorldModeling.world.remove_location("study_desk") is True
        assert len(TestWorldModeling.world.locations) == 1
        assert TestWorldModeling.world.get_location_by_name("study_desk") is None # Raises a warning

    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_create_hallway"])
    def test_remove_hallway():
        """ Tests removing a hallway """

        hallways = TestWorldModeling.world.get_hallways_from_rooms("kitchen", "bedroom")
        assert len(hallways) == 1
        assert isinstance(hallways[0], Hallway)

        assert TestWorldModeling.world.remove_hallway(hallways[0]) is True
        hallways = TestWorldModeling.world.get_hallways_from_rooms("kitchen", "bedroom")
        assert len(hallways) == 0
  
    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModeling::test_create_room"])
    def test_remove_room():
        """ Tests deleting rooms """

        assert TestWorldModeling.world.remove_room("bedroom") is True
        assert len(TestWorldModeling.world.rooms) == 1
        assert TestWorldModeling.world.rooms[0].name == "kitchen"

        