"""
Unit tests for core world modeling
"""

import pytest
import numpy as np

from pyrobosim.core.world import World
from pyrobosim.core.world import Room
from pyrobosim.utils.pose import Pose
from pyrobosim.core.world import Object


class TestWorldModeling():
    """ Tests for the world modelling tools"""

    @staticmethod
    @pytest.mark.dependency()
    def test_create_world_default():
        """ Tests the creation of a world """
        
        TestWorldModeling.world = World()
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
        # TODO : add tests for named locations.
        table = TestWorldModeling.world.add_location("table", "kitchen", Pose(
            x=0.85, y=-0.5, z=0.0, yaw=-np.pi/2))

        table_name = "table0" # We expect the suffix '0' to be added due to automatic naming
        assert len(TestWorldModeling.world.locations) == 1
        assert TestWorldModeling.world.get_location_by_name(table_name) == table

    @staticmethod
    @pytest.mark.dependency(depends=
        ["TestWorldModeling::test_create_world_default","TestWorldModeling::test_create_room", 
        "TestWorldModeling::test_create_hallway", "TestWorldModeling::test_create_location"])
    def test_add_objects():

        apple = Object(category="apple")
        apple.viz_color = [1, 0, 0]

        TestWorldModeling.world.add_object("apple","table0")
        TestWorldModeling.world.add_object("apple","table0")
        assert len(TestWorldModeling.world.objects) == 2

        # first apple
        test_obj = TestWorldModeling.world.objects[0]
        assert isinstance(test_obj, Object)
        assert test_obj.name == "apple0" # Automatic naming

        # second apple
        test_obj = TestWorldModeling.world.objects[1]
        assert isinstance(test_obj, Object)
        assert test_obj.name == "apple1" # Automatic naming