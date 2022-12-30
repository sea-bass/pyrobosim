"""
Unit tests for core world modeling
"""

import pytest
from pyrobosim.core.world import World
from pyrobosim.core.world import Room


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

        room_name = "room1"
        room_coords = [(-8, -2), (-8, 2), (-6, 2), (-6, -2)]
        room1 = Room(room_coords,
                    name=room_name,
                    color=[1, 0, 0])
        TestWorldModeling.world.add_room(room1)

        assert len(TestWorldModeling.world.rooms) == 1
        assert TestWorldModeling.world.get_room_by_name(room_name) == room1
        
        room_name = "room2"
        room_coords = [(8, -2), (8, 2), (6, 2), (6, -2)]
        room2 = Room(room_coords,
                    name=room_name,
                    color=[1, 0, 0])
        TestWorldModeling.world.add_room(room2)
        assert len(TestWorldModeling.world.rooms) == 2
        assert TestWorldModeling.world.get_room_by_name(room_name) == room2

    @staticmethod
    @pytest.mark.dependency(depends=
        ["TestWorldModeling::test_create_world_default","TestWorldModeling::test_create_room"])
    def test_create_hallway():
        """ Tests the creation of a hallway between 2 rooms"""

        TestWorldModeling.world.add_hallway("room1", "room2", offset=0.5,
            conn_method="auto", width = 0.5)
        assert len(TestWorldModeling.world.hallways) == 1
