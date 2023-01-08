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
