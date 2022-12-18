"""
Unit tests for core world modeling
"""

import pytest
from pyrobosim.core.world import World
from pyrobosim.core.world import Room


class TestWorldModelling():
    """ Tests for the world modelling tools"""

    @staticmethod
    @pytest.mark.dependency()
    def test_create_world_default():
        """ Tests the creation of a world """
        
        TestWorldModelling.world = World()
        assert TestWorldModelling.world is not None

    
    @staticmethod
    @pytest.mark.dependency(depends=["TestWorldModelling::test_create_world_default"])
    def test_create_room():
        """ Tests the creation of a room """

        room_name = "room1"
        # room_coords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
        room_coords = [(-8, -2), (-8, 2), (-6, 2), (-6, -2)]
        room1 = Room(room_coords,
                    name=room_name,
                    color=[1, 0, 0])
        TestWorldModelling.world.add_room(room1)

        assert len(TestWorldModelling.world.rooms) == 1
        assert TestWorldModelling.world.get_room_by_name(room_name) == room1
        
        room_name = "room2"
        room_coords = [(8, -2), (8, 2), (6, 2), (6, -2)]
        room2 = Room(room_coords,
                    name=room_name,
                    color=[1, 0, 0])
        TestWorldModelling.world.add_room(room2)
        assert len(TestWorldModelling.world.rooms) == 2
        assert TestWorldModelling.world.get_room_by_name(room_name) == room2

    @staticmethod
    @pytest.mark.dependency(depends=
        ["TestWorldModelling::test_create_world_default","TestWorldModelling::test_create_room"])
    def test_create_hallway():
        """ Tests the creation of a hallway between 2 rooms"""

        TestWorldModelling.world.add_hallway("room1", "room2", offset=0.5,
            conn_method="auto", width = 0.5)
        assert len(TestWorldModelling.world.hallways) == 1
