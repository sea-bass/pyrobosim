"""
Unit tests for core world modeling
"""

import pytest
from pyrobosim.core.world import World
from pyrobosim.core.world import Room

@pytest.mark.dependency(name="test_create_world_default")
def test_create_world_default():
    """ Tests the creation of a world """
    
    world = World()
    assert world is not None

@pytest.mark.dependency(depends=["test_create_world_default"])
def test_create_room():
    """ Tests the creation of a room """

    world = World()
    room_name = "room1"
    room_coords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
    room = Room(room_coords,
                name=room_name,
                color=[1, 0, 0])
    world.add_room(room)

    assert len(world.rooms) == 1
    assert world.get_room_by_name(room_name) == room
    