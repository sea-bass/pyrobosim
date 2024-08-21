#!/usr/bin/env python3

"""
Tests for room creation in pyrobosim.
"""

import pytest
from pyrobosim.core import Room, World


class TestRoom:
    def test_add_room_to_world_from_object(self):
        """Test adding a room from a Room object."""
        world = World()

        coords = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        room = Room(footprint=coords)
        result = world.add_room(room=room)

        assert isinstance(result, Room)
        assert world.num_rooms == 1
        assert world.rooms[0].name == "room0"

    def test_add_room_to_world_from_args(self):
        """Test adding a room from a list of named keyword arguments."""
        world = World()

        coords = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        color = [1.0, 0.0, 0.1]
        result = world.add_room(name="test_room", footprint=coords, color=color)

        assert isinstance(result, Room)
        assert world.num_rooms == 1
        assert world.rooms[0].name == "test_room"
        assert world.rooms[0].viz_color == color

    def test_add_room_to_world_empty_geometry(self):
        """Test adding a room with an empty footprint. Should raise an exception."""
        world = World()

        with pytest.raises(Exception):
            result = world.add_room(name="test_room")

    def test_add_room_to_world_in_collision(self):
        """Test adding a room in collision with another room."""
        world = World()

        # This room should be added correctly.
        orig_coords = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        result = world.add_room(footprint=orig_coords)
        assert isinstance(result, Room)
        assert world.num_rooms == 1

        # This new room should fail to add since it's in the collision with the first room.
        new_coords = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
        with pytest.warns(UserWarning):
            result = world.add_room(footprint=new_coords)
            assert result is None
            assert world.num_rooms == 1


if __name__ == "__main__":
    t = TestRoom()
    t.test_add_room_to_world_from_object()
    t.test_add_room_to_world_from_args()
    t.test_add_room_to_world_empty_geometry()
    t.test_add_room_to_world_in_collision()
    print("Tests passed!")
