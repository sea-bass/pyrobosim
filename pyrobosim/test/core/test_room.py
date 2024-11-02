#!/usr/bin/env python3

"""
Tests for room creation in pyrobosim.
"""

import pytest
from pyrobosim.core import Room, World


class TestRoom:
    def test_add_room_to_world_from_object(self, caplog):
        """Test adding a room from a Room object."""
        world = World()

        coords = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        room = Room(name="test_room", footprint=coords)
        result = world.add_room(room=room)

        assert isinstance(result, Room)
        assert world.num_rooms == 1
        assert world.rooms[0].name == "test_room"

        # Adding the same room again should fail due to duplicate names.
        result = world.add_room(room=room)
        assert result is None
        assert world.num_rooms == 1
        assert "Room test_room already exists in the world. Cannot add." in caplog.text

    def test_add_room_to_world_from_args(self):
        """Test adding a room from a list of named keyword arguments."""
        world = World()

        coords = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        color = [1.0, 0.0, 0.1]
        result = world.add_room(footprint=coords, color=color)

        assert isinstance(result, Room)
        assert world.num_rooms == 1
        assert world.rooms[0].name == "room0"
        assert world.rooms[0].viz_color == color

    def test_add_room_to_world_empty_geometry(self):
        """Test adding a room with an empty footprint. Should raise an exception."""
        world = World()

        with pytest.raises(Exception):
            result = world.add_room(name="test_room")

    def test_add_room_to_world_in_collision(self, caplog):
        """Test adding a room in collision with another room."""
        world = World()

        # This room should be added correctly.
        orig_coords = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        result = world.add_room(footprint=orig_coords)
        assert isinstance(result, Room)
        assert world.num_rooms == 1

        # This new room should fail to add since it's in the collision with the first room.
        new_coords = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
        result = world.add_room(footprint=new_coords)
        assert result is None
        assert world.num_rooms == 1
        assert "Room room1 in collision. Cannot add to world." in caplog.text
