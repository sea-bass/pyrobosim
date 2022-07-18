"""
Unit tests for core world modeling
"""

import os
import pytest
from pyrobosim.core.world import World
from pyrobosim.core.world import Room
from pyrobosim.utils.general import get_data_folder


def test_create_world():
    data_folder = get_data_folder()
    world = World()
    # Set the location and object metadata
    world.set_metadata(locations=os.path.join(data_folder, "example_location_data.yaml"),
                       objects=os.path.join(data_folder, "example_object_data.yaml"))

    assert World is not None


def test_create_room():
    world = World()
    room_cords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
    room = Room(room_cords,
                name="room_1",
                color=[1, 0, 0])
    world.add_room(room)

    assert len(world.rooms) == 1
    assert world.get_room_by_name("room_1") == room

    