#!/usr/bin/env python3

"""
Tests for location and object spawn creation in pyrobosim.
"""

import os

from pyrobosim.core import Location, World
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


data_folder = get_data_folder()


class TestLocations:
    def test_add_location_to_world_from_object(self):
        """Test adding a location from a Location object."""
        world = World()
        world.set_metadata(
            locations=os.path.join(data_folder, "example_location_data.yaml"),
            objects=os.path.join(data_folder, "example_object_data.yaml"),
        )

        coords = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        room = world.add_room(name="test_room", footprint=coords)
        assert room is not None

        location = Location(category="table", parent=room, pose=Pose(x=0.0, y=0.0))
        result = world.add_location(location=location)
        assert isinstance(result, Location)
        assert world.num_locations == 1
        assert world.locations[0].name == "table0"

    def test_add_location_to_world_from_args(self):
        """Test adding a location from a list of named keyword arguments."""
        world = World()
        world.set_metadata(
            locations=os.path.join(data_folder, "example_location_data.yaml"),
            objects=os.path.join(data_folder, "example_object_data.yaml"),
        )

        coords = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        room = world.add_room(name="test_room", footprint=coords)
        assert room is not None

        result = world.add_location(
            name="test_table",
            category="table",
            parent="test_room",
            pose=Pose(x=0.0, y=0.0),
        )
        assert isinstance(result, Location)
        assert world.num_locations == 1
        assert world.locations[0].name == "test_table"


if __name__ == "__main__":
    t = TestLocations()
    t.test_add_location_to_world_from_object()
    t.test_add_location_to_world_from_args()
    print("Tests passed!")
