#!/usr/bin/env python3

"""
Tests for object creation in pyrobosim.
"""

import os

from pyrobosim.core import Object, World
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


data_folder = get_data_folder()


class TestObjects:
    def test_add_object_to_world_from_object(self):
        """Test adding an object from an Object instance."""
        world = World()
        world.set_metadata(
            locations=os.path.join(data_folder, "example_location_data.yaml"),
            objects=os.path.join(data_folder, "example_object_data.yaml"),
        )

        coords = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        room = world.add_room(name="test_room", footprint=coords)
        assert room is not None

        table = world.add_location(
            name="test_table",
            category="table",
            parent="test_room",
            pose=Pose(x=0.0, y=0.0),
        )
        assert table is not None

        obj = Object(
            category="banana",
            parent=table.children[0],
            pose=Pose(x=0.0, y=0.0, yaw=1.0),
        )
        result = world.add_object(object=obj)
        assert isinstance(result, Object)
        assert world.num_objects == 1
        assert world.objects[0].name == "banana0"
        assert world.objects[0].parent.parent == table

    def test_add_object_to_world_from_args(self):
        """Test adding an object from a list of named keyword arguments."""
        world = World()
        world.set_metadata(
            locations=os.path.join(data_folder, "example_location_data.yaml"),
            objects=os.path.join(data_folder, "example_object_data.yaml"),
        )

        coords = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        room = world.add_room(name="test_room", footprint=coords)
        assert room is not None

        table = world.add_location(
            name="test_table",
            category="table",
            parent="test_room",
            pose=Pose(x=0.0, y=0.0),
        )
        assert table is not None

        result = world.add_object(name="test_banana", category="banana", parent=table)
        assert isinstance(result, Object)
        assert world.num_objects == 1
        assert world.objects[0].name == "test_banana"
        assert world.objects[0].parent.parent == table


if __name__ == "__main__":
    t = TestObjects()
    t.test_add_object_to_world_from_object()
    t.test_add_object_to_world_from_args()
    print("Tests passed!")
