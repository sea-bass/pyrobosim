#!/usr/bin/env python3

"""
Tests for location and object spawn creation in pyrobosim.
"""

import os
import pytest

from pyrobosim.core import Location, World
from pyrobosim.planning.actions import ExecutionStatus
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


class TestLocations:
    @pytest.fixture(autouse=True)
    def create_test_world(self):
        data_folder = get_data_folder()
        self.test_world = World()
        self.test_world.set_metadata(
            locations=os.path.join(data_folder, "example_location_data.yaml"),
            objects=os.path.join(data_folder, "example_object_data.yaml"),
        )

        coords = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        self.test_room = self.test_world.add_room(name="test_room", footprint=coords)

    def test_add_location_to_world_from_object(self, caplog):
        """Test adding a location from a Location object."""
        location = Location(
            category="table",
            name="test_table",
            parent=self.test_room,
            pose=Pose(x=0.0, y=0.0),
        )
        result = self.test_world.add_location(location=location)
        assert isinstance(result, Location)
        assert self.test_world.num_locations == 1
        assert self.test_world.locations[0].name == "test_table"
        assert self.test_world.locations[0].is_open
        assert not self.test_world.locations[0].is_locked

        # Adding the same location again should fail due to duplicate names.
        result = self.test_world.add_location(location=location)
        assert result is None
        assert self.test_world.num_locations == 1
        assert (
            "Location test_table already exists in the world. Cannot add."
            in caplog.text
        )

    def test_add_location_to_world_from_args(self):
        """Test adding a location from a list of named keyword arguments."""
        result = self.test_world.add_location(
            category="table",
            parent="test_room",
            pose=Pose(x=0.0, y=0.0),
            is_open=False,
            is_locked=True,
        )
        assert isinstance(result, Location)
        assert self.test_world.num_locations == 1
        assert self.test_world.locations[0].name == "table0"
        assert not self.test_world.locations[0].is_open
        assert self.test_world.locations[0].is_locked

    def test_add_location_open_close_lock_unlock(self):
        """Test the open, close, lock, and unlock capabilities of locations."""
        result = self.test_world.add_location(
            name="test_table",
            category="table",
            parent="test_room",
            pose=Pose(x=0.0, y=0.0),
        )

        assert isinstance(result, Location)
        assert self.test_world.num_locations == 1
        location = self.test_world.locations[0]
        assert location.is_open
        assert not location.is_locked

        # When trying to open the location, it should succeed and say it's already open.
        result = self.test_world.open_location(location)
        assert result.is_success()
        assert result.message == "Location: test_table is already open."
        assert location.is_open
        assert not location.is_locked

        # Closing should work
        result = self.test_world.close_location(location)
        assert result.is_success()
        assert not location.is_open
        assert not location.is_locked

        # Locking should work
        result = self.test_world.lock_location(location)
        assert result.is_success()
        assert not location.is_open
        assert location.is_locked

        # Opening should not work due to being locked
        result = self.test_world.open_location(location)
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert result.message == "Location: test_table is locked."
        assert not location.is_open
        assert location.is_locked

        # Closing should succeed due to already being closed
        result = self.test_world.close_location(location)
        assert result.is_success()
        assert result.message == "Location: test_table is already closed."
        assert not location.is_open
        assert location.is_locked

        # Locking should succeed due to already being locked
        result = self.test_world.lock_location(location)
        assert result.is_success()
        assert result.message == "Location: test_table is already locked."
        assert not location.is_open
        assert location.is_locked

        # Unlocking should work
        result = self.test_world.unlock_location(location)
        assert result.is_success()
        assert not location.is_open
        assert not location.is_locked

        # Unlocking should succeed due to already being unlocked
        result = self.test_world.unlock_location(location)
        assert result.is_success()
        assert result.message == "Location: test_table is already unlocked."
        assert not location.is_open
        assert not location.is_locked

        # Opening should work
        result = self.test_world.open_location(location)
        assert result.is_success()
        assert location.is_open
        assert not location.is_locked
