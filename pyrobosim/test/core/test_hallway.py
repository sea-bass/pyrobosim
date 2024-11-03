#!/usr/bin/env python3

"""
Tests for hallway creation in pyrobosim.
"""

import pytest

from pyrobosim.core import Hallway, World
from pyrobosim.planning.actions import ExecutionStatus


class TestHallway:
    @pytest.fixture(autouse=True)
    def create_test_world(self):
        self.test_world = World()

        coords_start = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        self.room_start = self.test_world.add_room(
            name="room_start", footprint=coords_start
        )

        coords_end = [(3.0, 5.0), (5.0, 3.0), (5.0, 5.0), (3.0, 5.0)]
        self.room_end = self.test_world.add_room(name="room_end", footprint=coords_end)

    def test_add_hallway_to_world_from_object(self, caplog):
        """Test adding a hallway from a Hallway object."""

        hallway = Hallway(
            room_start=self.room_start,
            room_end=self.room_end,
            name="test_hallway",
            width=0.1,
        )
        result = self.test_world.add_hallway(hallway=hallway)
        assert isinstance(result, Hallway)
        assert self.test_world.num_hallways == 1
        assert self.test_world.hallways[0].room_start == self.room_start
        assert self.test_world.hallways[0].room_end == self.room_end
        assert self.test_world.hallways[0].name == "test_hallway"
        assert self.test_world.hallways[0].reversed_name == "test_hallway"
        assert self.test_world.hallways[0].width == 0.1

        # Adding the same hallway again should fail due to duplicate names.
        result = self.test_world.add_hallway(hallway=hallway)
        assert result is None
        assert self.test_world.num_hallways == 1
        assert (
            "Hallway test_hallway already exists in the world. Cannot add."
            in caplog.text
        )

    def test_add_hallway_to_world_from_args(self):
        """Test adding a hallway from a list of named keyword arguments."""
        result = self.test_world.add_hallway(
            room_start=self.room_start,
            room_end="room_end",
            width=0.1,
            conn_method="points",
            conn_points=[(0.0, 0.0), (2.0, 0.0), (4.0, 2.0), (4.0, 4.0)],
        )
        assert isinstance(result, Hallway)
        assert self.test_world.num_hallways == 1
        assert self.test_world.hallways[0].room_start == self.room_start
        assert self.test_world.hallways[0].room_end == self.room_end
        assert self.test_world.hallways[0].name == "hall_room_end_room_start"
        assert self.test_world.hallways[0].reversed_name == "hall_room_start_room_end"
        assert self.test_world.hallways[0].width == 0.1

        # Adding a second room should add a mangle.
        result = self.test_world.add_hallway(
            room_start=self.room_start,
            room_end="room_end",
            width=0.08,
            conn_method="points",
            conn_points=[(0.0, 0.0), (-2.0, 0.0), (-2.0, 4.0)],
        )
        assert isinstance(result, Hallway)
        assert self.test_world.num_hallways == 2
        assert self.test_world.hallways[1].room_start == self.room_start
        assert self.test_world.hallways[1].room_end == self.room_end
        assert self.test_world.hallways[1].name == "hall_room_end_room_start_1"
        assert self.test_world.hallways[1].reversed_name == "hall_room_start_room_end_1"
        assert self.test_world.hallways[1].width == 0.08

    def test_add_hallway_fail_validation(self, caplog):
        """Test that all the hallway validation checks work."""
        with pytest.raises(ValueError) as exc_info:
            self.test_world.add_hallway(
                room_start="bad_start_room", room_end="room_end", width=0.1
            )
        assert "Room not found: bad_start_room" in caplog.text
        assert exc_info.value.args[0] == "room_start must be a valid Room object."

        caplog.clear()
        with pytest.raises(ValueError) as exc_info:
            self.test_world.add_hallway(
                room_start="room_start", room_end="bad_end_room", width=0.1
            )
        assert "Room not found: bad_end_room" in caplog.text
        assert exc_info.value.args[0] == "room_end must be a valid Room object."

        with pytest.raises(ValueError) as exc_info:
            self.test_world.add_hallway(
                room_start="room_start", room_end="room_end", width=0.0
            )
        assert exc_info.value.args[0] == "width must be a positive value."

        with pytest.raises(ValueError) as exc_info:
            self.test_world.add_hallway(
                room_start="room_start",
                room_end="room_end",
                width=0.1,
                conn_method="bad_conn_method",
            )
        assert exc_info.value.args[0] == "No valid connection method: bad_conn_method."

    def test_add_hallway_open_close_lock_unlock(self, caplog):
        """Test the open, close, lock, and unlock capabilities of hallways."""
        result = self.test_world.add_hallway(
            room_start="room_start",
            room_end="room_end",
            width=0.1,
        )

        assert isinstance(result, Hallway)
        assert self.test_world.num_hallways == 1
        hallway = self.test_world.hallways[0]
        assert hallway.is_open
        assert not hallway.is_locked

        # When trying to open the hallway, it should succeed and say it's already open.
        result = self.test_world.open_location(hallway)
        assert result.is_success()
        assert "Hallway: hall_room_end_room_start is already open." in caplog.text
        assert hallway.is_open
        assert not hallway.is_locked

        # Closing should work
        result = self.test_world.close_location(hallway)
        assert result.is_success()
        assert not hallway.is_open
        assert not hallway.is_locked

        # Locking should work
        result = self.test_world.lock_location(hallway)
        assert result.is_success()
        assert not hallway.is_open
        assert hallway.is_locked

        # Opening should not work due to being locked
        caplog.clear()
        result = self.test_world.open_location(hallway)
        assert result.status == ExecutionStatus.PRECONDITION_FAILURE
        assert "Hallway: hall_room_end_room_start is locked." in caplog.text
        assert not hallway.is_open
        assert hallway.is_locked

        # Closing should succeed due to already being closed
        caplog.clear()
        result = self.test_world.close_location(hallway)
        assert result.is_success()
        assert "Hallway: hall_room_end_room_start is already closed." in caplog.text
        assert not hallway.is_open
        assert hallway.is_locked

        # Locking should succeed due to already being locked
        caplog.clear()
        result = self.test_world.lock_location(hallway)
        assert result.is_success()
        assert "Hallway: hall_room_end_room_start is already locked." in caplog.text
        assert not hallway.is_open
        assert hallway.is_locked

        # Unlocking should work
        result = self.test_world.unlock_location(hallway)
        assert result.is_success()
        assert not hallway.is_open
        assert not hallway.is_locked

        # Unlocking should succeed due to already being unlocked
        caplog.clear()
        result = self.test_world.unlock_location(hallway)
        assert result.is_success()
        assert "Hallway: hall_room_end_room_start is already unlocked." in caplog.text
        assert not hallway.is_open
        assert not hallway.is_locked

        # Opening should work
        result = self.test_world.open_location(hallway)
        assert result.is_success()
        assert hallway.is_open
        assert not hallway.is_locked
